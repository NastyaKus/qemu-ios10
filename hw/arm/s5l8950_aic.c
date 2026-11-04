/*
 * Apple Interrupt Controller v1 ("aic,1"), as the S5L8950X (A6) has it.
 *
 * Read off 14G60's AppleInterruptController (kernelcache.release.iphone5,
 * kext at 0x80fa6000) and matching the AIC1 layout Asahi Linux documents for
 * its descendants:
 *
 *   0x0004 INFO        [9:0] external inputs, [20:16] CPUs - 1
 *   0x0010 CONFIG
 *   0x2000 WHOAMI      the reading CPU's number
 *   0x2004 IACK        read = take the next event: [18:16] type, [15:0] number.
 *                      Type 1 an external input (masked by the read), type 4
 *                      an IPI (1 other, 2 self; masked by the read), 0 none.
 *                      The kext panics on type 7 (its timer type): the
 *                      generic timer reaches the CPU as FIQ, not through here.
 *   0x2008 IPI_SET     bit n: "other" IPI to CPU n; bit 31: "self" IPI
 *   0x200c IPI_CLR     acknowledge the reading CPU's bits
 *   0x2024 IPI_MASK_SET, 0x2028 IPI_MASK_CLR
 *   0x3000 + 4n        TARGET of input n (CPU mask)
 *   0x4000 SW_SET, 0x4080 SW_CLR, 0x4100 MASK_SET, 0x4180 MASK_CLR,
 *   0x4200 HW_STATE    one bit per input, 32 inputs a word
 *   0x5000 + 0x80c     CPU c's own copy of the 0x2000 block (the kext drains
 *                      0x5004 at start and sets each CPU's IPI masks there)
 *
 * Registers with no behaviour modelled read back what was written.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/arm/s5l8950.h"
#include "hw/core/cpu.h"

#define AIC_INFO          0x0004
#define AIC_CONFIG        0x0010
#define AIC_CPU_BLOCK     0x2000
#define AIC_WHOAMI        0x00
#define AIC_IACK          0x04
#define AIC_IPI_SET       0x08
#define AIC_IPI_CLR       0x0c
#define AIC_IPI_MASK_SET  0x24
#define AIC_IPI_MASK_CLR  0x28
#define AIC_TARGET        0x3000
#define AIC_SW_SET        0x4000
#define AIC_SW_CLR        0x4080
#define AIC_MASK_SET      0x4100
#define AIC_MASK_CLR      0x4180
#define AIC_HW_STATE      0x4200
#define AIC_PERCPU        0x5000
#define AIC_PERCPU_STRIDE 0x80

#define AIC_IPI_OTHER     (1u << 0)
#define AIC_IPI_SELF      (1u << 31)

#define AIC_EVT_HW        1
#define AIC_EVT_IPI       4

#define NWORDS (S5L8950_AIC_NR_IRQ / 32)

static unsigned aic_cpu_now(AppleAIC1State *s)
{
    unsigned c = current_cpu ? current_cpu->cpu_index : 0;

    return c < s->ncpu ? c : 0;
}

static uint32_t aic_target(AppleAIC1State *s, unsigned n)
{
    /* An input nobody has targeted yet goes to the boot CPU. */
    return s->target[n] ? s->target[n] : 1;
}

/* The next external input pending for `cpu`, or -1. */
static int aic_next_hw(AppleAIC1State *s, unsigned cpu)
{
    for (unsigned w = 0; w < NWORDS; w++) {
        uint32_t live = (s->hw[w] | s->sw[w]) & ~s->mask[w];

        while (live) {
            unsigned b = ctz32(live), n = w * 32 + b;

            if (aic_target(s, n) & (1u << cpu)) {
                return n;
            }
            live &= live - 1;
        }
    }
    return -1;
}

static void aic_update(AppleAIC1State *s)
{
    for (unsigned c = 0; c < s->ncpu; c++) {
        bool ipi = s->ipi_pending[c] & ~s->ipi_mask[c];

        qemu_set_irq(s->irq[c], ipi || aic_next_hw(s, c) >= 0);
    }
}

static void aic_set_input(void *opaque, int n, int level)
{
    AppleAIC1State *s = opaque;

    if (n < 0 || n >= S5L8950_AIC_NR_IRQ) {
        return;
    }
    if (level) {
        s->hw[n / 32] |= 1u << (n % 32);
    } else {
        s->hw[n / 32] &= ~(1u << (n % 32));
    }
    aic_update(s);
}

static uint32_t aic_iack(AppleAIC1State *s, unsigned cpu)
{
    uint32_t live = s->ipi_pending[cpu] & ~s->ipi_mask[cpu];
    int n;

    if (live & AIC_IPI_OTHER) {
        s->ipi_mask[cpu] |= AIC_IPI_OTHER;
        aic_update(s);
        return AIC_EVT_IPI << 16 | 1;
    }
    if (live & AIC_IPI_SELF) {
        s->ipi_mask[cpu] |= AIC_IPI_SELF;
        aic_update(s);
        return AIC_EVT_IPI << 16 | 2;
    }
    n = aic_next_hw(s, cpu);
    if (n < 0) {
        return 0;
    }
    s->mask[n / 32] |= 1u << (n % 32);
    aic_update(s);
    return AIC_EVT_HW << 16 | n;
}

static uint64_t aic_cpu_read(AppleAIC1State *s, unsigned cpu, hwaddr off, bool *done)
{
    *done = true;
    switch (off) {
    case AIC_WHOAMI:
        return cpu;
    case AIC_IACK:
        return aic_iack(s, cpu);
    case AIC_IPI_SET:
        return s->ipi_pending[cpu];
    case AIC_IPI_MASK_SET:
    case AIC_IPI_MASK_CLR:
        return s->ipi_mask[cpu];
    }
    *done = false;
    return 0;
}

static bool aic_cpu_write(AppleAIC1State *s, unsigned cpu, hwaddr off, uint32_t v)
{
    switch (off) {
    case AIC_IPI_SET:
        for (unsigned c = 0; c < s->ncpu; c++) {
            if (v & (1u << c)) {
                s->ipi_pending[c] |= AIC_IPI_OTHER;
            }
        }
        if (v & AIC_IPI_SELF) {
            s->ipi_pending[cpu] |= AIC_IPI_SELF;
        }
        break;
    case AIC_IPI_CLR:
        s->ipi_pending[cpu] &= ~v;
        break;
    case AIC_IPI_MASK_SET:
        s->ipi_mask[cpu] |= v;
        break;
    case AIC_IPI_MASK_CLR:
        s->ipi_mask[cpu] &= ~v;
        break;
    default:
        return false;
    }
    aic_update(s);
    return true;
}

static uint64_t aic_read(void *opaque, hwaddr off, unsigned size)
{
    AppleAIC1State *s = opaque;
    bool done;
    uint64_t v;

    if (off == AIC_INFO) {
        return (s->ncpu - 1) << 16 | S5L8950_AIC_NR_IRQ;
    }
    if (off == AIC_CONFIG) {
        return s->config;
    }
    if (off >= AIC_CPU_BLOCK && off < AIC_CPU_BLOCK + AIC_PERCPU_STRIDE) {
        v = aic_cpu_read(s, aic_cpu_now(s), off - AIC_CPU_BLOCK, &done);
        if (done) {
            return v;
        }
    }
    if (off >= AIC_PERCPU && off < AIC_PERCPU + AIC_PERCPU_STRIDE * s->ncpu) {
        unsigned c = (off - AIC_PERCPU) / AIC_PERCPU_STRIDE;

        v = aic_cpu_read(s, c, (off - AIC_PERCPU) % AIC_PERCPU_STRIDE, &done);
        if (done) {
            return v;
        }
    }
    if (off >= AIC_TARGET && off < AIC_TARGET + 4 * S5L8950_AIC_NR_IRQ) {
        return s->target[(off - AIC_TARGET) / 4];
    }
    if (off >= AIC_SW_SET && off < AIC_SW_SET + 4 * NWORDS) {
        return s->sw[(off - AIC_SW_SET) / 4];
    }
    if (off >= AIC_SW_CLR && off < AIC_SW_CLR + 4 * NWORDS) {
        return s->sw[(off - AIC_SW_CLR) / 4];
    }
    if (off >= AIC_MASK_SET && off < AIC_MASK_SET + 4 * NWORDS) {
        return s->mask[(off - AIC_MASK_SET) / 4];
    }
    if (off >= AIC_MASK_CLR && off < AIC_MASK_CLR + 4 * NWORDS) {
        return s->mask[(off - AIC_MASK_CLR) / 4];
    }
    if (off >= AIC_HW_STATE && off < AIC_HW_STATE + 4 * NWORDS) {
        return s->hw[(off - AIC_HW_STATE) / 4];
    }
    qemu_log_mask(LOG_UNIMP, "aic1: read 0x%04" HWADDR_PRIx "\n", off);
    return s->scratch[off / 4];
}

static void aic_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    AppleAIC1State *s = opaque;
    uint32_t v = val;

    if (off == AIC_CONFIG) {
        s->config = v;
        return;
    }
    if (off >= AIC_CPU_BLOCK && off < AIC_CPU_BLOCK + AIC_PERCPU_STRIDE &&
        aic_cpu_write(s, aic_cpu_now(s), off - AIC_CPU_BLOCK, v)) {
        return;
    }
    if (off >= AIC_PERCPU && off < AIC_PERCPU + AIC_PERCPU_STRIDE * s->ncpu &&
        aic_cpu_write(s, (off - AIC_PERCPU) / AIC_PERCPU_STRIDE,
                      (off - AIC_PERCPU) % AIC_PERCPU_STRIDE, v)) {
        return;
    }
    if (off >= AIC_TARGET && off < AIC_TARGET + 4 * S5L8950_AIC_NR_IRQ) {
        s->target[(off - AIC_TARGET) / 4] = v;
    } else if (off >= AIC_SW_SET && off < AIC_SW_SET + 4 * NWORDS) {
        s->sw[(off - AIC_SW_SET) / 4] |= v;
    } else if (off >= AIC_SW_CLR && off < AIC_SW_CLR + 4 * NWORDS) {
        s->sw[(off - AIC_SW_CLR) / 4] &= ~v;
    } else if (off >= AIC_MASK_SET && off < AIC_MASK_SET + 4 * NWORDS) {
        s->mask[(off - AIC_MASK_SET) / 4] |= v;
    } else if (off >= AIC_MASK_CLR && off < AIC_MASK_CLR + 4 * NWORDS) {
        s->mask[(off - AIC_MASK_CLR) / 4] &= ~v;
    } else {
        qemu_log_mask(LOG_UNIMP, "aic1: write 0x%04" HWADDR_PRIx " = 0x%08x\n", off, v);
        s->scratch[off / 4] = v;
        return;
    }
    aic_update(s);
}

static const MemoryRegionOps aic_ops = {
    .read = aic_read,
    .write = aic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void aic_reset(DeviceState *dev)
{
    AppleAIC1State *s = APPLE_AIC1(dev);

    s->config = 0;
    memset(s->mask, 0xff, sizeof(s->mask));
    memset(s->sw, 0, sizeof(s->sw));
    memset(s->target, 0, sizeof(s->target));
    memset(s->ipi_pending, 0, sizeof(s->ipi_pending));
    memset(s->ipi_mask, 0xff, sizeof(s->ipi_mask));
    memset(s->scratch, 0, sizeof(s->scratch));
    aic_update(s);
}

static void aic_realize(DeviceState *dev, Error **errp)
{
    AppleAIC1State *s = APPLE_AIC1(dev);

    if (s->ncpu < 1 || s->ncpu > S5L8950_AIC_NR_CPU) {
        error_setg(errp, "aic1: ncpu must be 1..%d", S5L8950_AIC_NR_CPU);
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(s), &aic_ops, s, "aic1", S5L8950_AIC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    for (unsigned c = 0; c < S5L8950_AIC_NR_CPU; c++) {
        sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq[c]);
    }
    qdev_init_gpio_in(dev, aic_set_input, S5L8950_AIC_NR_IRQ);
}

static const VMStateDescription vmstate_aic = {
    .name = "apple-aic1",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(config, AppleAIC1State),
        VMSTATE_UINT32_ARRAY(mask, AppleAIC1State, NWORDS),
        VMSTATE_UINT32_ARRAY(sw, AppleAIC1State, NWORDS),
        VMSTATE_UINT32_ARRAY(hw, AppleAIC1State, NWORDS),
        VMSTATE_UINT32_ARRAY(target, AppleAIC1State, S5L8950_AIC_NR_IRQ),
        VMSTATE_UINT32_ARRAY(ipi_pending, AppleAIC1State, S5L8950_AIC_NR_CPU),
        VMSTATE_UINT32_ARRAY(ipi_mask, AppleAIC1State, S5L8950_AIC_NR_CPU),
        VMSTATE_UINT32_ARRAY(scratch, AppleAIC1State, 0x10000 / 4),
        VMSTATE_END_OF_LIST()
    }
};

static const Property aic_props[] = {
    DEFINE_PROP_UINT32("num-cpu", AppleAIC1State, ncpu, 1),
};

static void aic_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = aic_realize;
    device_class_set_legacy_reset(dc, aic_reset);
    dc->vmsd = &vmstate_aic;
    device_class_set_props(dc, aic_props);
}

static const TypeInfo aic_info = {
    .name = TYPE_APPLE_AIC1,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AppleAIC1State),
    .class_init = aic_class_init,
};

static void aic_register_types(void)
{
    type_register_static(&aic_info);
}

type_init(aic_register_types)
