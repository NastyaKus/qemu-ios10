/*
 * Apple S5L8950X "A6" boards: iPhone 5 (N41AP, -M iPhone-5; N42AP, -M
 * iPhone-5-GSM), dual Swift. Stage 1 of docs/n41/README.md: enter the iOS
 * 10.3.3 (14G60) kernel directly from a K48KBOOT bundle (firmwarekit's kboot,
 * the A4 machine's format; see hw/arm/ipad1.c), with no SecureROM or iBoot,
 * and get its console on UART0.
 *
 * Modelled: one Swift core (a Cortex-A15 that reports Apple's MIDR, so xnu's
 * cpuid takes it for CPUFAMILY_ARM_SWIFT), its generic timer on FIQ, 1 GiB of
 * DRAM, the AIC (s5l8950_aic.c), the Samsung UARTs and a plain scanout of the
 * boot framebuffer (boot_args Video). Every other arm-io address falls into an
 * unimplemented-device window that logs the access (-d unimp), so bring-up
 * can see what the kernel reaches for next.
 *
 * The CPU starts at the bundle's entry in ARM state with the MMU off and
 * r0 = boot_args, the state xnu's _start expects from iBoot.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/misc/unimp.h"
#include "hw/arm/exynos4210.h"
#include "hw/arm/s5l8950.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include "ui/console.h"

/* Apple's implementer, part 0: CPUFAMILY_ARM_SWIFT in 14G60's cpuid table (0x8037deb0). */
#define S5L8950_SWIFT_MIDR   0x610f0000
/* The DT's timebase-frequency (cpus/cpu0), which firmwarekit fills. */
#define S5L8950_TIMEBASE_HZ  24000000

#define KBOOT_MAGIC          "K48KBOOT"
#define KBOOT_TRAILER_LEN    24
#define KBOOT_SEGMENT_LEN    20

/* boot_args (xnu arm, Revision 1): Video {v_baseAddr, v_display, v_rowBytes, v_width, v_height, v_depth} at +0x14. */
#define BA_VIDEO_BASE        0x14
#define BA_VIDEO_ROWBYTES    0x1c
#define BA_VIDEO_WIDTH       0x20
#define BA_VIDEO_HEIGHT      0x24
#define BA_VIDEO_DEPTH       0x28

typedef struct A6Board {
    const char *desc;
    const char *board;       /* the catalog's / DT's board name */
    unsigned fb_width, fb_height;
} A6Board;

static const A6Board a6_n41 = {
    .desc = "Apple iPhone 5 (N41AP, S5L8950X)",
    .board = "n41ap",
    .fb_width = 640,
    .fb_height = 1136,
};

static const A6Board a6_n42 = {
    .desc = "Apple iPhone 5 GSM+CDMA (N42AP, S5L8950X)",
    .board = "n42ap",
    .fb_width = 640,
    .fb_height = 1136,
};

#define TYPE_S5L8950_MACHINE MACHINE_TYPE_NAME("s5l8950")
OBJECT_DECLARE_TYPE(S5L8950MachineState, S5L8950MachineClass, S5L8950_MACHINE)

struct S5L8950MachineClass {
    MachineClass parent;
    const A6Board *board;
};

struct S5L8950MachineState {
    MachineState parent;
    const A6Board *board;
    ARMCPU *cpu;
    AppleAIC1State *aic;
    QemuConsole *con;

    /* The boot framebuffer, from the bundle's boot_args. */
    uint32_t fb_pa, fb_stride, fb_width, fb_height;

    /* Properties. Those after kboot_path are what LightTouch's runtime passes every board
     * (Portable/src/Storage.cpp lt_device_boot_argv); stage 1 keeps them for later stages. */
    char *kboot_path;
    char *nand_path;
    char *nand_overlay;
    char *nor_rw;
    char *guest_package;
    char *die_id;
    char *wifi_mac;
    char *bt_mac;
    char *ecid;
    char *aes_uid;
    char *rtc_epoch;
    char *imei;
};

static void s5l8950_cpu_reset(void *opaque)
{
    S5L8950MachineState *s = S5L8950_MACHINE(opaque);
    CPUState *cs = CPU(s->cpu);
    g_autofree char *data = NULL;
    g_autoptr(GError) gerr = NULL;
    gsize size;
    const uint8_t *trailer;
    uint32_t load_pa, entry_pa, bootargs_pa, image_len, ba;

    cpu_reset(cs);
    if (!g_file_get_contents(s->kboot_path, &data, &size, &gerr)) {
        error_report("s5l8950: cannot read kboot bundle '%s': %s", s->kboot_path, gerr->message);
        exit(1);
    }
    if (size < KBOOT_TRAILER_LEN || memcmp(data + size - KBOOT_TRAILER_LEN, KBOOT_MAGIC, 8) != 0) {
        error_report("s5l8950: '%s' is not a K48KBOOT bundle", s->kboot_path);
        exit(1);
    }
    trailer = (const uint8_t *)data + size - KBOOT_TRAILER_LEN;
    load_pa = ldl_le_p(trailer + 8);
    entry_pa = ldl_le_p(trailer + 12);
    bootargs_pa = ldl_le_p(trailer + 16);
    image_len = ldl_le_p(trailer + 20);
    if (image_len > size - KBOOT_TRAILER_LEN || load_pa < S5L8950_DRAM_BASE ||
        (uint64_t)load_pa + image_len > (uint64_t)S5L8950_DRAM_BASE + MACHINE(s)->ram_size ||
        bootargs_pa < load_pa || bootargs_pa + 0x38 > load_pa + image_len) {
        error_report("s5l8950: kboot bundle does not fit in DRAM (load 0x%x len 0x%x)", load_pa, image_len);
        exit(1);
    }
    ba = bootargs_pa - load_pa;
    s->fb_pa = ldl_le_p(data + ba + BA_VIDEO_BASE);
    s->fb_stride = ldl_le_p(data + ba + BA_VIDEO_ROWBYTES);
    s->fb_width = ldl_le_p(data + ba + BA_VIDEO_WIDTH);
    s->fb_height = ldl_le_p(data + ba + BA_VIDEO_HEIGHT);
    if ((ldl_le_p(data + ba + BA_VIDEO_DEPTH) & 0xff) != 32 || s->fb_width > 4096 || s->fb_height > 4096 ||
        s->fb_stride < s->fb_width * 4) {
        warn_report("s5l8950: boot_args framebuffer is not 32 bpp; no display");
        s->fb_width = 0;
    }
    if (address_space_write(&address_space_memory, load_pa, MEMTXATTRS_UNSPECIFIED, data, image_len) != MEMTX_OK) {
        error_report("s5l8950: cannot stage kboot bundle at 0x%x", load_pa);
        exit(1);
    }
    for (gsize off = image_len; off + KBOOT_SEGMENT_LEN <= size - KBOOT_TRAILER_LEN;) {
        const uint8_t *seg = (const uint8_t *)data + off;
        uint32_t pa = ldl_le_p(seg + 8), len = ldl_le_p(seg + 12);
        bool zero = ldl_le_p(seg + 16) & 1;

        off += KBOOT_SEGMENT_LEN + (zero ? 0 : len);
        if (memcmp(seg, "K48SEG\0\0", 8) != 0 || off > size - KBOOT_TRAILER_LEN) {
            error_report("s5l8950: malformed segment in kboot bundle");
            exit(1);
        }
        if (zero) {
            address_space_set(&address_space_memory, pa, 0, len, MEMTXATTRS_UNSPECIFIED);
        } else {
            address_space_write(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED, seg + KBOOT_SEGMENT_LEN, len);
        }
    }
    /* cpu_reset leaves us in SVC mode, IRQ/FIQ masked, MMU and caches off. */
    s->cpu->env.regs[0] = bootargs_pa;
    cpu_set_pc(cs, entry_pa);
}

/* ---- the boot framebuffer ------------------------------------------------ */

static void s5l8950_fb_update(void *opaque)
{
    S5L8950MachineState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    unsigned w = s->fb_width ? s->fb_width : s->board->fb_width;
    unsigned h = s->fb_width ? s->fb_height : s->board->fb_height;

    if (surface_width(surface) != w || surface_height(surface) != h) {
        qemu_console_resize(s->con, w, h);
        surface = qemu_console_surface(s->con);
    }
    if (surface_bits_per_pixel(surface) != 32) {
        return;
    }
    for (unsigned y = 0; y < h; y++) {
        uint8_t *d = surface_data(surface) + y * surface_stride(surface);

        if (!s->fb_width) {
            memset(d, 0, w * 4);
            continue;
        }
        address_space_read(&address_space_memory, s->fb_pa + (hwaddr)y * s->fb_stride,
                           MEMTXATTRS_UNSPECIFIED, d, w * 4);
    }
    dpy_gfx_update(s->con, 0, 0, w, h);
}

static void s5l8950_fb_invalidate(void *opaque)
{
}

static const GraphicHwOps s5l8950_fb_ops = {
    .invalidate = s5l8950_fb_invalidate,
    .gfx_update = s5l8950_fb_update,
};

/* ---- machine ------------------------------------------------------------- */

static void s5l8950_init(MachineState *machine)
{
    S5L8950MachineState *s = S5L8950_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    DeviceState *cpudev, *dev;
    Object *cpuobj;

    s->board = S5L8950_MACHINE_GET_CLASS(s)->board;
    if (!s->kboot_path) {
        error_report("s5l8950: kboot= is required (no SecureROM or iBoot boot yet)");
        exit(1);
    }
    if (machine->ram_size != S5L8950_DRAM_SIZE) {
        error_report("s5l8950: the iPhone 5 has %d MiB of DRAM; -m must say so",
                     (int)(S5L8950_DRAM_SIZE / MiB));
        exit(1);
    }

    cpuobj = object_new(machine->cpu_type);
    s->cpu = ARM_CPU(cpuobj);
    cpudev = DEVICE(cpuobj);
    s->cpu->midr = S5L8950_SWIFT_MIDR;
    object_property_set_link(cpuobj, "memory", OBJECT(sysmem), &error_abort);
    object_property_set_bool(cpuobj, "has_el3", false, NULL);
    object_property_set_bool(cpuobj, "has_el2", false, NULL);
    object_property_set_int(cpuobj, "cntfrq", S5L8950_TIMEBASE_HZ, &error_abort);
    object_property_set_bool(cpuobj, "realized", true, &error_fatal);
    object_unref(cpuobj);
    /* xnu's rtclock runs on the generic timer and takes it as FIQ. */
    qdev_connect_gpio_out(cpudev, GTIMER_VIRT, qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
    qdev_connect_gpio_out(cpudev, GTIMER_PHYS, qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));

    memory_region_add_subregion(sysmem, S5L8950_DRAM_BASE, machine->ram);

    /* Everything in arm-io with no model yet; real devices sit on top of it. */
    create_unimplemented_device("s5l8950.io", S5L8950_IO_BASE, S5L8950_IO_SIZE);

    dev = qdev_new(TYPE_APPLE_AIC1);
    qdev_prop_set_uint32(dev, "num-cpu", 1);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, S5L8950_AIC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
    s->aic = APPLE_AIC1(dev);

    /* uart0 (iBoot's and the kernel's console, serial=3) through uart6, the A4's Samsung UART. */
    for (int i = 0; i < S5L8950_UART_COUNT; i++) {
        exynos4210_uart_create(S5L8950_UART_BASE(i), 256, i, i == 0 ? serial_hd(0) : NULL,
                               qdev_get_gpio_in(dev, S5L8950_IRQ_UART(i)), true);
    }

    s->con = graphic_console_init(NULL, 0, &s5l8950_fb_ops, s);
    qemu_register_reset(s5l8950_cpu_reset, s);
}

#define S5L8950_STR_PROP(field, name)                                                   \
    static char *s5l8950_get_##field(Object *obj, Error **errp)                        \
    {                                                                                  \
        return g_strdup(S5L8950_MACHINE(obj)->field);                                  \
    }                                                                                  \
    static void s5l8950_set_##field(Object *obj, const char *value, Error **errp)      \
    {                                                                                  \
        S5L8950MachineState *s = S5L8950_MACHINE(obj);                                 \
        g_free(s->field);                                                              \
        s->field = g_strdup(value);                                                    \
    }

S5L8950_STR_PROP(kboot_path, "kboot")
S5L8950_STR_PROP(nand_path, "nand")
S5L8950_STR_PROP(nand_overlay, "nand-overlay")
S5L8950_STR_PROP(nor_rw, "nor-rw")
S5L8950_STR_PROP(guest_package, "guest-package")
S5L8950_STR_PROP(die_id, "die-id")
S5L8950_STR_PROP(wifi_mac, "wifi-mac")
S5L8950_STR_PROP(bt_mac, "bt-mac")
S5L8950_STR_PROP(ecid, "ecid")
S5L8950_STR_PROP(aes_uid, "aes-uid")
S5L8950_STR_PROP(rtc_epoch, "rtc-epoch")
S5L8950_STR_PROP(imei, "imei")

static void s5l8950_instance_finalize(Object *obj)
{
    S5L8950MachineState *s = S5L8950_MACHINE(obj);

    g_free(s->kboot_path);
    g_free(s->nand_path);
    g_free(s->nand_overlay);
    g_free(s->nor_rw);
    g_free(s->guest_package);
    g_free(s->die_id);
    g_free(s->wifi_mac);
    g_free(s->bt_mac);
    g_free(s->ecid);
    g_free(s->aes_uid);
    g_free(s->rtc_epoch);
    g_free(s->imei);
}

static void s5l8950_class_init(ObjectClass *klass, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(klass);

    mc->init = s5l8950_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a15");
    mc->default_ram_size = S5L8950_DRAM_SIZE;
    mc->default_ram_id = "s5l8950.dram";
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->no_parallel = 1;

#define ADD(field, name, what)                                                          \
    object_class_property_add_str(klass, name, s5l8950_get_##field, s5l8950_set_##field); \
    object_class_property_set_description(klass, name, what)
    ADD(kboot_path, "kboot", "K48KBOOT bundle from firmwarekit (kernel, device tree, boot_args)");
    ADD(nand_path, "nand", "the prepared NAND (accepted; no PPN NAND model yet)");
    ADD(nand_overlay, "nand-overlay", "the device's NAND overlay (accepted; no PPN NAND model yet)");
    ADD(nor_rw, "nor-rw", "accepted; the iPhone 5 has no NOR");
    ADD(guest_package, "guest-package", "LightTouch guest package (accepted; not served yet)");
    ADD(die_id, "die-id", "the unit's die id (accepted)");
    ADD(wifi_mac, "wifi-mac", "Wi-Fi MAC (accepted)");
    ADD(bt_mac, "bt-mac", "Bluetooth MAC (accepted)");
    ADD(ecid, "ecid", "ECID (accepted)");
    ADD(aes_uid, "aes-uid", "UID key (accepted)");
    ADD(rtc_epoch, "rtc-epoch", "RTC epoch (accepted)");
    ADD(imei, "imei", "IMEI (accepted)");
#undef ADD
}

static void a6_board_class_init(ObjectClass *klass, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(klass);
    const A6Board *board = data;

    S5L8950_MACHINE_CLASS(klass)->board = board;
    mc->desc = board->desc;
}

static const TypeInfo s5l8950_types[] = {
    {
        .name = TYPE_S5L8950_MACHINE,
        .parent = TYPE_MACHINE,
        .abstract = true,
        .instance_size = sizeof(S5L8950MachineState),
        .instance_finalize = s5l8950_instance_finalize,
        .class_size = sizeof(S5L8950MachineClass),
        .class_init = s5l8950_class_init,
        .interfaces = arm_machine_interfaces,
    }, {
        .name = MACHINE_TYPE_NAME("iPhone-5"),
        .parent = TYPE_S5L8950_MACHINE,
        .class_init = a6_board_class_init,
        .class_data = &a6_n41,
    }, {
        .name = MACHINE_TYPE_NAME("iPhone-5-GSM"),
        .parent = TYPE_S5L8950_MACHINE,
        .class_init = a6_board_class_init,
        .class_data = &a6_n42,
    },
};

DEFINE_TYPES(s5l8950_types)
