/*
 * Apple S5L8950X ("A6") SoC, as used by the iPhone 5 (N41AP/N42AP, -M iPhone-5).
 *
 * Addresses: the 14G60 (iOS 10.3.3) N41AP device tree; arm-io maps child offsets
 * at 0x30000000 (ranges 0 0x30000000 0x10000000). Kernel-side contracts:
 * docs/n41/README.md.
 */
#ifndef HW_ARM_S5L8950_H
#define HW_ARM_S5L8950_H

#include "hw/core/sysbus.h"

/* Memory: 1 GiB of DRAM; xnu's physBase == virtBase == 0x80000000. */
#define S5L8950_DRAM_BASE        0x80000000
#define S5L8950_DRAM_SIZE        0x40000000

/* arm-io */
#define S5L8950_IO_BASE          0x30000000
#define S5L8950_IO_SIZE          0x10000000
#define S5L8950_UART_BASE(n)     (0x32500000 + (n) * 0x100000)   /* uart0 is iBoot's console */
#define S5L8950_UART_COUNT       7
#define S5L8950_PMGR_BASE        0x3f100000   /* device_type "timer"; wdt at +0x3020 */
#define S5L8950_PMGR_SIZE        0x7000
#define S5L8950_AIC_BASE         0x3f200000
#define S5L8950_AIC_SIZE         0x10000
#define S5L8950_GPIO_BASE        0x3fa00000
#define S5L8950_GPIO_SIZE        0x1000

/* AIC inputs (the DT's "interrupts") */
#define S5L8950_AIC_NR_IRQ       192          /* ipid-mask: 6 words; IPIs start at 0xc0 */
#define S5L8950_AIC_NR_CPU       2
#define S5L8950_IRQ_UART(n)      (0x15 + (n)) /* uart1 0x16 ... uart6 0x1b */
#define S5L8950_IRQ_WDT          0x04
#define S5L8950_IRQ_GPIO         0x77

/*
 * The Apple Interrupt Controller, version 1 (DT compatible "aic,1",
 * AppleInterruptController). Register map: docs/n41/README.md.
 */
#define TYPE_APPLE_AIC1 "apple-aic1"
OBJECT_DECLARE_SIMPLE_TYPE(AppleAIC1State, APPLE_AIC1)

struct AppleAIC1State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[S5L8950_AIC_NR_CPU];
    uint32_t ncpu;

    uint32_t config;
    uint32_t mask[S5L8950_AIC_NR_IRQ / 32];    /* 1 = masked */
    uint32_t sw[S5L8950_AIC_NR_IRQ / 32];      /* software-set lines */
    uint32_t hw[S5L8950_AIC_NR_IRQ / 32];      /* input levels */
    uint32_t target[S5L8950_AIC_NR_IRQ];       /* CPU mask per input */
    uint32_t ipi_pending[S5L8950_AIC_NR_CPU];  /* bit 0 other, bit 31 self */
    uint32_t ipi_mask[S5L8950_AIC_NR_CPU];     /* 1 = masked */
    uint32_t scratch[0x10000 / 4];             /* registers with no behaviour yet */
};

#endif
