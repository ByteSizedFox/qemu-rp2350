/*
 * Raspberry Pi RP2350 machine
 *
 * Dual Cortex-M33, 32KB ROM, 4MB XIP flash, 520KB SRAM,
 * PL011 UARTs at RP2350 peripheral addresses.
 *
 * Copyright (c) 2024 QEMU contributors
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "hw/arm/boot.h"
#include "hw/arm/armv7m.h"
#include "hw/arm/machines-qom.h"
#include "hw/char/pl011.h"
#include "hw/char/serial.h"
#include "hw/core/irq.h"
#include "chardev/char.h"
#include "hw/core/boards.h"
#include "hw/core/cpu.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/unimp.h"
#include "target/arm/cpu.h"
#include "target/arm/tcg/idau.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "system/system.h"
#include "qom/object.h"

/* UF2 file format constants */
#define UF2_MAGIC0           0x0A324655u   /* "UF2\n" */
#define UF2_MAGIC1           0x9E5D5157u
#define UF2_MAGIC_END        0x0AB16F30u
#define UF2_FLAG_NOFLASH     0x00000001u   /* block should not be flashed */
#define UF2_FLAG_FILE_CONT   0x00001000u   /* file-container block, not data */
#define UF2_BLOCK_SIZE       512u

/* RP2350 memory map */
#define RP2350_ROM_BASE     0x00000000u
#define RP2350_ROM_SIZE     (32u * KiB)
#define RP2350_FLASH_BASE   0x10000000u
#define RP2350_FLASH_SIZE   (4u * MiB)   /* Pico 2 default; override with flash-size= */
#define RP2350_SRAM_BASE    0x20000000u
#define RP2350_SRAM_SIZE    (520u * KiB)    /* 8×64KB + 2×4KB scratch */

/* APB peripheral region — one big stub, UARTs overlaid on top */
#define RP2350_APB_BASE     0x40000000u
#define RP2350_APB_SIZE     0x00200000u     /* covers all APB devices */

/*
 * CLOCKS peripheral at 0x40010000.
 * Each of 12 clock domains occupies 12 bytes: CTRL(+0), DIV(+4), SELECTED(+8).
 * Atomic access aliases occupy the 16 KB range:
 *   +0x0000 normal, +0x1000 XOR, +0x2000 SET, +0x3000 CLR.
 * SELECTED returns (1 << SRC) where SRC is the low two bits of CTRL.
 * Reset default: CTRL=0, SELECTED=1 (source 0 = ROSC).
 */
#define RP2350_CLOCKS_BASE  0x40010000u
#define RP2350_CLOCKS_SIZE  0x00004000u     /* 16 KB: normal + 3 atomic aliases */
#define CLOCKS_NUM_DOMAINS  12

/* BOOTRAM: always-on RAM in APB region, used by bootrom for callbacks/state */
#define RP2350_BOOTRAM_BASE 0x400E0000u
#define RP2350_BOOTRAM_SIZE 0x00001000u  /* 4 KB */

/* TIMER0/1: 64-bit µs-resolution free-running counters */
#define RP2350_TIMER0_BASE  0x400B0000u
#define RP2350_TIMER0_SIZE  0x00004000u  /* 16 KB: normal + 3 atomic aliases */
#define RP2350_TIMER1_BASE  0x400B8000u
#define RP2350_TIMER1_SIZE  0x00004000u

/* Timer register offsets (identical layout for TIMER0 and TIMER1) */
#define TIMER_TIMEHR        0x008   /* high half, latching read               */
#define TIMER_TIMELR        0x00C   /* low half, latching read                */
#define TIMER_TIMERAWH      0x024   /* high half, raw (no latch)              */
#define TIMER_TIMERAWL      0x028   /* low half, raw                          */

/* Key peripheral addresses */
#define RP2350_UART0_BASE   0x40070000u
#define RP2350_UART1_BASE   0x40078000u

/* AHB-Lite region (DMA / USB / PIO / etc.) */
#define RP2350_AHB_BASE     0x50000000u
#define RP2350_AHB_SIZE     0x00800000u

/* SIO: same physical block, but CPUID reads per-CPU */
#define RP2350_SIO_BASE         0xD0000000u
#define RP2350_SIO_SIZE         0x00020000u

/* SIO register offsets */
#define SIO_CPUID           0x000   /* reads 0 on CPU0, 1 on CPU1          */
#define SIO_GPIO_HI_IN      0x008   /* QSPI/high GPIO input state           */
#define SIO_FIFO_ST         0x050   /* inter-core FIFO status               */
#define SIO_FIFO_WR         0x054   /* inter-core FIFO write (CPU→other)    */
#define SIO_FIFO_RD         0x058   /* inter-core FIFO read                 */
#define SIO_SPINLOCK_BASE   0x100   /* 32 spinlocks at 0x100–0x17C          */
#define SIO_SPINLOCK_END    0x180

/* SIO_GPIO_HI_IN bit fields (RP2350 datasheet) */
#define SIO_GPIO_HI_IN_QSPI_CSN_BIT  (1u << 27)  /* QSPI CSn: HIGH = not pressed */

/* SIO FIFO status bits */
#define SIO_FIFO_ST_VLD     (1u << 0)  /* RX FIFO has data                  */
#define SIO_FIFO_ST_RDY     (1u << 1)  /* TX FIFO not full                  */
#define SIO_FIFO_ST_WOF     (1u << 2)  /* TX overflow error (W1C)           */
#define SIO_FIFO_ST_ROE     (1u << 3)  /* RX underflow error (W1C)          */

/* PSM (Power State Manager) */
#define RP2350_PSM_BASE         0x40018000u
#define RP2350_PSM_SIZE         0x00004000u  /* normal + 3 atomic aliases    */
#define PSM_FRCE_OFF_OFF        0x004        /* FRCE_OFF register offset     */
#define PSM_FRCE_OFF_PROC1_BIT  0x01000000u  /* bit 24 = PROC1               */

/* Inter-core FIFO depth (hardware is 8 entries) */
#define SIO_FIFO_DEPTH      8

/* IRQ numbers (NVIC lines, 0-based) */
#define RP2350_UART0_IRQ    33   /* RP2350 UART0 IRQ (RP2040 had 20) */
#define RP2350_UART1_IRQ    34
#define RP2350_NUM_IRQS     52

/* Default system clock: 125 MHz */
#define RP2350_SYSCLK_HZ    125000000u

#define TYPE_RP2350_MACHINE MACHINE_TYPE_NAME("rp2350")
OBJECT_DECLARE_SIMPLE_TYPE(RP2350MachineState, RP2350_MACHINE)

/* Inter-core FIFO queue (one direction) */
typedef struct {
    uint32_t buf[SIO_FIFO_DEPTH];
    int rptr, wptr, count;
    bool roe, wof;
} SioFifoQ;

struct RP2350MachineState {
    MachineState parent;

    ARMv7MState cpu[2];
    Clock      *sysclk;

    MemoryRegion rom;        /* Boot ROM at 0x00000000 (32 KB)               */
    MemoryRegion flash;      /* XIP flash at 0x10000000 (4 MB)               */
    MemoryRegion sram;       /* SRAM at 0x20000000 (520 KB)                  */
    MemoryRegion bootrom_stack; /* Bootrom CPU stack below 0xF0000000 (8 KB)  */
    MemoryRegion apb_stub;   /* catch-all APB stub — reads return 0xFFFFFFFF */
    MemoryRegion clocks_mr;  /* CLOCKS at 0x40010000 (with SELECTED support) */
    MemoryRegion sio_mr;     /* SIO at 0xD0000000                            */
    MemoryRegion psm_mr;     /* PSM at 0x40018000                            */
    MemoryRegion bootram_mr; /* BOOTRAM at 0x400E0000 (1 KB always-on RAM)   */
    MemoryRegion timer0_mr;  /* TIMER0 at 0x400B0000 (µs free-running)       */
    MemoryRegion timer1_mr;  /* TIMER1 at 0x400B8000                         */
    MemoryRegion usb_dpram;  /* USB DPRAM at 0x50100000 (4 KB NS stack)      */
    uint8_t      usb_dpram_buf[4096]; /* backing store for USB DPRAM IO region */
    MemoryRegion usb_mr;     /* USB controller at 0x50110000                  */

    uint32_t clocks_ctrl[CLOCKS_NUM_DOMAINS]; /* SRC bits per clock domain    */

    /*
     * ARMv7M adds its board_memory as a subregion of its internal container,
     * which sets board_memory->container.  Two ARMv7M instances therefore
     * cannot share the same MemoryRegion.  CPU1 gets an alias of sys_mem so
     * both CPUs see the same physical address space without aliasing conflicts.
     */
    MemoryRegion cpu1_mem_alias;

    /*
     * Inter-core SIO FIFOs.
     * sio_fifo[0] = CPU0 → CPU1  (CPU0 writes, CPU1 reads)
     * sio_fifo[1] = CPU1 → CPU0  (CPU1 writes, CPU0 reads)
     * These are accessed under the MMIO serialisation guarantee.
     */
    SioFifoQ sio_fifo[2];

    /* PSM (Power State Manager) register state */
    uint32_t psm_frce_off;

    /*
     * BOOTRAM backing store (4 KB).
     * The Secure function-table range (offsets 0x80C–0x828, written by the
     * ROM "RS" function via stmia) must survive NS zeroing from init_array[5].
     * NS writes to that range are silently discarded; Secure writes go through.
     */
    uint8_t bootram[RP2350_BOOTRAM_SIZE];

    /*
     * CPU1 multicore launch state machine.
     * The RP2350 bootrom ordinarily handles the 6-word FIFO handshake:
     *   {0, 0, 1, vtor, sp, entry}
     * Since CPU1's bootrom hits a lockup before reaching the FIFO poll loop,
     * we simulate the handshake entirely in the emulator.
     */
    int      cpu1_launch_step;   /* 0..5: next expected word index           */
    uint32_t cpu1_launch_vtor;
    uint32_t cpu1_launch_sp;
    uint32_t cpu1_launch_entry;

    /* QMI (QSPI Memory Interface) SPI flash state machine */
    int      qmi_rx_count;   /* bytes pending in RX FIFO (1 per NOPUSH=0 TX entry) */
    uint8_t  qmi_spi_cmd;    /* current SPI command byte (0 = idle) */
    uint8_t  qmi_spi_addr[3];/* address accumulator for erase/program */
    int      qmi_spi_addr_len;/* address bytes received so far */
    bool     qmi_spi_in_data; /* true while receiving PP page-program data */
    uint32_t qmi_spi_pp_off;  /* flash byte offset for current page program */
    uint32_t qmi_spi_pp_cnt;  /* data bytes written in current page program */

    /* USB CDC state machine */
    int      usb_state;
    uint32_t usb_main_ctrl;
    uint32_t usb_sie_status;    /* RW1C */
    uint32_t usb_buff_status;   /* RW1C */
    uint32_t usb_inte;
    uint32_t usb_intr;
    qemu_irq usb_irq;
    QEMUTimer *usb_timer;

    /* Persistent flash backing file (set via -machine rp2350,flash=path) */
    char    *flash_file;
    Notifier flash_save_notifier;
};

/* --------------------------------------------------------------------------
 * BOOTRAM custom IO handler (4 KB at 0x400E0000)
 *
 * The ROM "RS" function writes r0–r7 to BOOTRAM+0x80C–0x828 via stmia
 * twice during boot: once during Secure ROM init (writing values like the
 * ROM function-table pointer), and once from MicroPython's init_array[0]
 * via the NSC gate (writing MicroPython's init_array pointers).
 *
 * MicroPython's init_array[5] then zero-fills that same range, and
 * init_array[7] polls BOOTRAM+0x828 waiting for a non-zero value.
 *
 * On real RP2350 the bootrom locks that range against NS writes, so the
 * zero-fill is discarded and BOOTRAM+0x828 keeps the value set by the
 * second RS call.  In QEMU, attrs.secure reflects the CPU's Secure state
 * (MicroPython runs Secure here), so we cannot filter by security attribute.
 * Instead we mimic the lock: once a word in 0x80C–0x828 is non-zero, a
 * write of 0x0 to that word is silently dropped.  This lets both RS calls
 * (non-zero writes) go through while blocking init_array[5]'s zero-fill.
 * -------------------------------------------------------------------------- */

/* Byte range within BOOTRAM that is protected against zero-overwrite */
#define BOOTRAM_SECURE_LO  0x80Cu
#define BOOTRAM_SECURE_HI  0x82Cu   /* exclusive end (8 words, 32 bytes) */

static MemTxResult bootram_read_with_attrs(void *opaque, hwaddr addr,
                                            uint64_t *data, unsigned size,
                                            MemTxAttrs attrs)
{
    RP2350MachineState *s = opaque;
    uint64_t val = 0;
    for (unsigned i = 0; i < size; i++) {
        val |= (uint64_t)s->bootram[addr + i] << (8u * i);
    }
    *data = val;
    return MEMTX_OK;
}

static MemTxResult bootram_write_with_attrs(void *opaque, hwaddr addr,
                                             uint64_t data, unsigned size,
                                             MemTxAttrs attrs)
{
    RP2350MachineState *s = opaque;
    /*
     * Once a word in the protected function-table range has been set to a
     * non-zero value by the ROM "RS" function, discard any attempt to write
     * 0x00000000 back to it.  This prevents init_array[5]'s zero-fill from
     * clobbering the value that init_array[7] polls.
     */
    if (size == 4 && data == 0 &&
        addr >= BOOTRAM_SECURE_LO && addr < BOOTRAM_SECURE_HI) {
        uint32_t cur = 0;
        for (unsigned i = 0; i < 4; i++) {
            cur |= (uint32_t)s->bootram[addr + i] << (8u * i);
        }
        if (cur != 0) {
            return MEMTX_OK;  /* silently drop zero-over-nonzero */
        }
    }
    for (unsigned i = 0; i < size; i++) {
        s->bootram[addr + i] = (uint8_t)(data >> (8u * i));
    }
    return MEMTX_OK;
}

static const MemoryRegionOps bootram_ops = {
    .read_with_attrs  = bootram_read_with_attrs,
    .write_with_attrs = bootram_write_with_attrs,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* --------------------------------------------------------------------------
 * CLOCKS peripheral stub (0x40010000, 16 KB including atomic aliases)
 *
 * The SDK polls each clock's SELECTED register (at domain_offset + 8) waiting
 * for (1 << src) to become non-zero after switching sources.  We track the
 * SRC bits written to each CTRL register and return SELECTED = (1 << src).
 * Atomic aliases: +0x1000 XOR, +0x2000 SET, +0x3000 CLR.
 * -------------------------------------------------------------------------- */
static uint64_t clocks_read(void *opaque, hwaddr addr, unsigned int size)
{
    RP2350MachineState *s = opaque;
    uint32_t reg_off = addr & 0xFFF;
    uint32_t clk     = reg_off / 12;
    uint32_t reg     = reg_off % 12;

    if (clk >= CLOCKS_NUM_DOMAINS) {
        return 0;
    }

    switch (reg) {
    case 0: /* CTRL */
        return s->clocks_ctrl[clk];
    case 4: /* DIV */
        return 0x10000; /* Default divider = 1.0 (0x1 in integer part) */
    case 8: /* SELECTED */
        return 1u << (s->clocks_ctrl[clk] & 0x3f);
    default:
        return 0;
    }
}

static void clocks_write(void *opaque, hwaddr addr,
                         uint64_t data, unsigned int size)
{
    RP2350MachineState *s = opaque;
    uint32_t alias   = (addr >> 12) & 3;   /* 0=RW 1=XOR 2=SET 3=CLR       */
    uint32_t reg_off = addr & 0xFFF;
    uint32_t clk     = reg_off / 12;
    uint32_t reg     = reg_off % 12;

    if (clk >= CLOCKS_NUM_DOMAINS || reg != 0) {
        return;   /* only CTRL register (offset 0) affects SELECTED          */
    }
    uint32_t old = s->clocks_ctrl[clk];
    switch (alias) {
    case 0: s->clocks_ctrl[clk]  = (uint32_t)data; break;
    case 1: s->clocks_ctrl[clk] ^= (uint32_t)data; break;
    case 2: s->clocks_ctrl[clk] |= (uint32_t)data; break;
    case 3: s->clocks_ctrl[clk] &= ~(uint32_t)data; break;
    }
    qemu_log("clocks_write: clk=%d alias=%d data=0x%" PRIx64 " old=0x%x -> new=0x%x\n",
             clk, alias, data, old, s->clocks_ctrl[clk]);
}

static const MemoryRegionOps clocks_ops = {
    .read  = clocks_read,
    .write = clocks_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* --------------------------------------------------------------------------
 * APB peripheral stub
 * Programs poll RESETS.RESET_DONE and CLOCKS.PLL lock bits.
 * Returning all-ones lets them proceed without spinning.
 * -------------------------------------------------------------------------- */
static uint64_t apb_stub_read(void *opaque, hwaddr addr, unsigned int size)
{
    RP2350MachineState *s = opaque;
    uint32_t reg_off = addr & 0xFFF;
    uint32_t val = 0;

    /* SYSINFO at 0x40000000 (with atomic aliases up to 0x40003FFF) */
    if (addr < 0x4000) {
        switch (reg_off) {
        case 0x00: /* CHIP_ID */
            val = 0x00040001u; /* RP2350, stop 1 */
            goto out;
        case 0x04: /* PLATFORM */
            val = 0x00000001u; /* ASIC */
            goto out;
        default:
            break;
        }
    }

    /* ROSC at 0x400E8000 */
    if ((addr & ~0x3FFFu) == 0xE8000) {
        if (reg_off == 0x1c) { /* STATUS */
            val = 0x80000000u; /* STABLE=1 */
            goto out;
        }
        val = 0;
        goto out;
    }

    /* XOSC at 0x40048000 */
    if ((addr & ~0x3FFFu) == 0x48000) {
        if (reg_off == 0x04) { /* STATUS */
            val = 0x80000000u; /* STABLE=1 */
            goto out;
        }
        val = 0;
        goto out;
    }

    /* PLL_SYS at 0x40050000, PLL_USB at 0x40058000 (RP2350) with atomic aliases */
    if ((addr & ~0x3FFFu) == 0x50000 || (addr & ~0x3FFFu) == 0x58000) {
        if (reg_off == 0x00) { /* CS */
            val = 0x80000000u; /* LOCK=1 */
            goto out;
        }
        val = 0;
        goto out;
    }

    /* RESETS at 0x4000c000 (RP2040 alias, kept for safety) */
    if ((addr & ~0x3FFFu) == 0x0c000) {
        if (reg_off == 0x08) { /* RESET_DONE */
            val = 0xFFFFFFFFu;
            goto out;
        }
        val = 0;
        goto out;
    }

    /* RESETS at 0x40020000 (RP2350) — 4 atomic aliases at +0/+0x1000/+0x2000/+0x3000 */
    if ((addr & ~0x3FFFu) == 0x20000) {
        if (reg_off == 0x08) { /* RESET_DONE */
            val = 0xFFFFFFFFu;
            goto out;
        }
        val = 0;
        goto out;
    }

    /* IO_QSPI at 0x40030000 (status registers at 0x0, 0x8, 0x10, ... 0x38) */
    if ((addr & ~0x3FFFu) == 0x30000) {
        if ((reg_off & 0x7) == 0 && reg_off <= 0x38) {
            val = 0xFFFFFFFFu;
            goto out;
        }
        val = 0;
        goto out;
    }

    /* WATCHDOG at 0x40088000 (RP2350 address) */
    if ((addr & ~0x3FFFu) == 0x88000) {
        if (reg_off >= 0x0c) { /* SCRATCH registers */
            val = 0;
            goto out;
        }
        val = 0;
        goto out;
    }

    /* QMI at 0x400D0000 (with atomic aliases up to 0x400D3FFF) */
    if ((addr & ~0x3FFFu) == 0xD0000) {
        if (reg_off == 0) { /* DIRECT_CSR */
            val = 0x00000800u; /* TXEMPTY=1 */
            if (s->qmi_rx_count == 0) {
                val |= 0x00010000u; /* RXEMPTY=1 */
            }
            goto out;
        }
        if (reg_off == 0x08) { /* DIRECT_RX */
            if (s->qmi_rx_count > 0) {
                s->qmi_rx_count--;
            }
            val = 0;
            goto out;
        }
        val = 0;
        goto out;
    }

    /* SHA256 at 0x400F8000 */
    if ((addr & ~0x3FFFu) == 0xF8000) {
        if (reg_off == 0) { /* CSR */
            val = 0x1206; /* Reset value: SUM_VLD=1, WDATA_RDY=1, BSWAP=1, DMA_SIZE=32bit */
            goto out;
        }
        if (reg_off == 0x08) { /* SUM0 */
            val = 0x12345678; /* Dummy hash result */
            goto out;
        }
        val = 0;
        goto out;
    }

    /* TRNG at 0x400F0000 */
    if ((addr & ~0x3FFFu) == 0xF0000) {
        if (reg_off == 0x104 || reg_off == 0x110) { /* RNG_ISR or TRNG_VALID */
            val = 1; /* EHR_VALID=1 */
            goto out;
        }
        if (reg_off == 0x1b8) { /* TRNG_BUSY */
            val = 0; /* BUSY=0 */
            goto out;
        }
        if (reg_off >= 0x114 && reg_off <= 0x128) { /* EHR_DATA0..5 */
            static uint32_t seed = 0x12345678;
            seed = seed * 1103515245 + 12345;
            val = seed;
            goto out;
        }
        val = 0;
        goto out;
    }

    /* OTP DATA at 0x40130000 (DATA), 0x40134000 (RAW), 0x40138000 (GUARDED), 0x4013c000 (RAW_GUARDED) */
    if ((addr & ~0x3FFFu) == 0x130000 || (addr & ~0x3FFFu) == 0x134000 ||
        (addr & ~0x3FFFu) == 0x138000 || (addr & ~0x3FFFu) == 0x13C000) {
        /*
         * BOOT_FLAGS0 at word 72, 73, 74 (offsets 0x120, 0x124, 0x128)
         * Return 0 to ensure DISABLE_FLASH_BOOT (bit 12) is 0.
         */
        if (reg_off == 0x120 || reg_off == 0x124 || reg_off == 0x128) {
            val = 0;
            goto out;
        }
        val = 0;
        goto out;
    }

    /* TICKS at 0x40108000 */
    if ((addr & ~0x3FFFu) == 0x108000) {
        if (reg_off == 0x08) { /* COUNT */
            static uint32_t count = 0;
            val = count++;
            goto out;
        }
        val = 0;
        goto out;
    }

    /* POWMAN at 0x40100000: reset values are all 0; CHIP_RESET=0 means no
     * rescue flag (bit 4), no prior reset events — normal cold boot. */
    if (addr >= 0x100000 && addr < 0x108000) {
        val = 0;
        goto out;
    }

    /* Default: most RP2350 APB registers reset to 0. */
    val = 0;

out:
    qemu_log("apb_stub_read: @ 0x%" HWADDR_PRIx " = 0x%x\n", addr, val);
    return val;
}

static void apb_stub_write(void *opaque, hwaddr addr,
                           uint64_t data, unsigned int size)
{
    RP2350MachineState *s = opaque;
    uint32_t reg_off = addr & 0xFFF;

    qemu_log("apb_stub_write: @ 0x%" HWADDR_PRIx " = 0x%" PRIx64 "\n", addr, data);

    /* QMI at 0x400D0000 */
    if ((addr & ~0x3FFFu) == 0xD0000) {
        if (reg_off == 0x04) { /* DIRECT_TX */
            bool nopush = (data >> 20) & 1;
            bool dwidth = (data >> 18) & 1;
            /* Extract bytes: DWIDTH=1 sends DATA[7:0] first, then DATA[15:8] */
            uint8_t b[2] = { data & 0xFF, (data >> 8) & 0xFF };
            int nb = dwidth ? 2 : 1;

            for (int i = 0; i < nb; i++) {
                uint8_t byte = b[i];

                /*
                 * PP data phase ends when a NOPUSH=1 byte arrives — that's
                 * the start of the next command (e.g. RDSR busy-poll).
                 * The data has already been written byte-by-byte, so just
                 * reset the state.
                 */
                if (nopush && s->qmi_spi_in_data) {
                    s->qmi_spi_in_data = false;
                    s->qmi_spi_cmd     = 0;
                    s->qmi_spi_pp_cnt  = 0;
                }

                if (s->qmi_spi_cmd == 0) {
                    /* First byte of a new SPI transaction: decode command. */
                    switch (byte) {
                    case 0x06: /* WREN */
                    case 0x04: /* WRDI */
                        break; /* single-byte commands; no further bytes */
                    case 0x05: /* RDSR */
                    case 0x35: /* RDSR2 */
                    case 0x01: /* WRSR */
                    case 0x20: /* Sector Erase 4 KB */
                    case 0x02: /* Page Program */
                    case 0xD8: /* Block Erase 64 KB */
                        s->qmi_spi_cmd      = byte;
                        s->qmi_spi_addr_len = 0;
                        break;
                    default:
                        break;
                    }
                } else if (s->qmi_spi_cmd == 0x05 || s->qmi_spi_cmd == 0x35) {
                    /* RDSR/RDSR2: one dummy byte clocks out the status byte. */
                    s->qmi_spi_cmd = 0;
                } else if (s->qmi_spi_cmd == 0x01) {
                    /* WRSR: two data bytes (SR1, SR2). */
                    if (++s->qmi_spi_addr_len >= 2) {
                        s->qmi_spi_cmd = 0;
                    }
                } else if (s->qmi_spi_cmd == 0x02 && s->qmi_spi_in_data) {
                    /* Page-program data byte: AND into flash (NOR flash semantics).
                     * This must come BEFORE the address-accumulation branch below
                     * because cmd==0x02 also matches that branch. */
                    uint8_t *fp = memory_region_get_ram_ptr(&s->flash);
                    /* Writes wrap within the 256-byte page. */
                    uint32_t page = s->qmi_spi_pp_off & ~0xFFu;
                    uint32_t pos  = (s->qmi_spi_pp_off + s->qmi_spi_pp_cnt) & 0xFF;
                    uint32_t tgt  = page + pos;
                    if (tgt < RP2350_FLASH_SIZE) {
                        fp[tgt] &= byte;
                    }
                    s->qmi_spi_pp_cnt++;
                } else if (s->qmi_spi_cmd == 0x20 || s->qmi_spi_cmd == 0x02
                           || s->qmi_spi_cmd == 0xD8) {
                    /* Accumulate 3-byte address. */
                    if (s->qmi_spi_addr_len < 3) {
                        s->qmi_spi_addr[s->qmi_spi_addr_len++] = byte;
                    }
                    if (s->qmi_spi_addr_len == 3) {
                        uint32_t off =
                            ((uint32_t)s->qmi_spi_addr[0] << 16) |
                            ((uint32_t)s->qmi_spi_addr[1] <<  8) |
                             (uint32_t)s->qmi_spi_addr[2];
                        uint8_t *fp = memory_region_get_ram_ptr(&s->flash);

                        if (s->qmi_spi_cmd == 0x20) {
                            /* Sector erase: fill 4 KB with 0xFF */
                            uint32_t base = off & ~0xFFFu;
                            if (base + 4096 <= RP2350_FLASH_SIZE) {
                                memset(fp + base, 0xFF, 4096);
                            }
                            s->qmi_spi_cmd = 0;
                        } else if (s->qmi_spi_cmd == 0xD8) {
                            /* Block erase: fill 64 KB with 0xFF */
                            uint32_t base = off & ~0xFFFFu;
                            if (base + 65536 <= RP2350_FLASH_SIZE) {
                                memset(fp + base, 0xFF, 65536);
                            }
                            s->qmi_spi_cmd = 0;
                        } else {
                            /* Page Program: enter data phase. */
                            s->qmi_spi_pp_off  = off;
                            s->qmi_spi_pp_cnt  = 0;
                            s->qmi_spi_in_data = true;
                        }
                    }
                }
            }

            /* Each TX FIFO entry with NOPUSH=0 contributes one RX byte. */
            if (!nopush) {
                s->qmi_rx_count++;
            }
        }
    }

    /* silently accept — resets/clock writes need no modelling */
}

static const MemoryRegionOps apb_stub_ops = {
    .read  = apb_stub_read,
    .write = apb_stub_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* --------------------------------------------------------------------------
 * SIO inter-core FIFO helpers
 * -------------------------------------------------------------------------- */
static bool sio_fifo_push(SioFifoQ *q, uint32_t val)
{
    if (q->count >= SIO_FIFO_DEPTH) {
        q->wof = true;
        return false;
    }
    q->buf[q->wptr] = val;
    q->wptr = (q->wptr + 1) % SIO_FIFO_DEPTH;
    q->count++;
    return true;
}

static bool sio_fifo_pop(SioFifoQ *q, uint32_t *val)
{
    if (q->count == 0) {
        q->roe = true;
        return false;
    }
    *val = q->buf[q->rptr];
    q->rptr = (q->rptr + 1) % SIO_FIFO_DEPTH;
    q->count--;
    return true;
}

/* --------------------------------------------------------------------------
 * CPU1 launch via async work item (runs on CPU1's thread with BQL held)
 *
 * The RP2350 bootrom ordinarily processes the 6-word {0,0,1,vtor,sp,entry}
 * FIFO handshake and jumps CPU1 to the given entry point.  Because CPU1's
 * bootrom hits a lockup before reaching that loop we simulate the whole
 * handshake here instead.
 * -------------------------------------------------------------------------- */
typedef struct {
    RP2350MachineState *s;
    uint32_t vtor, sp, entry;
} Cpu1LaunchData;

static void cpu1_do_launch(CPUState *cs1, run_on_cpu_data data)
{
    Cpu1LaunchData *ld = data.host_ptr;
    ARMCPU *cpu1 = ARM_CPU(cs1);
    CPUARMState *env = &cpu1->env;

    /* Zero the reset-able portion of CPU state for a clean launch */
    memset(env, 0, offsetof(CPUARMState, end_reset_fields));

    /* Set up VTOR, stack pointer, and entry point */
    env->v7m.vecbase[M_REG_S]  = ld->vtor & 0xFFFFFF80u;
    env->v7m.vecbase[M_REG_NS] = ld->vtor & 0xFFFFFF80u;
    env->regs[13]  = ld->sp;
    env->regs[15]  = ld->entry & ~1u;
    env->thumb     = ld->entry & 1u;
    /* Thread mode, SPSEL=0 (MSP), nPRIV=0 */
    env->v7m.control[M_REG_S]  = 0;
    env->v7m.control[M_REG_NS] = 0;
    env->v7m.exception = 0;

    /* Un-halt */
    cpu1->power_state   = PSCI_ON;
    env->event_register = false;
    cs1->halted         = 0;
    cs1->exception_index = -1;
    cpu_reset_interrupt(cs1, CPU_INTERRUPT_HARD | CPU_INTERRUPT_RESET);
    qemu_cpu_kick(cs1);

    g_free(ld);
}

/* --------------------------------------------------------------------------
 * SIO (Single-cycle IO)
 *
 * CPUID (0x000)  — returns the current CPU's index (0 or 1).
 * FIFO_ST (0x050) — per-CPU view of the inter-core FIFOs.
 * FIFO_WR (0x054) — push to own TX FIFO (= other CPU's RX).
 * FIFO_RD (0x058) — pop from own RX FIFO (= other CPU's TX).
 * Spinlocks (0x100–0x17C) — reading claims; writing releases.
 *
 * FIFO layout:
 *   sio_fifo[0]: CPU0 → CPU1   (CPU0 writes here, CPU1 reads here)
 *   sio_fifo[1]: CPU1 → CPU0   (CPU1 writes here, CPU0 reads here)
 *
 * The 6-word launch handshake {0, 0, 1, vtor, sp, entry} is handled
 * entirely in the emulator: each word written by CPU0 to FIFO_WR is
 * immediately echoed back in sio_fifo[1] (CPU0's RX), simulating what
 * the real RP2350 bootrom running on CPU1 would do.
 * -------------------------------------------------------------------------- */
static void sio_handle_cpu1_launch_word(RP2350MachineState *s, uint32_t word)
{
    /* Expected sequence: {0, 0, 1, vtor, sp, entry} */
    static const uint32_t expected_prefix[3] = {0, 0, 1};

    qemu_log("sio_launch: step=%d word=0x%08x\n", s->cpu1_launch_step, word);

    if (s->cpu1_launch_step < 3) {
        if (word == expected_prefix[s->cpu1_launch_step]) {
            s->cpu1_launch_step++;
        } else {
            /* Mismatch: restart handshake */
            s->cpu1_launch_step = (word == 0) ? 1 : 0;
        }
    } else {
        switch (s->cpu1_launch_step) {
        case 3: s->cpu1_launch_vtor  = word; s->cpu1_launch_step++; break;
        case 4: s->cpu1_launch_sp    = word; s->cpu1_launch_step++; break;
        case 5:
            s->cpu1_launch_entry = word;
            s->cpu1_launch_step  = 0;
            /* All 6 words received: launch CPU1 */
            Cpu1LaunchData *ld = g_new(Cpu1LaunchData, 1);
            ld->s     = s;
            ld->vtor  = s->cpu1_launch_vtor;
            ld->sp    = s->cpu1_launch_sp;
            ld->entry = s->cpu1_launch_entry;
            async_run_on_cpu(CPU(s->cpu[1].cpu),
                             cpu1_do_launch, RUN_ON_CPU_HOST_PTR(ld));
            break;
        }
    }
}

static uint64_t sio_read(void *opaque, hwaddr addr, unsigned int size)
{
    RP2350MachineState *s = opaque;
    int cpu_idx = current_cpu ? current_cpu->cpu_index : 0;
    uint64_t val = 0;

    switch (addr) {
    case SIO_CPUID:
        val = cpu_idx;
        if (current_cpu) {
            ARMCPU *acpu = ARM_CPU(current_cpu);
            qemu_log("sio_cpuid: cpu=%d pc=0x%08x\n",
                     cpu_idx, acpu->env.regs[15]);
        }
        break;

    case SIO_GPIO_HI_IN:
        /* QSPI_CSN (bit 27) HIGH = flash CS idle = no BOOTSEL button press */
        val = SIO_GPIO_HI_IN_QSPI_CSN_BIT;
        break;

    case SIO_FIFO_ST: {
        /*
         * RX FIFO for this CPU is sio_fifo[1-cpu_idx]
         *   (CPU0 reads from sio_fifo[1], CPU1 reads from sio_fifo[0])
         * TX FIFO for this CPU is sio_fifo[cpu_idx]
         */
        SioFifoQ *rx = &s->sio_fifo[1 - cpu_idx];
        SioFifoQ *tx = &s->sio_fifo[cpu_idx];
        uint32_t st = 0;
        if (rx->count > 0)              st |= SIO_FIFO_ST_VLD;
        if (tx->count < SIO_FIFO_DEPTH) st |= SIO_FIFO_ST_RDY;
        if (tx->wof)                    st |= SIO_FIFO_ST_WOF;
        if (rx->roe)                    st |= SIO_FIFO_ST_ROE;
        val = st;
        break;
    }

    case SIO_FIFO_RD: {
        SioFifoQ *rx = &s->sio_fifo[1 - cpu_idx];
        uint32_t fval = 0;
        sio_fifo_pop(rx, &fval);
        val = fval;
        break;
    }

    default:
        if (addr >= SIO_SPINLOCK_BASE && addr < SIO_SPINLOCK_END) {
            val = 1u;  /* claim always succeeds */
        } else {
            qemu_log_mask(LOG_UNIMP,
                          "rp2350 SIO: unimplemented read @ 0x%" HWADDR_PRIx "\n",
                          addr);
            val = 0;
        }
        break;
    }
    qemu_log("sio_read:  @ 0x%" HWADDR_PRIx " = 0x%" PRIx64 "\n", addr, val);
    return val;
}

static void sio_write(void *opaque, hwaddr addr,
                      uint64_t data, unsigned int size)
{
    RP2350MachineState *s = opaque;
    int cpu_idx = current_cpu ? current_cpu->cpu_index : 0;

    qemu_log("sio_write: @ 0x%" HWADDR_PRIx " = 0x%" PRIx64 "\n", addr, data);

    switch (addr) {
    case SIO_FIFO_WR: {
        SioFifoQ *tx = &s->sio_fifo[cpu_idx];
        sio_fifo_push(tx, (uint32_t)data);

        if (cpu_idx == 0) {
            /*
             * CPU0 is writing to its TX FIFO (= CPU1's RX).
             * Intercept the multicore launch handshake and echo each word
             * back into CPU0's RX FIFO (sio_fifo[1]), simulating what the
             * RP2350 bootrom running on CPU1 would do.
             */
            sio_handle_cpu1_launch_word(s, (uint32_t)data);
            /* Echo the word back to CPU0's RX FIFO */
            sio_fifo_push(&s->sio_fifo[1], (uint32_t)data);
            /*
             * Wake CPU0 in case it's in WFE waiting for the echo.
             * Setting event_register is safe here: we're on CPU0's thread,
             * so the WFE (if any) hasn't been executed yet.
             */
            ARMCPU *cpu0 = ARM_CPU(s->cpu[0].cpu);
            cpu0->env.event_register = true;
        } else {
            /* CPU1 → CPU0: wake CPU0 so it can read the data */
            CPUState *cs0 = CPU(s->cpu[0].cpu);
            ARMCPU *cpu0 = ARM_CPU(cs0);
            cpu0->env.event_register = true;
            qemu_cpu_kick(cs0);
        }
        break;
    }

    case SIO_FIFO_ST:
        /* W1C: clear error bits */
        if (data & SIO_FIFO_ST_WOF) s->sio_fifo[cpu_idx].wof = false;
        if (data & SIO_FIFO_ST_ROE) s->sio_fifo[1 - cpu_idx].roe = false;
        break;

    default:
        if (addr >= SIO_SPINLOCK_BASE && addr < SIO_SPINLOCK_END) {
            break;  /* spinlock release — no state to model */
        }
        qemu_log_mask(LOG_UNIMP,
                      "rp2350 SIO: unimplemented write @ 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", addr, data);
        break;
    }
}

static const MemoryRegionOps sio_ops = {
    .read  = sio_read,
    .write = sio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* --------------------------------------------------------------------------
 * PSM (Power State Manager) at 0x40018000
 *
 * Only the FRCE_OFF register (offset 0x4) is implemented.
 * CPU1 (bit 24) in FRCE_OFF: 1 = held in reset, 0 = running.
 *
 * When PROC1 bit is SET  → halt CPU1, reset FIFOs and launch state.
 * When PROC1 bit is CLR  → CPU1 "starts": push 0 to CPU0's RX FIFO,
 *                           simulating the bootrom's readiness signal.
 * -------------------------------------------------------------------------- */
static uint64_t psm_read(void *opaque, hwaddr addr, unsigned int size)
{
    RP2350MachineState *s = opaque;
    uint32_t reg_off = addr & 0xFFF;
    if (reg_off == PSM_FRCE_OFF_OFF) {
        return s->psm_frce_off;
    }
    return 0xFFFFFFFFu;
}

static void psm_write(void *opaque, hwaddr addr,
                      uint64_t data, unsigned int size)
{
    RP2350MachineState *s = opaque;
    uint32_t alias   = (addr >> 12) & 3;
    uint32_t reg_off = addr & 0xFFF;

    if (reg_off != PSM_FRCE_OFF_OFF) {
        return;
    }

    uint32_t old = s->psm_frce_off;
    switch (alias) {
    case 0: s->psm_frce_off  = (uint32_t)data; break;
    case 1: s->psm_frce_off ^= (uint32_t)data; break;
    case 2: s->psm_frce_off |= (uint32_t)data; break;
    case 3: s->psm_frce_off &= ~(uint32_t)data; break;
    }

    bool was_off = (old & PSM_FRCE_OFF_PROC1_BIT) != 0;
    bool now_off = (s->psm_frce_off & PSM_FRCE_OFF_PROC1_BIT) != 0;

    if (!was_off && now_off) {
        /* CPU1 entering reset: halt it and reset launch state */
        CPUState *cs1 = CPU(s->cpu[1].cpu);
        ARMCPU *cpu1 = ARM_CPU(cs1);
        cpu1->power_state   = PSCI_OFF;
        cpu1->env.event_register = false;
        cs1->halted         = 1;
        cpu_reset_interrupt(cs1, CPU_INTERRUPT_HARD);
        /* Clear FIFOs and launch state machine */
        memset(&s->sio_fifo, 0, sizeof(s->sio_fifo));
        s->cpu1_launch_step = 0;
    } else if (was_off && !now_off) {
        /* CPU1 coming out of reset: signal readiness with 0 → CPU0's RX */
        sio_fifo_push(&s->sio_fifo[1], 0u);
        /* Wake CPU0 if it's in WFE waiting for this */
        CPUState *cs0 = CPU(s->cpu[0].cpu);
        ARMCPU *cpu0 = ARM_CPU(cs0);
        cpu0->env.event_register = true;
        qemu_cpu_kick(cs0);
    }
}

static const MemoryRegionOps psm_ops = {
    .read  = psm_read,
    .write = psm_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* --------------------------------------------------------------------------
 * TIMER0 / TIMER1 stubs (0x400B0000 / 0x400B8000, 16 KB each incl. aliases)
 *
 * Returns the QEMU virtual clock in microseconds for TIMEHR/TIMELR and their
 * raw equivalents.  All other registers read as 0.  This gives firmware a
 * monotonically increasing µs counter that advances in sync with simulated
 * time, so busy-wait loops and deadline comparisons work correctly.
 * -------------------------------------------------------------------------- */
static uint64_t timer_read(void *opaque, hwaddr addr, unsigned int size)
{
    uint32_t reg_off = addr & 0xFFF;   /* strip atomic-alias high bits */
    uint64_t now_us  = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);

    switch (reg_off) {
    case TIMER_TIMEHR:
    case TIMER_TIMERAWH:
        return (uint32_t)(now_us >> 32);
    case TIMER_TIMELR:
    case TIMER_TIMERAWL:
        return (uint32_t)now_us;
    default:
        return 0;
    }
}

static void timer_write(void *opaque, hwaddr addr,
                        uint64_t data, unsigned int size)
{
    /* silently accept alarm/control writes */
}

static const MemoryRegionOps timer_ops = {
    .read  = timer_read,
    .write = timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* --------------------------------------------------------------------------
 * IDAU (Implementation Defined Attribution Unit)
 *
 * RP2350 security attribution:
 *  - AHB-NS bus 0x50000000-0x5FFFFFFF (DMA, USB, PIO): exempt — accessible
 *    from both Secure and NS execution states.  The bootrom uses USB DPRAM
 *    (0x50100000) as the NS MSP target so it must be NS-reachable.
 *  - ARM PPB 0xE0000000-0xE00FFFFF: exempt per ARMv8-M spec.
 *  - ROM NSC veneer table 0x7E00-0x7FFF: Secure + Non-Secure-Callable.
 *    The bootrom places SG (Secure Gateway) instructions here so NS code can
 *    call back into Secure ROM.  SAU region 7 marks 0x4780-0x7FFF as NS, so
 *    the IDAU override (ns=false, nsc=true) is required to flip those 512 bytes
 *    to Secure+NSC so the SG instruction works correctly.
 *  - Flash XIP 0x10000000-0x1FFFFFFF and common aliases: exempt.
 *    The bootrom configures only SAU regions for SRAM and XIP-SRAM before
 *    entering the NS world; flash has no SAU region.  Per ARMv8-M the IDAU
 *    can only increase security (IDAU-NS cannot override SAU-Secure), so
 *    marking flash as merely NS would leave it inaccessible to NS code once
 *    the SAU is active (SAU default is Secure when no region matches).
 *    Marking it exempt makes it follow the caller's security state instead:
 *    NS callers see it as NS (can read/execute firmware), Secure callers see
 *    it as Secure.  This matches the real RP2350 IDAU behaviour.
 *  - BOOTRAM 0x400E0000-0x400E03FF: exempt (always-on RAM, used by bootrom).
 *  - Everything else: SAU decides (ROM NS range, SRAM, etc.).
 * -------------------------------------------------------------------------- */
static void rp2350_idau_check(IDAUInterface *ii, uint32_t address,
                              int *iregion, bool *exempt, bool *ns, bool *nsc)
{
    *iregion = IREGION_NOTVALID;
    *nsc     = false;
    *ns      = true;
    *exempt  = false;

    /* AHB-NS bus and ARM PPB: exempt from SAU security lookup */
    if ((address >= 0x50000000u && address < 0x60000000u) ||
        ((address & 0xfff00000u) == 0xe0000000u)) {
        *exempt = true;
        return;
    }

    /*
     * Flash XIP 0x10000000-0x1FFFFFFF (all four 16 MB chip-select windows)
     * and the no-translate alias at 0x1C000000.  Marked exempt so NS code
     * can execute and read firmware after the bootrom transfers control.
     */
    if ((address >= 0x10000000u && address < 0x20000000u) ||
        (address >= 0x1c000000u && address < 0x1d000000u)) {
        *exempt = true;
        return;
    }

    /*
     * BOOTRAM 0x400E0000-0x400E0FFF: always-on RAM used by bootrom for
     * callbacks and persistent state.  Mark the full 4 KB exempt so both
     * worlds see the same physical RAM, including the NS function table at
     * +0x800 that MicroPython polls before claiming the bootrom is ready.
     */
    if (address >= 0x400e0000u && address < 0x400e1000u) {
        *exempt = true;
        return;
    }

    /*
     * ROM NSC veneer table: IDAU region 2, Secure + NSC.
     * Covers 0x7E00-0x7FFF (entire upper 512 bytes of ROM).  SAU region 7
     * marks 0x4780-0x7FFF as NS; the IDAU overrides it to Secure+NSC so that
     * SG instructions work and TT on any address in this range (e.g. 0x7FE1
     * tested by the bootrom SAU verification loop) returns IDAU region 2.
     */
    if (address >= 0x7e00u && address < 0x8000u) {
        *iregion = 2;
        *ns  = false;
        *nsc = true;
        return;
    }

    /*
     * ROM Non-Secure range: 0x0000-0x7DFF.
     * The RP2350 bootrom configures the SAU to mark most of the ROM as NS
     * so that Non-Secure code can call helpers directly without going through
     * the NSC gateway.  In QEMU the SAU ends up disabled (the bootrom's SAU
     * init sequence does not complete cleanly), which makes all addresses
     * Secure by default and causes INVEP on direct NS->ROM branches.
     * Mark this range exempt so it inherits the caller's security state:
     * NS callers see it as NS (no INVEP), Secure callers see it as Secure.
     */
    if (address < 0x7e00u) {
        *exempt = true;
        return;
    }
}

/* --------------------------------------------------------------------------
 * SAU reset hook: runs after arm_cpu_reset() clears sau.ctrl to 0.
 * Set ALLNS=1 so all memory is Non-Secure by default; the IDAU then
 * overrides 0x7E00-0x7FFF to NSC (Secure Non-Callable gateway region).
 * -------------------------------------------------------------------------- */
static void rp2350_sau_reset(void *opaque)
{
    ARMCPU *cpu = opaque;
    cpu->env.sau.ctrl = 0x2; /* ENABLE=0, ALLNS=1 */
    /* The RP2350 bootrom does not update VTOR before handing off to the
     * firmware in our emulation.  Pre-initialize it to the XIP flash base
     * so that runtime_init_install_ram_vector_table copies from the
     * firmware's vector table (at 0x10000000) rather than the bootrom's
     * table (at 0x00000000). */
    cpu->env.v7m.vecbase[M_REG_S]  = RP2350_FLASH_BASE;
    cpu->env.v7m.vecbase[M_REG_NS] = RP2350_FLASH_BASE;
}

/* ==========================================================================
 * Minimal USB CDC stub (0x50110000, 16 KB with atomic aliases)
 *
 * Drives tinyUSB through USB enumeration via a timer-based state machine:
 *   BUS_RESET → SET_ADDRESS(1) → SET_CONFIGURATION(1) →
 *   CDC SET_CONTROL_LINE_STATE(DTR=1)
 * After this, tud_cdc_connected() returns true and the REPL starts.
 * CDC TX data is forwarded from USB DPRAM to the serial backend (stdio).
 * ==========================================================================*/
#define RP2350_USB_BASE     0x50110000u
#define RP2350_USB_SIZE     0x00004000u   /* 16 KB (normal + 3 atomic aliases) */

/* USB register offsets from 0x50110000 */
#define USB_REG_MAIN_CTRL   0x040u   /* bit 0 = CONTROLLER_EN */
#define USB_REG_SIE_CTRL    0x04Cu
#define USB_REG_SIE_STATUS  0x050u   /* RW1C: bit19=BUS_RESET,17=SETUP_REC,16=CONNECTED */
#define USB_REG_BUFF_STATUS 0x058u   /* RW1C: bit N = EP(N/2) IN (even) or OUT (odd)   */
#define USB_REG_EP_ABORT    0x060u
#define USB_REG_EP_ABTDONE  0x064u
#define USB_REG_INTR        0x08Cu   /* RO: bit16=SETUP_REQ,13=CONN_DIS,12=BUS_RESET,4=BUFF */
#define USB_REG_INTE        0x090u   /* RW: interrupt enable */
#define USB_REG_INTF        0x094u   /* RW: interrupt force  */
#define USB_REG_INTS        0x098u   /* RO: INTR & INTE      */

/* SIE_STATUS W1C bits */
#define USB_SIE_CONNECTED   (1u << 16)
#define USB_SIE_SETUP_REC   (1u << 17)
#define USB_SIE_BUS_RESET   (1u << 19)

/* INTR bits */
#define USB_INTR_BUFF       (1u << 4)
#define USB_INTR_BUS_RESET  (1u << 12)
#define USB_INTR_CONN_DIS   (1u << 13)
#define USB_INTR_SETUP_REQ  (1u << 16)

/* DPRAM offsets (from 0x50100000) */
#define DPRAM_SETUP_PKT     0x000u   /* 8 bytes: SETUP packet data              */
/*
 * RP2040/RP2350 USB DPRAM layout (base = 0x50100000):
 *   0x000 - 0x007: setup_packet[8]
 *   0x008 - 0x07F: ep_ctrl[EP1..EP15][in/out]   <-- endpoint control regs
 *   0x080 - 0x0FF: ep_buf_ctrl[EP0..EP15][in/out] <-- buffer control regs
 *   0x100 +      : data buffers
 *
 * ep_ctrl[N-1].in  = DPRAM + 0x008 + (N-1)*8 + 0  (EP1..EP15)
 * ep_buf_ctrl[N].in = DPRAM + 0x080 + N*8 + 0      (EP0..EP15)
 */
#define DPRAM_EP1_CTRL_IN   0x008u   /* EP1 IN  endpoint control                */
#define DPRAM_EP2_CTRL_IN   0x010u   /* EP2 IN  endpoint control                */
#define DPRAM_EP0_BUF_IN    0x080u   /* EP0 IN  buffer control                  */
#define DPRAM_EP1_BUF_IN    0x088u   /* EP1 IN  buffer control                  */
#define DPRAM_EP2_BUF_IN    0x090u   /* EP2 IN  buffer control                  */
#define DPRAM_EP2_BUF_OUT   0x094u   /* EP2 OUT buffer control                  */

/* USB buffer control bits */
#define BUF_CTRL_FULL       (1u << 15)
#define BUF_CTRL_AVAIL      (1u << 10)
#define BUF_CTRL_LEN_MASK   0x3FFu

/*
 * USB CDC enumeration state machine.
 *
 * Each control transfer has a STATUS phase (EP0 IN zero-length packet) that
 * tinyUSB needs BUFF_STATUS.EP0_IN to complete before advancing its internal
 * state.  We fake this from TIMER context (never from inside a write handler
 * or ISR) to avoid re-entrant ISR processing.
 *
 * State flow:
 *   IDLE → ENABLED → BUS_RESET → SET_ADDR →
 *   ADDR_STATUS → SET_CFG → CFG_STATUS → CDC_LST → LST_STATUS → RUNNING
 *
 * Transitions:
 *   ENABLED:     timer → inject BUS_RESET
 *   BUS_RESET:   SIE_STATUS.BUS_RESET W1C → timer 50µs → inject SET_ADDRESS
 *   SET_ADDR:    SETUP_REC W1C → state=ADDR_STATUS, timer 50µs
 *   ADDR_STATUS: timer → inject BUFF_STATUS.EP0_IN; fallback retry 200µs
 *                BUFF_STATUS.EP0_IN W1C → state=SET_CFG, timer 200µs
 *   SET_CFG:     timer → inject SET_CONFIGURATION SETUP
 *                SETUP_REC W1C → state=CFG_STATUS, timer 50µs
 *   CFG_STATUS:  timer → inject BUFF_STATUS.EP0_IN; fallback retry 200µs
 *                BUFF_STATUS.EP0_IN W1C → state=CDC_LST, timer 200µs
 *   CDC_LST:     timer → inject CDC LINE_STATE SETUP
 *                SETUP_REC W1C → state=LST_STATUS, timer 50µs
 *   LST_STATUS:  timer → inject BUFF_STATUS.EP0_IN; fallback retry 200µs
 *                BUFF_STATUS.EP0_IN W1C → state=RUNNING, timer 1000µs
 *   RUNNING:     timer → poll EP2 IN for CDC TX data
 */
enum {
    USB_ST_IDLE = 0,
    USB_ST_ENABLED,
    USB_ST_BUS_RESET,    /* BUS_RESET injected; wait for BUS_RESET W1C */
    USB_ST_SET_ADDR,     /* SET_ADDRESS SETUP injected; wait for SETUP_REC W1C */
    USB_ST_ADDR_STATUS,  /* waiting: inject BUFF_STATUS.EP0_IN from timer */
    USB_ST_SET_CFG,      /* inject SET_CONFIGURATION from timer */
    USB_ST_CFG_STATUS,   /* waiting: inject BUFF_STATUS.EP0_IN from timer */
    USB_ST_CDC_LST,      /* inject CDC LINE_STATE from timer */
    USB_ST_LST_STATUS,   /* waiting: inject BUFF_STATUS.EP0_IN from timer */
    USB_ST_RUNNING,      /* CDC connected; poll EP2 IN */
};

static inline void usb_update_irq(RP2350MachineState *s)
{
    uint32_t ints = (s->usb_intr | (s->usb_buff_status ? USB_INTR_BUFF : 0))
                    & s->usb_inte;
    int level = ints ? 1 : 0;
    qemu_log("usb_irq: level=%d ints=0x%x\n", level, ints);
    qemu_set_irq(s->usb_irq, level);
}

/*
 * DPRAM is mapped as a 16KB window (4 alias banks of 4KB each):
 *   bank 0 (+0x0000): normal  read/write
 *   bank 1 (+0x1000): XOR     (read normal; write XORs)
 *   bank 2 (+0x2000): SET     (read normal; write ORs)
 *   bank 3 (+0x3000): CLR     (read normal; write ANDs ~val)
 * addr here already includes the bank offset (bits [13:12]).
 */
static uint64_t usb_dpram_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350MachineState *s = opaque;
    hwaddr off = addr & 0x0FFFu;  /* strip alias bank bits */
    uint64_t val = 0;
    if (off + size <= sizeof(s->usb_dpram_buf))
        memcpy(&val, s->usb_dpram_buf + off, size);
    /* Log reads from setup packet + EP0 buf_ctrl area (important for USB debug) */
    if (((off < 0x10) || (off >= 0x07c && off <= 0x090)) && size >= 2)
        qemu_log("dpram_rd[0x%03x]=%0*llx (sz=%d)\n",
                 (uint32_t)off, size * 2, (unsigned long long)val, size);
    return val;
}

static void usb_dpram_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    RP2350MachineState *s = opaque;
    uint32_t alias = (uint32_t)(addr >> 12) & 3u;  /* 0=normal,1=XOR,2=SET,3=CLR */
    hwaddr off = addr & 0x0FFFu;
    if (off + size > sizeof(s->usb_dpram_buf)) return;

    /* Apply atomic alias operation */
    if (alias == 0) {
        memcpy(s->usb_dpram_buf + off, &val, size);
    } else {
        uint64_t cur = 0;
        memcpy(&cur, s->usb_dpram_buf + off, size);
        uint64_t newval = (alias == 1) ? (cur ^ val) :
                          (alias == 2) ? (cur | val) :
                                         (cur & ~val);
        memcpy(s->usb_dpram_buf + off, &newval, size);
        val = newval;  /* use final value for logging/detection below */
    }
    addr = off;  /* use unaliased offset for logging/detection */
    /* Log writes to EP ctrl area (0x008-0x07F) and buf ctrl area (0x080-0x0FF) */
    if (addr >= 0x008 && addr < 0x100) {
        uint32_t ep_n = ((uint32_t)addr - 0x008) / 8 + 1;
        const char *dir = (((uint32_t)addr - 0x008) % 8 < 4) ? "IN" : "OUT";
        qemu_log("dpram_wr[0x%03x]=0x%0*llx EP%u_%s %s\n",
                 (uint32_t)addr, size * 2, (unsigned long long)val,
                 ep_n, dir,
                 (addr < 0x080) ? "ctrl" : "buf_ctrl");
    }

    /*
     * Detect tinyUSB setting up EP0 IN for the STATUS phase.
     *
     * After processing a SETUP request, tinyUSB calls hw_endpoint_xfer_start()
     * for EP0 IN which (a) sets ep->active=true and (b) writes a non-zero value
     * to DPRAM[0x080] (EP0 IN buf_ctrl).  This write is our reliable signal
     * that ep->active is now true and it is safe to inject BUFF_STATUS.EP0_IN.
     *
     * Inject 50µs after the write (giving the AVAIL bit write and any ISB time).
     */
    if (addr == 0x080)
        qemu_log("dpram_wr[0x080]=0x%llx (EP0_IN buf_ctrl) state=%d buff=0x%x\n",
                 (unsigned long long)val, s->usb_state, s->usb_buff_status);
    if (addr == 0x080 && val != 0 && !(s->usb_buff_status & 0x1u) &&
        (s->usb_state == USB_ST_ADDR_STATUS ||
         s->usb_state == USB_ST_CFG_STATUS  ||
         s->usb_state == USB_ST_LST_STATUS)) {
        qemu_log("dpram_wr: EP0_IN buf_ctrl=0x%llx → schedule BUFF_STATUS state=%d\n",
                 (unsigned long long)val, s->usb_state);
        int64_t now = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
        timer_mod(s->usb_timer, now + 50);
    }
}

static const MemoryRegionOps usb_dpram_ops = {
    .read  = usb_dpram_read,
    .write = usb_dpram_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void usb_inject_setup(RP2350MachineState *s, const uint8_t pkt[8])
{
    memcpy(s->usb_dpram_buf + DPRAM_SETUP_PKT, pkt, 8);
    s->usb_sie_status |= USB_SIE_SETUP_REC;
    s->usb_intr       |= USB_INTR_SETUP_REQ;
    qemu_log("usb_inject_setup: bRequest=0x%02x state=%d intr=0x%x inte=0x%x\n",
             pkt[1], s->usb_state, s->usb_intr, s->usb_inte);
    usb_update_irq(s);
}

/* Minimal USB SETUP packets for CDC enumeration */
static const uint8_t usb_pkt_set_addr[]  = {0x00,0x05,0x01,0x00,0x00,0x00,0x00,0x00};
static const uint8_t usb_pkt_set_cfg[]   = {0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00};
static const uint8_t usb_pkt_cdc_lst[]   = {0x21,0x22,0x03,0x00,0x00,0x00,0x00,0x00};

static void usb_timer_cb(void *opaque)
{
    RP2350MachineState *s = opaque;
    uint8_t *dpram = s->usb_dpram_buf;
    uint64_t now = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);

    qemu_log("usb_timer: state=%d intr=0x%x inte=0x%x buff=0x%x\n",
             s->usb_state, s->usb_intr, s->usb_inte, s->usb_buff_status);

    switch (s->usb_state) {

    case USB_ST_ENABLED:
        s->usb_sie_status |= USB_SIE_BUS_RESET | USB_SIE_CONNECTED;
        s->usb_intr       |= USB_INTR_BUS_RESET;
        s->usb_state = USB_ST_BUS_RESET;
        usb_update_irq(s);
        break;

    case USB_ST_BUS_RESET:
        usb_inject_setup(s, usb_pkt_set_addr);
        s->usb_state = USB_ST_SET_ADDR;
        break;

    /*
     * STATUS-phase states.
     *
     * For ADDR_STATUS: inject BUFF_STATUS.EP0_IN through the ISR (works
     * because the USB ISR is not blocked by HardFault at this stage).
     *
     * For CFG_STATUS and LST_STATUS: tud_task's tud_control_status() call
     * panics via hard_assert() inside _hw_endpoint_buffer_control_update32
     * (reads CPUID 3× then BKPT), causing HardFault that blocks the USB ISR.
     * dcd_edpt0_status_complete() for SET_CONFIGURATION and CDC_LINE_STATE
     * does nothing at the DCD level, so we can safely skip the BUFF_STATUS
     * ISR and advance the state directly once DPRAM[0x080] FULL bit is set
     * (ep->active=true).  The next SETUP will call reset_ep0() which aborts
     * any pending EP0 IN transfer automatically.
     */
    case USB_ST_ADDR_STATUS: {
        uint32_t ep0_in_bc = 0;
        memcpy(&ep0_in_bc, s->usb_dpram_buf + 0x080, 4);
        bool ep0_in_ready = (ep0_in_bc & 0x8000u) != 0;  /* FULL bit = ep->active */
        if (s->usb_buff_status & 0x1u) {
            qemu_log("usb_timer: BUFF EP0_IN still pending state=%d\n", s->usb_state);
            usb_update_irq(s);
            timer_mod(s->usb_timer, now + 200);
        } else if (ep0_in_ready) {
            s->usb_buff_status |= 0x1u;
            qemu_log("usb_timer: inject BUFF EP0_IN state=%d (ep0bc=0x%x)\n",
                     s->usb_state, ep0_in_bc);
            usb_update_irq(s);
            timer_mod(s->usb_timer, now + 200);
        } else {
            qemu_log("usb_timer: ep0_in not ready (bc=0x%x) state=%d, retry\n",
                     ep0_in_bc, s->usb_state);
            timer_mod(s->usb_timer, now + 5000);
        }
        break;
    }

    case USB_ST_CFG_STATUS:
    case USB_ST_LST_STATUS: {
        /*
         * HardFault blocks the USB ISR, so we can't use BUFF_STATUS injection
         * here. Instead, wait for DPRAM[0x080] FULL bit (ep->active=true) and
         * then advance the state directly, simulating STATUS completion without
         * the ISR.  dcd_edpt0_status_complete does nothing for these requests
         * so skipping it is safe.
         */
        uint32_t ep0_in_bc = 0;
        memcpy(&ep0_in_bc, s->usb_dpram_buf + 0x080, 4);
        if (ep0_in_bc & 0x8000u) {
            /* ep->active=true: STATUS phase is set up. Advance directly. */
            qemu_log("usb_timer: direct advance state=%d (ep0bc=0x%x) skip BUFF ISR\n",
                     s->usb_state, ep0_in_bc);
            /* Clear buf_ctrl so reset_ep0 in next SETUP doesn't see stale FULL */
            uint32_t zero = 0;
            memcpy(s->usb_dpram_buf + 0x080, &zero, 4);
            /* Advance state machine as if BUFF_STATUS W1C happened */
            if (s->usb_state == USB_ST_CFG_STATUS) {
                s->usb_state = USB_ST_CDC_LST;
                timer_mod(s->usb_timer, now + 5000);
            } else {
                s->usb_state = USB_ST_RUNNING;
                timer_mod(s->usb_timer, now + 1000);
            }
        } else {
            qemu_log("usb_timer: ep0_in not ready (bc=0x%x) state=%d, retry\n",
                     ep0_in_bc, s->usb_state);
            timer_mod(s->usb_timer, now + 5000);
        }
        break;
    }

    case USB_ST_SET_CFG:
        if (s->usb_intr & USB_INTR_SETUP_REQ) {
            qemu_log("usb_timer: re-assert SETUP_REQ state=%d\n", s->usb_state);
            usb_update_irq(s);
        } else {
            usb_inject_setup(s, usb_pkt_set_cfg);
        }
        timer_mod(s->usb_timer, now + 500);
        break;

    case USB_ST_CDC_LST:
        if (s->usb_intr & USB_INTR_SETUP_REQ) {
            qemu_log("usb_timer: re-assert SETUP_REQ state=%d\n", s->usb_state);
            usb_update_irq(s);
        } else {
            usb_inject_setup(s, usb_pkt_cdc_lst);
        }
        timer_mod(s->usb_timer, now + 500);
        break;

    case USB_ST_RUNNING: {
        /*
         * Scan DPRAM buffer control region 0x080-0x0B0 (EP0-EP5 IN/OUT).
         * Also dump EP ctrl regs 0x100-0x110 (EP1-EP2 IN/OUT) and
         * the first 32 bytes of data buffers at 0x180.
         */
        static int running_count = 0;
        if (running_count++ < 5) {
            uint32_t v;
            qemu_log("usb_dpram_dump (call %d):\n", running_count);
            /* ep_ctrl[EP1..EP7]: 0x008 - 0x040 */
            for (uint32_t off = 0x008; off < 0x048; off += 4) {
                memcpy(&v, dpram + off, 4);
                if (v) qemu_log("  dpram[0x%03x]=0x%08x (ep_ctrl EP%d %s)\n",
                                off, v,
                                (off - 0x008) / 8 + 1,
                                ((off - 0x008) % 8) ? "OUT" : "IN");
            }
            /* ep_buf_ctrl[EP0..EP5]: 0x080 - 0x0B0 */
            for (uint32_t off = 0x080; off < 0x0B0; off += 4) {
                memcpy(&v, dpram + off, 4);
                if (v) qemu_log("  dpram[0x%03x]=0x%08x (ep_buf_ctrl EP%d %s)\n",
                                off, v,
                                (off - 0x080) / 8,
                                ((off - 0x080) % 8) ? "OUT" : "IN");
            }
            /* Data buffers start at 0x100 */
            for (uint32_t off = 0x100; off < 0x140; off += 4) {
                memcpy(&v, dpram + off, 4);
                if (v) qemu_log("  dpram[0x%03x]=0x%08x (data buf)\n", off, v);
            }
        }

        /* Check EP2 IN (bulk CDC TX): drain if FULL */
        uint32_t bc;
        memcpy(&bc, dpram + DPRAM_EP2_BUF_IN, 4);
        if (bc & BUF_CTRL_FULL) {
            uint16_t len = bc & BUF_CTRL_LEN_MASK;
            if (len > 0 && len <= 64) {
                uint32_t ep2_ctrl;
                memcpy(&ep2_ctrl, dpram + DPRAM_EP2_CTRL_IN, 4);
                uint32_t buf_addr = ep2_ctrl & 0x0000FFFFu;
                qemu_log("usb_ep2_in: FULL len=%d ctrl=0x%x buf_addr=0x%x\n",
                         len, ep2_ctrl, buf_addr);
                if (buf_addr >= 0x100 && buf_addr < 0x1000) {
                    Chardev *chr = serial_hd(0);
                    if (chr) {
                        qemu_chr_write_all(chr, dpram + buf_addr, len);
                    }
                }
            }
            bc &= ~(BUF_CTRL_FULL | BUF_CTRL_AVAIL);
            memcpy(dpram + DPRAM_EP2_BUF_IN, &bc, 4);
            s->usb_buff_status |= (1u << 4);  /* EP2 IN done */
            usb_update_irq(s);
        }
        timer_mod(s->usb_timer,
                  qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) + 1000);
        break;
    }

    default:
        break;
    }
}

static uint64_t usb_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350MachineState *s = opaque;
    uint32_t reg = addr & 0xFFFu;
    uint32_t val = 0;

    switch (reg) {
    case USB_REG_MAIN_CTRL:   val = s->usb_main_ctrl; break;
    case USB_REG_SIE_CTRL:    val = 0x00008000u; break;  /* PULLDOWN_EN reset */
    case USB_REG_SIE_STATUS:
        val = s->usb_sie_status;
        if (s->usb_state >= USB_ST_BUS_RESET) val |= USB_SIE_CONNECTED;
        qemu_log("usb_sie_read: val=0x%x state=%d\n", val, s->usb_state);
        break;
    case USB_REG_BUFF_STATUS:
        val = s->usb_buff_status;
        qemu_log("usb_buff_read: buff=0x%x state=%d\n", val, s->usb_state);
        break;
    case USB_REG_INTR:
        val = s->usb_intr;
        qemu_log("usb_intr_read: intr=0x%x state=%d\n", val, s->usb_state);
        break;
    case USB_REG_INTE:        val = s->usb_inte; break;
    case USB_REG_INTS:
        val = (s->usb_intr | (s->usb_buff_status ? USB_INTR_BUFF : 0))
              & s->usb_inte;
        qemu_log("usb_ints_read: ints=0x%x state=%d intr=0x%x\n",
                 val, s->usb_state, s->usb_intr);
        break;
    default:                  val = 0; break;
    }
    return val;
}

static void usb_reg_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    RP2350MachineState *s = opaque;
    uint32_t alias = (addr >> 12) & 3;  /* atomic aliases: 0=RW,1=XOR,2=SET,3=CLR */
    uint32_t reg   = addr & 0xFFFu;

    /* SIE_STATUS and BUFF_STATUS are write-1-to-clear */
    if (reg == USB_REG_SIE_STATUS) {
        qemu_log("usb_sie_status W1C: data=0x%x sie_was=0x%x intr_was=0x%x state=%d\n",
                 (uint32_t)data, s->usb_sie_status, s->usb_intr, s->usb_state);
        s->usb_sie_status &= ~(uint32_t)data;
        /* Mirror clears to INTR */
        if (data & USB_SIE_BUS_RESET)  s->usb_intr &= ~USB_INTR_BUS_RESET;
        if (data & USB_SIE_SETUP_REC)  s->usb_intr &= ~USB_INTR_SETUP_REQ;
        if (data & USB_SIE_CONNECTED)  s->usb_intr &= ~USB_INTR_CONN_DIS;

        /*
         * Advance the USB CDC enumeration state machine.
         *
         * BUS_RESET cleared → schedule inject SET_ADDRESS (small timer).
         * SETUP_REC cleared → schedule inject of next SETUP via timer.
         */
        uint64_t now = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
        if ((data & USB_SIE_BUS_RESET) && s->usb_state == USB_ST_BUS_RESET) {
            /* BUS_RESET acked → inject SET_ADDRESS after tiny delay */
            timer_mod(s->usb_timer, now + 50);
        }
        if (data & USB_SIE_SETUP_REC) {
            /*
             * SETUP_REC cleared: tinyUSB has processed the SETUP packet and
             * started its STATUS stage.  Schedule timer to inject BUFF_STATUS.EP0_IN
             * (the STATUS ACK) from outside ISR context, so tinyUSB can complete
             * the control transfer.
             */
            if (s->usb_state == USB_ST_SET_ADDR) {
                s->usb_state = USB_ST_ADDR_STATUS;
                /* Cancel any leftover timer from SET_ADDR re-assert path.
                 * BUFF_STATUS.EP0_IN will be injected by usb_dpram_write when
                 * tinyUSB writes EP0 IN buf_ctrl (ep->active = true). */
                timer_del(s->usb_timer);
            } else if (s->usb_state == USB_ST_SET_CFG) {
                s->usb_state = USB_ST_CFG_STATUS;
                timer_del(s->usb_timer);  /* cancel SET_CFG re-assert timer */
            } else if (s->usb_state == USB_ST_CDC_LST) {
                s->usb_state = USB_ST_LST_STATUS;
                timer_del(s->usb_timer);  /* cancel CDC_LST re-assert timer */
            }
            /* For CFG_STATUS and LST_STATUS: start polling ep0_in buf_ctrl
             * immediately so we inject BUFF_STATUS as soon as ep->active=true */
            if (s->usb_state == USB_ST_CFG_STATUS ||
                s->usb_state == USB_ST_LST_STATUS) {
                timer_mod(s->usb_timer, now + 5000);  /* first poll in 5ms */
            }
        }

        usb_update_irq(s);
        return;
    }
    if (reg == USB_REG_BUFF_STATUS) {
        uint64_t now = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
        qemu_log("usb_buff_status W1C: data=0x%x was=0x%x state=%d\n",
                 (uint32_t)data, s->usb_buff_status, s->usb_state);
        s->usb_buff_status &= ~(uint32_t)data;
        if (!s->usb_buff_status) s->usb_intr &= ~USB_INTR_BUFF;
        usb_update_irq(s);

        /* EP0_IN STATUS ACK cleared by tinyUSB → advance to next SETUP injection */
        if (data & 0x1u) {
            if (s->usb_state == USB_ST_ADDR_STATUS) {
                s->usb_state = USB_ST_SET_CFG;
                timer_mod(s->usb_timer, now + 5000);  /* 5ms: let tud_task finish SET_ADDRESS */
            } else if (s->usb_state == USB_ST_CFG_STATUS) {
                s->usb_state = USB_ST_CDC_LST;
                timer_mod(s->usb_timer, now + 5000);  /* 5ms: let tud_task finish SET_CONFIG */
            } else if (s->usb_state == USB_ST_LST_STATUS) {
                s->usb_state = USB_ST_RUNNING;
                timer_mod(s->usb_timer, now + 1000);
            }
        }
        return;
    }

    /* General register write with atomic-alias support */
    uint32_t *tgt = NULL;
    switch (reg) {
    case USB_REG_MAIN_CTRL: tgt = &s->usb_main_ctrl; break;
    case USB_REG_INTE:
        tgt = &s->usb_inte;
        qemu_log("usb_inte_write: alias=%d data=0x%x old_inte=0x%x state=%d\n",
                 alias, (uint32_t)data, s->usb_inte, s->usb_state);
        break;
    default:
        qemu_log("usb_reg_write: UNHANDLED reg=0x%03x data=0x%08x alias=%d state=%d\n",
                 reg, (uint32_t)data, alias, s->usb_state);
        return;
    }
    switch (alias) {
    case 0: *tgt  = (uint32_t)data; break;
    case 1: *tgt ^= (uint32_t)data; break;
    case 2: *tgt |= (uint32_t)data; break;
    case 3: *tgt &= ~(uint32_t)data; break;
    }

    /* Start USB when CONTROLLER_EN (bit 0) goes high */
    if (reg == USB_REG_MAIN_CTRL && (s->usb_main_ctrl & 1u)
        && s->usb_state == USB_ST_IDLE) {
        s->usb_state = USB_ST_ENABLED;
        timer_mod(s->usb_timer,
                  qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) + 50);
    }

    usb_update_irq(s);
}

static const MemoryRegionOps usb_reg_ops = {
    .read  = usb_reg_read,
    .write = usb_reg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* --------------------------------------------------------------------------
 * Flash persistence
 *
 * On QEMU exit, write the entire flash RAM back to the backing file so that
 * littlefs data, MicroPython frozen modules, etc. survive across runs.
 * -------------------------------------------------------------------------- */
static void rp2350_flash_save(Notifier *notifier, void *data)
{
    RP2350MachineState *s =
        container_of(notifier, RP2350MachineState, flash_save_notifier);
    const char *path = s->flash_file;
    FILE *f;
    void *ptr;

    if (!path) {
        return;
    }
    ptr = memory_region_get_ram_ptr(&s->flash);
    f = fopen(path, "wb");
    if (!f) {
        warn_report("rp2350: could not save flash to '%s': %s",
                    path, strerror(errno));
        return;
    }
    if (fwrite(ptr, 1, RP2350_FLASH_SIZE, f) != RP2350_FLASH_SIZE) {
        warn_report("rp2350: short write saving flash to '%s'", path);
    }
    fclose(f);
}

/* --------------------------------------------------------------------------
 * UF2 loader
 *
 * Parses a UF2 file and copies each data block into the flash RAM region.
 * Blocks that target addresses outside [FLASH_BASE, FLASH_BASE+FLASH_SIZE)
 * are silently skipped (e.g. blocks targeting SRAM or OTP).
 *
 * Returns the number of flash blocks loaded, or 0 on failure.
 * -------------------------------------------------------------------------- */
static int rp2350_load_uf2(RP2350MachineState *s, const char *filename)
{
    FILE *f;
    uint8_t blk[UF2_BLOCK_SIZE];
    uint8_t *flash_ptr = memory_region_get_ram_ptr(&s->flash);
    int n = 0;

    f = fopen(filename, "rb");
    if (!f) {
        return 0;
    }

    while (fread(blk, 1, UF2_BLOCK_SIZE, f) == UF2_BLOCK_SIZE) {
        /* Read fields (little-endian) */
        uint32_t m0    = (uint32_t)blk[0]  | ((uint32_t)blk[1]  << 8)
                       | ((uint32_t)blk[2]  << 16) | ((uint32_t)blk[3]  << 24);
        uint32_t m1    = (uint32_t)blk[4]  | ((uint32_t)blk[5]  << 8)
                       | ((uint32_t)blk[6]  << 16) | ((uint32_t)blk[7]  << 24);
        uint32_t flags = (uint32_t)blk[8]  | ((uint32_t)blk[9]  << 8)
                       | ((uint32_t)blk[10] << 16) | ((uint32_t)blk[11] << 24);
        uint32_t addr  = (uint32_t)blk[12] | ((uint32_t)blk[13] << 8)
                       | ((uint32_t)blk[14] << 16) | ((uint32_t)blk[15] << 24);
        uint32_t psz   = (uint32_t)blk[16] | ((uint32_t)blk[17] << 8)
                       | ((uint32_t)blk[18] << 16) | ((uint32_t)blk[19] << 24);
        uint32_t mend  = (uint32_t)blk[508] | ((uint32_t)blk[509] << 8)
                       | ((uint32_t)blk[510] << 16) | ((uint32_t)blk[511] << 24);

        /* Validate magic */
        if (m0 != UF2_MAGIC0 || m1 != UF2_MAGIC1 || mend != UF2_MAGIC_END) {
            continue;
        }
        /* Skip non-data blocks */
        if (flags & (UF2_FLAG_NOFLASH | UF2_FLAG_FILE_CONT)) {
            continue;
        }
        /* Sanity-check payload size (data field is 476 bytes) */
        if (psz == 0 || psz > 476) {
            continue;
        }
        /* Only load blocks that fall inside XIP flash */
        if (addr < RP2350_FLASH_BASE
            || (uint64_t)addr + psz > (uint64_t)RP2350_FLASH_BASE + RP2350_FLASH_SIZE) {
            continue;
        }

        memcpy(flash_ptr + (addr - RP2350_FLASH_BASE), blk + 32, psz);
        n++;
    }

    fclose(f);
    return n;
}

/*
 * Probe the first 8 bytes of a file for UF2 magic numbers.
 * Returns true if the file looks like a UF2 image.
 */
static bool rp2350_is_uf2(const char *filename)
{
    FILE *f = fopen(filename, "rb");
    uint8_t hdr[8];
    bool ok = false;

    if (!f) {
        return false;
    }
    if (fread(hdr, 1, 8, f) == 8) {
        uint32_t m0 = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8)
                    | ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
        uint32_t m1 = (uint32_t)hdr[4] | ((uint32_t)hdr[5] << 8)
                    | ((uint32_t)hdr[6] << 16) | ((uint32_t)hdr[7] << 24);
        ok = (m0 == UF2_MAGIC0 && m1 == UF2_MAGIC1);
    }
    fclose(f);
    return ok;
}

/* --------------------------------------------------------------------------
 * Machine initialisation
 * -------------------------------------------------------------------------- */
static void rp2350_init(MachineState *machine)
{
    RP2350MachineState *s = RP2350_MACHINE(machine);
    MemoryRegion *sys_mem = get_system_memory();
    int i;

    /* Fixed-frequency system clock */
    s->sysclk = clock_new(OBJECT(machine), "sysclk");
    clock_set_hz(s->sysclk, RP2350_SYSCLK_HZ);

    /* --- Boot ROM at 0x00000000 (32 KB) ---
     * Supply the real bootrom via -bios <bootrom-combined.bin>.
     * Without -bios the ROM is readable but zeroed; the SDK still works
     * because we also set init-svtor = FLASH_BASE so the CPU boots from
     * flash directly and only accesses ROM when calling ROM helper functions.
     */
    memory_region_init_rom(&s->rom, NULL, "rp2350.rom",
                           RP2350_ROM_SIZE, &error_fatal);
    memory_region_add_subregion(sys_mem, RP2350_ROM_BASE, &s->rom);
    if (machine->firmware) {
        ssize_t sz = load_image_targphys(machine->firmware,
                                         RP2350_ROM_BASE, RP2350_ROM_SIZE,
                                         NULL);
        if (sz < 0) {
            error_report("rp2350: failed to load bootrom '%s'",
                         machine->firmware);
            exit(1);
        }
    }

    /* --- Flash (XIP, writable RAM) at 0x10000000 and aliases ---
     *
     * Flash is a writable RAM region so that MicroPython's littlefs and other
     * firmware-managed storage survive.  If -machine rp2350,flash=<path> is
     * given, the image is loaded at startup (preserving any existing filesystem
     * data at the end of flash) and written back on exit.  The -kernel binary
     * is then stamped into offset 0, exactly as picotool would do when flashing
     * a Pico without erasing first.
     */
    memory_region_init_ram(&s->flash, NULL, "rp2350.flash",
                           RP2350_FLASH_SIZE, &error_fatal);
    memory_region_add_subregion(sys_mem, RP2350_FLASH_BASE, &s->flash);

    if (s->flash_file) {
        /* Load existing flash image (non-fatal if the file doesn't exist yet) */
        ssize_t sz = load_image_targphys(s->flash_file,
                                         RP2350_FLASH_BASE, RP2350_FLASH_SIZE,
                                         NULL);
        if (sz < 0) {
            /* File not found — flash initialised to zero, will be created on exit */
            memset(memory_region_get_ram_ptr(&s->flash), 0xff, RP2350_FLASH_SIZE);
        }
        /* Register exit notifier to persist the flash image on shutdown */
        s->flash_save_notifier.notify = rp2350_flash_save;
        qemu_add_exit_notifier(&s->flash_save_notifier);
    }

    /* Map common aliases used by SDK/bootrom */
    for (i = 1; i < 4; i++) {
        MemoryRegion *alias = g_new(MemoryRegion, 1);
        char *name = g_strdup_printf("rp2350.flash.alias%d", i);
        memory_region_init_alias(alias, NULL, name, &s->flash, 0, RP2350_FLASH_SIZE);
        memory_region_add_subregion(sys_mem, RP2350_FLASH_BASE + (hwaddr)i * 0x01000000u, alias);
        g_free(name);
    }
    /* XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE at 0x1c000000 */
    {
        MemoryRegion *alias = g_new(MemoryRegion, 1);
        memory_region_init_alias(alias, NULL, "rp2350.flash.alias_notrans", 
                                 &s->flash, 0, RP2350_FLASH_SIZE);
        memory_region_add_subregion(sys_mem, 0x1c000000u, alias);
    }

    /* --- SRAM 520 KB at 0x20000000 --- */
    memory_region_init_ram(&s->sram, NULL, "rp2350.sram",
                           RP2350_SRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sys_mem, RP2350_SRAM_BASE, &s->sram);

    /*
     * The RP2350 bootrom uses 0xF0000000 as the initial MSP (from the ROM
     * vector table, entry 0). ARM Cortex-M uses a FULL DESCENDING stack:
     * the first push writes to SP-4 = 0xEFFFFFFC, so the RAM must be
     * placed BELOW 0xF0000000.  Allocate 8 KB at 0xEFFFF000-0xEFFFFFFF.
     */
    memory_region_init_ram(&s->bootrom_stack, NULL, "rp2350.bootrom_stack",
                           8 * KiB, &error_fatal);
    memory_region_add_subregion(sys_mem, 0xEFFFF000u, &s->bootrom_stack);

    /* --- APB stub covering 0x40000000-0x401FFFFF (priority -1) --- */
    memory_region_init_io(&s->apb_stub, OBJECT(machine), &apb_stub_ops,
                          s, "rp2350.apb", RP2350_APB_SIZE);
    memory_region_add_subregion_overlap(sys_mem, RP2350_APB_BASE,
                                        &s->apb_stub, -1);

    /* --- CLOCKS at 0x40010000 (overlaid on APB stub, priority 0) --- */
    memory_region_init_io(&s->clocks_mr, OBJECT(machine), &clocks_ops,
                          s, "rp2350.clocks", RP2350_CLOCKS_SIZE);
    memory_region_add_subregion_overlap(sys_mem, RP2350_CLOCKS_BASE,
                                        &s->clocks_mr, 0);

    /* --- SIO at 0xD0000000 --- */
    memory_region_init_io(&s->sio_mr, OBJECT(machine), &sio_ops,
                          s, "rp2350.sio", RP2350_SIO_SIZE);
    memory_region_add_subregion(sys_mem, RP2350_SIO_BASE, &s->sio_mr);

    /* --- PSM at 0x40018000 --- */
    memory_region_init_io(&s->psm_mr, OBJECT(machine), &psm_ops,
                          s, "rp2350.psm", RP2350_PSM_SIZE);
    memory_region_add_subregion_overlap(sys_mem, RP2350_PSM_BASE,
                                        &s->psm_mr, 0);

    /*
     * BOOTRAM: 4 KB always-on RAM at 0x400E0000.
     * Implemented as a custom IO region so that NS writes to the Secure
     * function-table range (offsets 0x80C–0x828) can be silently discarded
     * while Secure writes (from the bootrom via the NSC gate) go through.
     * This prevents MicroPython's init_array[5] from clearing the value
     * that ROM[0x28CC] deposited there, allowing init_array[7]'s poll to
     * see a non-zero result and continue booting.
     */
    memory_region_init_io(&s->bootram_mr, OBJECT(machine), &bootram_ops,
                          s, "rp2350.bootram", RP2350_BOOTRAM_SIZE);
    memory_region_add_subregion_overlap(sys_mem, RP2350_BOOTRAM_BASE,
                                        &s->bootram_mr, 0);

    /* --- TIMER0 at 0x400B0000, TIMER1 at 0x400B8000 --- */
    memory_region_init_io(&s->timer0_mr, OBJECT(machine), &timer_ops,
                          NULL, "rp2350.timer0", RP2350_TIMER0_SIZE);
    memory_region_add_subregion_overlap(sys_mem, RP2350_TIMER0_BASE,
                                        &s->timer0_mr, 0);

    memory_region_init_io(&s->timer1_mr, OBJECT(machine), &timer_ops,
                          NULL, "rp2350.timer1", RP2350_TIMER1_SIZE);
    memory_region_add_subregion_overlap(sys_mem, RP2350_TIMER1_BASE,
                                        &s->timer1_mr, 0);

    /* --- AHB stub (DMA / USB / PIO / etc.) at 0x50000000 --- */
    create_unimplemented_device("rp2350.ahb", RP2350_AHB_BASE, RP2350_AHB_SIZE);

    /* USB DPRAM: 16 KB at 0x50100000 covering all 4 atomic alias banks
     * (normal +0, XOR +0x1000, SET +0x2000, CLR +0x3000).
     * The read/write handlers decode bits [13:12] to apply the alias op. */
    memset(s->usb_dpram_buf, 0, sizeof(s->usb_dpram_buf));
    memory_region_init_io(&s->usb_dpram, OBJECT(machine), &usb_dpram_ops,
                          s, "rp2350.usb_dpram", 16 * KiB);
    memory_region_add_subregion_overlap(sys_mem, 0x50100000u, &s->usb_dpram, 1);

    /* USB controller registers at 0x50110000, overlaid on AHB stub */
    s->usb_state = USB_ST_IDLE;
    s->usb_timer = timer_new_us(QEMU_CLOCK_VIRTUAL, usb_timer_cb, s);
    memory_region_init_io(&s->usb_mr, OBJECT(machine), &usb_reg_ops,
                          s, "rp2350.usb", RP2350_USB_SIZE);
    memory_region_add_subregion_overlap(sys_mem, RP2350_USB_BASE, &s->usb_mr, 2);

    /*
     * CPU1 needs a separate MemoryRegion object for board_memory (even though
     * it physically sees the same address space).  Create an alias covering
     * the full 4 GiB before realizing the CPUs.
     */
    memory_region_init_alias(&s->cpu1_mem_alias, NULL, "rp2350.cpu1.mem",
                             sys_mem, 0, UINT64_MAX);

    /* --- Dual Cortex-M33 CPUs --- */
    for (i = 0; i < 2; i++) {
        char name[16];
        snprintf(name, sizeof(name), "cpu%d", i);

        object_initialize_child(OBJECT(machine), name, &s->cpu[i],
                                TYPE_ARMV7M);
        qdev_prop_set_string(DEVICE(&s->cpu[i]), "cpu-type",
                             ARM_CPU_TYPE_NAME("cortex-m33"));
        qdev_prop_set_uint32(DEVICE(&s->cpu[i]), "num-irq", RP2350_NUM_IRQS);
        /*
         * When the bootrom is loaded via -bios, start the CPU from the ROM
         * vector table so the bootrom's reset handler runs and initialises
         * BOOTRAM, SAU, MPU, and then launches the firmware image.
         * Without the bootrom the ROM region is zeroed, so fall back to
         * booting directly from flash.
         */
        {
            hwaddr svtor = machine->firmware ? RP2350_ROM_BASE
                                             : RP2350_FLASH_BASE;
            qdev_prop_set_uint32(DEVICE(&s->cpu[i]), "init-svtor",  svtor);
            qdev_prop_set_uint32(DEVICE(&s->cpu[i]), "init-nsvtor", svtor);
        }

        /* CPU0 uses sys_mem directly; CPU1 uses an alias of it */
        object_property_set_link(OBJECT(&s->cpu[i]), "memory",
                                 i == 0 ? OBJECT(sys_mem)
                                        : OBJECT(&s->cpu1_mem_alias),
                                 &error_fatal);
        qdev_connect_clock_in(DEVICE(&s->cpu[i]), "cpuclk", s->sysclk);

        /* Connect the RP2350 IDAU so AHB-NS addresses are NS-accessible */
        object_property_set_link(OBJECT(&s->cpu[i]), "idau",
                                 OBJECT(machine), &error_fatal);

        /* CPU1 is held powered-off until the program wakes it via SIO FIFO */
        if (i == 1) {
            qdev_prop_set_bit(DEVICE(&s->cpu[i]), "start-powered-off", true);
        }

        sysbus_realize(SYS_BUS_DEVICE(&s->cpu[i]), &error_fatal);
    }

    /* Wire USB IRQ 14 to CPU0 now that CPU is realized */
    s->usb_irq = qdev_get_gpio_in(DEVICE(&s->cpu[0]), 14);

    /* --- UART0 at 0x40070000 (PL011, serial port 0) --- */
    pl011_create(RP2350_UART0_BASE,
                 qdev_get_gpio_in(DEVICE(&s->cpu[0]), RP2350_UART0_IRQ),
                 serial_hd(0));

    /* --- UART1 at 0x40078000 (PL011, serial port 1) --- */
    pl011_create(RP2350_UART1_BASE,
                 qdev_get_gpio_in(DEVICE(&s->cpu[0]), RP2350_UART1_IRQ),
                 serial_hd(1));

    /*
     * Load the kernel.
     *
     * UF2 files are detected by their magic bytes and parsed block-by-block
     * into the flash RAM.  armv7m_load_kernel is then called with a NULL
     * filename so it registers the armv7m_reset hook (which reads SP/PC from
     * the vector table) without trying to re-load the file.
     *
     * Raw binaries and ELFs are passed through to armv7m_load_kernel as
     * usual; they land at FLASH_BASE and the same reset hook applies.
     */
    if (machine->kernel_filename
        && rp2350_is_uf2(machine->kernel_filename)) {
        int blocks = rp2350_load_uf2(s, machine->kernel_filename);
        if (blocks == 0) {
            error_report("rp2350: failed to load UF2 kernel '%s'",
                         machine->kernel_filename);
            exit(1);
        }
        armv7m_load_kernel(s->cpu[0].cpu, NULL,
                           RP2350_FLASH_BASE, RP2350_FLASH_SIZE);
    } else {
        armv7m_load_kernel(s->cpu[0].cpu, machine->kernel_filename,
                           RP2350_FLASH_BASE, RP2350_FLASH_SIZE);
    }

    /*
     * Register post-reset hooks AFTER armv7m_load_kernel, which itself
     * calls qemu_register_reset(armv7m_reset, cpu).  Our hooks must run
     * AFTER armv7m_reset so they are not overwritten by a subsequent
     * arm_cpu_reset_hold call.  The hooks restore SAU_CTRL.ALLNS=1 and
     * pre-initialize VTOR to FLASH_BASE so that the firmware's
     * runtime_init_install_ram_vector_table copies from the correct table.
     */
    for (i = 0; i < 2; i++) {
        qemu_register_reset(rp2350_sau_reset, s->cpu[i].cpu);
    }
}

static char *rp2350_get_flash_file(Object *obj, Error **errp)
{
    RP2350MachineState *s = RP2350_MACHINE(obj);
    return g_strdup(s->flash_file);
}

static void rp2350_set_flash_file(Object *obj, const char *value, Error **errp)
{
    RP2350MachineState *s = RP2350_MACHINE(obj);
    g_free(s->flash_file);
    s->flash_file = g_strdup(value);
}

static void rp2350_machine_class_init(ObjectClass *oc, const void *data)
{
    IDAUInterfaceClass *iic = IDAU_INTERFACE_CLASS(oc);
    MachineClass *mc = MACHINE_CLASS(oc);

    iic->check = rp2350_idau_check;

    mc->desc        = "Raspberry Pi RP2350 (dual Cortex-M33)";
    mc->init        = rp2350_init;
    mc->max_cpus    = 2;
    mc->default_cpus = 2;
    mc->no_floppy   = 1;
    mc->no_cdrom    = 1;
    mc->no_parallel = 1;
    mc->ignore_memory_transaction_failures = true;
    /*
     * -bios <bootrom-combined.bin>  loads the real RP2350 boot ROM.
     * Without -bios the ROM region is zeroed; the CPU still boots from
     * flash (init-svtor = 0x10000000) so Pico SDK programs work, but
     * ROM helper functions (floating-point, USB) won't be available.
     */

    /*
     * flash=<path>  — persistent 4 MB flash backing file.
     *
     * The file is loaded at startup (0xFF-filled if it doesn't exist yet).
     * The -kernel binary is then stamped into offset 0, just like picotool
     * flashing without erasing, so any littlefs data beyond the firmware
     * survives across runs.  On exit the whole region is written back.
     *
     * Example: -machine rp2350,flash=pico2.bin -kernel firmware.bin
     */
    object_class_property_add_str(oc, "flash",
                                  rp2350_get_flash_file,
                                  rp2350_set_flash_file);
    object_class_property_set_description(oc, "flash",
        "Path to 4 MiB flash backing file (preserves littlefs across runs)");
}

static const InterfaceInfo rp2350_machine_interfaces[] = {
    { TYPE_TARGET_ARM_MACHINE },
    { TYPE_TARGET_AARCH64_MACHINE },
    { TYPE_IDAU_INTERFACE },
    { }
};

static const TypeInfo rp2350_machine_typeinfo = {
    .name           = TYPE_RP2350_MACHINE,
    .parent         = TYPE_MACHINE,
    .instance_size  = sizeof(RP2350MachineState),
    .class_init     = rp2350_machine_class_init,
    .interfaces     = rp2350_machine_interfaces,
};

static void rp2350_machine_register(void)
{
    type_register_static(&rp2350_machine_typeinfo);
}
type_init(rp2350_machine_register)
