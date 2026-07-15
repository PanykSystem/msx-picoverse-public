// MSX PICOVERSE PROJECT
// (c) 2026 Cristiano Goncalves
// The Retro Hacker
//
// yamanooto.c - Yamanooto flash cartridge emulation for MSX PICOVERSE 2350
//
// Emulates the "Yamanooto" flash cartridge on the PicoVerse 2350 hardware.
// The Yamanooto is a Konami-SCC compatible 8 MB flash-ROM cartridge that also
// provides SCC / SCC+ audio and a secondary (dual) AY-3-8910 PSG.
//
// As a PicoVerse extension the cartridge is always presented as an expanded slot
// with an MSX-MUSIC (YM2413 / OPLL) FM-PAC BIOS in sub-slot 3, so FM games detect
// the OPLL. Because a single flash image can hold a mix of SCC games and FM games,
// the audio core renders either the SCC or the FM on the fly, choosing whichever
// chip the running program is currently driving (the two are never used together
// by the same game). The OPLL responds both to the FM-PAC memory registers
// (0x7FF4/0x7FF5) and to the raw I/O ports (0x7C/0x7D).
//
// The secondary (dual) PSG and a mirror of the main MSX PSG (ports 0xA0/0xA1)
// are also mixed into the cartridge DAC, but only while a channel is actually
// audible, so idle PSGs never add a buzz to the (quieter) FM music.
//
// This firmware reproduces the behaviour documented in the openMSX Yamanooto
// device (src/memory/Yamanooto.cc):
//   * Konami-SCC mapper (default) and Konami-4 mapper (CFGR bit K4)
//   * 10-bit bank registers with a global bank offset (OFFR + CFGR SUBOFF bits)
//   * Memory-mapped register window at 0x7FFC-0x7FFF (ENAR/OFFR/CFGR/FPGA)
//   * SCC and SCC+ audio (emu2212), auto-detected from the bank registers
//   * Secondary PSG on I/O ports 0x10/0x11, optionally echoed to 0xA0/0xA1
//
// The PIO state machines handle MSX bus timing deterministically: SM0 answers
// memory reads (asserting /WAIT while the RP2350 looks up the byte) and SM1
// captures memory writes for bank switching / register / SCC updates. A second
// PIO handles the I/O bus for the secondary PSG.
//
// The ROM image is concatenated after the firmware binary. A 59-byte config
// record (name + type + size + offset) precedes the raw image.
//
// ENAR WREN enables the S29GL064-compatible command interface. Its mutable
// image is held in PSRAM and changed pages are journaled in unused board flash.
//
// This work is licensed under a "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
// License". https://creativecommons.org/licenses/by-nc-sa/4.0/

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/regs/qmi.h"
#include "hardware/structs/qmi.h"
#include "pico/flash.h"
#include "hardware/structs/xip_ctrl.h"
#include "pico/audio_i2s.h"
#include "yamanooto.h"
#include "emu2212.h"
#include "emu2149.h"
#include "emu2413.h"
#include "msx_bus.pio.h"

#ifndef YAMANOOTO_FMVOICE_TRACE
#define YAMANOOTO_FMVOICE_TRACE 0
#endif

// Size of the FM-PAC BIOS image appended after the ROM payload when MSX-MUSIC
// is enabled (64 KB, paged in four 16 KB banks).
#define FMPAC_BIOS_SIZE 65536u

// -----------------------------------------------------------------------
// PIO bus contexts
// -----------------------------------------------------------------------
typedef struct {
    PIO pio;
    uint sm_read;
    uint sm_write;
    uint offset_read;
    uint offset_write;
} msx_pio_bus_t;

static msx_pio_bus_t msx_bus;
static bool msx_bus_programs_loaded = false;

typedef struct {
    PIO pio_read;
    PIO pio_write;
    uint sm_io_read;
    uint sm_io_write;
    uint offset_io_read;
    uint offset_io_write;
} msx_pio_io_bus_t;

static msx_pio_io_bus_t msx_io_bus;
static bool msx_io_bus_programs_loaded = false;

static bool debug_stdio_ready = false;

static void debug_stdio_init(void)
{
#if YAMANOOTO_USB_STDIO_DEBUG
    if (!debug_stdio_ready)
    {
        stdio_init_all();
        debug_stdio_ready = true;
    }
#endif
}

static void debug_trace(const char *fmt, ...)
{
#if YAMANOOTO_USB_STDIO_DEBUG
    if (!debug_stdio_ready)
        return;
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\r\n");
    fflush(stdout);
#else
    (void)fmt;
#endif
}

// -----------------------------------------------------------------------
// ROM source
// -----------------------------------------------------------------------
static uint32_t active_rom_size = 0;         // ROM image size in bytes
static const uint8_t *rom_base = NULL;       // Start of the raw ROM image in flash

// The board's PSRAM holds the mutable S29GL064 image. A compact journal in
// the unused upper 4 MB of the board's 16 MB flash makes programmed pages and
// erased sectors survive power cycles without modifying the XIP firmware/ROM.
#define YAMA_FLASH_SIZE             (8u * 1024u * 1024u)
#define YAMA_PSRAM_BASE             0x11000000u
#define YAMA_SAVE_STORAGE_SIZE      (4u * 1024u * 1024u)
#define YAMA_SAVE_STORAGE_OFFSET    (PICO_FLASH_SIZE_BYTES - YAMA_SAVE_STORAGE_SIZE)
#define YAMA_SAVE_RECORD_SIZE       512u
#define YAMA_SAVE_PAGE_SIZE         256u
#define YAMA_FLASH_PAGE_COUNT       (YAMA_FLASH_SIZE / YAMA_SAVE_PAGE_SIZE)
#define YAMA_FLASH_DIRTY_BYTES      (YAMA_FLASH_PAGE_COUNT / 8u)
#define YAMA_SAVE_MAGIC             0x53414D59u  // "YMAS"
#define YAMA_SAVE_TYPE_PAGE         1u
#define YAMA_SAVE_TYPE_ERASE        2u
#define YAMA_SAVE_TYPE_CHIP_ERASE   3u
#define YAMA_FLASH_BUFFER_SIZE      32u

typedef enum {
    YAMA_FLASH_READ,
    YAMA_FLASH_UNLOCK_2,
    YAMA_FLASH_COMMAND,
    YAMA_FLASH_PROGRAM,
    YAMA_FLASH_ERASE_UNLOCK_1,
    YAMA_FLASH_ERASE_UNLOCK_2,
    YAMA_FLASH_ERASE_COMMAND,
    YAMA_FLASH_AUTOSELECT,
    YAMA_FLASH_CFI,
    YAMA_FLASH_BUFFER_COUNT,
    YAMA_FLASH_BUFFER_DATA,
    YAMA_FLASH_BUFFER_CONFIRM,
} yama_flash_state_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t rom_signature;
    uint32_t address;
    uint8_t type;
    uint8_t reserved[3];
} yama_save_record_header_t;

typedef struct {
    uint32_t offset;
    const uint8_t *data;
    size_t length;
    bool erase;
} yama_flash_storage_op_t;

static uint8_t *flash_image = NULL;
static uint8_t flash_dirty_pages[YAMA_FLASH_DIRTY_BYTES];
static yama_flash_state_t flash_state = YAMA_FLASH_READ;
static uint32_t flash_rom_signature = 0;
static uint32_t save_record_cursor = 0;
static uint32_t flash_buffer_sector = 0;
static uint32_t flash_buffer_page = 0;
static uint8_t flash_buffer_count = 0;
static uint8_t flash_buffer_index = 0;
static uint8_t flash_buffer[YAMA_FLASH_BUFFER_SIZE];

static inline void __not_in_flash_func(yama_flash_page_set_dirty)(uint32_t page);
static void __not_in_flash_func(yama_flash_mark_dirty_range)(uint32_t address, uint32_t size);

// -----------------------------------------------------------------------
// SCC + dual PSG + MSX-MUSIC emulation state and I2S audio
// -----------------------------------------------------------------------
static SCC scc_instance;
static PSG psg_instance;                     // Secondary (dual) PSG on I/O 0x10/0x11
static PSG main_psg_instance;                // PSG mirror of the main MSX PSG (I/O 0xA0/0xA1)
static OPLL *msx_music_instance = NULL;      // MSX-MUSIC (YM2413) engine (always created)
static struct audio_buffer_pool *audio_pool = NULL;
static bool audio_ready = false;
static int audio_dma_channel = -1;

// The SCC (5 voices) and the FM/OPLL (9 voices) are both far too heavy to render
// together every sample on core 1, so only one is rendered at a time. Because a
// single Yamanooto image can hold a mix of SCC games and FM games, the choice is
// made on the fly from recent write activity: whichever chip the running program
// is currently driving is the one that is rendered. These counters are bumped on
// each chip's *sound-register* writes and sampled once per audio buffer.
static volatile uint32_t scc_activity = 0;   // bumped on SCC register-window writes (core 0)
static volatile uint32_t fm_activity = 0;    // bumped on OPLL register writes (core 1)
static volatile bool scc_render_requested = false;
static volatile uint32_t scc_debug_writes = 0;
static volatile uint32_t fm_debug_writes = 0;

#if YAMANOOTO_USB_STDIO_DEBUG
static volatile uint32_t audio_debug_buffers = 0;
static volatile uint32_t audio_debug_last_us = 0;
static volatile uint32_t audio_debug_max_us = 0;
static volatile uint32_t audio_debug_overruns = 0;
static volatile uint32_t audio_debug_engine = 0;
#define YAMA_AUDIO_BUFFER_BUDGET_US ((SCC_AUDIO_BUFFER_SAMPLES * 1000000u) / SCC_SAMPLE_RATE)
#endif

#if YAMANOOTO_USB_STDIO_DEBUG && YAMANOOTO_FMVOICE_TRACE
static uint8_t fm_debug_selected_reg = 0;
static uint8_t fm_debug_reg_shadow[0x40];
static uint16_t fm_debug_custom_channel_mask = 0;
#endif


// -----------------------------------------------------------------------
// Yamanooto register file
// -----------------------------------------------------------------------
static volatile uint8_t enableReg = 0;   // ENAR (0x7FFF)
static volatile uint8_t offsetReg = 0;   // OFFR (0x7FFE)
static volatile uint8_t configReg = 0;   // CFGR (0x7FFD)
static volatile uint8_t fpgaFsm   = 0;   // FPGA ID read state machine
static uint16_t bankRegs[4] = {0, 1, 2, 3};  // Offset-adjusted 10-bit bank registers
static uint8_t  rawBanks[4] = {0, 1, 2, 3};  // Raw values (used for SCC activation)
static uint8_t sccMode = 0;                    // Yamanooto SCC+ mode register (0xBFFE/0xBFFF)

enum { YAMA_ENGINE_NONE = 0, YAMA_ENGINE_SCC, YAMA_ENGINE_FM };
#define YAMA_ENGINE_IDLE_BUFFERS 90

// FPGA "read ID" response sequence (just enough for detection code to pass)
static const uint8_t FPGA_ID[5] = { 0xFFu, 0x1Fu, 0x23u, 0x00u, 0x00u };

// -----------------------------------------------------------------------
// GPIO initialisation
// -----------------------------------------------------------------------
static inline void setup_gpio(void)
{
    for (uint pin = PIN_A0; pin <= PIN_A15; ++pin)
    {
        gpio_init(pin);
        gpio_set_input_hysteresis_enabled(pin, true);
        gpio_set_dir(pin, GPIO_IN);
    }
    for (uint pin = PIN_D0; pin <= PIN_D7; ++pin)
    {
        gpio_init(pin);
        gpio_set_input_hysteresis_enabled(pin, true);
    }
    gpio_init(PIN_RD);      gpio_set_dir(PIN_RD, GPIO_IN);
    gpio_init(PIN_WR);      gpio_set_dir(PIN_WR, GPIO_IN);
    gpio_init(PIN_IORQ);    gpio_set_dir(PIN_IORQ, GPIO_IN);
    gpio_init(PIN_SLTSL);   gpio_set_dir(PIN_SLTSL, GPIO_IN);
    gpio_init(PIN_BUSSDIR); gpio_set_dir(PIN_BUSSDIR, GPIO_IN);
    gpio_init(PIN_PSRAM);   gpio_set_dir(PIN_PSRAM, GPIO_IN);
}

// -----------------------------------------------------------------------
// External PSRAM and persistent flash journal
// -----------------------------------------------------------------------
static inline void __not_in_flash_func(psram_delay_cycles)(uint32_t cycles)
{
    for (volatile uint32_t cycle = 0; cycle < cycles; ++cycle)
        __asm volatile ("nop");
}

static inline void __not_in_flash_func(psram_wait_direct_done)(void)
{
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_TXEMPTY_BITS) == 0) { }
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) { }
}

static inline void __not_in_flash_func(psram_send_direct_cmd)(uint8_t cmd, bool quad_width)
{
    qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
    qmi_hw->direct_tx = quad_width
        ? QMI_DIRECT_TX_OE_BITS | (QMI_DIRECT_TX_IWIDTH_VALUE_Q << QMI_DIRECT_TX_IWIDTH_LSB) | cmd
        : cmd;
    psram_wait_direct_done();
    qmi_hw->direct_csr &= ~QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
    (void)qmi_hw->direct_rx;
}

static bool __no_inline_not_in_flash_func(psram_init)(void)
{
    gpio_set_function(PIN_PSRAM, GPIO_FUNC_XIP_CS1);
    uint32_t irq_state = save_and_disable_interrupts();
    qmi_hw->direct_csr = (30u << QMI_DIRECT_CSR_CLKDIV_LSB) | QMI_DIRECT_CSR_EN_BITS;
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) { }
    psram_send_direct_cmd(0xF5u, true);
    psram_delay_cycles(128u);
    psram_send_direct_cmd(0x66u, false);
    psram_delay_cycles(128u);
    psram_send_direct_cmd(0x99u, false);
    psram_delay_cycles(50000u);
    psram_send_direct_cmd(0x35u, false);
    psram_delay_cycles(128u);
    qmi_hw->direct_csr &= ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS | QMI_DIRECT_CSR_EN_BITS);
    qmi_hw->m[1].timing =
        (QMI_M0_TIMING_PAGEBREAK_VALUE_1024 << QMI_M0_TIMING_PAGEBREAK_LSB) |
        (1u << QMI_M0_TIMING_SELECT_HOLD_LSB) |
        (1u << QMI_M0_TIMING_COOLDOWN_LSB) |
        (2u << QMI_M0_TIMING_RXDELAY_LSB) |
        (26u << QMI_M0_TIMING_MAX_SELECT_LSB) |
        (5u << QMI_M0_TIMING_MIN_DESELECT_LSB) |
        (2u << QMI_M0_TIMING_CLKDIV_LSB);
    qmi_hw->m[1].rfmt =
        (QMI_M0_RFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_PREFIX_WIDTH_LSB) |
        (QMI_M0_RFMT_ADDR_WIDTH_VALUE_Q << QMI_M0_RFMT_ADDR_WIDTH_LSB) |
        (QMI_M0_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_SUFFIX_WIDTH_LSB) |
        (QMI_M0_RFMT_DUMMY_WIDTH_VALUE_Q << QMI_M0_RFMT_DUMMY_WIDTH_LSB) |
        (QMI_M0_RFMT_DUMMY_LEN_VALUE_24 << QMI_M0_RFMT_DUMMY_LEN_LSB) |
        (QMI_M0_RFMT_DATA_WIDTH_VALUE_Q << QMI_M0_RFMT_DATA_WIDTH_LSB) |
        (QMI_M0_RFMT_PREFIX_LEN_VALUE_8 << QMI_M0_RFMT_PREFIX_LEN_LSB) |
        (QMI_M0_RFMT_SUFFIX_LEN_VALUE_NONE << QMI_M0_RFMT_SUFFIX_LEN_LSB);
    qmi_hw->m[1].rcmd = (0xEBu << QMI_M0_RCMD_PREFIX_LSB);
    qmi_hw->m[1].wfmt =
        (QMI_M0_WFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_PREFIX_WIDTH_LSB) |
        (QMI_M0_WFMT_ADDR_WIDTH_VALUE_Q << QMI_M0_WFMT_ADDR_WIDTH_LSB) |
        (QMI_M0_WFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_SUFFIX_WIDTH_LSB) |
        (QMI_M0_WFMT_DUMMY_WIDTH_VALUE_Q << QMI_M0_WFMT_DUMMY_WIDTH_LSB) |
        (QMI_M0_WFMT_PREFIX_LEN_VALUE_8 << QMI_M0_WFMT_PREFIX_LEN_LSB) |
        (QMI_M0_WFMT_SUFFIX_LEN_VALUE_NONE << QMI_M0_WFMT_SUFFIX_LEN_LSB);
    qmi_hw->m[1].wcmd = (0x38u << QMI_M0_WCMD_PREFIX_LSB);
    xip_ctrl_hw->ctrl |= XIP_CTRL_WRITABLE_M1_BITS;
    restore_interrupts(irq_state);
    volatile uint32_t *test = (volatile uint32_t *)0x15000000u;
    test[0] = 0x12345678u;
    return test[0] == 0x12345678u;
}

static void __no_inline_not_in_flash_func(yama_flash_storage_callback)(void *param)
{
    const yama_flash_storage_op_t *op = (const yama_flash_storage_op_t *)param;
    if (op->erase)
        flash_range_erase(op->offset, op->length);
    else
        flash_range_program(op->offset, op->data, op->length);
}

static bool yama_flash_storage_execute(yama_flash_storage_op_t *op, uint32_t timeout_ms)
{
    return flash_safe_execute(yama_flash_storage_callback, op, timeout_ms) == PICO_OK;
}

static uint32_t yama_rom_signature(void)
{
    uint32_t hash = 2166136261u;
    for (uint32_t index = 0; index < active_rom_size; ++index)
        hash = (hash ^ rom_base[index]) * 16777619u;
    return hash ^ active_rom_size;
}

static bool yama_save_write_record(uint8_t type, uint32_t address)
{
    static uint8_t record[YAMA_SAVE_RECORD_SIZE];
    memset(record, 0xFF, sizeof(record));
    yama_save_record_header_t header = {
        .magic = YAMA_SAVE_MAGIC,
        .rom_signature = flash_rom_signature,
        .address = address,
        .type = type,
    };
    memcpy(record, &header, sizeof(header));
    if (type == YAMA_SAVE_TYPE_PAGE)
        memcpy(record + YAMA_SAVE_PAGE_SIZE, flash_image + address, YAMA_SAVE_PAGE_SIZE);

    yama_flash_storage_op_t op = {
        .offset = YAMA_SAVE_STORAGE_OFFSET + save_record_cursor * YAMA_SAVE_RECORD_SIZE,
        .data = record,
        .length = sizeof(record),
        .erase = false,
    };
    if (!yama_flash_storage_execute(&op, 1000u))
        return false;
    ++save_record_cursor;
    return true;
}

static bool yama_flash_page_differs_from_initial(uint32_t page)
{
    if ((flash_dirty_pages[page / YAMA_SAVE_PAGE_SIZE / 8u] &
         (1u << ((page / YAMA_SAVE_PAGE_SIZE) & 7u))) == 0u)
        return false;

    for (uint32_t index = 0; index < YAMA_SAVE_PAGE_SIZE; ++index)
    {
        uint32_t address = page + index;
        uint8_t initial = address < active_rom_size ? rom_base[address] : 0xFFu;
        if (flash_image[address] != initial)
            return true;
    }
    return false;
}

static bool yama_save_compact(void)
{
    const uint32_t record_count = YAMA_SAVE_STORAGE_SIZE / YAMA_SAVE_RECORD_SIZE;
    uint32_t changed_pages = 0;
    for (uint32_t page = 0; page < YAMA_FLASH_SIZE; page += YAMA_SAVE_PAGE_SIZE)
    {
        if (yama_flash_page_differs_from_initial(page) && ++changed_pages > record_count)
            return false;
    }

    yama_flash_storage_op_t erase = {
        .offset = YAMA_SAVE_STORAGE_OFFSET,
        .data = NULL,
        .length = YAMA_SAVE_STORAGE_SIZE,
        .erase = true,
    };
    if (!yama_flash_storage_execute(&erase, 10000u))
        return false;

    save_record_cursor = 0;
    for (uint32_t page = 0; page < YAMA_FLASH_SIZE; page += YAMA_SAVE_PAGE_SIZE)
    {
        if (yama_flash_page_differs_from_initial(page) &&
            !yama_save_write_record(YAMA_SAVE_TYPE_PAGE, page))
            return false;
    }
    return true;
}

static bool yama_save_append(uint8_t type, uint32_t address)
{
    const uint32_t record_count = YAMA_SAVE_STORAGE_SIZE / YAMA_SAVE_RECORD_SIZE;
    if (save_record_cursor >= record_count && !yama_save_compact())
        return false;
    return yama_save_write_record(type, address);
}

static uint32_t yama_flash_sector_start(uint32_t address)
{
    return address < 0x10000u ? address & ~0x1FFFu : address & ~0xFFFFu;
}

static uint32_t yama_flash_sector_size(uint32_t address)
{
    return address < 0x10000u ? 0x2000u : 0x10000u;
}

static void yama_save_restore(void)
{
    const uint32_t record_count = YAMA_SAVE_STORAGE_SIZE / YAMA_SAVE_RECORD_SIZE;
    const uint8_t *storage = (const uint8_t *)(XIP_BASE + YAMA_SAVE_STORAGE_OFFSET);
    bool has_foreign_records = false;
    save_record_cursor = 0;
    for (; save_record_cursor < record_count; ++save_record_cursor)
    {
        const yama_save_record_header_t *header = (const yama_save_record_header_t *)(storage + save_record_cursor * YAMA_SAVE_RECORD_SIZE);
        if (header->magic == 0xFFFFFFFFu)
            break;
        if (header->magic != YAMA_SAVE_MAGIC || header->rom_signature != flash_rom_signature)
        {
            has_foreign_records = true;
            break;
        }
        if (header->address >= YAMA_FLASH_SIZE)
            continue;
        if (header->type == YAMA_SAVE_TYPE_ERASE)
        {
            memset(flash_image + yama_flash_sector_start(header->address), 0xFF,
                   yama_flash_sector_size(header->address));
            yama_flash_mark_dirty_range(yama_flash_sector_start(header->address),
                                        yama_flash_sector_size(header->address));
        }
        else if (header->type == YAMA_SAVE_TYPE_CHIP_ERASE)
        {
            memset(flash_image, 0xFF, YAMA_FLASH_SIZE);
            memset(flash_dirty_pages, 0xFF, sizeof(flash_dirty_pages));
        }
        else if (header->type == YAMA_SAVE_TYPE_PAGE && header->address <= YAMA_FLASH_SIZE - YAMA_SAVE_PAGE_SIZE)
        {
            memcpy(flash_image + header->address, storage + save_record_cursor * YAMA_SAVE_RECORD_SIZE + YAMA_SAVE_PAGE_SIZE,
                   YAMA_SAVE_PAGE_SIZE);
            yama_flash_page_set_dirty(header->address / YAMA_SAVE_PAGE_SIZE);
        }
    }
    if (has_foreign_records)
    {
        yama_flash_storage_op_t op = {
            .offset = YAMA_SAVE_STORAGE_OFFSET,
            .data = NULL,
            .length = YAMA_SAVE_STORAGE_SIZE,
            .erase = true,
        };
        if (yama_flash_storage_execute(&op, 10000u))
            save_record_cursor = 0;
    }
}

static bool yama_flash_init(void)
{
    if (!psram_init())
        return false;
    flash_image = (uint8_t *)YAMA_PSRAM_BASE;
    memset(flash_dirty_pages, 0, sizeof(flash_dirty_pages));
    flash_rom_signature = yama_rom_signature();
    yama_save_restore();
    return true;
}

// -----------------------------------------------------------------------
// PIO bus initialisation (memory reads/writes on PIO0)
// -----------------------------------------------------------------------
static void msx_pio_bus_init(void)
{
    msx_bus.pio = pio0;
    msx_bus.sm_read  = 0;
    msx_bus.sm_write = 1;

    if (!msx_bus_programs_loaded)
    {
        msx_bus.offset_read  = pio_add_program(msx_bus.pio, &msx_read_responder_program);
        msx_bus.offset_write = pio_add_program(msx_bus.pio, &msx_write_captor_program);
        msx_bus_programs_loaded = true;
    }

    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_read, false);
    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_write, false);
    pio_sm_clear_fifos(msx_bus.pio, msx_bus.sm_read);
    pio_sm_clear_fifos(msx_bus.pio, msx_bus.sm_write);
    pio_sm_restart(msx_bus.pio, msx_bus.sm_read);
    pio_sm_restart(msx_bus.pio, msx_bus.sm_write);

    pio_sm_config cfg_read = msx_read_responder_program_get_default_config(msx_bus.offset_read);
    sm_config_set_in_pins(&cfg_read, PIN_A0);
    sm_config_set_in_shift(&cfg_read, false, false, 16);
    sm_config_set_out_pins(&cfg_read, PIN_D0, 8);
    sm_config_set_out_shift(&cfg_read, true, false, 32);
    sm_config_set_sideset_pins(&cfg_read, PIN_WAIT);
    sm_config_set_jmp_pin(&cfg_read, PIN_RD);
    sm_config_set_clkdiv(&cfg_read, 1.0f);
    pio_sm_init(msx_bus.pio, msx_bus.sm_read, msx_bus.offset_read, &cfg_read);

    pio_sm_config cfg_write = msx_write_captor_program_get_default_config(msx_bus.offset_write);
    sm_config_set_in_pins(&cfg_write, PIN_A0);
    sm_config_set_in_shift(&cfg_write, false, false, 32);
    sm_config_set_fifo_join(&cfg_write, PIO_FIFO_JOIN_RX);
    sm_config_set_jmp_pin(&cfg_write, PIN_WR);
    sm_config_set_clkdiv(&cfg_write, 1.0f);
    pio_sm_init(msx_bus.pio, msx_bus.sm_write, msx_bus.offset_write, &cfg_write);

    pio_gpio_init(msx_bus.pio, PIN_WAIT);
    pio_sm_set_consecutive_pindirs(msx_bus.pio, msx_bus.sm_read, PIN_WAIT, 1, true);

    for (uint pin = PIN_D0; pin <= PIN_D7; ++pin)
    {
        pio_gpio_init(msx_bus.pio, pin);
    }
    pio_sm_set_consecutive_pindirs(msx_bus.pio, msx_bus.sm_read, PIN_D0, 8, false);
    pio_sm_set_consecutive_pindirs(msx_bus.pio, msx_bus.sm_write, PIN_D0, 8, false);

    gpio_put(PIN_WAIT, 1);

    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_read, true);
    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_write, true);
}

// -----------------------------------------------------------------------
// PIO I/O bus initialisation (secondary PSG on PIO1)
// -----------------------------------------------------------------------
static void msx_pio_io_bus_init(void)
{
    msx_io_bus.pio_read = pio1;
    msx_io_bus.pio_write = pio1;
    msx_io_bus.sm_io_read  = 0;
    msx_io_bus.sm_io_write = 1;

    if (!msx_io_bus_programs_loaded)
    {
        msx_io_bus.offset_io_read = pio_add_program(msx_io_bus.pio_read, &msx_io_read_responder_program);
        msx_io_bus.offset_io_write = pio_add_program(msx_io_bus.pio_write, &msx_io_write_captor_program);
        msx_io_bus_programs_loaded = true;
    }

    pio_sm_set_enabled(msx_io_bus.pio_read, msx_io_bus.sm_io_read, false);
    pio_sm_set_enabled(msx_io_bus.pio_write, msx_io_bus.sm_io_write, false);
    pio_sm_clear_fifos(msx_io_bus.pio_read, msx_io_bus.sm_io_read);
    pio_sm_clear_fifos(msx_io_bus.pio_write, msx_io_bus.sm_io_write);
    pio_sm_restart(msx_io_bus.pio_read, msx_io_bus.sm_io_read);
    pio_sm_restart(msx_io_bus.pio_write, msx_io_bus.sm_io_write);

    pio_sm_config cfg_io_read = msx_io_read_responder_program_get_default_config(msx_io_bus.offset_io_read);
    sm_config_set_in_pins(&cfg_io_read, PIN_A0);
    sm_config_set_in_shift(&cfg_io_read, false, false, 16);
    sm_config_set_out_pins(&cfg_io_read, PIN_D0, 8);
    sm_config_set_out_shift(&cfg_io_read, true, false, 32);
    sm_config_set_jmp_pin(&cfg_io_read, PIN_RD);
    sm_config_set_clkdiv(&cfg_io_read, 1.0f);
    pio_sm_init(msx_io_bus.pio_read, msx_io_bus.sm_io_read, msx_io_bus.offset_io_read, &cfg_io_read);

    pio_sm_config cfg_io_write = msx_io_write_captor_program_get_default_config(msx_io_bus.offset_io_write);
    sm_config_set_in_pins(&cfg_io_write, PIN_A0);
    sm_config_set_in_shift(&cfg_io_write, false, false, 32);
    sm_config_set_fifo_join(&cfg_io_write, PIO_FIFO_JOIN_RX);
    sm_config_set_jmp_pin(&cfg_io_write, PIN_WR);
    sm_config_set_clkdiv(&cfg_io_write, 1.0f);
    pio_sm_init(msx_io_bus.pio_write, msx_io_bus.sm_io_write, msx_io_bus.offset_io_write, &cfg_io_write);

    pio_sm_set_consecutive_pindirs(msx_io_bus.pio_read, msx_io_bus.sm_io_read, PIN_D0, 8, false);

    pio_sm_set_enabled(msx_io_bus.pio_read, msx_io_bus.sm_io_read, true);
    pio_sm_set_enabled(msx_io_bus.pio_write, msx_io_bus.sm_io_write, true);
}

// -----------------------------------------------------------------------
// Token / FIFO helpers
// -----------------------------------------------------------------------
static inline uint16_t __not_in_flash_func(pio_build_token)(bool drive, uint8_t data)
{
    uint8_t dir_mask = drive ? 0xFFu : 0x00u;
    return (uint16_t)data | ((uint16_t)dir_mask << 8);
}

static inline bool __not_in_flash_func(pio_try_get_write)(uint16_t *addr_out, uint8_t *data_out)
{
    if (pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_write))
        return false;

    uint32_t sample = pio_sm_get(msx_bus.pio, msx_bus.sm_write);
    *addr_out = (uint16_t)(sample & 0xFFFFu);
    *data_out = (uint8_t)((sample >> 16) & 0xFFu);
    return true;
}

static inline bool __not_in_flash_func(pio_try_get_io_write)(uint16_t *addr_out, uint8_t *data_out)
{
    if (pio_sm_is_rx_fifo_empty(msx_io_bus.pio_write, msx_io_bus.sm_io_write))
        return false;

    uint32_t sample = pio_sm_get(msx_io_bus.pio_write, msx_io_bus.sm_io_write);
    *addr_out = (uint16_t)(sample & 0xFFFFu);
    *data_out = (uint8_t)((sample >> 16) & 0xFFu);
    return true;
}

static inline bool __not_in_flash_func(pio_try_get_io_read)(uint16_t *addr_out)
{
    if (pio_sm_is_rx_fifo_empty(msx_io_bus.pio_read, msx_io_bus.sm_io_read))
        return false;

    *addr_out = (uint16_t)pio_sm_get(msx_io_bus.pio_read, msx_io_bus.sm_io_read);
    return true;
}

static inline int16_t __not_in_flash_func(clamp_i16)(int32_t sample)
{
    if (sample > 32767) return 32767;
    if (sample < -32768) return -32768;
    return (int16_t)sample;
}

// -----------------------------------------------------------------------
// Yamanooto address decoding helpers
// -----------------------------------------------------------------------
static inline uint32_t __not_in_flash_func(yama_bank_offset)(void)
{
    return ((uint32_t)offsetReg << 2) | ((uint32_t)(configReg & YAMA_SUBOFF) >> 4);
}

static inline uint16_t __not_in_flash_func(yama_mirror)(uint16_t address)
{
    // mirror 0x4000<->0xC000 and 0x8000<->0x0000: real hardware ignores the
    // upper address bit. In-slot accesses are already in [0x4000,0xBFFF].
    if (address < 0x4000u || address >= 0xC000u)
        address ^= 0x8000u;
    return address;
}

static inline uint32_t __not_in_flash_func(yama_flash_addr)(uint16_t address)
{
    uint32_t page8kB = (uint32_t)(address >> 13) - 2u;  // 0..3 for [0x4000,0xBFFF]
    uint32_t bank = (uint32_t)bankRegs[page8kB] & 0x3FFu;
    return (bank << 13) | (uint32_t)(address & 0x1FFFu);
}

static inline bool __not_in_flash_func(yama_flash_page_is_dirty)(uint32_t page)
{
    return (flash_dirty_pages[page / 8u] & (1u << (page & 7u))) != 0u;
}

static inline void __not_in_flash_func(yama_flash_page_set_dirty)(uint32_t page)
{
    flash_dirty_pages[page / 8u] |= (uint8_t)(1u << (page & 7u));
}

static void __not_in_flash_func(yama_flash_materialize_page)(uint32_t faddr)
{
    uint32_t page_address = faddr & ~(YAMA_SAVE_PAGE_SIZE - 1u);
    uint32_t page = page_address / YAMA_SAVE_PAGE_SIZE;
    if (yama_flash_page_is_dirty(page))
        return;

    uint32_t available = page_address < active_rom_size ? active_rom_size - page_address : 0u;
    uint32_t copy_size = available < YAMA_SAVE_PAGE_SIZE ? available : YAMA_SAVE_PAGE_SIZE;
    if (copy_size != 0u)
        memcpy(flash_image + page_address, rom_base + page_address, copy_size);
    if (copy_size < YAMA_SAVE_PAGE_SIZE)
        memset(flash_image + page_address + copy_size, 0xFFu, YAMA_SAVE_PAGE_SIZE - copy_size);
    yama_flash_page_set_dirty(page);
}

static void __not_in_flash_func(yama_flash_mark_dirty_range)(uint32_t address, uint32_t size)
{
    uint32_t first_page = address / YAMA_SAVE_PAGE_SIZE;
    uint32_t last_page = (address + size - 1u) / YAMA_SAVE_PAGE_SIZE;
    for (uint32_t page = first_page; page <= last_page; ++page)
        yama_flash_page_set_dirty(page);
}

static inline uint8_t __not_in_flash_func(yama_read_flash)(uint32_t faddr)
{
    if (flash_state == YAMA_FLASH_AUTOSELECT)
    {
        switch (faddr)
        {
            case 0x0000u: return 0x01u;  // AMD manufacturer ID
            case 0x0001u: return 0x00u;
            case 0x0002u: return 0x7Eu;  // x16 device-ID prefix
            case 0x0003u: return 0x22u;
            case 0x0004u: return 0x00u;  // sector-protection state: unprotected
            case 0x0005u: return 0x00u;
            case 0x0006u: return 0x0Au;  // extended ID 0xFF0A
            case 0x0007u: return 0xFFu;
            case 0x001Cu: return 0x10u;  // device ID byte 0
            case 0x001Du: return 0x22u;
            case 0x001Eu: return 0x00u;  // device ID byte 1
            case 0x001Fu: return 0x22u;
            default: return 0xFFu;
        }
    }
    if (flash_state == YAMA_FLASH_CFI)
    {
        // The chip is x16. CFI addresses are native-word addresses, so an
        // MSX byte address selects the low or high byte of that word.
        uint32_t cfi_address = faddr >> 1;
        uint8_t value = 0xFFu;
        switch (cfi_address)
        {
            case 0x10u: value = 'Q'; break;
            case 0x11u: value = 'R'; break;
            case 0x12u: value = 'Y'; break;
            case 0x13u: value = 0x02u; break;
            case 0x15u: value = 0x40u; break;
            case 0x1Bu: value = 0x27u; break;
            case 0x1Cu: value = 0x36u; break;
            case 0x1Fu: value = 7u; break;
            case 0x20u: value = 7u; break;
            case 0x21u: value = 10u; break;
            case 0x22u: value = 0u; break;
            case 0x23u: value = 3u; break;
            case 0x24u: value = 5u; break;
            case 0x25u: value = 4u; break;
            case 0x26u: value = 0u; break;
            case 0x27u: value = 23u; break;
            case 0x28u: value = 0x02u; break;
            case 0x2Au: value = 5u; break;
            case 0x2Cu: value = 2u; break;
            case 0x2Du: value = 7u; break;
            case 0x2Fu: value = 0x20u; break;
            case 0x31u: value = 126u; break;
            case 0x33u: value = 0x00u; break;
            case 0x40u: value = 'P'; break;
            case 0x41u: value = 'R'; break;
            case 0x42u: value = 'I'; break;
            case 0x43u: value = '1'; break;
            case 0x44u: value = '3'; break;
            case 0x45u: value = 0x20u; break;
            case 0x46u: value = 2u; break;
            case 0x47u: value = 1u; break;
            case 0x49u: value = 8u; break;
            case 0x4Cu: value = 2u; break;
            case 0x4Du: value = 0xB5u; break;
            case 0x4Eu: value = 0xC5u; break;
            case 0x4Fu: value = 0x02u; break;
            case 0x50u: value = 1u; break;
            default: break;
        }
        return (faddr & 1u) ? 0x00u : value;
    }
    if (flash_image && yama_flash_page_is_dirty(faddr / YAMA_SAVE_PAGE_SIZE))
        return flash_image[faddr];
    return faddr < active_rom_size ? rom_base[faddr] : 0xFFu;
}

static inline bool __not_in_flash_func(yama_flash_unlock_1)(uint32_t faddr)
{
    // Pampas uses the S29GL064 x16 command addresses through the 8-bit mapper;
    // accept the conventional x8 aliases as well for other Yamanooto software.
    return faddr == 0x0AAAu || faddr == 0x0555u;
}

static inline bool __not_in_flash_func(yama_flash_unlock_2)(uint32_t faddr)
{
    return faddr == 0x0555u || faddr == 0x02AAu;
}

static void __not_in_flash_func(yama_flash_program)(uint32_t faddr, uint8_t data)
{
    if (!flash_image || faddr >= YAMA_FLASH_SIZE)
        return;
    yama_flash_materialize_page(faddr);
    flash_image[faddr] &= data;  // NOR flash can only change 1 bits to 0.
    (void)yama_save_append(YAMA_SAVE_TYPE_PAGE, faddr & ~(YAMA_SAVE_PAGE_SIZE - 1u));
}

static void __not_in_flash_func(yama_flash_erase_sector)(uint32_t faddr)
{
    if (!flash_image || faddr >= YAMA_FLASH_SIZE)
        return;
    uint32_t sector = yama_flash_sector_start(faddr);
    memset(flash_image + sector, 0xFF, yama_flash_sector_size(faddr));
    yama_flash_mark_dirty_range(sector, yama_flash_sector_size(faddr));
    (void)yama_save_append(YAMA_SAVE_TYPE_ERASE, sector);
}

static void __not_in_flash_func(yama_flash_erase_chip)(void)
{
    if (!flash_image)
        return;
    memset(flash_image, 0xFF, YAMA_FLASH_SIZE);
    memset(flash_dirty_pages, 0xFF, sizeof(flash_dirty_pages));
    (void)yama_save_append(YAMA_SAVE_TYPE_CHIP_ERASE, 0u);
}

static void __not_in_flash_func(yama_flash_program_buffer)(void)
{
    if (!flash_image)
        return;
    for (uint8_t index = 0; index < flash_buffer_count; ++index)
    {
        yama_flash_materialize_page(flash_buffer_page + index);
        flash_image[flash_buffer_page + index] &= flash_buffer[index];
    }
    (void)yama_save_append(YAMA_SAVE_TYPE_PAGE, flash_buffer_page & ~(YAMA_SAVE_PAGE_SIZE - 1u));
}

static void __not_in_flash_func(yama_flash_write)(uint32_t faddr, uint8_t data)
{
    if (data == 0xF0u)
    {
        flash_state = YAMA_FLASH_READ;
        return;
    }

    switch (flash_state)
    {
        case YAMA_FLASH_READ:
        case YAMA_FLASH_AUTOSELECT:
            if (data == 0x98u && ((faddr >> 1) & 0x7FFu) == 0x55u)
            {
                flash_state = YAMA_FLASH_CFI;
                break;
            }
            flash_state = (data == 0xAAu && yama_flash_unlock_1(faddr))
                ? YAMA_FLASH_UNLOCK_2 : YAMA_FLASH_READ;
            break;
        case YAMA_FLASH_UNLOCK_2:
            flash_state = (data == 0x55u && yama_flash_unlock_2(faddr))
                ? YAMA_FLASH_COMMAND : YAMA_FLASH_READ;
            break;
        case YAMA_FLASH_COMMAND:
            if (data == 0x25u)
            {
                flash_buffer_sector = yama_flash_sector_start(faddr);
                flash_state = YAMA_FLASH_BUFFER_COUNT;
                break;
            }
            if (!yama_flash_unlock_1(faddr))
            {
                flash_state = YAMA_FLASH_READ;
                break;
            }
            if (data == 0xA0u) flash_state = YAMA_FLASH_PROGRAM;
            else if (data == 0x80u) flash_state = YAMA_FLASH_ERASE_UNLOCK_1;
            else if (data == 0x90u) flash_state = YAMA_FLASH_AUTOSELECT;
            else flash_state = YAMA_FLASH_READ;
            break;
        case YAMA_FLASH_PROGRAM:
            yama_flash_program(faddr, data);
            flash_state = YAMA_FLASH_READ;
            break;
        case YAMA_FLASH_ERASE_UNLOCK_1:
            flash_state = (data == 0xAAu && yama_flash_unlock_1(faddr))
                ? YAMA_FLASH_ERASE_UNLOCK_2 : YAMA_FLASH_READ;
            break;
        case YAMA_FLASH_ERASE_UNLOCK_2:
            flash_state = (data == 0x55u && yama_flash_unlock_2(faddr))
                ? YAMA_FLASH_ERASE_COMMAND : YAMA_FLASH_READ;
            break;
        case YAMA_FLASH_ERASE_COMMAND:
            if (data == 0x30u)
                yama_flash_erase_sector(faddr);
            else if (data == 0x10u)
                yama_flash_erase_chip();
            flash_state = YAMA_FLASH_READ;
            break;
        case YAMA_FLASH_CFI:
            break;
        case YAMA_FLASH_BUFFER_COUNT:
            if (faddr < flash_buffer_sector || faddr >= flash_buffer_sector + yama_flash_sector_size(faddr) ||
                data >= YAMA_FLASH_BUFFER_SIZE)
            {
                flash_state = YAMA_FLASH_READ;
                break;
            }
            flash_buffer_count = (uint8_t)(data + 1u);
            flash_buffer_index = 0;
            memset(flash_buffer, 0xFF, sizeof(flash_buffer));
            flash_state = YAMA_FLASH_BUFFER_DATA;
            break;
        case YAMA_FLASH_BUFFER_DATA:
            if (faddr < flash_buffer_sector || faddr >= flash_buffer_sector + yama_flash_sector_size(faddr))
            {
                flash_state = YAMA_FLASH_READ;
                break;
            }
            if (flash_buffer_index == 0u)
                flash_buffer_page = faddr & ~(YAMA_FLASH_BUFFER_SIZE - 1u);
            if ((faddr & ~(YAMA_FLASH_BUFFER_SIZE - 1u)) != flash_buffer_page)
            {
                flash_state = YAMA_FLASH_READ;
                break;
            }
            flash_buffer[faddr & (YAMA_FLASH_BUFFER_SIZE - 1u)] = data;
            ++flash_buffer_index;
            if (flash_buffer_index == flash_buffer_count)
                flash_state = YAMA_FLASH_BUFFER_CONFIRM;
            break;
        case YAMA_FLASH_BUFFER_CONFIRM:
            if (data == 0x29u && faddr >= flash_buffer_sector &&
                faddr < flash_buffer_sector + yama_flash_sector_size(faddr))
                yama_flash_program_buffer();
            flash_state = YAMA_FLASH_READ;
            break;
    }
}

static inline bool __not_in_flash_func(yama_scc_register_access)(uint16_t address)
{
    if (configReg & YAMA_K4)
        return false;

    if (sccMode & 0x20u)
        return (rawBanks[3] & 0x80u) && address >= 0xB800u && address < 0xBFFEu;

    return ((rawBanks[2] & 0x3Fu) == 0x3Fu) && address >= 0x9800u && address < 0xA000u;
}

static inline bool __not_in_flash_func(yama_scc_raw_enabled)(void)
{
    if (configReg & YAMA_K4)
        return false;

    if (sccMode & 0x20u)
        return (rawBanks[3] & 0x80u) != 0u;

    return (rawBanks[2] & 0x3Fu) == 0x3Fu;
}

static inline uint16_t __not_in_flash_func(yama_scc_emulator_address)(uint16_t address)
{
    return (uint16_t)(address & 0x00FFu);
}

static inline void __not_in_flash_func(yama_sync_scc_mapper_state)(void)
{
    static uint8_t last_active = 0xFFu;
    static uint8_t last_mode = 0xFFu;
    static uint16_t last_base = 0xFFFFu;

    if (configReg & YAMA_K4)
    {
        scc_instance.active = 0;
        scc_render_requested = false;
        if (last_active != 0u || last_mode != 0u || last_base != 0u)
        {
            debug_trace("YAMA SCC sync K4 active=0 cfgr=%02X raw=%02X/%02X", configReg, rawBanks[2], rawBanks[3]);
            last_active = 0u;
            last_mode = 0u;
            last_base = 0u;
        }
        return;
    }

    if (sccMode & 0x20u)
    {
        scc_instance.base_adr = 0xB000u;
        scc_instance.mode = 1;
        scc_instance.active = (rawBanks[3] & 0x80u) ? 1u : 0u;
    }
    else
    {
        scc_instance.base_adr = 0x9000u;
        scc_instance.mode = 0;
        scc_instance.active = ((rawBanks[2] & 0x3Fu) == 0x3Fu) ? 1u : 0u;
    }

    if (last_active != (uint8_t)scc_instance.active ||
        last_mode != (uint8_t)scc_instance.mode ||
        last_base != (uint16_t)scc_instance.base_adr)
    {
        debug_trace("YAMA SCC sync active=%u mode=%u base=%04X sccMode=%02X raw2=%02X raw3=%02X banks=%03X/%03X cfgr=%02X render=%u",
                    (unsigned)scc_instance.active,
                    (unsigned)scc_instance.mode,
                    (unsigned)scc_instance.base_adr,
                    (unsigned)sccMode,
                    (unsigned)rawBanks[2],
                    (unsigned)rawBanks[3],
                    (unsigned)bankRegs[2],
                    (unsigned)bankRegs[3],
                    (unsigned)configReg,
                    scc_render_requested ? 1u : 0u);
        last_active = (uint8_t)scc_instance.active;
        last_mode = (uint8_t)scc_instance.mode;
        last_base = (uint16_t)scc_instance.base_adr;
    }
}

// -----------------------------------------------------------------------
// Memory read handler (returns the byte the cartridge drives onto the bus)
// -----------------------------------------------------------------------
static inline uint8_t __not_in_flash_func(yama_mem_read)(uint16_t addr)
{
    // Register window at 0x7FFC-0x7FFF (NOT mirrored) when REGEN is set
    if (addr >= YAMA_FPGA_REG && addr <= YAMA_ENAR && (enableReg & YAMA_REGEN))
    {
        switch (addr)
        {
            case YAMA_FPGA_REG:
                if (!(configReg & YAMA_FPGA_EN))
                    return 0xFFu;
                return FPGA_ID[fpgaFsm];
            case YAMA_CFGR:
                return (uint8_t)(configReg | YAMA_FPGA_WAIT);
            case YAMA_OFFR:
                return offsetReg;
            case YAMA_ENAR:
                return enableReg;
            default:
                break;
        }
    }

    uint16_t maddr = yama_mirror(addr);

    // SCC / SCC+ register reads (K4 mode has no SCC)
    if (!(configReg & YAMA_K4))
    {
        if (yama_scc_register_access(maddr))
            return (uint8_t)SCC_readMem(&scc_instance, yama_scc_emulator_address(maddr));
        // SCC+ mode register readback at 0xBFFE-0xBFFF
        if ((maddr & 0xFFFEu) == 0xBFFEu)
            return (uint8_t)SCC_read(&scc_instance, maddr);
    }

    if (configReg & YAMA_ROMDIS)
        return 0xFFu;

    return yama_read_flash(yama_flash_addr(maddr));
}

// -----------------------------------------------------------------------
// Memory write handler (bank switching, SCC and register updates)
// -----------------------------------------------------------------------
static inline void __not_in_flash_func(yama_mem_write)(uint16_t addr, uint8_t data)
{
    // Register window at 0x7FFC-0x7FFF (NOT mirrored)
    if (addr >= YAMA_FPGA_REG && addr <= YAMA_ENAR)
    {
        if (addr == YAMA_ENAR)
        {
            enableReg = data;
            debug_trace("YAMA ENAR=%02X", data);
        }
        else if (enableReg & YAMA_REGEN)
        {
            switch (addr)
            {
                case YAMA_FPGA_REG:
                    if (configReg & YAMA_FPGA_EN)
                    {
                        // Minimal FPGA "read ID" command handshake
                        switch (fpgaFsm)
                        {
                            case 0: if (data == 0x9Fu) fpgaFsm = 1; break;
                            case 1: case 2: case 3: ++fpgaFsm; break;
                            default: fpgaFsm = 0; break;
                        }
                    }
                    break;
                case YAMA_CFGR:
                    configReg = data;
                    yama_sync_scc_mapper_state();
                    debug_trace("YAMA CFGR=%02X offset=%u", data, (unsigned)yama_bank_offset());
                    break;
                case YAMA_OFFR:
                    offsetReg = data;
                    debug_trace("YAMA OFFR=%02X offset=%u", data, (unsigned)yama_bank_offset());
                    break;
                default:
                    break;
            }
        }
        return; // register addresses never trigger mapper/SCC updates
    }

    uint16_t maddr = yama_mirror(addr);
    uint32_t page8kB = (uint32_t)(maddr >> 13) - 2u;

    if (enableReg & YAMA_WREN)
    {
        if (!(configReg & YAMA_ROMDIS))
            yama_flash_write(yama_flash_addr(maddr), data);
        return;
    }

    uint32_t offset = yama_bank_offset();

    if (configReg & YAMA_K4)
    {
        yama_sync_scc_mapper_state();
        // Konami-4: bank 0 (0x4000-0x5FFF) is fixed; the others switch
        if (!(configReg & YAMA_MDIS) && maddr >= 0x6000u)
        {
            bankRegs[page8kB] = (uint16_t)(((uint32_t)data + offset) & 0x3FFu);
            rawBanks[page8kB] = data;
        }
        return;
    }

    bool bank_switch_write = ((maddr & 0x1800u) == 0x1000u) && !(configReg & YAMA_MDIS);
    if (bank_switch_write)
    {
        bankRegs[page8kB] = (uint16_t)(((uint32_t)data + offset) & 0x3FFu);
        rawBanks[page8kB] = data;
    }

    if ((maddr & 0xFFFEu) == 0xBFFEu)
    {
        sccMode = data;
        debug_trace("YAMA SCC mode write addr=%04X data=%02X", maddr, data);
    }

    yama_sync_scc_mapper_state();

    bool scc_register_write = yama_scc_register_access(maddr);

    // Konami-SCC: Yamanooto applies the ROM offset only to flash banks. SCC
    // visibility comes from raw bank/mode state, then the SCC core receives
    // only the low SCC register byte, exactly like openMSX's Yamanooto device.
    if (scc_register_write)
    {
        scc_render_requested = true;
        SCC_writeMem(&scc_instance, yama_scc_emulator_address(maddr), data);
        uint32_t count = ++scc_debug_writes;
        if (count <= 128u || ((count & 0xFFu) == 0u))
        {
            debug_trace("YAMA SCC write#%lu addr=%04X reg=%02X data=%02X active=%u mode=%u render=%u raw2=%02X raw3=%02X",
                        (unsigned long)count,
                        (unsigned)maddr,
                        (unsigned)yama_scc_emulator_address(maddr),
                        (unsigned)data,
                        (unsigned)scc_instance.active,
                        (unsigned)scc_instance.mode,
                        scc_render_requested ? 1u : 0u,
                        (unsigned)rawBanks[2],
                        (unsigned)rawBanks[3]);
        }
    }

    // Record SCC sound activity for the on-the-fly engine selector. Yamanooto's
    // SCC visibility is controlled by raw bank values, not the offset-adjusted
    // flash bank, so use the same raw-bank window test as the read path.
    if (scc_register_write || scc_render_requested)
        scc_activity++;
}

// -----------------------------------------------------------------------
// MSX-MUSIC (YM2413 / OPLL) rendering
// -----------------------------------------------------------------------
// The OPLL is touched ONLY on core 1 (drained from the write ring + I/O FIFO,
// then rendered), so no spin lock is needed. FM-PAC memory-register writes
// captured on core 0 are handed to core 1 through a lock-free ring; this both
// decouples core 0 from the audio path and buffers register bursts so the
// (lossy) MSX write FIFOs do not drop OPLL writes -- dropped or serialised
// register writes are what makes the FM sound "robotic".
#define MSX_MUSIC_WRITE_RING_SIZE 256u
#define MSX_MUSIC_WRITE_RING_MASK (MSX_MUSIC_WRITE_RING_SIZE - 1u)
static volatile uint32_t msx_music_write_ring[MSX_MUSIC_WRITE_RING_SIZE];
static volatile uint32_t msx_music_write_ring_head; // producer (core 0)
static volatile uint32_t msx_music_write_ring_tail; // consumer (core 1)
static volatile uint32_t msx_music_write_ring_drops;

typedef enum {
    MSX_MUSIC_SOURCE_FMPAC_MEM = 0,
    MSX_MUSIC_SOURCE_RAW_IO = 1,
} msx_music_write_source_t;

#if YAMANOOTO_USB_STDIO_DEBUG
static void audio_debug_report(void)
{
    static uint32_t last_buffers = 0;
    uint32_t buffers = audio_debug_buffers;
    if (buffers == last_buffers || (buffers - last_buffers) < 128u)
        return;

    uint32_t max_us = audio_debug_max_us;
    uint32_t overruns = audio_debug_overruns;
    audio_debug_max_us = 0;
    last_buffers = buffers;
    debug_trace("YAMA AUDIO budget=%luus last=%luus max=%luus over=%lu buffers=%lu engine=%lu fmWrites=%lu keys=%08lX drops=%lu",
                (unsigned long)YAMA_AUDIO_BUFFER_BUDGET_US,
                (unsigned long)audio_debug_last_us,
                (unsigned long)max_us,
                (unsigned long)overruns,
                (unsigned long)buffers,
                (unsigned long)audio_debug_engine,
                (unsigned long)fm_debug_writes,
                msx_music_instance ? (unsigned long)msx_music_instance->slot_key_status : 0ul,
                (unsigned long)msx_music_write_ring_drops);
}
#endif

#if YAMANOOTO_USB_STDIO_DEBUG && YAMANOOTO_FMVOICE_TRACE
static const char *__not_in_flash_func(msx_music_source_name)(msx_music_write_source_t source)
{
    return source == MSX_MUSIC_SOURCE_FMPAC_MEM ? "mem" : "io";
}

static void __not_in_flash_func(msx_music_debug_trace_write)(msx_music_write_source_t source, uint8_t port, uint8_t data)
{
    if (port == MSX_MUSIC_PORT_REG)
    {
        fm_debug_selected_reg = data & 0x3Fu;
        return;
    }

    if (port != MSX_MUSIC_PORT_DATA)
        return;

    uint8_t reg = fm_debug_selected_reg;
    uint8_t old_data = fm_debug_reg_shadow[reg];
    fm_debug_reg_shadow[reg] = data;

    uint32_t count = fm_debug_writes;
    if (reg <= 0x07u && old_data != data)
    {
        debug_trace("YAMA FMVOICE patch#%lu src=%s reg=%02X data=%02X old=%02X patch=%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X keys=%08lX drops=%lu",
                    (unsigned long)count,
                    msx_music_source_name(source),
                    (unsigned)reg,
                    (unsigned)data,
                    (unsigned)old_data,
                    (unsigned)fm_debug_reg_shadow[0],
                    (unsigned)fm_debug_reg_shadow[1],
                    (unsigned)fm_debug_reg_shadow[2],
                    (unsigned)fm_debug_reg_shadow[3],
                    (unsigned)fm_debug_reg_shadow[4],
                    (unsigned)fm_debug_reg_shadow[5],
                    (unsigned)fm_debug_reg_shadow[6],
                    (unsigned)fm_debug_reg_shadow[7],
                    msx_music_instance ? (unsigned long)msx_music_instance->slot_key_status : 0ul,
                    (unsigned long)msx_music_write_ring_drops);
    }
    else if (reg >= 0x30u && reg <= 0x38u)
    {
        uint8_t channel = reg - 0x30u;
        uint8_t instrument = (uint8_t)(data >> 4);
        uint16_t channel_bit = (uint16_t)(1u << channel);
        if (instrument == 0u && (fm_debug_custom_channel_mask & channel_bit) == 0u)
        {
            fm_debug_custom_channel_mask |= channel_bit;
            debug_trace("YAMA FMVOICE custom ch=%u vol=%02X patch=%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X keys=%08lX drops=%lu",
                        (unsigned)channel,
                        (unsigned)(data & 0x0Fu),
                        (unsigned)fm_debug_reg_shadow[0],
                        (unsigned)fm_debug_reg_shadow[1],
                        (unsigned)fm_debug_reg_shadow[2],
                        (unsigned)fm_debug_reg_shadow[3],
                        (unsigned)fm_debug_reg_shadow[4],
                        (unsigned)fm_debug_reg_shadow[5],
                        (unsigned)fm_debug_reg_shadow[6],
                        (unsigned)fm_debug_reg_shadow[7],
                        msx_music_instance ? (unsigned long)msx_music_instance->slot_key_status : 0ul,
                        (unsigned long)msx_music_write_ring_drops);
        }
        else if (instrument != 0u)
        {
            fm_debug_custom_channel_mask &= (uint16_t)~channel_bit;
        }
    }
}
#endif

// core 0: queue an OPLL register write for core 1 (no OPLL access here).
static inline void __not_in_flash_func(msx_music_ring_push)(uint8_t port, uint8_t data, msx_music_write_source_t source)
{
    uint32_t head = msx_music_write_ring_head;
    uint32_t next = (head + 1u) & MSX_MUSIC_WRITE_RING_MASK;
    if (next == msx_music_write_ring_tail)
    {
        uint32_t drops = ++msx_music_write_ring_drops;
        if (drops <= 16u || ((drops & 0xFFu) == 0u))
            debug_trace("YAMA FM ring drop#%lu src=%u port=%02X data=%02X head=%lu tail=%lu",
                        (unsigned long)drops,
                        (unsigned)source,
                        (unsigned)port,
                        (unsigned)data,
                        (unsigned long)head,
                        (unsigned long)msx_music_write_ring_tail);
        return; // ring full: drop (audio only)
    }
    msx_music_write_ring[head] = ((uint32_t)source << 16) | ((uint32_t)port << 8) | data;
    __dmb();
    msx_music_write_ring_head = next;
}

// core 1: apply a register write directly to the OPLL.
static inline void __not_in_flash_func(msx_music_apply)(uint8_t port, uint8_t data, msx_music_write_source_t source)
{
    if (msx_music_instance)
    {
        OPLL_writeIO(msx_music_instance, port, data);
        scc_render_requested = false;
        fm_activity++;   // record FM activity for the on-the-fly engine selector
        uint32_t count = ++fm_debug_writes;
    #if YAMANOOTO_USB_STDIO_DEBUG && YAMANOOTO_FMVOICE_TRACE
        msx_music_debug_trace_write(source, port, data);
    #elif !YAMANOOTO_USB_STDIO_DEBUG
        if (count <= 64u || ((count & 0xFFu) == 0u))
            debug_trace("YAMA FM write#%lu port=%u data=%02X keys=%08lX", (unsigned long)count, (unsigned)port, (unsigned)data, (unsigned long)msx_music_instance->slot_key_status);
    #else
        (void)count;
        (void)source;
#endif
    }
}

// core 1: drain everything core 0 queued.
static inline void __not_in_flash_func(msx_music_drain_ring)(void)
{
    if (!msx_music_instance)
        return;
    uint32_t tail = msx_music_write_ring_tail;
    while (tail != msx_music_write_ring_head)
    {
        __dmb();
        uint32_t entry = msx_music_write_ring[tail];
        msx_music_apply((uint8_t)((entry >> 8) & 0xFFu),
                        (uint8_t)(entry & 0xFFu),
                        (msx_music_write_source_t)((entry >> 16) & 0xFFu));
        tail = (tail + 1u) & MSX_MUSIC_WRITE_RING_MASK;
    }
    msx_music_write_ring_tail = tail;
}

// Output conditioning (mirrors openMSX / the Explorer FM path): a ~35 Hz DC
// blocker removes the OPLL DC offset, a gentle 1-pole low-pass (~16 kHz) softens
// the YM2413 exponential-DAC quantization noise, and a soft-knee limiter tames
// peaks instead of hard-clipping (hard clipping sounds harsh / robotic).
#define MSX_MUSIC_DC_R       0.995f
#define MSX_MUSIC_LP_A       0.70f
#define MSX_MUSIC_LIMIT_KNEE 24000
#define MSX_MUSIC_LIMIT_CEIL 32767
static float msx_music_dc_x1 = 0.0f;
static float msx_music_dc_y1 = 0.0f;
static float msx_music_lp_y1 = 0.0f;

static inline void msx_music_reset_filters(void)
{
    msx_music_dc_x1 = 0.0f;
    msx_music_dc_y1 = 0.0f;
    msx_music_lp_y1 = 0.0f;
}

static inline int32_t __not_in_flash_func(msx_music_filter_sample)(int16_t in)
{
    float x = (float)in;
    float dc = x - msx_music_dc_x1 + MSX_MUSIC_DC_R * msx_music_dc_y1;
    msx_music_dc_x1 = x;
    msx_music_dc_y1 = dc;
    msx_music_lp_y1 += MSX_MUSIC_LP_A * (dc - msx_music_lp_y1);
    return (int32_t)msx_music_lp_y1;
}

static inline int16_t __not_in_flash_func(msx_music_soft_limit)(int32_t g)
{
    int32_t mag = (g < 0) ? -g : g;
    if (mag <= MSX_MUSIC_LIMIT_KNEE)
        return (int16_t)g;
    const int32_t range = MSX_MUSIC_LIMIT_CEIL - MSX_MUSIC_LIMIT_KNEE;
    int32_t excess = mag - MSX_MUSIC_LIMIT_KNEE;
    int32_t compressed = (int32_t)(((int64_t)range * excess) / (range + excess));
    int32_t out = MSX_MUSIC_LIMIT_KNEE + compressed;
    if (out > MSX_MUSIC_LIMIT_CEIL)
        out = MSX_MUSIC_LIMIT_CEIL;
    return (int16_t)((g < 0) ? -out : out);
}

// core 1: render the next filtered/gained OPLL sample (not yet limited -- the
// soft limiter is applied once to the final SCC/FM + PSG mix).
static inline int32_t __not_in_flash_func(msx_music_calc_fm)(void)
{
    return msx_music_filter_sample(OPLL_calc(msx_music_instance)) << MSX_MUSIC_VOLUME_SHIFT;
}

// FM is rendered while a voice is keyed on, or for this many audio buffers after
// the last OPLL write (covers release tails). One buffer is SCC_AUDIO_BUFFER_SAMPLES
// / 44100 s (~5.8 ms), so ~90 buffers is ~0.5 s.
#define FM_IDLE_BUFFERS 90

// core 1: sum both PSGs (secondary + main-PSG mirror), clocking and mixing them
// every sample -- never dropped. Used when the PSG is the primary source (pure
// PSG or SCC games), like the Explorer standalone PSG path: no phase freezes or
// mute/unmute clicks. Idle PSGs output 0 (voltbl[0] == 0) and the DC blocker
// removes the unipolar offset, so mixing them costs only a little CPU.
static inline int32_t __not_in_flash_func(psg_mix_sample)(void)
{
    return ((int32_t)PSG_calc(&psg_instance) << PSG_VOLUME_SHIFT)
         + ((int32_t)PSG_calc(&main_psg_instance) << PSG_VOLUME_SHIFT);
}

// Is any PSG channel actually producing sound? (non-zero fixed volume or a
// running envelope, AND tone or noise enabled in the mixer).
static inline bool __not_in_flash_func(psg_has_audible_channels)(const PSG *psg)
{
    uint8_t mixer = psg->reg[7];
    for (uint8_t channel = 0; channel < 3; channel++)
    {
        uint8_t volume = psg->reg[8 + channel] & 0x1Fu;
        if ((volume & 0x10u) != 0u)
        {
            if (psg->env_freq == 0u)
                continue;
        }
        else if ((volume & 0x0Fu) == 0u)
        {
            continue;
        }
        bool tone_enabled = (mixer & (1u << channel)) == 0u;
        bool noise_enabled = (mixer & (1u << (channel + 3u))) == 0u;
        if (tone_enabled || noise_enabled)
            return true;
    }
    return false;
}

// core 1: audible-gated PSG sum. Used only when FM is the primary source: the
// OPLL render is heavy, so skipping PSG_calc while a PSG is silent keeps core 1
// within its per-sample budget (otherwise it underruns -> noise under FM). The
// gate transitions are inaudible here because they are masked by the FM.
// Mirrors the Explorer MSX-MUSIC + PSG path.
static inline int32_t __not_in_flash_func(psg_mix_gated)(void)
{
    int32_t out = 0;
    if (psg_has_audible_channels(&psg_instance))
        out += (int32_t)PSG_calc(&psg_instance) << PSG_VOLUME_SHIFT;
    if (psg_has_audible_channels(&main_psg_instance))
        out += (int32_t)PSG_calc(&main_psg_instance) << PSG_VOLUME_SHIFT;
    return out;
}

// -----------------------------------------------------------------------
// Secondary PSG I/O servicing (runs on the audio core)
// -----------------------------------------------------------------------
static inline void __not_in_flash_func(yama_service_psg_io)(void)
{
    // Apply any OPLL register writes queued by core 0 (FM-PAC 0x7FF4/0x7FF5).
    msx_music_drain_ring();

    uint16_t io_addr;
    uint8_t io_data;
    while (pio_try_get_io_write(&io_addr, &io_data))
    {
        uint8_t port = (uint8_t)(io_addr & 0xFFu);
        if (port == PSG_PORT_REG || port == PSG_PORT_DATA)
        {
            PSG_writeIO(&psg_instance, port, io_data);   // secondary (dual) PSG
        }
        else if (port == PSG_ECHO_PORT_REG || port == PSG_ECHO_PORT_DATA)
        {
            // PSG mirror: render the main MSX PSG (0xA0/0xA1) through the DAC.
            PSG_writeIO(&main_psg_instance, port, io_data);
            // Echo mode also drives the secondary PSG from the main PSG ports.
            if (configReg & YAMA_ECHO)
                PSG_writeIO(&psg_instance, port, io_data);
        }
        else if (msx_music_instance &&
                 (port == MSX_MUSIC_PORT_REG || port == MSX_MUSIC_PORT_DATA))
        {
            // Optional MSX-MUSIC (YM2413) on I/O ports 0x7C/0x7D (core 1 owns it).
            msx_music_apply(port, io_data, MSX_MUSIC_SOURCE_RAW_IO);
        }
    }

    // The secondary PSG is write-only; acknowledge captured I/O reads with a
    // tri-state token so the bus is never driven.
    while (pio_try_get_io_read(&io_addr))
    {
        pio_sm_put_blocking(msx_io_bus.pio_read, msx_io_bus.sm_io_read,
                            pio_build_token(false, 0xFFu));
    }
}

// -----------------------------------------------------------------------
// Audio: mix SCC + secondary PSG into the I2S DAC (core 1)
// -----------------------------------------------------------------------
static void i2s_audio_init(void)
{
    if (audio_ready)
        return;

    // Unmute DAC (active-high mute: low = unmuted)
    gpio_init(I2S_MUTE_PIN);
    gpio_set_dir(I2S_MUTE_PIN, GPIO_OUT);
    gpio_put(I2S_MUTE_PIN, 0);

    static audio_format_t audio_format = {
        .sample_freq = SCC_SAMPLE_RATE,
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .channel_count = 2,
    };

    static struct audio_buffer_format producer_format = {
        .format = &audio_format,
        .sample_stride = 4,  // 2 channels * 2 bytes
    };

    audio_pool = audio_new_producer_pool(&producer_format, 3, SCC_AUDIO_BUFFER_SAMPLES);

    if (audio_dma_channel < 0)
    {
        for (int ch = 0; ch < NUM_DMA_CHANNELS; ++ch)
        {
            if (!dma_channel_is_claimed((uint)ch))
            {
                audio_dma_channel = ch;
                break;
            }
        }
    }
    if (audio_dma_channel < 0)
    {
        audio_pool = NULL;
        return;
    }

    static struct audio_i2s_config i2s_config = {
        .data_pin = I2S_DATA_PIN,
        .clock_pin_base = I2S_BCLK_PIN,
        .dma_channel = 0,
        .pio_sm = 2,  // PIO0 SM2 (SM0/SM1 are the MSX memory bus)
    };
    i2s_config.dma_channel = (uint)audio_dma_channel;

    audio_i2s_setup(&audio_format, &i2s_config);
    audio_i2s_connect(audio_pool);
    audio_i2s_set_enabled(true);
    audio_ready = true;
}

static inline struct audio_buffer *__not_in_flash_func(take_audio_buffer_servicing_psg)(void)
{
    while (true)
    {
        yama_service_psg_io();
        struct audio_buffer *buffer = take_audio_buffer(audio_pool, false);
        if (buffer)
            return buffer;
        tight_loop_contents();
    }
}

static void __no_inline_not_in_flash_func(core1_yamanooto_audio)(void)
{
    flash_safe_execute_core_init();
    msx_music_reset_filters();

    // On-the-fly engine selection. SCC and FM are never driven together by one
    // game, and a single image can hold SCC games, FM games and pure-PSG games.
    // Render only the chip the program is *currently* driving (plus the PSG):
    //   * FM  while the OPLL has a voice keyed on, or was written very recently
    //         (covers release tails);
    //   * SCC while the SCC is enabled;
    //   * otherwise nothing (pure PSG).
    // This must NOT latch: the FM-PAC BIOS writes the OPLL once at boot, and a
    // latched "render FM" would then play the idle OPLL as a constant tone
    // (a beep) under pure-PSG games. Rendering both heavy engines at once would
    // also overrun the core-1 sample budget.
    int engine = YAMA_ENGINE_NONE;
    uint32_t seen_scc = scc_activity;
    uint32_t seen_fm = fm_activity;
    int scc_idle = YAMA_ENGINE_IDLE_BUFFERS;
    int fm_idle = YAMA_ENGINE_IDLE_BUFFERS;   // start fully idle (no FM)
    int last_engine = -1;

    while (true)
    {
        struct audio_buffer *buffer = take_audio_buffer_servicing_psg();

        uint32_t now_scc = scc_activity;
        if (now_scc != seen_scc)
        {
            seen_scc = now_scc;
            scc_idle = 0;
            engine = YAMA_ENGINE_SCC;
        }
        else if (scc_idle < YAMA_ENGINE_IDLE_BUFFERS)
        {
            ++scc_idle;
        }

        uint32_t now_fm = fm_activity;
        if (now_fm != seen_fm)
        {
            seen_fm = now_fm;
            fm_idle = 0;
            if (scc_idle >= YAMA_ENGINE_IDLE_BUFFERS)
                engine = YAMA_ENGINE_FM;
        }
        else if (fm_idle < YAMA_ENGINE_IDLE_BUFFERS)
        {
            ++fm_idle;
        }
        bool scc_playing = scc_render_requested;
        bool fm_keyed = (msx_music_instance != NULL) && (msx_music_instance->slot_key_status != 0u);
        bool fm_playing = !scc_playing && (fm_keyed || (fm_idle < YAMA_ENGINE_IDLE_BUFFERS));

        if (engine == YAMA_ENGINE_SCC && !scc_playing)
            engine = fm_playing ? YAMA_ENGINE_FM : YAMA_ENGINE_NONE;
        else if (engine == YAMA_ENGINE_FM && !fm_playing)
            engine = scc_playing ? YAMA_ENGINE_SCC : YAMA_ENGINE_NONE;
        else if (scc_playing)
            engine = YAMA_ENGINE_SCC;
        else if (engine == YAMA_ENGINE_NONE)
            engine = scc_playing ? YAMA_ENGINE_SCC : (fm_playing ? YAMA_ENGINE_FM : YAMA_ENGINE_NONE);

        if (engine != last_engine)
        {
#if !YAMANOOTO_USB_STDIO_DEBUG
            debug_trace("YAMA audio engine=%d scc=%u fm=%u fmKey=%u sccReq=%u sccWrites=%lu fmWrites=%lu keys=%08lX",
                        engine,
                        scc_playing ? 1u : 0u,
                        fm_playing ? 1u : 0u,
                        fm_keyed ? 1u : 0u,
                        scc_render_requested ? 1u : 0u,
                        (unsigned long)scc_debug_writes,
                        (unsigned long)fm_debug_writes,
                        msx_music_instance ? (unsigned long)msx_music_instance->slot_key_status : 0ul);
#endif
            last_engine = engine;
        }

        int16_t *samples = (int16_t *)buffer->buffer->bytes;
    #if YAMANOOTO_USB_STDIO_DEBUG
        uint32_t render_start_us = time_us_32();
    #endif
        for (int i = 0; i < SCC_AUDIO_BUFFER_SAMPLES; i++)
        {
            yama_service_psg_io();
            // Only the FM (heavy OPLL) path gates the PSG, to stay within the
            // core-1 budget. SCC and pure-PSG render the full PSG mix every
            // sample, exactly like Explorer's SCC + PSG path -- no software DC
            // blocker (the I2S DAC output is AC-coupled in hardware), so the
            // SCC + PSG sum is never pushed into extra clipping/noise.
            int32_t psg = (engine == YAMA_ENGINE_FM) ? psg_mix_gated() : psg_mix_sample();
            int16_t s;
            if (engine == YAMA_ENGINE_FM)
            {
                // Soft-limit the FM + PSG mix so peaks are compressed rather than
                // hard-clipped (hard clipping sounds harsh / robotic).
                s = msx_music_soft_limit(msx_music_calc_fm() + psg);
            }
            else if (engine == YAMA_ENGINE_SCC)
            {
                // SCC + PSG: hard clamp.
                s = clamp_i16(((int32_t)SCC_calc(&scc_instance) << SCC_VOLUME_SHIFT) + psg);
            }
            else
            {
                // Pure PSG: neither heavy engine is being driven.
                s = clamp_i16(psg);
            }
            samples[i * 2]     = s;  // left
            samples[i * 2 + 1] = s;  // right
        }
#if YAMANOOTO_USB_STDIO_DEBUG
        uint32_t render_us = time_us_32() - render_start_us;
        audio_debug_last_us = render_us;
        if (render_us > audio_debug_max_us)
            audio_debug_max_us = render_us;
        if (render_us > YAMA_AUDIO_BUFFER_BUDGET_US)
            audio_debug_overruns++;
        audio_debug_engine = (uint32_t)engine;
        audio_debug_buffers++;
#endif
        buffer->sample_count = SCC_AUDIO_BUFFER_SAMPLES;
        give_audio_buffer(audio_pool, buffer);
    }
}

// -----------------------------------------------------------------------
// FM-PAC BIOS emulation (used only in the expanded-slot MSX-MUSIC mode)
// -----------------------------------------------------------------------
// The FM-PAC presents a 64 KB BIOS in four 16 KB banks at 0x4000-0x7FFF, an
// 8 KB battery-backed SRAM (unlocked by writing 0x4D/0x69 to 0x5FFE/0x5FFF),
// a control register (0x7FF6), a bank register (0x7FF7) and the OPLL register
// pair at 0x7FF4/0x7FF5.
typedef struct {
    uint8_t page;
    uint8_t control;
    uint8_t sram_key_5ffe;
    uint8_t sram_key_5fff;
    uint8_t sram[8192];
} fmpac_state_t;

static inline bool __not_in_flash_func(fmpac_sram_enabled)(const fmpac_state_t *fmpac)
{
    return fmpac->sram_key_5ffe == 0x4Du && fmpac->sram_key_5fff == 0x69u;
}

static inline void __not_in_flash_func(fmpac_handle_write)(fmpac_state_t *fmpac, uint16_t addr, uint8_t data)
{
    if (addr == 0x7FF4u)
    {
        msx_music_ring_push(MSX_MUSIC_PORT_REG, data, MSX_MUSIC_SOURCE_FMPAC_MEM);
    }
    else if (addr == 0x7FF5u)
    {
        msx_music_ring_push(MSX_MUSIC_PORT_DATA, data, MSX_MUSIC_SOURCE_FMPAC_MEM);
    }
    else if (addr == 0x7FF6u)
    {
        fmpac->control = data;
        debug_trace("YAMA FMPAC control=%02X sram=%u page=%u", (unsigned)data, fmpac_sram_enabled(fmpac) ? 1u : 0u, (unsigned)fmpac->page);
    }
    else if (addr == 0x7FF7u)
    {
        fmpac->page = data & 0x03u;
        debug_trace("YAMA FMPAC page=%u raw=%02X control=%02X", (unsigned)fmpac->page, (unsigned)data, (unsigned)fmpac->control);
    }
    else if (addr == 0x5FFEu)
    {
        fmpac->sram_key_5ffe = data;
        debug_trace("YAMA FMPAC sram-key 5FFE=%02X enabled=%u", (unsigned)data, fmpac_sram_enabled(fmpac) ? 1u : 0u);
    }
    else if (addr == 0x5FFFu)
    {
        fmpac->sram_key_5fff = data;
        debug_trace("YAMA FMPAC sram-key 5FFF=%02X enabled=%u", (unsigned)data, fmpac_sram_enabled(fmpac) ? 1u : 0u);
    }
    else if (fmpac_sram_enabled(fmpac) && addr >= 0x4000u && addr <= 0x5FFFu)
    {
        fmpac->sram[addr - 0x4000u] = data;
    }
}

static inline uint8_t __not_in_flash_func(fmpac_handle_read)(const fmpac_state_t *fmpac,
                                                             const uint8_t *bios_base, uint16_t addr)
{
    if (addr == 0x7FF6u)
        return fmpac->control;
    if (addr == 0x7FF7u)
        return fmpac->page;
    if (fmpac_sram_enabled(fmpac) && addr >= 0x4000u && addr <= 0x5FFFu)
        return fmpac->sram[addr - 0x4000u];

    if (addr >= 0x4000u && addr <= 0x7FFFu)
    {
        uint32_t rel = ((uint32_t)(fmpac->page & 0x03u) << 14) | (addr & 0x3FFFu);
        if (rel < FMPAC_BIOS_SIZE)
            return bios_base[rel];
    }
    return 0xFFu;
}

// -----------------------------------------------------------------------
// Sound-chip initialisation (SCC, secondary PSG, optional MSX-MUSIC)
// -----------------------------------------------------------------------
static void __not_in_flash_func(yama_init_sound_chips)(void)
{
    // SCC engine: ENHANCED type handles both SCC and SCC+ dynamically.
    memset(&scc_instance, 0, sizeof(SCC));
    scc_instance.clk  = SCC_CLOCK;
    scc_instance.rate = SCC_SAMPLE_RATE;
    SCC_set_quality(&scc_instance, 1);
    scc_instance.type = SCC_ENHANCED;
    SCC_reset(&scc_instance);

    // Secondary (dual) PSG engine on 0x10/0x11
    memset(&psg_instance, 0, sizeof(psg_instance));
    psg_instance.rate = PSG_SAMPLE_RATE;
    PSG_setVolumeMode(&psg_instance, 2);
    PSG_setClock(&psg_instance, PSG_CLOCK);
    PSG_setQuality(&psg_instance, 1);
    PSG_reset(&psg_instance);

    // PSG mirror of the main MSX PSG (0xA0/0xA1)
    memset(&main_psg_instance, 0, sizeof(main_psg_instance));
    main_psg_instance.rate = PSG_SAMPLE_RATE;
    PSG_setVolumeMode(&main_psg_instance, 2);
    PSG_setClock(&main_psg_instance, PSG_CLOCK);
    PSG_setQuality(&main_psg_instance, 1);
    PSG_reset(&main_psg_instance);

    // MSX-MUSIC (YM2413) engine. Always created: the FM-PAC BIOS is always
    // present in the expanded slot, and the audio core decides on the fly
    // whether to render SCC or FM based on which chip the game is driving.
    msx_music_instance = OPLL_new(MSX_MUSIC_CLOCK, MSX_MUSIC_SAMPLE_RATE);
    if (msx_music_instance)
    {
        OPLL_reset(msx_music_instance);
        OPLL_setChipType(msx_music_instance, OPLL_2413_TONE);
        OPLL_resetPatch(msx_music_instance, OPLL_2413_TONE);
    }
}

// -----------------------------------------------------------------------
// Expanded-slot cold-boot bootstrap (used before presenting the FM-PAC).
// Serves a tiny extension ROM whose INIT sets the boot flag (port 0xF4 bit 7)
// and restarts the machine, so the firmware can present the expanded slot on
// the second boot when audio/PIO are fully initialised. Mirrors the loadrom
// hybrid loaders.
// -----------------------------------------------------------------------
static void __not_in_flash_func(yama_wait_for_expanded_bootstrap)(void)
{
    static const uint8_t bootstrap_rom[] = {
        0x41, 0x42,          // "AB"
        0x0A, 0x40,          // INIT = 0x400A
        0x00, 0x00,          // STATEMENT
        0x00, 0x00,          // DEVICE
        0x00, 0x00,          // TEXT
        0xF3,                // DI
        0xDB, 0xF4,          // IN A,(0xF4)
        0xF6, 0x80,          // OR 0x80
        0xD3, 0xF4,          // OUT (0xF4),A
        0xC7                 // RST 0 -> restart
    };

    msx_pio_bus_init();

    bool restart_detected = false;
    bool init_called = false;

    while (!restart_detected)
    {
        uint16_t waddr;
        uint8_t wdata;
        while (pio_try_get_write(&waddr, &wdata))
        {
        }

        if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
            if (init_called && addr == 0x0000u)
            {
                pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(false, 0xFFu));
                restart_detected = true;
            }
            else
            {
                if (addr >= 0x400Au && addr <= 0x4011u)
                    init_called = true;

                bool in_window = (addr >= 0x4000u) && (addr <= 0x7FFFu);
                uint8_t data = 0xFFu;
                if (in_window)
                {
                    uint32_t rel = addr - 0x4000u;
                    if (rel < sizeof(bootstrap_rom))
                        data = bootstrap_rom[rel];
                }
                pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
            }
        }
        else if (init_called && !gpio_get(PIN_RD) && ((gpio_get_all() & 0xFFFFu) == 0x0000u))
        {
            restart_detected = true;
        }
    }
}

// -----------------------------------------------------------------------
// Main emulation loop (core 0)
// -----------------------------------------------------------------------
// The cartridge is always presented as an expanded slot (secondary slot
// register at 0xFFFF). Sub-slot 0 is the Yamanooto game (full mapper /
// registers / SCC); sub-slot 3 is the FM-PAC BIOS at page 1 (0x4000-0x7FFF).
// The OPLL is fed both by the FM-PAC memory registers (handled here on core 0)
// and by the raw I/O ports (handled on core 1 in yama_service_psg_io). The
// audio core renders SCC or FM on the fly depending on what the game drives.
static void __no_inline_not_in_flash_func(yamanooto_run_fmpac)(void)
{
    rom_base = rom + ROM_RECORD_SIZE;
    const uint8_t *fmpac_bios_base = rom_base + active_rom_size;

    debug_trace("YAMA run_fmpac rom_size=%lu", (unsigned long)active_rom_size);

    yama_init_sound_chips();

    // Cold-boot expanded-slot handshake (serves a bootstrap ROM, then restarts).
    yama_wait_for_expanded_bootstrap();

    // Freeze the bus while re-initialising for the expanded configuration.
    pio_sm_set_enabled(pio0, 0, false);
    pio_sm_set_enabled(pio0, 1, false);
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 0);

    // The bootstrap responder must be ready immediately after power-up. Delay
    // the PSRAM bring-up and ROM copy until its forced restart is held on WAIT.
    if (!yama_flash_init())
        debug_trace("YAMA PSRAM unavailable; flash save support disabled");

    static fmpac_state_t fmpac;
    memset(&fmpac, 0, sizeof(fmpac));
    fmpac.control = 0x10u;

    uint8_t subslot_reg = 0x00u;  // all pages default to sub-slot 0 (the game)

    // Memory bus (PIO0), I/O bus (PIO1) then audio on core 1. Audio is launched
    // only now, after the bootstrap: an active audio core/DMA perturbs the
    // timing-critical cold-boot expanded-slot handshake.
    msx_pio_bus_init();
    msx_pio_io_bus_init();
    i2s_audio_init();
    multicore_launch_core1(core1_yamanooto_audio);

    while (true)
    {
        uint16_t waddr;
        uint8_t  wdata;
        while (pio_try_get_write(&waddr, &wdata))
        {
            if (waddr == 0xFFFFu)
            {
                subslot_reg = wdata;
                debug_trace("YAMA subslot=%02X", subslot_reg);
                continue;
            }

            uint8_t page = (uint8_t)((waddr >> 14) & 0x03u);
            uint8_t active_subslot = (uint8_t)((subslot_reg >> (page * 2)) & 0x03u);
            if (active_subslot == 0)
            {
                yama_mem_write(waddr, wdata);
            }
            else if (active_subslot == 3 && page == 1)
            {
                fmpac_handle_write(&fmpac, waddr, wdata);
            }
        }

        if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);

            // Drain writes that arrived while waiting
            while (pio_try_get_write(&waddr, &wdata))
            {
                if (waddr == 0xFFFFu)
                {
                    subslot_reg = wdata;
                    debug_trace("YAMA subslot=%02X", subslot_reg);
                    continue;
                }
                uint8_t wpage = (uint8_t)((waddr >> 14) & 0x03u);
                uint8_t wsub = (uint8_t)((subslot_reg >> (wpage * 2)) & 0x03u);
                if (wsub == 0)
                    yama_mem_write(waddr, wdata);
                else if (wsub == 3 && wpage == 1)
                    fmpac_handle_write(&fmpac, waddr, wdata);
            }

            uint8_t data = 0xFFu;
            if (addr == 0xFFFFu)
            {
                data = (uint8_t)~subslot_reg;  // expanded-slot detection
            }
            else
            {
                uint8_t page = (uint8_t)((addr >> 14) & 0x03u);
                uint8_t active_subslot = (uint8_t)((subslot_reg >> (page * 2)) & 0x03u);
                if (active_subslot == 0)
                {
                    data = yama_mem_read(addr);
                }
                else if (active_subslot == 3 && page == 1 && addr >= 0x4000u && addr <= 0x7FFFu)
                {
                    data = fmpac_handle_read(&fmpac, fmpac_bios_base, addr);
                }
            }

            pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(true, data));
        }
    }
}

// -----------------------------------------------------------------------
// Main emulation loop (core 0) - plain slot, MSX-MUSIC raw I/O only
// -----------------------------------------------------------------------
// Selected by default (config-record YAMA_MSX_MUSIC_FLAG clear). The cartridge
// is presented exactly like the real Yamanooto hardware / openMSX: a single,
// non-expanded slot that answers the mapper directly at 0x4000-0xBFFF (with the
// usual mirroring), with no secondary-slot register (0xFFFF) and no FM-PAC BIOS
// sub-slot. Games that mis-handle the cartridge slot being expanded (e.g. Aleste
// Gaiden) work in this mode. Raw MSX-MUSIC I/O ports 0x7C/0x7D remain active so
// direct OPLL I/O can be tested without exposing the FM-PAC BIOS.
static void __no_inline_not_in_flash_func(yamanooto_run_simple)(void)
{
    rom_base = rom + ROM_RECORD_SIZE;

    // Plain-slot images must answer the MSX cold-boot slot scan immediately.
    // USB debug uses a zero CDC connect wait, so it can be brought up before
    // the bus without blocking the initial cartridge probe.
#if YAMANOOTO_USB_STDIO_DEBUG
    debug_stdio_init();
    debug_trace("YAMA boot USB_DEBUG=%u", (unsigned)YAMANOOTO_USB_STDIO_DEBUG);
    debug_trace("YAMA config active_rom_size=%lu", (unsigned long)active_rom_size);
    debug_trace("YAMA plain slot mode (MSX-MUSIC IO only, no FM-PAC BIOS)");
#endif

    msx_pio_bus_init();

    yama_init_sound_chips();
#if !YAMANOOTO_USB_STDIO_DEBUG
    if (!yama_flash_init())
        debug_trace("YAMA PSRAM unavailable; flash save support disabled");
#else
    debug_trace("YAMA debug plain mode: PSRAM save support skipped");
#endif
    msx_pio_io_bus_init();
    i2s_audio_init();
    multicore_launch_core1(core1_yamanooto_audio);
    debug_trace("YAMA run_simple (plain slot, MSX-MUSIC IO only) rom_size=%lu",
                (unsigned long)active_rom_size);

    while (true)
    {
        uint16_t waddr;
        uint8_t  wdata;
        while (pio_try_get_write(&waddr, &wdata))
            yama_mem_write(waddr, wdata);

        if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);

            // Drain writes that arrived while waiting
            while (pio_try_get_write(&waddr, &wdata))
                yama_mem_write(waddr, wdata);

            uint8_t data = yama_mem_read(addr);
            pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(true, data));
        }
    }
}

int __no_inline_not_in_flash_func(main)(void)
{
    // RP2350 flash timing + system clock (same as the loadrom firmware).
    qmi_hw->m[0].timing = 0x40000202;
    set_sys_clock_khz(210000, true);

    setup_gpio();

    // Configuration record: name(50) + type(1) + size(4) + offset(4).
    // The FM-PAC BIOS is always appended after the ROM image by the tool.
    memcpy(&active_rom_size, rom + ROM_NAME_MAX + 1, sizeof(uint32_t));

    // The config-record 'type' byte selects the FM-PAC. By default (flag clear)
    // the cartridge is a plain, non-expanded slot exactly like the real
    // Yamanooto. The PC tool's --fmpac option sets YAMA_MSX_MUSIC_FLAG to add
    // the expanded slot with the FM-PAC BIOS in sub-slot 3 for FM games.
    uint8_t config_type = rom[ROM_NAME_MAX];
    if (config_type & YAMA_MSX_MUSIC_FLAG)
    {
        debug_stdio_init();
        debug_trace("YAMA boot USB_DEBUG=%u", (unsigned)YAMANOOTO_USB_STDIO_DEBUG);
        debug_trace("YAMA config active_rom_size=%lu", (unsigned long)active_rom_size);
        debug_trace("YAMA MSX-MUSIC flag set: expanded slot + FM-PAC");
        yamanooto_run_fmpac();   // expanded slot: game (sub-slot 0) + FM-PAC BIOS (sub-slot 3)
    }
    else
    {
        yamanooto_run_simple();  // plain slot: game + raw MSX-MUSIC I/O, no FM-PAC BIOS
    }
    return 0;
}
