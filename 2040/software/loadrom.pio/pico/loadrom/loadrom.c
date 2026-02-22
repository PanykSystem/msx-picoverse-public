// MSX PICOVERSE PROJECT
// (c) 2026 Cristiano Goncalves
// The Retro Hacker
//
// loadrom.c - PIO-based ROM loader for MSX PICOVERSE project - v2.0
//
// This program loads ROM images using the MSX PICOVERSE project with the RP2040 PIO
// (Programmable I/O) hardware to handle MSX bus timing deterministically.
// The PIO state machines monitor /SLTSL and /RD//WR signals, assert /WAIT to freeze
// the Z80, and exchange address/data with the CPU through FIFOs. This frees the CPU
// from tight bit-banging loops and guarantees timing via the /WAIT mechanism.
//
// You need to concatenate the ROM image to the end of this program binary in order to load it.
// The program will then act as a simple ROM cartridge that responds to memory read requests from the MSX.
// 
// This work is licensed under a "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
// License". https://creativecommons.org/licenses/by-nc-sa/4.0/

#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "loadrom.h"
#include "sunrise_ide.h"
#include "msx_bus.pio.h"

// -----------------------------------------------------------------------
// PIO bus context
// -----------------------------------------------------------------------
typedef struct {
    PIO pio;
    uint sm_read;
    uint sm_write;
    uint offset_read;
    uint offset_write;
} msx_pio_bus_t;

static msx_pio_bus_t msx_bus;

// I/O bus context (PIO1) for memory mapper port access
typedef struct {
    PIO pio_read;
    PIO pio_write;
    uint sm_io_read;
    uint sm_io_write;
    uint offset_io_read;
    uint offset_io_write;
} msx_pio_io_bus_t;

static msx_pio_io_bus_t msx_io_bus;

// Tracks how many bytes of the ROM are cached in SRAM (0 = no cache)
static uint32_t rom_cached_size = 0;
// Current cache capacity in bytes. Normal mode uses full rom_sram size,
// mapper mode reduces this to 64KB so mapper RAM can share SRAM safely.
static uint32_t rom_cache_capacity = CACHE_SIZE;

// -----------------------------------------------------------------------
// ROM source preparation (cache to SRAM, flash fallback for large ROMs)
// -----------------------------------------------------------------------
// For ROMs that fit in the 192KB SRAM cache, the entire ROM is copied to
// SRAM and rom_base is redirected there.  For ROMs larger than the cache,
// the first 192KB is cached in SRAM and rom_base stays pointing to flash.
// read_rom_byte() transparently serves from SRAM for offsets within the
// cached region and falls back to flash XIP for the rest.
static inline void __not_in_flash_func(prepare_rom_source)(
    uint32_t offset,
    bool cache_enable,
    uint32_t preferred_size,
    const uint8_t **rom_base_out,
    uint32_t *available_length_out)
{
    const uint8_t *rom_base = rom + offset;
    uint32_t available_length = active_rom_size;

    if (preferred_size != 0u && (available_length == 0u || available_length > preferred_size))
    {
        available_length = preferred_size;
    }

    if (cache_enable && available_length > 0u)
    {
        uint32_t bytes_to_cache = (available_length > rom_cache_capacity)
                      ? rom_cache_capacity
                                  : available_length;

        gpio_init(PIN_WAIT);
        gpio_set_dir(PIN_WAIT, GPIO_OUT);
        gpio_put(PIN_WAIT, 0);

        // DMA bulk copy from flash XIP to SRAM.
        // Byte transfers are used because rom_base may not be 4-byte aligned
        // (ROM data starts at __flash_binary_end + 55-byte header).  With
        // DMA_SIZE_32 the RP2040 silently masks the two LSBs of the address,
        // reading from the wrong location and corrupting the cache.
        int dma_chan = dma_claim_unused_channel(true);
        dma_channel_config dma_cfg = dma_channel_get_default_config(dma_chan);
        channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
        channel_config_set_read_increment(&dma_cfg, true);
        channel_config_set_write_increment(&dma_cfg, true);
        dma_channel_configure(dma_chan, &dma_cfg,
            rom_sram,                        // write address (SRAM)
            rom_base,                        // read address (flash XIP)
            bytes_to_cache,                  // transfer count (bytes)
            true);                           // start immediately
        dma_channel_wait_for_finish_blocking(dma_chan);
        dma_channel_unclaim(dma_chan);
        gpio_put(PIN_WAIT, 1);

        rom_cached_size = bytes_to_cache;

        if (available_length <= rom_cache_capacity)
        {
            // Entire ROM fits in SRAM cache
            rom_base = rom_sram;
        }
        // else: ROM exceeds cache.  rom_base stays pointing to flash.
        // read_rom_byte() serves from SRAM for offsets < rom_cached_size,
        // and falls back to flash XIP for the rest.
    }
    else
    {
        rom_cached_size = 0;
    }

    *rom_base_out = rom_base;
    *available_length_out = available_length;
}

// -----------------------------------------------------------------------
// GPIO initialisation (address, data, control pins)
// -----------------------------------------------------------------------
static inline void setup_gpio(void)
{
    // Address pins A0-A15 as inputs
    for (uint pin = PIN_A0; pin <= PIN_A15; ++pin)
    {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
    }

    // Data pins D0-D7 (will be managed by PIO)
    for (uint pin = PIN_D0; pin <= PIN_D7; ++pin)
    {
        gpio_init(pin);
    }

    // Control signals as inputs
    gpio_init(PIN_RD);      gpio_set_dir(PIN_RD, GPIO_IN);
    gpio_init(PIN_WR);      gpio_set_dir(PIN_WR, GPIO_IN);
    gpio_init(PIN_IORQ);    gpio_set_dir(PIN_IORQ, GPIO_IN);
    gpio_init(PIN_SLTSL);   gpio_set_dir(PIN_SLTSL, GPIO_IN);
    gpio_init(PIN_BUSSDIR); gpio_set_dir(PIN_BUSSDIR, GPIO_IN);
}

// -----------------------------------------------------------------------
// PIO bus initialisation
// -----------------------------------------------------------------------
static void msx_pio_bus_init(void)
{
    msx_bus.pio = pio0;
    msx_bus.sm_read  = 0;
    msx_bus.sm_write = 1;

    // Load PIO programs
    msx_bus.offset_read  = pio_add_program(msx_bus.pio, &msx_read_responder_program);
    msx_bus.offset_write = pio_add_program(msx_bus.pio, &msx_write_captor_program);

    // ----- Read responder SM (SM0) -----
    pio_sm_config cfg_read = msx_read_responder_program_get_default_config(msx_bus.offset_read);
    sm_config_set_in_pins(&cfg_read, PIN_A0);                // in base = GPIO 0
    sm_config_set_in_shift(&cfg_read, false, false, 16);     // shift left, no autopush, 16 bits
    sm_config_set_out_pins(&cfg_read, PIN_D0, 8);            // out base = GPIO 16, 8 pins
    sm_config_set_out_shift(&cfg_read, true, false, 32);     // shift right (LSB first), no autopull
    sm_config_set_sideset_pins(&cfg_read, PIN_WAIT);         // side-set = GPIO 28
    sm_config_set_jmp_pin(&cfg_read, PIN_RD);                 // jmp pin = /RD for polling
    sm_config_set_clkdiv(&cfg_read, 1.0f);                   // Run at full system clock
    pio_sm_init(msx_bus.pio, msx_bus.sm_read, msx_bus.offset_read, &cfg_read);

    // ----- Write captor SM (SM1) -----
    pio_sm_config cfg_write = msx_write_captor_program_get_default_config(msx_bus.offset_write);
    sm_config_set_in_pins(&cfg_write, PIN_A0);               // in base = GPIO 0
    sm_config_set_in_shift(&cfg_write, false, false, 32);    // shift left, no autopush, 32 bits
    sm_config_set_fifo_join(&cfg_write, PIO_FIFO_JOIN_RX);   // Join FIFOs for 8-deep RX buffer
    sm_config_set_jmp_pin(&cfg_write, PIN_WR);                // jmp pin = /WR for polling
    sm_config_set_clkdiv(&cfg_write, 1.0f);
    pio_sm_init(msx_bus.pio, msx_bus.sm_write, msx_bus.offset_write, &cfg_write);

    // ----- Pin configuration for PIO -----
    // /WAIT pin: PIO side-set output, initially deasserted (high)
    pio_gpio_init(msx_bus.pio, PIN_WAIT);
    pio_sm_set_consecutive_pindirs(msx_bus.pio, msx_bus.sm_read, PIN_WAIT, 1, true);

    // Data pins: hand over to PIO, initially tri-stated (input)
    for (uint pin = PIN_D0; pin <= PIN_D7; ++pin)
    {
        pio_gpio_init(msx_bus.pio, pin);
    }
    pio_sm_set_consecutive_pindirs(msx_bus.pio, msx_bus.sm_read, PIN_D0, 8, false);
    pio_sm_set_consecutive_pindirs(msx_bus.pio, msx_bus.sm_write, PIN_D0, 8, false);

    // Ensure /WAIT starts high before enabling state machines
    gpio_put(PIN_WAIT, 1);

    // Enable both state machines
    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_read, true);
    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_write, true);
}

// -----------------------------------------------------------------------
// PIO I/O bus initialisation (for memory mapper port access on PIO1)
// -----------------------------------------------------------------------
static void msx_pio_io_bus_init(void)
{
    // I/O read/write on PIO1: lightweight software responses are used only
    // for mapper ports FC-FF, with no WAIT stretching.
    msx_io_bus.pio_read = pio1;
    msx_io_bus.pio_write = pio1;
    msx_io_bus.sm_io_read  = 0;
    msx_io_bus.sm_io_write = 1;

    // Load PIO programs for I/O
    msx_io_bus.offset_io_read = pio_add_program(msx_io_bus.pio_read, &msx_io_read_responder_program);
    msx_io_bus.offset_io_write = pio_add_program(msx_io_bus.pio_write, &msx_io_write_captor_program);

    // ----- I/O Read responder SM (PIO1 SM0) -----
    pio_sm_config cfg_io_read = msx_io_read_responder_program_get_default_config(msx_io_bus.offset_io_read);
    sm_config_set_in_pins(&cfg_io_read, PIN_A0);               // in base = GPIO 0
    sm_config_set_in_shift(&cfg_io_read, false, false, 16);    // shift left, no autopush, 16 bits
    sm_config_set_out_pins(&cfg_io_read, PIN_D0, 8);           // out base = GPIO 16, 8 pins
    sm_config_set_out_shift(&cfg_io_read, true, false, 32);    // shift right (LSB first), no autopull
    sm_config_set_jmp_pin(&cfg_io_read, PIN_RD);               // jmp pin = /RD
    sm_config_set_clkdiv(&cfg_io_read, 1.0f);
    pio_sm_init(msx_io_bus.pio_read, msx_io_bus.sm_io_read, msx_io_bus.offset_io_read, &cfg_io_read);

    // ----- I/O Write captor SM (PIO1 SM1) -----
    pio_sm_config cfg_io_write = msx_io_write_captor_program_get_default_config(msx_io_bus.offset_io_write);
    sm_config_set_in_pins(&cfg_io_write, PIN_A0);            // in base = GPIO 0
    sm_config_set_in_shift(&cfg_io_write, false, false, 32); // shift left, no autopush, 32 bits
    sm_config_set_fifo_join(&cfg_io_write, PIO_FIFO_JOIN_RX); // Join FIFOs for 8-deep RX buffer
    sm_config_set_jmp_pin(&cfg_io_write, PIN_WR);            // jmp pin = /WR
    sm_config_set_clkdiv(&cfg_io_write, 1.0f);
    pio_sm_init(msx_io_bus.pio_write, msx_io_bus.sm_io_write, msx_io_bus.offset_io_write, &cfg_io_write);

    // Data pins are controlled by PIO1 for I/O read responses.
    pio_sm_set_consecutive_pindirs(msx_io_bus.pio_read, msx_io_bus.sm_io_read, PIN_D0, 8, false);

    // Enable I/O read and I/O write state machines.
    pio_sm_set_enabled(msx_io_bus.pio_read, msx_io_bus.sm_io_read, true);
    pio_sm_set_enabled(msx_io_bus.pio_write, msx_io_bus.sm_io_write, true);
}

// -----------------------------------------------------------------------
// Token helpers
// -----------------------------------------------------------------------

// Read a byte from the ROM, using SRAM cache when possible, flash otherwise.
static inline uint8_t __not_in_flash_func(read_rom_byte)(const uint8_t *rom_base, uint32_t rel)
{
    return (rel < rom_cached_size) ? rom_sram[rel] : rom_base[rel];
}

// Build a 16-bit token to send back to the read SM via TX FIFO.
//   bits[7:0]  = data byte
//   bits[15:8] = pindirs mask (0xFF = drive bus, 0x00 = tri-state)
static inline uint16_t __not_in_flash_func(pio_build_token)(bool drive, uint8_t data)
{
    uint8_t dir_mask = drive ? 0xFFu : 0x00u;
    return (uint16_t)data | ((uint16_t)dir_mask << 8);
}

// Try to consume a write event from the write captor FIFO.
// Returns false if FIFO is empty.
static inline bool __not_in_flash_func(pio_try_get_write)(uint16_t *addr_out, uint8_t *data_out)
{
    if (pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_write))
        return false;

    uint32_t sample = pio_sm_get(msx_bus.pio, msx_bus.sm_write);
    *addr_out = (uint16_t)(sample & 0xFFFFu);
    *data_out = (uint8_t)((sample >> 16) & 0xFFu);
    return true;
}

// Drain all pending write events, invoking a handler for each.
static inline void __not_in_flash_func(pio_drain_writes)(void (*handler)(uint16_t addr, uint8_t data, void *ctx), void *ctx)
{
    uint16_t addr;
    uint8_t data;
    while (pio_try_get_write(&addr, &data))
    {
        handler(addr, data, ctx);
    }
}

// Try to consume an I/O write event from the I/O write captor FIFO (PIO1).
// Returns false if FIFO is empty.
static inline bool __not_in_flash_func(pio_try_get_io_write)(uint16_t *addr_out, uint8_t *data_out)
{
    if (pio_sm_is_rx_fifo_empty(msx_io_bus.pio_write, msx_io_bus.sm_io_write))
        return false;

    uint32_t sample = pio_sm_get(msx_io_bus.pio_write, msx_io_bus.sm_io_write);
    *addr_out = (uint16_t)(sample & 0xFFFFu);
    *data_out = (uint8_t)((sample >> 16) & 0xFFu);
    return true;
}

// Map an 8-bit mapper register value to a valid mapper page index.
// 192KB mapper RAM provides 12 pages of 16KB each.
static inline uint8_t __not_in_flash_func(mapper_page_from_reg)(uint8_t reg)
{
    return (uint8_t)(reg % MAPPER_PAGES);
}

// Try to consume an I/O read event from the I/O read responder FIFO (PIO1).
// Returns false if FIFO is empty.
static inline bool __not_in_flash_func(pio_try_get_io_read)(uint16_t *addr_out)
{
    if (pio_sm_is_rx_fifo_empty(msx_io_bus.pio_read, msx_io_bus.sm_io_read))
        return false;

    *addr_out = (uint16_t)pio_sm_get(msx_io_bus.pio_read, msx_io_bus.sm_io_read);
    return true;
}

// Drain all pending I/O write events, invoking a handler for each.
static inline void __not_in_flash_func(pio_drain_io_writes)(void (*handler)(uint16_t addr, uint8_t data, void *ctx), void *ctx)
{
    uint16_t addr;
    uint8_t data;
    while (pio_try_get_io_write(&addr, &data))
    {
        handler(addr, data, ctx);
    }
}

// -----------------------------------------------------------------------
// Bank switching write handlers (used by mapper ROM types)
// -----------------------------------------------------------------------

// Context structures passed to write handlers
typedef struct {
    uint8_t *bank_regs;
} bank8_ctx_t;

typedef struct {
    uint16_t *bank_regs;
} bank16_ctx_t;

// Konami SCC write handler
static inline void __not_in_flash_func(handle_konamiscc_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint8_t *regs = ((bank8_ctx_t *)ctx)->bank_regs;
    if      (addr >= 0x5000u && addr <= 0x57FFu) regs[0] = data;
    else if (addr >= 0x7000u && addr <= 0x77FFu) regs[1] = data;
    else if (addr >= 0x9000u && addr <= 0x97FFu) regs[2] = data;
    else if (addr >= 0xB000u && addr <= 0xB7FFu) regs[3] = data;
}

// Konami (no SCC) write handler
static inline void __not_in_flash_func(handle_konami_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint8_t *regs = ((bank8_ctx_t *)ctx)->bank_regs;
    if      (addr >= 0x6000u && addr <= 0x67FFu) regs[1] = data;
    else if (addr >= 0x8000u && addr <= 0x87FFu) regs[2] = data;
    else if (addr >= 0xA000u && addr <= 0xA7FFu) regs[3] = data;
}

// ASCII8 write handler
static inline void __not_in_flash_func(handle_ascii8_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint8_t *regs = ((bank8_ctx_t *)ctx)->bank_regs;
    if      (addr >= 0x6000u && addr <= 0x67FFu) regs[0] = data;
    else if (addr >= 0x6800u && addr <= 0x6FFFu) regs[1] = data;
    else if (addr >= 0x7000u && addr <= 0x77FFu) regs[2] = data;
    else if (addr >= 0x7800u && addr <= 0x7FFFu) regs[3] = data;
}

// ASCII16 write handler
static inline void __not_in_flash_func(handle_ascii16_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint8_t *regs = ((bank8_ctx_t *)ctx)->bank_regs;
    if      (addr >= 0x6000u && addr <= 0x67FFu) regs[0] = data;
    else if (addr >= 0x7000u && addr <= 0x77FFu) regs[1] = data;
}

// NEO8 write handler (16-bit bank registers, 12-bit segment)
static inline void __not_in_flash_func(handle_neo8_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint16_t *regs = ((bank16_ctx_t *)ctx)->bank_regs;
    uint16_t base_addr = addr & 0xF800u;
    uint8_t bank_index = 6;

    switch (base_addr)
    {
        case 0x5000: case 0x1000: case 0x9000: case 0xD000: bank_index = 0; break;
        case 0x5800: case 0x1800: case 0x9800: case 0xD800: bank_index = 1; break;
        case 0x6000: case 0x2000: case 0xA000: case 0xE000: bank_index = 2; break;
        case 0x6800: case 0x2800: case 0xA800: case 0xE800: bank_index = 3; break;
        case 0x7000: case 0x3000: case 0xB000: case 0xF000: bank_index = 4; break;
        case 0x7800: case 0x3800: case 0xB800: case 0xF800: bank_index = 5; break;
    }

    if (bank_index < 6u)
    {
        if (addr & 0x01u)
            regs[bank_index] = (regs[bank_index] & 0x00FFu) | ((uint16_t)data << 8);
        else
            regs[bank_index] = (regs[bank_index] & 0xFF00u) | data;
        regs[bank_index] &= 0x0FFFu;
    }
}

// NEO16 write handler (16-bit bank registers, 12-bit segment)
static inline void __not_in_flash_func(handle_neo16_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint16_t *regs = ((bank16_ctx_t *)ctx)->bank_regs;
    uint16_t base_addr = addr & 0xF800u;
    uint8_t bank_index = 3;

    switch (base_addr)
    {
        case 0x5000: case 0x1000: case 0x9000: case 0xD000: bank_index = 0; break;
        case 0x6000: case 0x2000: case 0xA000: case 0xE000: bank_index = 1; break;
        case 0x7000: case 0x3000: case 0xB000: case 0xF000: bank_index = 2; break;
    }

    if (bank_index < 3u)
    {
        if (addr & 0x01u)
            regs[bank_index] = (regs[bank_index] & 0x00FFu) | ((uint16_t)data << 8);
        else
            regs[bank_index] = (regs[bank_index] & 0xFF00u) | data;
        regs[bank_index] &= 0x0FFFu;
    }
}

// -----------------------------------------------------------------------
// Generic banked ROM loop (8KB banks, 8-bit bank registers)
// -----------------------------------------------------------------------
// Services both read and write events from the PIO state machines.
// On each iteration:
//   1. Drain pending write events from the write captor FIFO
//   2. Get the next read address from the read responder FIFO (blocking)
//   3. Drain any writes that arrived during the wait
//   4. Look up data using bank registers and respond
static void __no_inline_not_in_flash_func(banked8_loop)(
    const uint8_t *rom_base,
    uint32_t available_length,
    uint8_t *bank_regs,
    void (*write_handler)(uint16_t, uint8_t, void *))
{
    bank8_ctx_t ctx = { .bank_regs = bank_regs };

    while (true)
    {
        pio_drain_writes(write_handler, &ctx);

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        pio_drain_writes(write_handler, &ctx);

        bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint32_t rel = ((uint32_t)bank_regs[(addr - 0x4000u) >> 13] * 0x2000u) + (addr & 0x1FFFu);
            if (available_length == 0u || rel < available_length)
                data = read_rom_byte(rom_base, rel);
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// -----------------------------------------------------------------------
// loadrom_plain32 - Plain 16/32KB ROM (no mapper)
// -----------------------------------------------------------------------
// 32KB ROMs occupy 0x4000-0xBFFF. 16KB ROMs occupy 0x4000-0x7FFF.
// No bank switching; pure address-to-data lookup.
void __no_inline_not_in_flash_func(loadrom_plain32)(uint32_t offset, bool cache_enable)
{
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 32768u, &rom_base, &available_length);

    msx_pio_bus_init();

    while (true)
    {
        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
        uint8_t data = in_window ? rom_base[addr - 0x4000u] : 0xFFu;

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// -----------------------------------------------------------------------
// loadrom_linear48 - 48KB Linear0 ROM (no mapper)
// -----------------------------------------------------------------------
// Three pages: 0x0000-0x3FFF, 0x4000-0x7FFF, 0x8000-0xBFFF
void __no_inline_not_in_flash_func(loadrom_linear48)(uint32_t offset, bool cache_enable)
{
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 49152u, &rom_base, &available_length);

    msx_pio_bus_init();

    while (true)
    {
        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        bool in_window = (addr <= 0xBFFFu);
        uint8_t data = in_window ? rom_base[addr] : 0xFFu;

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// -----------------------------------------------------------------------
// loadrom_konamiscc - Konami SCC mapper
// -----------------------------------------------------------------------
// 8KB banks: 4000-5FFF, 6000-7FFF, 8000-9FFF, A000-BFFF
// Switch: 5000-57FF→bank0, 7000-77FF→bank1, 9000-97FF→bank2, B000-B7FF→bank3
void __no_inline_not_in_flash_func(loadrom_konamiscc)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    msx_pio_bus_init();
    banked8_loop(rom_base, available_length, bank_registers, handle_konamiscc_write);
}

// -----------------------------------------------------------------------
// loadrom_konami - Konami (without SCC) mapper
// -----------------------------------------------------------------------
// 8KB banks: 4000-5FFF, 6000-7FFF, 8000-9FFF, A000-BFFF
// Switch: bank0 fixed, 6000-67FF→bank1, 8000-87FF→bank2, A000-A7FF→bank3
void __no_inline_not_in_flash_func(loadrom_konami)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    msx_pio_bus_init();
    banked8_loop(rom_base, available_length, bank_registers, handle_konami_write);
}

// -----------------------------------------------------------------------
// Sunrise IDE write handler (cReg mapper at 0x4104 + IDE registers)
// -----------------------------------------------------------------------
typedef struct {
    sunrise_ide_t *ide;
} sunrise_ctx_t;

static inline void __not_in_flash_func(handle_sunrise_write)(uint16_t addr, uint8_t data, void *ctx)
{
    sunrise_ctx_t *sctx = (sunrise_ctx_t *)ctx;

    // IDE register / control writes (0x4104, 0x7C00-0x7DFF, 0x7E00-0x7EFF)
    if (addr >= 0x4000u && addr <= 0x7FFFu)
    {
        sunrise_ide_handle_write(sctx->ide, addr, data);
    }
}

// -----------------------------------------------------------------------
// loadrom_sunrise - Sunrise IDE Nextor ROM with emulated IDE interface
// -----------------------------------------------------------------------
// Uses the Sunrise IDE mapper: a single 16KB ROM window at 0x4000-0x7FFF
// with page selection via the control register at 0x4104 (bits 7:5 = page).
// No ROM at 0x8000-0xBFFF — the MSX provides its own RAM there.
// IDE register overlay when bit 0 of the control register is set:
//   - 0x7C00-0x7DFF = 16-bit data register (low/high byte latch)
//   - 0x7E00-0x7EFF = ATA task-file registers (mirrored every 16 bytes)
//   - 0x7F00-0x7FFF = ROM data (excluded from IDE space)
// ATA commands are translated to USB MSC operations on Core 1.
void __no_inline_not_in_flash_func(loadrom_sunrise)(uint32_t offset, bool cache_enable)
{
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    // Initialise Sunrise IDE state
    static sunrise_ide_t ide;
    sunrise_ide_init(&ide);

    // Share IDE context with Core 1 and launch USB host task
    sunrise_usb_set_ide_ctx(&ide);
    multicore_launch_core1(sunrise_usb_task);

    // Initialise PIO bus engine
    msx_pio_bus_init();

    sunrise_ctx_t ctx = { .ide = &ide };

    // Main loop: service PIO read/write events
    //
    // Unlike other mapper loops, the Sunrise IDE requires continuous write
    // draining.  Each ATA command involves a burst of 8-9 writes (bank
    // switch + IDE_ON + 6 task-file registers + command) with no
    // intervening reads.  The PIO write SM FIFO is 8 entries deep
    // (joined).  If we block on the read FIFO without draining writes,
    // the FIFO can overflow and the SM stalls, silently dropping writes.
    // A lost command or LBA register write causes data corruption.
    //
    // The fix: poll both FIFOs in a tight loop so writes are drained
    // continuously — even while waiting for the next read event.
    while (true)
    {
        uint16_t addr;

        // Poll: drain write FIFO while waiting for a read event
        while (true)
        {
            pio_drain_writes(handle_sunrise_write, &ctx);
            if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
            {
                addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
                break;
            }
        }

        // Drain any writes that arrived alongside the read
        pio_drain_writes(handle_sunrise_write, &ctx);

        // Sunrise IDE ROM is only at 0x4000-0x7FFF (one 16KB window)
        bool in_window = (addr >= 0x4000u) && (addr <= 0x7FFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            // Check if IDE intercepts this read (0x7C00-0x7EFF when enabled)
            uint8_t ide_data;
            if (sunrise_ide_handle_read(&ide, addr, &ide_data))
            {
                data = ide_data;
            }
            else
            {
                // Sunrise mapper: page selected by ide.segment (cReg bits 7:5)
                uint8_t seg = ide.segment;
                uint32_t rel = ((uint32_t)seg << 14) + (addr & 0x3FFFu);
                if (available_length == 0u || rel < available_length)
                    data = read_rom_byte(rom_base, rel);
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// -----------------------------------------------------------------------
// loadrom_ascii8 - ASCII8 mapper
// -----------------------------------------------------------------------
// 8KB banks: 4000-5FFF, 6000-7FFF, 8000-9FFF, A000-BFFF
// Switch: 6000-67FF→bank0, 6800-6FFF→bank1, 7000-77FF→bank2, 7800-7FFF→bank3
void __no_inline_not_in_flash_func(loadrom_ascii8)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    msx_pio_bus_init();
    banked8_loop(rom_base, available_length, bank_registers, handle_ascii8_write);
}

// -----------------------------------------------------------------------
// loadrom_ascii16 - ASCII16 mapper
// -----------------------------------------------------------------------
// 16KB banks: 4000-7FFF, 8000-BFFF
// Switch: 6000-67FF→bank0, 7000-77FF→bank1
void __no_inline_not_in_flash_func(loadrom_ascii16)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[2] = {0, 1};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    msx_pio_bus_init();

    bank8_ctx_t ctx = { .bank_regs = bank_registers };

    while (true)
    {
        pio_drain_writes(handle_ascii16_write, &ctx);

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        pio_drain_writes(handle_ascii16_write, &ctx);

        bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint8_t bank = (addr >> 15) & 1;
            uint32_t rel = ((uint32_t)bank_registers[bank] << 14) + (addr & 0x3FFFu);
            if (available_length == 0u || rel < available_length)
                data = read_rom_byte(rom_base, rel);
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// -----------------------------------------------------------------------
// loadrom_neo8 - NEO8 mapper (8KB segments, 16-bit bank registers)
// -----------------------------------------------------------------------
// 6 banks of 8KB covering 0x0000-0xBFFF
void __no_inline_not_in_flash_func(loadrom_neo8)(uint32_t offset)
{
    uint16_t bank_registers[6] = {0};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, false, 0u, &rom_base, &available_length);

    msx_pio_bus_init();

    bank16_ctx_t ctx = { .bank_regs = bank_registers };

    while (true)
    {
        pio_drain_writes(handle_neo8_write, &ctx);

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        pio_drain_writes(handle_neo8_write, &ctx);

        bool in_window = (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint8_t bank_index = addr >> 13;
            if (bank_index < 6u)
            {
                uint32_t segment = bank_registers[bank_index] & 0x0FFFu;
                uint32_t rel = (segment << 13) + (addr & 0x1FFFu);
                if (available_length == 0u || rel < available_length)
                    data = read_rom_byte(rom_base, rel);
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// -----------------------------------------------------------------------
// loadrom_neo16 - NEO16 mapper (16KB segments, 16-bit bank registers)
// -----------------------------------------------------------------------
// 3 banks of 16KB covering 0x0000-0xBFFF
void __no_inline_not_in_flash_func(loadrom_neo16)(uint32_t offset)
{
    uint16_t bank_registers[3] = {0};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, false, 0u, &rom_base, &available_length);

    msx_pio_bus_init();

    bank16_ctx_t ctx = { .bank_regs = bank_registers };

    while (true)
    {
        pio_drain_writes(handle_neo16_write, &ctx);

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        pio_drain_writes(handle_neo16_write, &ctx);

        bool in_window = (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint8_t bank_index = addr >> 14;
            if (bank_index < 3u)
            {
                uint32_t segment = bank_registers[bank_index] & 0x0FFFu;
                uint32_t rel = (segment << 14) + (addr & 0x3FFFu);
                if (available_length == 0u || rel < available_length)
                    data = read_rom_byte(rom_base, rel);
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// -----------------------------------------------------------------------
// loadrom_sunrise_mapper - Sunrise IDE Nextor + 192KB Memory Mapper (test)
// -----------------------------------------------------------------------
// Implements expanded slot with two sub-slots:
//   Sub-slot 0: Nextor ROM (Sunrise IDE) — 16KB window at 0x4000-0x7FFF
//   Sub-slot 1: 192KB Memory Mapper RAM — all 4 pages (0x0000-0xFFFF)
//
// The sub-slot register at 0xFFFF controls which sub-slot is selected
// for each 16KB page (bits 1:0 = page 0, bits 3:2 = page 1, etc.).
// Reading 0xFFFF returns the bitwise NOT of the sub-slot register.
//
// Memory mapper page registers (I/O ports FC-FF) select which 16KB page
// of mapper RAM appears in each address range:
//   Port FC → page at 0x0000-0x3FFF
//   Port FD → page at 0x4000-0x7FFF
//   Port FE → page at 0x8000-0xBFFF
//   Port FF → page at 0xC000-0xFFFF
//
// Reset values per BIOS convention: FC=3, FD=2, FE=1, FF=0
//
// The mapper RAM is 192KB = 12 pages of 16KB. Page registers are treated
// as 8-bit values and normalized to 0..11 when accessing RAM.
void __no_inline_not_in_flash_func(loadrom_sunrise_mapper)(uint32_t offset, bool cache_enable)
{
    (void)cache_enable;

    // Mapper mode: disable ROM cache and reserve full SRAM for mapper RAM.
    rom_cache_capacity = 0u;

    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, false, 0u, &rom_base, &available_length);

    // Initialise Sunrise IDE state
    static sunrise_ide_t ide;
    sunrise_ide_init(&ide);

    // Share IDE context with Core 1 and launch USB host task
    sunrise_usb_set_ide_ctx(&ide);
    multicore_launch_core1(sunrise_usb_task);

    // Initialise mapper page registers (BIOS convention)
    uint8_t mapper_reg[4] = { 3, 2, 1, 0 };  // FC, FD, FE, FF

    // Sub-slot register: bits [1:0]=page0, [3:2]=page1, [5:4]=page2, [7:6]=page3
    // Boot mapping for Nextor MAP_INIT compatibility:
    //   page0 -> sub-slot0
    //   page1 -> sub-slot0 (Sunrise ROM/IDE)
    //   page2 -> sub-slot1 (mapper RAM, probe target at 0x8000)
    //   page3 -> sub-slot0
    // Value: 0b00010000 = 0x10
    uint8_t subslot_reg = 0x10;

    // Clear mapper RAM
    memset(mapper_ram, 0xFF, MAPPER_SIZE);

    // Initialise PIO bus engines
    msx_pio_bus_init();
    msx_pio_io_bus_init();

    sunrise_ctx_t ctx = { .ide = &ide };

    // Main loop: service memory reads/writes and I/O reads/writes
    //
    // We must poll all four FIFOs continuously:
    //   PIO0 SM0 RX (memory read) — respond with data
    //   PIO0 SM1 RX (memory write) — handle Sunrise IDE writes + mapper RAM writes + sub-slot reg
    //   PIO1 SM0 RX (I/O read) — respond with mapper page register values
    //   PIO1 SM1 RX (I/O write) — update mapper page registers
    while (true)
    {
        // --- Drain memory writes (Sunrise IDE + mapper RAM + sub-slot) ---
        {
            uint16_t waddr;
            uint8_t wdata;
            while (pio_try_get_write(&waddr, &wdata))
            {
                // Sub-slot register write at 0xFFFF
                if (waddr == 0xFFFFu)
                {
                    subslot_reg = wdata;
                }
                // Determine sub-slot for this write address
                else
                {
                    uint8_t page = (waddr >> 14) & 0x03u;
                    uint8_t active_subslot = (subslot_reg >> (page * 2)) & 0x03u;

                    if (active_subslot == 0)
                    {
                        // Sub-slot 0: Sunrise IDE (0x4000-0x7FFF only)
                        if (waddr >= 0x4000u && waddr <= 0x7FFFu)
                        {
                            sunrise_ide_handle_write(&ide, waddr, wdata);
                        }
                    }
                    else if (active_subslot == 1)
                    {
                        // Sub-slot 1: Memory mapper RAM — write to mapped page
                        uint8_t mapper_page = mapper_page_from_reg(mapper_reg[page]);
                        uint32_t mapper_offset = ((uint32_t)mapper_page << 14) | (waddr & 0x3FFFu);
                        mapper_ram[mapper_offset] = wdata;
                    }
                    // Sub-slots 2 and 3: unused, writes are ignored
                }
            }
        }

        // --- Drain I/O writes (mapper page registers FC-FF) ---
        {
            uint16_t io_addr;
            uint8_t io_data;
            while (pio_try_get_io_write(&io_addr, &io_data))
            {
                // Nextor MAP_INIT tests mapper RAM in page 2 (0x8000-0xBFFF)
                // while switching mapper segments with OUT (0xFC-0xFF),A.
                // Expose mapper ports only when page 2 is mapped to mapper
                // sub-slot 1, to emulate slot-dependent visibility.
                uint8_t page2_subslot = (subslot_reg >> 4) & 0x03u;
                if (page2_subslot != 1u)
                    continue;

                uint8_t port = io_addr & 0xFFu;
                if (port >= 0xFCu && port <= 0xFFu)
                {
                    mapper_reg[port - 0xFCu] = io_data & 0x0Fu;
                }
            }
        }

        // --- Handle I/O reads (mapper page registers FC-FF) ---
        {
            uint16_t io_addr;
            while (pio_try_get_io_read(&io_addr))
            {
                uint8_t page2_subslot = (subslot_reg >> 4) & 0x03u;
                uint8_t port = io_addr & 0xFFu;
                bool in_window = false;
                uint8_t data = 0xFFu;

                if (page2_subslot == 1u && port >= 0xFCu && port <= 0xFFu)
                {
                    in_window = true;
                    data = (uint8_t)(0xF0u | (mapper_reg[port - 0xFCu] & 0x0Fu));
                }

                pio_sm_put_blocking(msx_io_bus.pio_read, msx_io_bus.sm_io_read, pio_build_token(in_window, data));
            }
        }

        // --- Handle memory reads ---
        if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
            uint8_t data = 0xFFu;
            bool in_window = false;

            // Sub-slot register read at 0xFFFF: return ~subslot_reg
            if (addr == 0xFFFFu)
            {
                // Only respond if page 3 (0xC000-0xFFFF) is in a sub-slot we own.
                // The MSX reads 0xFFFF to detect expanded slot; we always respond
                // because the cartridge owns the entire slot.
                in_window = true;
                data = ~subslot_reg;
            }
            else
            {
                uint8_t page = (addr >> 14) & 0x03u;
                uint8_t active_subslot = (subslot_reg >> (page * 2)) & 0x03u;

                if (active_subslot == 0)
                {
                    // Sub-slot 0: Nextor ROM (0x4000-0x7FFF only)
                    if (addr >= 0x4000u && addr <= 0x7FFFu)
                    {
                        in_window = true;

                        // Check if IDE intercepts this read
                        uint8_t ide_data;
                        if (sunrise_ide_handle_read(&ide, addr, &ide_data))
                        {
                            data = ide_data;
                        }
                        else
                        {
                            // Sunrise mapper: page selected by ide.segment
                            uint8_t seg = ide.segment;
                            uint32_t rel = ((uint32_t)seg << 14) + (addr & 0x3FFFu);
                            if (available_length == 0u || rel < available_length)
                                data = read_rom_byte(rom_base, rel);
                        }
                    }
                }
                else if (active_subslot == 1)
                {
                    // Sub-slot 1: Memory mapper RAM — all 4 pages
                    in_window = true;
                    uint8_t mapper_page = mapper_page_from_reg(mapper_reg[page]);
                    uint32_t mapper_offset = ((uint32_t)mapper_page << 14) | (addr & 0x3FFFu);
                    data = mapper_ram[mapper_offset];
                }
                // Sub-slots 2 and 3: unused, return 0xFF (not in window)
            }

            pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
        }
    }
}

// -----------------------------------------------------------------------
// Main program
// -----------------------------------------------------------------------
int __no_inline_not_in_flash_func(main)()
{
    // Set system clock to 250MHz for maximum headroom
    set_sys_clock_khz(250000, true);

    // Initialize GPIO
    setup_gpio();

    // Parse ROM header
    char rom_name[ROM_NAME_MAX];
    memcpy(rom_name, rom, ROM_NAME_MAX);
    uint8_t rom_type = rom[ROM_NAME_MAX];

    uint32_t rom_size;
    memcpy(&rom_size, rom + ROM_NAME_MAX + 1, sizeof(uint32_t));
    active_rom_size = rom_size;

    // Load the ROM based on the detected mapper type
    // 1 - 16KB ROM
    // 2 - 32KB ROM
    // 3 - Konami SCC ROM
    // 4 - 48KB Linear0 ROM
    // 5 - ASCII8 ROM
    // 6 - ASCII16 ROM
    // 7 - Konami (without SCC) ROM
    // 8 - NEO8 ROM
    // 9 - NEO16 ROM
    // 10 - Sunrise IDE Nextor ROM (Konami mapper)
    // 11 - Sunrise IDE Nextor ROM + 128KB Memory Mapper
    switch (rom_type) 
    {
        case 1:
        case 2:
            loadrom_plain32(ROM_RECORD_SIZE, true); 
            break;
        case 3:
            loadrom_konamiscc(ROM_RECORD_SIZE, true); 
            break;
        case 4:
            loadrom_linear48(ROM_RECORD_SIZE, true); 
            break;
        case 5:
            loadrom_ascii8(ROM_RECORD_SIZE, true); 
            break;
        case 6:
            loadrom_ascii16(ROM_RECORD_SIZE, true); 
            break;
        case 7:
            loadrom_konami(ROM_RECORD_SIZE, true); 
            break;
        case 8:
            loadrom_neo8(ROM_RECORD_SIZE);
            break;
        case 9:
            loadrom_neo16(ROM_RECORD_SIZE);
            break;
        case 10:
            loadrom_sunrise(ROM_RECORD_SIZE, true);
            break;
        case 11:
            loadrom_sunrise_mapper(ROM_RECORD_SIZE, true);
            break;
        default:
            break;
    }
    return 0;
}
