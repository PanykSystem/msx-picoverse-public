// MSX PICOVERSE PROJECT
// (c) 2025 Cristiano Goncalves
// The Retro Hacker
//
// multirom.c - This is the Raspberry Pico firmware that will be used to load ROMs into the MSX
//
// This firmware is responsible for loading the multirom menu and the ROMs selected by the user into the MSX. When flashed through the 
// multirom tool, it will be stored on the pico flash memory followed by the MSX MENU ROM (with the config) and all the ROMs processed by the 
// multirom tool. The sofware in this firmware will load the first 32KB ROM that contains the menu into the MSX and it will allow the user
// to select a ROM to be loaded into the MSX. The selected ROM will be loaded into the MSX and the MSX will be reseted to run the selected ROM.
//
// This work is licensed  under a "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
// License". https://creativecommons.org/licenses/by-nc-sa/4.0/

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/structs/watchdog.h"
#include "hardware/watchdog.h"
#include "hardware/regs/addressmap.h"
#include "hardware/regs/m0plus.h"

#include "multirom.h"
#include "sunrise_ide.h"
#include "msx_bus.pio.h"

// config area and buffer for the ROM data
#define ROM_NAME_MAX    50          // Maximum size of the ROM name
#define MAX_ROM_RECORDS 128         // Maximum ROM files supported
#define ROM_RECORD_SIZE (ROM_NAME_MAX + 1 + (sizeof(uint32_t) * 2)) // Name + mapper + size + offset
#define MONITOR_ADDR    (0x8000 + (ROM_RECORD_SIZE * MAX_ROM_RECORDS) + 1) // Monitor ROM address within image (currently 0x9D81)
#define CACHE_SIZE      196608     // 192KB cache size for ROM data

// This symbol marks the end of the main program in flash.
// Custom data starts right after it
extern unsigned char __flash_binary_end;

// SRAM — shared between ROM cache and mapper RAM.
// Normal modes use the full 192KB as ROM cache.
// Sunrise+Mapper mode uses the full 192KB as mapper RAM (no ROM cache).
static union {
    uint8_t rom_sram[CACHE_SIZE];
    struct {
        uint8_t mapper_ram[MAPPER_SIZE];
    } mapper;
} sram_pool;

#define rom_sram    sram_pool.rom_sram
#define mapper_ram  sram_pool.mapper.mapper_ram

static uint32_t active_rom_size = 0;

//pointer to the custom data
const uint8_t *rom = (const uint8_t *)&__flash_binary_end;

// Structure to represent a ROM record
// The ROM record will contain the name of the ROM, the mapper code, the size of the ROM and the offset in the flash memory
// Name: ROM_NAME_MAX bytes
// Mapper: 1 byte
// Size: 4 bytes
// Offset: 4 bytes
typedef struct {
    char Name[ROM_NAME_MAX];
    unsigned char Mapper;
    unsigned long Size;
    unsigned long Offset;
} ROMRecord;

ROMRecord records[MAX_ROM_RECORDS]; // Array to store the ROM records

typedef struct {
    PIO pio;
    uint sm_read;
    uint sm_write;
    uint offset_read;
    uint offset_write;
} msx_pio_bus_t;

static msx_pio_bus_t msx_bus;
static uint32_t rom_cached_size = 0;
static bool msx_bus_programs_loaded = false;

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
static bool msx_io_bus_programs_loaded = false;

// Initialize GPIO pins
static inline void setup_gpio()
{
    for (uint pin = PIN_A0; pin <= PIN_A15; ++pin)
    {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
    }

    for (uint pin = PIN_D0; pin <= PIN_D7; ++pin)
    {
        gpio_init(pin);
    }

    gpio_init(PIN_RD);      gpio_set_dir(PIN_RD, GPIO_IN);
    gpio_init(PIN_WR);      gpio_set_dir(PIN_WR, GPIO_IN);
    gpio_init(PIN_IORQ);    gpio_set_dir(PIN_IORQ, GPIO_IN);
    gpio_init(PIN_SLTSL);   gpio_set_dir(PIN_SLTSL, GPIO_IN);
    gpio_init(PIN_BUSSDIR); gpio_set_dir(PIN_BUSSDIR, GPIO_IN);
}

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
        uint32_t bytes_to_cache = (available_length > sizeof(rom_sram))
                                  ? sizeof(rom_sram)
                                  : available_length;

        gpio_init(PIN_WAIT);
        gpio_set_dir(PIN_WAIT, GPIO_OUT);
        gpio_put(PIN_WAIT, 0);

        int dma_chan = dma_claim_unused_channel(true);
        dma_channel_config dma_cfg = dma_channel_get_default_config(dma_chan);
        channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
        channel_config_set_read_increment(&dma_cfg, true);
        channel_config_set_write_increment(&dma_cfg, true);
        dma_channel_configure(dma_chan, &dma_cfg,
            rom_sram,
            rom_base,
            bytes_to_cache,
            true);
        dma_channel_wait_for_finish_blocking(dma_chan);
        dma_channel_unclaim(dma_chan);
        gpio_set_dir(PIN_WAIT, GPIO_IN);

        rom_cached_size = bytes_to_cache;

        if (available_length <= sizeof(rom_sram))
        {
            rom_base = rom_sram;
        }
    }
    else
    {
        rom_cached_size = 0;
    }

    *rom_base_out = rom_base;
    *available_length_out = available_length;
}

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
    sm_config_set_set_pins(&cfg_read, PIN_WAIT, 1);
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

    pio_sm_set_pins_with_mask(msx_bus.pio, msx_bus.sm_read, 0u, (1u << PIN_WAIT));
    pio_gpio_init(msx_bus.pio, PIN_WAIT);
    pio_sm_set_consecutive_pindirs(msx_bus.pio, msx_bus.sm_read, PIN_WAIT, 1, false);

    for (uint pin = PIN_D0; pin <= PIN_D7; ++pin)
    {
        pio_gpio_init(msx_bus.pio, pin);
    }
    pio_sm_set_consecutive_pindirs(msx_bus.pio, msx_bus.sm_read, PIN_D0, 8, false);
    pio_sm_set_consecutive_pindirs(msx_bus.pio, msx_bus.sm_write, PIN_D0, 8, false);

    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_read, true);
    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_write, true);
}

// -----------------------------------------------------------------------
// PIO I/O bus initialisation (for memory mapper port access on PIO1)
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

static inline uint8_t __not_in_flash_func(read_rom_byte)(const uint8_t *rom_base, uint32_t rel)
{
    return (rel < rom_cached_size) ? rom_sram[rel] : rom_base[rel];
}

static inline uint16_t __not_in_flash_func(pio_build_token)(bool drive, uint8_t data)
{
    uint8_t dir_mask = drive ? 0xFFu : 0x00u;
    return (uint16_t)data | ((uint16_t)dir_mask << 8);
}

static inline bool __not_in_flash_func(pio_try_get_write)(uint16_t *addr_out, uint8_t *data_out)
{
    if (pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_write))
    {
        return false;
    }

    uint32_t sample = pio_sm_get(msx_bus.pio, msx_bus.sm_write);
    *addr_out = (uint16_t)(sample & 0xFFFFu);
    *data_out = (uint8_t)((sample >> 16) & 0xFFu);
    return true;
}

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
static inline uint8_t __not_in_flash_func(mapper_page_from_reg)(uint8_t reg)
{
    return (uint8_t)(reg % MAPPER_PAGES);
}

// Try to consume an I/O read event from the I/O read responder FIFO (PIO1).
static inline bool __not_in_flash_func(pio_try_get_io_read)(uint16_t *addr_out)
{
    if (pio_sm_is_rx_fifo_empty(msx_io_bus.pio_read, msx_io_bus.sm_io_read))
        return false;

    *addr_out = (uint16_t)pio_sm_get(msx_io_bus.pio_read, msx_io_bus.sm_io_read);
    return true;
}

typedef struct {
    uint8_t *bank_regs;
} bank8_ctx_t;

typedef struct {
    uint16_t *bank_regs;
} bank16_ctx_t;

typedef struct {
    uint8_t rom_index;
    bool rom_selected;
} menu_ctx_t;

static inline void __not_in_flash_func(handle_konamiscc_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint8_t *regs = ((bank8_ctx_t *)ctx)->bank_regs;
    if      (addr >= 0x5000u && addr <= 0x57FFu) regs[0] = data;
    else if (addr >= 0x7000u && addr <= 0x77FFu) regs[1] = data;
    else if (addr >= 0x9000u && addr <= 0x97FFu) regs[2] = data;
    else if (addr >= 0xB000u && addr <= 0xB7FFu) regs[3] = data;
}

static inline void __not_in_flash_func(handle_konami_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint8_t *regs = ((bank8_ctx_t *)ctx)->bank_regs;
    if      (addr >= 0x6000u && addr <= 0x67FFu) regs[1] = data;
    else if (addr >= 0x8000u && addr <= 0x87FFu) regs[2] = data;
    else if (addr >= 0xA000u && addr <= 0xA7FFu) regs[3] = data;
}

static inline void __not_in_flash_func(handle_ascii8_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint8_t *regs = ((bank8_ctx_t *)ctx)->bank_regs;
    if      (addr >= 0x6000u && addr <= 0x67FFu) regs[0] = data;
    else if (addr >= 0x6800u && addr <= 0x6FFFu) regs[1] = data;
    else if (addr >= 0x7000u && addr <= 0x77FFu) regs[2] = data;
    else if (addr >= 0x7800u && addr <= 0x7FFFu) regs[3] = data;
}

static inline void __not_in_flash_func(handle_ascii16_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint8_t *regs = ((bank8_ctx_t *)ctx)->bank_regs;
    if      (addr >= 0x6000u && addr <= 0x67FFu) regs[0] = data;
    else if (addr >= 0x7000u && addr <= 0x77FFu) regs[1] = data;
}

// ASCII16-X write handler
// Two 12-bit bank registers with mirrored write windows:
//   Page 1 register: 2000/6000/A000/E000 - 2FFF/6FFF/AFFF/EFFF
//   Page 2 register: 3000/7000/B000/F000 - 3FFF/7FFF/BFFF/FFFF
// Bank number format: bits 0-7 from data bus, bits 8-11 from A8-A11.
static inline void __not_in_flash_func(handle_ascii16x_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint16_t *regs = ((bank16_ctx_t *)ctx)->bank_regs;
    uint8_t high_nibble = (uint8_t)((addr >> 8) & 0x0Fu);
    uint16_t bank = ((uint16_t)high_nibble << 8) | data;

    switch (addr & 0xF000u)
    {
        case 0x2000u:
        case 0x6000u:
        case 0xA000u:
        case 0xE000u:
            regs[0] = bank;
            break;

        case 0x3000u:
        case 0x7000u:
        case 0xB000u:
        case 0xF000u:
            regs[1] = bank;
            break;
    }
}

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

static inline void __not_in_flash_func(handle_menu_write)(uint16_t addr, uint8_t data, void *ctx)
{
    menu_ctx_t *menu = (menu_ctx_t *)ctx;
    if (addr == MONITOR_ADDR)
    {
        menu->rom_index = data;
        menu->rom_selected = true;
    }
}

// -----------------------------------------------------------------------
// Generic mapper-aware LRU bank cache
// -----------------------------------------------------------------------
// Divides the 192KB SRAM pool into fixed-size slots matching the mapper's
// bank size.  Two configurations are used:
//
//   16KB slots -> 12 slots  (ASCII16-X, NEO16)
//    8KB slots -> 24 slots  (NEO8)
//
// LRU eviction ensures pinned banks (those mapped to active pages) are
// never evicted.  On a bank-register write the handler ensures the new
// bank is resident; flash is only accessed on a cache miss.  The read
// path then always hits SRAM, keeping /WAIT hold time minimal.

#define BANK_CACHE_MAX_SLOTS 24   // max slots (192KB / 8KB)
#define BANK_CACHE_MAX_PINS   6   // max simultaneously pinned pages
#define BANK_EMPTY           0xFFFFu

typedef struct {
    uint16_t      slot_bank[BANK_CACHE_MAX_SLOTS]; // bank loaded per slot
    uint8_t       slot_lru[BANK_CACHE_MAX_SLOTS];  // LRU counter (0 = MRU)
    int8_t        page_slot[BANK_CACHE_MAX_PINS];  // slot pinned per page
    const uint8_t *flash_base;
    uint32_t      rom_length;
    uint8_t       num_slots;    // actual slot count
    uint8_t       num_pins;     // active pinned pages
    uint16_t      slot_size;    // bytes per slot (8192 or 16384)
    uint8_t       slot_shift;   // log2(slot_size): 13 or 14
} bank_cache_t;

static inline void __not_in_flash_func(bcache_init)(
    bank_cache_t *c, uint16_t slot_size, uint8_t num_pins,
    const uint8_t *flash_base, uint32_t rom_length)
{
    uint8_t num_slots = (uint8_t)(CACHE_SIZE / slot_size);
    c->num_slots  = num_slots;
    c->num_pins   = num_pins;
    c->slot_size  = slot_size;
    c->slot_shift = (slot_size == 8192u) ? 13 : 14;
    c->flash_base = flash_base;
    c->rom_length = rom_length;
    for (int i = 0; i < BANK_CACHE_MAX_SLOTS; i++)
    {
        c->slot_bank[i] = BANK_EMPTY;
        c->slot_lru[i]  = (uint8_t)i;
    }
    for (int i = 0; i < BANK_CACHE_MAX_PINS; i++)
        c->page_slot[i] = -1;
}

// Promote a slot to MRU.
static inline void __not_in_flash_func(bcache_touch)(bank_cache_t *c, int8_t slot)
{
    uint8_t prev = c->slot_lru[slot];
    if (prev == 0) return;
    uint8_t n = c->num_slots;
    for (uint8_t i = 0; i < n; i++)
        if (c->slot_lru[i] < prev) c->slot_lru[i]++;
    c->slot_lru[slot] = 0;
}

// Find the slot holding a given bank, or -1.
static inline int8_t __not_in_flash_func(bcache_find)(bank_cache_t *c, uint16_t bank)
{
    uint8_t n = c->num_slots;
    for (uint8_t i = 0; i < n; i++)
        if (c->slot_bank[i] == bank) return (int8_t)i;
    return -1;
}

// Pick a victim slot (highest LRU counter, not pinned).
static inline int8_t __not_in_flash_func(bcache_evict)(bank_cache_t *c)
{
    int8_t best = -1;
    uint8_t best_lru = 0;
    uint8_t n = c->num_slots;
    uint8_t np = c->num_pins;
    for (uint8_t i = 0; i < n; i++)
    {
        bool pinned = false;
        for (uint8_t p = 0; p < np; p++)
            if ((int8_t)i == c->page_slot[p]) { pinned = true; break; }
        if (pinned) continue;
        if (best < 0 || c->slot_lru[i] >= best_lru)
        {
            best = (int8_t)i;
            best_lru = c->slot_lru[i];
        }
    }
    return best;
}

// Ensure a bank is resident; returns its slot index.
static inline int8_t __not_in_flash_func(bcache_ensure)(bank_cache_t *c, uint16_t bank)
{
    // Normalize bank number modulo ROM size so out-of-range banks wrap
    // around, matching real hardware mirror behaviour.
    if (c->rom_length > 0u)
    {
        uint16_t total_banks = (uint16_t)(c->rom_length >> c->slot_shift);
        if (total_banks > 0u)
            bank = bank % total_banks;
    }

    int8_t slot = bcache_find(c, bank);
    if (slot >= 0) { bcache_touch(c, slot); return slot; }

    slot = bcache_evict(c);
    uint32_t src_off = (uint32_t)(bank & 0x0FFFu) << c->slot_shift;
    uint8_t *dst = &rom_sram[(uint32_t)slot * c->slot_size];
    uint16_t ss = c->slot_size;

    if (c->rom_length == 0u || src_off < c->rom_length)
    {
        uint32_t n = ss;
        if (c->rom_length > 0u && src_off + n > c->rom_length)
            n = c->rom_length - src_off;
        memcpy(dst, c->flash_base + src_off, n);
        if (n < ss) memset(dst + n, 0xFFu, ss - n);
    }
    else
    {
        memset(dst, 0xFFu, ss);
    }

    c->slot_bank[slot] = bank;
    bcache_touch(c, slot);
    return slot;
}

// Pre-fill the cache with the first N banks of the ROM.
static inline void __not_in_flash_func(bcache_prefill)(bank_cache_t *c)
{
    uint16_t max_banks = c->num_slots;
    if (c->rom_length > 0u)
    {
        uint16_t rom_banks = (uint16_t)((c->rom_length + c->slot_size - 1u) / c->slot_size);
        if (rom_banks < max_banks) max_banks = rom_banks;
    }
    for (uint16_t b = 0; b < max_banks; b++)
        bcache_ensure(c, b);
}

// -----------------------------------------------------------------------
// AMD-compatible flash command emulation for ASCII16-X
// -----------------------------------------------------------------------
// Real ASCII16-X cartridges use AMD/SST-compatible flash chips.  Some
// ROMs detect the cartridge by issuing a flash byte-program command and
// verifying the write.  Others use flash for persistent save data.
//
// The state machine below intercepts the standard AMD unlock + command
// sequences and emulates byte-program and sector-erase by modifying the
// SRAM bank cache directly.

typedef enum {
    FLASH_IDLE = 0,
    FLASH_UNLOCK1,
    FLASH_UNLOCK2,
    FLASH_BYTE_PGM,
    FLASH_ERASE_SETUP,
    FLASH_ERASE_UNLOCK1,
    FLASH_ERASE_UNLOCK2,
} flash_cmd_state_t;

typedef struct {
    uint16_t          bank_regs[2];
    bank_cache_t      cache;
    flash_cmd_state_t flash_state;
} ascii16x_state_t;

static inline void __not_in_flash_func(flash_process_write)(
    ascii16x_state_t *st, uint16_t addr, uint8_t data)
{
    uint16_t cmd_addr = addr & 0x0FFFu;

    switch (st->flash_state)
    {
        case FLASH_IDLE:
            if (cmd_addr == 0x0AAAu && data == 0xAAu)
                st->flash_state = FLASH_UNLOCK1;
            else if (data == 0xF0u)
                st->flash_state = FLASH_IDLE;
            break;

        case FLASH_UNLOCK1:
            if (cmd_addr == 0x0555u && data == 0x55u)
                st->flash_state = FLASH_UNLOCK2;
            else
                st->flash_state = FLASH_IDLE;
            break;

        case FLASH_UNLOCK2:
            if (cmd_addr == 0x0AAAu)
            {
                switch (data)
                {
                    case 0xA0u: st->flash_state = FLASH_BYTE_PGM;    break;
                    case 0x80u: st->flash_state = FLASH_ERASE_SETUP;  break;
                    default:    st->flash_state = FLASH_IDLE;         break;
                }
            }
            else
                st->flash_state = FLASH_IDLE;
            break;

        case FLASH_BYTE_PGM:
        {
            uint8_t page_idx = ((addr >> 14) & 0x01u) ? 0u : 1u;
            int8_t slot = st->cache.page_slot[page_idx];
            if (slot >= 0)
            {
                uint32_t off = (uint32_t)slot * st->cache.slot_size
                             + (addr & 0x3FFFu);
                rom_sram[off] &= data;
            }
            st->flash_state = FLASH_IDLE;
            break;
        }

        case FLASH_ERASE_SETUP:
            if (cmd_addr == 0x0AAAu && data == 0xAAu)
                st->flash_state = FLASH_ERASE_UNLOCK1;
            else
                st->flash_state = FLASH_IDLE;
            break;

        case FLASH_ERASE_UNLOCK1:
            if (cmd_addr == 0x0555u && data == 0x55u)
                st->flash_state = FLASH_ERASE_UNLOCK2;
            else
                st->flash_state = FLASH_IDLE;
            break;

        case FLASH_ERASE_UNLOCK2:
            if (data == 0x30u)
            {
                uint8_t page_idx = ((addr >> 14) & 0x01u) ? 0u : 1u;
                int8_t slot = st->cache.page_slot[page_idx];
                if (slot >= 0)
                    memset(&rom_sram[(uint32_t)slot * st->cache.slot_size],
                           0xFFu, st->cache.slot_size);
            }
            st->flash_state = FLASH_IDLE;
            break;
    }
}

// Cached ASCII16-X write handler with AMD flash emulation.
static inline void __not_in_flash_func(handle_ascii16x_write_cached)(uint16_t addr, uint8_t data, void *ctx)
{
    ascii16x_state_t *st = (ascii16x_state_t *)ctx;

    flash_process_write(st, addr, data);

    uint8_t high_nibble = (uint8_t)((addr >> 8) & 0x0Fu);
    uint16_t bank = ((uint16_t)high_nibble << 8) | data;
    uint8_t page;

    switch (addr & 0xF000u)
    {
        case 0x2000u: case 0x6000u: case 0xA000u: case 0xE000u: page = 0; break;
        case 0x3000u: case 0x7000u: case 0xB000u: case 0xF000u: page = 1; break;
        default: return;
    }

    st->bank_regs[page] = bank;
    st->cache.page_slot[page] = bcache_ensure(&st->cache, bank);
}

// -----------------------------------------------------------------------
// NEO8 / NEO16 cached state and write handlers
// -----------------------------------------------------------------------

typedef struct {
    uint16_t     bank_regs[6];   // NEO8: 6 registers, NEO16: 3 used
    bank_cache_t cache;
} neo_state_t;

// Cached NEO8 write handler.
static inline void __not_in_flash_func(handle_neo8_write_cached)(uint16_t addr, uint8_t data, void *ctx)
{
    neo_state_t *st = (neo_state_t *)ctx;
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
            st->bank_regs[bank_index] = (st->bank_regs[bank_index] & 0x00FFu) | ((uint16_t)data << 8);
        else
            st->bank_regs[bank_index] = (st->bank_regs[bank_index] & 0xFF00u) | data;
        st->bank_regs[bank_index] &= 0x0FFFu;
        st->cache.page_slot[bank_index] = bcache_ensure(&st->cache, st->bank_regs[bank_index]);
    }
}

// Cached NEO16 write handler.
static inline void __not_in_flash_func(handle_neo16_write_cached)(uint16_t addr, uint8_t data, void *ctx)
{
    neo_state_t *st = (neo_state_t *)ctx;
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
            st->bank_regs[bank_index] = (st->bank_regs[bank_index] & 0x00FFu) | ((uint16_t)data << 8);
        else
            st->bank_regs[bank_index] = (st->bank_regs[bank_index] & 0xFF00u) | data;
        st->bank_regs[bank_index] &= 0x0FFFu;
        st->cache.page_slot[bank_index] = bcache_ensure(&st->cache, st->bank_regs[bank_index]);
    }
}

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
            {
                data = read_rom_byte(rom_base, rel);
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// read_ulong - Read a 4-byte value from the memory area
// This function will read a 4-byte value from the memory area pointed by ptr and return the value as an unsigned long
// Parameters:
//   ptr - Pointer to the memory area to read the value from
// Returns:
//   The 4-byte value as an unsigned long 
unsigned long read_ulong(const unsigned char *ptr) {
    return (unsigned long)ptr[0] |
           ((unsigned long)ptr[1] << 8) |
           ((unsigned long)ptr[2] << 16) |
           ((unsigned long)ptr[3] << 24);
}

// isEndOfData - Check if the memory area is the end of the data
// This function will check if the memory area pointed by memory is the end of the data. The end of the data is defined by all bytes being 0xFF.
// Parameters:
//   memory - Pointer to the memory area to check
// Returns:
//   1 if the memory area is the end of the data, 0 otherwise
int isEndOfData(const unsigned char *memory) {
    for (int i = 0; i < ROM_RECORD_SIZE; i++) {
        if (memory[i] != 0xFF) {
            return 0;
        }
    }
    return 1;
}

//load the MSX Menu ROM into the MSX
int __no_inline_not_in_flash_func(loadrom_msx_menu)(uint32_t offset)
{
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, true, 32768u, &rom_base, &available_length);

    int record_count = 0; // Record count
    const uint8_t *record_ptr = rom + offset + 0x4000; // Pointer to the ROM records
    for (int i = 0; i < MAX_ROM_RECORDS; i++)      // Read the ROMs from the configuration area
    {
        if (isEndOfData(record_ptr)) {
            break; // Stop if end of data is reached
        }
        memcpy(records[record_count].Name, record_ptr, ROM_NAME_MAX); // Copy the ROM name
        record_ptr += ROM_NAME_MAX; // Move the pointer to the next field
        records[record_count].Mapper = *record_ptr++; // Read the mapper code
        records[record_count].Size = read_ulong(record_ptr); // Read the ROM size
        record_ptr += sizeof(unsigned long); // Move the pointer to the next field
        records[record_count].Offset = read_ulong(record_ptr); // Read the ROM offset
        record_ptr += sizeof(unsigned long); // Move the pointer to the next record
        record_count++; // Increment the record count
    }

    msx_pio_bus_init();

    menu_ctx_t menu_ctx = {
        .rom_index = 0,
        .rom_selected = false,
    };

    while (true)  // Loop until a ROM is selected
    {
        pio_drain_writes(handle_menu_write, &menu_ctx);

        if (menu_ctx.rom_selected)
        {
            // ROM selected — switch to non-blocking PIO reads.
            // After the MSX executes rst 0x00, the cartridge slot is
            // deselected.  On MSX2 the BIOS rescans via expanded slots
            // and reads addr 0x0000 through the cartridge (PIO catches
            // it).  On MSX1 the BIOS never selects the cartridge at
            // addr 0x0000, so detect the reboot on the raw bus via GPIO
            // instead (same approach as the non-PIO firmware).
            if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
            {
                uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
                pio_drain_writes(handle_menu_write, &menu_ctx);

                // MSX2 path: expanded-slot scan reaches addr 0x0000
                if (addr == 0x0000u)
                {
                    pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read,
                                        pio_build_token(false, 0xFFu));
                    return menu_ctx.rom_index;
                }

                // Serve the remaining menu ROM reads before the reset
                bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
                uint8_t data = 0xFFu;
                if (in_window)
                {
                    uint32_t rel = addr - 0x4000u;
                    if (available_length == 0u || rel < available_length)
                        data = read_rom_byte(rom_base, rel);
                }
                pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read,
                                    pio_build_token(in_window, data));
            }
            else
            {
                // PIO idle (SLTSL high) — the cartridge slot is not
                // selected.  MSX1 path: detect the reboot by watching
                // for addr 0x0000 on the address bus with RD active,
                // regardless of SLTSL.
                if (!gpio_get(PIN_RD) &&
                    ((gpio_get_all() & 0xFFFFu) == 0x0000u))
                {
                    return menu_ctx.rom_index;
                }
            }
        }
        else
        {
            // Normal operation — blocking PIO read
            uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio,
                                                           msx_bus.sm_read);

            pio_drain_writes(handle_menu_write, &menu_ctx);

            if (menu_ctx.rom_selected && addr == 0x0000u)
            {
                pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read,
                                    pio_build_token(false, 0xFFu));
                return menu_ctx.rom_index;
            }

            bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
            uint8_t data = 0xFFu;

            if (in_window)
            {
                uint32_t rel = addr - 0x4000u;
                if (available_length == 0u || rel < available_length)
                {
                    data = read_rom_byte(rom_base, rel);
                }
            }

            pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read,
                                pio_build_token(in_window, data));
        }
    }
}

// loadrom_planar32 - Load a simple 32KB (or less) ROM into the MSX directly from the pico flash
// 32KB ROMS have two pages of 16Kb each in the following areas:
// 0x4000-0x7FFF and 0x8000-0xBFFF
// AB is on 0x0000, 0x0001
// 16KB ROMS have only one page in the 0x4000-0x7FFF area
// AB is on 0x0000, 0x0001
void __no_inline_not_in_flash_func(loadrom_planar32)(uint32_t offset, bool cache_enable)
{
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 32768u, &rom_base, &available_length);

    msx_pio_bus_init();

    while (true)
    {
        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint32_t rel = addr - 0x4000u;
            if (available_length == 0u || rel < available_length)
            {
                data = read_rom_byte(rom_base, rel);
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// loadrom_planar48 - Load a simple 48KB Planar ROM into the MSX directly from the pico flash
// Those ROMs have three pages of 16Kb each in the following areas:
// 0x0000-0x3FFF, 0x4000-0x7FFF and 0x8000-0xBFFF
// AB is on 0x4000, 0x4001
void __no_inline_not_in_flash_func(loadrom_planar48)(uint32_t offset, bool cache_enable)
{
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 49152u, &rom_base, &available_length);

    msx_pio_bus_init();

    while (true) 
    {
        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        bool in_window = (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint32_t rel = addr;
            if (available_length == 0u || rel < available_length)
            {
                data = read_rom_byte(rom_base, rel);
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// loadrom_planar64 - Load a 64KB Planar ROM into the MSX from pico flash
// Those ROMs have four 16KB pages mapped over the full address space:
// 0x0000-0x3FFF, 0x4000-0x7FFF, 0x8000-0xBFFF and 0xC000-0xFFFF
void __no_inline_not_in_flash_func(loadrom_planar64)(uint32_t offset, bool cache_enable)
{
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 65536u, &rom_base, &available_length);

    msx_pio_bus_init();

    while (true)
    {
        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        bool in_window = true;
        uint8_t data = 0xFFu;

        if (available_length == 0u || (uint32_t)addr < available_length)
        {
            data = read_rom_byte(rom_base, (uint32_t)addr);
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// loadrom_konamiscc - Load a any Konami SCC ROM into the MSX directly from the pico flash
// The KonamiSCC ROMs are divided into 8KB segments, managed by a memory mapper that allows dynamic switching of these segments 
// into the MSX's address space. Since the size of the mapper is 8Kb, the memory banks are:
// Bank 1: 4000h - 5FFFh , Bank 2: 6000h - 7FFFh, Bank 3: 8000h - 9FFFh, Bank 4: A000h - BFFFh
// And the address to change banks are:
// Bank 1: 5000h - 57FFh (5000h used), Bank 2: 7000h - 77FFh (7000h used), Bank 3: 9000h - 97FFh (9000h used), Bank 4: B000h - B7FFh (B000h used)
// AB is on 0x0000, 0x0001
void __no_inline_not_in_flash_func(loadrom_konamiscc)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3}; // Initial banks 0-3 mapped
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    msx_pio_bus_init();
    banked8_loop(rom_base, available_length, bank_registers, handle_konamiscc_write);
}

// loadrom_konami - Load a Konami (without SCC) ROM into the MSX directly from the pico flash
// The Konami (without SCC) ROM is divided into 8KB segments, managed by a memory mapper that allows dynamic switching of these segments into the MSX's address space
// Since the size of the mapper is 8Kb, the memory banks are:
//  Bank 1: 4000h - 5FFFh, Bank 2: 6000h - 7FFFh, Bank 3: 8000h - 9FFFh, Bank 4: A000h - BFFFh
// And the addresses to change banks are:
//	Bank 1: <none>, Bank 2: 6000h - 67FFh (6000h used), Bank 3: 8000h - 87FFh (8000h used), Bank 4: A000h - A7FFh (A000h used)
// AB is on 0x0000, 0x0001
void __no_inline_not_in_flash_func(loadrom_konami)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3}; // Initial banks 0-3 mapped
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    msx_pio_bus_init();
    banked8_loop(rom_base, available_length, bank_registers, handle_konami_write);
}

// loadrom_ascii8 - Load an ASCII8 ROM into the MSX directly from the pico flash
// The ASCII8 ROM is divided into 8KB segments, managed by a memory mapper that allows dynamic switching of these segments into the MSX's address space
// Since the size of the mapper is 8Kb, the memory banks are:
// Bank 1: 4000h - 5FFFh , Bank 2: 6000h - 7FFFh, Bank 3: 8000h - 9FFFh, Bank 4: A000h - BFFFh
// And the address to change banks are:
// Bank 1: 6000h - 67FFh (6000h used), Bank 2: 6800h - 6FFFh (6800h used), Bank 3: 7000h - 77FFh (7000h used), Bank 4: 7800h - 7FFFh (7800h used)
// AB is on 0x0000, 0x0001
void __no_inline_not_in_flash_func(loadrom_ascii8)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3}; // Initial banks 0-3 mapped
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    msx_pio_bus_init();
    banked8_loop(rom_base, available_length, bank_registers, handle_ascii8_write);
}

// loadrom_ascii16 - Load an ASCII16 ROM into the MSX directly from the pico flash
// The ASCII16 ROM is divided into 16KB segments, managed by a memory mapper that allows dynamic switching of these segments into the MSX's address space
// Since the size of the mapper is 16Kb, the memory banks are:
// Bank 1: 4000h - 7FFFh , Bank 2: 8000h - BFFFh
// And the address to change banks are:
// Bank 1: 6000h - 67FFh (6000h used), Bank 2: 7000h - 77FFh (7000h and 77FFh used)
void __no_inline_not_in_flash_func(loadrom_ascii16)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[2] = {0, 1}; // Initial banks 0 and 1 mapped
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
            uint8_t bank = (addr >> 15) & 1u;
            uint32_t rel = ((uint32_t)bank_registers[bank] << 14) + (addr & 0x3FFFu);
            if (available_length == 0u || rel < available_length)
            {
                data = read_rom_byte(rom_base, rel);
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// -----------------------------------------------------------------------
// loadrom_ascii16x - ASCII16-X mapper
// -----------------------------------------------------------------------
// Two 16KB banks mirrored to all four 16KB address quadrants:
//   page 1 bank at 4000-7FFF and C000-FFFF
//   page 2 bank at 0000-3FFF and 8000-BFFF
//
// Bank register mirrors:
//   page 1: 2000-2FFF, 6000-6FFF, A000-AFFF, E000-EFFF
//   page 2: 3000-3FFF, 7000-7FFF, B000-BFFF, F000-FFFF
//
// Bank number is 12-bit:
//   bits 0-7 from data bus (D0-D7)
//   bits 8-11 from address lines A8-A11
void __no_inline_not_in_flash_func(loadrom_ascii16x)(uint32_t offset, bool cache_enable)
{
    (void)cache_enable;

    ascii16x_state_t state;
    memset(&state, 0, sizeof(state));

    bcache_init(&state.cache, 16384u, 2, rom + offset, active_rom_size);
    bcache_prefill(&state.cache);

    // Both pages start at bank 0 after reset
    state.cache.page_slot[0] = bcache_find(&state.cache, 0);
    state.cache.page_slot[1] = state.cache.page_slot[0];

    msx_pio_bus_init();

    while (true)
    {
        pio_drain_writes(handle_ascii16x_write_cached, &state);

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        pio_drain_writes(handle_ascii16x_write_cached, &state);

        // ASCII16-X mirrors ROM across all 4 quadrants.
        // Bit 14 selects the page: 1 = page 1 (regs[0]), 0 = page 2 (regs[1]).
        uint8_t data = 0xFFu;
        uint8_t page_idx = ((addr >> 14) & 0x01u) ? 0u : 1u;
        int8_t slot = state.cache.page_slot[page_idx];

        if (slot >= 0)
            data = rom_sram[(uint32_t)slot * state.cache.slot_size + (addr & 0x3FFFu)];

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(true, data));
    }
}

// loadrom_neo8 - Load an NEO8 ROM into the MSX directly from the pico flash
// The NEO8 ROM is divided into 8KB segments, managed by a memory mapper that allows dynamic switching of these segments into the MSX's address space
// Size of a segment: 8 KB
// Segment switching addresses:
// Bank 0: 0000h~1FFFh, Bank 1: 2000h~3FFFh, Bank 2: 4000h~5FFFh, Bank 3: 6000h~7FFFh, Bank 4: 8000h~9FFFh, Bank 5: A000h~BFFFh
// Switching address: 
// 5000h (mirror at 1000h, 9000h and D000h), 
// 5800h (mirror at 1800h, 9800h and D800h), 
// 6000h (mirror at 2000h, A000h and E000h), 
// 6800h (mirror at 2800h, A800h and E800h), 
// 7000h (mirror at 3000h, B000h and F000h), 
// 7800h (mirror at 3800h, B800h and F800h)
void __no_inline_not_in_flash_func(loadrom_neo8)(uint32_t offset)
{
    neo_state_t state;
    memset(&state, 0, sizeof(state));

    bcache_init(&state.cache, 8192u, 6, rom + offset, active_rom_size);
    bcache_prefill(&state.cache);

    // All 6 banks start at segment 0 after reset
    int8_t slot0 = bcache_find(&state.cache, 0);
    for (int i = 0; i < 6; i++)
        state.cache.page_slot[i] = slot0;

    msx_pio_bus_init();

    while (true)
    {
        pio_drain_writes(handle_neo8_write_cached, &state);

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        pio_drain_writes(handle_neo8_write_cached, &state);

        bool in_window = (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint8_t bank_index = addr >> 13;
            if (bank_index < 6u)
            {
                int8_t slot = state.cache.page_slot[bank_index];
                if (slot >= 0)
                    data = rom_sram[(uint32_t)slot * state.cache.slot_size + (addr & 0x1FFFu)];
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// loadrom_neo16 - Load an NEO16 ROM into the MSX directly from the pico flash
// The NEO16 ROM is divided into 16KB segments, managed by a memory mapper that allows dynamic switching of these segments into the MSX's address space
// Size of a segment: 16 KB
// Segment switching addresses:
// Bank 0: 0000h~3FFFh, Bank 1: 4000h~7FFFh, Bank 2: 8000h~BFFFh
// Switching address:
// 5000h (mirror at 1000h, 9000h and D000h),
// 6000h (mirror at 2000h, A000h and E000h),
// 7000h (mirror at 3000h, B000h and F000h)
void __no_inline_not_in_flash_func(loadrom_neo16)(uint32_t offset)
{
    neo_state_t state;
    memset(&state, 0, sizeof(state));

    bcache_init(&state.cache, 16384u, 3, rom + offset, active_rom_size);
    bcache_prefill(&state.cache);

    // All 3 banks start at segment 0 after reset
    int8_t slot0 = bcache_find(&state.cache, 0);
    for (int i = 0; i < 3; i++)
        state.cache.page_slot[i] = slot0;

    msx_pio_bus_init();

    while (true)
    {
        pio_drain_writes(handle_neo16_write_cached, &state);

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        pio_drain_writes(handle_neo16_write_cached, &state);

        bool in_window = (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint8_t bank_index = addr >> 14;
            if (bank_index < 3u)
            {
                int8_t slot = state.cache.page_slot[bank_index];
                if (slot >= 0)
                    data = rom_sram[(uint32_t)slot * state.cache.slot_size + (addr & 0x3FFFu)];
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
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
// loadrom_sunrise_mapper - Sunrise IDE Nextor + 192KB Memory Mapper
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

    // ---------------------------------------------------------------
    // Hold the MSX bus with WAIT while we initialise the mapper.
    // After the cold reboot the BIOS starts probing slots immediately.
    // We must freeze the CPU until the PIO engines, mapper RAM and I/O
    // port handlers are all ready, otherwise the BIOS probe races
    // against our initialisation and fails to detect the 192KB mapper.
    // ---------------------------------------------------------------
    // Stop PIO state machines left over from the menu phase so we can
    // reclaim PIN_WAIT as a regular GPIO output.
    pio_sm_set_enabled(pio0, 0, false);
    pio_sm_set_enabled(pio0, 1, false);
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 0);            // Assert WAIT — freeze MSX bus

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
    uint8_t subslot_reg = 0x10;

    // Clear mapper RAM
    memset(mapper_ram, 0xFF, MAPPER_SIZE);

    // Initialise PIO I/O bus FIRST (mapper port handlers must be ready
    // before the memory bus releases WAIT and the BIOS starts probing).
    msx_pio_io_bus_init();

    // Initialise PIO memory bus — this hands PIN_WAIT back to the PIO
    // read SM, which releases WAIT as hi-Z until it needs to stretch a read,
    // so the MSX resumes execution with everything fully initialised.
    msx_pio_bus_init();

    sunrise_ctx_t ctx = { .ide = &ide };

    // Main loop: service memory reads/writes and I/O reads/writes
    while (true)
    {
        // --- Drain memory writes (Sunrise IDE + mapper RAM + sub-slot) ---
        {
            uint16_t waddr;
            uint8_t wdata;
            while (pio_try_get_write(&waddr, &wdata))
            {
                if (waddr == 0xFFFFu)
                {
                    subslot_reg = wdata;
                }
                else
                {
                    uint8_t page = (waddr >> 14) & 0x03u;
                    uint8_t active_subslot = (subslot_reg >> (page * 2)) & 0x03u;

                    if (active_subslot == 0)
                    {
                        if (waddr >= 0x4000u && waddr <= 0x7FFFu)
                        {
                            sunrise_ide_handle_write(&ide, waddr, wdata);
                        }
                    }
                    else if (active_subslot == 1)
                    {
                        uint8_t mapper_page = mapper_page_from_reg(mapper_reg[page]);
                        uint32_t mapper_offset = ((uint32_t)mapper_page << 14) | (waddr & 0x3FFFu);
                        mapper_ram[mapper_offset] = wdata;
                    }
                }
            }
        }

        // --- Drain I/O writes (mapper page registers FC-FF) ---
        // I/O ports are global (not slot-dependent), so the mapper must
        // always track writes to FC-FF regardless of subslot selection.
        {
            uint16_t io_addr;
            uint8_t io_data;
            while (pio_try_get_io_write(&io_addr, &io_data))
            {
                uint8_t port = io_addr & 0xFFu;
                if (port >= 0xFCu && port <= 0xFFu)
                {
                    mapper_reg[port - 0xFCu] = io_data & 0x0Fu;
                }
            }
        }

        // --- Handle I/O reads (mapper page registers FC-FF) ---
        // I/O ports are global (not slot-dependent), so the mapper must
        // always respond to FC-FF reads regardless of subslot selection.
        {
            uint16_t io_addr;
            while (pio_try_get_io_read(&io_addr))
            {
                uint8_t port = io_addr & 0xFFu;
                bool in_window = false;
                uint8_t data = 0xFFu;

                if (port >= 0xFCu && port <= 0xFFu)
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

            if (addr == 0xFFFFu)
            {
                in_window = true;
                data = ~subslot_reg;
            }
            else
            {
                uint8_t page = (addr >> 14) & 0x03u;
                uint8_t active_subslot = (subslot_reg >> (page * 2)) & 0x03u;

                if (active_subslot == 0)
                {
                    if (addr >= 0x4000u && addr <= 0x7FFFu)
                    {
                        in_window = true;

                        uint8_t ide_data;
                        if (sunrise_ide_handle_read(&ide, addr, &ide_data))
                        {
                            data = ide_data;
                        }
                        else
                        {
                            uint8_t seg = ide.segment;
                            uint32_t rel = ((uint32_t)seg << 14) + (addr & 0x3FFFu);
                            if (available_length == 0u || rel < available_length)
                                data = read_rom_byte(rom_base, rel);
                        }
                    }
                }
                else if (active_subslot == 1)
                {
                    in_window = true;
                    uint8_t mapper_page = mapper_page_from_reg(mapper_reg[page]);
                    uint32_t mapper_offset = ((uint32_t)mapper_page << 14) | (addr & 0x3FFFu);
                    data = mapper_ram[mapper_offset];
                }
            }

            pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
        }
    }
}

// -----------------------------------------------------------------------
// Manbow2 mapper — Konami SCC banking + AMD AM29F040B flash emulation
// -----------------------------------------------------------------------
// The Manbow2 cartridge uses standard Konami SCC bank switching (4×8KB
// windows at 0x4000-0xBFFF) backed by an AMD AM29F040B 512KB flash chip.
// The last 64KB sector (0x70000-0x7FFFF) is writable and used for save
// data.  All writes in the 0x4000-0xBFFF window pass through both the
// bank-switch decoder and the flash command state machine.
//
// Flash protocol (simplified — only the commands the game actually uses):
//   Program byte:  AA→[x555] 55→[x2AA] A0→[x555] data→[target]
//   Sector erase:  AA→[x555] 55→[x2AA] 80→[x555] AA→[x555] 55→[x2AA] 30→[sector]
//   Auto-select:   AA→[x555] 55→[x2AA] 90→[x555]
//   Reset:         F0→[any]
//
// Save data is backed by SRAM (volatile — lost on power off).  The last
// 64KB of the 192KB SRAM pool is reserved for the writable sector, leaving
// 128KB for the normal ROM cache.

// Flash state machine states
enum manbow2_flash_state {
    MBW2_READ = 0,    // Normal read mode
    MBW2_CMD1,        // Received AA at x555, waiting for 55 at x2AA
    MBW2_CMD2,        // Received 55 at x2AA, waiting for command byte at x555
    MBW2_AUTOSELECT,  // Auto-select / ID mode
    MBW2_PROGRAM,     // Waiting for single data byte to program
    MBW2_ERASE_CMD1,  // Received 80, waiting for AA at x555
    MBW2_ERASE_CMD2,  // Received AA, waiting for 55 at x2AA
    MBW2_ERASE_CMD3   // Received 55, waiting for 30 + sector address
};

// Context for the Manbow2 mapper write handler and main loop
typedef struct {
    uint8_t bank_regs[4];         // Bank registers (Konami SCC style)
    enum manbow2_flash_state state;
    uint8_t *writable_sram;       // Pointer to 64KB SRAM backing the writable sector
    uint32_t writable_offset;     // First flash byte offset of writable sector (0x70000)
    uint32_t writable_size;       // Size of writable sector (0x10000 = 64KB)
    uint32_t rom_size_mask;       // ROM size - 1 (for bank wrapping)
} manbow2_ctx_t;

// Manbow2 write handler — processes both bank switching and flash commands.
// Every write in 0x4000-0xBFFF is routed here.
static inline void __not_in_flash_func(handle_manbow2_write)(uint16_t addr, uint8_t data, void *ctx)
{
    manbow2_ctx_t *mb = (manbow2_ctx_t *)ctx;

    if (addr < 0x4000u || addr > 0xBFFFu)
        return;

    uint8_t page = (uint8_t)((addr - 0x4000u) >> 13);

    // --- Flash command state machine ---
    // Compute the absolute flash address BEFORE updating the bank register.
    // This matches openMSX: flash.write(addr, value) uses the OLD bank,
    // then setRom(page, value) updates it afterward.
    uint32_t flash_addr = (uint32_t)mb->bank_regs[page] * 0x2000u + (addr & 0x1FFFu);

    // --- Bank switching (Konami SCC style) ---
    // Writes where (addr & 0x1800) == 0x1000 within each 8KB page select banks.
    // Updated AFTER flash_addr is computed (matches real hardware ordering).
    if ((addr & 0x1800u) == 0x1000u)
    {
        mb->bank_regs[page] = data & 0x3Fu;  // mask to 64 blocks (512KB / 8KB)
    }

    // Reset command (F0) from any state
    if (data == 0xF0u)
    {
        mb->state = MBW2_READ;
        return;
    }

    switch (mb->state)
    {
        case MBW2_READ:
            if ((flash_addr & 0x7FFu) == 0x555u && data == 0xAAu)
                mb->state = MBW2_CMD1;
            break;

        case MBW2_CMD1:
            if ((flash_addr & 0x7FFu) == 0x2AAu && data == 0x55u)
                mb->state = MBW2_CMD2;
            else
                mb->state = MBW2_READ;
            break;

        case MBW2_CMD2:
            if (data == 0xA0u)
                mb->state = MBW2_PROGRAM;
            else if (data == 0x90u)
                mb->state = MBW2_AUTOSELECT;
            else if (data == 0x80u)
                mb->state = MBW2_ERASE_CMD1;
            else
                mb->state = MBW2_READ;
            break;

        case MBW2_PROGRAM:
        {
            // Program a single byte: flash can only clear bits (AND with existing data).
            // Only the writable sector accepts programming.
            if (flash_addr >= mb->writable_offset &&
                flash_addr < mb->writable_offset + mb->writable_size)
            {
                uint32_t sram_off = flash_addr - mb->writable_offset;
                mb->writable_sram[sram_off] &= data;  // Flash program = AND
            }
            mb->state = MBW2_READ;
            break;
        }

        case MBW2_ERASE_CMD1:
            if ((flash_addr & 0x7FFu) == 0x555u && data == 0xAAu)
                mb->state = MBW2_ERASE_CMD2;
            else
                mb->state = MBW2_READ;
            break;

        case MBW2_ERASE_CMD2:
            if ((flash_addr & 0x7FFu) == 0x2AAu && data == 0x55u)
                mb->state = MBW2_ERASE_CMD3;
            else
                mb->state = MBW2_READ;
            break;

        case MBW2_ERASE_CMD3:
            if (data == 0x30u)
            {
                // Erase the sector containing flash_addr, if writable.
                if (flash_addr >= mb->writable_offset &&
                    flash_addr < mb->writable_offset + mb->writable_size)
                {
                    memset(mb->writable_sram, 0xFF, mb->writable_size);
                }
            }
            mb->state = MBW2_READ;
            break;

        case MBW2_AUTOSELECT:
            // Only F0 (handled above) exits auto-select mode.
            // Any other write is ignored while in this state.
            break;
    }
}

// -----------------------------------------------------------------------
// loadrom_manbow2 - Manbow2 (Konami SCC + AMD flash) mapper
// -----------------------------------------------------------------------
// 8KB banks: 4000-5FFF, 6000-7FFF, 8000-9FFF, A000-BFFF
// Same bank select addresses as Konami SCC.
// Flash sector 7 (last 64KB of 512KB ROM) is writable via SRAM.
void __no_inline_not_in_flash_func(loadrom_manbow2)(uint32_t offset, bool cache_enable)
{
    const uint8_t *rom_base;
    uint32_t available_length;

    // Reserve the last 64KB of the 192KB SRAM pool for the writable flash sector.
    // This leaves 128KB for the ROM cache (sufficient for the first portion
    // of the 512KB ROM; the remainder is served from flash XIP).
    static const uint32_t WRITABLE_SECTOR_SIZE   = 0x10000u;  // 64KB
    static const uint32_t WRITABLE_SECTOR_OFFSET = 0x70000u;  // Last sector of 512KB
    uint32_t reduced_cache = CACHE_SIZE - WRITABLE_SECTOR_SIZE;

    // Cache only the first 128KB by passing reduced_cache as preferred_size.
    // prepare_rom_source will DMA that much into rom_sram and set
    // rom_cached_size accordingly.
    prepare_rom_source(offset, cache_enable, reduced_cache, &rom_base, &available_length);

    // Override: prepare_rom_source capped available_length at 128KB and
    // pointed rom_base at rom_sram.  Reset both so the full ROM is
    // accessible: read_rom_byte serves the first 128KB from the SRAM cache
    // (via rom_cached_size) and the remainder from flash XIP.
    rom_base = rom + offset;
    available_length = active_rom_size;

    // Set up the writable SRAM area (last 64KB of rom_sram)
    uint8_t *writable_sram = &rom_sram[reduced_cache];

    // Initialise writable sector with ROM content (the original save data area).
    // Always read from flash, not the SRAM cache, because the cache only
    // covers the first 128KB.
    if (available_length >= WRITABLE_SECTOR_OFFSET + WRITABLE_SECTOR_SIZE)
    {
        const uint8_t *flash_rom = rom + offset;
        memcpy(writable_sram, flash_rom + WRITABLE_SECTOR_OFFSET, WRITABLE_SECTOR_SIZE);
    }
    else if (available_length > WRITABLE_SECTOR_OFFSET)
    {
        const uint8_t *flash_rom = rom + offset;
        uint32_t partial = available_length - WRITABLE_SECTOR_OFFSET;
        memcpy(writable_sram, flash_rom + WRITABLE_SECTOR_OFFSET, partial);
        memset(writable_sram + partial, 0xFF, WRITABLE_SECTOR_SIZE - partial);
    }
    else
    {
        memset(writable_sram, 0xFF, WRITABLE_SECTOR_SIZE);
    }

    // Initialize Manbow2 context
    manbow2_ctx_t mb = {
        .bank_regs = {0, 1, 2, 3},
        .state = MBW2_READ,
        .writable_sram = writable_sram,
        .writable_offset = WRITABLE_SECTOR_OFFSET,
        .writable_size = WRITABLE_SECTOR_SIZE,
        .rom_size_mask = 0
    };

    msx_pio_bus_init();

    // Main loop: service PIO read/write events.
    // Unlike the generic banked8_loop, this loop continuously drains the
    // write FIFO even while waiting for reads.  Manbow2 generates heavy
    // write traffic (flash data written through 0x5000-0x57FF etc.) that
    // can overflow the 8-entry PIO FIFO if writes are only drained around
    // read events.  Lost bank-switch writes cause the Z80 to read from
    // the wrong bank, crashing the game.  Same pattern as loadrom_sunrise.
    while (true)
    {
        uint16_t addr;

        // Poll: drain write FIFO continuously while waiting for a read
        while (true)
        {
            uint16_t waddr;
            uint8_t wdata;
            while (pio_try_get_write(&waddr, &wdata))
            {
                handle_manbow2_write(waddr, wdata, &mb);
            }
            if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
            {
                addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
                break;
            }
        }

        // Drain any writes that arrived alongside the read
        {
            uint16_t waddr;
            uint8_t wdata;
            while (pio_try_get_write(&waddr, &wdata))
            {
                handle_manbow2_write(waddr, wdata, &mb);
            }
        }

        bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint8_t page = (uint8_t)((addr - 0x4000u) >> 13);
            uint32_t rel = (uint32_t)mb.bank_regs[page] * 0x2000u + (addr & 0x1FFFu);

            if (mb.state == MBW2_AUTOSELECT)
            {
                // AMD auto-select: manufacturer ID at offset 0, device ID at offset 1
                // Sector protection at offset 2, extra code at offset 3
                uint32_t id_addr = rel & 0x03u;
                if (id_addr == 0x00u)
                    data = 0x01u;  // AMD manufacturer ID
                else if (id_addr == 0x01u)
                    data = 0xA4u;  // AM29F040B device ID
                else if (id_addr == 0x02u)
                {
                    // Sector write-protect status: 0 = writable, 1 = protected
                    // Only sector 7 (0x70000-0x7FFFF) is writable
                    data = (rel >= mb.writable_offset && rel < mb.writable_offset + mb.writable_size)
                           ? 0x00u : 0x01u;
                }
                else
                    data = 0x00u;
            }
            else
            {
                // Normal read: check writable sector first, then ROM
                if (rel >= mb.writable_offset &&
                    rel < mb.writable_offset + mb.writable_size)
                {
                    data = writable_sram[rel - mb.writable_offset];
                }
                else if (available_length == 0u || rel < available_length)
                {
                    data = read_rom_byte(rom_base, rel);
                }
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// Main function running on core 0
int main(void)
{
    set_sys_clock_khz(230000, true);     // Set system clock to 230MHz
    stdio_init_all();   // Initialize stdio
    setup_gpio();       // Initialize GPIO

    // Multicore setup
    // multicore_launch_core1(wireless_main); // Launch core 1
    // Load the ROM data from flash memory
    int rom_index = loadrom_msx_menu(0x0000); //load the first 32KB ROM into the MSX (The MSX PICOVERSE MENU)

    ROMRecord const *selected = &records[rom_index];
    active_rom_size = selected->Size;

    // Load the selected ROM into the MSX according to the mapper
    switch (selected->Mapper) {
        case 1:
        case 2:
            loadrom_planar32(selected->Offset, true);
            break;
        case 3:
            loadrom_konamiscc(selected->Offset, true);
            break;
        case 4:
            loadrom_planar48(selected->Offset, true);
            break;
        case 5:
            loadrom_ascii8(selected->Offset, true); 
            break;
        case 6:
            loadrom_ascii16(selected->Offset, true); 
            break;
        case 7:
            loadrom_konami(selected->Offset, true); 
            break;
        case 8:
            loadrom_neo8(selected->Offset); 
            break;
        case 9:
            loadrom_neo16(selected->Offset); 
            break;
        case 10:
            loadrom_sunrise(selected->Offset, true); 
           break;
        case 11:
            loadrom_sunrise_mapper(selected->Offset, true);
            break;
        case 12:
            loadrom_ascii16x(selected->Offset, true);
            break;
        case 13:
            loadrom_planar64(selected->Offset, true);
            break;
        case 14:
            loadrom_manbow2(selected->Offset, true);
            break;
        default:
            printf("Debug: Unsupported ROM mapper: %d\n", selected->Mapper);
            break;
    }
    
}
