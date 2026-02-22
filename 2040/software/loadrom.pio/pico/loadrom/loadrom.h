// MSX PICOVERSE PROJECT
// (c) 2026 Cristiano Goncalves
// The Retro Hacker
//
// loadrom.h - Pin definitions and constants for MSX PICOVERSE PIO-based ROM loader - v2.0
//
// Defines GPIO pin assignments, ROM record layout, and SRAM cache for the RP2040
// PIO-based MSX bus engine. The ROM image is concatenated after the program binary
// in flash; at runtime it is optionally copied into SRAM for faster access.
//
// This work is licensed under a "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
// License". https://creativecommons.org/licenses/by-nc-sa/4.0/

#ifndef LOADROM_H
#define LOADROM_H

#define ROM_NAME_MAX    50       // Maximum length of the ROM name string
#define ROM_RECORD_SIZE (ROM_NAME_MAX + 1 + (sizeof(uint32_t) * 2)) // Name + mapper type + size + offset
#define CACHE_SIZE      196608   // 192 KB SRAM cache for ROM data
#define MAPPER_SIZE     196608   // 192 KB memory mapper RAM (test mode)
#define MAPPER_PAGES    12       // 192 KB / 16 KB = 12 pages
#define MAPPER_PAGE_SIZE 16384   // 16 KB per mapper page
#define PICO_FLASH_SPI_CLKDIV 2

// -----------------------------------------------------------------------
// GPIO pin assignments (directly mapped to MSX bus signals)
// -----------------------------------------------------------------------
// Address lines (A0-A15) — directly readable by PIO via "in pins"
#define PIN_A0     0 
#define PIN_A1     1
#define PIN_A2     2
#define PIN_A3     3
#define PIN_A4     4
#define PIN_A5     5
#define PIN_A6     6
#define PIN_A7     7
#define PIN_A8     8
#define PIN_A9     9
#define PIN_A10    10
#define PIN_A11    11
#define PIN_A12    12
#define PIN_A13    13
#define PIN_A14    14
#define PIN_A15    15

// Data lines (D0-D7) — bidirectional, driven by PIO "out pins" / "out pindirs"
#define PIN_D0     16
#define PIN_D1     17
#define PIN_D2     18
#define PIN_D3     19
#define PIN_D4     20
#define PIN_D5     21
#define PIN_D6     22
#define PIN_D7     23

// Control signals (directly monitored by PIO via wait/jmp pin instructions)
#define PIN_RD     24   // /RD  — active-low read strobe
#define PIN_WR     25   // /WR  — active-low write strobe
#define PIN_IORQ   26   // /IORQ — active-low I/O request
#define PIN_SLTSL  27   // /SLTSL — active-low slot select
#define PIN_WAIT    28  // /WAIT — active-low, driven by PIO side-set
#define PIN_BUSSDIR 29  // BUSSDIR — bus direction control

// -----------------------------------------------------------------------
// ROM storage
// -----------------------------------------------------------------------

// Linker symbol marking the end of the program binary in flash.
// The ROM image is concatenated immediately after this address.
extern unsigned char __flash_binary_end;

// SRAM — shared between ROM cache and mapper RAM.
// Normal modes use the full 192KB as ROM cache.
// Sunrise+Mapper mode uses the full 192KB as mapper RAM (no ROM cache).
static union {
    uint8_t rom_sram[CACHE_SIZE];           // normal: full 192KB ROM cache
    struct {
        uint8_t mapper_ram[MAPPER_SIZE];      // mapper: 192KB mapper RAM
    } mapper;
} sram_pool;

#define rom_sram    sram_pool.rom_sram
#define mapper_ram  sram_pool.mapper.mapper_ram

static uint32_t active_rom_size = 0;

// Pointer to the ROM data in flash (right after the program binary)
const uint8_t *rom = (const uint8_t *)&__flash_binary_end;

#endif