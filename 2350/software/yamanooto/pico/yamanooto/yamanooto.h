// MSX PICOVERSE PROJECT
// (c) 2026 Cristiano Goncalves
// The Retro Hacker
//
// yamanooto.h - Yamanooto flash cartridge emulation for MSX PICOVERSE 2350 - v1.01
//
// This header defines the pin assignments, memory layout and Yamanooto register
// constants used by the PIO-based Yamanooto emulation firmware. The Yamanooto is
// a Konami-SCC compatible flash cartridge with an 8 MB flash-ROM, SCC/SCC+ audio
// and a secondary (dual) AY-3-8910 PSG.
//
// The ROM image is concatenated right after the firmware binary in flash. The
// firmware reads a small configuration record (name + type + size + offset) and
// then serves the image through the Yamanooto mapper.
//
// This work is licensed under a "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
// License". https://creativecommons.org/licenses/by-nc-sa/4.0/

#ifndef YAMANOOTO_H
#define YAMANOOTO_H

#include <stdint.h>

#define ROM_NAME_MAX    50   // Maximum size of the ROM name
#define ROM_RECORD_SIZE (ROM_NAME_MAX + 1 + (sizeof(uint32_t) * 2)) // Name + type + size + offset

// -----------------------
// Raspberry Pi Pico (RP2350B) pin assignments - shared with the loadrom firmware
// -----------------------
// Address lines (A0-A15) as inputs from MSX
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

// Data lines (D0-D7)
#define PIN_D0     16
#define PIN_D1     17
#define PIN_D2     18
#define PIN_D3     19
#define PIN_D4     20
#define PIN_D5     21
#define PIN_D6     22
#define PIN_D7     23

// Control signals
#define PIN_RD     24   // Read strobe from MSX
#define PIN_WR     25   // Write strobe from MSX
#define PIN_IORQ   26   // IO Request line from MSX
#define PIN_SLTSL  27   // Slot Select for this cartridge slot
#define PIN_WAIT    28  // WAIT line to MSX
#define PIN_BUSSDIR 37  // Bus direction line
#define PIN_PSRAM  47   // PSRAM select line

// I2S DAC pins
#define I2S_DATA_PIN  29   // I2S serial data
#define I2S_BCLK_PIN  30   // I2S bit clock
#define I2S_WSEL_PIN  31   // I2S word select (LRCLK)
#define I2S_MUTE_PIN  32   // I2S DAC mute control

// -----------------------
// SCC emulation constants
// -----------------------
#define SCC_SAMPLE_RATE 22050
#define SCC_CLOCK       3579545
#define SCC_VOLUME_SHIFT 2    // Left-shift SCC output for volume boost (4x)
#define SCC_AUDIO_BUFFER_SAMPLES 256

// -----------------------
// Dual PSG emulation constants
// -----------------------
#define PSG_SAMPLE_RATE 22050
#define PSG_CLOCK       1789773
#define PSG_VOLUME_SHIFT 2
#define PSG_PORT_REG    0x10u   // Secondary PSG register-select port
#define PSG_PORT_DATA   0x11u   // Secondary PSG data port
#define PSG_ECHO_PORT_REG   0xA0u  // Primary PSG register port (echoed when ECHO set)
#define PSG_ECHO_PORT_DATA  0xA1u  // Primary PSG data port (echoed when ECHO set)

// -----------------------
// MSX-MUSIC / YM2413 (OPLL) emulation constants (optional add-on)
// -----------------------
#define MSX_MUSIC_SAMPLE_RATE  22050
#define MSX_MUSIC_CLOCK        3579545
#define MSX_MUSIC_VOLUME_SHIFT 2
#define MSX_MUSIC_PORT_REG     0x7Cu   // MSX-MUSIC register-select port
#define MSX_MUSIC_PORT_DATA    0x7Du   // MSX-MUSIC data port

// Config record 'type' byte flags (see the PC tool)
#define YAMA_MSX_MUSIC_FLAG    0x20u   // Enable the expanded slot + FM-PAC (YM2413); off = plain slot

// -----------------------
// Yamanooto register interface (memory mapped, 0x7FFC-0x7FFF, NOT mirrored)
// -----------------------
#define YAMA_FPGA_REG   0x7FFCu   // Bi-directional FPGA communication channel
#define YAMA_CFGR       0x7FFDu   // Configuration register
#define YAMA_OFFR       0x7FFEu   // Bank offset register
#define YAMA_ENAR       0x7FFFu   // Enable register

// ENAR (0x7FFF) bits
#define YAMA_REGEN      0x01u     // Register interface enable
#define YAMA_WREN       0x10u     // Flash write enable

// CFGR (0x7FFD) bits
#define YAMA_MDIS       0x01u     // Mapper disable
#define YAMA_ECHO       0x02u     // Mirror secondary PSG onto ports 0xA0/0xA1
#define YAMA_ROMDIS     0x04u     // ROM disable
#define YAMA_K4         0x08u     // Konami-4 mapper mode (no SCC)
#define YAMA_SUBOFF     0x30u     // Sub-offset bits (added to the bank offset)
#define YAMA_FPGA_EN    0x40u     // Enable FPGA command channel
#define YAMA_FPGA_WAIT  0x80u     // Read-back: FPGA ready flag (always ready here)

// This symbol marks the end of the main program in flash.
// The ROM data is concatenated immediately after this point.
extern unsigned char __flash_binary_end;

// The ROM is concatenated right after the main program binary.
const uint8_t *rom = (const uint8_t *)&__flash_binary_end;

#endif // YAMANOOTO_H
