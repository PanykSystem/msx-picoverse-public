# MSX PicoVerse 2350 — SCC / SCC+ Sound Emulation

The PicoVerse 2350 LoadROM PIO firmware can emulate the Konami SCC and SCC+ (SCC-I) sound chips in hardware, generating audio through an I2S DAC connected to the RP2350. This gives MSX games that use SCC or SCC+ sound their full soundtrack without requiring an original SCC cartridge.

---

## How it works

### Architecture

The emulation runs across both cores of the RP2350:

| Core | Role |
| --- | --- |
| **Core 0** | MSX bus handling via PIO0. Intercepts SCC/SCC+ register writes from the Z80 and forwards them to the emulator. Serves ROM reads and SCC register reads back to the MSX. |
| **Core 1** | Audio generation. Continuously calls the SCC emulator to produce PCM samples and feeds them to the I2S DAC through `pico_audio_i2s` (PIO1). |

### Emulator library

Sound synthesis is handled by **emu2212** (Mitsutaka Okazaki), a cycle-accurate SCC/SCC+ emulator. The library supports both chip variants:

- **SCC (Standard)** — Konami 051649. Five channels of wavetable synthesis. Register window at `0x9800`–`0x98FF` when enabled by writing `0x3F` to `0x9000`.
- **SCC+ (Enhanced)** — Konami 052539 (SCC-I). Five independent waveform channels (channel 4 and 5 have separate waveforms, unlike standard SCC). Additional mode register at `0xBFFE`–`0xBFFF` can relocate the register window to `0xB800`–`0xB8FF`.

### I2S DAC output

Audio is output as 16-bit stereo PCM at 44100 Hz via the `pico_audio_i2s` library from pico-extras, using PIO1 state machine 0.

| Signal | GPIO | Description |
| --- | --- | --- |
| DATA | 29 | I2S serial data |
| BCLK | 30 | I2S bit clock |
| WSEL | 31 | I2S word select (LRCLK) |
| MUTE | 32 | DAC mute control (directly driven low to unmute) |

The SCC output is mono and is duplicated to both left and right channels. A volume boost of 4× (2-bit left shift) is applied to the raw SCC output before clipping to 16-bit range.

---

## SCC memory map

When SCC emulation is active, the firmware intercepts both writes and reads in the cartridge slot address space (`0x4000`–`0xBFFF`):

### Bank switching (Konami SCC mapper)

| Address range | Function |
| --- | --- |
| `0x5000`–`0x57FF` | Select 8 KB bank for page 0 (`0x4000`–`0x5FFF`) |
| `0x7000`–`0x77FF` | Select 8 KB bank for page 1 (`0x6000`–`0x7FFF`) |
| `0x9000`–`0x97FF` | Select 8 KB bank for page 2 (`0x8000`–`0x9FFF`) |
| `0xB000`–`0xB7FF` | Select 8 KB bank for page 3 (`0xA000`–`0xBFFF`) |

All bank-switching writes are also forwarded to the SCC emulator so it can track the enable/disable state.

### SCC register access (Standard mode)

When the SCC is active (bank 2 register written with `0x3F`):

| Address range | Access | Function |
| --- | --- | --- |
| `0x9800`–`0x981F` | R/W | Channel 1 waveform (32 bytes) |
| `0x9820`–`0x983F` | R/W | Channel 2 waveform (32 bytes) |
| `0x9840`–`0x985F` | R/W | Channel 3 waveform (32 bytes) |
| `0x9860`–`0x987F` | R/W | Channel 4 & 5 waveform (shared, 32 bytes) |
| `0x9880`–`0x9889` | W | Frequency registers (2 bytes per channel × 5) |
| `0x988A`–`0x988E` | W | Volume registers (1 byte per channel × 5) |
| `0x988F` | W | Channel enable/disable mask |
| `0x98E0`–`0x98FF` | R | Test/deformation register |

### SCC+ register access (Enhanced mode)

When in SCC+ mode (mode register at `0xBFFE` written with bit 5 set):

| Address range | Access | Function |
| --- | --- | --- |
| `0xB800`–`0xB81F` | R/W | Channel 1 waveform (32 bytes) |
| `0xB820`–`0xB83F` | R/W | Channel 2 waveform (32 bytes) |
| `0xB840`–`0xB85F` | R/W | Channel 3 waveform (32 bytes) |
| `0xB860`–`0xB87F` | R/W | Channel 4 waveform (32 bytes, independent) |
| `0xB880`–`0xB89F` | R/W | Channel 5 waveform (32 bytes, independent) |
| `0xB8A0`–`0xB8A9` | W | Frequency registers (2 bytes per channel × 5) |
| `0xB8AA`–`0xB8AE` | W | Volume registers (1 byte per channel × 5) |
| `0xB8AF` | W | Channel enable/disable mask |
| `0xBFFE`–`0xBFFF` | R/W | SCC+ mode register |

---

## Using the LoadROM tool

SCC/SCC+ emulation is enabled through command-line flags when creating the UF2 image.

### Command-line options

```
loadrom.exe [options] <romfile>

  -scc, --scc          Enable SCC (standard) sound emulation
  -sccplus, --sccplus  Enable SCC+ (enhanced) sound emulation
  -o <filename>         Set UF2 output filename (default loadrom.uf2)
```

The `-scc` and `-sccplus` flags are **mutually exclusive** — use only one. Both flags require the ROM mapper to be Konami SCC (type 3); the tool prints a warning and ignores the flag for other mapper types.

### Examples

**Standard SCC emulation:**

```
loadrom.exe -scc "Space Manbow.rom"
```

**SCC+ emulation:**

```
loadrom.exe -sccplus "Snatcher.KonSCC.rom"
```

**With custom output filename:**

```
loadrom.exe -scc -o manbow.uf2 "Space Manbow.rom"
```

### Console output

When SCC is enabled:

```
MSX PICOVERSE 2350 LoadROM UF2 Creator v2.00
(c) 2025 The Retro Hacker

ROM Type: Konami SCC [Auto-detected]
SCC Emulation: Enabled
ROM Name: Space Manbow
ROM Size: 131072 bytes
Pico Offset: 0x0000003B
UF2 Output: loadrom.uf2

Successfully wrote 664 blocks to loadrom.uf2.
```

When SCC+ is enabled, the line reads `SCC+ Emulation: Enabled` instead.

---

## Internal encoding

The tool encodes the SCC mode in the mapper byte of the configuration record:

| Bit | Flag | Meaning |
| --- | --- | --- |
| 7 | `0x80` | SCC standard emulation enabled |
| 6 | `0x40` | SCC+ enhanced emulation enabled |
| 5–0 | | Base mapper type (3 = Konami SCC) |

The firmware decodes these flags at boot and initialises the emu2212 library with `SCC_STANDARD` or `SCC_ENHANCED` accordingly.

---

## Emulation parameters

| Parameter | Value |
| --- | --- |
| SCC clock | 3,579,545 Hz (MSX master clock) |
| Sample rate | 44,100 Hz |
| Quality mode | 1 (interpolated) |
| Volume boost | 4× (2-bit left shift with clipping) |
| Audio buffer | 256 stereo samples per block, 3 blocks |
| I2S PIO instance | PIO1, state machine 0 |

---

## Hardware requirements

- **PicoVerse 2350 board** with I2S DAC connected to GPIOs 29–32.
- **LoadROM PIO firmware** (`loadrom.pio` variant, not the legacy bit-banged version).
- An MSX game ROM that uses the Konami SCC mapper.

---

## Compatibility notes

- Games using standard SCC work with `-scc`. Most Konami SCC titles fall into this category (e.g., *Space Manbow*, *Salamander*, *Nemesis 2*, *Penguin Adventure*, *F1 Spirit*, *Parodius*, *Gradius 2*).
- Games or homebrew software that require SCC+ features (independent channel 4/5 waveforms, mode switching) should use `-sccplus`.
- The emulation runs entirely on the Pico — no external SCC chip is needed.
- The SCC audio output is independent of the MSX PSG. The MSX's own PSG channels still play through the MSX audio output as normal.

---

## Limitations

- SCC emulation is only available in the `loadrom.pio` firmware for the 2350 platform.
- The `-scc` and `-sccplus` flags only apply to Konami SCC mapper ROMs (type 3).
- Audio output requires an I2S DAC on the PicoVerse board. Without the DAC, the SCC registers are still emulated (reads return correct values) but no sound is produced.
- The volume boost factor is fixed at compile time.

---

Author: Cristiano Almeida Goncalves  
Last updated: 02/15/2026
