# MSX PicoVerse 2040 LoadROM Tool Manual (en-US)

The PicoVerse 2040 cartridge extends MSX systems by flashing different Raspberry Pi Pico firmwares. While the MultiROM firmware offers a menu-driven launcher, some workflows require flashing a single ROM image that boots immediately on power‑on. That is the purpose of the `loadrom` firmware and of the companion `loadrom.exe` console tool documented here.

`loadrom.exe` bundles the Pico firmware, a configuration record (game name, mapper code, ROM size and offset), and a single MSX ROM payload into an RP2040-compatible UF2 image. Copying the generated UF2 to the Pico’s `RPI-RP2` drive programs the cartridge so that it boots directly into the embedded ROM whenever the MSX starts.
Alternatively, the `-s` (Sunrise) option can be used to flash the cartridge with the Sunrise IDE Nextor firmware, which exposes the USB-C port as a block device for use with Nextor-compatible loaders like SofaRun.
The `-m` option flashes Sunrise IDE Nextor with an additional 192KB memory mapper implementation. On-screen mapper text for this mode is still presented as `SYSTEM` for end users.
## Overview

1. **Input**: one `.ROM` file (case-insensitive extension) supplied via the command line.
2. **Processing**: the tool trims/normalizes the ROM name, detects or forces the mapper type, and streams the Pico firmware, configuration record, and ROM payload into UF2 blocks.
3. **Output**: a UF2 file (default `loadrom.uf2`) that can be copied to the Pico while it is in BOOTSEL mode.

Key characteristics:

- Works on Windows (console app). Tested in `cmd.exe` and PowerShell.
- Supports ROM sizes from 8 KB up to 16 MB (subject to Pico flash capacity).
- Detects common mapper types automatically. Mapper can be forced via filename tags (same scheme as `multirom`): `PL-16`, `PL-32`, `KonSCC`, `Linear`, `ASC-08`, `ASC-16`, `Konami`, `NEO-8`, `NEO-16`.
- Generates UF2 files recognized by the RP2040 ROM bootloader (sets the RP2040 family ID flag).

## Command-line usage

Only the Windows executable is currently distributed.

### Basic syntax

```
loadrom.exe [options] [romfile]
```

### Options

- `-h`, `--help` : Print usage information and exit.
- `-s`, `--sunrise` : Embed the Sunrise IDE Nextor ROM instead of a regular MSX ROM. When this option is used, the cartridge boots into Nextor firmware, and the USB-C port exposes a block device for use with Nextor loaders. No ROM file argument is needed with this option.
- `-m`, `--mapper` : Embed Sunrise IDE Nextor with 192KB memory mapper support. This is intended for Nextor workflows that require mapper-capable behavior. No ROM file argument is needed with this option.
- `-o <filename>`, `--output <filename>` : Override the UF2 output name (default `loadrom.uf2`).
- Positional argument: the path to the MSX ROM you wish to embed (not used with `-s` or `-m`).

### Mapper forcing via filename tags

`loadrom.exe` shares the same forcing mechanism as the MultiROM tool. Append a dot-separated mapper tag before the `.ROM` extension to override detection:

```
Penguin Adventure.PL-32.ROM
Space Manbow.KonSCC.rom
```

Tags are case-insensitive. If no valid tag is present, the tool falls back to heuristic detection.

## Typical workflow

### Loading a regular MSX ROM

1. Place `loadrom.exe` in a working directory alongside the desired ROM file.
2. Open a Command Prompt or PowerShell window in that directory.
3. Run `loadrom.exe` pointing to the ROM and optionally overriding the UF2 name:
   ```
   loadrom.exe "Space Manbow.rom" -o space_manbow.uf2
   ```
4. Observe the console output:
   - Tool banner with embedded version.
   - Resolved ROM name (max 50 characters), size, mapper result, and Pico flash offset.
   - Progress message when UF2 blocks are written.
5. Put the Pico in BOOTSEL mode (hold BOOTSEL, plug USB) to mount the `RPI-RP2` drive.
6. Copy the generated UF2 file (e.g., `space_manbow.uf2`) to the drive. The Pico reboots once flashing completes.
7. Insert the PicoVerse cartridge into the MSX and power on—the embedded ROM boots immediately without a menu.

### Loading Sunrise IDE Nextor

1. Place `loadrom.exe` in a working directory.
2. Open a Command Prompt or PowerShell window in that directory.
3. Run `loadrom.exe` with the `-s` option and set the output filename:
   ```
   loadrom.exe -s -o nextor_sunrise.uf2
   ```
4. Put the Pico in BOOTSEL mode (hold BOOTSEL, plug USB) to mount the `RPI-RP2` drive.
5. Copy the generated UF2 file to the drive. The Pico reboots once flashing completes.
6. Insert the PicoVerse cartridge into the MSX with a USB thumb drive connected (use a USB OTG adapter if needed).
7. Power on the MSX to boot into Nextor. The cartridge's USB-C port now acts as a block device for loading ROMs and DSK files.

### Loading Sunrise IDE Nextor + 192KB Mapper

1. Place `loadrom.exe` in a working directory.
2. Open a Command Prompt or PowerShell window in that directory.
3. Run `loadrom.exe` with the `-m` option and set the output filename:
   ```
   loadrom.exe -m -o nextor_sunrise_mapper.uf2
   ```
4. Put the Pico in BOOTSEL mode (hold BOOTSEL, plug USB) to mount the `RPI-RP2` drive.
5. Copy the generated UF2 file to the drive. The Pico reboots once flashing completes.
6. Insert the PicoVerse cartridge into the MSX with a USB thumb drive connected (use a USB OTG adapter if needed).
7. Power on the MSX to boot into Nextor with the mapper-capable mode enabled. UI mapper text is shown as `SYSTEM`.

## Troubleshooting

| Symptom | Possible cause | Resolution |
| --- | --- | --- |
| "Invalid ROM file" | Missing or wrong extension | Ensure the ROM ends with `.ROM` (case-insensitive). |
| "Failed to detect the ROM type" | Mapper heuristics inconclusive | Add a mapper tag suffix (e.g., `.Konami.ROM`). |
| UF2 not recognized by Pico | File copied before BOOTSEL, or wrong family ID | Enter BOOTSEL mode before copying, and regenerate the UF2 with the latest tool. |
| Game title looks truncated | Name exceeds 50 characters | Use shorter filenames; the config record stores 50 bytes max. |

## Known limitations

- With a regular ROM: only one ROM can be embedded per UF2; use the MultiROM tool for menus and multiple titles.
- With Sunrise Nextor: requires a compatible USB thumb drive and appropriate Nextor loaders on the MSX side.
- Linux/macOS binaries are not yet provided; use Wine or build from source with GCC.
- The tool does not verify ROM integrity beyond size checks and simple header heuristics.

## Tested MSX models

The PicoVerse 2040 cartridge with LoadROM firmware has been tested on the following MSX models:

| Model | Type | Status | Comments |
| --- | --- | --- | --- |
| Adermir Carchano Expert 4 | MSX2+ | OK | Verified operation |
| Gradiente Expert | MSX1 | OK | Verified operation |
| JFF MSX | MSX1 | OK | Verified operation |
| MSX Book | MSX2+ (FPGA clone) | OK | Verified operation |
| MSX One | MSX1 | Not OK | Cartridge not recognized |
| National FS-4500 | MSX1 | OK | Verified operation |
| Omega MSX | MSX2+ | OK | Verified operation |
| Panasonic FS-A1GT | TurboR | OK | Verified operation |
| Panasonic FS-A1ST | TurboR | OK | Verified operation |
| Panasonic FS-A1WX | MSX2+ | OK | Verified operation |
| Panasonic FS-A1WSX | MSX2+ | OK | Verified operation |
| Sanyo Wavy 70FD | MSX2+ | OK | Verified operation |
| Sharp HotBit HB8000 | MSX1 | OK | Verified operation |
| SMX-HB | MSX2+ (FPGA clone) | OK | Verified operation |
| Sony HB-F1XD | MSX2 | OK | Verified operation |
| Sony HB-F1XDJ | MSX2 | OK | Verified operation |
| Sony HB-F1XV | MSX2+ | OK | Verified operation |
| Sony Hit-Bit 20HB | MSX1 | OK | Verified operation |
| TRHMSX | MSX2+ (FPGA clone) | OK | Verified operation |
| uMSX | MSX2+ (FPGA clone) | OK | Verified operation |
| Yamaha YIS604 | MSX1 | OK | Verified operation |

Sunrise IDE Nextor ROM (options `-s` and `-m`) has been tested on the following MSX models:

| Model | Type | Status | Comments |
| --- | --- | --- | --- |
| Adermir Carchano Expert 4 | MSX2+ | OK | Verified operation |
| Gradiente Expert | MSX1 | OK | Verified operation |
| JFF MSX | MSX1 | OK | Verified operation |
| MSX Book | MSX2+ (FPGA clone) | OK | Verified operation |
| National FS-4500 | MSX1 | OK | Verified operation |
| Omega MSX | MSX2+ | OK | Verified operation |
| Panasonic FS-A1GT | TurboR | OK | Verified operation |
| Panasonic FS-A1ST | TurboR | OK | Verified operation |
| Panasonic FS-A1WX | MSX2+ | OK | Verified operation |
| Panasonic FS-A1WSX | MSX2+ | OK | Verified operation |
| Sanyo Wavy 70FD | MSX2+ | OK | Verified operation |
| Sharp HotBit HB8000 | MSX1 | OK | Verified operation |
| SMX-HB | MSX2+ (FPGA clone) | OK | Verified operation |
| Sony HB-F1XD | MSX2 | OK | Verified operation |
| Sony HB-F1XDJ | MSX2 | OK | Verified operation |
| Sony HB-F1XV | MSX2+ | OK | Verified operation |
| Sony Hit-Bit 20HB | MSX1 | OK | Verified operation |
| TRHMSX | MSX2+ (FPGA clone) | OK | Verified operation |
| uMSX | MSX2+ (FPGA clone) | OK | Verified operation |
| Yamaha YIS604 | MSX1 | OK | Verified operation |

Author: Cristiano Almeida Goncalves
Last updated: 02/21/2026
