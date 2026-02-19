# MSX PicoVerse 2350 MultiROM Tool Manual (EN-US)

The PicoVerse 2350 MultiROM tool creates a UF2 image that flashes the PicoVerse 2350 cartridge with:
- The PicoVerse 2350 MultiROM firmware.
- The MSX MultiROM menu ROM.
- A configuration table describing each ROM entry.
- The ROM payloads themselves.

Use this tool when you want to bundle multiple MSX ROM files into a single UF2 for the PicoVerse 2350.

## Requirements

- A Windows PC (the distributed binary is a Windows console application).
- Your MSX ROM files in a single folder (no subdirectories).
- A PicoVerse 2350 cartridge and a USB-C cable.

## Limits

- Maximum ROM files per image: 128.
- Maximum combined ROM payload size: ~14 MB.
- Per-ROM name length in menu: 50 characters (longer names are truncated in the menu list).

## Basic workflow

1. Place all `.ROM` files you want to include in a single folder.
2. Run the tool in that folder.
3. Copy the generated UF2 file to the PicoVerse 2350 in BOOTSEL mode.
4. Insert the cartridge into the MSX and power on to access the MultiROM menu.

## Command-line usage

```
multirom.exe [options]
```

### Options

- `-h`, `--help` : Show usage help and exit.
- `-o <filename>`, `--output <filename>` : Set UF2 output filename (default is `multirom.uf2`).
- `-n`, `--nextor` : Include embedded Nextor ROM (experimental; MSX2-only). In the 2350 build, this is labeled “Nextor SD (IO)”.

### Example

```
multirom.exe
```

This scans the current directory for `.ROM` files and generates `multirom.uf2`.

## Flashing the UF2

1. Hold BOOTSEL on the PicoVerse 2350 while connecting USB.
2. The `RPI-RP2` drive appears.
3. Copy `multirom.uf2` to the drive.
4. The drive disconnects automatically after flashing.

## ROM mapper detection and tags

The tool analyzes each ROM to determine the correct mapper type. If detection fails or you want to override it, add a mapper tag to the filename before `.ROM`. Tags are case-insensitive.

Supported tags:
`PL-16`, `PL-32`, `KonSCC`, `Linear`, `ASC-08`, `ASC-16`, `Konami`, `NEO-8`, `NEO-16`.

SCC behavior:
- ROMs detected or forced as `KonSCC` (mapper 3) automatically run with SCC sound emulation in the MultiROM firmware.
- If a `KonSCC` entry carries the SCC+ mapper flag in its metadata, SCC+ (enhanced) mode is used automatically.

Example:

```
Knight Mare.PL-32.ROM
```

## Menu usage (on MSX)

- Up/Down: move selection.
- Left/Right: change pages.
- Enter/Space: load selected ROM.
- H: show help screen.

## Notes

- Only ROM files in the current folder are included (no subfolders).
- Unsupported or invalid ROMs are skipped.
- Konami SCC mapper entries (`KonSCC`) do not require an extra runtime option in MultiROM; SCC emulation is enabled automatically.
- The `-n` Nextor option is experimental and may not work on all MSX2 models.

Author: Cristiano Almeida Goncalves
Last updated: 02/16/2026
