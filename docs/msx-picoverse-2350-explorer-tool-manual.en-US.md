# MSX PicoVerse 2350 Explorer Tool Manual (EN-US)

The Explorer tool creates a UF2 image that flashes the PicoVerse 2350 cartridge with the Explorer firmware. The UF2 bundles:
- PicoVerse 2350 Explorer firmware.
- The MSX Explorer menu ROM.
- A configuration table describing each ROM stored in flash.
- The ROM payloads that will live in Pico flash.

Use the Explorer tool when you want a menu that loads ROMs from both flash and microSD on the PicoVerse 2350.

## Requirements

- Windows PC (the distributed binary is a Windows console app).
- A folder containing the `.ROM` files you want stored in flash.
- A PicoVerse 2350 cartridge and USB-C cable.
- Optional: a microSD card (for additional ROMs on SD).

## Limits

- Flash ROM entries created by the tool: up to 128 files.
- Total flash ROM payload size: ~14 MB combined.
- Supported ROM size range: 8 KB to 15 MB.
- ROM names in the menu are limited to 50 characters (longer names are truncated).

## Basic workflow

1. Put all `.ROM` files you want in flash into a single folder (no subfolders).
2. Run the tool in that folder to create `explorer.uf2`.
3. Put the PicoVerse 2350 into BOOTSEL mode and copy the UF2 to the `RPI-RP2` drive.
4. (Optional) Copy more `.ROM` files to a microSD card for SD loading.
5. Insert the cartridge into your MSX and power on.

## Command-line usage

```
explorer.exe [options]
```

### Options

- `-h`, `--help` : Show usage help and exit.
- `-o <filename>`, `--output <filename>` : Set UF2 output filename (default is `explorer.uf2`).
- `-n`, `--nextor` : Include embedded Nextor ROM (experimental; MSX2-only). In the 2350 build, this is labeled “Nextor SD (IO)”.

### Example

```
explorer.exe
```

This scans the current directory and generates `explorer.uf2`.

## Flashing the UF2

1. Hold BOOTSEL on the PicoVerse 2350 while connecting USB.
2. The `RPI-RP2` drive appears.
3. Copy `explorer.uf2` to the drive.
4. The drive disconnects automatically when flashing completes.

## ROM mapper detection and tags

The tool analyzes each ROM to determine the mapper. If you need to override it, add a mapper tag to the filename before `.ROM`. Tags are case-insensitive.

Supported tags:
`PL-16`, `PL-32`, `KonSCC`, `Linear`, `ASC-08`, `ASC-16`, `Konami`, `NEO-8`, `NEO-16`.

Example:

```
Knight Mare.PL-32.ROM
```

Notes:
- The `SYSTEM` tag is ignored and cannot be forced.
- Unsupported ROMs are skipped.

## Using microSD with Explorer

Explorer can load ROMs from a microSD card in addition to flash. ROMs on SD are merged into the menu list and marked with the “SD” source tag.

- Format the microSD card as FAT/FAT32.
- Copy `.ROM` files to the root of the card (subfolders are not scanned).
- SD ROMs appear in the menu with the source label “SD”.
- Flash ROMs appear with the source label “FL”.

### microSD limitations

- The combined list is capped at 512 entries total (up to 128 from flash + up to 384 from microSD).
- Only `.ROM` files in the card root are scanned (no subfolders).
- microSD ROM files are limited to 256 KB each.
- Unsupported or invalid ROMs are skipped (same mapper and size rules as flash).

## Menu usage (on MSX)

- Up/Down: move selection.
- Left/Right: change pages.
- Enter/Space: load selected ROM.
- H: show help screen.
- F: search the ROM list, then press Enter to jump to the first match.

## Known limitations

- Only ROMs in the current folder are packaged into flash by the tool (no subfolders).
- The `-n` Nextor option is experimental and may not work on all MSX2 models.
- ROMs with unknown or unsupported mappers are skipped unless you force a mapper tag.

Author: Cristiano Almeida Goncalves
Last updated: 01/25/2026
