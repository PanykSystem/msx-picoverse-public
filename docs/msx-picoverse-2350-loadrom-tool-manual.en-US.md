# MSX PicoVerse 2350 LoadROM Tool Manual (EN-US)

The PicoVerse 2350 cartridge can boot either a MultiROM menu or a single ROM that starts immediately. The LoadROM tool creates a UF2 image that flashes the PicoVerse 2350 with a single ROM payload and the LoadROM firmware.

`loadrom.exe` bundles the Pico firmware blob, a configuration record (game name, mapper code, ROM size, and flash offset), and one MSX ROM file into an RP2350-compatible UF2 image. Copy the generated UF2 to the board in BOOTSEL mode and the MSX will boot straight into the embedded ROM.

For RP2350, both firmware packages are available:

- `2350/software/loadrom` (legacy bit-banged bus handling).
- `2350/software/loadrom.pio` (PIO-based bus handling, recommended).

---

## Overview

1. **Input**: one `.ROM` file (case-insensitive extension).
2. **Processing**: the tool normalizes the ROM name, detects or forces the mapper, and streams firmware + config + ROM into UF2 blocks.
3. **Output**: a UF2 file (default `loadrom.uf2`) ready for the RP2350 bootloader.

Key characteristics:

- Windows console application.
- Supports ROM sizes from 8 KB up to 16 MB (subject to flash capacity).
- Mapper auto-detection plus optional filename tags to force the mapper.
- UF2 uses RP2350 family ID (`0xE48BFF59`).

---

## Command-line usage

```
loadrom.exe [options] <romfile>
```

### Options

- `-h`, `--help` : Print usage information and exit.
- `-o <filename>`, `--output <filename>` : Override the UF2 output name (default `loadrom.uf2`).
- Positional argument: the ROM file to embed. Exactly one ROM is required.

### Mapper forcing via filename tags

Append a dot-separated mapper tag before the `.ROM` extension to override detection. Tags are case-insensitive.

Supported tags:
`PL-16`, `PL-32`, `KonSCC`, `Linear`, `ASC-08`, `ASC-16`, `Konami`, `NEO-8`, `NEO-16`.

Example:

```
Penguin Adventure.PL-32.ROM
Space Manbow.KonSCC.rom
```

`SYSTEM` is ignored and cannot be forced.

---

## Typical workflow

1. Place `loadrom.exe` and your ROM file in a working folder.
2. Open a Command Prompt or PowerShell window in that folder.
3. Run the tool:
   ```
   loadrom.exe "Space Manbow.rom" -o space_manbow.uf2
   ```
4. Review the console output (name, size, mapper, and flash offset).
5. Hold BOOTSEL while connecting the PicoVerse 2350 to USB.
6. Copy the generated UF2 to the `RPI-RP2` drive.
7. Insert the cartridge into the MSX and power on.

---

## Output layout details

The UF2 image contains:

1. **Firmware blob** – embedded `loadrom` firmware.
2. **Configuration record** (59 bytes):
   - 50 bytes: ROM name (ASCII, padded/truncated).
   - 1 byte : mapper ID.
   - 4 bytes: ROM size (little-endian).
   - 4 bytes: ROM flash offset (little-endian).
3. **ROM payload** – raw ROM data appended after the config record.

The UF2 writer sets `UF2_FLAG_FAMILYID_PRESENT` and uses the RP2350 family ID (`0xE48BFF59`) so the bootloader accepts the image.

---

## Troubleshooting

| Symptom | Possible cause | Resolution |
| --- | --- | --- |
| "Invalid ROM size" | ROM < 8 KB or > 16 MB | Use a valid ROM size. |
| "Failed to detect the ROM type" | Mapper heuristics failed | Add a mapper tag (e.g., `.Konami.ROM`). |
| UF2 not recognized | Not in BOOTSEL, or wrong file | Enter BOOTSEL and copy the UF2 again. |
| Name truncated in menu | Filename too long | Shorten the filename. |

---

## Known limitations

- Only one ROM per UF2 (use the MultiROM or Explorer tools for multiple titles).
- Linux/macOS binaries are not provided (use Windows or build from source).
- The tool does not verify ROM integrity beyond size and mapper heuristics.
- Excessive flashing can wear out flash memory.

---

## Future improvements

- Cross-platform builds (Linux/macOS).
- Optional ROM integrity checks.
- GUI wrapper for mapper forcing.
- Additional mapper heuristics.

Author: Cristiano Almeida Goncalves  
Last updated: 02/15/2026
