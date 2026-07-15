# MSX PicoVerse 2350 LoadROM Tool Manual (EN-US)

The PicoVerse 2350 cartridge can boot either a MultiROM menu or a single ROM that starts immediately. The LoadROM tool creates a UF2 image that flashes the PicoVerse 2350 with a single ROM payload and the LoadROM firmware.

`loadrom.exe` bundles the Pico firmware blob, a configuration record (game name, mapper code, ROM size, and flash offset), and one MSX ROM file into an RP2350-compatible UF2 image. Copy the generated UF2 to the board in BOOTSEL mode and the MSX will boot straight into the embedded ROM.

For RP2350, the current LoadROM package is:

- `2350/software/loadrom.pio`.

> **Important:** SCC/SCC+ emulation options (`-scc`, `-sccplus`), the secondary AY-3-8910 dual PSG option (`-d`), and the MSX-MUSIC/YM2413 option (`-f` or `-fmpac`) are supported by the current RP2350 LoadROM package (`2350/software/loadrom.pio`). Exactly one on-cartridge audio mode can be active per UF2 image.

> **OPL4 / MoonSound:** The `-4` / `--opl4` option builds a dedicated, standalone OPL4 / YMF278B / MoonSound cartridge instead of a ROM loader. It is mutually exclusive with every other mode and does not take a ROM file. See [Standalone OPL4 / MoonSound cartridge](#standalone-opl4--moonsound-cartridge).

---

## Overview

1. **Input**: one `.ROM` file (case-insensitive extension).
2. **Processing**: the tool normalizes the ROM name, detects or forces the mapper, and streams firmware + config + ROM into UF2 blocks.
3. **Output**: a UF2 file (default `loadrom.uf2`) ready for the RP2350 bootloader.

Key characteristics:

- Windows console application.
- Supports ROM sizes from 8 KB up to 16 MB (subject to flash capacity).
- Mapper auto-detection plus optional filename tags to force the mapper.
- Embedded SHA-1 database derived from openMSX `softwaredb.xml` for improved mapper identification.
- Built-in Sunrise IDE Nextor ROM for standalone Nextor boot or Nextor + 1MB PSRAM mapper, using either the on-board microSD card (`-s1`/`-m1`) or a USB flash drive (`-s2`/`-m2`).
- Built-in Nextor + 1MB mapper + 1MB MegaRAM system images, using either microSD (`-r1`) or USB mass storage (`-r2`).
- Optional ESP-01 WiFi support for those Sunrise IDE modes via `-w`, adding an embedded ESP8266P system ROM and memio UART backend.
- ROM loading into the 1MB PSRAM mapper through Carnivore2-compatible RAM emulation for `SROM.COM /D15`, using either microSD (`-c1`) or USB mass storage (`-c2`) as the Nextor backend.
- UF2 uses RP2350 family ID (`0xE48BFF59`).

---

## Command-line usage

```
loadrom.exe [options] [romfile]
```

### Options

- `-h`, `--help` : Print usage information and exit.
- `-s1`, `--sunrise-sd` : Build a UF2 with the embedded Sunrise IDE Nextor ROM using the on-board microSD card slot for storage. No external ROM file is needed.
- `-m1`, `--mapper-sd` : Same as `-s1` plus a 1MB memory mapper (64 × 16KB pages) backed by external PSRAM on GPIO47 / QMI CS1 in an expanded sub-slot architecture. No external ROM file is needed.
- `-s2`, `--sunrise-usb` : Build a UF2 with the embedded Sunrise IDE Nextor ROM using the cartridge's USB-C port for storage (USB mass storage). No external ROM file is needed.
- `-m2`, `--mapper-usb` : Same as `-s2` plus a 1MB memory mapper (64 × 16KB pages) backed by external PSRAM on GPIO47 / QMI CS1 in an expanded sub-slot architecture. No external ROM file is needed.
- `-c1`, `--carnivore2-sd` : Build a UF2 with Sunrise IDE Nextor on microSD plus the 1MB PSRAM mapper and Carnivore2 RAM-mode emulation for `SROM.COM /D15`. In this mode, `SROM.COM` can upload a ROM image into the PSRAM-backed mapper and launch it from RAM without reflashing the cartridge.
- `-c2`, `--carnivore2-usb` : Same as `-c1`, but uses USB mass storage on the cartridge's USB-C port as the Nextor backend.
- `-r1`, `--megaram-sd` : Build a UF2 with Sunrise IDE Nextor on microSD, a 1MB MSX memory mapper, and a separate 1MB PSRAM-backed MegaRAM subslot using 128 × 8KB banks. No external ROM file is needed.
- `-r2`, `--megaram-usb` : Same as `-r1`, but uses USB mass storage on the cartridge's USB-C port as the Nextor backend.
- `-w`, `--wifi` : Enable ESP-01 WiFi support for `-s1`, `-m1`, `-s2`, or `-m2`. This adds the ESP8266P system ROM plus a memory-mapped UART backend expected by the WiFi ROM/software stack.
- `-scc`, `--scc` : Enable SCC (standard) sound emulation. For embedded ROM builds, this applies to Konami SCC or Manbow2 mapper ROMs. With `-c1` or `-c2`, it enables SCC playback for Konami SCC ROMs uploaded later through `SROM.COM /D15`.
- `-sccplus`, `--sccplus` : Enable SCC+ (enhanced) sound emulation. For embedded ROM builds, this applies to Konami SCC or Manbow2 mapper ROMs. With `-c1` or `-c2`, it enables SCC+ playback for compatible ROMs uploaded later through `SROM.COM /D15`.
- `-d`, `--dual-psg` : Enable secondary AY-3-8910 (PSG) emulation on I/O ports `0x10` (register select) and `0x11` (data). The Pico captures `OUT (0x10/0x11),A` writes via PIO1 and streams a mixed signal to the I2S DAC alongside the host MSX's PSG. Only valid with external ROM files whose mapper is not Konami SCC or Manbow2 (those mappers carry an on-cartridge SCC chip and reserve the second audio slot for SCC emulation).
- `-f`, `-fmpac` : Enable MSX-MUSIC / YM2413 emulation on I/O ports `0x7C` (register select) and `0x7D` (data). The Pico captures OPLL writes via PIO1 and streams the generated FM audio to the I2S DAC. The UF2 also embeds the FM-PAC BIOS and exposes it in an expanded FM-PAC subslot so ROMs that use MSX-MUSIC BIOS calls can find it. Only valid with non-SYSTEM external ROM files.
- `-4`, `--opl4` : Build a dedicated, standalone OPL4 / YMF278B / MoonSound cartridge. This is **not** a ROM loader: the UF2 contains only the OPL4 firmware and the 2 MB YRW801-M wave ROM, turning the cartridge into a MoonSound-compatible sound device (2 MB wave ROM + 2 MB PCM sample RAM) on the standard MoonSound I/O ports. It does not take a ROM file and is mutually exclusive with every other option.
- `--22khz` : With `-4` / `--opl4`, build the dedicated OPL4 firmware variant that renders and outputs at 22050 Hz instead of the default 44100 Hz.
- `-o <filename>`, `--output <filename>` : Override the UF2 output name (default `loadrom.uf2`).
- Positional argument: the ROM file to embed. Required for normal ROM loading; not accepted with `-s1`/`-m1`/`-s2`/`-m2`/`-c1`/`-c2`/`-r1`/`-r2`.

`-s1`, `-m1`, `-s2`, `-m2`, `-c1`, `-c2`, `-r1`, and `-r2` are mutually exclusive. `-w` is valid only with `-s1`, `-m1`, `-s2`, or `-m2`. The audio options `-scc`, `-sccplus`, `-d`, and `-f`/`-fmpac` are mutually exclusive — only one on-cartridge audio engine can be active per UF2 image. `-d` is additionally rejected for Konami SCC and Manbow2 ROMs, while `-d` and `-f`/`-fmpac` are rejected for the embedded Sunrise/Carnivore2/MegaRAM system modes. `-f`/`-fmpac` is also rejected for Konami SCC and Manbow2 mapper ROMs. If conflicting options are provided, the tool exits with an error.

`-4`/`--opl4` is fully standalone: it cannot be combined with any other option (no Sunrise/Carnivore2 mode, no audio flag, no `-w`, and no ROM file). If `-4` is mixed with any of those, the tool exits with an error.

For the firmware architecture behind `-d`, see the [PicoVerse 2350 Dual PSG implementation guide](./msx-picoverse-2350-dualpsg.md).

For the firmware architecture behind `-r1` / `-r2`, see the [PicoVerse 2350 MegaRAM implementation guide](./msx-picoverse-2350-megaram.md).

When `-f` or `-fmpac` is selected, LoadROM keeps the selected game mapper active and adds the FM-PAC BIOS/register area in the cartridge's expanded slot layout. The Pico responds to both direct YM2413 I/O writes and FM-PAC memory-mapped register writes at `0x7FF4`/`0x7FF5`.

For the firmware architecture behind `-f` / `-fmpac`, see the [PicoVerse 2350 MSX-MUSIC / FM-PAC implementation guide](./msx-picoverse-2350-fmpac.md).

### Standalone OPL4 / MoonSound cartridge

The `-4` / `--opl4` option produces a firmware-only UF2 that turns the PicoVerse 2350 into a dedicated **OPL4 (Yamaha YMF278B) / MoonSound** sound cartridge. Unlike every other LoadROM mode, it does not embed an MSX ROM and does not load any game — the cartridge is purely a MoonSound-compatible sound device.

What the cartridge provides:

- Full **YMF278B**: OPL3 FM (18 channels) plus the 24-voice PCM wavetable engine.
- The **2 MB YRW801-M** wave ROM (embedded in the UF2) and **2 MB of PCM sample RAM**.
- The standard MoonSound host I/O ports: `0x7E`/`0x7F` (wavetable) and `0xC4`–`0xC7` (FM).
- 16-bit stereo audio at 44.1 kHz through the on-cartridge I2S DAC.

Because `-4` is standalone, it cannot be combined with any Sunrise/Carnivore2 mode, any other audio flag (`-scc`, `-sccplus`, `-d`, `-f`/`-fmpac`), `-w`, or a ROM file. The only other accepted option is `-o` to rename the output.

Usage:

```
loadrom.exe -4                       # writes loadrom.uf2
loadrom.exe -4 -o moonsound.uf2      # custom output name
loadrom.exe -4 --22khz -o moonsound-22k.uf2
```

Flash the resulting UF2 in BOOTSEL mode, then use any MoonSound software on the MSX — for example MoonTest (detection and RAM test), SETOPL4 (sample upload/playback), and MoonBlaster / MBWAVE (music playback). No microSD, USB drive, or ESP-01 is required for this mode.

### Mapper forcing via filename tags

Append a dot-separated mapper tag before the `.ROM` extension to override detection. Tags are case-insensitive.

Supported tags:
`PLA-16`, `PLA-32`, `KonSCC`, `PLN-48`, `PLN-64`, `ASC-08`, `ASC-16`, `ASC-16X`, `Konami`, `NEO-8`, `NEO-16`, `MANBW2`.

Additional aliases are accepted for backward compatibility: `PL-16`, `PL-32`, `PL-48`, `PL-64`, `PLN-32`, `PLANAR`, `LINEAR`, `LINEAR0`, `PLANAR48`, `PLANAR64`, `MANBOW2`, `MBW-2`.

Example:

```
Penguin Adventure.PL-32.ROM
Space Manbow.KonSCC.rom
```

Tags are case-insensitive. If no valid tag is present, the tool first computes the ROM's SHA-1 hash and looks it up in an embedded database derived from the openMSX `softwaredb.xml`. When a match is found the database mapper type is used directly. Otherwise the tool falls back to heuristic detection.

`SYSTEM` is ignored and cannot be forced.

---

## Typical workflow

1. Place `loadrom.exe` and your ROM file in a working folder.
2. Open a Command Prompt or PowerShell window in that folder.
3. Run the tool:
   ```
   loadrom.exe "Space Manbow.rom" -o space_manbow.uf2
   ```
   Sunrise IDE standalone (Nextor from microSD card):
   ```
   loadrom.exe -s1
   loadrom.exe -s1 -o nextor_sd.uf2
   ```
   Sunrise IDE standalone with WiFi (microSD):
   ```
   loadrom.exe -s1 -w
   loadrom.exe -s1 -w -o nextor_sd_wifi.uf2
   ```
   Sunrise IDE + 1MB PSRAM mapper (microSD):
   ```
   loadrom.exe -m1
   loadrom.exe -m1 -o nextor_mapper_sd.uf2
   ```
   Sunrise IDE + 1MB PSRAM mapper with WiFi (microSD):
   ```
   loadrom.exe -m1 -w
   loadrom.exe -m1 -w -o nextor_mapper_sd_wifi.uf2
   ```
   Sunrise IDE standalone (Nextor from USB flash drive):
   ```
   loadrom.exe -s2
   loadrom.exe -s2 -o nextor_usb.uf2
   ```
   Sunrise IDE standalone with WiFi (USB):
   ```
   loadrom.exe -s2 -w
   loadrom.exe -s2 -w -o nextor_usb_wifi.uf2
   ```
   Sunrise IDE + 1MB PSRAM mapper (USB):
   ```
   loadrom.exe -m2
   loadrom.exe -m2 -o nextor_mapper_usb.uf2
   ```
   Sunrise IDE + 1MB PSRAM mapper with WiFi (USB):
   ```
   loadrom.exe -m2 -w
   loadrom.exe -m2 -w -o nextor_mapper_usb_wifi.uf2
   ```
   Carnivore2 RAM-mode loader for `SROM.COM /D15` (microSD):
   ```
   loadrom.exe -c1
   loadrom.exe -c1 -o srom_c2_sd.uf2
   ```
   Carnivore2 RAM-mode loader for `SROM.COM /D15` (USB):
   ```
   loadrom.exe -c2
   loadrom.exe -c2 -o srom_c2_usb.uf2
   ```
  Sunrise IDE + 1MB PSRAM mapper + 1MB MegaRAM (microSD):
  ```
  loadrom.exe -r1
  loadrom.exe -r1 -o nextor_megaram_sd.uf2
  ```
  Sunrise IDE + 1MB PSRAM mapper + 1MB MegaRAM (USB):
  ```
  loadrom.exe -r2
  loadrom.exe -r2 -o nextor_megaram_usb.uf2
  ```
   Carnivore2 RAM-mode loader with SCC audio for SROM-loaded Konami SCC ROMs:
   ```
   loadrom.exe -c1 -scc -o srom_c2_sd_scc.uf2
   loadrom.exe -c2 -scc -o srom_c2_usb_scc.uf2
   ```
   SCC/SCC+ examples:
   ```
   loadrom.exe -scc "Space Manbow.rom"
   loadrom.exe -sccplus "Snatcher.KonSCC.rom"
   ```
   Dual PSG example (secondary AY-3-8910 on ports `0x10`/`0x11`):
   ```
   loadrom.exe -d "Penguin Adventure - 2 x PSG - Darky.rom"
   ```
   MSX-MUSIC example (YM2413 on ports `0x7C`/`0x7D`):
   ```
   loadrom.exe -f "Game with MSX-MUSIC.rom"
   loadrom.exe -fmpac "Game with FM music.rom"
   ```
   OPL4 / MoonSound standalone cartridge (no ROM file):
   ```
   loadrom.exe -4
   loadrom.exe -4 -o moonsound.uf2
   ```
4. Review the console output (name, size, mapper, and flash offset).
5. Hold BOOTSEL while connecting the PicoVerse 2350 to USB.
6. Copy the generated UF2 to the `RPI-RP2` drive.
7. Insert the cartridge into the MSX and power on.
8. For `-s1`/`-m1`/`-r1` modes, insert a FAT-formatted microSD card into the cartridge's microSD slot before powering the MSX.
9. For `-s2`/`-m2`/`-r2` modes, connect a USB flash drive (via OTG adapter if needed) to the cartridge's USB-C port before powering the MSX.
10. For `-w` builds, install the ESP-01 module on the PicoVerse 2350 before power-up. The WiFi system ROM and memio UART interface are then available to compatible MSX software.
11. For `-c1`/`-c2` modes, boot Nextor first and then use `SROM.COM /D15` to upload the ROM into the PicoVerse PSRAM mapper. The cartridge is presented as a Carnivore2-compatible RAM target so the uploaded ROM can be launched directly from RAM.
12. For `-r1`/`-r2` modes, boot Nextor first and use a MegaRAM-aware loader or workflow that enables writes with `IN (0x8E)` or `IN (0x8F)`, loads data into the MegaRAM banks, disables writes with `OUT (0x8E)` or `OUT (0x8F)`, and launches software from the MegaRAM subslot.

---

## Output layout details

The UF2 image contains:

1. **Firmware blob** – embedded `loadrom` firmware.
2. **Configuration record** (59 bytes):
   - 50 bytes: ROM name (ASCII, padded/truncated).
   - 1 byte : mapper ID plus optional audio/WiFi flags. Normal ROM mapper IDs use the low nibble; embedded system modes use base IDs 10, 11, and 15-20 after the firmware masks off option flags:
     - Bit 7 (`0x80`) = SCC emulation enabled.
     - Bit 6 (`0x40`) = SCC+ emulation enabled.
     - Bit 5 (`0x20`) = WiFi support enabled for Sunrise IDE modes, or MSX-MUSIC/YM2413 enabled for non-SYSTEM ROMs.
     - Bit 4 (`0x10`) = Dual PSG emulation enabled (secondary AY-3-8910 on ports `0x10`/`0x11`).
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
| "Sunrise options are mutually exclusive" | More than one of `-s1`/`-m1`/`-s2`/`-m2`/`-c1`/`-c2`/`-r1`/`-r2` passed | Use only one firmware mode at a time. The `-m` variants add mapper RAM; the `-c` variants add Carnivore2 RAM-mode loading; the `-r` variants add a separate MegaRAM subslot. |
| "Sunrise options do not accept an external ROM file" | ROM file passed with a Sunrise/Carnivore2/MegaRAM option | Remove the ROM file argument when using `-s1`/`-m1`/`-s2`/`-m2`/`-c1`/`-c2`/`-r1`/`-r2`. |
| "Error: -w/--wifi is supported only with -s1, -m1, -s2 or -m2" | `-w` was used without a supported Sunrise mode | Pair `-w` only with `-s1`, `-m1`, `-s2`, or `-m2`. |
| USB pendrive not detected with `-s2`/`-m2`/`-r2` | VBUS not connected or no OTG adapter | Ensure the USB-C port has VBUS power (use an OTG adapter that supplies VBUS). |
| microSD card not detected with `-s1`/`-m1`/`-r1` | Card not inserted or not FAT-formatted | Insert a FAT16/FAT32-formatted microSD card before powering on. |
| WiFi software does not detect the adapter with `-w` | Missing ESP-01 module, incompatible ESP firmware, or wrong WiFi-capable UF2 | Reflash a `-w` build, verify the ESP-01 is installed, and use software targeting the ESP8266P memio interface. |
| `SROM.COM /D15` does not list the cartridge with `-c1`/`-c2` | Wrong firmware mode or outdated build | Reflash with the correct `-c1` or `-c2` UF2 and use a build that includes Carnivore2 RAM-mode emulation. |
| `SROM.COM /D15` upload succeeds but the launched ROM fails | ROM/runtime compatibility issue in the emulation path | Try the latest firmware build and retest; this mode depends on Carnivore2-compatible bank/window presentation from PSRAM. |
| "Disk driver not found. System halted." with `-m1`/`-m2` | Firmware issue | Rebuild with latest firmware; check that the PIO bus init guard is present. |
| "Warning: -scc flag ignored" | ROM is not Konami SCC or Manbow2 mapper, or the selected Nextor mode is not `-c1`/`-c2` | Use a Konami SCC or Manbow2 ROM, or pair `-scc` with `-c1`/`-c2` for SROM-loaded Konami SCC titles. |
| "Warning: -sccplus flag ignored" | ROM is not Konami SCC or Manbow2 mapper, or the selected Nextor mode is not `-c1`/`-c2` | Use a compatible ROM, or pair `-sccplus` with `-c1`/`-c2` for SROM-loaded SCC+ titles. |
| "Error: -scc and -sccplus are mutually exclusive" | Both options were passed together | Use only one of the two options. |
| `-4`/`--opl4` rejected or errors out | `-4` was combined with another mode, an audio flag, `-w`, or a ROM file | Use `-4` (optionally with `-o`) on its own; it is a standalone MoonSound build and accepts no other options or ROM file. |
| MoonSound software does not detect the cartridge with `-4` | Wrong UF2 or board without the OPL4 audio/`/INT` wiring | Reflash a `-4` build and use a PicoVerse 2350 with the I2S DAC and `/INT` (GPIO40) connected. |
| UF2 not recognized | Not in BOOTSEL, or wrong file | Enter BOOTSEL and copy the UF2 again. |
| Name truncated in menu | Filename too long | Shorten the filename. |

---

## Known limitations

- Only one ROM per UF2 (use the MultiROM or Explorer tools for multiple titles).
- The `-s1`, `-m1`, and `-r1` options require a FAT-formatted microSD card in the cartridge's microSD slot.
- The `-s2`, `-m2`, and `-r2` options require a USB flash drive connected to the cartridge's USB-C port (via OTG adapter) for disk access.
- The `-w` option is available only with `-s1`, `-m1`, `-s2`, or `-m2`.
- `-w` requires an ESP-01 module and compatible ESP-side firmware; the PicoVerse firmware provides the ROM mapping and serial transport, not the ESP application layer itself.
- The `-m1` and `-m2` options provide 1MB mapper RAM (64 × 16KB pages) backed by external PSRAM on GPIO47 / QMI CS1.
- The `-r1` and `-r2` options provide that same 1MB mapper RAM plus a separate 1MB MegaRAM surface with 128 x 8KB banks, four visible windows in `0x4000`-`0xBFFF`, Cartucho II-compatible bank latches selected by address bits A14:A13, `IN (0x8E/0x8F)` write enable, and `OUT (0x8E/0x8F)` write disable.
- The `-c1` and `-c2` options reuse that same 1MB PSRAM-backed mapper RAM and expose it through Carnivore2-compatible RAM-mode behavior intended for `SROM.COM /D15`.
- The `-w` Sunrise IDE WiFi system-ROM option is not currently exposed in Explorer or the `-c1`/`-c2` Carnivore2 loader modes. MultiROM supports `-w` for `-s1`, `-m1`, `-s2`, and `-m2` Nextor entries. Explorer has its own built-in ESP-01 path for File Hunter browsing and WiFi setup.
- The `-c1` and `-c2` modes are loader modes, not direct ROM-embedding modes. They boot Nextor first; the ROM is uploaded later from DOS into PSRAM.
- SCC/SCC+ audio for `SROM.COM /D15` uploads is enabled by building the loader UF2 with `-c1/-c2` plus `-scc` or `-sccplus` before flashing the cartridge.
- Linux/macOS binaries are not provided (use Windows or build from source).
- The tool does not verify ROM integrity beyond size and mapper heuristics.
- SCC/SCC+ flags are applied for Konami SCC mapper (type 3) and Manbow2 mapper (type 14) ROMs; otherwise they are ignored with a warning.
- SCC/SCC+ emulation is available in the current RP2350 LoadROM package.
- The `-4` / `--opl4` OPL4 / MoonSound build is a standalone sound cartridge: it embeds no game ROM, ignores all other options, and overclocks the RP2350 to 300 MHz. It requires the I2S DAC and the `/INT` line (GPIO40) on the PicoVerse 2350 board.
- Excessive flashing can wear out flash memory.

---

## Future improvements

- Cross-platform builds (Linux/macOS).
- Optional ROM integrity checks.
- GUI wrapper for mapper forcing.
- Additional mapper heuristics.

Author: Cristiano Almeida Goncalves  
Last updated: 06/06/2026
