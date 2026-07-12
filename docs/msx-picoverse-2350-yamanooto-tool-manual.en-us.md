# MSX PicoVerse 2350 Yamanooto Tool Manual (EN-US)

The Yamanooto tool (`yamanooto.exe`) creates a UF2 image that flashes the PicoVerse 2350 with the
**Yamanooto** flash-cartridge emulation and a single ROM image. The cartridge then behaves like a
 *Yamanooto*: a Konami-SCC compatible flash cartridge with **SCC / SCC+** audio and a
**secondary (dual) PSG**, plus the PicoVerse additions of an **MSX-MUSIC (YM2413 / FM-PAC)** engine
and a **primary PSG mirror**.

`yamanooto.exe` bundles the Pico firmware blob, a configuration record (game name, type, ROM size,
and offset), the MSX ROM image, and the FM-PAC BIOS into an RP2350-compatible UF2 image. Copy the
generated UF2 to the board in BOOTSEL mode and the MSX boots straight into the embedded ROM.

Package location: `2350/software/yamanooto`.

For the firmware architecture behind this tool, see the
[PicoVerse 2350 Yamanooto implementation guide](./msx-picoverse-2350-yamanooto.md).

---

## Overview

1. **Input**: one `.ROM` file — a Konami-SCC / Konami-4 compatible image up to 8 MB.
2. **Processing**: the tool derives the ROM name from the filename and streams
   firmware + configuration record + ROM + FM-PAC BIOS into UF2 blocks.
3. **Output**: a UF2 file (default `yamanooto.uf2`) ready for the RP2350 bootloader.

Key characteristics:

- Windows console application.
- ROM sizes from 8 KB up to 8 MB (the Yamanooto flash size).
- SCC/SCC+, the secondary (dual) PSG, the primary PSG mirror, and MSX-MUSIC/FM-PAC are **always
  available** — no per-mode flags. The firmware selects SCC, FM, or pure PSG **on the fly** based on
  what the running game drives.
- The English FM-PAC BIOS is always embedded and appended after the ROM image.
- UF2 uses the RP2350 family ID (`0xE48BFF59`).

---

## Command-line usage

```
yamanooto.exe [options] <rom_file>
```

### Options

- `-h`, `--help` : Print usage information and exit.
- `-o <filename>`, `--output <filename>` : Override the UF2 output name (default `yamanooto.uf2`).
- Positional argument: the ROM image to embed (required).

There are no audio-mode flags: SCC/SCC+, dual PSG, PSG mirror, and MSX-MUSIC are all present in every
build and chosen automatically at runtime.

### Examples

```
:: Default output name
yamanooto.exe MyKonamiGame.rom

:: Custom output name
yamanooto.exe -o mygame.uf2 MyKonamiGame.rom
```

The tool prints the ROM name, ROM size, and output filename before writing the UF2. It rejects files
smaller than 8 KB or larger than 8 MB.

---

## Workflow

1. Open a Command Prompt or PowerShell window in `2350/software/yamanooto/tool` (the tool also accepts
   drag-and-drop of a ROM onto the EXE).
2. Run `yamanooto.exe -o mygame.uf2 \path\to\Game.ROM`.
3. Confirm the reported ROM name and size.
4. Put the Pico into BOOTSEL mode (hold BOOTSEL while connecting via USB-C) and copy the generated
   UF2 to the `RPI-RP2` drive.
5. Insert the cartridge into your MSX — on power-up the embedded game launches.

---

## Audio behaviour

- **SCC / SCC+** — Konami SCC games are detected and played automatically (SCC when bank 2 = `0x3F`,
  SCC+ via the enhanced mode register).
- **Secondary (dual) PSG** — a second AY-3-8910 on I/O ports `0x10`/`0x11`.
- **Primary PSG mirror** — the main MSX PSG (`0xA0`/`0xA1`) is mirrored through the cartridge DAC.
- **MSX-MUSIC / FM-PAC** — the FM-PAC BIOS is exposed in an expanded subslot so FM games detect the
  YM2413; the OPLL responds to both the FM-PAC memory registers (`0x7FF4`/`0x7FF5`) and the raw I/O
  ports (`0x7C`/`0x7D`).
- The firmware renders **one heavy engine at a time** (SCC *or* FM), chosen live from the running
  game, and mixes the PSG on top. A single flash image can therefore contain a mix of SCC games, FM
  games, and plain PSG games.

Audio is 16-bit stereo at 44.1 kHz through the on-cartridge I2S DAC.

---

## Limitations

- **Game-save flash commands are supported.** Yamanooto software can use `WREN` with standard AMD
  single-byte programming, 32-byte write-buffer programming, sector erase, chip erase, CFI query and
  autoselect sequences. Saves persist in a 4 MB board-flash journal and are restored when the same ROM
  image boots; repeated saves compact into their latest changed pages. The journal holds up to 2 MB of
  distinct changed pages and does not support bulk on-MSX rewriting of the entire cartridge image.
- **PicoVerse 2350 only.** The Yamanooto firmware requires the RP2350B board, dual cores, and the I2S
  DAC; it is not available for the PicoVerse 2040.
- With MSX-MUSIC always present, the machine performs one extra reset at cold boot (during loading)
  while the expanded slot is set up.

---

## UF2 image layout

The generated UF2 contains, in order: the Yamanooto firmware, a 59-byte configuration record
(50-byte name + 1-byte type + 4-byte size + 4-byte offset), the ROM image, and the 64 KB FM-PAC BIOS.
The firmware locates the ROM immediately after the configuration record and the FM-PAC BIOS
immediately after the ROM.
