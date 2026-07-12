# MSX PicoVerse 2350 Yamanooto Implementation

The **Yamanooto** emulation turns the PicoVerse 2350 cartridge into a *Yamanooto*-style
flash cartridge: a Konami-SCC compatible 8 MB flash-ROM with **SCC / SCC+** audio and a **secondary
(dual) PSG**. On top of the original hardware feature set, the PicoVerse implementation adds an
 **MSX-MUSIC (YM2413 / FM-PAC)** engine and a **primary PSG mirror**, and selects which
sound engine to render **on the fly** so a single flash image can hold a mix of SCC games, FM games,
and plain PSG games.

> This is a **PicoVerse 2350 only** firmware — it relies on the RP2350B's 48 GPIO, dual cores, and
> the I2S DAC. It is built and flashed with the dedicated Yamanooto PC tool (see the
> [Yamanooto Tool Manual](./msx-picoverse-2350-yamanooto-tool-manual.en-us.md)).

Package location: `2350/software/yamanooto`.

---

## References and credits

The implementation follows the behaviour documented in the openMSX Yamanooto device and reuses the
sound cores already used elsewhere in PicoVerse:

- **openMSX `Yamanooto.cc`** — the register map, mapper behaviour, bank/offset handling, SCC/SCC+
  activation rules, and FPGA ID handshake were modelled on the openMSX implementation.
  <https://github.com/openMSX/openMSX/blob/master/src/memory/Yamanooto.cc>
- **Genami "Yamanooto" hardware** — the emulated cartridge is the Genami Yamanooto flash cartridge
  (8 MB flash, SCC/SCC+, dual PSG). Product/behaviour reference:
  <https://genami.shop/blogs/news/programming-the-yamanooto>
- **emu2212** (Konami SCC / SCC+) — © Mitsutaka Okazaki, MIT License.
  <https://github.com/digital-sound-antiques/emu2212>
- **emu2149** (AY-3-8910 / YM2149 PSG) — © Mitsutaka Okazaki, MIT License.
  <https://github.com/digital-sound-antiques/emu2149>
- **emu2413** (YM2413 / OPLL) — © Mitsutaka Okazaki, MIT License.
  <https://github.com/digital-sound-antiques/emu2413>
- **FMPCCMFC.BIN** — the English FM-PAC BIOS embedded for the MSX-MUSIC support. The FM-PAC BIOS ROM
  is copyrighted by Matsushita Corp.; the English translation is credited to 232, Max Iwamoto, and GDX.

The PicoVerse Yamanooto firmware, the RP2350 bus/PIO handling, the runtime engine selector, and the
audio pipeline are original PicoVerse work.

---

## Feature summary

| Feature | Notes |
|---|---|
| Flash-ROM image | Up to 8 MB, served through the Yamanooto mapper (10-bit bank registers). |
| Konami-SCC mapper | Default mapper: 4 × 8 KB banks, switched by writes in the `0x4000`-`0xBFFF` window. |
| Konami-4 mapper | Selected by the `CFGR` `K4` bit; bank 0 fixed, banks switch on writes ≥ `0x6000`. |
| Bank offset | Global offset from `OFFR` + `CFGR` sub-offset bits, added to every bank register. |
| Register interface | Memory-mapped at `0x7FFC`-`0x7FFF` (`ENAR`/`OFFR`/`CFGR`/FPGA channel). |
| SCC / SCC+ | Auto-detected from the bank registers (emu2212, ENHANCED mode). |
| Secondary (dual) PSG | Second AY-3-8910 on I/O ports `0x10`/`0x11` (emu2149). |
| PSG mirror | Mirrors the main MSX PSG (`0xA0`/`0xA1`) through the cartridge DAC. |
| MSX-MUSIC / FM-PAC | Optional YM2413 (emu2413) exposed via an expanded FM-PAC subslot (always embedded). |
| Runtime engine select | SCC, FM, or pure PSG chosen automatically from what the running game drives. |
| Audio output | 16-bit stereo, 44.1 kHz through the on-cartridge I2S DAC. |

---

## Register interface (0x7FFC – 0x7FFF)

The Yamanooto exposes four memory-mapped registers at the top of page 1. These addresses are **not
mirrored** and are only visible for reads while the register interface is enabled (`ENAR` `REGEN`).

| Address | Name | Purpose |
|---|---|---|
| `0x7FFC` | `FPGA_REG` | FPGA communication channel (read-ID handshake). |
| `0x7FFD` | `CFGR` | Configuration register. |
| `0x7FFE` | `OFFR` | Bank offset register. |
| `0x7FFF` | `ENAR` | Enable register. |

**`ENAR` (0x7FFF)**

| Bit | Name | Meaning |
|---|---|---|
| `0x01` | `REGEN` | Register interface enable (registers become readable). |
| `0x10` | `WREN` | Enable the S29GL064N-compatible flash command interface. |

**`CFGR` (0x7FFD)**

| Bit | Name | Meaning |
|---|---|---|
| `0x01` | `MDIS` | Mapper disable. |
| `0x02` | `ECHO` | Mirror the secondary PSG onto the primary PSG ports `0xA0`/`0xA1`. |
| `0x04` | `ROMDIS` | ROM disable (reads return `0xFF`). |
| `0x08` | `K4` | Konami-4 mapper mode (disables SCC). |
| `0x30` | `SUBOFF` | Sub-offset bits (added to the bank offset). |
| `0x40` | `FPGA_EN` | Enable the FPGA command channel. |
| `0x80` | `FPGA_WAIT` | Read-back flag: FPGA ready (always reported ready). |

The FPGA channel implements just enough of the read-ID handshake to satisfy detection code: after a
`0x9F` command it returns the ID sequence `{0xFF, 0x1F, 0x23, 0x00, 0x00}`.

---

## Mapper behaviour

The default mapper is **Konami-SCC**: four 8 KB banks cover `0x4000`-`0xBFFF`. A bank register is
written when the CPU writes to an address matching `(address & 0x1800) == 0x1000` — i.e.
`0x5000`-`0x57FF`, `0x7000`-`0x77FF`, `0x9000`-`0x97FF`, `0xB000`-`0xB7FF`.

- **Bank offset**: `offset = (OFFR << 2) | ((CFGR & SUBOFF) >> 4)`. Each stored bank register is
  `(written_value + offset) & 0x3FF` (10 bits, so up to 8 MB / 1024 × 8 KB banks). The raw written
  value is also kept because SCC/SCC+ activation depends on it.
- **Flash address**: `page8kB = (address >> 13) - 2`, `flash_addr = (bankRegs[page8kB] << 13) |
  (address & 0x1FFF)`. Reads beyond the ROM image return `0xFF`.
- **Address mirroring**: `0x4000 ↔ 0xC000` and `0x8000 ↔ 0x0000`, matching the openMSX model (the
  real hardware ignores the upper address bit). In-slot accesses are already inside
  `0x4000`-`0xBFFF`.
- **Konami-4** (`CFGR` `K4` set): bank 0 (`0x4000`-`0x5FFF`) is fixed; the remaining banks switch on
  writes at or above `0x6000`. SCC is disabled in this mode.
- **`MDIS`** disables bank switching; **`ROMDIS`** makes flash reads return `0xFF`.

---

## SCC / SCC+ audio

SCC/SCC+ is emulated with **emu2212** in ENHANCED mode. Every mapper-region write is fed to the SCC
engine, which reproduces the real Konami SCC detection:

- **SCC** activates when bank register 2 is written with `0x3F`; the register window is
  `0x9800`-`0x9FFF`.
- **SCC+** activates when bank register 3 has bit 7 set and the mode register is enabled; the window
  moves to `0xB800`-`0xBFFF`.
- The SCC+ mode register at `0xBFFE` selects between compatible (SCC) and enhanced (SCC+) modes.

SCC is disabled while `CFGR` `K4` is set.

---

## Secondary (dual) PSG, PSG mirror and ECHO

The Yamanooto's second AY-3-8910 is emulated with **emu2149** on I/O ports `0x10` (register select)
and `0x11` (data). In addition, the PicoVerse implementation mirrors the **main MSX PSG**
(`0xA0`/`0xA1`) through the cartridge DAC (a separate PSG instance), so the system PSG is heard on
the same audio output as the SCC/FM.

- **`ECHO`** (`CFGR` bit `0x02`): when set, the secondary PSG also follows writes to the primary PSG
  ports `0xA0`/`0xA1` (the Yamanooto "echo" behaviour), in addition to the always-on primary PSG
  mirror.
- The PSG output is **AC-coupled** with a ~35 Hz DC blocker (the AY/YM output is unipolar and carries
  a DC offset that would otherwise thump on note transitions), matching how real MSX audio is coupled
  through a series capacitor.

---

## MSX-MUSIC / FM-PAC (expanded slot)

MSX-MUSIC is a PicoVerse extension (the physical Yamanooto has no FM chip). Because many FM games
detect the OPLL through the FM-PAC BIOS, the cartridge is always presented as an **expanded slot**:

- **Sub-slot 0** — the Yamanooto game (full mapper, registers, SCC, PSG).
- **Sub-slot 3** — the 64 KB FM-PAC BIOS at page 1 (`0x4000`-`0x7FFF`), so the `PAC2OPLL` signature is
  found during the boot slot scan.

The OPLL (emu2413, YM2413 tone set) responds both to the FM-PAC memory-mapped registers
(`0x7FF4` = register, `0x7FF5` = data) and to the raw I/O ports (`0x7C`/`0x7D`). The FM-PAC surface
also provides the control register (`0x7FF6`), bank register (`0x7FF7`), and 8 KB battery SRAM
(unlocked by writing `0x4D`/`0x69` to `0x5FFE`/`0x5FFF`).

At cold boot the firmware serves a tiny bootstrap ROM whose INIT sets the boot flag and restarts the
machine, so the expanded slot is presented cleanly on the second boot with audio and PIO fully
initialised (mirrors the PicoVerse LoadROM/Explorer FM-PAC hybrid loaders).

The FM-PAC BIOS is **always embedded and appended** after the ROM image by the tool — there is no
separate enable flag.

---

## Runtime engine selection and the audio pipeline

The SCC (5 voices) and the FM/OPLL (9 voices) are each too heavy to render together every sample on
the audio core, so **only one heavy engine is rendered at a time**. Because a single flash image can
hold SCC games, FM games, and pure PSG games, the choice is made **on the fly** from what the running
program is actually driving — it is never latched:

- **FM** is rendered while the OPLL has a voice keyed on, or was written within the last ~0.5 s
  (covers note release tails).
- **SCC** is rendered while the SCC is enabled.
- **Otherwise** neither heavy engine runs and the output is pure PSG.

This avoids, for example, rendering the idle OPLL (which the FM-PAC BIOS touches once at boot) as a
constant tone under a pure-PSG game.

The PSG is handled differently depending on context (mirroring the two Explorer PSG paths):

- When **FM** is the primary source, the PSGs are **audible-gated** — an idle PSG skips its
  per-sample work so the OPLL render stays within the core budget; the gate transitions are masked by
  the FM and the DC blocker.
- When the PSG is the primary source (pure PSG or SCC games), both PSGs are **clocked and mixed every
  sample** (never dropped), so there are no phase freezes or mute/unmute clicks.

Final conditioning:

- The FM path uses a DC blocker, a gentle ~16 kHz low-pass (softens the YM2413 exponential-DAC
  quantization), and a soft-knee limiter on the FM + PSG mix instead of hard clipping.
- The SCC path uses the SCC + PSG mix with a hard clamp.
- The PSG mix is AC-coupled by its own DC blocker before mixing.

### Core / PIO layout

- **Core 0** runs the MSX memory bus on PIO0 (SM0 read responder with `/WAIT`, SM1 write captor) and
  handles the mapper, registers, SCC writes, subslot routing, and FM-PAC memory-register writes
  (queued to core 1 through a lock-free ring).
- **Core 1** runs the I2S audio (PIO0 SM2), services the secondary-PSG / primary-PSG / OPLL I/O bus
  on PIO1, drains the FM register ring, and renders the selected engine + PSG mix.
- System clock 210 MHz; board definition `pimoroni_pga2350` (RP2350B, 48 GPIO).

---

## Configuration record and UF2 layout

The tool builds a UF2 (RP2350 family ID `0xE48BFF59`, 256-byte payloads) containing, in order:

1. the Yamanooto firmware,
2. a 59-byte configuration record — 50-byte name, 1-byte type, 4-byte ROM size, 4-byte offset,
3. the ROM image (up to 8 MB),
4. the 64 KB FM-PAC BIOS.

The firmware finds the ROM image right after the configuration record and the FM-PAC BIOS right after
the ROM image (`rom + record + rom_size`).

---

## Limitations

- **Game saves use the emulated flash.** `ENAR` `WREN` implements the Yamanooto S29GL064N command
  interface used by save-capable software: AMD single-byte programming, 32-byte write-buffer programming,
  sector erase, chip erase, CFI query, autoselect IDs, and reset-to-read mode. The mutable 8 MB image
  lives in PSRAM while running; changed pages are retained across power cycles in a 4 MB board-flash
  journal. Repeated saves compact automatically into their latest changed pages. The persistence region
  can retain up to 2 MB of distinct changed pages, so it is designed for game-state saves rather than
  bulk on-MSX rewriting of the complete cartridge image.
- **One heavy engine at a time.** SCC and FM are never rendered simultaneously. Real games never use
  both at once, so the runtime selector picks the one in use; a title that expected both SCC and FM
  together would only hear the currently-driven one.
- **MSX-MUSIC uses the FM-PAC BIOS** (`PAC2OPLL`), which covers FM-PAC-aware software; it is the same
  BIOS used by the PicoVerse LoadROM/Explorer FM-PAC modes.
- With MSX-MUSIC always on, the cold-boot expanded-slot bootstrap performs one extra machine reset at
  power-up (during loading, before any game audio).
