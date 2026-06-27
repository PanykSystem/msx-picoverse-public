# MSX PicoVerse 2350 PIO Bus Strategy

This document describes the RP2350 PIO strategy currently used in:

- `2350/software/loadrom.pio`
- `2350/software/multirom.pio`
- `2350/software/explorer.pio`

The 2350 targets share the same PIO bus engine, I/O bus engine, and audio architecture. MultiROM extends LoadROM by embedding multiple ROMs with a menu, while Explorer adds microSD browsing, MP3 playback, File Hunter, and per-ROM audio selection. The mapper set covers types 1-16 including Sunrise IDE, SCC/SCC+, Manbow2, and ASCII16-X.

---

## 1) Core Strategy

| Owner | Responsibility |
|---|---|
| **PIO bus engine (`pio0`)** | Detect `/SLTSL` + `/RD`/`/WR`, assert `/WAIT`, capture writes, drive/release D0..D7 |
| **PIO I/O bus engine (`pio1`, mapper mode)** | Capture `/IORQ` + `/RD`/`/WR` for mapper page register access (ports FC–FF) |
| **PIO I2S audio (`pio1`, audio mode)** | Generate I2S bitstream for SCC/SCC+, Dual PSG, MSX-MUSIC, MP3, and Explorer primary PSG mirror output via DAC |
| **CPU (core 0)** | Mapper translation, ROM lookup, SCC/SCC+ register forwarding, Sunrise IDE register handling, Explorer menu/File Hunter control |
| **CPU (core 1, audio mode)** | Audio generation for SCC/SCC+ via emu2212, PSG via emu2149, MSX-MUSIC via emu2413, MP3 decoding, and I2S output |
| **CPU (core 1, Sunrise mode)** | TinyUSB host stack — USB mass storage read/write for Sunrise IDE emulation |

The bus strategy is still PIO + FIFO tokenization, but RP2350 adds audio subsystem paths (SCC/SCC+, Dual PSG, MSX-MUSIC, MP3, and Explorer primary PSG mirroring), a Sunrise IDE path with USB host, and platform-specific GPIO/clock settings.

> **Note:** PIO1 is shared by the I/O bus captors and the I2S audio path. Explorer and the audio-enabled ROM modes keep the MSX memory bus on PIO0 and place I2S on PIO1 SM2 so I/O write capture and DAC streaming can coexist.

---

## 2) PIO Bus Architecture

`loadrom.pio` and `multirom.pio` use two state machines on `pio0` for MSX memory bus access:

| SM | Program | Role |
|---|---|---|
| `SM0` | `msx_read_responder` | Read-cycle responder with `/WAIT` side-set and tokenized data output |
| `SM1` | `msx_write_captor` | Write-cycle capture for mapper and SCC register writes |

In mapper mode (`-m`), two additional state machines on `pio1` handle I/O bus access:

| SM | Program | Role |
|---|---|---|
| `SM0` | `msx_io_read_responder` | I/O read-cycle responder for mapper page registers (FC–FF) |
| `SM1` | `msx_io_write_captor` | I/O write-cycle capture for mapper page registers (FC–FF) |

The implementation uses `jmp pin` polling to re-check slot/IORQ validity while waiting for read/write strobes, preventing stale-slot race behavior.

---

## 3) RP2350 Pin Map (PIO bus + extras)

### Bus pins

| GPIO | Signal |
|---|---|
| 0–15 | A0–A15 |
| 16–23 | D0–D7 |
| 24 | `/RD` |
| 25 | `/WR` |
| 26 | `/IORQ` |
| 27 | `/SLTSL` |
| 28 | `/WAIT` |
| 37 | `BUSSDIR` |
| 47 | `PSRAM select` |

### Audio pins (SCC/SCC+ mode)

| GPIO | Signal |
|---|---|
| 29 | I2S DATA |
| 30 | I2S BCLK |
| 31 | I2S WSEL/LRCLK |
| 32 | DAC MUTE control |

---

## 4) FIFO Contract

### Write captor (`SM1 -> CPU`)

- `bits[15:0]` = address
- `bits[23:16]` = data

### Read token (`CPU -> SM0`)

- `bits[7:0]` = data byte
- `bits[15:8]` = pin-direction mask (`0xFF` drive, `0x00` tri-state)

---

## 5) RP2350-Specific Runtime Details

### Clocking and flash timing

- Firmware sets QMI flash timing (`qmi_hw->m[0].timing = 0x40000202`).
- System clock is configured to **210 MHz**.

### ROM serving

- Uses a 256 KB SRAM cache with flash fallback for ROMs exceeding cache capacity.
- microSD-resident ROMs (Explorer firmware) up to 4 MB are streamed into the cartridge's external 8 MB PSRAM (QMI CS1, 52.5 MHz QPI). The leading window is mirrored into the same 256 KB SRAM cache used by flash ROMs, so the mapper hot-loop accesses go through fast SRAM regardless of source.
- GPIO input hysteresis is enabled on address (A0–A15) and data (D0–D7) pins to filter bus glitches.
- Mapper support includes Plain16/32, Planar48, Planar64, Konami SCC, Konami, ASCII8, ASCII16, ASCII16-X, NEO8, NEO16, Manbow2, Sunrise IDE, and Sunrise IDE + 1MB PSRAM Mapper.

### Sunrise IDE integration

The firmware supports Sunrise IDE modes selected by mapper type:

| Option | Type | Core 1 Backend | Storage |
|--------|------|----------------|---------|
| `-s1` | 15 | `sunrise_sd_task()` | microSD via SPI0 |
| `-m1` | 16 | `sunrise_sd_task()` | microSD via SPI0 |
| `-c1` | 17 | `sunrise_sd_task()` | microSD via SPI0 |
| `-r1` | 19 | `sunrise_sd_task()` | microSD via SPI0 |
| `-s2` | 10 | `sunrise_usb_task()` | USB via TinyUSB |
| `-m2` | 11 | `sunrise_usb_task()` | USB via TinyUSB |
| `-c2` | 18 | `sunrise_usb_task()` | USB via TinyUSB |
| `-r2` | 20 | `sunrise_usb_task()` | USB via TinyUSB |

For all modes:

- Core 0 handles the PIO memory bus, Sunrise IDE register emulation, and ROM banking.
- Core 1 runs the appropriate storage backend (USB or SD), translating ATA commands into block I/O.
- The embedded Nextor 2.1.4 Sunrise IDE kernel ROM (128KB, 8 × 16KB pages) is served from flash.
- IDE register overlay at `0x7C00`–`0x7EFF` is intercepted when the IDE enable bit is set.

For USB modes (`-s2`/`-m2`/`-c2`/`-r2`):

- Core 1 runs the TinyUSB USB host stack with asynchronous MSC read/write and a 3-second timeout watchdog.
- Device info for ATA IDENTIFY comes from the USB SCSI INQUIRY response.

For microSD modes (`-s1`/`-m1`/`-c1`/`-r1`):

- Core 1 runs synchronous SPI block I/O via `disk_read()`/`disk_write()` from the no-OS-FatFS library.
- Device info for ATA IDENTIFY is extracted from the SD card's CID register (OEM ID, Product Name, Revision).
- SD card hardware: SPI0, CS=GPIO33, SCK=GPIO34, MOSI=GPIO35, MISO=GPIO36 at 31.25 MHz.

In mapper-backed Sunrise modes (`-m1`/`-m2`/`-c1`/`-c2`/`-r1`/`-r2`), additionally:

- PIO1 SM0/SM1 run `msx_io_read_responder` / `msx_io_write_captor` to intercept I/O ports FC–FF.
- For `loadrom.pio` `-m1`/`-m2`, 1MB of external PSRAM on QMI CS1 (GPIO47) is used as mapper RAM (64 × 16KB pages), accessed through the uncached CS1 window.
- An expanded sub-slot architecture provides sub-slot 0 (Nextor ROM) and sub-slot 1 (mapper RAM).
- A bootstrap ROM phase ensures clean cold-boot before the expanded-slot mapper is activated.

### Audio integration

When an audio mode is selected, core 0 keeps serving the MSX memory bus while core 1 runs the active audio producer and I2S output.

For `-scc` or `-sccplus` flags encoded in the ROM type byte, or the matching Explorer audio profiles:

- Core 0 continues bus/memory service and forwards SCC writes.
- Core 1 continuously generates PCM audio from emu2212.
- Audio is sent using `pico_audio_i2s`.
- Build sets `PICO_AUDIO_I2S_PIO=1` so audio runs on **PIO1**, keeping **PIO0** dedicated to MSX bus handling.

Explorer can also route these audio sources through the same DAC path:

- **Dual PSG**: captures secondary PSG writes on ports `0x10`/`0x11` with PIO1 I/O write capture and generates audio with emu2149.
- **Primary PSG mirror**: when the ROM detail **PSG** option is set to Yes, captures primary PSG writes on ports `0xA0`/`0xA1` and mixes a DAC-side copy into the active audio profile, or starts a PSG-only audio loop when no other profile is selected.
- **MSX-MUSIC**: captures YM2413 writes on ports `0x7C`/`0x7D` and generates audio with emu2413.
- **MP3 playback**: decodes MP3 files from microSD on core 1 and streams PCM to I2S while the Explorer MSX menu remains on core 0.

For full audio behavior and registers, see:

- `docs/msx-picoverse-2350-scc.md`
- `docs/msx-picoverse-2350-dualpsg.md`
- `docs/msx-picoverse-2350-fmpac.md`
- `docs/msx-picoverse-2350-explorer-tool-manual.en-us.md`

---

## 6) Build Integration

Both `2350/software/loadrom.pio/pico/loadrom/CMakeLists.txt` and `2350/software/multirom.pio/pico/multirom/CMakeLists.txt` include:

- `pico_generate_pio_header(... msx_bus.pio)`
- `hardware_pio`
- `hardware_dma`
- `pico_multicore`
- `pico_audio_i2s` (via `pico_extras_import.cmake`)
- `tinyusb_board` and `tinyusb_host` (for Sunrise IDE USB host support)
- `no-OS-FatFS-SD-SDIO-SPI-RPi-Pico` (for Sunrise IDE microSD support)
- `sunrise_ide.c` (Sunrise IDE ATA emulation + USB MSC bridge)
- `sunrise_sd.c` (Sunrise IDE microSD SPI backend)
- `hw_config.c` (microSD SPI pin configuration)

---

Cristiano Goncalves  
03/29/26
