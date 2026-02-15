# MSX PicoVerse 2350 PIO Bus Strategy

This document describes the RP2350 PIO strategy currently used in:

- `2350/software/loadrom.pio`

Unlike the 2040 family, there is no `multirom.pio` target in `2350/software`; the PIO bus engine is currently applied to the LoadROM PIO path.

---

## 1) Core Strategy

| Owner | Responsibility |
|---|---|
| **PIO bus engine (`pio0`)** | Detect `/SLTSL` + `/RD`/`/WR`, assert `/WAIT`, capture writes, drive/release D0..D7 |
| **CPU (core 0)** | Mapper translation, ROM lookup, SCC/SCC+ register forwarding |
| **CPU (core 1, SCC mode)** | Audio generation for SCC/SCC+ via emu2212 and I2S output |

The bus strategy is still PIO + FIFO tokenization, but RP2350 adds an audio subsystem path (SCC/SCC+) and platform-specific GPIO/clock settings.

---

## 2) PIO Bus Architecture

`loadrom.pio` uses two state machines on `pio0`:

| SM | Program | Role |
|---|---|---|
| `SM0` | `msx_read_responder` | Read-cycle responder with `/WAIT` side-set and tokenized data output |
| `SM1` | `msx_write_captor` | Write-cycle capture for mapper and SCC register writes |

The implementation uses `jmp pin` polling to re-check slot validity while waiting for read/write strobes, preventing stale-slot race behavior.

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
- System clock is configured to **285 MHz**.

### ROM serving

- Uses the same 192 KB cache model with flash fallback for large ROMs.
- Mapper support includes Plain16/32, Linear48, Konami SCC, Konami, ASCII8/16, NEO8/16.

### SCC / SCC+ integration

When `-scc` or `-sccplus` flags are encoded in the ROM type byte:

- Core 0 continues bus/memory service and forwards SCC writes.
- Core 1 continuously generates PCM audio from emu2212.
- Audio is sent using `pico_audio_i2s`.
- Build sets `PICO_AUDIO_I2S_PIO=1` so audio runs on **PIO1**, keeping **PIO0** dedicated to MSX bus handling.

For full SCC/SCC+ behavior and registers, see:

- `docs/msx-picoverse-2350-scc.md`

---

## 6) Build Integration

`2350/software/loadrom.pio/pico/loadrom/CMakeLists.txt` includes:

- `pico_generate_pio_header(... msx_bus.pio)`
- `hardware_pio`
- `hardware_dma`
- `pico_multicore`
- `pico_audio_i2s` (via `pico_extras_import.cmake`)

---

Cristiano Goncalves  
02/15/26
