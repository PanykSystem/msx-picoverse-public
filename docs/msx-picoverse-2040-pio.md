# MSX PicoVerse 2040 PIO Bus Strategy

This document describes the RP2040 PIO strategy used by PicoVerse 2040 firmware targets:

- `2040/software/loadrom.pio`
- `2040/software/multirom.pio`

Both reuse the same bus-engine architecture and differ mainly in workflow (single-ROM vs menu-driven multi-ROM).

---

## 1) Core Strategy

| Owner | Responsibility |
|---|---|
| **PIO** | Detect slot read/write cycles, assert `/WAIT`, capture address/data, drive/release data bus |
| **CPU** | Drain/fill FIFOs, apply mapper bank translation, return ROM bytes |

The key goal is deterministic bus timing while allowing mapper logic to run in C without violating Z80 timing.

---

## 2) PIO Architecture (Shared by LoadROM and MultiROM)

Both firmware targets use two state machines on `pio0`:

| SM | Program | Role |
|---|---|---|
| `SM0` | `msx_read_responder` | Handles `/RD` cycles, controls `/WAIT`, captures address, outputs tokenized data |
| `SM1` | `msx_write_captor` | Handles `/WR` cycles, captures address+data for mapper updates |

### Why `jmp pin` polling is used

Both programs use the race-safe pattern:

- wait for `/SLTSL` low
- poll `/RD` or `/WR` with `jmp pin` while re-checking slot selection

This avoids stale-slot race conditions that can cause false reads/writes after deselection.

---

## 3) RP2040 Pin Map

| GPIO | Signal |
|---|---|
| 0–15 | A0–A15 |
| 16–23 | D0–D7 |
| 24 | `/RD` |
| 25 | `/WR` |
| 26 | `/IORQ` |
| 27 | `/SLTSL` |
| 28 | `/WAIT` |
| 29 | `BUSSDIR` |

---

## 4) FIFO Contract

### Write captor word (`SM1 -> CPU`)

- `bits[15:0]` = address A0..A15
- `bits[23:16]` = data D0..D7

### Read response token (`CPU -> SM0`)

- `bits[7:0]` = data byte
- `bits[15:8]` = pin-direction mask (`0xFF` drive, `0x00` tri-state)

---

## 5) ROM Serving Strategy

- Uses a 192 KB SRAM cache when enabled.
- Small ROMs are fully cached.
- Large ROMs use SRAM-first + flash XIP fallback.
- `/WAIT` stretching covers CPU lookup latency.

Supported mappers in PIO paths include:

- Plain16 / Plain32
- Linear48
- Konami SCC
- Konami
- ASCII8 / ASCII16
- NEO8 / NEO16

---

## 6) LoadROM vs MultiROM on 2040

| Area | `loadrom.pio` | `multirom.pio` |
|---|---|---|
| Runtime | Single ROM from one record | Menu ROM first, then selected ROM from record table |
| Menu monitor write | N/A | Captured through `SM1` to detect selected entry |
| Lifecycle | One main flow | Re-initializes SM/FIFO state between menu and selected-ROM phases |
| Mapper set | 1..9 | 1..10 (`10` uses dedicated Nextor bridge path) |

---

## 7) Build Integration

Both RP2040 PIO firmware targets require:

- `pico_generate_pio_header(... msx_bus.pio)`
- `hardware_pio`
- `hardware_dma`

---

Cristiano Goncalves  
02/15/26
