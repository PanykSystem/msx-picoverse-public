# MSX PicoVerse RP2040 MultiROM PIO Bus Engine

## Overview

This document describes the PIO-based bus engine used by the `multirom.pio` firmware target for the RP2040-based MSX PicoVerse cartridge.

Like `loadrom.pio`, this implementation uses RP2040 PIO state machines to provide deterministic MSX bus timing:

- PIO detects bus cycles, controls `/WAIT`, captures writes, and drives data bus timing.
- CPU services FIFO messages, performs mapper translation, and returns ROM bytes.

For MultiROM, the same engine is reused in two phases:

1. **Menu phase**: serve the 32 KB menu ROM and capture menu selection writes.
2. **Selected ROM phase**: serve the chosen ROM through the mapper-specific loop.

This replaces the previous bit-banging loops for all ROM mappers in `multirom.pio` (except mapper 10 Nextor path, described later).

## Source Files

| File | Purpose |
|------|---------|
| `2040/software/multirom.pio/pico/multirom/msx_bus.pio` | PIO assembly programs (`msx_read_responder`, `msx_write_captor`) |
| `2040/software/multirom.pio/pico/multirom/multirom.c` | MultiROM firmware (PIO init, menu handling, mapper loops) |
| `2040/software/multirom.pio/pico/multirom/multirom.h` | Pin map and loader prototypes |
| `2040/software/multirom.pio/pico/multirom/CMakeLists.txt` | Build config, PIO header generation and link libraries |
| `2040/software/multirom.pio/Makefile` | Top-level build (firmware + tool) |

## Shared PIO Architecture (same as LoadROM)

MultiROM uses the same two-state-machine architecture as `loadrom.pio`:

| SM | Program | Function |
|----|---------|----------|
| SM0 | `msx_read_responder` | Handles memory reads, asserts/releases `/WAIT`, drives D0..D7 |
| SM1 | `msx_write_captor` | Captures memory writes for mapper bank register updates |

Both state machines run on `pio0` at full system clock (`clkdiv = 1.0`, system at 250 MHz).

### Read responder (`msx_read_responder`)

- Waits for `/SLTSL` low.
- Polls `/RD` through `jmp pin` so read capture only occurs while the slot is still selected.
- Asserts `/WAIT` immediately (`side-set`) before any CPU FIFO dependency.
- Pushes address to RX FIFO.
- Pulls response token from TX FIFO.
- Drives data/pindirs.
- Releases `/WAIT` and tri-states bus at end of cycle.

### Write captor (`msx_write_captor`)

- Waits for `/SLTSL` low.
- Polls `/WR` through `jmp pin` (same race-avoidance pattern).
- Captures pins (`A0..A15`, `D0..D7`) and pushes snapshot to RX FIFO.
- Waits for `/WR` release and loops.

### FIFO format and token format

**SM1 captured write word:**
- bits `[15:0]`: address `A0..A15`
- bits `[23:16]`: data `D0..D7`

**SM0 response token (CPU -> PIO):**
- bits `[7:0]`: data byte
- bits `[15:8]`: pindirs mask (`0xFF` drive bus, `0x00` tri-state)

## Pin Mapping

The pin map is equivalent to `loadrom.pio` and defined in `multirom.h`:

| GPIO | Signal | Direction |
|------|--------|-----------|
| 0-15 | A0-A15 | Input |
| 16-23 | D0-D7 | Bidirectional (PIO controlled) |
| 24 | `/RD` | Input |
| 25 | `/WR` | Input |
| 26 | `/IORQ` | Input |
| 27 | `/SLTSL` | Input |
| 28 | `/WAIT` | Output (PIO side-set) |
| 29 | `BUSSDIR` | Input |

## Core C Integration

### Bus init lifecycle (`msx_pio_bus_init`)

A critical MultiROM requirement is that menu and selected ROM phases occur in the same execution, so PIO setup must support re-entry.

Current MultiROM behavior:

1. Programs are added to instruction memory **once** (`msx_bus_programs_loaded`).
2. On each call, SMs are disabled, FIFOs cleared, and SMs restarted.
3. SM configs are re-applied.
4. Data pins are assigned to PIO and tri-stated initially.
5. `/WAIT` starts high and both SMs are enabled.

This prevents:
- repeated `pio_add_program()` allocations,
- stale FIFO data between menu and selected-ROM phase,
- SM state carry-over from previous loop.

### ROM source preparation (`prepare_rom_source`)

For cache-enabled modes, the ROM region is copied from flash XIP to SRAM (`rom_sram`, 192 KB) with DMA (`DMA_SIZE_8`) while `/WAIT` is held low.

- If ROM length fits SRAM cache, `rom_base` is redirected to SRAM.
- If larger, initial region is cached and `read_rom_byte()` serves SRAM-first then flash fallback.
- For NEO mappers in MultiROM, cache is intentionally disabled (`cache_enable = false`).

Helper functions mirror `loadrom.pio`:
- `read_rom_byte()`
- `pio_build_token()`
- `pio_try_get_write()`
- `pio_drain_writes()`

## MultiROM-Specific Additions (vs LoadROM)

## 1) Two-phase runtime model

Unlike `loadrom.pio`, MultiROM does not directly boot one ROM. It:

1. Boots **menu ROM** from offset `0x0000`.
2. Parses ROM records from config area after menu image.
3. Waits for menu selection write.
4. Returns selected index.
5. Starts selected mapper loop.

## 2) ROM records table parsing

MultiROM parses up to 128 ROM records from the image metadata block:

- Name (50 bytes)
- Mapper (1 byte)
- Size (4 bytes)
- Offset (4 bytes)

End-of-table sentinel: a full record filled with `0xFF`.

## 3) Menu selection write capture

Menu ROM writes selected ROM index to monitor address:

- `MONITOR_ADDR = 0x8000 + (ROM_RECORD_SIZE * MAX_ROM_RECORDS) + 1`

In PIO mode, this write is captured by SM1 and consumed by `handle_menu_write()` via `pio_drain_writes()`.

When a read of address `0x0000` occurs after selection, firmware returns from menu loop and starts selected ROM loader.

## 4) Mapper dispatch includes Nextor path

After menu selection, MultiROM dispatches by mapper code:

- `1,2`: Plain 16/32 KB
- `3`: Konami SCC
- `4`: Linear48
- `5`: ASCII8
- `6`: ASCII16
- `7`: Konami
- `8`: NEO8
- `9`: NEO16
- `10`: Nextor

Mappers `1..9` use PIO-based loops.
Mapper `10` (`loadrom_nextor`) currently remains a dedicated non-PIO path with multicore USB host bridge logic (`nextor.c`).

## 5) Menu + selected-ROM PIO continuity

Because MultiROM reuses the same PIO resources for both phases, the implementation explicitly reinitializes SM state before each loader loop. This is additional lifecycle management not needed in the simpler single-ROM `loadrom.pio` flow.

## Mapper Loops in MultiROM (PIO mode)

### Plain 32 (`loadrom_plain32`)

- Window: `0x4000..0xBFFF`
- No write handling
- PIO read FIFO -> direct lookup -> response token

### Linear48 (`loadrom_linear48`)

- Window: `0x0000..0xBFFF`
- No write handling

### Generic 8 KB banked loop (`banked8_loop`)

Used by:
- `loadrom_konamiscc`
- `loadrom_konami`
- `loadrom_ascii8`

Per iteration:
1. Drain writes and update bank regs.
2. Wait for read address from SM0.
3. Drain writes again (writes arriving while blocked).
4. Translate address by bank table.
5. Return token.

### ASCII16

Uses dedicated loop due to 16 KB bank stride:
- two bank registers
- bank index from `addr >> 15`

### NEO8 / NEO16

- 16-bit bank registers with 12-bit segment field
- mirrored write ranges handled in write handlers
- windows include `0x0000..0xBFFF`
- cache disabled by design in current firmware entry points

## Build and Configuration

`CMakeLists.txt` for MultiROM PIO includes:

- `pico_generate_pio_header(multirom .../msx_bus.pio)`
- `hardware_pio` link
- `hardware_dma` link

This is required so `#include "msx_bus.pio.h"` resolves and DMA cache copy compiles.

## Timing and Determinism

At 250 MHz system clock:

- PIO instructions run at 4 ns each.
- `/WAIT` is asserted in deterministic instruction timing before CPU lookup.
- CPU lookup latency is absorbed by stretched cycle while Z80 is halted.

The `jmp pin` polling strategy prevents stale `/SLTSL` state races and avoids unintended drive cycles after deselection.

## Known Limitations and Notes

1. **Nextor path remains non-PIO** in this firmware variant (mapper 10).
2. **CMake toolchain availability** is required for full build validation (`cmake`, Pico SDK environment).
3. **PIO resource usage** assumes `pio0` SM0/SM1 are reserved for MultiROM bus engine.
4. **Cache behavior**: first 192 KB may be SRAM-cached (when enabled), with flash fallback beyond cached range.

## Differences Summary: LoadROM PIO vs MultiROM PIO

| Area | LoadROM PIO | MultiROM PIO |
|------|-------------|--------------|
| Runtime model | Single selected ROM from image header | Menu ROM first, then selected ROM from record table |
| Metadata source | One ROM header record | Multi-record table (`MAX_ROM_RECORDS`) |
| Menu write handling | Not applicable | Monitor write captured through SM1 (`handle_menu_write`) |
| PIO lifecycle complexity | One main init flow | Re-init between menu and target ROM loops; one-time program load guard |
| Mapper set | 1..9 | 1..10 (10 is Nextor bridge path) |
| Core-1 USB integration | Not part of loadrom | Present for mapper 10 (`nextor_io`) |

## Recommended Verification Checklist

1. Boot menu ROM and verify normal menu rendering.
2. Select one ROM of each mapper type 1..9 and verify execution.
3. Stress test repeated menu selections/power cycles to validate SM re-entry stability.
4. Confirm mapper writes are captured correctly (bank switching works).
5. For large ROMs, verify upper banks are reachable (SRAM+flash fallback path).
6. Verify mapper 10 (Nextor) behavior independently as separate non-PIO path.

---

Cristiano Goncalves  
02/14/26
