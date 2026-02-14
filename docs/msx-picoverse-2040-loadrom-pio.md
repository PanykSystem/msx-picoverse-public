# MSX PicoVerse RP2040 LoadROM PIO Bus Engine

## Overview

This document describes the PIO-based bus engine used by the `loadrom.pio` firmware target for the RP2040-based MSX PicoVerse cartridge. It replaces the earlier bit-banging approach with RP2040 PIO (Programmable I/O) state machines, achieving deterministic bus timing and freeing the CPU to focus on ROM lookup and mapper logic.

**Split-responsibility model:**

| Owner | Responsibility |
|-------|---------------|
| PIO   | Detect /SLTSL + /RD or /WR, assert /WAIT, capture address/data, drive data bus, release /WAIT |
| CPU   | Service PIO FIFOs, translate addresses through mapper bank registers, push ROM bytes back |

All ROM types — plain (16/32 KB), linear 48 KB, and all banked mappers (Konami, Konami SCC, ASCII8, ASCII16, NEO8, NEO16) — are serviced through the PIO FIFOs.

## Source Files

| File | Purpose |
|------|---------|
| `2040/software/loadrom.pio/pico/loadrom/msx_bus.pio` | PIO assembly — read responder and write captor state machines |
| `2040/software/loadrom.pio/pico/loadrom/loadrom.c` | C firmware — PIO init, ROM cache, mapper handlers, main loop |
| `2040/software/loadrom.pio/pico/loadrom/loadrom.h` | Pin definitions, ROM record layout, SRAM cache declaration |
| `2040/software/loadrom.pio/pico/loadrom/CMakeLists.txt` | Build config — generates PIO header, links `hardware_pio` |
| `2040/software/loadrom.pio/Makefile` | Top-level build — firmware + PC tool |

## Pin Map

The GPIO mapping is defined in `loadrom.h` and is shared between the PIO programs and the C code:

| GPIO | Signal | Direction | Notes |
|------|--------|-----------|-------|
| 0–15 | A0–A15 | Input | Address bus, always input |
| 16–23 | D0–D7 | Bidirectional | Data bus, driven by PIO `out pins` / `out pindirs` |
| 24 | /RD | Input | Active-low read strobe, used as `jmp pin` in SM0 |
| 25 | /WR | Input | Active-low write strobe, used as `jmp pin` in SM1 |
| 26 | /IORQ | Input | Active-low I/O request (monitored but not used by PIO) |
| 27 | /SLTSL | Input | Active-low slot select, used by PIO `wait` instruction |
| 28 | /WAIT | Output | Active-low, driven by PIO side-set |
| 29 | BUSSDIR | Input | Bus direction (directly active) |

## MSX Z80 Bus Timing

The Z80 runs at 3.58 MHz (T-cycle ≈ 279 ns). A memory read cycle looks like:

```
T1: MSX places address on bus; /SLTSL goes low
T2: /RD goes low — cartridge must start responding
T3: CPU samples the data bus (data must be valid)
T4: /RD and /SLTSL go high; cartridge releases bus
```

Budget from /RD falling to data valid ≈ 279 ns (one T-cycle). This firmware uses /WAIT to stretch the cycle, giving the Pico CPU time to look up data in SRAM and feed it back through the TX FIFO.

## PIO Architecture

Two PIO state machines run on `pio0`, both clocked at the full system clock (250 MHz, 4 ns per instruction):

| SM | Program | Role |
|----|---------|------|
| SM0 | `msx_read_responder` | Memory read cycles — captures address, drives data, controls /WAIT |
| SM1 | `msx_write_captor` | Memory write cycles — captures address + data for bank switching |

### msx_read_responder (SM0)

**Features:** side-set on /WAIT (1 pin), `jmp pin` on /RD, 13 instructions.

**Flow:**

```
 wait_sltsl:                                         
   wait 0 gpio 27    side 1    ; /SLTSL=0? (/WAIT=1) 
   jmp pin wait_sltsl side 1   ; /RD=1? re-check     
   nop               side 0    ; Assert /WAIT=0      
   in pins, 16       side 0    ; Capture A0..A15       
   push block        side 0    ; → RX FIFO (to CPU)    
   pull block        side 0    ; ← TX FIFO (from CPU)  
   out pins, 8       side 0    ; Drive D0..D7           
   out pindirs, 8    side 0    ; Enable outputs         
   nop               side 1 [1]; Release /WAIT (+hold)  
   wait 1 gpio 24    side 1    ; Wait /RD=1             
   set x, 0          side 1    ; Prepare zero           
   mov osr, x        side 1    ; Load into OSR          
   out pindirs, 8    side 1    ; Tri-state D0..D7       
   (wrap to wait_sltsl)                                 
```

**Key design decisions:**

1. **`jmp pin` polling instead of double `wait`:** The original design used two sequential `wait` instructions (`wait 0 gpio 27` then `wait 0 gpio 24`). This had a race condition: during a write cycle to our slot, SM0 would pass the /SLTSL wait, stay blocked at `/RD` wait, and then respond to an unrelated `/RD` assertion after `/SLTSL` went high — causing bus contention. The `jmp pin` approach re-checks `/SLTSL` on every iteration, ensuring both signals are simultaneously active before proceeding.

2. **Side-set for /WAIT:** /WAIT is asserted in the same instruction that follows the `jmp pin` fall-through, giving deterministic timing. The Z80 is frozen before any FIFO stall can cause data-bus glitches.

3. **Token format:** The CPU pushes a 16-bit token to the TX FIFO:
   - bits [7:0] = data byte for D0..D7
   - bits [15:8] = pindirs mask (0xFF = drive bus, 0x00 = tri-state)

4. **Tri-state cleanup:** After /RD goes high, the SM uses `set x, 0` → `mov osr, x` → `out pindirs, 8` to tri-state the data bus, ensuring clean release.

### msx_write_captor (SM1)

**Features:** no side-set, `jmp pin` on /WR, FIFO joined RX (8-deep), 6 instructions.

**Flow:**

```
 wait_sltsl:                                          
   wait 0 gpio 27            ; /SLTSL=0?              
   jmp pin wait_sltsl         ; /WR=1? re-check       
   nop              [2]      ; 3 cycles settle time   
   mov isr, pins             ; Snapshot all GPIOs      
   push block                ; → RX FIFO (to CPU)     
   wait 1 gpio 25            ; Wait /WR=1             
   (wrap to wait_sltsl)                                
```

**RX FIFO word format:**
- bits [15:0] = A0..A15 (write address)
- bits [23:16] = D0..D7 (write data)
- bits [24+] = control pins (ignored)

The same `jmp pin` race-condition fix applies here: the SM re-checks `/SLTSL` while waiting for `/WR`, preventing false captures from unrelated bus activity.

## C Integration

### PIO Initialization (`msx_pio_bus_init`)

1. Load both PIO programs into `pio0` instruction memory
2. Configure SM0 (read responder):
   - `in_base` = GPIO 0 (A0), shift left, 16 bits
   - `out_base` = GPIO 16 (D0), 8 pins, shift right (LSB first)
   - `sideset_pin` = GPIO 28 (/WAIT)
   - `jmp_pin` = GPIO 24 (/RD)
   - Clock divider = 1.0 (full 250 MHz)
3. Configure SM1 (write captor):
   - `in_base` = GPIO 0 (A0), shift left, 32 bits
   - `jmp_pin` = GPIO 25 (/WR)
   - FIFO joined RX for 8-deep buffering
   - Clock divider = 1.0
4. Hand data bus pins (GPIO 16–23) to PIO, initially tri-stated
5. /WAIT pin driven by PIO, initially deasserted (high)
6. Enable both state machines

### FIFO Data Flow

**Read path (per bus cycle):**

```
  SM0 pushes address (16-bit) → CPU reads RX FIFO
  CPU looks up ROM byte, builds token → pushes to TX FIFO
  SM0 pulls token → drives data bus, releases /WAIT
```

**Write path (bank switching):**

```
  SM1 pushes snapshot (24-bit) → CPU drains RX FIFO
  CPU extracts address + data → updates bank registers
```

### Token Helpers

| Function | Purpose |
|----------|---------|
| `read_rom_byte(rom_base, rel)` | Read ROM byte — serves from SRAM cache if `rel < rom_cached_size`, falls back to flash XIP otherwise |
| `pio_build_token(drive, data)` | Build 16-bit token: data byte + pindirs mask |
| `pio_try_get_write(addr, data)` | Non-blocking read from write captor FIFO |
| `pio_drain_writes(handler, ctx)` | Drain all pending writes, invoke handler per event |

### ROM Cache (`prepare_rom_source`)

When cache is enabled, the ROM data is copied from flash into `rom_sram[]` (192 KB SRAM buffer) for faster read access. During the copy, /WAIT is held low to prevent the MSX from reading stale data.

**Small ROMs (≤ 192 KB):** The entire ROM fits in cache. `rom_base` is redirected to SRAM and all reads come from fast SRAM. This is the path used by plain 16/32 KB, linear 48 KB, and smaller banked ROMs.

**Large ROMs (> 192 KB):** The first 192 KB is cached in SRAM, but `rom_base` stays pointing to flash and `available_length` retains the full ROM size. The `read_rom_byte()` helper transparently serves from SRAM for offsets within the cached region (the most commonly accessed initial banks) and falls back to flash XIP for higher offsets. This ensures all bank data is accessible — previously, ROMs larger than 192 KB were silently truncated.

Flash XIP reads at 250 MHz system clock with the default CLKDIV=4 give a 62.5 MHz flash clock (within spec for W25Q series in QSPI mode). Each XIP cache miss costs approximately 700 ns (~2–3 Z80 wait states), which is acceptable since /WAIT holds the Z80 during the lookup.

## Mapper Implementations

### Plain ROMs (no mapper)

**`loadrom_plain32`** — 16 KB or 32 KB ROM at 0x4000–0xBFFF. Pure address-to-data lookup, no write handling.

**`loadrom_linear48`** — 48 KB ROM at 0x0000–0xBFFF. Three pages, no bank switching.

### Banked Mappers — Generic Loop (`banked8_loop`)

All 8 KB-banked mappers (Konami SCC, Konami, ASCII8) use `banked8_loop`, which:

1. Drains pending writes from SM1 and invokes the mapper's write handler
2. Blocks on SM0 RX FIFO for the next read address
3. Drains any writes that arrived during the blocking wait
4. Translates the address through bank registers: `offset = bank_regs[(addr - 0x4000) >> 13] * 0x2000 + (addr & 0x1FFF)`
5. Pushes the ROM byte (or 0xFF if out of range) back to SM0

### Mapper Details

| Type ID | Mapper | Bank Size | Address Window | Switch Registers |
|---------|--------|-----------|----------------|-----------------|
| 1, 2 | Plain 16/32 KB | — | 0x4000–0xBFFF | None |
| 3 | Konami SCC | 8 KB | 0x4000–0xBFFF | 0x5000, 0x7000, 0x9000, 0xB000 |
| 4 | Linear 48 KB | — | 0x0000–0xBFFF | None |
| 5 | ASCII8 | 8 KB | 0x4000–0xBFFF | 0x6000, 0x6800, 0x7000, 0x7800 |
| 6 | ASCII16 | 16 KB | 0x4000–0xBFFF | 0x6000, 0x7000 |
| 7 | Konami | 8 KB | 0x4000–0xBFFF | Bank 0 fixed; 0x6000, 0x8000, 0xA000 |
| 8 | NEO8 | 8 KB | 0x0000–0xBFFF | 6 banks, 16-bit registers, 12-bit segment addressing |
| 9 | NEO16 | 16 KB | 0x0000–0xBFFF | 3 banks, 16-bit registers, 12-bit segment addressing |

### ASCII16

Uses its own loop instead of `banked8_loop` because address translation uses 16 KB banks:  
`offset = bank_regs[addr >> 15 & 1] << 14 + (addr & 0x3FFF)`

### NEO8 / NEO16

Use 16-bit bank registers with 12-bit segment addressing, covering the full 0x0000–0xBFFF range. Each register is written byte-by-byte (low/high) through mirrored address ranges. Cache is disabled for NEO mappers (`cache_enable = false`) since ROM images commonly exceed the 192 KB SRAM cache; all reads go through flash XIP. The `read_rom_byte()` helper is still used for consistency (with `rom_cached_size = 0`, it always reads from flash).

## Build Configuration

**CMakeLists.txt** uses:
- `pico_generate_pio_header(loadrom ${CMAKE_CURRENT_LIST_DIR}/msx_bus.pio)` to compile the PIO assembly into `msx_bus.pio.h`
- `hardware_pio` linked alongside `pico_stdlib`
- UART and USB stdio disabled (no serial output in production)
- Post-build copies `loadrom.bin` to `dist/` for the PC tool

**Makefile** builds:
- `firmware` target — runs CMake/Ninja for the Pico firmware
- `tool` target — builds the PC-side tool that packages ROM images with the firmware binary

## System Clock

The RP2040 is clocked at 250 MHz (`set_sys_clock_khz(250000, true)`), giving:
- 4 ns per PIO instruction
- The `jmp pin` polling loop runs at 2 instructions per iteration (8 ns), transparent to the bus
- Plenty of headroom for CPU-side ROM lookup and FIFO servicing

## ROM Binary Format

The ROM image is concatenated after the program binary in flash. The record header is:

| Offset | Size | Field |
|--------|------|-------|
| 0 | 50 bytes | ROM name (null-padded) |
| 50 | 1 byte | Mapper type (1–9) |
| 51 | 4 bytes | ROM size (little-endian uint32) |
| 55 | 4 bytes | ROM offset (little-endian uint32) |

ROM data follows immediately after the header.

## Known Limitations

- PIO0 uses SM0 and SM1. If other PIO usage is needed, adjust SM indices or use PIO1.
- The SRAM cache is 192 KB. ROMs larger than this use a hybrid approach: the first 192 KB is served from fast SRAM while higher banks fall back to flash XIP (~700 ns per cache miss, 2–3 extra Z80 wait states). NEO mappers skip caching entirely and serve all data from flash.
- The `jmp pin` polling loop introduces a worst-case 8 ns (2 PIO cycles) latency between /SLTSL and /RD going low simultaneously and the SM detecting it. This is negligible at 3.58 MHz.

---

Cristiano Goncalves  
02/14/26
