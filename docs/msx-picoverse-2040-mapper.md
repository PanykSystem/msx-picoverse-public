# PicoVerse 2040 Mapper Implementation (Sunrise + Nextor)

## Scope
This document describes the current `loadrom.pio` implementation that supports Sunrise IDE emulation and Nextor with 192KB mapper RAM in PicoVerse 2040. We  have iterated on this design to achieve stable Nextor boot and runtime behavior in both MSX1 and MSX2 environments while maintaining Sunrise IDE compatibility. Key features include:

- Sunrise IDE Nextor ROM on page 1 (`0x4000-0x7FFF`)
- Expanded slot behavior with subslot register at `0xFFFF`
- Memory mapper RAM in subslot 1
- Mapper page registers at I/O ports `0xFC-0xFF`
- Compatibility fixes validated on MSX1 and MSX2

The implementation described here corresponds to:

- `2040/software/loadrom.pio/pico/loadrom/loadrom.c`
- `2040/software/loadrom.pio/pico/loadrom/loadrom.h`
- `2040/software/loadrom.pio/pico/loadrom/msx_bus.pio`

## Design Goals
- Keep Sunrise IDE behavior compatible with existing Nextor kernels.
- Provide mapper behavior compatible with Nextor `MAP_INIT` and DOS initialization flow.
- Avoid bus contention and avoid global I/O-side effects that can freeze boot.
- Keep enough mapped RAM for stable boot/runtime while preserving ROM visibility where expected.

## SRAM Partitioning and Capacity
Current test profile keeps **192KB mapper RAM** and disables ROM cache in Sunrise+Mapper mode.

Constants in `loadrom.h`:

- `CACHE_SIZE = 196608` (192KB)
- `MAPPER_SIZE = 196608` (192KB)
- `MAPPER_PAGES = 12`
- `MAPPER_PAGE_SIZE = 16384` (16KB)

Union layout:

- Normal modes use full `rom_sram[192KB]` as ROM cache.
- Sunrise+Mapper mode uses full `mapper_ram[192KB]` and does **not** cache ROM to SRAM.

Runtime behavior in `loadrom_sunrise_mapper`:

- `rom_cache_capacity = 0`
- `prepare_rom_source(..., false, ...)`

This forces ROM reads to come directly from flash while all SRAM is used as mapper RAM.

## Bus and PIO Topology
### Memory bus (PIO0)
- `SM0`: memory read responder (`msx_read_responder`)
- `SM1`: memory write captor (`msx_write_captor`)

### I/O bus (PIO1)
- `SM0`: I/O read responder (`msx_io_read_responder`)
- `SM1`: I/O write captor (`msx_io_write_captor`)

Important implementation detail:

- I/O read responder is used for mapper readback (`IN FC-FF`) without `/WAIT` stretching.
- Earlier experiments with broad `/WAIT` control in I/O paths caused severe regressions (freeze/no boot), so current code avoids that in I/O responder.

## Expanded Slot and Subslot Model
### Subslot register (`0xFFFF`)
- Write `0xFFFF`: updates local `subslot_reg` image.
- Read `0xFFFF`: returns `~subslot_reg` (bitwise inverted), matching MSX expanded-slot convention.

### Current boot default (`subslot_reg = 0x10`)
Per-page initial mapping:

- Page 0 (`0x0000-0x3FFF`) -> subslot 0
- Page 1 (`0x4000-0x7FFF`) -> subslot 0
- Page 2 (`0x8000-0xBFFF`) -> subslot 1
- Page 3 (`0xC000-0xFFFF`) -> subslot 0

Rationale:

- Nextor mapper probing (`MAP_INIT`) validates mapper RAM through page 2 (`0x8000`) while switching segments with mapper ports.
- Exposing mapper RAM on page 2 at boot makes this probe path deterministic.

## Sunrise ROM + IDE Exposure
In subslot 0:

- Only `0x4000-0x7FFF` is served by Sunrise ROM/IDE logic.
- Reads in this window:
- First checked by `sunrise_ide_handle_read` for IDE register overlays.
- Otherwise treated as ROM fetch using Sunrise segment (`ide.segment`).

Writes in subslot 0:

- Only `0x4000-0x7FFF` are consumed by `sunrise_ide_handle_write`.

Outside this window in subslot 0:

- Not mapped by Sunrise/mapper logic in this mode (`in_window=false` for reads, writes ignored).

## Mapper Register and RAM Behavior
### Mapper register initialization
On reset:

- `FC=3`, `FD=2`, `FE=1`, `FF=0`

This follows BIOS-era mapper convention used by many implementations.

### Page index mapping for 192KB
- Mapper RAM has 12 pages (`0..11`).
- Register values are normalized by:
- `mapper_page = reg % 12`

This is implemented by `mapper_page_from_reg()`.

### Port writes (`OUT FC-FF`)
- Captured by I/O write SM.
- Stored as `reg & 0x0F`.
- Effective RAM page is computed later by modulo 12.

### Port reads (`IN FC-FF`)
- Captured by I/O read SM.
- Return format is `0xF0 | (reg & 0x0F)`.

### Critical visibility gating
A major compatibility trick is slot-context gating:

- `FC-FF` ports are only accepted/responded when page 2 is currently mapped to mapper subslot 1.
- Code checks `page2_subslot = (subslot_reg >> 4) & 0x03` and only allows port operation when `page2_subslot == 1`.

Rationale:

- Nextor `MAP_INIT` probes mapper in specific slot/page contexts.
- Ungated global port visibility produced wrong detection/state and boot failures.

## Memory Read/Write Resolution
For each memory access, page and active subslot are computed from address and `subslot_reg`.

Subslot 0:

- `0x4000-0x7FFF`: Sunrise ROM/IDE window.
- Other ranges: not handled by this cartridge logic.

Subslot 1:

- Full mapper RAM (all pages).
- Active mapper page selected by `FC/FD/FE/FF` according to 16KB CPU page.

Subslots 2 and 3:

- Treated as unused by this implementation.

## Why This Works with Nextor Boot
Nextor DOS initialization (`DOSINIT`) calls `MAP_INIT` very early. If mapper probing fails or is inconsistent, kernel bootstrap can abort or hang before prompt.

Key assumptions from Nextor code:

- Mapper segment switching is done with `OUT (0FEh),A`/`OUT (0FCh-0FFh),A`.
- Probe address is in page 2 (`0x8000`).
- Mapper behavior must be consistent with currently selected slot/subslot context.

Current implementation aligns with that by:

- Ensuring page 2 starts on mapper subslot (`subslot_reg=0x10`).
- Gating mapper port visibility on page-2 mapper presence.
- Keeping expanded slot register semantics (`0xFFFF`, read invert).

## Key Troubleshooting Tricks We Had to Implement
These changes were critical during stabilization:

- Avoided broad I/O `/WAIT` behavior in mapper I/O responder. Broad software-mediated I/O wait caused freezes and boot lockups.
- Avoided always-on global `FC-FF` behavior. Slot-context gating was necessary for robust `MAP_INIT` behavior.
- Restored hardware-like expanded-slot behavior at `0xFFFF` (write register, read inverted image).
- Moved to 192KB mapper RAM test profile and disabled mapper-mode ROM cache to simplify memory ownership and remove cache-side interference.
- Kept Sunrise ROM limited to page 1 window, matching Sunrise/Carnivore style exposure.

## References
### Nextor sources
- `mapinit.mac` (mapper probe/init logic):
- https://github.com/Konamiman/Nextor/blob/v2.1/source/kernel/bank1/mapinit.mac
- `dosinit.mac` (boot sequence that calls mapper init):
- https://github.com/Konamiman/Nextor/blob/v2.1/source/kernel/bank1/dosinit.mac

### Nextor docs
- Nextor 2.1 Driver Development Guide:
- https://github.com/Konamiman/Nextor/blob/v2.1/docs/Nextor%202.1%20Driver%20Development%20Guide.md
- Nextor 2.1 Programmers Reference:
- https://github.com/Konamiman/Nextor/blob/v2.1/docs/Nextor%202.1%20Programmers%20Reference.md

### Hardware reference behavior used for comparison
- Carnivore2 firmware VHD (slot/mapper/IDE behavior):
- `Carnivore2/Firmware/Sources/mcscc.vhd`

## Notes for Future Iteration
- 192KB mapper is currently a test profile kept intentionally.
- If future regression appears, first check:
- subslot reset value
- page-2 mapper visibility during early boot
- mapper port visibility gating
- I/O read responder timing side effects
- If needed, add trace instrumentation for first accesses to:
- `0xFFFF`
- `0x4104`
- ports `0xFC-0xFF`

This gives immediate insight into where boot diverges from expected Nextor `MAP_INIT` flow.

Cristiano Goncalves
02/21/2026
