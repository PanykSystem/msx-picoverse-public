# MSX PicoVerse RP2040 LoadROM PIO Bus Engine

## Overview
This document explains the PIO-based bus engine that replaces the bit-banged MSX bus handling for the 2040 loadrom.pio target. The goal is to provide deterministic timing for /RD and /WR cycles, reduce CPU overhead, and make data output timing more consistent across MSX clock variations.

The approach used here is a split responsibility model:

- PIO owns the bus timing (detect /SLTSL, /RD, /WR), controls WAIT and BUSSDIR, and captures the address/data bus snapshot.
- The CPU services PIO FIFOs, translates the address into a ROM byte, and writes the data back to the PIO TX FIFO.

This version migrates the plain ROM handlers (32KB and linear 48KB) and all mapper handlers (Konami, Konami SCC, ASCII8, ASCII16, NEO8, NEO16) to PIO. All read and mapper write cycles are now serviced through the PIO FIFOs.

## Files and Targets

- PIO program: 2040/software/loadrom.pio/pico/loadrom/loadrom_bus.pio
- C integration: 2040/software/loadrom.pio/pico/loadrom/loadrom.c
- Build integration: 2040/software/loadrom.pio/pico/loadrom/CMakeLists.txt

The build now generates a PIO header for the program and links hardware_pio so the code can configure and run the state machines.

## Pin Map and Signals

The pin layout is unchanged and matches loadrom.h:

- Address bus: GPIO 0-15 (A0-A15)
- Data bus: GPIO 16-23 (D0-D7)
- Control: /RD 24, /WR 25, /IORQ 26, /SLTSL 27
- WAIT: GPIO 28
- BUSSDIR: GPIO 29

PIO is configured to read the full 32-bit GPIO snapshot, so address and data are captured at the same time. Data output is only driven when /RD is asserted and after the CPU provides the response byte via the TX FIFO.

## High-Level Flow

1. PIO waits for the slot to be selected (/SLTSL low).
2. On /RD low, PIO samples the GPIO pins and pushes the full 32-bit snapshot into the RX FIFO.
3. PIO asserts WAIT low to stall the MSX until data is ready.
4. CPU reads the RX FIFO, decodes the address, fetches a ROM byte, and writes it to the PIO TX FIFO.
5. PIO pulls the byte, drives it onto the data bus, deasserts WAIT, and waits for /RD to return high.
6. PIO releases the data bus; if /SLTSL remains low it immediately accepts the next /RD, otherwise it waits for the next slot select.

For /WR cycles, a second SM (msx_wr) captures a GPIO snapshot and pushes it to its RX FIFO. Mapper handlers consume those events to update bank registers.

## PIO Programs

The PIO source is in loadrom_bus.pio and defines two programs:

- msx_rd: Read cycle engine with side-set control for WAIT and BUSSDIR.
- msx_wr: Write cycle capture engine.

### msx_rd logic summary

- Wait for /SLTSL low and /RD low.
- Sample 32 GPIO pins into ISR and push to RX FIFO.
- Assert WAIT low (stall).
- Pull one byte from TX FIFO.
- Drive the data bus with that byte, set BUSSDIR to output, then deassert WAIT.
- Wait for /RD high, then release the data bus; if /SLTSL stays low, continue servicing reads without a full re-arm.

Side-set is used to control WAIT and BUSSDIR in a deterministic way. The code expects WAIT to be active-low and BUSSDIR active-high for output enable. The SLTSL jmp pin allows the PIO to keep servicing read bursts without waiting for a full deassert of /SLTSL.

### msx_wr logic summary

- Wait for /SLTSL low and /WR low.
- Sample 32 GPIO pins into ISR and push to RX FIFO.
- Wait for /WR high and /SLTSL high.

This keeps the CPU aware of write cycles while the PIO handles timing and sampling.

## FIFO Data Format

Both state machines push a full 32-bit GPIO snapshot. The layout is:

- Bits 0-15: Address bus A0-A15
- Bits 16-23: Data bus D0-D7
- Bits 24-27: /RD, /WR, /IORQ, /SLTSL
- Bits 28-29: WAIT, BUSSDIR
- Bits 30-31: Unused

In the current implementation the CPU reads the lower 16 bits for the address and, for mapper writes, the 8-bit data field. Mapper handlers use the data byte and the write address to update bank registers.

## C Integration Details

The integration is in loadrom.c and uses two state machines on PIO0:

- SM0 runs msx_rd
- SM1 runs msx_wr

Key integration points:

- bus_pio_configure_pins sets all bus pins to GPIO_FUNC_PIO0.
- bus_pio_init adds the PIO programs, configures pin mappings, shifts, and enables the SMs.
- loadrom_plain32, loadrom_linear48, and all mapper handlers now call bus_pio_init once and then loop on PIO FIFOs instead of polling GPIO.

The read/write loop is:

- Drain msx_wr RX FIFO and apply any mapper write updates
- Pull msx_rd RX FIFO samples, decode address, fetch ROM byte
- Push the byte to msx_rd TX FIFO

This preserves the ROM address logic and cache behavior while delegating bus timing to PIO.

## WAIT and BUSSDIR Behavior

WAIT and BUSSDIR are controlled in the PIO side-set of msx_rd:

- WAIT is driven low immediately after capturing the read cycle, stalling the bus.
- WAIT is driven high right after the ROM byte is placed on the data bus.
- BUSSDIR is driven high only while the data bus is driven by the Pico.

If your external bus transceiver uses inverted BUSSDIR polarity, the side-set constants in the PIO program should be flipped accordingly.

## Cache and Flash Reads

The CPU still owns ROM data retrieval and caching:

- When cache is enabled, rom_sram is filled and reads are served from SRAM.
- If cache is disabled or out of range, reads are served from flash.

This is unchanged from the prior bit-banging logic. The only difference is that WAIT is asserted by PIO instead of the CPU.

## Build Notes

The loadrom.pio target now uses:

- pico_generate_pio_header to emit loadrom_bus.pio.h
- hardware_pio to access PIO APIs

This is configured in CMakeLists.txt under loadrom.pio.

## Mapper Handling via PIO

The FIFO approach now supports mapper handlers:

1. Consume msx_wr FIFO events.
2. Extract address (bits 0-15) and data (bits 16-23).
3. Update the mapper bank registers according to the mapper rules.
4. Continue serving read cycles from msx_rd FIFO as done for plain/linear ROMs.

## Known Limitations and Considerations

- The PIO program assumes /SLTSL and /RD are properly synchronized to the MSX bus. If the bus is excessively noisy, extra filtering or sync logic may be needed.
- Read bursts are supported while /SLTSL remains asserted; this was required to make linear ROMs stable when the slot stays selected across sequential reads.
- PIO0 uses SM0 and SM1. If other PIO use is needed, adjust SM indices or move to PIO1.
- The current implementation uses a full 32-bit snapshot per cycle. If FIFO pressure becomes an issue, the PIO program can be modified to push only 16 bits and read control pins separately.

## Summary

This PIO approach replaces the timing-critical bit-banged loops with deterministic state machines and exposes a clean FIFO interface to the CPU. The result is more stable read timing and a clear path to future mapper migration while keeping the existing ROM address logic intact.


Cristiano Goncalves
02/07/26
