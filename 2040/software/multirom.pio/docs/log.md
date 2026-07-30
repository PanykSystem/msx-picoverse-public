# Change Log

## PicoVerse 2040 Multirom v2.63

- Version bumped to v2.63 (top-level and tool Makefiles).
- Fixed lost bank-switch writes in every banked mapper loop (`banked8_loop` for Konami/Konami-SCC/ASCII8, plus ASCII16, ASCII16-X, Neo-8 and Neo-16). Those loops blocked on `pio_sm_get_blocking()` for the next cartridge read and only drained the memory write captor FIFO before and after it, so game code executing from MSX RAM that issued a burst of segment switches without any intervening cartridge read overflowed the 8-entry RX FIFO. The write state machine then stalled on `push block` and the extra switches were silently dropped, leaving the mapper register on a stale segment (and, with an ASCII16-X/Neo cache miss, forcing the bank refill to run while `/WAIT` was already asserted). Added `pio_get_read_draining_writes()`, which drains the memory write FIFO while waiting for the next read, and switched all banked loops to it. Same fix as PicoVerse 2040 Loadrom v2.62 and PicoVerse 2350 Loadrom v2.70, reported with "Go Figure v1.2" (ASCII16-X), which alternates the page-1 segment twice per call from a page-3 RAM routine and hung on the palette cross-fade.
- Kept the new idle loop at the same read-response latency as the previous blocking wait by sampling `FSTAT` once per iteration and testing the read and write RX-empty flags from that single sample. Polling the two FIFOs with separate `pio_sm_is_rx_fifo_empty()` calls doubles the PIO register accesses in the loop, which on the Cortex-M0+ stretches `/WAIT` on every cartridge read enough to slow VDP transfer loops and trigger "VDP too slow" reports in timing-sensitive games.
- The planar 16/32/48/64KB loops keep the plain blocking wait: they have no mapper registers and therefore no write handler.

## PicoVerse 2040 Multirom v2.62

- Fixed the PC tool mapper-detection read path to reject truncated ROM reads before hashing or scanning the allocated ROM buffer.
- Version bumped to v2.62 (top-level and tool Makefiles).

## PicoVerse 2040 Multirom v2.61

- Bumped the multirom.pio version to v2.61.
- Improved MSX MultiROM menu list drawing by rendering each ROM row through a single VRAM block write instead of per-character BIOS output.
- Fixed the MultiROM tool mapper detector so 8KB ROMs do not read past the ROM buffer when building images with embedded Sunrise/mapper options.
- Restored selected-row name scrolling so long ROM names advance from the first visible character while using the VRAM row renderer.
- Reduced page-navigation redraws so LEFT/RIGHT and page-boundary moves update only the ROM rows and footer instead of repainting the static title and separator lines.
- Frogger - Konami (1983) [RC-704] rom on the MSX1 Konami compilation had a bug and was replaced by a working dump from File Hunter.

## PicoVerse 2040 Multirom v2.60

- Moved the shared MultiROM version declaration into the aggregate and tool Makefiles, removing the separate `version.mk` include.
- Refreshed the generated ROM mapper SHA1 database from the current openMSX `softwaredb.xml` and updated the generator to parse the new attribute-based XML format.
  
## PicoVerse 2040 Multirom v2.59

- Bumped the multirom.pio version to v2.59.
- Changed the memory-read `/WAIT` driver to open-drain behaviour: the firmware now asserts `/WAIT` by pulling it low and releases the shared line as hi-Z after stretched read cycles and ROM-cache setup.

## PicoVerse 2040 Multirom v2.58

- Bumped the multirom.pio version to v2.58.
- Set all multirom.pio firmware target system clocks to 230 MHz to improve stability with pico boards with bad quality flash memory.

## PicoVerse 2040 Multirom v2.57

- Updated the MSX MultiROM menu separator lines to use the same custom MSX glyph used by Explorer, copying character pattern `0x17` into printable slot `0x7E` and rendering that glyph instead of ASCII hyphens.
- Removed the unused MSX menu configuration screen and disabled the `C`/`c` key shortcut that opened it.
