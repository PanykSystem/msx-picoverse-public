# Change Log

## PicoVerse 2350 Multirom v2.63
- Version bumped to v2.63 (top-level, MSX, and tool Makefiles).
- Fixed banked mapper ROMs hanging when the game switches segments in bursts from MSX RAM without reading the cartridge in between. The bus loops blocked waiting for a read and let the 8-entry write FIFO overflow, silently dropping bank switches and leaving the mapper on a stale segment. All banked loops (Konami/Konami-SCC/ASCII8, Konami SCC with SCC audio, ASCII16, ASCII16-X, Neo-8, Neo-16) now drain writes while waiting for the next read, using a single `FSTAT` sample per idle iteration so the read-response latency is unchanged. Reported with "Go Figure v1.2"; same fix as PicoVerse 2350 Loadrom v2.70.

## PicoVerse 2350 Multirom v2.62
- Reorganized `pico/multirom` sources into per-type subfolders: `audio/` (`emu2212`), `memory/` (`c2_emu`, the Carnivore2-style mapper/RAM emulation), and `storage/` (`hw_config.c`, `sunrise_ide`, `sunrise_sd`), updating `CMakeLists.txt` and `multirom.c` includes accordingly. Verified with a full reconfigure and rebuild.
- Removed the unused `nextor.c`/`nextor.h` files, which were not referenced by `CMakeLists.txt` or included by any other source (dead code left over from an earlier Nextor bridge implementation superseded by `sunrise_ide.c`/`sunrise_sd.c`).
- Fixed the PC tool mapper-detection read path to reject truncated ROM reads before hashing or scanning the allocated ROM buffer.
- Version bumped to v2.62 (top-level, MSX, and tool Makefiles).

## PicoVerse 2350 Multirom v2.61
- Bumped the multirom.pio version to v2.61.
- Improved MSX MultiROM menu drawing by rendering ROM rows with a single VRAM block write, preserving long-name scrolling from the first visible character.
- Fixed short mapper labels in the optimized row renderer so they pad with spaces instead of leaking stray characters at the end of the line.
- Reduced page-navigation redraws so page changes update only the ROM rows and footer instead of repainting the static title and separator lines.
- Fixed the MultiROM tool mapper detector so 8KB ROMs do not read past the ROM buffer when building images with embedded Sunrise/mapper options.
- Frogger - Konami (1983) [RC-704] rom on the MSX1 Konami compilation had a bug and was replaced by a working dump from File Hunter.

## PicoVerse 2350 Multirom v2.59
- Refreshed the generated ROM mapper SHA1 database from the current openMSX `softwaredb.xml` and updated the shared generator to parse the new attribute-based XML format.

## PicoVerse 2350 Multirom v2.58

- Bumped the multirom.pio version to v2.58.
- Changed the memory-read `/WAIT` driver to open-drain behaviour: the firmware now asserts `/WAIT` by pulling it low and releases the shared line as hi-Z after stretched read cycles and ROM-cache setup.

## PicoVerse 2350 Multirom v2.57

- Updated the MSX MultiROM menu separator lines to use the same custom MSX glyph used by Explorer, copying character pattern `0x17` into printable slot `0x7E` and rendering that glyph instead of ASCII hyphens.
- Removed the unused MSX menu configuration screen and disabled the `C`/`c` key shortcut that opened it.
