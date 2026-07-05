# Change Log

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
