# Change Log

## PicoVerse 2350 Multirom v2.58

- Bumped the multirom.pio version to v2.58.
- Changed the memory-read `/WAIT` driver to open-drain behaviour: the firmware now asserts `/WAIT` by pulling it low and releases the shared line as hi-Z after stretched read cycles and ROM-cache setup.

## PicoVerse 2350 Multirom v2.57

- Updated the MSX MultiROM menu separator lines to use the same custom MSX glyph used by Explorer, copying character pattern `0x17` into printable slot `0x7E` and rendering that glyph instead of ASCII hyphens.
- Removed the unused MSX menu configuration screen and disabled the `C`/`c` key shortcut that opened it.
