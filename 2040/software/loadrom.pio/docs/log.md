# Change Log

## PicoVerse 2040 Loadrom v2.61

- Fixed the PC tool mapper-detection read path to reject truncated ROM reads before hashing or scanning the allocated ROM buffer.
- Version bumped to v2.61 (top-level and tool Makefiles).

## PicoVerse 2040 Loadrom v2.60

- Refreshed the generated ROM mapper SHA1 database from the current openMSX `softwaredb.xml` and updated the generator to parse the new attribute-based XML format.
  
## PicoVerse 2040 Loadrom v2.59

- Bumped the loadrom.pio version to v2.59.
- Changed the PIO LoadROM memory-read `/WAIT` driver to open-drain behaviour: the cartridge now pulls `/WAIT` low only while stretching its own slot read cycles and releases the shared line as hi-Z afterward, avoiding contention with another `/WAIT`-using cartridge in the other MSX slot.
- Changed the MIDI and joystick I/O read `/WAIT` responders to the same open-drain behaviour so those auxiliary firmware targets no longer drive the shared `/WAIT` line high when idle or after an I/O read response.

## PicoVerse 2040 Loadrom v2.58

- Bumped the loadrom.pio version to v2.58.
- Set all loadrom.pio firmware target system clocks to 230 MHz to improve stability with pico boards with bad quality flash memory.
