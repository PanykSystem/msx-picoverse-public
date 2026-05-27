# Change Log

## v2.58

- Bumped the loadrom sub-project version to v2.58.
- Raised the MSX-MUSIC/FM-PAC audio output gain to match the existing SCC, SCC+, and Dual PSG I2S volume boost while preserving sample clipping protection.
- Fixed embedded Sunrise+Mapper SD and Carnivore2 SD/USB type decoding so system type values 16-18 are not mistaken for audio-flagged external ROMs.

## v2.57

- Bumped the loadrom sub-project version to v2.57.
- Added dual PSG emulation: a secondary AY-3-8910 instance (emu2149) clocked at 1.7897725 MHz that captures `OUT (0x10),A` / `OUT (0x11),A` on the MSX I/O bus via a dedicated PIO1 write captor and streams its mix to the I2S DAC alongside the existing SCC audio path.
- Added the loadrom tool `-d` / `--dual-psg` flag that sets bit 4 of the configuration `rom_type` byte; the firmware reads this flag at boot to enable the dual PSG engine.
- Centralised cartridge audio configuration in the firmware as a single mutually-exclusive `audio_mode_t` (`NONE` / `SCC` / `SCC_PLUS` / `DUAL_PSG`) so future on-cartridge sound chips can be plugged in without conflicting with existing engines.
- Enforced audio-mode exclusivity in the tool: `-d` is rejected against `-scc`/`-sccplus`, against Konami SCC and Manbow2 mappers (whose second audio slot is reserved for the on-cartridge SCC chip), and against the embedded Sunrise/Carnivore2 system modes that already use core 1 for storage backends.
- Simplified the `-d` rejection message for Konami SCC / Manbow2 ROMs to "Second PSG is not supported with Konami SCC ROMs."
- Moved dual PSG I/O servicing to core 1 (the audio producer core) so secondary PSG port writes are consumed continuously even while game code runs from MSX RAM and no cartridge read cycles occur.
- Delayed the PIO1 I/O write sample point inside `/WR` low so plain `0x10`/`0x11` port writes are latched after the address and data lines have fully settled.
- Added the shared PicoVerse 2350 Dual PSG implementation reference covering LoadROM and Explorer behavior.
- Added MSX-MUSIC/YM2413 emulation for non-SYSTEM ROMs through `-f` / `-fmpac`, capturing OPLL writes on I/O ports `0x7C`/`0x7D` via PIO1 and streaming emu2413 audio through the I2S DAC.
- Enforced MSX-MUSIC as a mutually-exclusive cartridge audio mode with SCC, SCC+, and Dual PSG; documented the emu2413 credit in the public README.
- Embedded the English FM-PAC BIOS (`FMPCCMFC.BIN`) into the LoadROM tool output for every `-f` / `-fmpac` non-SYSTEM ROM image and exposed it from the firmware through an expanded FM-PAC subslot.
- Added firmware handling for FM-PAC memory-mapped YM2413 registers (`0x7FF4`/`0x7FF5`), FM-PAC control/page registers, and PAC SRAM key gating while preserving the selected game mapper in the primary subslot.
- Updated the 2350 LoadROM manual and public copyright notes to describe FM-PAC BIOS inclusion and credit the bundled BIOS/translation sources.
- Added a detailed PicoVerse 2350 MSX-MUSIC / FM-PAC implementation guide covering the tool packaging, UF2 layout, firmware audio path, expanded-slot BIOS exposure, mapper routing, and limitations.
- Updated the public README with user-facing LoadROM instructions for MSX-MUSIC/FM-PAC and Dual PSG command-line builds.
- Removed Konami SCC and Manbow2 from the supported MSX-MUSIC/FM-PAC mapper set; the LoadROM tool now rejects `-f` / `-fmpac` for those ROMs and the firmware FM-PAC wrapper no longer carries SCC-class mapper branches.
