# Change Log

## PicoVerse 2350 Explorer v2.36

- Bumped Explorer version to v2.36.
- Fixed Nextor + 1MB mapper instability and random errors that appeared only when PSG Mirror was enabled. 
- Extended the same lock-free hand-off to every Sunrise mapper audio profile so no audio chip write can stall core0 and drop mapper segment-register writes. 
- Disabled the MSX-MUSIC (YM2413/FM-PAC) audio profile for Sunrise Nextor ROMs that add the 1MB mapper (the `+ 1MB Mapper` USB/SD options), keeping it available only for the non-mapper Sunrise Nextor options and regular game ROMs. MSX-MUSIC with the mapper is pushing the limit of the core coordination and need more research to make it stable. The other Sunrise mapper audio profiles (Dual PSG, YM2151/SFG01, YM2151/SFG05, SCC, SCC+) remain available for the Sunrise + 1MB mapper options. This option will be re-enabled in a future Explorer release once the core coordination is improved to avoid dropped mapper writes and random errors.
- Disabled the PSG Mirror option for Sunrise Nextor ROMs that add the 1MB mapper.

## PicoVerse 2350 Explorer v2.35

- Bumped Explorer version to v2.35.
- Refreshed the generated ROM mapper SHA1 databases used by the Explorer tool and Pico firmware from the current openMSX `softwaredb.xml`.
- Fixed the Kikikaikai - Mystery - TAITO (ASCII8) hang under the YM2151 (SFG05/SFG01) audio profiles (the earlier v2.34 ASCII8 banking patch did not address the real cause). The combined SFG bus loops (`loadrom_external_sfg` and `loadrom_sunrise_sfg_common`) only drained the captured-write FIFO at the top of the loop, then serviced a read without re-draining. A bank-switch or `ENASLT` secondary-slot (`0xFFFF`) write landing between that drain and the read dequeue was applied only after the read, so the read was answered with a stale subslot/bank and returned a corrupt byte. The game's music driver remaps the SFG into page 2 via `ENASLT` every frame, so this race eventually corrupted a read and froze the MSX after some time. Both SFG loops now re-drain pending writes immediately after dequeuing the read address (matching the proven `banked8_loop` ordering), so the response always reflects the latest subslot/bank state.
- Improved speed when drawing of the ROM/MP3/WAV list by rendering each menu row with a single VRAM block write.
- Added MP3/WAV folder playback controls with Single, All, and Random modes, plus guarded Next/Previous music navigation.
- Improved the MP3/WAV detail screen so it keeps the current track information and controls aligned while playback changes.
- Fixed microSD folder navigation after returning from subfolders, keeping the page and cursor state consistent.
- Improved WAVEGAME audio startup/restart reliability and reduced MSX menu ROM size so the menu stays below the Pico page-buffer window.
- Fixed WAVEGAME ROMs launching with no audio after an MP3/WAV had been played first.
- Fixed File Hunter downloads being rejected with "audio busy" after MP3/WAV playback.
- Fixed the MSX freezing (and the audio dying permanently) when entering File Hunter after playing an MP3/WAV. 
- Reordered the MP3/WAV detail screen options so the playback Mode is the last option and Action: Play is selected by default.
- Added the Pico unique Chip ID to the MSX help screen and moved the return prompt to the row below it.
- Shortened the selected-row inverted highlight by two columns on the right to align it with the menu frame.
- Added Sunrise Nextor SD partition selection for FAT16 microSD partitions up to 4GB, persisted in each ROM's `.PVC` options file.
- Changed Sunrise Nextor SD partition selection to show Pico-supplied partition labels/messages without detail-screen footer text, and added F2 microSD partition cycling across FAT16, FAT32, and exFAT partitions with `/PICOVERSE.PVC` persistence.
- Fixed F2 microSD partition cycling so it can move through supported primary/logical partitions and keeps F1/F3 source switching responsive after a partition change.
- Fixed the empty F2 microSD partition screen so `P` can cycle back to another partition and `1`/`2` source shortcuts still work.
- Changed the ROM detail screen to show only the selected-option helper footer, restored the SD Part helper text, and show the actual compatible FAT16 partition label instead of `SINGLE PARTITION`.
- Show the selected F2 microSD partition label in the status area whenever no transient status message is active.
- Fixed FAT16/FAT32/exFAT partition label detection so F2 and Sunrise SD partition names use the filesystem volume label instead of boot-sector defaults or generic partition names.
- Fixed a regression where scanning a filesystem label could overwrite the MBR/EBR sector buffer and hide later microSD partitions from the F2 partition cycle.
- Added free-space MB reporting to the F2 microSD partition status label, including the first time F2 selects the microSD source.
- Changed the ROM detail and quick-run defaults so PSG Mirror starts enabled unless saved `.PVC` options override it.
- Fixed F3 File Hunter launching from an empty F2 microSD partition screen.
- Fixed blinking status messages so the hidden phase never substitutes the F2 partition buffer and cuts the start of File Hunter network text.
- Added a confirmed `D` delete command for selected files on the F2 microSD screen while leaving folders protected.
- Changed the 40-column F2 microSD status to alternate between the partition label and free-space amount instead of truncating the free-space text.
- Improved help screen with additional `D` and `P` commands.
- Updated Explorer, feature, public README, and Sunrise Nextor documentation for the new `P`/`D` commands, multi-partition microSD browsing, and FAT16 up-to-4GB Nextor partition requirements.
- Fixed Sunrise Nextor SD partition detection for exact 4GB FAT16 partitions and show `PARTITION1` fallback text when the filesystem has no user volume label.
- Added a per-ROM audio volume control to the ROM detail screen, persisted it in `.PVC` options files, and raised the YM2151 SFG01/SFG05 baseline output to better match other audio profiles.
- Fixed MP3/WAV Random mode so pressing `N` or `P` while playback is active chooses a random track instead of stepping through the folder list.
- Fixed MP3/WAV and WAVEGAME audio becoming silent after ROM detail SD work by quiescing the lazy MP3 core before option load/save and mapper detection, preserving the I2S handoff pool for the next audio launch.

## PicoVerse 2350 Explorer v2.34

- Bumped Explorer to v2.34.
- Fixed long microSD folder names by sending the selected record index to the Pico instead of truncating through the query buffer.
- Changed the memory-read and ROM-load `/WAIT` release paths to open-drain behaviour.
- Added microSD WAV discovery and playback through the existing MP3 detail screen controls.
- Added WAVEGAME runtime support for microSD game ROMs via MSX I/O port `0x92` (stop/pause/start/loop, fade-out/in, play-once/loop, `pause.wav`, `multi.wav`, deferred commands, per-song cfg offsets), with looped WAV playback in the shared MP3/WAV core.
- Fixed non-standard ASCII8 WaveGame ROMs (e.g. the 33-block Outrun ROM) that showed the intro but black-screened into the game, by resetting all ASCII8 banks to block 0 (matching openMSX) and wrapping out-of-range banks for non power-of-two images. 
- The non-standard ASCII8 patch also fixed the Kikikaikai - Mystery - TAITO hanging when using SFG-01/SFG-05 reported issue (#19) by ensuring the music data is correctly banked in the expected location.
- Added combined WAVEGAME + PSG Mirror support and WAVEGAME sidecar auto-detection that forces the ROM audio profile to None while keeping PSG Mirror selectable.
- Updated WAVEGAME and public README documentation.

## PicoVerse 2350 Explorer v2.33

- Bumped Explorer to v2.33.
- Improved MSX-MUSIC output with a soft-knee limiter, DC blocking, and light low-pass filtering so dense FM arrangements avoid hard clipping while normal-level material remains linear.
- Refined the ROM detail workflow with chip-style audio profile labels, aligned option text, ENTER-to-detail behavior, SPACE quick-run using saved/default `.PVC` options, and mapper detection before launch when needed.
- Added external SCC/SCC+ profiles for non-SYSTEM ROMs, keeping the game mapper in expanded subslot 0 and exposing the virtual SCC/SCC+ surface in subslot 1.
- Enabled external SCC/SCC+ profiles for Sunrise Nextor SYSTEM ROMs by exposing the virtual SCC/SCC+ cartridge in a free expanded subslot while keeping Nextor storage, optional WiFi, and optional mapper RAM in their existing subslots.
- Fixed Sunrise Nextor external SCC/SCC+ startup by guarding SCC audio servicing until the I2S audio pool is fully initialized, avoiding an early core1 storage-backend fault during boot.
- Added YM2151 (SFG05) and YM2151 (SFG01) profiles for non-SYSTEM ROMs, keeping the game mapper in subslot 0 and exposing an SFG-like YM2151 register surface plus the selected 32K Yamaha SFG BIOS image in subslot 1.
- Enabled YM2151 (SFG05/SFG01) profiles for Sunrise Nextor SYSTEM ROMs by exposing the virtual SFG cartridge in a free expanded subslot while keeping mapper RAM available for Nextor.
- Fixed the YM2151 (SFG05/SFG01) profiles so the plain "Nextor Sunrise IDE (SD/USB)" options no longer expose the 1MB mapper RAM (only the explicit "+ 1MB Mapper" options do), while still exposing the SFG cartridge subslot; gated the mapper RAM region, mapper memory subslot, and mapper I/O ports (0xFC-0xFF) behind a new `mapper_enable` flag in `loadrom_sunrise_sfg_common`.
- Renamed the MSX menu's SFG05 audio profile to YM2164 (SFG05) and route SFG05 audio through YM2164/OPP test-register and Timer B behavior while leaving SFG01 as YM2151/OPM.
- Exposed the FM-PAC BIOS for Sunrise Nextor SYSTEM ROMs when MSX-MUSIC is selected by launching a Sunrise + FM-PAC expanded-slot layout while keeping Nextor storage, optional WiFi, mapper RAM, and YM2413 audio servicing active.
- Disabled the optional PSRAM `0xC0` post-QPI initialization command after LY68L6400S compatibility issues were reported by Ludovic Avot.
- Matched the RBSC-style SFG top-128-byte control aperture outside the BIOS ROM window and mixed primary PSG mirror audio only when the mirrored PSG has audible channels.
- Consolidated the bundled Nextor and SFG ROM payloads under `resources/` and updated the Explorer tool to embed them directly from that shared project folder.

## PicoVerse 2350 Explorer v2.32

- Bumped Explorer to v2.32.
- Reworked MSX-MUSIC plus PSG mixing to follow the openMSX mixer model: keep FM and PSG as independent sources, apply per-device gain, accumulate them, and run DC removal on the final mixed output.
- Removed the previous PSG-only DC filter and fixed 3:1 weighted blend that could make active PSG writes corrupt MSX-MUSIC playback.
- Separated MSX-MUSIC and PSG output completely when both are enabled, routing FM and PSG to independent stereo channels instead of mixing their samples together.
- Reduced MSX-MUSIC plus PSG runtime contention by using the fast PSG renderer only when primary PSG mirroring is paired with MSX-MUSIC and by servicing I/O between FM and PSG sample generation.

## PicoVerse 2350 Explorer v2.31

- Bumped Explorer to v2.31.
- Mixed primary PSG mirroring into MSX-MUSIC with lower gain and DC filtering so MSX-MUSIC ROMs can enable PSG emulation without FM distortion.
- Gated the MSX-MUSIC PSG mix on audible PSG channel state so silent PSG emulation no longer changes the MSX-MUSIC output.
- Mixed active PSG with MSX-MUSIC using reserved headroom so combined PSG/FM playback avoids clipping artifacts.
- Allowed the MSX-MUSIC audio profile to be selected and launched with every non-SYSTEM mapper.
- Enabled MSX-MUSIC selection and SYSTEM audio servicing for Sunrise Nextor SYSTEM ROMs.
- Kept Sunrise Nextor SYSTEM ROMs on the Sunrise loader when MSX-MUSIC is selected instead of routing them through the FM-PAC ROM launcher.
- Added KonamiSCC mapper handling to the MSX-MUSIC/FM-PAC launcher so SCC mapper ROMs still boot when MSX-MUSIC is selected.

## PicoVerse 2350 Explorer v2.30

- Bumped Explorer to v2.30.
- Enabled the primary PSG DAC mirror for Sunrise SYSTEM ROM launches through a reusable SYSTEM audio service, including Sunrise + Mapper modes without stealing mapper I/O or blocking mapper bootstrap.
- Enabled the Dual PSG audio profile for Sunrise SYSTEM ROMs through the SYSTEM audio service.
- Split Dual PSG and primary PSG mirror output into separate stereo channels when both are enabled.
- Kept Sunrise SYSTEM PSG I/O serviced while producing audio buffers so non-mapper Sunrise launches remain stable with Dual PSG plus PSG mirror enabled.

## PicoVerse 2350 Explorer v2.29

- Bumped Explorer to v2.29.
- Applied each decoded MP3 frame's sample rate to the I2S output clock so MP3 files encoded at rates other than 44.1 kHz play at the correct pitch and speed.
- Retuned the existing MP3 I2S producer format in place on sample-rate changes instead of allocating a second audio pool during playback.
- Expanded Explorer ROM/MP3 record names to the 80-column detail-screen width and only truncate detail-screen names when they exceed the active screen width.
- Reported a missing microSD card to the MSX menu, display a clear no-card message when switching to the microSD source without a card inserted, and avoid reading stale records while keeping the MSX ROM code below the Pico communication window.
- Enabled SDCC code-size optimization for the MSX menu build so the ROM code stays clear of the `0xB900` Pico page-buffer window.
- Show the missing-microSD state as a single menu entry so the user can still switch to Flash or File Hunter from the microSD page.
- Reduced the MSX menu ROM code size by reusing shared row rendering, status text selection, last-line blinking, and source-switch helpers.
- Block File Hunter offline/status message rows from opening the detail screen and fail File Hunter network checks quickly when the ESP8266 does not respond.
- Align the missing-microSD row with File Hunter status rows so it is visible but not selected, scrolled, or actionable.
- Shorten 40-column missing-microSD and File Hunter offline messages so they fit without truncation.
- Refresh status-row text after column toggles so 80-column missing-microSD/File Hunter messages are not reused in 40-column mode.
- Bound each packed Explorer page name so all 19 microSD rows receive a visible name when long filenames fill the page-buffer string pool.
- Persist each ROM detail screen's audio profile and primary PSG setting to a `.PVC` file on microSD, reusing it on later launches and keeping those files out of the menu listing.
- Added the selected mapper to persisted `.PVC` ROM settings while keeping existing audio/PSG option files readable.
- Updated File Hunter catalog requests to use `http://msxpico.file-hunter.com/picoverse.php` while preserving the packed-list `base=1BA0` ROM query parameters required by the Explorer parser.
- Expanded File Hunter page records and detail rendering so 80-column ROM detail screens can show names up to the shared 71-character limit.


## PicoVerse 2350 Explorer v2.28

- Bumped Explorer to v2.28.
- Added microSD MP3 discovery, MP3 list/detail UI, play counter/status display, full filename rendering, and Play/Stop plus Pause/Resume controls backed by the Pico MP3 decoder service.
- Kept MP3 startup lazy so idle Stop/Pause/Resume commands do not start Core 1 during Explorer startup or normal flash browsing.
- Moved MP3 stream buffering to a dedicated 64 KB PSRAM allocation and reduced the SRAM fallback buffer to 8 KB, freeing SRAM for MSX-MUSIC initialization after MP3 playback.
- Kept the MP3/I2S backend on PIO1 SM2 and preserved the live MSX bus PIO programs when selecting, stopping, or leaving MP3 playback.
- Blocked File Hunter downloads above the 4 MB microSD/PSRAM launch limit used for SD-loaded ROMs.
- Kept the I2S DAC mute line asserted while Explorer is idle, only unmuting it after MP3 playback or a ROM audio profile starts.
- Added a ROM detail PSG option that mirrors primary PSG writes to the Pico DAC and mixes with existing ROM audio profiles.
- Updated Explorer, public, feature, Dual PSG, and PIO documentation for the MP3 player and primary PSG DAC mirroring behavior.
- Reused the MP3 I2S pipeline for ROM audio profiles after MP3 playback, resetting Core 1 before ROM launch and forcing the Pico audio singleton through disable/enable so SCC and MSX-MUSIC DMA output can restart cleanly.
- Moved MSX-MUSIC/FM-PAC I/O bus responders to PIO2, keeping FM-PAC ports isolated from the PIO1 I2S audio backend used by MP3 and ROM audio.
- Started MSX-MUSIC audio output only after the final FM-PAC ROM bus initialization releases `/WAIT`.
- Added temporary USB CDC diagnostics, with TinyUSB host disabled and a CDC-only PicoVerse debug product descriptor, to capture MP3-to-ROM audio launch checkpoints without using UART.

## PicoVerse 2350 Explorer v2.27

- Bumped Explorer to v2.27.
- Raised the MSX-MUSIC/FM-PAC audio output gain to match the existing SCC, SCC+, and Dual PSG I2S volume boost while preserving sample clipping protection.

## PicoVerse 2350 Explorer v2.26

- Bumped Explorer to v2.26.
- Added an MSX-MUSIC audio profile to the MSX Explorer ROM detail screen for non-system, non-SCC-class ROMs.
- Ported the LoadROM YM2413/emu2413 audio engine into Explorer and added runtime FM-PAC expanded-slot handling that reads the bundled FM-PAC BIOS from the Explorer UF2 flash payload.
- Moved the Explorer ROM cache from RP2350 SRAM to PSRAM so MSX-MUSIC can coexist with the existing Explorer features without overflowing firmware RAM.
- Updated the Explorer user documentation for MSX-MUSIC profile selection, mapper restrictions, and flash-payload FM-PAC BIOS storage.
- Allowed SPACE as well as ENTER to execute a ROM from the ROM detail screen when `Action: Run` is selected.
- Changed the File Hunter ROM download flow to show `Downloading...` instead of the generic network status check message.
- Removed the WiFi setup hint and F4 special handling from the help page return prompt.
- Increased the Explorer microSD ROM size limit and PSRAM SD ROM buffer from 2 MB to 4 MB.

## PicoVerse 2350 Explorer v2.25

- Bumped Explorer to v2.25.
- Reworked ROM audio selection around named, mutually exclusive audio profiles so new audio chips can be added without conflicting with SCC or system ROM options.
- Added a Dual PSG audio option for non-system, non-Konami SCC/Manbow2 ROMs, using the same secondary PSG port model as LoadROM.
- Updated the Explorer and PicoVerse 2350 documentation to describe Dual PSG support, audio profile exclusivity, and mapper restrictions.
- Added the shared PicoVerse 2350 Dual PSG implementation reference covering Explorer and LoadROM behavior.
- Updated the public README with user-facing Explorer instructions for selecting the Dual PSG audio profile on supported ROMs.
