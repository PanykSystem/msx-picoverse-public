# Change Log

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
