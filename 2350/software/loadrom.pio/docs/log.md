# Change Log

## PicoVerse 2350 Loadrom v2.65

- Bumped the loadrom build version to v2.65.
- Gated the standalone OPL4 USB diagnostic counters out of production builds so timer/IRQ debug increments and output peak globals are only compiled when `OPL4_USB_DEBUG` is enabled.

## PicoVerse 2350 Loadrom v2.64

- Bumped the loadrom build version to v2.64.
- Added `-r1`/`--megaram-sd` and `-r2`/`--megaram-usb` Nextor system images with the existing Sunrise IDE ROM, a 1MB MSX memory mapper, and a new 1MB PSRAM-backed MegaRAM subslot.

## PicoVerse 2350 Loadrom v2.63

- Tuned the standalone OPL4 adaptive PCM voice limiter to be less aggressive: it now lowers the voice cap only after sustained late buffers, restores voices after repeated headroom, and keeps a higher minimum cap so melodic leads are less likely to be ducked when `--opl4-limit` is enabled.

## PicoVerse 2350 Loadrom v2.62

- Bumped the loadrom build version to v2.62.
- Expanded the standalone OPL4 `OPL4DBG` USB CDC report with timer, read latency, write FIFO/ring-drop, limiter, and clock-mode counters for field captures of missing-instrument issues, while keeping USB CDC disabled by default for normal timing validation. Left enabled by default so we can capture data in case of future reports of missing instruments or other audio issues.
- Added the OPL4-only `--lowclock` tool option for `-4`/`--opl4` images; default OPL4 UF2s keep the 300 MHz RP2350 clock, while `--lowclock` marks the OPL4 config header so the firmware boots at 282 MHz. This is to cover low quality PSRAM modules that cannot sustain the 300 MHz clock at full polyphony, which can cause audio underruns and missing instruments. The firmware reads the config header and sets the RP2350 clock divider to 282 MHz if the option is present.
- Raised the standalone OPL4 firmware's final I2S output gain to better match the SCC, MSX-MUSIC, and other emulated audio profiles while preserving signed 16-bit clipping protection (issue reported by Peter).
- Regenerated the embedded standalone OPL4 firmware payload and rebuilt the loadrom tool so the `loadrom -4` image uses the intended 300 MHz RP2350 clock instead of a stale 250 MHz binary (issue reported by Peter).

## PicoVerse 2350 Loadrom v2.61

- Updated the public README OPL4 credits to describe the PicoVerse 2350 standalone MoonSound firmware architecture, host-port handling, YMF278B/ymfm wrapper, YRW801-M ROM/RAM model, and third-party reference/ROM attribution more accurately.
- Fixed Neo-8/Neo-16 ROMs not launching when MSX-MUSIC/FM-PAC (`-f` / `-fmpac`) is enabled (the same ROM booted fine without `-f`). The FM-PAC expanded-slot launcher reset the Neo-8/Neo-16 bank windows to `{0,1,2,...}`, exposing segment 1 at MSX `0x4000`, so the cartridge "AB" boot header (which lives in segment 0) was hidden during the BIOS expanded-slot scan and the MSX dropped to BASIC. The windows now power on at segment 0 (`{0,0,0,...}`), matching the standalone `loadrom_neo8`/`loadrom_neo16` loaders, so the header is visible at `0x4000` and the game launches alongside FM-PAC.
- Fixed MSX-MUSIC/FM-PAC ROMs (`-f` / `-fmpac`) booting to BASIC instead of launching the game. The firmware was starting the YM2413 Core 1 + I2S audio output in `main()` *before* `loadrom_fmpac()` ran the expanded-slot cold-boot bootstrap, so the running audio core/DMA perturbed the timing-critical FM-PAC bus handshake and the MSX dropped to BIOS. The audio output launch (`msx_music_start_audio()`) is now deferred until after the bootstrap completes and `msx_pio_bus_init()` has brought the FM-PAC bus responder live — matching the Explorer ordering ("start MSX-MUSIC audio output only after the FM-PAC ROM bus initialization releases `/WAIT`"). `main()` now only runs `msx_music_init()` (emulator setup) for the MSX-MUSIC audio mode.
- Refreshed the generated ROM mapper SHA1 database from the current openMSX `softwaredb.xml` and updated the shared generator to parse the new attribute-based XML format.
- Changed the standalone OPL4 cartridge `/BUSDIR` driver from open-drain to push-pull so it matches a real OPL4/MoonSound cartridge (the WozBlaster CPLD drives `/BUSDIR = /RD & /CS` as a continuous push-pull output). The dedicated PIO2 state machine now drives the line actively HIGH when released and LOW only while answering an OPL4 I/O read, instead of releasing it to hi-Z. The previous open-drain release could let the MSX data-bus-buffer direction input float during register writes on machines without a strong `/BUSDIR` pull-up, occasionally corrupting OPL4 register writes and dropping instruments from some songs; driving it push-pull keeps the buffer direction defined at all times.
- Added an optional adaptive PCM voice limiter to the standalone OPL4 cartridge, selectable at programming time with the new `loadrom -4 --opl4-limit` option. Extreme-polyphony homebrew (e.g. Neon Horizon) drives the full YMF278B ~1.6-1.9x past the single-core real-time synthesis budget, underrunning the I2S and producing harsh broken audio. When the limiter is enabled, Core 1 measures its own per-buffer fill time and dynamically lowers the number of rendered PCM voices (`ymf278b::generate` renders only voices `[0, cap)`) just enough to stay real-time, raising the cap back toward the full 24 when the load eases. Light passages stay bit-identical; only the densest peaks shed their highest-numbered PCM voices, trading some peak polyphony for continuous, in-tune playback instead of underrun noise. The option is off by default (full fidelity, may underrun on extreme songs); the firmware reads the choice from a 16-byte config header the tool writes between the firmware image and the appended YRW801-M ROM.

## PicoVerse 2350 Loadrom v2.60

- Bumped the loadrom version to v2.60.
- Major OPL4/MoonSound overhaul with a new dual-core architecture on the RP2350, significantly improving compatibility, stability, and performance.
- Full MoonSound support now working correctly, including successful operation with MoonTest, SETOPL4, MBWAVE, and MoonBlaster.
- Fixed critical OPL4 emulation issues, improving compatibility with software that relies on accurate YMF278B behavior, status reporting, and memory access.
- Added proper OPL4 timer interrupt support through the MSX /INT line, allowing music players and replayers to run at the correct tempo.
- Corrected FM timer timing, fixing playback speed problems that previously caused some MoonSound applications to run extremely slowly.
- Improved audio synthesis performance, enabling real-time playback at full polyphony through SRAM execution, optimized memory access, and a 300 MHz RP2350 clock speed.
- Redesigned the OPL4 write handling path, eliminating dropped register writes during heavy audio workloads and improving reliability in complex games and music software.
- Updated /WAIT signal handling to use open-drain behavior, preventing conflicts when multiple cartridges are installed in the MSX.
- Held /WAIT asserted during the brief OPL4 boot initialization so the Z80 (and any game in another cartridge slot) is frozen until the cartridge's bus responder is live, fixing cases where a second cartridge's game started — and began its MoonSound music — before the OPL4 cartridge was ready. The hold spans only the sub-millisecond PSRAM/ymfm bring-up (well under the Z80 DRAM-refresh limit) and is released when the PIO takes over /WAIT; the PSRAM-init failure path releases /WAIT so the MSX can still boot.
- Added advanced OPL3 diagnostic capabilities to investigate remaining audio differences in a small number of FM-only titles.
- Compiled the YMF278B synthesis core (the ymfm sources and the `emu278b` wrapper) at `-O3 -funroll-loops` instead of the SDK default `-O2`, since they are the firmware's hottest per-sample path and sit at the real-time budget under maximum polyphony. The change is optimization-only (bit-identical audio) and gives extra instruction-level throughput; it does not fully resolve the documented extreme-polyphony ceiling but recovers a small amount of headroom.
- Drove the MSX `/BUSDIR` line (GPIO 37) during OPL4 I/O read responses, matching how a real OPL4/MoonSound cartridge behaves (the WozBlaster CPLD asserts `/BUSDIR = /RD & /CS`). The cartridge now flips the MSX main-board data-bus buffer toward the CPU only while it answers one of its own ports (0x7E/0x7F, 0xC4-0xC7) and drives D0-D7; otherwise `/BUSDIR` is released hi-Z (open-drain). Previously `/BUSDIR` was left as an unused input, so the firmware's data-bus drive fought the host's slot data buffer on every OPL4 read — visible as screen flashing / graphics artifacts (without a freeze) when a game in another cartridge slot was also driving the bus. Because `/BUSDIR` (GPIO 37) lies outside the 32-pin window the read/write state machines use for A0-A7, it is driven by a dedicated state machine on PIO2 (GPIO base 16) that mirrors the read responder's selection: it holds `/BUSDIR` low from the moment the read SM asserts `/WAIT` for an OPL4 port until `/RD` is released.
- Removed the temporary `OPL4_DRIVE_INT` diagnostic build switch; the `/INT` line is once again always driven from the YMF278B FM-timer interrupt. The switch had been added to test whether the maskable-interrupt path caused the multi-cartridge graphics-corruption crash, but that was ruled out — the cause was the missing `/BUSDIR` drive above.
Production-ready release, with hardware validation completed and accompanying implementation documentation added.

## PicoVerse 2350 Loadrom v2.59

- Bumped the loadrom sub-project version to v2.59.
- Added a dedicated, standalone OPL4 / YMF278B / MoonSound / WozBlaster cartridge firmware under `pico/opl4/`. This firmware implements a MoonSound-compatible OPL4 register interface and exposes the YRW801-M ROM as directly mapped wave memory, allowing it to pass the full MoonTest OPL4 detection and RAM test sequence without any host-side patches or timing compromises. The cartridge is built with the new `-4` / `--opl4` option in the loadrom tool, which produces a firmware-only UF2 with the OPL4 firmware and YRW801-M ROM embedded back-to-back in flash; this option is mutually exclusive with all other modes and external ROM files.
- Tightened the standalone OPL4 read/write synchronization window so `/WAIT`-held reads only proceed after the decoded write FIFO is empty and `/WR` has remained released long enough for rare MoonTest RAM readback edges.
- Added lock-free FM and wave register readback shadows for software such as MBWAVE that probes the cartridge by reading back recently written OPL4 register/data ports.
- Expanded the standalone OPL4 lock-free host facade to mirror the YMF278 PCM register file more completely: reset defaults, masked device/control bits, address-register auto-increment readback, memory-data readback, and queued PCM control/register writes for the audio core while keeping host memory transfers direct and non-busy.
- Reduced sporadic MoonTest RAM-test errors by keeping PCM memory-window control/address writes local to the lock-free host facade while memory access mode is active, using volatile PSRAM byte accesses for host RAM transfers, and sampling decoded MSX write data later inside the `/WR` low window.
- Removed the experimental Core1 synthesis pause because it did not improve MoonTest RAM stability and could interfere with repeated SETOPL4 `/P` ROM sample playback when tools leave the memory-access-mode bit enabled.
- Hardened USB diagnostics so debug builds never enqueue per-byte `0x7F`/wave-register-`0x06` memory read/write events while memory access mode is active, preventing MoonTest RAM loops from flooding the debug ring and perturbing bus timing.
- Changed the standalone OPL4 audio core to drain queued ymfm writes before each small audio chunk instead of once per 256-sample audio buffer, reducing key-on, wavetable, mixer, SETOPL4, and MBWAVE register latency without the per-sample locking overhead that made tracker playback crawl.

## PicoVerse 2350 Loadrom v2.58

- Bumped the loadrom sub-project version to v2.58.
- Raised the MSX-MUSIC/FM-PAC audio output gain to match the existing SCC, SCC+, and Dual PSG I2S volume boost while preserving sample clipping protection.
- Fixed embedded Sunrise+Mapper SD and Carnivore2 SD/USB type decoding so system type values 16-18 are not mistaken for audio-flagged external ROMs.

## PicoVerse 2350 Loadrom v2.57

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
