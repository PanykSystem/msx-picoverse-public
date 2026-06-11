# MSX PicoVerse 2350 WAVEGAME

WAVEGAME is an Explorer feature for microSD game ROMs that lets a running MSX game control streamed WAV playback from the PicoVerse 2350 cartridge. The game continues to execute from the cartridge while the Pico streams sidecar PCM audio from the same microSD folder as the ROM.

The MSX game writes one-byte commands to I/O port `0x92`. Explorer captures those writes, resolves the requested WAV and optional config files, and plays them through the same I2S DAC path used by Explorer's MP3/WAV player.

## File Layout

Place the game ROM and WAVEGAME audio files in the same microSD folder.

```text
/WAVEGAME/Example Game/
  Example Game.rom
  01.wav
  01.cfg
  02.wav
  pause.wav
```

Numbered songs use two-digit decimal names from `00.wav` through `63.wav`. If a requested numbered file does not exist, Explorer tries `multi.wav` in the same folder. If neither the numbered WAV nor `multi.wav` exists, no song is played.

WAVEGAME WAV files should be 48 kHz, mono, 16-bit PCM WAV. Explorer's parser can accept some other PCM WAV variants, but WAVEGAME titles should use the required format for predictable timing and offsets.

## Port 92h Protocol

The MSX game controls playback by writing a byte to I/O port `0x92`.

```z80
ld a,$85
out ($92),a
```

Commands:

| Value pattern | Meaning |
| --- | --- |
| `00000000` | Stop music with a 1-second fade out. |
| `0000xxxx` | Fade out music in `xxxx` seconds. A value of `0` is treated as 1 second. |
| `01xxxxxx` | Play song `xxxxxx` once. Example: `01000101` plays `05.wav` once. |
| `10xxxxxx` | Play song `xxxxxx` looped. Example: `10000101` plays `05.wav` continuously. |
| `11000000` | Toggle pause/resume of the current audio stream. |
| `11010000` | Defer the next command until the current song stops; if already stopped, run it immediately. |
| `11100000` | Continue the previous song with a 1-second fade in. |
| `11110000` | Play `pause.wav`. Use `11100000` to continue the previous song. |

Songs do not need to be stopped before starting another song. Starting a new song stores the previous song path, loop mode, loop offset, and current sample position so `11100000` can continue it later. This is mainly intended for pause/continue flows using `pause.wav`.

## Config Files

Offsets are decimal sample counts from the start of the song. Missing config files or missing lines mean offset `0`.

For a numbered song, an optional `NN.cfg` file may be placed next to `NN.wav`:

```text
loop_offset
start_offset
```

- Line 1 is the loop offset, used when the song is played once or looped.
- Line 2 is the start offset, used by both once and looped play commands.

When a numbered `NN.wav` is missing, Explorer falls back to `multi.wav`. In that mode, `multi.cfg` supplies offsets:

```text
loop_offset
song_00_start_offset
song_01_start_offset
song_02_start_offset
...
```

Line 1 is the shared loop offset. Line `N+1` is the start offset for song `N`. For example, song `05` reads its start offset from line 6.

## Explorer Setup

1. Copy the WAVEGAME folder to the microSD card.
2. Open Explorer and switch to the microSD source with `F2`.
3. Enter the WAVEGAME folder.
4. Select the ROM and open the ROM detail screen.
5. Use the detected mapper or manually select the correct mapper.
6. Keep the Explorer audio profile as `None`. When Explorer detects WAVEGAME sidecar audio next to the ROM, the ROM detail screen forces the audio profile list to `None` because cartridge audio profiles are incompatible with WAVEGAME streaming.
7. Enable PSG Mirror only when the WAVEGAME title also uses the primary PSG and should have those writes mirrored to the PicoVerse audio output.
8. Run the ROM.

WAVEGAME is active only for microSD-launched game ROMs. Flash-resident ROMs do not have a same-folder microSD path, so WAVEGAME is not enabled for them.

## Activation Rules

Explorer enables WAVEGAME during ROM launch when all of these are true:

- The selected ROM is launched from microSD.
- The ROM is not a SYSTEM/Sunrise launch mode.
- No Explorer cartridge audio profile is selected.
- A full SD path is available for the selected ROM.
- WAVEGAME sidecar audio is detected in the ROM folder: a numbered `NN.wav`, `multi.wav`, or `pause.wav` file.

These restrictions avoid conflicts with features that already own Core 1, the I2S audio pool, or the PIO I/O bus responder. PSG Mirror is allowed with WAVEGAME as a combined mode: WAVEGAME remains the Core 1/I2S owner, PSG register writes are handled from the same I/O write stream, and PSG samples are mixed into the WAV output. WAVEGAME initializes only the PIO2 I/O write captor needed to observe MSX `OUT` cycles; it does not enable an I/O read responder for the combined mode.

## PSG Mirror Mixing

WAVEGAME is compatible with Explorer's primary PSG Mirror option, but only through the WAVEGAME audio path. Cartridge audio profiles such as SCC/SCC+, Dual PSG, MSX-MUSIC, external SCC/SCC+, and YM2151/SFG are disabled for detected WAVEGAME ROMs and the audio profile remains `None`. The PSG Mirror option is still available independently.

When PSG Mirror is enabled, Explorer captures primary PSG writes to ports `0xA0` and `0xA1` from the same PIO2 I/O write stream that observes WAVEGAME writes to port `0x92`. The standalone PSG Core 1 audio loop is not started. Instead, WAVEGAME owns Core 1 and the I2S DAC, and the audio core adds one rendered PSG sample to each WAV output frame before sending the mixed PCM to I2S.

The PSG renderer follows the active WAV sample rate. WAVEGAME titles should use 48 kHz mono 16-bit PCM WAV files, so this keeps mirrored PSG pitch and timing aligned with the streamed soundtrack. If PSG Mirror is disabled, the WAVEGAME WAV stream is sent to I2S without the PSG mix layer.

## Firmware Flow

At launch time, Explorer loads the selected microSD ROM, derives the WAVEGAME folder from the ROM path, and starts the mapper loop. Mapper loops periodically drain captured I/O writes. When a game writes to port `0x92`, Explorer decodes the protocol byte and queues a WAVEGAME command to the audio core.

The audio core resolves `NN.wav`, `NN.cfg`, `multi.wav`, `multi.cfg`, and `pause.wav` on Core 1. This keeps FatFS audio/config reads on the same core that streams WAV data. The WAV player can seek to configured sample offsets, loop from a configured sample offset, and apply fade in/out envelopes during PCM output.

When PSG Mirror is enabled for a WAVEGAME ROM, Explorer does not start the standalone PSG audio thread. Instead, port `0xA0`/`0xA1` writes update the primary PSG state from the WAVEGAME I/O service loop, and the WAV output mixer adds one PSG sample to each WAV output frame. The PSG renderer is retuned to the active WAV sample rate so 48 kHz WAVEGAME files keep PSG pitch aligned.

## Debug Output

When USB CDC diagnostics are enabled, useful WAVEGAME messages include:

```text
WAVEGAME: active dir=/WAVEGAME/Example Game
WAVEGAME: PSG mirror active
WAVEGAME: port=92 data=85
WAVEGAME: index=5 loop=1 start=123456 loop_offset=120000 path=/WAVEGAME/Example Game/05.wav
MP3: core1 loop start
MP3: select '/WAVEGAME/Example Game/05.wav' size=0
MP3: resolved file size=12345678
WAV: fmt ch=1 rate=48000 bits=16 align=2 data_offset=44 data_size=12345634 total=128
```

SD ROM load diagnostics use the `SDLOAD:` prefix. For a successful WAVEGAME ROM launch you should see a complete SD load before WAVEGAME activation.

## Developer Checklist

- Confirm the ROM mapper first. WAVEGAME does not change mapper behavior.
- Keep the ROM and WAV/config files in the same folder.
- Export WAVEGAME audio as 48 kHz mono 16-bit PCM WAV.
- Use two-digit numbered WAV names or provide `multi.wav` plus `multi.cfg`.
- Keep Explorer's audio profile set to `None` for the WAVEGAME ROM.
- Test simple one-shot and looped commands before adding pause/resume or deferred commands.
- Watch for `WAVEGAME:` and `WAV:` diagnostics when USB CDC debug is enabled.

## Limitations

- WAVEGAME is Explorer-only and applies to microSD-launched game ROMs.
- WAVEGAME is mutually exclusive with Explorer cartridge audio profiles that already use the shared audio backend; PSG Mirror is the supported exception and is mixed through the WAVEGAME audio path.
- The current protocol supports 64 numeric song indices, `00` through `63`.
- WAV files are streamed from microSD by the shared audio core; very slow cards or unusually large files may affect start latency.
