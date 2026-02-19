# MSX PicoVerse 2350 Explorer Tool Manual (EN-US)

The Explorer tool creates a UF2 image that flashes the PicoVerse 2350 cartridge with the Explorer firmware. The UF2 bundles:
- PicoVerse 2350 Explorer firmware.
- The full 32KB MSX Explorer menu ROM.
- A configuration table describing each ROM stored in flash (stored in flash immediately after the menu ROM).
- The ROM payloads that will live in Pico flash.

Use the Explorer tool when you want a menu that loads ROMs from both flash and microSD on the PicoVerse 2350 or if you want to explore advanced features like MP3 playback on your MSX computer.

## Requirements

- Windows PC (the distributed binary is a Windows console app).
- A folder containing the `.ROM` files you want stored in flash.
- A PicoVerse 2350 cartridge and USB-C cable.
- Optional: a microSD card (for additional ROMs and MP3 files on SD).

## Limits

- Flash ROM entries created by the tool: up to 128 files.
- Combined Explorer menu limit: 1024 entries per folder view (folders + ROMs + MP3s; the root view also includes ROM files stored directly on the flash memory).
- Total flash ROM payload size: ~14 MB combined.
- Supported ROM size range on the flash: 8 KB to ~14 MB.
- ROM names in the menu are limited to 60 characters (longer names are truncated).

## Basic workflow

1. Put all `.ROM` files you want in the Pico flash into a single folder (no subfolders).
2. Run the tool in that folder to create `explorer.uf2`.
3. Put the PicoVerse 2350 into BOOTSEL mode and copy the UF2 to the `RPI-RP2` drive.
4. (Optional) Copy more `.ROM` and `.MP3` files to a microSD card for SD loading.
5. Insert the cartridge into your MSX and power on.

### Explorer menu capabilities

- **Folder navigation**: Organize your ROMs into folders on the microSD card. Enter folders by pressing Enter/Space on a folder name, and navigate back to parent folders using the ".." entry or by pressing Esc.
- **Search** by ROM name directly on the MSX by pressing `/`, typing part of the name, and pressing Enter to jump to the first match. Note that this feature only searches ROMs inside the current folder. When in the root, it searches all flash and SD ROMs located in the root.
- **Automatic detection** of MSX models that support 80-column text mode. Compatible machines boot the menu in 80 columns; others fall back to 40 columns, and you can press `C` at any time to toggle between layouts.
- MP3 entries are listed in the menu with a "MP3" type label and open an **MP3 player** screen when selected.
- ROM entries open a ROM screen that lets you inspect mapper detection and choose **audio profiles** before running.

## Command-line usage

```
explorer.exe [options]
```

### Options

- `-h`, `--help` : Show usage help and exit.
- `-o <filename>`, `--output <filename>` : Set UF2 output filename (default is `explorer.uf2`).
- `-n`, `--nextor` : Include embedded Nextor ROM (experimental; MSX2-only). In the 2350 build, this is labeled “Nextor SD (IO)”.

### Example

```
explorer.exe
```

This scans the current directory and generates `explorer.uf2`.

## ROM mapper detection and tags

For the flash entries, the PC tool analyzes each ROM to determine the mapper. If you need to override it, add a mapper tag to the filename before `.ROM`. Tags are case-insensitive.

Supported tags:
`PL-16`, `PL-32`, `KonSCC`, `Linear`, `ASC-08`, `ASC-16`, `Konami`, `NEO-8`, `NEO-16`.

Example:

```
Knight Mare.PL-32.ROM
```

Notes:
- The `SYSTEM` tag is ignored and cannot be forced.
- Unsupported ROMs are skipped.

## Using microSD with Explorer

Explorer can load ROMs from a microSD card in addition to flash. ROMs on SD are merged into the menu list and marked with the "SD" source tag. You can organize your ROM collection into folders for easier navigation.

- Format the microSD card as FAT/FAT32.
- Copy `.ROM` and `.MP3` files to the root of the card or organize them in subfolders for better organization.
- SD ROMs appear in the menu with the source label "SD".
- MP3 files appear in the menu with the "MP3" type label and open the MP3 player screen.
- Flash ROMs appear with the source label "FL".
- Folders are displayed in the menu and can be entered to browse their contents.
- The special ".." entry appears when inside a folder, allowing you to navigate back to the parent directory.

### microSD limitations

- The combined list is capped at 1024 entries per folder view (folders + ROMs + MP3s; the root view can also include flash entries).
- microSD ROM files are limited to 256 KB each (this is temporary and only valid for the prototype versions, this limit will be lifted in future revisions).
- Unsupported or invalid ROMs are skipped (same mapper and size rules as flash).

### Performance note: MSX Response Time

MSX computers have slower processors compared to modern hardware. When navigating to a folder with a large number of ROMs (100+), the Pico will scan the directory contents and you will see a blinking "Loading..." message. This is normal behavior:

- **Small folders (< 50 ROMs)**: Directory listing typically completes in 1-2 seconds.
- **Large folders (100-900 ROMs)**: Directory listing may take over 5 seconds depending on the microSD card speed and the number of files.

For the best experience with very large ROM collections, consider organizing ROMs into subfolders by category (e.g., ACTION, PUZZLE, RPG) rather than storing all ROMs in a single folder.

## Menu usage (on MSX)

- **Up/Down**: Move selection up or down in the list.
- **Left/Right**: Change pages (when the list spans multiple pages).
- **Enter/Space**: Open the selected entry. Folders enter the directory, MP3 entries open the MP3 player screen, and ROM entries open the ROM screen.
- **Esc**: Navigate back to the parent folder (when inside a folder). Same as pressing Enter/Space on the ".." entry.
- **H**: Show help screen.
- **/**: Search ROM names. Type a partial name and press Enter to jump to the first matching ROM.
- **C**: Toggle between 40-column and 80-column layouts when your MSX supports it (auto-detects 80-column capable machines and defaults to 80 columns unless forced otherwise).

### Folder navigation workflow

1. The root menu lists all flash ROMs and files/folders from the microSD card root.
2. Folders appear in the list with a \<DIR\> indicator.
3. Press Enter or Space on a folder name to enter it and view its contents.
4. While loading a folder (especially large folders with many ROMs), you will see a blinking "Loading..." message on the bottom left corner. This is normal and indicates the Pico is scanning the directory.
5. Once loaded, navigate using Up/Down arrows as usual.
6. Press Esc or select the ".." entry to return to the parent folder or root.
7. Repeat to drill down through nested folders as needed.

## Known limitations

- Flash ROMs packaged by the tool must be in the root of the source folder (no subfolders in the flashing process, though SD folders are fully supported in the menu).
- The `-n` Nextor option is experimental and may not work on all MSX2 models.
- ROMs with unknown or unsupported mappers are skipped unless you force a mapper tag.
- Very deep folder nesting (more than 10+ levels) is supported but may have perception of slowness due to repeated folder scans.

## MP3 player screen

Selecting an MP3 entry opens a dedicated player screen with playback controls, mode selection, and a visualizer.

The screen displays four selectable options arranged vertically:

1. **Action**: Play or Stop the current MP3 file.
2. **Mute**: Toggle audio mute on/off.
3. **Mode**: Select playback mode (Single, All, or Random).
4. **Visualizer**: Toggle the audio visualizer on/off.

### Controls

- **Up/Down**: Move between the four selectable options.
- **Enter**: Toggle or cycle the selected option:
  - **Action**: Plays the file if stopped, or stops if playing.
  - **Mute**: Toggles mute on/off.
  - **Mode**: Cycles through Single → All → Random → Single.
  - **Visualizer**: Toggles the audio visualizer on/off.
- **Esc**: Stop playback (if playing) and return to the menu.
- **C**: Toggle 40/80-column layout (supported systems only).

### Playback modes

- **Single**: Plays only the selected MP3 file. When the song ends, playback stops.
- **All**: Automatically plays all MP3 files in the current folder in sequence. After the last file finishes, playback loops back to the first MP3 file in the folder.
- **Random**: Automatically plays MP3 files from the current folder in random order. After each file ends, a random MP3 file from the folder is selected and played.

The playback mode is displayed in the Mode line (e.g., "Mode: All"). When a new file starts playing in All or Random mode, the file name and size are automatically updated on the screen.

### Status display

Status details are shown at the bottom of the screen, including:
- **Play/Stop state**: "PLAY" or "STOP"
- **Elapsed time**: Current playback position in MM:SS format
- **Mute indicator**: "MUTE" when muted, blank when unmuted
- **Error indicator**: "ERR" if an error occurs, blank otherwise

## ROM screen

Selecting a ROM entry opens a ROM details screen before running:

- **Mapper**: Shows the detected mapper (for SD ROMs) and allows manual override using Left/Right.
- **Audio**: Choose an audio profile with Left/Right (None, SCC, SCC+).
- **Action: Run**: Press Enter to launch the ROM.
- **Esc**: Return to the menu without running.

If a ROM mapper is unknown, the screen will briefly show "Detecting..." while the Pico attempts detection.

### Audio profile options

- **None**: No sound emulation. The ROM runs with its original audio through the MSX's built-in PSG (if the game uses PSG sound).
- **SCC**: Enables Konami SCC (standard) sound emulation. Use this for games that use the standard SCC chip with shared channel 4/5 waveforms (e.g., *Space Manbow*, *Salamander*, *Nemesis 2*, *Gradius 2*).
- **SCC+**: Enables Konami SCC+ (enhanced/SCC-I) sound emulation. Use this for games or homebrew that require SCC+ features with independent channel 4/5 waveforms.

The SCC and SCC+ audio profiles only work with ROMs using the Konami SCC mapper (KonSCC). For non-SCC mapper ROMs, the audio selection has no effect.

## SCC/SCC+ sound emulation

The PicoVerse 2350 Explorer firmware can emulate the Konami SCC and SCC+ (SCC-I) sound chips in hardware, generating audio through an I2S DAC connected to the RP2350. This gives MSX games that use SCC or SCC+ sound their full soundtrack without requiring an original SCC cartridge.

Author: Cristiano Goncalves
Last updated: 02/16/2026
