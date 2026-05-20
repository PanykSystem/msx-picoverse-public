# MSX PicoVerse 2350 Explorer Tool Manual (EN-US)

The Explorer tool creates a UF2 image that flashes the PicoVerse 2350 cartridge with the Explorer firmware. The UF2 bundles:
- PicoVerse 2350 Explorer firmware.
- The full 32KB MSX Explorer menu ROM.
- A configuration table describing each ROM stored in flash (stored in flash immediately after the menu ROM).
- The ROM payloads that will live in Pico flash.
- The integrated File Hunter browser and WiFi configuration support used by the Explorer menu.

Use the Explorer tool when you want a menu that loads ROMs from flash, microSD, and the online File Hunter catalog on the PicoVerse 2350, or if you want to explore advanced features like MP3 playback, SCC/SCC+ audio, Dual PSG audio, and MSX-MUSIC on your MSX computer.

## Requirements

- Windows PC (the distributed binary is a Windows console app).
- A folder containing the `.ROM` files you want stored in flash.
- A PicoVerse 2350 cartridge and USB-C cable.
- Optional: a microSD card (for additional ROMs and MP3 files on SD).
- Optional but required for File Hunter: an ESP-01 / ESP8266 module with compatible firmware, installed on the PicoVerse 2350 WiFi header and configured for your wireless network.

## Limits

- Flash ROM entries created by the tool: up to 128 files.
- Combined Explorer menu limit: 1024 entries per folder view (folders + ROMs + MP3s; the root view also includes ROM files stored directly on the flash memory).
- Total flash ROM payload size: ~14 MB combined.
- Supported ROM size range on the flash: 8 KB to ~14 MB.
- ROM names in the menu are limited to 60 characters (longer names are truncated).
- File Hunter search text is limited to 24 characters.
- File Hunter downloads are saved as `.ROM` files directly in the root of the microSD card.

## Basic workflow

1. Put all `.ROM` files you want in the Pico flash into a single folder (no subfolders).
2. Run the tool in that folder to create `explorer.uf2`.
3. Put the PicoVerse 2350 into BOOTSEL mode and copy the UF2 to the `RPI-RP2` drive.
4. (Optional) Copy more `.ROM` and `.MP3` files to a microSD card for SD loading.
5. (Optional) Insert an ESP-01 module and configure WiFi from the Explorer menu if you want to browse File Hunter.
6. Insert the cartridge into your MSX and power on.

### Explorer menu capabilities

- **Folder navigation**: Organize your ROMs into folders on the microSD card. Enter folders by pressing Enter/Space on a folder name, and navigate back to parent folders using the ".." entry or by pressing Esc.
- **Search** by ROM name directly on the MSX by pressing `/`, typing part of the name, and pressing Enter to jump to the first match. Note that this feature only searches ROMs inside the current folder. When in the root, it searches all flash and SD ROMs located in the root.
- **File Hunter browser**: Press `F3` to browse online ROM results from File Hunter through the ESP-01 WiFi module. Results show the ROM name, size, and source, and selected ROMs can be downloaded and saved to the microSD root.
- **WiFi configuration**: Press `F4` to enter the WiFi setup flow used by the ESP8266P-compatible WiFi firmware.
- **Automatic detection** of MSX models that support 80-column text mode. Compatible machines boot the menu in 80 columns; others fall back to 40 columns, and you can press `C` at any time to toggle between layouts.
- MP3 entries are listed in the menu with a "MP3" type label and open an **MP3 player** screen when selected.
- ROM entries open a ROM screen that lets you inspect mapper detection, choose **audio profiles** including SCC/SCC+, Dual PSG, or MSX-MUSIC where supported, and enable optional WiFi support for Sunrise Nextor entries before running.

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
`PL-16`, `PL-32`, `KonSCC`, `Linear`, `PL-64`, `ASC-08`, `ASC-16`, `Konami`, `NEO-8`, `NEO-16`, `ASC-16X`, `MANBW2`.

Mapper detection on the Pico (for unknown SD ROMs) and on the PC tool both use a shared SHA-1 ROM database derived from openMSX's `softwaredb.xml`, falling back to heuristic scanning when the SHA-1 is not in the database.

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
- File Hunter downloads are saved directly to the root of the microSD card and then appear as normal SD ROM files in the Explorer root list.
- Folders are displayed in the menu and can be entered to browse their contents.
- The special ".." entry appears when inside a folder, allowing you to navigate back to the parent directory.

### microSD limitations

- The combined list is capped at 1024 entries per folder view (folders + ROMs + MP3s; the root view can also include flash entries).
- microSD ROM files are limited to 4 MB each. ROMs are streamed into the cartridge's 8 MB external PSRAM (QMI CS1, 52.5 MHz QPI) and executed from there; the first 256 KB are mirrored into the PSRAM ROM cache for mapper access. ROMs larger than 4 MB are skipped during enumeration.
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
- **F1**: Switch the Explorer list to flash ROMs.
- **F2**: Switch the Explorer list to microSD files and folders.
- **F3**: Open the integrated File Hunter browser.
- **F4**: Open WiFi configuration.
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

## File Hunter browser

Explorer includes an integrated File Hunter browser for PicoVerse 2350 cartridges fitted with an ESP-01 / ESP8266 module. The browser talks directly from the Pico firmware to the ESP module, queries the public File Hunter service, downloads the selected ROM into PSRAM, and then saves it as a `.ROM` file in the root of the microSD card.

The File Hunter integration is inspired by NataliaPC's MSX File Hunter Browser project and uses the public File Hunter catalog endpoint. See the external reference section near the end of this document for the upstream project link.

### Before using File Hunter

- Install an ESP-01 / ESP8266 module on the PicoVerse 2350 WiFi header.
- Use ESP firmware compatible with the ESP8266P / memio command protocol used by PicoVerse.
- Insert a FAT/FAT32 microSD card. File Hunter downloads are saved to the card root.
- If the card has not been mounted in the current session, press `F2` once to open the microSD list before downloading from File Hunter.
- Configure WiFi from the Explorer menu if the ESP module has not already been configured for your network.

### Opening File Hunter

Press `F3` from the Explorer menu. The menu frame is drawn immediately, then the lower-left status area shows "Retrieving File Hunter" in 80-column mode or "Retrieving..." in 40-column mode while the first page is fetched.

The initial File Hunter query starts with `1`, matching the current Explorer behavior for the first catalog request. After the first page is loaded, the browser shows the File Hunter result list with the same Explorer frame, footer, page counter, and 40/80-column support used by the normal menu.

### File Hunter list screen

Each result row shows:

- ROM name on the left.
- Size aligned to the right side of the row.
- Source label `FH`.
- Type label `ROM` aligned at the far right.

In 40-column mode, long selected names slide horizontally while the cursor remains on the row. In 80-column mode, longer names fit directly in the wider row.

The network state appears in the lower-left status area as "Network: Online" / "Network: Offline" in 80-column mode or "Net: Online" / "Net: Offline" in 40-column mode. The standard command hints stay on the right side of the last line.

### File Hunter controls

- **Up/Down**: Move through the result list.
- **Left/Right**: Change result pages.
- **Enter/Space**: Open the selected ROM detail screen.
- **/**: Search File Hunter. Type a query and press Enter to load matching results. Press Esc at the search prompt to cancel.
- **F1**: Leave File Hunter and return to the flash ROM list.
- **F2**: Leave File Hunter and return to the microSD list.
- **F3**: File Hunter is already selected; pressing it again has no effect.
- **F4**: Open WiFi configuration.
- **C**: Toggle 40/80-column mode when supported.
- **Esc**: Exit File Hunter and return to the normal Explorer menu.

### Searching File Hunter

Press `/` while inside File Hunter, type part of a title, and press Enter. Explorer sends that text as the File Hunter query and reloads the result list from page 1. Search text is limited to 24 characters.

If the ESP module is not connected to WiFi, the browser reports an offline state and the request fails instead of hanging indefinitely. When WiFi is still associating, the Pico waits briefly and shows a waiting status before reporting failure.

### ROM detail and download workflow

Selecting a File Hunter result opens a detail screen showing:

- ROM name.
- Size.
- Source: `FH`.
- Action: Download.

Press Enter or Space on the detail screen to download the selected ROM. Explorer first downloads the ROM from File Hunter into PSRAM. The status line shows a percentage counter from 0% to 100% while the download is active. After the download completes, the Pico saves the ROM from PSRAM to the root of the microSD card using the same filename shown by File Hunter.

When the save succeeds, the detail screen shows "Saved to microSD. Press key." Press any key to return to the File Hunter list. The Explorer root microSD list is refreshed after a successful save, so returning to the microSD root with `F2` lets you search for and launch the newly downloaded ROM as a normal SD ROM.

### File Hunter limitations and notes

- File Hunter requires a working ESP-01 / ESP8266 module and WiFi connection.
- A microSD card must be present because downloads are saved directly to the card root.
- Downloaded ROMs use File Hunter's filename. If a file with the same name already exists in the microSD root, it is replaced.
- Download and save speed depends on WiFi quality, the ESP module firmware, File Hunter response time, and microSD card performance.
- File Hunter entries are online catalog results, not flash entries. After saving, they become SD ROM entries.

## Known limitations

- Flash ROMs packaged by the tool must be in the root of the source folder (no subfolders in the flashing process, though SD folders are fully supported in the menu).
- The `-n` Nextor option is experimental and may not work on all MSX2 models.
- ROMs with unknown or unsupported mappers are skipped unless you force a mapper tag.
- Very deep folder nesting (more than 10+ levels) is supported but may have perception of slowness due to repeated folder scans.
- File Hunter browsing is unavailable without an ESP-01 / ESP8266 module, compatible ESP firmware, and a configured WiFi network.
- File Hunter downloads are stored in the root of the microSD card; Explorer does not create a separate File Hunter folder.

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
- **Audio**: Choose an audio profile with Left/Right (None, SCC, SCC+, Dual PSG, MSX-MUSIC). The menu only cycles through profiles supported by the selected ROM mapper.
- **Wifi**: For Sunrise Nextor entries only, choose whether to expose the ESP8266P WiFi BIOS before running. The default is No.
- **Action: Run**: Press Enter to launch the ROM.
- **Esc**: Return to the menu without running.

If a ROM mapper is unknown, the screen will briefly show "Detecting..." while the Pico attempts detection.

### Audio profile options

- **None**: No sound emulation. The ROM runs with its original audio through the MSX's built-in PSG (if the game uses PSG sound).
- **SCC**: Enables Konami SCC (standard) sound emulation. Use this for games that use the standard SCC chip with shared channel 4/5 waveforms (e.g., *Space Manbow*, *Salamander*, *Nemesis 2*, *Gradius 2*).
- **SCC+**: Enables Konami SCC+ (enhanced/SCC-I) sound emulation. Use this for games or homebrew that require SCC+ features with independent channel 4/5 waveforms.
- **Dual PSG**: Enables a secondary AY-3-8910 compatible PSG on I/O ports `0x10` (register select) and `0x11` (data write), matching the common Carnivore2 / MegaFlashROM / FlashJacks style second-PSG convention. Use this for ROMs or patches that explicitly support dual PSG music.
- **MSX-MUSIC**: Enables YM2413/MSX-MUSIC audio using an FM-PAC-compatible BIOS from the Explorer UF2 flash payload. Use this for regular ROMs that can use MSX-MUSIC ports `0x7C` and `0x7D`.

Explorer treats audio profiles as mutually exclusive: a ROM can run with no extra audio, SCC, SCC+, Dual PSG, or MSX-MUSIC, but not multiple cartridge audio engines at the same time.

The SCC and SCC+ audio profiles work with ROMs using the Konami SCC mapper (`KonSCC`) or the Manbow2 mapper (`MANBW2`). When a ROM is detected as `KonSCC` or `MANBW2`, the ROM screen pre-selects **SCC** in the Audio field; you can change it to None or SCC+ before pressing Run. Dual PSG and MSX-MUSIC are not offered for these mappers because the cartridge audio slot is reserved for SCC/SCC+.

The Dual PSG and MSX-MUSIC profiles are available for regular non-SYSTEM ROMs that are not Konami SCC or Manbow2. They are not offered for Sunrise/Nextor SYSTEM entries, NEO system entries, folders, or SCC-capable ROMs.

## SCC/SCC+ sound emulation

The PicoVerse 2350 Explorer firmware can emulate the Konami SCC and SCC+ (SCC-I) sound chips in hardware, generating audio through an I2S DAC connected to the RP2350. This gives MSX games that use SCC or SCC+ sound their full soundtrack without requiring an original SCC cartridge.

## Dual PSG sound emulation

The PicoVerse 2350 Explorer firmware can also emulate a second AY-3-8910 compatible PSG for ROMs that write to the standard secondary PSG ports. The Pico captures writes to `0x10` and `0x11`, feeds them to the emulated PSG, and streams the generated audio through the same I2S DAC path used by the other cartridge audio engines.

The MSX's internal PSG remains active as usual. The Dual PSG profile adds the external/secondary PSG only; software must be written or patched to use ports `0x10` and `0x11` for the extra three channels.

For firmware architecture, port handling, audio routing, and LoadROM/Explorer differences, see the [PicoVerse 2350 Dual PSG implementation guide](./msx-picoverse-2350-dualpsg.md).

## MSX-MUSIC emulation

The PicoVerse 2350 Explorer firmware can emulate the YM2413/MSX-MUSIC audio chip and expose an FM-PAC-compatible BIOS in an expanded cartridge slot when the **MSX-MUSIC** audio profile is selected. The BIOS is stored as a hidden payload in the Explorer UF2 after the WiFi configuration and ESP8266P BIOS payloads; it is read from flash at launch time and is not compiled into the Pico firmware binary.

MSX-MUSIC uses I/O ports `0x7C` for register select and `0x7D` for data writes, with synthesized audio routed to the same I2S DAC path used by the other Explorer audio profiles. The profile is mutually exclusive with SCC, SCC+, and Dual PSG, and is only offered for regular non-SYSTEM ROMs that are not Konami SCC or Manbow2.

## External references

- NataliaPC MSX File Hunter Browser: https://github.com/nataliapc/msx_filehunterbrowser
- File Hunter service: http://file-hunter.com/

Author: Cristiano Goncalves
Last updated: 05/16/2026
