# PicoVerse for MSX

**The MSX experience driven by the Raspberry Pi Pico family of microcontrollers.**

PicoVerse is a community-driven effort to build versatile MSX cartridges powered by Raspberry Pi Pico development boards. The project pairs accessible hardware designs with ready-to-flash firmware so MSX users can jump straight into loading games, tools, and Nextor without having to compile sources.

| Cartridge Label - 2040 Variant | Cartridge Label - 2350 Variant |
|---------|---------|
|![PicoVerse 2040 Label](/labels/PicoVerse_2040_Label_1.png) | ![PicoVerse 2350 Label](/labels/PicoVerse_2350_Label_1.png) |

PicoVerse is designed as an open-source, independent, and documented MSX cartridge platform. Compatibility with other projects is neither a goal nor guaranteed (I tested some without much success); running third‑party software on PicoVerse hardware, or PicoVerse firmware on other boards, is at your own risk. The source and manufacturing files are openly available so you can learn, experiment, and build on them for the MSX community, subject to the project license.

## Whats New in PicoVerse?

- New PIO based **MultiROM** firmware for PicoVerse 2040, which allows more reliable ROM loading and better support for edge cases. ***(NEW!)***
- Updated documentation with details about the MultiROM PIO implementation for PicoVerse 2040 [MSX PicoVerse 2040 MultiROM PIO Implementation](/docs/msx-picoverse-2040-multirom-pio.md) ***(NEW!)***

## Project Highlights
- Multi-ROM loader with an on-screen menu and mapper auto-detection.
- Single-ROM LoadROM workflow for instant booting of one title without entering the menu.
- Explorer firmware (RP2350) merges flash and microSD ROMs, labels the source (FL/SD), adds MP3 playback, and supports on-device search.
- Ready-made Nextor builds with USB (RP2040) or microSD (RP2350) storage bridges.
- PC-side tooling that generates UF2 images locally for quick drag-and-drop flashing.
- BOMs, and production-ready Gerbers.
- Active development roadmap covering RP2040 and RP2350-based cartridges.

## Documentation

**MultiROM Guides**
- [PicoVerse 2040 MultiROM Guide Manual (English)](/docs/msx-picoverse-2040-multirom-tool-manual.en-US.md)
- [PicoVerse 2350 MultiROM Tool Manual (English)](/docs/msx-picoverse-2350-multirom-tool-manual.en-us.md)

**LoadROM Guides**
- [MSX PicoVerse 2040 LoadROM Tool Manual (English)](/docs/msx-picoverse-2040-loadrom-tool-manual.en-us.md)
- [MSX PicoVerse 2350 LoadROM Tool Manual (English)](/docs/msx-picoverse-2350-loadrom-tool-manual.en-us.md)

**Explorer Guides** 
- [MSX PicoVerse 2350 Explorer Tool Manual (English)](/docs/msx-picoverse-2350-explorer-tool-manual.en-us.md)

**Reference Material**
- [PicoVerse 2040 Features Overview](/docs/msx-picoverse-2040-features.md)
- [PicoVerse 2350 Features Overview](/docs/msx-picoverse-2350-features.md)
- [MSX PicoVerse 2040 LoadROM PIO Implementation](/docs/msx-picoverse-2040-loadrom-pio.md)
- [MSX PicoVerse 2040 MultiROM PIO Implementation](/docs/msx-picoverse-2040-multirom-pio.md) ***(NEW!)***
- [Nextor Pico Bridge Protocol](/docs/Nextor-Pico-Bridge-Protocol.md)

## Hardware Variants

### PicoVerse 2040 Cartridge

| Prototype PCB (front) | Prototype PCB (back) |
|---------|---------|
| ![Image 1](/images/20241230_001854885_iOS.jpg) | ![Image 2](/images/20241230_001901504_iOS.jpg) | 

- Based on RP2040 boards exposing 30 GPIO pins (not compatible with stock Raspberry Pi Pico pinout).
- Up to 16 MB of flash for MSX ROMs with support for Plain16/32, Linear0, Konami SCC, Konami, ASCII8/16, NEO-8, and NEO-16 mappers.
- USB-C port doubles as a bridge for Nextor mass storage.

#### Bill of Materials

![alt text](/images/2025-12-02_20-05.png)

Interactive BOM available at [PicoVerse 2040 BOM](https://htmlpreview.github.io/?https://raw.githubusercontent.com/cristianoag/msx-picoverse-public/refs/heads/main/2040/hardware/MSX_PicoVerse_2040_1.3_bom.html)

| Reference | Description | Quantity | Link |
| --- | --- | --- | --- |
| U1 | RP2040 Dev Board 30 GPIO pins exposed | 1 | [AliExpress](https://s.click.aliexpress.com/e/_c4MuM9st) |
| C1 | 0603 0.1 µF Ceramic Capacitor | 1 | [AliExpress](https://s.click.aliexpress.com/e/_c2w5e36V) |
| C2 | 0603 10 µF Ceramic Capacitor | 1 | [AliExpress](https://s.click.aliexpress.com/e/_c2w5e36V)|
| R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13| 0603 10 kΩ Resistor | 12 | [AliExpress](https://s.click.aliexpress.com/e/_c3XBv4od)|
| R1 | 0603 2 KΩ Resistor | 1 | [AliExpress](https://s.click.aliexpress.com/e/_c3XBv4od)|
| D1 | 1N5819 SOD-123 Diode | 1 | [AliExpress](https://s.click.aliexpress.com/e/_c4WEKCuz) |
| Q1, Q2, Q3, Q4, Q5 | BSS138 SOT-23 Transistor | 5 | [AliExpress](https://s.click.aliexpress.com/e/_c2veWxcD)|


### PicoVerse 2350 Cartridge

| Prototype PCB (front) | Prototype PCB (back) |
|---------|---------|
| ![Image 1](/images/20250208_180923511_iOS.jpg) | ![Image 2](/images/20250208_181032059_iOS.jpg) |

- Targets RP2350 boards exposing all 48 GPIO pins (not compatible with standard Pico 2 boards).
- Adds microSD storage, ESP8266 WiFi header, and I2S audio expansion alongside 16 MB flash space.
- Extra RAM to support advanced emulation features in future firmware releases.
- Explorer firmware can load ROMs from both flash and microSD, with source labels and search.
- Shares the same ROM mapper support list as the 2040 build.

#### Bill of Materials ***(NEW!)***

![alt text](/images/2026-02-07_19-11.png)

Interactive BOM available at [PicoVerse 2350 BOM](https://htmlpreview.github.io/?https://raw.githubusercontent.com/cristianoag/msx-picoverse-public/refs/heads/main/2350/hardware/MSX_PicoVerse_2350_1.0-BETA_bom.html) 

| Reference | Description | Quantity | Link |
| --- | --- | --- | --- |
| U1 | WaveShare Core2350B Dev Board 48 GPIO pins exposed (8Mb PSRAM)| 1 | [AliExpress](https://pt.aliexpress.com/item/1005009578742534.html?spm=a2g0o.order_list.order_list_main.112.62b91802b6L8HW&gatewayAdapt=glo2bra) |
|U2| UDA1334A I2S Stereo DAC| 1 | [AliExpress](https://s.click.aliexpress.com/e/_c3gam5lH) |
|U3| ESP-01 ESP8266 WiFi Module| 1 | [AliExpress](https://s.click.aliexpress.com/e/_c4a5rnGj) |
|C1| 0603 0.1 µF Ceramic Capacitor | 1 | [AliExpress](https://s.click.aliexpress.com/e/_c3tkUxHz) |
|C2,C3| 0603 10 µF Ceramic Capacitor | 2 | [AliExpress](https://s.click.aliexpress.com/e/_c3tkUxHz)|
|R1| 0603 2 KΩ Resistor | 1 | [AliExpress](https://s.click.aliexpress.com/e/_c43uKcEj)|
|R2, R3, R6, R7, R8| 0603 10 kΩ Resistor | 4 | [AliExpress](https://s.click.aliexpress.com/e/_c43uKcEj)|
|R4, R5| 0603 5.1 kΩ Resistor | 2 | [AliExpress](https://s.click.aliexpress.com/e/_c43uKcEj)|
|R9, R10| 0603 330 Ω Resistor | 2 | [AliExpress](https://s.click.aliexpress.com/e/_c43uKcEj)|
|D1| 1N5819 SOD-123 Diode | 1 | [AliExpress](https://s.click.aliexpress.com/e/_c4WEKCuz) |
|SW1| Tactile Push Button Switch | 1 | [AliExpress](https://s.click.aliexpress.com/e/_c2IPGkzv) |
|J1| USB-C 16 Pin Connector | 1 | [AliExpress](https://s.click.aliexpress.com/e/_c4PtBc51) |
|J2| microSD Card Slot | 1 | [AliExpress](https://s.click.aliexpress.com/e/_c4Pzbd7Z) |

## Repository Contents
- `hardware/` – Production-ready Gerbers, fabrication notes, and BOMs for each supported dev board.
- `software/` – MultiROM PC utilities (`multirom.exe`) and menu ROM assets for both cartridge families.
- `docs/` – Feature lists, usage walkthroughs, and revision history for each cartridge family.
- `images/` – Board renders and build photos for quick identification.
- `labels/` – Printable Patola style cartridge label designs for both hardware variants.

## Quick Start
1. **Pick your target board**: Select the hardware revision that matches the RP2040 or RP2350 carrier you own, then grab the corresponding Gerber/BOM pack.
2. **Manufacture or assemble**: Send the Gerbers to your PCB house or build from an ordered kit. Follow the assembly notes included in each hardware bundle.
3. **Generate the UF2 image**:
   - For multiple titles, place your `.rom` files beside the MultiROM tool for your cartridge family (`2040/software/multirom/multirom.exe` or `2350/software/multirom/multirom.exe`) and run `multirom.exe` to create `multirom.uf2`.
   - For an instant boot into a single game, place the desired `.rom` next to the LoadROM tool for your cartridge family and run `loadrom.exe <file> [-o custom.uf2]` to create a dedicated UF2 that skips the menu.
   - For a combined flash + microSD menu on RP2350, use the Explorer tool (`2350/software/explorer/tool/explorer.exe`) to create `explorer.uf2`, then copy extra `.rom` and `.mp3` files to the microSD card.
4. **Flash the firmware**:
   - Hold BOOTSEL while connecting the cartridge to your PC via USB-C.
   - Copy the freshly generated UF2 (`multirom.uf2`, `loadrom.uf2`, or `explorer.uf2`) to the RPI-RP2 drive that appears.
   - Eject the drive; the board reboots and stores the image in flash.
5. **Enjoy on MSX**: Insert the cartridge, power on the computer, pick a ROM from the menu, or launch Nextor to access USB or microSD storage.

## MultiROM Menu

| PicoVerse 2040 MultiRom | PicoVerse 2350 MultiRom |
|---------|---------|
|![alt text](/images/multirom_2040_menu.png)|![alt text](/images/multirom_2350_menu.png)|

To use the MultiROM menu, insert the PicoVerse cartridge into your MSX and power it on. The system will boot directly into the MultiROM menu, which displays a list of all ROMs embedded in the cartridge. Each entry is named after the ROM filename (up to 50 characters), and longer names are shown using a scrolling effect.

Navigate the menu using the keyboard arrow keys. Use the Up and Down keys to move through the list of ROMs, and if more than 19 ROMs are present, use the Left and Right keys to switch between pages. To start a game or application, select it and press Enter or Space; the MSX will boot the ROM using the appropriate mapper configuration automatically.

While in the menu, pressing the H key opens a help screen with basic instructions; press any key to return to the main menu. Once a ROM is launched, control is handed over entirely to the selected software, just as if it were a physical cartridge inserted into the MSX.

Check the detailed MultiROM guide in the documentation folder for advanced features, troubleshooting tips, and mapper support details.

## LoadROM Tool

The LoadROM tool targets situations where you want the PicoVerse to behave like a traditional single-game cartridge. Instead of showing the MultiROM menu, the Pico boots straight into one ROM embedded in the UF2 image.

- **Input**: exactly one `.ROM` file. Mapper type is auto-detected with the same heuristics as MultiROM, and you can still force a mapper via filename tags such as `.KonSCC.ROM` or `.PL-32.ROM`.
- **Output**: `loadrom.uf2` by default, or any filename you pass via `-o`. The UF2 contains the firmware, a 59-byte configuration record (title, mapper, size, flash offset), and the ROM payload.
- **Workflow**:
   1. Open a Command Prompt or PowerShell window in `2040/software/loadrom/tool`.
   2. Run `loadrom.exe -o mygame.uf2 \\path\\to\\Game.ROM` (the tool also accepts drag-and-drop onto the EXE).
   3. Observe the reported ROM name, size, mapper status (auto vs forced), and Pico offset before the UF2 is written.
   4. Put the Pico into BOOTSEL mode and copy the generated UF2 to the `RPI-RP2` drive.
   5. Insert the cartridge into your MSX—on power-up the embedded game launches immediately.

Consult the LoadROM manuals linked above for screenshots, troubleshooting, and in-depth explanations of mapper forcing, UF2 structure, and limitations.

## Explorer Firmware (RP2350)

|Explorer Menu - 80 Columns|Explorer Search|
|---|---|
|![](/images/WIN_20260207_19_59_37_Pro.jpg)|![](/images/WIN_20260207_20_00_51_Pro.jpg)|
|<center>**MP3 Selection Screen**|<center>**MP3 Player Screen**|
|![](/images/WIN_20260207_20_01_21_Pro.jpg)|![](/images/WIN_20260207_20_01_58_Pro.jpg)|
|<center>**ROM Details Screen**||
|![](/images/WIN_20260207_20_03_20_Pro.jpg)||


Explorer is a 2350-only firmware that merges ROMs stored in flash with additional ROMs and MP3 files on the microSD card. ROMs are labeled with source tags (FL/SD), MP3 entries open a player screen, the list supports paging, and you can search by name directly in the menu. Use the Explorer tool to build the UF2 and copy extra ROMs and MP3 files to the microSD card. See the Explorer manual for limits (flash vs SD capacity, 256 KB SD ROM limit, and supported formats).

You can have up to 1024 entries per folder view (folders + ROMs + MP3s; the root view can also include flash entries). The menu auto-detects whether the MSX supports 80-column text mode and boots accordingly; you can also press `C` at any time to toggle between 40- and 80-column layouts.

A search function is available by pressing `/` in the menu. Type part of a ROM name and press Enter to jump to the first matching entry. Press `H` to view the help screen.

## Compatibility & Requirements
- Works with MSX, MSX2, MSX2+, and MSX TurboR systems. Mapper support covers the most common game and utility formats.
- Requires Windows or Linux to run the PC-side UF2 builder utilities.
- Ensure your development board matches the pinout documented for each hardware revision before soldering.

## License & Usage

![Creative Commons Attribution-NonCommercial-ShareAlike 4.0](/images/ccans.png)

All hardware and firmware binaries in this repository are released under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International license. Personal builds and community tinkering are encouraged, but commercial use or resale requires explicit authorization from the author.

## Feedback & Community
Questions, test reports, and build photos are welcome. Open an issue on the public repository or reach out through the MSX retro hardware forums where PicoVerse updates are posted.
