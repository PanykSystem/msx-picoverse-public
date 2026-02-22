# MSX PicoVerse 2350 Features

The MSX PicoVerse 2350 is a Raspberry Pi Pico 2350-based MSX-compatible cartridge that offers a variety of features to enhance the MSX experience. Below is a list of its key features:

- **Multi-ROM Support**: The cartridge can store and run multiple MSX ROMs, allowing users to run different games and applications from a single cartridge.
- **Built-in Menu System**: A user-friendly menu system allows users to select and launch ROMs easily.
- **Explorer Firmware**: Unified menu that merges ROMs from flash and microSD in a single list.
- **Source Labels (FL/SD)**: Each ROM entry shows whether it comes from flash or microSD.
- **High Compatibility**: Supports a wide range of MSX ROMs, including games and applications for MSX1, MSX2, and MSX2+ systems.
- **Nextor DOS Support**: Compatible with Nextor DOS, enabling advanced file management and storage options. Currently supports Nextor OS 2.1.4 on the SD card. You can run DSK files using Nextor.
- **Long Name Support**: Supports ROM names up to 60 characters, making it easier to identify games and applications.
- **Support for Various Mappers**: Includes support for multiple ROM mappers, enhancing compatibility with different types of MSX software. Mappers supported include: PL-16, PL-32, KonSCC, Linear, ASC-08, ASC-16, Konami, NEO-8, NEO-16, and others.
- **Support for up to 1024 entries (Explorer)**: Per folder view, including ROMs and MP3s; the root view can also include flash entries.
- **Paged ROM List**: The menu loads pages on demand to keep the ROM list responsive with large libraries.
- **Search by Name (Explorer)**: Press `/`, type a query, and press Enter to jump to the first match.
- **On-device 40/80 Column Toggle**: Press `C` to switch between compact 40-column and wide 80-column menu layouts (when supported by the MSX model).
- **MP3 Player**: MP3 files on microSD appear in the menu and open a player screen with play/stop, mute, and visualizer controls.
- **ROM Detail Screen**: ROM entries open a details screen with mapper detection/override and audio profile selection before running.
- **SYSTEM ROM Priority**: System ROM entries remain at the top of the list, separate from name sorting.
- **microSD ROM Size Limit**: microSD ROM files are limited to 256 KB each (temporary limitation for prototype versions).
- **Easy ROM Management**: Users can easily add, remove, and organize ROMs using a simple tool on their PC.
- **Fast Loading Times**: Utilizes the high-speed capabilities of the Raspberry Pi Pico to ensure quick loading times for games and applications.
- **Firmware Updates**: The cartridge firmware can be updated via USB, allowing users to benefit from new features and improvements over time.
- **Compact Design**: The cartridge is designed to fit seamlessly into MSX systems without adding bulk.
- **SD Card Slot**: Equipped with an SD card slot for easy storage and transfer of ROMs and files.
- **Audio Profile Selection**: Allows users to select different audio profiles for enhanced sound compatibility with various MSX models. 
- **SCC/SCC+ Emulation**: When enabled for Konami SCC mapper ROMs, the cartridge can emulate the SCC and SCC+ sound chips in hardware, providing accurate audio output through an I2S DAC connected to the RP2350. This allows games that use SCC or SCC+ sound to have their full soundtrack without requiring an original SCC cartridge. For details on supported registers and behavior, see the [SCC/SCC+ documentation](docs/msx-picoverse-2350-scc.md).