# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_COPY_TO_RAM 0)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Fetch pico-extras if not available locally
set(PICO_EXTRAS_FETCH_FROM_GIT ON)
include(pico_extras_import.cmake)

project(explorer C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(explorer 
        hw_config.c
        nextor.c 
    explorer.c 
    mp3.c
    emu2212.c
    msx_bus.pio
)

pico_generate_pio_header(explorer ${CMAKE_CURRENT_SOURCE_DIR}/msx_bus.pio)

add_subdirectory(lib/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src build)
add_subdirectory(lib/picomp3lib/src picomp3lib_build)

pico_set_program_name(explorer "explorer")
pico_set_program_version(explorer "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(explorer 0)
pico_enable_stdio_usb(explorer 0)

# Add the standard library to the build
target_link_libraries(explorer
        pico_stdlib
        no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
        pico_multicore
    pico_audio_i2s
    picomp3lib
        )

target_compile_definitions(explorer PRIVATE
)

# Add the standard include files to the build
target_include_directories(explorer PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(explorer)
