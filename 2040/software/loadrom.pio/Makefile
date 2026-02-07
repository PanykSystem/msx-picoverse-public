######################################################################
# MSX PICOVERSE PROJECT
# (c) 2025 Cristiano Goncalves
# The Retro Hacker
#
# Makefile - build script for loadrom.pio
#
# Builds both the RP2040 firmware and the PC tool.
######################################################################

# Shared version (can be overridden on the command line)
VERSION ?= v2.00

# Paths
PICO_DIR := pico/loadrom
PICO_BUILD := $(PICO_DIR)/build
PICO_CONFIG := $(PICO_BUILD)/CMakeCache.txt
TOOL_DIR := tool

# CMake options
CMAKE ?= c:/Users/Cristiano/.pico-sdk/cmake/v3.29.9/bin/cmake
CMAKE_GENERATOR ?=
CMAKE_GENERATOR_FLAG := $(if $(CMAKE_GENERATOR),-G "$(CMAKE_GENERATOR)",)
CMAKE_BUILD_TYPE ?= Release

# Make options
MAKE ?= make



.PHONY: all firmware tool clean

all: firmware tool

firmware: $(PICO_CONFIG)
	@echo "Building Pico firmware"
	@$(CMAKE) --build "$(PICO_BUILD)"

$(PICO_CONFIG):
	@$(CMAKE) $(CMAKE_GENERATOR_FLAG) -S "$(PICO_DIR)" -B "$(PICO_BUILD)" -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE)

tool:
	@echo "Building PC tool"
	@$(MAKE) -C "$(TOOL_DIR)" VERSION=$(VERSION)

clean:
	@echo "Cleaning firmware and tool"
	@$(MAKE) -C "$(TOOL_DIR)" clean VERSION=$(VERSION)
	@$(CMAKE) --build "$(PICO_BUILD)" --target clean || true
