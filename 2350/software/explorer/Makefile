######################################################################
# MSX PICOVERSE 2350 Explorer - Aggregate Makefile                   #
#                                                                    #
# Invokes the individual build systems for each Explorer component.  #
# Build order matters because later stages embed artifacts produced  #
# by earlier ones, so the tool target always runs last.              #
######################################################################

# Shared version (can be overridden on the command line)
VERSION ?= v1.25

# Sub-project locations
MSX_DIR       := msx
NEXTOR_DIR    := nextor_sd
PICO_DIR      := pico/explorer
PICO_BUILD    := $(PICO_DIR)/build
PICO_CONFIG   := $(PICO_BUILD)/CMakeCache.txt
TOOL_DIR      := tool

# Optional: allow callers to pick a specific CMake generator
CMAKE_GENERATOR ?=
CMAKE_GENERATOR_FLAG := $(if $(CMAKE_GENERATOR),-G "$(CMAKE_GENERATOR)",)
CMAKE ?= c:/Users/Cristiano/.pico-sdk/cmake/v3.29.9/bin/cmake

.PHONY: all msx nextor_sd pico tool clean

all: tool

msx:
	$(MAKE) -C $(MSX_DIR) VERSION=$(VERSION)

nextor_sd: msx
	$(MAKE) -C $(NEXTOR_DIR) VERSION=$(VERSION)

pico: nextor_sd $(PICO_CONFIG)
	$(CMAKE) --build $(PICO_BUILD)

$(PICO_CONFIG):
	$(CMAKE) $(CMAKE_GENERATOR_FLAG) -S $(PICO_DIR) -B $(PICO_BUILD)

tool: pico
	$(MAKE) -C $(TOOL_DIR) VERSION=$(VERSION)

clean:
	-$(MAKE) -C $(MSX_DIR) clean VERSION=$(VERSION)
	-$(MAKE) -C $(NEXTOR_DIR) clean VERSION=$(VERSION)
	-@$(CMAKE) --build $(PICO_BUILD) --target clean
	-$(MAKE) -C $(TOOL_DIR) clean VERSION=$(VERSION)
