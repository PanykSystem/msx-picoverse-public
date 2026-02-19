######################################################################
# MSX PICOVERSE PROJECT                                               
# (c) 2025 Cristiano Goncalves                                        
# The Retro Hacker                                                    
#                                                                      
# Makefile - build script for the MSX PICOVERSE 2350 Explorer tool    
#                                                                      
# This Makefile compiles the Explorer utility, regenerates the embedded 
# Raspberry Pi Pico firmware header, and packages the executable.      
######################################################################

# Toolchain configuration
CC      := gcc
XXD     := xxd

# Directory layout
SRCDIR  := src
BINDIR  := build
DISDIR  := dist
UTLDIR  := utils

# Build flags
VERBOSE ?= --verbose

# Debug control: set DEBUG=1 on the make command line to enable debug build
# Example: `make DEBUG=1`
DEBUG ?= 0

ifeq ($(DEBUG),1)
CCFLAGS := -g -DDEBUG
else
CCFLAGS := -g
endif

# Application metadata
VERSION ?= v1.00

# Project files
SOURCES := explorer.c
OUTFILE := explorer.exe

# Firmware assets (generated header embeds the Pico firmware image)
PICOBIN   := ../pico/explorer/build/explorer.bin
PICOBIN_H := $(SRCDIR)/explorer.h
MSXMENU := ../msx/dist/menu.rom
MSXMENU_H := $(SRCDIR)/menu.h
NEXTOR := ../nextor_sd/dist/nextor.rom
NEXTOR_H := $(SRCDIR)/nextor.h

# Helpers
RM := rm -f

.PHONY: all compile package clean

all: clean compile package

compile: $(BINDIR)/$(OUTFILE)

package: $(DISDIR)/explorer_$(VERSION).exe

$(BINDIR)/$(OUTFILE): $(BINDIR) $(SRCDIR)/$(SOURCES) $(PICOBIN_H) $(PICOBIN) $(MSXMENU_H) $(MSXMENU) $(NEXTOR_H) $(NEXTOR)
	@echo "Compiling $@"
	$(CC) $(CCFLAGS) -DAPP_VERSION=\"$(VERSION)\" $(SRCDIR)/$(SOURCES) -o $@

$(PICOBIN_H): $(PICOBIN)
	@echo "Embedding Pico firmware into header"
	$(UTLDIR)/$(XXD) -i $< $@

$(MSXMENU_H): $(MSXMENU)
	@echo "Embedding MSX menu ROM into header"
	$(UTLDIR)/$(XXD) -i $< $@

$(NEXTOR_H): $(NEXTOR)
	@echo "Embedding Nextor ROM into header"
	$(UTLDIR)/$(XXD) -i $< $@

$(BINDIR):
	@mkdir $@

$(DISDIR):
	@mkdir $@

$(DISDIR)/explorer_$(VERSION).exe: $(DISDIR) $(BINDIR)/$(OUTFILE)
	@echo "Packaging $@"
	cp $(BINDIR)/$(OUTFILE) $@

clean:
	@echo "Cleaning ...."
	$(RM) $(BINDIR)/*.exe $(BINDIR)/*.uf2 $(SRCDIR)/multirom.h $(SRCDIR)/menu.h $(SRCDIR)/nextor.h $(BINDIR)/explorer.rom $(BINDIR)/explorer_payload.bin
	$(RM) $(DISDIR)/*
