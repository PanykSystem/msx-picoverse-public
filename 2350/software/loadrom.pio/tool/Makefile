######################################################################
# MSX PICOVERSE PROJECT                                               
# (c) 2025 Cristiano Goncalves                                        
# The Retro Hacker                                                    
#                                                                      
# Makefile - build script for the MSX PICOVERSE 2350 loadROM tool    
#                                                                      
# This Makefile compiles the loadROM utility, regenerates the embedded 
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
CCFLAGS := -g

# Application metadata
VERSION ?= v1.01

# Project files
SOURCES := loadrom.c
OUTFILE := loadrom.exe

# Firmware assets (generated header embeds the Pico firmware image)
PICOBIN   := ../pico/loadrom/dist/loadrom.bin
PICOBIN_H := $(SRCDIR)/loadrom.h

# Helpers
RM := rm -f

.PHONY: all compile package clean

all: clean compile package

compile: $(BINDIR)/$(OUTFILE)

package: $(DISDIR)/loadrom_$(VERSION).exe

$(BINDIR)/$(OUTFILE): $(BINDIR) $(SRCDIR)/$(SOURCES) $(PICOBIN_H) $(PICOBIN)
	@echo "Compiling $@"
	$(CC) $(CCFLAGS) -DAPP_VERSION=\"$(VERSION)\" $(SRCDIR)/$(SOURCES) -o $@

$(PICOBIN_H): $(PICOBIN)
	@echo "Embedding Pico firmware into header"
	$(UTLDIR)/$(XXD) -i $< $@

$(BINDIR):
	@mkdir $@

$(DISDIR):
	@mkdir $@

$(DISDIR)/loadrom_$(VERSION).exe: $(DISDIR) $(BINDIR)/$(OUTFILE)
	@echo "Packaging $@"
	cp $(BINDIR)/$(OUTFILE) $@

clean:
	@echo "Cleaning ...."
	$(RM) $(BINDIR)/*.exe $(BINDIR)/*.uf2 $(SRCDIR)/loadrom.h
	$(RM) $(DISDIR)/*
