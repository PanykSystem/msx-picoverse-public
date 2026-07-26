// MSX PICOVERSE PROJECT
// (c) 2026 Cristiano Goncalves
// The Retro Hacker
//
// sunrise_sd.h - Sunrise IDE SD card backend for MSX PicoVerse
//
// Provides a microSD-backed block I/O backend for the Sunrise IDE ATA
// emulation.  The public interface mirrors sunrise_ide.h's USB functions
// so that loadrom.c can launch either backend on Core 1.
//
// This work is licensed under a "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
// License". https://creativecommons.org/licenses/by-nc-sa/4.0/

#ifndef SUNRISE_SD_H
#define SUNRISE_SD_H

#include <stdint.h>
#include "sunrise_ide.h"

typedef struct {
	uint8_t number;
	uint8_t type;
	uint32_t start_lba;
	uint32_t sector_count;
	char label[12];
} sunrise_sd_partition_t;

#define SUNRISE_SD_FS_FAT16 1u
#define SUNRISE_SD_FS_FAT32 2u
#define SUNRISE_SD_FS_EXFAT 3u

// Set the pointer to the shared IDE context (call before launching core 1).
void sunrise_sd_set_ide_ctx(sunrise_ide_t *ide);

// Select the primary FAT16 partition number to expose as the Sunrise IDE disk.
void sunrise_sd_select_partition(uint8_t partition_number);

// Enumerate MBR primary FAT16 partitions up to 4GB.
uint8_t sunrise_sd_list_fat16_partitions(sunrise_sd_partition_t *out, uint8_t max_count);

// Enumerate MBR primary/logical FAT16, FAT32, and exFAT partitions.
uint8_t sunrise_sd_list_supported_partitions(sunrise_sd_partition_t *out, uint8_t max_count);

// SD card task loop — runs on Core 1.
// Initialises the microSD card via SPI and services IDE read/write requests.
void __not_in_flash_func(sunrise_sd_task)(void);

#endif // SUNRISE_SD_H
