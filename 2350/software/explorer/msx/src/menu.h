// MSX PICOVERSE PROJECT
// (c) 2025 Cristiano Goncalves
// The Retro Hacker
//
// menu.h - MSX Explorer menu definitions for PicoVerse 2350
// 
// This work is licensed  under a "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
// License". https://creativecommons.org/licenses/by-nc-sa/4.0/

#include <msx_fusion.h>
#include <stdio.h>
#include <stdint.h>
#include "bios.h"

#ifndef EXPLORER_VERSION
#define EXPLORER_VERSION "v1.00"
#endif

// Define maximum files per page and screen properties
#define FILES_PER_PAGE 19   // Maximum files per page on the menu
#define MAX_FILE_NAME_LENGTH 60     // Maximum size of the ROM name
#define ROM_RECORD_SIZE (MAX_FILE_NAME_LENGTH + 1 + sizeof(unsigned long) + sizeof(unsigned long))  // Size of the ROM record in bytes
#define MAX_ROM_RECORDS 65535 // Maximum ROM files supported (counter limit)
#define MEMORY_START 0xA000 // Start of the memory area to read the ROM records
#define ROM_SELECT_REGISTER 0xBB01 // Memory-mapped register that selects the ROM to load
#define JIFFY 0xFC9E
#define SOURCE_SD_FLAG 0x80
#define FOLDER_FLAG 0x40
#define MP3_FLAG 0x20
#define OVERRIDE_FLAG 0x10
#define NAME_COL_WIDTH 22
#define MENU_FORCE_40_COLUMNS 0
#define CTRL_BASE_ADDR 0xBFF0
#define CTRL_COUNT_L (CTRL_BASE_ADDR + 0)
#define CTRL_COUNT_H (CTRL_BASE_ADDR + 1)
#define CTRL_PAGE    (CTRL_BASE_ADDR + 2)
#define CTRL_STATUS  (CTRL_BASE_ADDR + 3)
#define CTRL_CMD     (CTRL_BASE_ADDR + 4)
#define CTRL_MATCH_L (CTRL_BASE_ADDR + 5)
#define CTRL_MATCH_H (CTRL_BASE_ADDR + 6)
#define CTRL_MAPPER  (CTRL_BASE_ADDR + 7)
#define CTRL_ACK     (CTRL_BASE_ADDR + 8)
#define CTRL_MAGIC   0xA5
#define CTRL_QUERY_BASE 0xBFC0
#define CTRL_QUERY_SIZE 32
#define DATA_BUFFER_END CTRL_QUERY_BASE
#define DATA_BUFFER_SIZE (DATA_BUFFER_END - MEMORY_START)
#define CMD_APPLY_FILTER 0x01
#define CMD_FIND_FIRST  0x02
#define CMD_ENTER_DIR   0x03
#define CMD_DETECT_MAPPER 0x04
#define CMD_SET_MAPPER 0x05

#define DATA_MAGIC_0 'P'
#define DATA_MAGIC_1 'V'
#define DATA_MAGIC_2 'E'
#define DATA_MAGIC_3 'X'
#define DATA_VERSION 1
#define DATA_HEADER_SIZE 24
#define DATA_HDR_MAGIC 0
#define DATA_HDR_VERSION 4
#define DATA_HDR_FLAGS 5
#define DATA_HDR_SIZE 6
#define DATA_HDR_TOTAL_COUNT 8
#define DATA_HDR_PAGE_INDEX 10
#define DATA_HDR_PAGE_COUNT 12
#define DATA_HDR_TABLE_OFFSET 14
#define DATA_HDR_TABLE_SIZE 16
#define DATA_HDR_STRING_OFFSET 18
#define DATA_HDR_STRING_SIZE 20
#define DATA_HDR_PAYLOAD_OFFSET 22
#define DATA_RECORD_TABLE_ENTRY_SIZE 4
#define TLV_NAME_OFFSET 0x01
#define TLV_MAPPER 0x02
#define TLV_SIZE 0x03

#define MP3_CTRL_BASE      0xBFE0
#define MP3_CTRL_CMD       (MP3_CTRL_BASE + 0)
#define MP3_CTRL_STATUS    (MP3_CTRL_BASE + 1)
#define MP3_CTRL_INDEX_L   (MP3_CTRL_BASE + 2)
#define MP3_CTRL_INDEX_H   (MP3_CTRL_BASE + 3)
#define MP3_CTRL_ELAPSED_L (MP3_CTRL_BASE + 4)
#define MP3_CTRL_ELAPSED_H (MP3_CTRL_BASE + 5)
#define MP3_CTRL_TOTAL_L   (MP3_CTRL_BASE + 6)
#define MP3_CTRL_TOTAL_H   (MP3_CTRL_BASE + 7)

#define MP3_CMD_SELECT      0x01
#define MP3_CMD_PLAY        0x02
#define MP3_CMD_STOP        0x03
#define MP3_CMD_TOGGLE_MUTE 0x04
#define MP3_CMD_TONE        0x05

#define MP3_STATUS_PLAYING  0x01
#define MP3_STATUS_MUTED    0x02
#define MP3_STATUS_ERROR    0x04
#define MP3_STATUS_EOF      0x08
#define MP3_STATUS_BUSY     0x10
#define MP3_STATUS_READING  0x20
#define MP3_STATUS_TONE     0x40

// Folders to exclude from SD listing (keep in sync with pico/explorer.c)
#define EXCLUDED_SD_FOLDER_COUNT 1
static const char *EXCLUDED_SD_FOLDERS[EXCLUDED_SD_FOLDER_COUNT] = {
    "System Volume Information"
};

// Structure to represent a ROM record
// The ROM record will contain the name of the ROM, the mapper code, the size of the ROM and the offset in the flash memory
// Name: MAX_FILE_NAME_LENGTH bytes
// Mapper: 1 byte
// Size: 4 bytes
// Offset: 4 bytes
typedef struct {
    char Name[MAX_FILE_NAME_LENGTH + 1];
    unsigned char Mapper;
    unsigned long Size;
    unsigned long Offset;
} ROMRecord;


// Control Variables
extern int currentPage;    // Current page
extern int totalPages;     // Total pages
extern int currentIndex;   // Current file index
extern unsigned int totalFiles;     // Total files
extern unsigned long totalSize;
extern ROMRecord records[FILES_PER_PAGE]; // Page buffer for ROM records

// Declare the functions
unsigned long read_ulong(const unsigned char *ptr);
int isEndOfData(const unsigned char *memory);
void readROMData(ROMRecord *records, unsigned int *recordCount, unsigned long *sizeTotal);
int putchar (int character);
void invert_chars(unsigned char startChar, unsigned char endChar);
void print_str_normal(const char *str);
void print_str_inverted(const char *str);
void print_str_inverted_padded(const char *str, unsigned char width);
char* mapper_description(int number);
int record_is_folder(const ROMRecord *record);
int record_is_mp3(const ROMRecord *record);
void trim_name_to_buffer(const char *src, char *dst, int max_len);
void charMap(); //debug
void displayMenu();
void navigateMenu();
void configMenu();
void helpMenu();
void loadGame(int index);
void main();
void delay_ms(uint16_t milliseconds);



