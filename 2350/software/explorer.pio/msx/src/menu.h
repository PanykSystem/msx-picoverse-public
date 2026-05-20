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
#define MEMORY_START 0xB900 // Start of the memory area to read the ROM records
#define ROM_SELECT_REGISTER 0xBF7F // Memory-mapped register that selects the ROM to load
#define JIFFY 0xFC9E
#define SOURCE_SD_FLAG 0x80
#define FOLDER_FLAG 0x40
#define OVERRIDE_FLAG 0x10
#define NAME_COL_WIDTH 22
#define NAME_COL_WIDTH_80 60
#define MENU_FORCE_40_COLUMNS 0
#define MENU_KEY_F4_CONFIG 0x04
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
#define CTRL_AUDIO   (CTRL_BASE_ADDR + 9)
#define CTRL_WIFI_SUPPORT (CTRL_BASE_ADDR + 10)
#define CTRL_MAGIC   0xA5
#define CTRL_QUERY_BASE 0xBFC0
#define CTRL_QUERY_SIZE 32
#define AUDIO_PROFILE_NONE 0
#define AUDIO_PROFILE_SCC 1
#define AUDIO_PROFILE_SCC_PLUS 2
#define AUDIO_PROFILE_DUAL_PSG 3
#define AUDIO_PROFILE_MSX_MUSIC 4
#define DATA_BUFFER_END CTRL_FH_STATUS_TEXT_BASE
#define DATA_BUFFER_SIZE (DATA_BUFFER_END - MEMORY_START)
#define CMD_APPLY_FILTER 0x01
#define CMD_FIND_FIRST  0x02
#define CMD_ENTER_DIR   0x03
#define CMD_DETECT_MAPPER 0x04
#define CMD_SET_MAPPER 0x05
#define CMD_SET_SOURCE  0x06
#define CMD_FH_LIST_PAGE 0x40
#define CMD_FH_DOWNLOAD  0x41
#define CMD_FH_SEARCH    0x42
#define CMD_FH_WIFI_STATUS 0x43
#define CMD_FH_WIFI_CONFIG 0x44
#define CTRL_FH_PROGRESS_L CTRL_MAPPER
#define CTRL_FH_PROGRESS_H CTRL_ACK
#define CTRL_FH_RESULT CTRL_AUDIO
#define CTRL_FH_STATUS_TEXT_BASE 0xBF80
#define CTRL_FH_STATUS_TEXT_SIZE 64
#define CTRL_FH_RECORD_SIZE 64

#define SOURCE_MODE_ALL   0x00
#define SOURCE_MODE_FLASH 0x01
#define SOURCE_MODE_SD    0x02
#define SOURCE_MODE_FILEHUNTER 0x03

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

#define MENU_ROW_FORMAT_80 " %-61.61s %6s %-2s %-5s"
#define MENU_ROW_FORMAT_40 " %-21.21s %6s %-2s %-5s"

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
char* mapper_description(int number);
unsigned char record_mapper_code(unsigned char mapper);
int record_mapper_is_override(unsigned char mapper);
int record_is_folder(const ROMRecord *record);
void trim_name_to_buffer(const char *src, char *dst, int max_len);
int build_sliding_name_window(const char *str, int startPos, char *out, unsigned char width);
void displayMenu();
void navigateMenu();
void helpMenu();
void loadGame(int index);
void launch_wifi_config(void);
void execute_rst00();
void main();
void delay_ms(uint16_t milliseconds);



