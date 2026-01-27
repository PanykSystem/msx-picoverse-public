// MSX PICOVERSE PROJECT
// (c) 2025 Cristiano Goncalves
// The Retro Hacker
//
// explorer.c - PicoVerse 2350 Explorer firmware (ROM loader + SD support)
//
// This work is licensed  under a "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
// License". https://creativecommons.org/licenses/by-nc-sa/4.0/

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "ff.h"
#include "diskio.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/structs/qmi.h"
#include "hw_config.h"
#include "explorer.h"
#include "nextor.h"

// config area and buffer for the ROM data
#define ROM_NAME_MAX    60          // Maximum size of the ROM name
#define MAX_ROM_RECORDS 1024        // Maximum ROM files supported (flash + SD)
#define MAX_FLASH_RECORDS 128       // Maximum ROM files stored in flash
#define ROM_RECORD_SIZE (ROM_NAME_MAX + 1 + (sizeof(uint32_t) * 2)) // Name + mapper + size + offset
#define MONITOR_ADDR    (0xBB01)    // Monitor ROM address within image
#define CACHE_SIZE      262144     // 256KB cache size for ROM data
#define SOURCE_SD_FLAG  0x80
#define FILES_PER_PAGE  19
#define CTRL_BASE_ADDR  0xBFF0
#define CTRL_COUNT_L    (CTRL_BASE_ADDR + 0)
#define CTRL_COUNT_H    (CTRL_BASE_ADDR + 1)
#define CTRL_PAGE       (CTRL_BASE_ADDR + 2)
#define CTRL_STATUS     (CTRL_BASE_ADDR + 3)
#define CTRL_CMD        (CTRL_BASE_ADDR + 4)
#define CTRL_MATCH_L    (CTRL_BASE_ADDR + 5)
#define CTRL_MATCH_H    (CTRL_BASE_ADDR + 6)
#define CTRL_MAGIC      0xA5
#define CTRL_QUERY_BASE 0xBFC0
#define CTRL_QUERY_SIZE 32
#define CMD_APPLY_FILTER 0x01
#define CMD_FIND_FIRST   0x02

// This symbol marks the end of the main program in flash.
// Custom data starts right after it
extern const uint8_t __flash_binary_end[];

// SRAM buffer to cache ROM data
static uint8_t rom_sram[CACHE_SIZE];
static uint32_t active_rom_size = 0;

// pointer to the custom data
static const uint8_t *flash_rom = (const uint8_t *)&__flash_binary_end;
static const uint8_t *rom_data = (const uint8_t *)&__flash_binary_end;
static bool rom_data_in_ram = false;

uint8_t ctr_val = 0xFF;    
BYTE const pdrv = 0;  // Physical drive number
DSTATUS ds = 1; // Disk status (1 = not initialized)

// Structure to represent a ROM record
// The ROM record will contain the name of the ROM, the mapper code, the size of the ROM and the offset in the flash memory
// Name: ROM_NAME_MAX bytes
// Mapper: 1 byte
// Size: 4 bytes
// Offset: 4 bytes
typedef struct {
    char Name[ROM_NAME_MAX];
    unsigned char Mapper;
    unsigned long Size;
    unsigned long Offset;
} ROMRecord;

ROMRecord records[MAX_ROM_RECORDS]; // Array to store the ROM records

#define SD_PATH_MAX        96
#define SD_PATH_BUFFER_SIZE 32768
#define MIN_ROM_SIZE       8192
#define MAX_ROM_SIZE       (15u * 1024u * 1024u)
#define MAX_ANALYSIS_SIZE  20480
#define MAPPER_SYSTEM      10

static const char *MAPPER_DESCRIPTIONS[] = {
    "PL-16", "PL-32", "KonSCC", "Linear", "ASC-08",
    "ASC-16", "Konami", "NEO-8", "NEO-16", "SYSTEM"
};

#define MAPPER_DESCRIPTION_COUNT (sizeof(MAPPER_DESCRIPTIONS) / sizeof(MAPPER_DESCRIPTIONS[0]))

static char sd_path_buffer[SD_PATH_BUFFER_SIZE];
static uint16_t sd_path_offsets[MAX_ROM_RECORDS];
static uint16_t sd_path_buffer_used = 0;
static uint16_t sd_record_count = 0;
static bool records_from_sd = false;
static bool sd_mounted = false;
static sd_card_t *sd_card = NULL;
static uint8_t rom_analysis_buffer[MAX_ANALYSIS_SIZE];
static uint16_t total_record_count = 0;
static uint16_t full_record_count = 0;
static uint8_t current_page = 0;
static uint16_t filtered_indices[MAX_ROM_RECORDS];
static char filter_query[CTRL_QUERY_SIZE];
static uint8_t ctrl_cmd_state = 0;
static uint16_t match_index = 0xFFFF;

static void write_u32_le(uint8_t *ptr, uint32_t value);
static void build_page_buffer(uint8_t page_index);

static int compare_record_names(const ROMRecord *a, const ROMRecord *b) {
    return strncmp(a->Name, b->Name, ROM_NAME_MAX);
}

static bool is_system_record(const ROMRecord *record) {
    return ((record->Mapper & ~SOURCE_SD_FLAG) == MAPPER_SYSTEM);
}

static size_t trim_name_copy(char *dest, const char *src) {
    size_t len = 0;
    for (size_t i = 0; i < ROM_NAME_MAX; i++) {
        dest[i] = src[i];
    }
    dest[ROM_NAME_MAX] = '\0';
    len = ROM_NAME_MAX;
    while (len > 0 && dest[len - 1] == ' ') {
        dest[len - 1] = '\0';
        len--;
    }
    return len;
}

static bool contains_ignore_case(const char *text, const char *query) {
    if (!query || query[0] == '\0') {
        return true;
    }
    size_t text_len = strlen(text);
    size_t query_len = strlen(query);
    if (query_len == 0 || query_len > text_len) {
        return false;
    }
    for (size_t i = 0; i + query_len <= text_len; i++) {
        size_t j = 0;
        while (j < query_len) {
            char a = text[i + j];
            char b = query[j];
            if (toupper((unsigned char)a) != toupper((unsigned char)b)) {
                break;
            }
            j++;
        }
        if (j == query_len) {
            return true;
        }
    }
    return false;
}

static void apply_filter(void) {
    total_record_count = 0;
    if (filter_query[0] == '\0') {
        for (uint16_t i = 0; i < full_record_count; i++) {
            filtered_indices[total_record_count++] = i;
        }
        return;
    }

    char name_buf[ROM_NAME_MAX + 1];
    for (uint16_t i = 0; i < full_record_count; i++) {
        trim_name_copy(name_buf, records[i].Name);
        if (contains_ignore_case(name_buf, filter_query)) {
            filtered_indices[total_record_count++] = i;
        }
    }
}

static void find_first_match(void) {
    match_index = 0xFFFF;
    if (filter_query[0] == '\0') {
        return;
    }

    char name_buf[ROM_NAME_MAX + 1];
    for (uint16_t i = 0; i < full_record_count; i++) {
        trim_name_copy(name_buf, records[i].Name);
        if (contains_ignore_case(name_buf, filter_query)) {
            match_index = i;
            return;
        }
    }
}

static void sort_records(ROMRecord *list, uint16_t count) {
    uint16_t system_count = 0;
    for (uint16_t i = 0; i < count; i++) {
        if (is_system_record(&list[i])) {
            if (i != system_count) {
                ROMRecord temp = list[i];
                memmove(&list[system_count + 1], &list[system_count], (i - system_count) * sizeof(ROMRecord));
                list[system_count] = temp;
            }
            system_count++;
        }
    }

    for (uint16_t i = system_count; i + 1 < count; i++) {
        for (uint16_t j = i + 1; j < count; j++) {
            if (compare_record_names(&list[i], &list[j]) > 0) {
                ROMRecord temp = list[i];
                list[i] = list[j];
                list[j] = temp;
            }
        }
    }
}

static void write_config_area(uint8_t *config_area, const ROMRecord *list, uint16_t count) {
    for (uint16_t i = 0; i < count; i++) {
        uint8_t *entry = config_area + (i * ROM_RECORD_SIZE);
        memset(entry, 0xFF, ROM_RECORD_SIZE);
        memset(entry, ' ', ROM_NAME_MAX);
        size_t name_len = strlen(list[i].Name);
        if (name_len > ROM_NAME_MAX) {
            name_len = ROM_NAME_MAX;
        }
        memcpy(entry, list[i].Name, name_len);
        entry[ROM_NAME_MAX] = list[i].Mapper;
        write_u32_le(entry + ROM_NAME_MAX + 1, (uint32_t)list[i].Size);
        write_u32_le(entry + ROM_NAME_MAX + 1 + sizeof(uint32_t), (uint32_t)list[i].Offset);
    }

    if (count < MAX_ROM_RECORDS) {
        memset(config_area + (count * ROM_RECORD_SIZE), 0xFF,
               (MAX_ROM_RECORDS - count) * ROM_RECORD_SIZE);
    }
}

static void build_page_buffer(uint8_t page_index) {
    uint8_t *config_area = rom_sram + 0x4000;
    uint16_t start_index = (uint16_t)page_index * FILES_PER_PAGE;

    for (uint16_t i = 0; i < FILES_PER_PAGE; i++) {
        uint16_t filtered_index = (uint16_t)(start_index + i);
        uint8_t *entry = config_area + (i * ROM_RECORD_SIZE);
        if (filtered_index >= total_record_count) {
            memset(entry, 0xFF, ROM_RECORD_SIZE);
            continue;
        }

        uint16_t record_index = filtered_indices[filtered_index];

        memset(entry, 0xFF, ROM_RECORD_SIZE);
        memset(entry, ' ', ROM_NAME_MAX);
        size_t name_len = strlen(records[record_index].Name);
        if (name_len > ROM_NAME_MAX) {
            name_len = ROM_NAME_MAX;
        }
        memcpy(entry, records[record_index].Name, name_len);
        entry[ROM_NAME_MAX] = records[record_index].Mapper;
        write_u32_le(entry + ROM_NAME_MAX + 1, (uint32_t)records[record_index].Size);
        write_u32_le(entry + ROM_NAME_MAX + 1 + sizeof(uint32_t), (uint32_t)records[record_index].Offset);
    }
}

static bool equals_ignore_case(const char *a, const char *b) {
    if (!a || !b) {
        return false;
    }
    while (*a && *b) {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

static void write_u32_le(uint8_t *ptr, uint32_t value) {
    ptr[0] = (uint8_t)(value & 0xFFu);
    ptr[1] = (uint8_t)((value >> 8) & 0xFFu);
    ptr[2] = (uint8_t)((value >> 16) & 0xFFu);
    ptr[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static uint8_t mapper_number_from_filename(const char *filename) {
    char name_copy[SD_PATH_MAX];
    strncpy(name_copy, filename, sizeof(name_copy));
    name_copy[sizeof(name_copy) - 1] = '\0';

    char *dot = strrchr(name_copy, '.');
    if (dot) {
        *dot = '\0';
    }

    size_t name_len = strlen(name_copy);
    for (size_t i = 0; i + 1 < MAPPER_DESCRIPTION_COUNT; ++i) {
        const char *tag = MAPPER_DESCRIPTIONS[i];
        size_t tag_len = strlen(tag);
        if (name_len > tag_len + 1 && name_copy[name_len - tag_len - 1] == '.') {
            if (equals_ignore_case(name_copy + name_len - tag_len, tag)) {
                return (uint8_t)(i + 1);
            }
        }
    }
    return 0;
}

static void build_display_name(const char *filename, char *out, size_t out_size) {
    char name_copy[SD_PATH_MAX];
    strncpy(name_copy, filename, sizeof(name_copy));
    name_copy[sizeof(name_copy) - 1] = '\0';

    char *dot = strrchr(name_copy, '.');
    if (dot) {
        *dot = '\0';
    }

    size_t name_len = strlen(name_copy);
    for (size_t i = 0; i + 1 < MAPPER_DESCRIPTION_COUNT; ++i) {
        const char *tag = MAPPER_DESCRIPTIONS[i];
        size_t tag_len = strlen(tag);
        if (name_len > tag_len + 1 && name_copy[name_len - tag_len - 1] == '.') {
            if (equals_ignore_case(name_copy + name_len - tag_len, tag)) {
                name_copy[name_len - tag_len - 1] = '\0';
                break;
            }
        }
    }

    strncpy(out, name_copy, out_size);
    out[out_size - 1] = '\0';
}

static bool has_rom_extension(const char *filename) {
    const char *dot = strrchr(filename, '.');
    if (!dot || dot[1] == '\0') {
        return false;
    }
    return equals_ignore_case(dot + 1, "ROM");
}

static uint8_t detect_rom_type_from_buffer(const uint8_t *rom, size_t size, size_t read_size) {
    const char neo8_signature[] = "ROM_NEO8";
    const char neo16_signature[] = "ROM_NE16";

    int konami_score = 0;
    int konami_scc_score = 0;
    int ascii8_score = 0;
    int ascii16_score = 0;

    const int KONAMI_WEIGHT = 2;
    const int KONAMI_SCC_WEIGHT = 2;
    const int ASCII8_WEIGHT_HIGH = 3;
    const int ASCII8_WEIGHT_LOW = 1;
    const int ASCII16_WEIGHT = 2;

    if (size > MAX_ROM_SIZE || size < MIN_ROM_SIZE || read_size < 2) {
        return 0;
    }

    if (rom[0] == 'A' && rom[1] == 'B' && size == 16384) {
        return 1; // Plain 16KB
    }

    if (rom[0] == 'A' && rom[1] == 'B' && size <= 32768) {
        if (read_size > 0x4001 && rom[0x4000] == 'A' && rom[0x4001] == 'B') {
            return 4; // Linear0 32KB
        }
        return 2; // Plain 32KB
    }

    if (rom[0] == 'A' && rom[1] == 'B' && read_size > 16) {
        if ((read_size >= 16 + sizeof(neo8_signature) - 1) &&
            memcmp(&rom[16], neo8_signature, sizeof(neo8_signature) - 1) == 0) {
            return 8; // NEO8
        }
        if ((read_size >= 16 + sizeof(neo16_signature) - 1) &&
            memcmp(&rom[16], neo16_signature, sizeof(neo16_signature) - 1) == 0) {
            return 9; // NEO16
        }
    }

    if (read_size > 0x4001 && rom[0x4000] == 'A' && rom[0x4001] == 'B' && size <= 49152) {
        return 4; // Linear0 48KB
    }

    if (size > 32768 && read_size > 3) {
        for (size_t i = 0; i < read_size - 3; i++) {
            if (rom[i] == 0x32) {
                uint16_t addr = rom[i + 1] | (rom[i + 2] << 8);
                switch (addr) {
                    case 0x4000:
                    case 0x8000:
                    case 0xA000:
                        konami_score += KONAMI_WEIGHT;
                        break;
                    case 0x5000:
                    case 0x9000:
                    case 0xB000:
                        konami_scc_score += KONAMI_SCC_WEIGHT;
                        break;
                    case 0x6800:
                    case 0x7800:
                        ascii8_score += ASCII8_WEIGHT_HIGH;
                        break;
                    case 0x77FF:
                        ascii16_score += ASCII16_WEIGHT;
                        break;
                    case 0x6000:
                        konami_score += KONAMI_WEIGHT;
                        konami_scc_score += KONAMI_SCC_WEIGHT;
                        ascii8_score += ASCII8_WEIGHT_LOW;
                        ascii16_score += ASCII16_WEIGHT;
                        break;
                    case 0x7000:
                        konami_scc_score += KONAMI_SCC_WEIGHT;
                        ascii8_score += ASCII8_WEIGHT_LOW;
                        ascii16_score += ASCII16_WEIGHT;
                        break;
                }
            }
        }

        if (ascii8_score == 1) {
            ascii8_score--;
        }

        if (konami_scc_score > konami_score && konami_scc_score > ascii8_score && konami_scc_score > ascii16_score) {
            return 3; // Konami SCC
        }
        if (konami_score > konami_scc_score && konami_score > ascii8_score && konami_score > ascii16_score) {
            return 7; // Konami
        }
        if (ascii8_score > konami_score && ascii8_score > konami_scc_score && ascii8_score > ascii16_score) {
            return 5; // ASCII8
        }
        if (ascii16_score > konami_score && ascii16_score > konami_scc_score && ascii16_score > ascii8_score) {
            return 6; // ASCII16
        }
        if (ascii16_score == konami_scc_score) {
            return 6; // ASCII16
        }
    }

    return 0;
}

static uint8_t detect_rom_type_from_file(const char *path, uint32_t size) {
    if (size > MAX_ROM_SIZE || size < MIN_ROM_SIZE) {
        return 0;
    }

    FIL fil;
    FRESULT fr = f_open(&fil, path, FA_READ);
    if (fr != FR_OK) {
        return 0;
    }

    UINT br = 0;
    size_t read_size = (size > MAX_ANALYSIS_SIZE) ? MAX_ANALYSIS_SIZE : size;
    fr = f_read(&fil, rom_analysis_buffer, (UINT)read_size, &br);
    f_close(&fil);
    if (fr != FR_OK || br < read_size) {
        return 0;
    }

    return detect_rom_type_from_buffer(rom_analysis_buffer, size, read_size);
}

static bool sd_mount_card(void) {
    if (sd_mounted) {
        return true;
    }

    if (!sd_init_driver()) {
        return false;
    }

    sd_card = sd_get_by_num(0);
    if (!sd_card) {
        return false;
    }

    FRESULT fr = f_mount(&sd_card->state.fatfs, "", 1);
    if (fr != FR_OK) {
        return false;
    }

    sd_mounted = true;
    return true;
}

static uint16_t populate_records_from_sd(uint8_t *config_area, uint16_t start_index) {
    if (!sd_mount_card()) {
        return 0;
    }

    DIR dir;
    FILINFO fno;
    FRESULT fr = f_opendir(&dir, "/");
    if (fr != FR_OK) {
        return false;
    }

    sd_record_count = 0;
    uint16_t record_index = start_index;
    while (record_index < MAX_ROM_RECORDS) {
        fr = f_readdir(&dir, &fno);
        if (fr != FR_OK || fno.fname[0] == '\0') {
            break;
        }
        if (fno.fattrib & AM_DIR) {
            continue;
        }
        if (!has_rom_extension(fno.fname)) {
            continue;
        }
        if (fno.fsize < MIN_ROM_SIZE || fno.fsize > MAX_ROM_SIZE) {
            continue;
        }
        if (fno.fsize > CACHE_SIZE) {
            continue;
        }

        char path[SD_PATH_MAX];
        snprintf(path, sizeof(path), "/%s", fno.fname);

        uint8_t mapper = mapper_number_from_filename(fno.fname);
        if (mapper == 0) {
            mapper = detect_rom_type_from_file(path, (uint32_t)fno.fsize);
        }
        if (mapper == 0) {
            continue;
        }

        size_t path_len = strlen(path);
        if (sd_path_buffer_used + path_len + 1 > SD_PATH_BUFFER_SIZE) {
            break;
        }
        sd_path_offsets[record_index] = sd_path_buffer_used;
        memcpy(sd_path_buffer + sd_path_buffer_used, path, path_len + 1);
        sd_path_buffer_used = (uint16_t)(sd_path_buffer_used + path_len + 1);

        char display_name[ROM_NAME_MAX + 1];
        build_display_name(fno.fname, display_name, sizeof(display_name));

        memset(records[record_index].Name, 0, sizeof(records[record_index].Name));
        strncpy(records[record_index].Name, display_name, sizeof(records[record_index].Name) - 1);
        records[record_index].Mapper = (unsigned char)(mapper | SOURCE_SD_FLAG);
        records[record_index].Size = (unsigned long)fno.fsize;
        records[record_index].Offset = (unsigned long)record_index;

        if (config_area) {
            uint8_t *entry = config_area + (record_index * ROM_RECORD_SIZE);
            memset(entry, 0xFF, ROM_RECORD_SIZE);
            memset(entry, ' ', ROM_NAME_MAX);
            size_t name_len = strlen(display_name);
            if (name_len > ROM_NAME_MAX) {
                name_len = ROM_NAME_MAX;
            }
            memcpy(entry, display_name, name_len);
            entry[ROM_NAME_MAX] = (uint8_t)(mapper | SOURCE_SD_FLAG);
            write_u32_le(entry + ROM_NAME_MAX + 1, (uint32_t)fno.fsize);
            write_u32_le(entry + ROM_NAME_MAX + 1 + sizeof(uint32_t), (uint32_t)record_index);
        }

        sd_record_count++;
        record_index++;
    }

    f_closedir(&dir);

    if (config_area && record_index < MAX_ROM_RECORDS) {
        memset(config_area + (record_index * ROM_RECORD_SIZE), 0xFF,
               (MAX_ROM_RECORDS - record_index) * ROM_RECORD_SIZE);
    }

    return sd_record_count;
}

static bool load_rom_from_sd(uint16_t record_index, uint32_t size) {
    if (!sd_mount_card()) {
        return false;
    }
    if (record_index >= MAX_ROM_RECORDS || size > CACHE_SIZE) {
        return false;
    }
    if (sd_path_offsets[record_index] == 0xFFFF) {
        return false;
    }

    const char *path = sd_path_buffer + sd_path_offsets[record_index];

    FIL fil;
    FRESULT fr = f_open(&fil, path, FA_READ);
    if (fr != FR_OK) {
        return false;
    }

    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 0);

    memset(rom_sram, 0, size);

    UINT total = 0;
    while (total < size) {
        UINT to_read = (UINT)((size - total) > 4096 ? 4096 : (size - total));
        UINT br = 0;
        fr = f_read(&fil, rom_sram + total, to_read, &br);
        if (fr != FR_OK || br == 0) {
            break;
        }
        total += br;
    }

    f_close(&fil);
    gpio_put(PIN_WAIT, 1);

    return total == size;
}

// Initialize GPIO pins
static inline void setup_gpio()
{

    for (int i = 0; i <= 23; i++) {
        gpio_init(i);
        gpio_set_input_hysteresis_enabled(i, true);
    }

    for (int i = 0; i <= 15; i++) {
        gpio_set_dir(i, GPIO_IN);
    }

    // Initialize control pins as input
    gpio_init(PIN_RD); gpio_set_dir(PIN_RD, GPIO_IN);    
    gpio_init(PIN_WR); gpio_set_dir(PIN_WR, GPIO_IN); 
    gpio_init(PIN_IORQ); gpio_set_dir(PIN_IORQ, GPIO_IN); 
    gpio_init(PIN_SLTSL); gpio_set_dir(PIN_SLTSL, GPIO_IN); 
    gpio_init(PIN_BUSSDIR); gpio_set_dir(PIN_BUSSDIR, GPIO_IN); 
}

// read_ulong - Read a 4-byte value from the memory area
// This function will read a 4-byte value from the memory area pointed by ptr and return the value as an unsigned long
// Parameters:
//   ptr - Pointer to the memory area to read the value from
// Returns:
//   The 4-byte value as an unsigned long 
unsigned long __no_inline_not_in_flash_func(read_ulong)(const unsigned char *ptr) {
    return (unsigned long)ptr[0] |
           ((unsigned long)ptr[1] << 8) |
           ((unsigned long)ptr[2] << 16) |
           ((unsigned long)ptr[3] << 24);
}

// isEndOfData - Check if the memory area is the end of the data
// This function will check if the memory area pointed by memory is the end of the data. The end of the data is defined by all bytes being 0xFF.
// Parameters:
//   memory - Pointer to the memory area to check
// Returns:
//   1 if the memory area is the end of the data, 0 otherwise
int __no_inline_not_in_flash_func(isEndOfData)(const unsigned char *memory) {
    for (int i = 0; i < ROM_RECORD_SIZE; i++) {
        if (memory[i] != 0xFF) {
            return 0;
        }
    }
    return 1;
}

//load the MSX Menu ROM into the MSX
int __no_inline_not_in_flash_func(loadrom_msx_menu)(uint32_t offset)
{
    //setup the rom_sram buffer for the 32KB ROM
    gpio_init(PIN_WAIT); // Init wait signal pin
    gpio_set_dir(PIN_WAIT, GPIO_OUT); // Set the WAIT signal as output
    gpio_put(PIN_WAIT, 0); // Wait until we are ready to read the ROM
    memset(rom_sram, 0, 32768); // Clear the SRAM buffer
    memcpy(rom_sram, flash_rom + offset, 32768); //for 32KB ROMs we start at 0x4000
    gpio_put(PIN_WAIT, 1); // Lets go!

    int record_count = 0; // Record count
    const uint8_t *record_ptr = flash_rom + offset + 0x4000; // Pointer to the ROM records
    for (int i = 0; i < MAX_FLASH_RECORDS; i++)      // Read the ROMs from the configuration area
    {
        if (isEndOfData(record_ptr)) {
            break; // Stop if end of data is reached
        }
        memcpy(records[record_count].Name, record_ptr, ROM_NAME_MAX); // Copy the ROM name
        record_ptr += ROM_NAME_MAX; // Move the pointer to the next field
        records[record_count].Mapper = *record_ptr++; // Read the mapper code
        records[record_count].Size = read_ulong(record_ptr); // Read the ROM size
        record_ptr += sizeof(unsigned long); // Move the pointer to the next field
        records[record_count].Offset = read_ulong(record_ptr); // Read the ROM offset
        record_ptr += sizeof(unsigned long); // Move the pointer to the next record
        record_count++; // Increment the record count
    }

    for (int i = 0; i < MAX_ROM_RECORDS; i++) {
        sd_path_offsets[i] = 0xFFFF;
    }
    sd_path_buffer_used = 0;

    uint16_t sd_added = populate_records_from_sd(NULL, (uint16_t)record_count);
    records_from_sd = (sd_added > 0);

    uint16_t total_count = (uint16_t)(record_count + sd_added);
    sort_records(records, total_count);
    full_record_count = total_count;
    memset(filter_query, 0, sizeof(filter_query));
    apply_filter();
    current_page = 0;
    build_page_buffer(current_page);

    uint8_t rom_index = 0;
    gpio_set_dir_in_masked(0xFF << 16); // Set data bus to input mode
    bool rom_selected = false; // ROM selected flag
    while (true)  // Loop until a ROM is selected
    {
        // Check control signals
        bool sltsl = !(gpio_get(PIN_SLTSL)); // Slot selected (active low)
        bool rd = !(gpio_get(PIN_RD));       // Read cycle (active low)

        uint16_t addr = gpio_get_all() & 0x00FFFF; // Read the address bus
        //uint16_t addr = sio_hw->gpio_in & 0x00FFFF;
        if (sltsl) 
        {
            bool wr = !(gpio_get(PIN_WR));       // Write cycle (active low, not used)

            if (wr && addr >= CTRL_QUERY_BASE && addr < (CTRL_QUERY_BASE + CTRL_QUERY_SIZE))
            {
                    uint8_t value = (gpio_get_all() >> 16) & 0xFF;
                    filter_query[addr - CTRL_QUERY_BASE] = (char)value;
                    while (!(gpio_get(PIN_WR))) {
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16);
            }

            if (wr && addr == CTRL_CMD)
            {
                    uint8_t cmd = (gpio_get_all() >> 16) & 0xFF;
                    ctrl_cmd_state = cmd;
                    if (cmd == CMD_APPLY_FILTER)
                    {
                        filter_query[CTRL_QUERY_SIZE - 1] = '\0';
                        apply_filter();
                        current_page = 0;
                        build_page_buffer(current_page);
                    }
                    else if (cmd == CMD_FIND_FIRST)
                    {
                        filter_query[CTRL_QUERY_SIZE - 1] = '\0';
                        find_first_match();
                    }
                    ctrl_cmd_state = 0;
                    while (!(gpio_get(PIN_WR))) {
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16);
            }

            if (wr && addr == CTRL_PAGE)
            {
                    uint8_t page = (gpio_get_all() >> 16) & 0xFF;
                    if (page != current_page)
                    {
                        current_page = page;
                        build_page_buffer(current_page);
                    }
                    while (!(gpio_get(PIN_WR))) { // Wait until the write cycle completes (WR goes high)
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16); // Set data bus to input mode
            }

            if (wr && addr == MONITOR_ADDR) // Monitor ROM address to select the ROM
            {   
                    uint8_t selected_index = (gpio_get_all() >> 16) & 0xFF;
                    if (selected_index < total_record_count) {
                        rom_index = (uint8_t)filtered_indices[selected_index];
                    } else {
                        rom_index = 0;
                    }
                    while (!(gpio_get(PIN_WR))) { // Wait until the write cycle completes (WR goes high){
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16); // Set data bus to input mode
                    rom_selected = true;    // ROM selected
            }

            if (addr >= 0x4000 && addr <= 0xBFFF) // Check if the address is within the ROM range
            {   
                if (rd)
                {
                    if (addr >= CTRL_BASE_ADDR && addr <= (CTRL_BASE_ADDR + 0x0F))
                    {
                        uint8_t ctrl_value = 0xFF;
                        switch (addr)
                        {
                            case CTRL_COUNT_L:
                                ctrl_value = (uint8_t)(total_record_count & 0xFFu);
                                break;
                            case CTRL_COUNT_H:
                                ctrl_value = (uint8_t)((total_record_count >> 8) & 0xFFu);
                                break;
                            case CTRL_PAGE:
                                ctrl_value = current_page;
                                break;
                            case CTRL_STATUS:
                                ctrl_value = CTRL_MAGIC;
                                break;
                            case CTRL_CMD:
                                ctrl_value = ctrl_cmd_state;
                                break;
                            case CTRL_MATCH_L:
                                ctrl_value = (uint8_t)(match_index & 0xFFu);
                                break;
                            case CTRL_MATCH_H:
                                ctrl_value = (uint8_t)((match_index >> 8) & 0xFFu);
                                break;
                        }
                        gpio_set_dir_out_masked(0xFF << 16); // Set data bus to output mode
                        gpio_put_masked(0xFF0000, (uint32_t)ctrl_value << 16);
                        while (!(gpio_get(PIN_RD))) {
                            tight_loop_contents();
                        }
                        gpio_set_dir_in_masked(0xFF << 16);
                        continue;
                    }

                    gpio_set_dir_out_masked(0xFF << 16); // Set data bus to output mode
                    uint32_t rom_addr = offset + (addr - 0x4000); // Calculate flash address
                    gpio_put_masked(0xFF0000, rom_sram[rom_addr] << 16); // Write the data to the data bus
                    while (!(gpio_get(PIN_RD))) { // Wait until the read cycle completes (RD goes high)
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16); // Set data bus to input mode
                }
            } 
        }

        if (rd && addr == 0x0000 && rom_selected)   // lets return the rom_index and load the selected ROM
        {
            return rom_index;
        }
    }
}

// loadrom_plain32 - Load a simple 32KB (or less) ROM into the MSX directly from the pico flash
// 32KB ROMS have two pages of 16Kb each in the following areas:
// 0x4000-0x7FFF and 0x8000-0xBFFF
// AB is on 0x0000, 0x0001
// 16KB ROMS have only one page in the 0x4000-0x7FFF area
// AB is on 0x0000, 0x0001
void __no_inline_not_in_flash_func(loadrom_plain32)(uint32_t offset, bool cache_enable)
{
    const uint8_t *rom_base = rom_data + offset;

     if (cache_enable)
    {
        gpio_init(PIN_WAIT);
        gpio_set_dir(PIN_WAIT, GPIO_OUT);
        gpio_put(PIN_WAIT, 0);
        memset(rom_sram, 0, 32768);
        memcpy(rom_sram, rom_base, 32768);
        gpio_put(PIN_WAIT, 1);
        rom_base = rom_sram;
    }

    gpio_set_dir_in_masked(0xFF << 16);
    while (true)
    {
        if (!gpio_get(PIN_SLTSL))
        {
            uint16_t addr = gpio_get_all() & 0x00FFFF;
            if ((addr >= 0x4000) && (addr <= 0xBFFF) && !gpio_get(PIN_RD))
            {
                uint8_t data = rom_base[addr - 0x4000];
                gpio_set_dir_out_masked(0xFF << 16);
                gpio_put_masked(0xFF0000, (uint32_t)data << 16);
                while (!gpio_get(PIN_RD))
                    tight_loop_contents();
                gpio_set_dir_in_masked(0xFF << 16);
            }
        }
    }
}

// loadrom_linear48 - Load a simple 48KB Linear0 ROM into the MSX directly from the pico flash
// Those ROMs have three pages of 16Kb each in the following areas:
// 0x0000-0x3FFF, 0x4000-0x7FFF and 0x8000-0xBFFF
// AB is on 0x4000, 0x4001
void __no_inline_not_in_flash_func(loadrom_linear48)(uint32_t offset, bool cache_enable)
{
    const uint8_t *rom_base = rom_data + offset;

    if (cache_enable)
    {
        gpio_init(PIN_WAIT);
        gpio_set_dir(PIN_WAIT, GPIO_OUT);
        gpio_put(PIN_WAIT, 0);
        memset(rom_sram, 0, 49152);
        memcpy(rom_sram, rom_base, 49152);
        gpio_put(PIN_WAIT, 1);
        rom_base = rom_sram;
    }
    
    gpio_set_dir_in_masked(0xFF << 16); // Set data bus to input mode
    while (true) 
    {
        if (!gpio_get(PIN_SLTSL))
        {
            uint16_t addr = gpio_get_all() & 0x00FFFF; // Read the address bus
            if ((addr >= 0x0000) && (addr <= 0xBFFF) && !gpio_get(PIN_RD)) // Check if the address is within the ROM range
            {
                uint8_t data = rom_base[addr];
                gpio_set_dir_out_masked(0xFF << 16);
                gpio_put_masked(0xFF0000, (uint32_t)data << 16);
                while (!gpio_get(PIN_RD))
                    tight_loop_contents();
                gpio_set_dir_in_masked(0xFF << 16);
            }
        }
    }
}

// loadrom_konamiscc - Load a any Konami SCC ROM into the MSX directly from the pico flash
// The KonamiSCC ROMs are divided into 8KB segments, managed by a memory mapper that allows dynamic switching of these segments 
// into the MSX's address space. Since the size of the mapper is 8Kb, the memory banks are:
// Bank 1: 4000h - 5FFFh , Bank 2: 6000h - 7FFFh, Bank 3: 8000h - 9FFFh, Bank 4: A000h - BFFFh
// And the address to change banks are:
// Bank 1: 5000h - 57FFh (5000h used), Bank 2: 7000h - 77FFh (7000h used), Bank 3: 9000h - 97FFh (9000h used), Bank 4: B000h - B7FFh (B000h used)
// AB is on 0x0000, 0x0001
void __no_inline_not_in_flash_func(loadrom_konamiscc)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3}; // Initial banks 0-3 mapped
    uint32_t cached_length = 0;

    if (cache_enable)
    {
        gpio_init(PIN_WAIT);
        gpio_set_dir(PIN_WAIT, GPIO_OUT);
        gpio_put(PIN_WAIT, 0);

        uint32_t bytes_to_cache = active_rom_size;
        if (bytes_to_cache == 0 || bytes_to_cache > sizeof(rom_sram))
        {
            bytes_to_cache = sizeof(rom_sram);
        }

        memset(rom_sram, 0, bytes_to_cache);
        memcpy(rom_sram, rom_data + offset, bytes_to_cache);
        gpio_put(PIN_WAIT, 1);
        cached_length = bytes_to_cache;
    }

    gpio_set_dir_in_masked(0xFF << 16); // Set data bus to input mode
    while (true) 
    {
        // Check control signals
        bool sltsl = !(gpio_get(PIN_SLTSL)); // Slot selected (active low)
        bool rd = !(gpio_get(PIN_RD));       // Read cycle (active low)
        bool wr = !(gpio_get(PIN_WR));       // Write cycle (active low)

        if (sltsl)
        {
            uint16_t addr = gpio_get_all() & 0x00FFFF; // Read the address bus
            if (addr >= 0x4000 && addr <= 0xBFFF)  // Check if the address is within the ROM range
            {
                if (rd) 
                {
                    gpio_set_dir_out_masked(0xFF << 16); // Set data bus to output mode
                    uint32_t const rom_offset = offset + (bank_registers[(addr - 0x4000) >> 13] * 0x2000u) + (addr & 0x1FFFu); // Calculate the ROM offset

                    uint8_t data;
                    uint32_t const relative_offset = (rom_offset >= offset) ? (rom_offset - offset) : cached_length;
                    if (cache_enable && relative_offset < cached_length)
                    {
                        data = rom_sram[relative_offset];
                    }
                    else
                    {
                        gpio_put(PIN_WAIT, 0);
                        data = rom_data[rom_offset];
                        gpio_put(PIN_WAIT, 1);
                    }

                    gpio_put_masked(0xFF0000, (uint32_t)data << 16); // Write the data to the data bus
                    while (!(gpio_get(PIN_RD)))  // Wait until the read cycle completes (RD goes high)
                    {
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16); // Return data bus to input mode after cycle completes
                } else if (wr) 
                {
                    // Handle writes to bank switching addresses
                    if ((addr >= 0x5000)  && (addr <= 0x57FF)) { 
                        bank_registers[0] = (gpio_get_all() >> 16) & 0xFF; // Read the data bus and store in bank register
                    } else if ((addr >= 0x7000) && (addr <= 0x77FF)) {
                        bank_registers[1] = (gpio_get_all() >> 16) & 0xFF;
                    } else if ((addr >= 0x9000) && (addr <= 0x97FF)) {
                        bank_registers[2] = (gpio_get_all() >> 16) & 0xFF;
                    } else if ((addr >= 0xB000) && (addr <= 0xB7FF)) {
                        bank_registers[3] = (gpio_get_all() >> 16) & 0xFF;
                    }

                    while (!(gpio_get(PIN_WR)))
                    {
                        tight_loop_contents();
                    }
                }
            }
        }
    }
}

// loadrom_konami - Load a Konami (without SCC) ROM into the MSX directly from the pico flash
// The Konami (without SCC) ROM is divided into 8KB segments, managed by a memory mapper that allows dynamic switching of these segments into the MSX's address space
// Since the size of the mapper is 8Kb, the memory banks are:
//  Bank 1: 4000h - 5FFFh, Bank 2: 6000h - 7FFFh, Bank 3: 8000h - 9FFFh, Bank 4: A000h - BFFFh
// And the addresses to change banks are:
//	Bank 1: <none>, Bank 2: 6000h - 67FFh (6000h used), Bank 3: 8000h - 87FFh (8000h used), Bank 4: A000h - A7FFh (A000h used)
// AB is on 0x0000, 0x0001
void __no_inline_not_in_flash_func(loadrom_konami)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3}; // Initial banks 0-3 mapped
    uint32_t cached_length = 0;

    if (cache_enable)
    {
        gpio_init(PIN_WAIT);
        gpio_set_dir(PIN_WAIT, GPIO_OUT);
        gpio_put(PIN_WAIT, 0);

        uint32_t bytes_to_cache = active_rom_size;
        if (bytes_to_cache == 0 || bytes_to_cache > sizeof(rom_sram))
        {
            bytes_to_cache = sizeof(rom_sram);
        }

        memset(rom_sram, 0, bytes_to_cache);
        memcpy(rom_sram, rom_data + offset, bytes_to_cache);
        gpio_put(PIN_WAIT, 1);
        cached_length = bytes_to_cache;
    }

    gpio_set_dir_in_masked(0xFF << 16);
    while (true) 
    {
        // Check control signals
        bool sltsl = !(gpio_get(PIN_SLTSL)); // Slot selected (active low)
        bool rd = !(gpio_get(PIN_RD));       // Read cycle (active low)
        bool wr = !(gpio_get(PIN_WR));       // Write cycle (active low)

        if (sltsl)
        {
            uint16_t addr = gpio_get_all() & 0x00FFFF; // Read the address bus
            if (addr >= 0x4000 && addr <= 0xBFFF) 
            {
                if (rd) 
                {
                    gpio_set_dir_out_masked(0xFF << 16);
                    uint32_t const rom_offset = offset + (bank_registers[(addr - 0x4000) >> 13] * 0x2000u) + (addr & 0x1FFFu); // Calculate the ROM offset

                    uint8_t data;
                    uint32_t const relative_offset = (rom_offset >= offset) ? (rom_offset - offset) : cached_length;
                    if (cache_enable && relative_offset < cached_length)
                    {
                        data = rom_sram[relative_offset];
                    }
                    else
                    {
                        gpio_put(PIN_WAIT, 0);
                        data = rom_data[rom_offset];
                        gpio_put(PIN_WAIT, 1);
                    }

                    gpio_put_masked(0xFF0000, (uint32_t)data << 16);
                    while (!(gpio_get(PIN_RD))) 
                    {
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16);

                }else if (wr) {
                    // Handle writes to bank switching addresses
                    if ((addr >= 0x6000) && (addr <= 0x67FF)) {
                        bank_registers[1] = (gpio_get_all() >> 16) & 0xFF;
                    } else if ((addr >= 0x8000) && (addr <= 0x87FF)) {
                        bank_registers[2] = (gpio_get_all() >> 16) & 0xFF;
                    } else if ((addr >= 0xA000) && (addr <= 0xA7FF)) {
                        bank_registers[3] = (gpio_get_all() >> 16) & 0xFF;
                    }

                    while (!(gpio_get(PIN_WR))) 
                    {
                        tight_loop_contents();
                    }
                }
            }
        }
    }
}

// loadrom_ascii8 - Load an ASCII8 ROM into the MSX directly from the pico flash
// The ASCII8 ROM is divided into 8KB segments, managed by a memory mapper that allows dynamic switching of these segments into the MSX's address space
// Since the size of the mapper is 8Kb, the memory banks are:
// Bank 1: 4000h - 5FFFh , Bank 2: 6000h - 7FFFh, Bank 3: 8000h - 9FFFh, Bank 4: A000h - BFFFh
// And the address to change banks are:
// Bank 1: 6000h - 67FFh (6000h used), Bank 2: 6800h - 6FFFh (6800h used), Bank 3: 7000h - 77FFh (7000h used), Bank 4: 7800h - 7FFFh (7800h used)
// AB is on 0x0000, 0x0001
void __no_inline_not_in_flash_func(loadrom_ascii8)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3}; // Initial banks 0-3 mapped
    const uint8_t *rom_base = rom_data + offset;
    uint32_t cached_length = 0;

    if (cache_enable)
    {
        gpio_init(PIN_WAIT);
        gpio_set_dir(PIN_WAIT, GPIO_OUT);
        gpio_put(PIN_WAIT, 0);

        uint32_t bytes_to_cache = active_rom_size;
        if (bytes_to_cache == 0 || bytes_to_cache > sizeof(rom_sram))
        {
            bytes_to_cache = sizeof(rom_sram);
        }

        memset(rom_sram, 0, bytes_to_cache);
        memcpy(rom_sram, rom_base, bytes_to_cache);
        gpio_put(PIN_WAIT, 1);
        cached_length = bytes_to_cache;
    }

    gpio_set_dir_in_masked(0xFF << 16);
    while (true) 
    {
        bool sltsl = !(gpio_get(PIN_SLTSL)); // Slot selected (active low)
        bool rd = !(gpio_get(PIN_RD));       // Read cycle (active low)
        bool wr = !(gpio_get(PIN_WR));       // Write cycle (active low)

        if (sltsl) {
            uint16_t addr = gpio_get_all() & 0x00FFFF; // Read the address bus
            if (addr >= 0x4000 && addr <= 0xBFFF) 
            {
                if (rd) 
                {
                    gpio_set_dir_out_masked(0xFF << 16); // Set data bus to output mode
                    uint8_t const bank = bank_registers[(addr - 0x4000) >> 13];
                    uint32_t const rom_offset = offset + (bank * 0x2000u) + (addr & 0x1FFFu); // Calculate the ROM offset

                    uint8_t data;
                    uint32_t const relative_offset = (rom_offset >= offset) ? (rom_offset - offset) : cached_length;
                    if (cache_enable && relative_offset < cached_length)
                    {
                        data = rom_sram[relative_offset];
                    }
                    else
                    {
                        gpio_put(PIN_WAIT, 0);
                        data = rom_data[rom_offset];
                        gpio_put(PIN_WAIT, 1);
                    }

                    gpio_put_masked(0xFF0000, (uint32_t)data << 16); // Write the data to the data bus
                    while (!(gpio_get(PIN_RD)))  { tight_loop_contents(); } // Wait until the read cycle completes (RD goes high)        }
                    gpio_set_dir_in_masked(0xFF << 16); // Return data bus to input mode after the read cycle
                } else if (wr)  // Handle writes to bank switching addresses
                { 
                    if ((addr >= 0x6000) && (addr <= 0x67FF)) { 
                        bank_registers[0] = (gpio_get_all() >> 16) & 0xFF; // Read the data bus and store in bank register
                    } else if ((addr >= 0x6800) && (addr <= 0x6FFF)) {
                        bank_registers[1] = (gpio_get_all() >> 16) & 0xFF;
                    } else if ((addr >= 0x7000) && (addr <= 0x77FF)) {
                        bank_registers[2] = (gpio_get_all() >> 16) & 0xFF;
                    } else if ((addr >= 0x7800) && (addr <= 0x7FFF)) {
                        bank_registers[3] = (gpio_get_all() >> 16) & 0xFF;
                    }

                    while (!(gpio_get(PIN_WR))) 
                    {
                        tight_loop_contents();
                    }
                }
            }
        }
    }
}

// loadrom_ascii16 - Load an ASCII16 ROM into the MSX directly from the pico flash
// The ASCII16 ROM is divided into 16KB segments, managed by a memory mapper that allows dynamic switching of these segments into the MSX's address space
// Since the size of the mapper is 16Kb, the memory banks are:
// Bank 1: 4000h - 7FFFh , Bank 2: 8000h - BFFFh
// And the address to change banks are:
// Bank 1: 6000h - 67FFh (6000h used), Bank 2: 7000h - 77FFh (7000h and 77FFh used)
void __no_inline_not_in_flash_func(loadrom_ascii16)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[2] = {0, 1}; // Initial banks 0 and 1 mapped
    uint32_t cached_length = 0;

    if (cache_enable)
    {
        gpio_init(PIN_WAIT);
        gpio_set_dir(PIN_WAIT, GPIO_OUT);
        gpio_put(PIN_WAIT, 0);

        uint32_t bytes_to_cache = active_rom_size;
        if (bytes_to_cache == 0 || bytes_to_cache > sizeof(rom_sram))
        {
            bytes_to_cache = sizeof(rom_sram);
        }

        memset(rom_sram, 0, bytes_to_cache);
        memcpy(rom_sram, rom_data + offset, bytes_to_cache);
        gpio_put(PIN_WAIT, 1);
        cached_length = bytes_to_cache;
    }

    gpio_set_dir_in_masked(0xFF << 16);
    while (true) {
        // Check control signals
        bool sltsl = !(gpio_get(PIN_SLTSL)); // Slot selected (active low)
        bool rd = !(gpio_get(PIN_RD));       // Read cycle (active low)
        bool wr = !(gpio_get(PIN_WR));       // Write cycle (active low)

        if (sltsl) {
            uint16_t addr = gpio_get_all() & 0x00FFFF; // Read the address bus
            if (addr >= 0x4000 && addr <= 0xBFFF)  
            {
                if (rd) {
                    gpio_set_dir_out_masked(0xFF << 16); // Set data bus to output mode
                    uint8_t const bank = (addr >> 15) & 1;
                    uint32_t const rom_offset = offset + ((uint32_t)bank_registers[bank] << 14) + (addr & 0x3FFF);

                    uint8_t data;
                    uint32_t const relative_offset = (rom_offset >= offset) ? (rom_offset - offset) : cached_length;
                    if (cache_enable && relative_offset < cached_length)
                    {
                        data = rom_sram[relative_offset];
                    }
                    else
                    {
                        gpio_put(PIN_WAIT, 0);
                        data = rom_data[rom_offset];
                        gpio_put(PIN_WAIT, 1);
                    }

                    gpio_put_masked(0xFF0000, (uint32_t)data << 16); // Write the data to the data bus
                    while (!(gpio_get(PIN_RD)))  // Wait for the read cycle to complete
                    {
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16); // Return data bus to input mode after the read cycle
                }
                else if (wr) 
                {
                    // Update bank registers based on the specific switching addresses
                    if ((addr >= 0x6000) && (addr <= 0x67FF)) {
                        bank_registers[0] = (gpio_get_all() >> 16) & 0xFF;
                    } else if (addr >= 0x7000 && addr <= 0x77FF) {
                        bank_registers[1] = (gpio_get_all() >> 16) & 0xFF;
                    }
                    while (!(gpio_get(PIN_WR))) {
                        tight_loop_contents();
                    }
                }
                
            }
        }
    }
}


void __no_inline_not_in_flash_func(loadrom_nextor_sd_io)(uint32_t offset)
{
    //static uint8_t rom_sram[131072]; // Buffer for the ROM data (128KB) - TEST

    //overclock to test performance gains
    //qmi_hw->m[0].timing = 0x40000201; // Set the QMI timing for the MSX bus
    //set_sys_clock_khz(150000, true);     // Set system clock to 285Mhz

    //runs the IO code in the second core
    multicore_launch_core1(nextor_sd_io);    // Launch core 1

    //Test copying to RAM to check performance gains
    gpio_init(PIN_WAIT); // Init wait signal pin
    gpio_set_dir(PIN_WAIT, GPIO_OUT); // Set the WAIT signal as output
    gpio_put(PIN_WAIT, 0); // Wait until we are ready to read the ROM
    if (!rom_data_in_ram) {
        memset(rom_sram, 0, 131072); // Clear the SRAM buffer
        memcpy(rom_sram, rom_data + offset, 131072); //for 32KB ROMs we start at 0x4000
    }
    gpio_put(PIN_WAIT, 1); // Lets go!

    uint8_t bank_registers[2] = {0, 1}; // Initial banks 0 and 1 mapped

    gpio_set_dir_in_masked(0xFF << 16);
    while (true) {
        // Check control signals
        bool sltsl = !(gpio_get(PIN_SLTSL)); // Slot selected (active low)
        bool rd = !(gpio_get(PIN_RD));       // Read cycle (active low)
        bool wr = !(gpio_get(PIN_WR));       // Write cycle (active low)

        if (sltsl) {
            uint16_t addr = gpio_get_all() & 0x00FFFF; // Read the address bus
            if (addr >= 0x4000 && addr <= 0xBFFF) 
            {
                if (rd) {
                    gpio_set_dir_out_masked(0xFF << 16); // Set data bus to output mode
                    //uint32_t rom_offset = offset + (bank_registers[(addr >> 15) & 1] << 14) + (addr & 0x3FFF);
                    //gpio_put_masked(0xFF0000, rom[rom_offset] << 16); // Write the data to the data bus
                    //Sram - Tests
                    uint32_t rom_offset = (bank_registers[(addr >> 15) & 1] << 14) + (addr & 0x3FFF);
                    gpio_put_masked(0xFF0000, rom_sram[rom_offset] << 16); // Write the data to the data bus

                    while (!(gpio_get(PIN_RD)))  // Wait for the read cycle to complete
                    {
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16); // Return data bus to input mode after the read cycle
                }
                else if (wr) 
                {
                    // Update bank registers based on the specific switching addresses
                    if ((addr >= 0x6000) && (addr <= 0x67FF)) {
                        bank_registers[0] = (gpio_get_all() >> 16) & 0xFF;
                    } else if (addr >= 0x7000 && addr <= 0x77FF) {
                        bank_registers[1] = (gpio_get_all() >> 16) & 0xFF;
                    }
                    while (!(gpio_get(PIN_WR))) {
                        tight_loop_contents();
                    }
                }
            }
        }
    }
}

// loadrom_neo8 - Load an NEO8 ROM into the MSX directly from the pico flash
// The NEO8 ROM is divided into 8KB segments, managed by a memory mapper that allows dynamic switching of these segments into the MSX's address space
// Size of a segment: 8 KB
// Segment switching addresses:
// Bank 0: 0000h~1FFFh, Bank 1: 2000h~3FFFh, Bank 2: 4000h~5FFFh, Bank 3: 6000h~7FFFh, Bank 4: 8000h~9FFFh, Bank 5: A000h~BFFFh
// Switching address: 
// 5000h (mirror at 1000h, 9000h and D000h), 
// 5800h (mirror at 1800h, 9800h and D800h), 
// 6000h (mirror at 2000h, A000h and E000h), 
// 6800h (mirror at 2800h, A800h and E800h), 
// 7000h (mirror at 3000h, B000h and F000h), 
// 7800h (mirror at 3800h, B800h and F800h)
void __no_inline_not_in_flash_func(loadrom_neo8)(uint32_t offset)
{
    uint16_t bank_registers[6] = {0}; // 16-bit bank registers initialized to zero (12-bit segment, 4 MSB reserved)

    gpio_set_dir_in_masked(0xFF << 16);    // Configure GPIO pins for input mode
    while (true)
    {
        bool sltsl = !(gpio_get(PIN_SLTSL)); // Slot selected (active low)
        bool rd = !(gpio_get(PIN_RD));       // Read cycle (active low)
        bool wr = !(gpio_get(PIN_WR));       // Write cycle (active low)

        if (sltsl)
        {
            uint16_t addr = gpio_get_all() & 0x00FFFF; // Read address bus

            if (addr <= 0xBFFF)
            {
                if (rd)
                {
                    // Handle read access
                    gpio_set_dir_out_masked(0xFF << 16); // Data bus output mode
                    uint8_t bank_index = addr >> 13;     // Determine bank index (0-5)

                    if (bank_index < 6)
                    {
                        uint32_t segment = bank_registers[bank_index] & 0x0FFF; // 12-bit segment number
                        uint32_t rom_offset = offset + (segment << 13) + (addr & 0x1FFF); // Calculate ROM offset

                        gpio_put(PIN_WAIT, 0);
                        gpio_put_masked(0xFF0000, rom_data[rom_offset] << 16); // Place data on data bus
                        gpio_put(PIN_WAIT, 1);
                    }
                    else
                    {
                        gpio_put_masked(0xFF0000, 0xFF << 16); // Invalid page handling (Page 3)
                    }

                    while (!(gpio_get(PIN_RD))) // Wait for read cycle to complete
                    {
                        tight_loop_contents();
                    }

                    gpio_set_dir_in_masked(0xFF << 16); // Return data bus to input mode
                }
                else if (wr)
                {
                    // Handle write access
                    uint16_t base_addr = addr & 0xF800; // Mask to identify base address
                    uint8_t bank_index = 6;             // Initialize to invalid bank

                    // Determine bank index based on base address
                    switch (base_addr)
                    {
                        case 0x5000:
                        case 0x1000:
                        case 0x9000:
                        case 0xD000:
                            bank_index = 0;
                            break;
                        case 0x5800:
                        case 0x1800:
                        case 0x9800:
                        case 0xD800:
                            bank_index = 1;
                            break;
                        case 0x6000:
                        case 0x2000:
                        case 0xA000:
                        case 0xE000:
                            bank_index = 2;
                            break;
                        case 0x6800:
                        case 0x2800:
                        case 0xA800:
                        case 0xE800:
                            bank_index = 3;
                            break;
                        case 0x7000:
                        case 0x3000:
                        case 0xB000:
                        case 0xF000:
                            bank_index = 4;
                            break;
                        case 0x7800:
                        case 0x3800:
                        case 0xB800:
                        case 0xF800:
                            bank_index = 5;
                            break;
                    }

                    if (bank_index < 6)
                    {
                        uint8_t data = (gpio_get_all() >> 16) & 0xFF;
                        if (addr & 0x01)
                        {
                            // Write to MSB
                            bank_registers[bank_index] = (bank_registers[bank_index] & 0x00FF) | (data << 8);
                        }
                        else
                        {
                            // Write to LSB
                            bank_registers[bank_index] = (bank_registers[bank_index] & 0xFF00) | data;
                        }

                        // Ensure reserved MSB bits are zero
                        bank_registers[bank_index] &= 0x0FFF;
                    }

                    while (!(gpio_get(PIN_WR))) // Wait for write cycle to complete
                    {
                        tight_loop_contents();
                    }
                }
            }
        }
    }
}

// loadrom_neo16 - Load an NEO16 ROM into the MSX directly from the pico flash
// The NEO16 ROM is divided into 16KB segments, managed by a memory mapper that allows dynamic switching of these segments into the MSX's address space
// Size of a segment: 16 KB
// Segment switching addresses:
// Bank 0: 0000h~3FFFh, Bank 1: 4000h~7FFFh, Bank 2: 8000h~BFFFh
// Switching address:
// 5000h (mirror at 1000h, 9000h and D000h),
// 6000h (mirror at 2000h, A000h and E000h),
// 7000h (mirror at 3000h, B000h and F000h)
void __no_inline_not_in_flash_func(loadrom_neo16)(uint32_t offset)
{
    // 16-bit bank registers initialized to zero (12-bit segment, 4 MSB reserved)
    uint16_t bank_registers[3] = {0};

    // Configure GPIO pins for input mode
    gpio_set_dir_in_masked(0xFF << 16);
    while (true)
    {
        bool sltsl = !(gpio_get(PIN_SLTSL)); // Slot selected (active low)
        bool rd = !(gpio_get(PIN_RD));       // Read cycle (active low)
        bool wr = !(gpio_get(PIN_WR));       // Write cycle (active low)

        if (sltsl)
        {
            uint16_t addr = gpio_get_all() & 0x00FFFF; // Read address bus
            if (addr <= 0xBFFF)
            {
                if (rd)
                {
                    // Handle read access
                    gpio_set_dir_out_masked(0xFF << 16); // Data bus output mode
                    uint8_t bank_index = addr >> 14;     // Determine bank index (0-2)

                    if (bank_index < 3)
                    {
                        uint32_t segment = bank_registers[bank_index] & 0x0FFF; // 12-bit segment number
                        uint32_t rom_offset = offset + (segment << 14) + (addr & 0x3FFF); // Calculate ROM offset

                        gpio_put(PIN_WAIT, 0);
                        gpio_put_masked(0xFF0000, rom_data[rom_offset] << 16); // Place data on data bus
                        gpio_put(PIN_WAIT, 1);
                    }
                    else
                    {
                        gpio_put_masked(0xFF0000, 0xFF << 16); // Invalid page handling
                    }

                    while (!(gpio_get(PIN_RD))) // Wait for read cycle to complete
                    {
                        tight_loop_contents();
                    }

                    gpio_set_dir_in_masked(0xFF << 16); // Return data bus to input mode
                }
                else if (wr)
                {
                    // Handle write access
                    uint16_t base_addr = addr & 0xF800; // Mask to identify base address
                    uint8_t bank_index = 3;             // Initialize to invalid bank

                    // Determine bank index based on base address
                    switch (base_addr)
                    {
                        case 0x5000:
                        case 0x1000:
                        case 0x9000:
                        case 0xD000:
                            bank_index = 0;
                            break;
                        case 0x6000:
                        case 0x2000:
                        case 0xA000:
                        case 0xE000:
                            bank_index = 1;
                            break;
                        case 0x7000:
                        case 0x3000:
                        case 0xB000:
                        case 0xF000:
                            bank_index = 2;
                            break;
                    }

                    if (bank_index < 3)
                    {
                        uint8_t data = (gpio_get_all() >> 16) & 0xFF; // Read 8-bit data from bus
                        if (addr & 0x01)
                        {
                            // Write to MSB
                            bank_registers[bank_index] = (bank_registers[bank_index] & 0x00FF) | (data << 8);
                        }
                        else
                        {
                            // Write to LSB
                            bank_registers[bank_index] = (bank_registers[bank_index] & 0xFF00) | data;
                        }

                        // Ensure reserved MSB bits are zero
                        bank_registers[bank_index] &= 0x0FFF;
                    }

                    while (!(gpio_get(PIN_WR))) // Wait for write cycle to complete
                    {
                        tight_loop_contents();
                    }
                }
            }
        }
    }
}


// Main function running on core 0
int __no_inline_not_in_flash_func(main)()
{
    qmi_hw->m[0].timing = 0x40000202; // Set the QMI timing for the MSX bus
    set_sys_clock_khz(250000, true);     // Set system clock to 250Mhz

    stdio_init_all();     // Initialize stdio
    setup_gpio();     // Initialize GPIO

    int rom_index = loadrom_msx_menu(0x0000); //load the first 32KB ROM into the MSX (The MSX PICOVERSE MENU)

    ROMRecord const *selected = &records[rom_index];
    active_rom_size = selected->Size;

    uint32_t rom_offset = selected->Offset;
    rom_data = flash_rom;
    rom_data_in_ram = false;

    bool is_sd_rom = (selected->Mapper & SOURCE_SD_FLAG) != 0;
    if (is_sd_rom) {
        if (!load_rom_from_sd((uint16_t)selected->Offset, (uint32_t)selected->Size)) {
            printf("Debug: Failed to load ROM from SD card\n");
            while (true) { tight_loop_contents(); }
        }
        rom_data = rom_sram;
        rom_data_in_ram = true;
        rom_offset = 0;
    }

    bool cache_enable = !rom_data_in_ram;
    uint8_t mapper = (uint8_t)(selected->Mapper & ~SOURCE_SD_FLAG);

    // Load the selected ROM into the MSX according to the mapper
    switch (mapper) {
       
        case 1:
        case 2:
            loadrom_plain32(rom_offset, cache_enable);
            break;
        case 3:
            loadrom_konamiscc(rom_offset, cache_enable);
            break;
        case 4:
            loadrom_linear48(rom_offset, cache_enable);
            break;
        case 5:
            loadrom_ascii8(rom_offset, cache_enable); 
            break;
        case 6:
            loadrom_ascii16(rom_offset, cache_enable); 
            break;
        case 7:
            loadrom_konami(rom_offset, cache_enable); 
            break;
        case 8:
            loadrom_neo8(rom_offset); 
            break;
        case 9:
            loadrom_neo16(rom_offset); 
            break;
        case 10:
            loadrom_nextor_sd_io(rom_offset);
            break;
        default:
                printf("Debug: Unsupported ROM mapper: %d\n", mapper);
            break;
    }
    
}
