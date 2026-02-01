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
#include <stdlib.h>
#include "ff.h"
#include "diskio.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/structs/qmi.h"
#include "hw_config.h"
#include "explorer.h"
#include "nextor.h"
#include "mp3.h"

// config area and buffer for the ROM data
#define ROM_NAME_MAX    60          // Maximum size of the ROM name
#define MAX_ROM_RECORDS 1024        // Maximum ROM files supported (flash + SD)
#define MAX_FLASH_RECORDS 128       // Maximum ROM files stored in flash
#define ROM_RECORD_SIZE (ROM_NAME_MAX + 1 + (sizeof(uint32_t) * 2)) // Name + mapper + size + offset
#define MENU_ROM_SIZE   (32u * 1024u) // Full menu ROM size stored before config records
#define MONITOR_ADDR    (0xBB01)    // Monitor ROM address within image
#define CACHE_SIZE      (128u * 1024u)     // 128KB cache size for ROM data (SD ROM limit)

#define SOURCE_SD_FLAG  0x80 // Flag in the mapper byte indicating the ROM is on SD
#define FOLDER_FLAG     0x40 // Flag in the mapper byte indicating the record is a folder
#define MP3_FLAG        0x20 // Flag in the mapper byte indicating the record is an MP3 file
#define OVERRIDE_FLAG   0x10 // Flag in the mapper byte indicating manual override

#define FILES_PER_PAGE  19 // Maximum files per page on the menu
#define CTRL_BASE_ADDR  0xBFF0 // Control registers base address
#define CTRL_COUNT_L    (CTRL_BASE_ADDR + 0) // Control: total record count low byte
#define CTRL_COUNT_H    (CTRL_BASE_ADDR + 1) // Control: total record count high byte
#define CTRL_PAGE       (CTRL_BASE_ADDR + 2) // Control: current page index
#define CTRL_STATUS     (CTRL_BASE_ADDR + 3) // Control: status register
#define CTRL_CMD        (CTRL_BASE_ADDR + 4) // Control: command register
#define CTRL_MATCH_L    (CTRL_BASE_ADDR + 5) // Control: match index low byte
#define CTRL_MATCH_H    (CTRL_BASE_ADDR + 6) // Control: match index high byte
#define CTRL_MAPPER     (CTRL_BASE_ADDR + 7) // Control: mapper value
#define CTRL_ACK        (CTRL_BASE_ADDR + 8) // Control: command ack
#define CTRL_MAGIC      0xA5 // Control: magic value to indicate valid command
#define CTRL_QUERY_BASE 0xBFC0 // Control: query string base address
#define CTRL_QUERY_SIZE 32 // Control: query string size
#define CMD_APPLY_FILTER 0x01 // Command: apply filter
#define CMD_FIND_FIRST   0x02 // Command: find first match
#define CMD_ENTER_DIR    0x03 // Command: enter directory
#define CMD_DETECT_MAPPER 0x04 // Command: detect mapper on demand
#define CMD_SET_MAPPER    0x05 // Command: set mapper override

#define DATA_BASE_ADDR   0xA000 // Data buffer base address
#define DATA_BUFFER_SIZE (CTRL_QUERY_BASE - DATA_BASE_ADDR) // Data buffer size
#define DATA_MAGIC_0     'P' // Data header magic bytes
#define DATA_MAGIC_1     'V' // Data header magic bytes
#define DATA_MAGIC_2     'E' // Data header magic bytes
#define DATA_MAGIC_3     'X' // Data header magic bytes
#define DATA_VERSION     1 // Data header version
#define DATA_HEADER_SIZE 24 // Data header size in bytes
#define DATA_RECORD_TABLE_ENTRY_SIZE 4 // Size of each record table entry
#define DATA_RECORD_PAYLOAD_SIZE 13 // Minimum size of each record payload
#define TLV_NAME_OFFSET  0x01 // Type-Length-Value: Name offset
#define TLV_MAPPER       0x02 // Type-Length-Value: Mapper
#define TLV_SIZE         0x03 // Type-Length-Value: Size


// MP3 control registers
#define MP3_CTRL_BASE      0xBFE0 // MP3 control registers base address
#define MP3_CTRL_CMD       (MP3_CTRL_BASE + 0)
#define MP3_CTRL_STATUS    (MP3_CTRL_BASE + 1)
#define MP3_CTRL_INDEX_L   (MP3_CTRL_BASE + 2)
#define MP3_CTRL_INDEX_H   (MP3_CTRL_BASE + 3)
#define MP3_CTRL_ELAPSED_L (MP3_CTRL_BASE + 4)
#define MP3_CTRL_ELAPSED_H (MP3_CTRL_BASE + 5)
#define MP3_CTRL_TOTAL_L   (MP3_CTRL_BASE + 6)
#define MP3_CTRL_TOTAL_H   (MP3_CTRL_BASE + 7)

// MP3 command codes
#define MP3_CMD_SELECT      0x01
#define MP3_CMD_PLAY        0x02
#define MP3_CMD_STOP        0x03
#define MP3_CMD_TOGGLE_MUTE 0x04
#define MP3_CMD_TONE        0x05

// This symbol marks the end of the main program in flash.
// Custom data starts right after it
extern const uint8_t __flash_binary_end[];

// SRAM buffer to cache ROM data
static uint8_t rom_sram[CACHE_SIZE];
static uint32_t active_rom_size = 0;
static uint8_t page_buffer[DATA_BUFFER_SIZE];

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
#define SD_PATH_BUFFER_SIZE 19000
#define MIN_ROM_SIZE       8192
#define MAX_ROM_SIZE       (15u * 1024u * 1024u)
#define MAX_ANALYSIS_SIZE  65536
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
static bool sd_mounted = false;
static sd_card_t *sd_card = NULL;
static uint16_t total_record_count = 0;
static uint16_t full_record_count = 0;
static uint8_t current_page = 0;
static uint16_t filtered_indices[MAX_ROM_RECORDS];
static char filter_query[CTRL_QUERY_SIZE];
static volatile uint8_t ctrl_cmd_state = 0;
static volatile uint8_t ctrl_mapper_value = 0;
static volatile uint8_t ctrl_ack_value = 0;
static uint16_t match_index = 0xFFFF;
static ROMRecord flash_records[MAX_FLASH_RECORDS];
static uint16_t flash_record_count = 0;
static char sd_current_path[SD_PATH_MAX] = "/";
static volatile bool refresh_requested = false;
static volatile bool refresh_in_progress = false;
static bool refresh_worker_started = false;
static volatile bool detect_mapper_pending = false;
static volatile uint16_t detect_mapper_index = 0;
static volatile uint16_t mp3_selected_index = 0;
static bool mp3_initialized = false;
static volatile bool mp3_pending_select = false;
static volatile uint16_t mp3_pending_index = 0;
static volatile uint8_t mp3_pending_cmd = 0;

static const char *EXCLUDED_SD_FOLDERS[] = {
    "System Volume Information"
};
static const size_t EXCLUDED_SD_FOLDER_COUNT = sizeof(EXCLUDED_SD_FOLDERS) / sizeof(EXCLUDED_SD_FOLDERS[0]);

static void write_u32_le(uint8_t *ptr, uint32_t value);
static void write_u16_le(uint8_t *ptr, uint16_t value);
static void build_page_buffer(uint8_t page_index);
static void refresh_records_for_current_path(void);
static void refresh_worker(void);

static int compare_record_names(const ROMRecord *a, const ROMRecord *b) {
    return strncmp(a->Name, b->Name, ROM_NAME_MAX);
}

static bool is_system_record(const ROMRecord *record) {
    return ((record->Mapper & ~(SOURCE_SD_FLAG | OVERRIDE_FLAG)) == MAPPER_SYSTEM);
}

static bool is_folder_record(const ROMRecord *record) {
    return (record->Mapper & FOLDER_FLAG) != 0;
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

static void swap_records(uint16_t a, uint16_t b) {
    if (a == b) {
        return;
    }
    ROMRecord temp = records[a];
    records[a] = records[b];
    records[b] = temp;
    uint16_t path_temp = sd_path_offsets[a];
    sd_path_offsets[a] = sd_path_offsets[b];
    sd_path_offsets[b] = path_temp;
}

static void sort_records_range(uint16_t start, uint16_t count) {
    if (count < 2) {
        return;
    }
    uint16_t end = (uint16_t)(start + count);
    for (uint16_t i = start; i + 1 < end; i++) {
        for (uint16_t j = i + 1; j < end; j++) {
            if (compare_record_names(&records[i], &records[j]) > 0) {
                swap_records(i, j);
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
    uint8_t *buffer = page_buffer;
    const uint16_t buffer_size = DATA_BUFFER_SIZE;
    uint16_t start_index = (uint16_t)page_index * FILES_PER_PAGE;
    uint16_t record_count = 0;

    if (start_index < total_record_count) {
        uint16_t remaining = (uint16_t)(total_record_count - start_index);
        record_count = (remaining > FILES_PER_PAGE) ? FILES_PER_PAGE : remaining;
    }

    memset(buffer, 0xFF, buffer_size);

    uint16_t table_offset = DATA_HEADER_SIZE;
    uint16_t table_size = (uint16_t)(record_count * DATA_RECORD_TABLE_ENTRY_SIZE);
    uint16_t payload_size = (uint16_t)(record_count * DATA_RECORD_PAYLOAD_SIZE);
    uint16_t string_pool_offset = (uint16_t)(table_offset + table_size);
    uint16_t max_string_pool = 0;
    if ((uint32_t)string_pool_offset + payload_size <= buffer_size) {
        max_string_pool = (uint16_t)(buffer_size - (string_pool_offset + payload_size));
    }

    uint16_t name_offsets[FILES_PER_PAGE];
    uint16_t string_cursor = 0;
    for (uint16_t i = 0; i < record_count; i++) {
        uint16_t filtered_index = (uint16_t)(start_index + i);
        uint16_t record_index = filtered_indices[filtered_index];
        char name_buf[ROM_NAME_MAX + 1];
        size_t name_len = trim_name_copy(name_buf, records[record_index].Name);
        if (name_len + 1 <= (size_t)(max_string_pool - string_cursor)) {
            name_offsets[i] = string_cursor;
            memcpy(buffer + string_pool_offset + string_cursor, name_buf, name_len);
            buffer[string_pool_offset + string_cursor + name_len] = '\0';
            string_cursor = (uint16_t)(string_cursor + name_len + 1);
        } else {
            name_offsets[i] = 0xFFFF;
        }
    }

    uint16_t string_pool_size = string_cursor;
    uint16_t payload_offset = (uint16_t)(string_pool_offset + string_pool_size);
    uint16_t payload_cursor = 0;

    for (uint16_t i = 0; i < record_count; i++) {
        uint16_t filtered_index = (uint16_t)(start_index + i);
        uint16_t record_index = filtered_indices[filtered_index];
        uint8_t *entry = buffer + table_offset + (i * DATA_RECORD_TABLE_ENTRY_SIZE);
        uint16_t record_offset = (uint16_t)(payload_offset + payload_cursor);
        uint16_t record_length = DATA_RECORD_PAYLOAD_SIZE;

        write_u16_le(entry, record_offset);
        write_u16_le(entry + 2, record_length);

        uint8_t *rec = buffer + record_offset;
        rec[0] = TLV_NAME_OFFSET;
        rec[1] = 2;
        write_u16_le(rec + 2, name_offsets[i]);
        rec[4] = TLV_MAPPER;
        rec[5] = 1;
        rec[6] = records[record_index].Mapper;
        rec[7] = TLV_SIZE;
        rec[8] = 4;
        write_u32_le(rec + 9, (uint32_t)records[record_index].Size);

        payload_cursor = (uint16_t)(payload_cursor + record_length);
    }

    buffer[0] = DATA_MAGIC_0;
    buffer[1] = DATA_MAGIC_1;
    buffer[2] = DATA_MAGIC_2;
    buffer[3] = DATA_MAGIC_3;
    buffer[4] = DATA_VERSION;
    buffer[5] = 0;
    write_u16_le(buffer + 6, DATA_HEADER_SIZE);
    write_u16_le(buffer + 8, total_record_count);
    write_u16_le(buffer + 10, page_index);
    write_u16_le(buffer + 12, record_count);
    write_u16_le(buffer + 14, table_offset);
    write_u16_le(buffer + 16, table_size);
    write_u16_le(buffer + 18, string_pool_offset);
    write_u16_le(buffer + 20, string_pool_size);
    write_u16_le(buffer + 22, payload_offset);
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

static bool is_excluded_folder(const char *name) {
    if (!name || name[0] == '\0') {
        return true;
    }
    if (equals_ignore_case(name, ".") || equals_ignore_case(name, "..")) {
        return true;
    }
    for (size_t i = 0; i < EXCLUDED_SD_FOLDER_COUNT; i++) {
        if (equals_ignore_case(name, EXCLUDED_SD_FOLDERS[i])) {
            return true;
        }
    }
    return false;
}

static bool is_root_path(const char *path) {
    if (!path || path[0] == '\0') {
        return true;
    }
    if (path[0] == '/' && path[1] == '\0') {
        return true;
    }
    return false;
}

static void set_root_path(void) {
    sd_current_path[0] = '/';
    sd_current_path[1] = '\0';
}

static void go_up_one_level(void) {
    if (is_root_path(sd_current_path)) {
        set_root_path();
        return;
    }

    size_t len = strlen(sd_current_path);
    if (len == 0) {
        set_root_path();
        return;
    }

    while (len > 1 && sd_current_path[len - 1] == '/') {
        sd_current_path[len - 1] = '\0';
        len--;
    }

    char *slash = strrchr(sd_current_path, '/');
    if (!slash || slash == sd_current_path) {
        set_root_path();
    } else {
        *slash = '\0';
    }
}

static void append_folder_to_path(const char *folder) {
    if (!folder || folder[0] == '\0') {
        return;
    }
    if (equals_ignore_case(folder, "..")) {
        go_up_one_level();
        return;
    }

    size_t base_len = strlen(sd_current_path);
    size_t folder_len = strlen(folder);
    if (base_len + folder_len + 2 >= sizeof(sd_current_path)) {
        return;
    }

    if (!is_root_path(sd_current_path)) {
        sd_current_path[base_len] = '/';
        base_len++;
    }
    memcpy(sd_current_path + base_len, folder, folder_len + 1);
}

static void write_u32_le(uint8_t *ptr, uint32_t value) {
    ptr[0] = (uint8_t)(value & 0xFFu);
    ptr[1] = (uint8_t)((value >> 8) & 0xFFu);
    ptr[2] = (uint8_t)((value >> 16) & 0xFFu);
    ptr[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static void write_u16_le(uint8_t *ptr, uint16_t value) {
    ptr[0] = (uint8_t)(value & 0xFFu);
    ptr[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static const char *basename_from_path(const char *path) {
    if (!path) {
        return "";
    }
    const char *slash = strrchr(path, '/');
    return slash ? (slash + 1) : path;
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

static bool has_mp3_extension(const char *filename) {
    const char *dot = strrchr(filename, '.');
    if (!dot || dot[1] == '\0') {
        return false;
    }
    return equals_ignore_case(dot + 1, "MP3");
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
    uint8_t *rom_analysis_buffer = (uint8_t *)malloc(read_size);
    if (!rom_analysis_buffer) {
        f_close(&fil);
        return 0;
    }
    fr = f_read(&fil, rom_analysis_buffer, (UINT)read_size, &br);
    f_close(&fil);
    if (fr != FR_OK || br < read_size) {
        free(rom_analysis_buffer);
        return 0;
    }

    uint8_t mapper = detect_rom_type_from_buffer(rom_analysis_buffer, size, read_size);
    free(rom_analysis_buffer);
    return mapper;
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

static uint16_t add_folder_record(uint16_t record_index, const char *name) {
    if (record_index >= MAX_ROM_RECORDS) {
        return record_index;
    }
    memset(records[record_index].Name, 0, sizeof(records[record_index].Name));
    strncpy(records[record_index].Name, name, sizeof(records[record_index].Name) - 1);
    records[record_index].Mapper = (unsigned char)(FOLDER_FLAG | SOURCE_SD_FLAG);
    records[record_index].Size = 0;
    records[record_index].Offset = 0;
    sd_path_offsets[record_index] = 0xFFFF;
    return (uint16_t)(record_index + 1);
}

static uint16_t add_sd_rom_record(uint16_t record_index, const char *path, const char *filename, uint32_t size, uint8_t mapper) {
    if (record_index >= MAX_ROM_RECORDS) {
        return record_index;
    }
    size_t path_len = strlen(path);
    if (sd_path_buffer_used + path_len + 1 > SD_PATH_BUFFER_SIZE) {
        return record_index;
    }
    sd_path_offsets[record_index] = sd_path_buffer_used;
    memcpy(sd_path_buffer + sd_path_buffer_used, path, path_len + 1);
    sd_path_buffer_used = (uint16_t)(sd_path_buffer_used + path_len + 1);

    char display_name[ROM_NAME_MAX + 1];
    build_display_name(filename, display_name, sizeof(display_name));

    memset(records[record_index].Name, 0, sizeof(records[record_index].Name));
    strncpy(records[record_index].Name, display_name, sizeof(records[record_index].Name) - 1);
    records[record_index].Mapper = (unsigned char)(mapper | SOURCE_SD_FLAG);
    records[record_index].Size = (unsigned long)size;
    records[record_index].Offset = (unsigned long)record_index;
    return (uint16_t)(record_index + 1);
}

static uint16_t add_sd_mp3_record(uint16_t record_index, const char *path, const char *filename, uint32_t size) {
    if (record_index >= MAX_ROM_RECORDS) {
        return record_index;
    }
    size_t path_len = strlen(path);
    if (sd_path_buffer_used + path_len + 1 > SD_PATH_BUFFER_SIZE) {
        return record_index;
    }
    sd_path_offsets[record_index] = sd_path_buffer_used;
    memcpy(sd_path_buffer + sd_path_buffer_used, path, path_len + 1);
    sd_path_buffer_used = (uint16_t)(sd_path_buffer_used + path_len + 1);

    char display_name[ROM_NAME_MAX + 1];
    build_display_name(filename, display_name, sizeof(display_name));

    memset(records[record_index].Name, 0, sizeof(records[record_index].Name));
    strncpy(records[record_index].Name, display_name, sizeof(records[record_index].Name) - 1);
    records[record_index].Mapper = (unsigned char)(MP3_FLAG | SOURCE_SD_FLAG);
    records[record_index].Size = (unsigned long)size;
    records[record_index].Offset = (unsigned long)record_index;
    return (uint16_t)(record_index + 1);
}

static void refresh_records_for_current_path(void) {
    sd_record_count = 0;
    sd_path_buffer_used = 0;
    for (uint16_t i = 0; i < MAX_ROM_RECORDS; i++) {
        sd_path_offsets[i] = 0xFFFF;
    }

    uint16_t record_index = 0;
    bool has_parent = !is_root_path(sd_current_path);

    if (has_parent) {
        record_index = add_folder_record(record_index, "..");
    }

    uint16_t folder_count = record_index;
    if (sd_mount_card()) {
        DIR dir;
        FILINFO fno;
        FRESULT fr = f_opendir(&dir, sd_current_path);
        if (fr == FR_OK) {
            while (record_index < MAX_ROM_RECORDS) {
                fr = f_readdir(&dir, &fno);
                if (fr != FR_OK || fno.fname[0] == '\0') {
                    break;
                }
                if (!(fno.fattrib & AM_DIR)) {
                    continue;
                }
                if (is_excluded_folder(fno.fname)) {
                    continue;
                }
                record_index = add_folder_record(record_index, fno.fname);
            }
        }
        f_closedir(&dir);

        folder_count = record_index;
        uint16_t folder_sort_start = has_parent ? 1u : 0u;
        if (folder_count > folder_sort_start) {
            sort_records_range(folder_sort_start, (uint16_t)(folder_count - folder_sort_start));
        }

        fr = f_opendir(&dir, sd_current_path);
        if (fr == FR_OK) {
            while (record_index < MAX_ROM_RECORDS) {
                fr = f_readdir(&dir, &fno);
                if (fr != FR_OK || fno.fname[0] == '\0') {
                    break;
                }
                if (fno.fattrib & AM_DIR) {
                    continue;
                }
                bool is_mp3 = has_mp3_extension(fno.fname);
                bool is_rom = has_rom_extension(fno.fname);

                if (!is_mp3 && !is_rom) {
                    continue;
                }

                char path[SD_PATH_MAX];
                if (is_root_path(sd_current_path)) {
                    snprintf(path, sizeof(path), "/%s", fno.fname);
                } else {
                    snprintf(path, sizeof(path), "%s/%s", sd_current_path, fno.fname);
                }

                if (is_mp3) {
                    if (fno.fsize == 0) {
                        continue;
                    }
                    record_index = add_sd_mp3_record(record_index, path, fno.fname, (uint32_t)fno.fsize);
                    sd_record_count++;
                    continue;
                }

                if (fno.fsize < MIN_ROM_SIZE || fno.fsize > MAX_ROM_SIZE) {
                    continue;
                }
                if (fno.fsize > CACHE_SIZE) {
                    continue;
                }

                uint8_t mapper = mapper_number_from_filename(fno.fname);
                record_index = add_sd_rom_record(record_index, path, fno.fname, (uint32_t)fno.fsize, mapper);
                sd_record_count++;
            }
        }
        f_closedir(&dir);
    }

    if (is_root_path(sd_current_path) && flash_record_count > 0) {
        for (uint16_t i = 0; i < flash_record_count && record_index < MAX_ROM_RECORDS; i++) {
            records[record_index] = flash_records[i];
            sd_path_offsets[record_index] = 0xFFFF;
            record_index++;
        }
    }

    uint16_t rom_count = (uint16_t)(record_index - folder_count);
    if (rom_count > 1) {
        uint16_t system_insert = folder_count;
        for (uint16_t i = folder_count; i < record_index; i++) {
            if (is_system_record(&records[i])) {
                swap_records(i, system_insert);
                system_insert++;
            }
        }

        uint16_t system_count = (uint16_t)(system_insert - folder_count);
        uint16_t non_system_count = (uint16_t)(record_index - system_insert);
        if (system_count > 1) {
            sort_records_range(folder_count, system_count);
        }
        if (non_system_count > 1) {
            sort_records_range(system_insert, non_system_count);
        }
    }

    full_record_count = record_index;
    memset(filter_query, 0, sizeof(filter_query));
    apply_filter();
    current_page = 0;
    build_page_buffer(current_page);
}

static void process_detect_mapper_request(uint16_t filtered_index) {
    ctrl_mapper_value = 0;

    if (!sd_mount_card()) {
        ctrl_cmd_state = 0;
        return;
    }

    if (filtered_index >= total_record_count) {
        ctrl_cmd_state = 0;
        return;
    }

    uint16_t record_index = filtered_indices[filtered_index];
    if (record_index >= full_record_count) {
        ctrl_cmd_state = 0;
        return;
    }

    ROMRecord *rec = &records[record_index];
    uint8_t flags = rec->Mapper & (SOURCE_SD_FLAG | FOLDER_FLAG | MP3_FLAG);

    if (!(flags & SOURCE_SD_FLAG) || (flags & (FOLDER_FLAG | MP3_FLAG)) || sd_path_offsets[record_index] == 0xFFFF) {
        ctrl_mapper_value = rec->Mapper & ~(SOURCE_SD_FLAG | FOLDER_FLAG | MP3_FLAG | OVERRIDE_FLAG);
        ctrl_cmd_state = 0;
        return;
    }

    const char *path = sd_path_buffer + sd_path_offsets[record_index];
    const char *filename = basename_from_path(path);

    uint8_t mapper = mapper_number_from_filename(filename);
    if (mapper == 0) {
        mapper = detect_rom_type_from_file(path, (uint32_t)rec->Size);
    }

    ctrl_mapper_value = mapper;
    if (mapper != 0) {
        rec->Mapper = (uint8_t)(flags | mapper);
    } else {
        rec->Mapper = flags;
    }

    ctrl_cmd_state = 0;
}

static void refresh_worker(void) {
    while (true) {
        if (refresh_requested) {
            refresh_requested = false;
            refresh_in_progress = true;
            refresh_records_for_current_path();
            refresh_in_progress = false;
            ctrl_cmd_state = 0;
        }
        if (detect_mapper_pending && !refresh_in_progress) {
            uint16_t pending_index = detect_mapper_index;
            detect_mapper_pending = false;
            process_detect_mapper_request(pending_index);
        }
        if (mp3_pending_select) {
            uint16_t pending_index = mp3_pending_index;
            mp3_pending_select = false;
            if (!mp3_initialized) {
                printf("MP3: init on select\n");
                mp3_init();
                mp3_initialized = true;
            }
            if (pending_index < total_record_count) {
                uint16_t record_index = filtered_indices[pending_index];
                if (record_index < full_record_count) {
                    ROMRecord const *rec = &records[record_index];
                    if ((rec->Mapper & MP3_FLAG) && (sd_path_offsets[record_index] != 0xFFFF)) {
                        const char *path = sd_path_buffer + sd_path_offsets[record_index];
                        printf("MP3: select path=%s size=%lu\n", path, (unsigned long)rec->Size);
                        mp3_select_file(path, (uint32_t)rec->Size);
                    } else {
                        printf("MP3: select ignored (not mp3 or no path)\n");
                    }
                } else {
                    printf("MP3: select index out of range\n");
                }
            } else {
                printf("MP3: select index invalid (total=%u)\n", (unsigned int)total_record_count);
            }
        }
        if (mp3_pending_cmd != 0) {
            uint8_t cmd = mp3_pending_cmd;
            mp3_pending_cmd = 0;
            printf("MP3: cmd=0x%02X\n", cmd);
            switch (cmd) {
                case MP3_CMD_PLAY:
                    mp3_play();
                    break;
                case MP3_CMD_STOP:
                    mp3_stop();
                    break;
                case MP3_CMD_TOGGLE_MUTE:
                    mp3_toggle_mute();
                    break;
                case MP3_CMD_TONE:
                    mp3_play_tone();
                    break;
                default:
                    break;
            }
        }
        mp3_update();
        tight_loop_contents();
    }
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
    memset(rom_sram, 0, MENU_ROM_SIZE); // Clear the SRAM buffer
    memcpy(rom_sram, flash_rom + offset, MENU_ROM_SIZE); // Load full 32KB menu ROM
    gpio_put(PIN_WAIT, 1); // Lets go!

    int record_count = 0; // Record count
    const uint8_t *record_ptr = flash_rom + offset + MENU_ROM_SIZE; // Pointer to the ROM records
    for (int i = 0; i < MAX_FLASH_RECORDS; i++)      // Read the ROMs from the configuration area
    {
        if (isEndOfData(record_ptr)) {
            break; // Stop if end of data is reached
        }
        char flash_name[ROM_NAME_MAX + 1];
        memcpy(flash_name, record_ptr, ROM_NAME_MAX);
        flash_name[ROM_NAME_MAX] = '\0';
        memset(flash_records[record_count].Name, 0, sizeof(flash_records[record_count].Name));
        trim_name_copy(flash_records[record_count].Name, flash_name);
        record_ptr += ROM_NAME_MAX; // Move the pointer to the next field
        flash_records[record_count].Mapper = *record_ptr++; // Read the mapper code
        flash_records[record_count].Size = read_ulong(record_ptr); // Read the ROM size
        record_ptr += sizeof(unsigned long); // Move the pointer to the next field
        flash_records[record_count].Offset = read_ulong(record_ptr); // Read the ROM offset
        record_ptr += sizeof(unsigned long); // Move the pointer to the next record
        record_count++; // Increment the record count
    }
    flash_record_count = (uint16_t)record_count;
    set_root_path();
    refresh_records_for_current_path();

    if (!refresh_worker_started) {
        refresh_worker_started = true;
        multicore_launch_core1(refresh_worker);
    }

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
                    else if (cmd == CMD_ENTER_DIR)
                    {
                        filter_query[CTRL_QUERY_SIZE - 1] = '\0';
                        if (filter_query[0] == '\0') {
                            go_up_one_level();
                        } else {
                            append_folder_to_path(filter_query);
                        }
                        refresh_requested = true;
                        ctrl_cmd_state = CMD_ENTER_DIR;
                    }
                    else if (cmd == CMD_DETECT_MAPPER)
                    {
                        uint16_t index = (uint16_t)((uint8_t)filter_query[0]) |
                                         (uint16_t)(((uint8_t)filter_query[1]) << 8);
                        detect_mapper_index = index;
                        detect_mapper_pending = true;
                        ctrl_mapper_value = 0;
                        ctrl_cmd_state = CMD_DETECT_MAPPER;
                    }
                    else if (cmd == CMD_SET_MAPPER)
                    {
                        uint16_t index = (uint16_t)((uint8_t)filter_query[0]) |
                                         (uint16_t)(((uint8_t)filter_query[1]) << 8);
                        uint8_t mapper = (uint8_t)filter_query[2];
                        ctrl_ack_value = 0;

                        if (index < total_record_count && mapper != 0 && mapper != MAPPER_SYSTEM && mapper < MAPPER_DESCRIPTION_COUNT) {
                            uint16_t record_index = filtered_indices[index];
                            if (record_index < full_record_count) {
                                ROMRecord *rec = &records[record_index];
                                uint8_t flags = rec->Mapper & (SOURCE_SD_FLAG | FOLDER_FLAG | MP3_FLAG);
                                if ((flags & (FOLDER_FLAG | MP3_FLAG)) == 0) {
                                    rec->Mapper = (uint8_t)(flags | OVERRIDE_FLAG | mapper);
                                    ctrl_ack_value = 1;
                                }
                            }
                        }
                        ctrl_cmd_state = 0;
                    }
                    if (cmd != CMD_ENTER_DIR && cmd != CMD_DETECT_MAPPER) {
                        ctrl_cmd_state = 0;
                    }
                    while (!(gpio_get(PIN_WR))) {
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16);
            }

            if (wr && addr == MP3_CTRL_INDEX_L)
            {
                    uint8_t value = (gpio_get_all() >> 16) & 0xFF;
                    mp3_selected_index = (uint16_t)((mp3_selected_index & 0xFF00u) | value);
                    while (!(gpio_get(PIN_WR))) {
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16);
            }

            if (wr && addr == MP3_CTRL_INDEX_H)
            {
                    uint8_t value = (gpio_get_all() >> 16) & 0xFF;
                    mp3_selected_index = (uint16_t)((mp3_selected_index & 0x00FFu) | ((uint16_t)value << 8));
                    while (!(gpio_get(PIN_WR))) {
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16);
            }

            if (wr && addr == MP3_CTRL_CMD)
            {
                    uint8_t cmd = (gpio_get_all() >> 16) & 0xFF;

                    if (cmd == MP3_CMD_SELECT)
                    {
                        if (mp3_selected_index < total_record_count)
                        {
                            mp3_pending_index = mp3_selected_index;
                            mp3_pending_select = true;
                        }
                    }
                    else if (cmd == MP3_CMD_PLAY || cmd == MP3_CMD_STOP || cmd == MP3_CMD_TOGGLE_MUTE || cmd == MP3_CMD_TONE)
                    {
                        mp3_pending_cmd = cmd;
                    }

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
                            case CTRL_MAPPER:
                                ctrl_value = ctrl_mapper_value;
                                break;
                            case CTRL_ACK:
                                ctrl_value = ctrl_ack_value;
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

                    if (addr >= MP3_CTRL_BASE && addr <= (MP3_CTRL_BASE + 0x0F))
                    {
                        uint8_t ctrl_value = 0xFF;
                        uint16_t elapsed = mp3_get_elapsed_seconds();
                        uint16_t total = mp3_get_total_seconds();

                        switch (addr)
                        {
                            case MP3_CTRL_STATUS:
                                ctrl_value = mp3_get_status();
                                break;
                            case MP3_CTRL_ELAPSED_L:
                                ctrl_value = (uint8_t)(elapsed & 0xFFu);
                                break;
                            case MP3_CTRL_ELAPSED_H:
                                ctrl_value = (uint8_t)((elapsed >> 8) & 0xFFu);
                                break;
                            case MP3_CTRL_TOTAL_L:
                                ctrl_value = (uint8_t)(total & 0xFFu);
                                break;
                            case MP3_CTRL_TOTAL_H:
                                ctrl_value = (uint8_t)((total >> 8) & 0xFFu);
                                break;
                            case MP3_CTRL_INDEX_L:
                                ctrl_value = (uint8_t)(mp3_selected_index & 0xFFu);
                                break;
                            case MP3_CTRL_INDEX_H:
                                ctrl_value = (uint8_t)((mp3_selected_index >> 8) & 0xFFu);
                                break;
                        }
                        gpio_set_dir_out_masked(0xFF << 16);
                        gpio_put_masked(0xFF0000, (uint32_t)ctrl_value << 16);
                        while (!(gpio_get(PIN_RD))) {
                            tight_loop_contents();
                        }
                        gpio_set_dir_in_masked(0xFF << 16);
                        continue;
                    }

                    gpio_set_dir_out_masked(0xFF << 16); // Set data bus to output mode
                    uint8_t data;
                    if (addr >= DATA_BASE_ADDR && addr < (DATA_BASE_ADDR + DATA_BUFFER_SIZE)) {
                        data = page_buffer[addr - DATA_BASE_ADDR];
                    } else {
                        uint32_t rom_addr = (uint32_t)(addr - 0x4000); // Offset within 32KB menu ROM
                        data = rom_sram[rom_addr];
                    }
                    gpio_put_masked(0xFF0000, (uint32_t)data << 16); // Write the data to the data bus
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

    multicore_reset_core1();

    ROMRecord const *selected = &records[rom_index];
    active_rom_size = selected->Size;

    uint32_t rom_offset = selected->Offset;
    rom_data = flash_rom;
    rom_data_in_ram = false;

    bool is_sd_rom = (selected->Mapper & SOURCE_SD_FLAG) != 0;
    if (is_sd_rom) {
        if (!load_rom_from_sd((uint16_t)rom_index, (uint32_t)selected->Size)) {
            printf("Debug: Failed to load ROM from SD card\n");
            while (true) { tight_loop_contents(); }
        }
        rom_data = rom_sram;
        rom_data_in_ram = true;
        rom_offset = 0;
    }

    bool cache_enable = !rom_data_in_ram;
    uint8_t mapper = (uint8_t)(selected->Mapper & ~(SOURCE_SD_FLAG | OVERRIDE_FLAG | FOLDER_FLAG | MP3_FLAG));

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
