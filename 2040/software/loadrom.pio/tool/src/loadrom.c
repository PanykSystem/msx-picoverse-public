// MSX PICOVERSE PROJECT
// (c) 2026 Cristiano Goncalves
// The Retro Hacker
//
// loadrom.c - Windows console application to create a loadrom UF2 file for the MSX PICOVERSE 2040
//
// This program creates a UF2 file to program the Raspberry Pi Pico with the MSX PICOVERSE 2040 loadROM firmware. The UF2 file is
// created with the combined PICO firmware binary file, the configuration area and the ROM file. The configuration area contains the
// information of the ROM file processed by the tool so the MSX can have the required information to load the ROM and execute.
// 
// The configuration record has the following structure:
//  game - Game name                            - 20 bytes (padded by 0x00)
//  mapp - Mapper code                          - 01 byte  (1 - Plain16, 2 - Plain32, 3 - KonamiSCC, 4 - Linear0, 5 - ASCII8, 6 - ASCII16, 7 - Konami)
//  size - Size of the ROM in bytes             - 04 bytes 
//  offset - Offset of the game in the flash    - 04 bytes 
//
// This work is licensed  under a "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. 
// https://creativecommons.org/licenses/by-nc-sa/4.0/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <dirent.h>
#include <string.h>
#include <ctype.h>
#include "uf2format.h"
#include "loadrom.h"

#ifndef APP_VERSION
#define APP_VERSION "v1.0"
#endif

#define UF2FILENAME             "loadrom.uf2"          // this is the UF2 file to program the Raspberry Pi Pico
#define UF2_FLAG_FAMILYID_PRESENT 0x00002000UL           // Signals that fileSize stores the RP2040 family ID
#define RP2040_FAMILY_ID        0xE48BFF56UL             // RP2040 family identifier expected by UF2 bootloader

#define MAX_FILE_NAME_LENGTH    50              // Maximum length of a ROM name
#define MAX_ROM_SIZE            16*1024*1024    // Maximum size of a ROM file
#define FLASH_START             0x10000000      // Start of the flash memory on the Raspberry Pi Pico
#define MIN_ROM_SIZE            8192           // Minimum size of a ROM file
#define MAX_ANALYSIS_SIZE       131072         // 128KB for the mapper analysis
#define CONFIG_RECORD_SIZE      (MAX_FILE_NAME_LENGTH + 1 + sizeof(uint32_t) + sizeof(uint32_t))


uint32_t file_size(const char *filename);
uint8_t detect_rom_type(const char *filename, uint32_t size);
void write_padding(FILE *file, size_t current_size, size_t target_size, uint8_t padding_byte);
void create_uf2_file(const char *rom_filename, uint32_t rom_size, uint8_t rom_type,
                     const char *rom_name, uint32_t base_offset, const char *uf2_filename);

static const char *MAPPER_DESCRIPTIONS[] = {
    "PL-16", "PL-32", "KonSCC", "Linear", "ASC-08",
    "ASC-16", "Konami", "NEO-8", "NEO-16", "SYSTEM"
};

static const char *rom_types[] = {
    "Unknown",
    "Plain16",
    "Plain32",
    "Konami SCC",
    "Linear0",
    "ASCII8",
    "ASCII16",
    "Konami",
    "NEO8",
    "NEO16"
};

#define MAPPER_DESCRIPTION_COUNT (sizeof(MAPPER_DESCRIPTIONS) / sizeof(MAPPER_DESCRIPTIONS[0]))

static bool equals_ignore_case(const char *a, const char *b) {
    if (!a || !b) {
        return false;
    }
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

static uint8_t mapper_number_from_description(const char *description) {
    for (size_t i = 0; i < MAPPER_DESCRIPTION_COUNT; ++i) {
        if (equals_ignore_case(description, MAPPER_DESCRIPTIONS[i])) {
            return (uint8_t)(i + 1);
        }
    }
    return 0;
}

// Return byte length of a file on disk, 0 on failure.
uint32_t file_size(const char *filename) {
    FILE *file = fopen(filename, "rb");
    if (!file) {
        return 0;
    }
    fseek(file, 0, SEEK_END);
    uint32_t size = ftell(file);
    fclose(file);
    return size;
}

// Return a textual description of the mapper type given its number.
const char* mapper_description(int number) {
    if (number <= 0 || (size_t)number > MAPPER_DESCRIPTION_COUNT) {
        return "Unknown";
    }
    return MAPPER_DESCRIPTIONS[number - 1];
}

// Attempt to guess the mapper type from the ROM contents.
// Returns the mapper byte expected by the firmware (0 signals unsupported/unknown).
// Code adapted from openMSX mapper detection routines.
uint8_t detect_rom_type(const char *filename, uint32_t size) {
    
    // Define the NEO8 signature
    const char neo8_signature[] = "ROM_NEO8";
    const char neo16_signature[] = "ROM_NE16";

    // Initialize weighted scores for different mapper types
    int konami_score = 0;
    int konami_scc_score = 0;
    int ascii8_score = 0;
    int ascii16_score = 0;

    // Define weights for specific addresses
    const int KONAMI_WEIGHT = 2;
    const int KONAMI_SCC_WEIGHT = 2;
    const int ASCII8_WEIGHT_HIGH = 3;
    const int ASCII8_WEIGHT_LOW = 1;
    const int ASCII16_WEIGHT = 2;

    //size_t size = file_size(filename);
    if (size > MAX_ROM_SIZE || size < MIN_ROM_SIZE) {
        printf("Invalid ROM size\n");
        return 0; // unknown mapper
    }

    FILE *file = fopen(filename, "rb");
    if (!file) {
        printf("Failed to open ROM file\n");
        return 0; // unknown mapper
    }

    // Determine the size to read (max 128KB or the actual size if smaller)
    size_t read_size = (size > MAX_ANALYSIS_SIZE) ? MAX_ANALYSIS_SIZE : size;
    //size_t read_size = size; 
    // Dynamically allocate memory for the ROM
    uint8_t *rom = (uint8_t *)malloc(read_size);
    if (!rom) {
        printf("Failed to allocate memory for ROM\n");
        fclose(file);
        return 0; // unknown mapper
    }

    fread(rom, 1, read_size, file);
    fclose(file);
    
    // Check if the ROM has the signature "AB" at 0x0000 and 0x0001
    // Those are the cases for 16KB and 32KB ROMs
    if (rom[0] == 'A' && rom[1] == 'B' && size == 16384) {
        free(rom);
        return 1;     // Plain 16KB 
    }

    if (rom[0] == 'A' && rom[1] == 'B' && size <= 32768) {

        //check if it is a normal 32KB ROM or linear0 32KB ROM
        if (rom[0x4000] == 'A' && rom[0x4001] == 'B') {
            free(rom);
            return 4; // Linear0 32KB
        }
        
        free(rom);
        return 2;     // Plain 32KB 
    }

    // Check for the "AB" header at the start
    if (rom[0] == 'A' && rom[1] == 'B') {
        // Check for the NEO8 signature at offset 16
        if (memcmp(&rom[16], neo8_signature, sizeof(neo8_signature) - 1) == 0) {
            free(rom);
            return 8; // NEO8 mapper detected
        } else if (memcmp(&rom[16], neo16_signature, sizeof(neo16_signature) - 1) == 0) {
            free(rom);
            return 9; // NEO16 mapper detected
        }
    }

    // Check if the ROM has the signature "AB" at 0x4000 and 0x4001
    // That is the case for 48KB ROMs with Linear page 0 config
    if (rom[0x4000] == 'A' && rom[0x4001] == 'B' && size <= 49152) {
        free(rom);
        return 4; // Linear0 48KB
    }

    // Heuristic analysis for larger ROMs
    if (size > 32768) {
        // Scan through the ROM data to detect patterns
        for (size_t i = 0; i < read_size - 3; i++) {
            if (rom[i] == 0x32) { // Check for 'ld (nnnn),a' instruction
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
                    // Add more cases as needed
                }
            }
        }
         
        
        
        /* Debug-only output: enabled when building with DEBUG or _DEBUG defined */
#if defined(DEBUG) || defined(_DEBUG)
        printf("DEBUG: ascii8_score = %d\n", ascii8_score);
        printf("DEBUG: ascii16_score = %d\n", ascii16_score);
        printf("DEBUG: konami_score = %d\n", konami_score);
        printf("DEBUG: konami_scc_score = %d\n\n", konami_scc_score);
#endif
        
        
        if (ascii8_score==1) ascii8_score--;

        // Determine the ROM type based on the highest weighted score
        if (konami_scc_score > konami_score && konami_scc_score > ascii8_score && konami_scc_score > ascii16_score) {
            free(rom);
            return 3; // Konami SCC
        }
        if (konami_score > konami_scc_score && konami_score > ascii8_score && konami_score > ascii16_score) {
            free(rom);
            return 7; // Konami
        }
        if (ascii8_score > konami_score && ascii8_score > konami_scc_score && ascii8_score > ascii16_score) {
            free(rom);
            return 5; // ASCII8
        }
        if (ascii16_score > konami_score && ascii16_score > konami_scc_score && ascii16_score > ascii8_score) {
            free(rom);
            return 6; // ASCII16
        }

        if (ascii16_score == konami_scc_score)
        {
            free(rom);
            return 6; // Konami SCC
        }

        free(rom);
        return 0; // unknown mapper
    }
    
    free(rom);
    return 0;
}

// Print usage information
static void print_usage(const char *prog_name) {

    printf("Usage: %s [-h] [-o <filename>] <romfile>\n", prog_name);
    printf("\n");
    printf("Options:\n");
    printf("  -h, --help Show this help message\n");
    printf("  -o <filename>, --output <filename>  Set UF2 output filename (default %s)\n", UF2FILENAME);
    printf("\n");
    printf("Mapper forcing: append tags (case-insensitive) before the ROM extension.\n");
    printf("Example: \"Knight Mare.PL-32.ROM\" forces PL-32; \"SYSTEM\" tags are ignored.\n");
}

// create_uf2_file - Create the UF2 file
// This function streams the firmware binary, a single configuration record, and the ROM payload into UF2 blocks.

void create_uf2_file(const char *rom_filename, uint32_t rom_size, uint8_t rom_type,
                     const char *rom_name, uint32_t base_offset, const char *uf2_filename) {
    if (!rom_filename || !rom_name || rom_size == 0) {
        printf("Invalid parameters provided for UF2 generation.\n");
        return;
    }

    const uint8_t *firmware_data = ___pico_loadrom_dist_loadrom_bin;
    const size_t firmware_size = sizeof(___pico_loadrom_dist_loadrom_bin);
    uint8_t config_record[CONFIG_RECORD_SIZE] = {0};

    size_t cursor = 0;
    memcpy(config_record + cursor, rom_name, MAX_FILE_NAME_LENGTH);
    cursor += MAX_FILE_NAME_LENGTH;
    memcpy(config_record + cursor, &rom_type, sizeof(rom_type));
    cursor += sizeof(rom_type);
    memcpy(config_record + cursor, &rom_size, sizeof(rom_size));
    cursor += sizeof(rom_size);
    memcpy(config_record + cursor, &base_offset, sizeof(base_offset));

    FILE *rom_file = fopen(rom_filename, "rb");
    if (!rom_file) {
        perror("Failed to open ROM file for UF2 creation");
        return;
    }

    FILE *uf2_file = fopen(uf2_filename, "wb");
    if (!uf2_file) {
        perror("Failed to create UF2 file");
        fclose(rom_file);
        return;
    }

    UF2_Block bl;
    memset(&bl, 0, sizeof(bl));
    bl.magicStart0 = UF2_MAGIC_START0;
    bl.magicStart1 = UF2_MAGIC_START1;
    bl.flags = UF2_FLAG_FAMILYID_PRESENT;
    bl.magicEnd = UF2_MAGIC_END;
    bl.targetAddr = FLASH_START;
    bl.payloadSize = 256;

    const size_t total_binary_size = firmware_size + CONFIG_RECORD_SIZE + (size_t)rom_size;
    bl.numBlocks = (uint32_t)((total_binary_size + bl.payloadSize - 1) / bl.payloadSize);
    bl.fileSize = RP2040_FAMILY_ID;

    size_t firmware_offset = 0;
    size_t config_offset = 0;
    size_t rom_bytes_written = 0;
    size_t total_written = 0;
    uint32_t block_no = 0;
    bool success = false;

    while (total_written < total_binary_size) {
        memset(bl.data, 0, sizeof(bl.data));
        size_t chunk_filled = 0;

        while (chunk_filled < bl.payloadSize && total_written < total_binary_size) {
            size_t to_copy = 0;

            if (firmware_offset < firmware_size) {
                size_t remaining = firmware_size - firmware_offset;
                to_copy = remaining < (bl.payloadSize - chunk_filled) ? remaining : (bl.payloadSize - chunk_filled);
                memcpy(bl.data + chunk_filled, firmware_data + firmware_offset, to_copy);
                firmware_offset += to_copy;
            } else if (config_offset < CONFIG_RECORD_SIZE) {
                size_t remaining = CONFIG_RECORD_SIZE - config_offset;
                to_copy = remaining < (bl.payloadSize - chunk_filled) ? remaining : (bl.payloadSize - chunk_filled);
                memcpy(bl.data + chunk_filled, config_record + config_offset, to_copy);
                config_offset += to_copy;
            } else {
                size_t remaining = (size_t)rom_size - rom_bytes_written;
                if (remaining == 0) {
                    break;
                }
                size_t request = remaining < (bl.payloadSize - chunk_filled) ? remaining : (bl.payloadSize - chunk_filled);
                size_t read_now = fread(bl.data + chunk_filled, 1, request, rom_file);
                if (read_now != request) {
                    printf("Failed to read ROM data while building UF2 file.\n");
                    goto cleanup;
                }
                rom_bytes_written += read_now;
                to_copy = read_now;
            }

            chunk_filled += to_copy;
            total_written += to_copy;
        }

        bl.blockNo = block_no++;
        if (fwrite(&bl, 1, sizeof(bl), uf2_file) != sizeof(bl)) {
            printf("Failed to write UF2 block %u.\n", block_no - 1);
            goto cleanup;
        }
        bl.targetAddr += bl.payloadSize;
    }

    success = true;

cleanup:
    fclose(rom_file);
    fclose(uf2_file);

    if (success) {
        printf("\nSuccessfully wrote %u blocks to %s.\n", block_no, uf2_filename);
    }
}

// Main function
int main(int argc, char *argv[])
{
    printf("MSX PICOVERSE 2040 LoadROM UF2 Creator %s\n", APP_VERSION);
    printf("(c) 2025 The Retro Hacker\n\n");

    const char *output_filename = UF2FILENAME;
    const char *rom_filename = NULL;

    for (int i = 1; i < argc; ++i) {
        if ((strcmp(argv[i], "-h") == 0) || (strcmp(argv[i], "--help") == 0)) {
            print_usage(argv[0]);
            return 0;
        } else if ((strcmp(argv[i], "-o") == 0) || (strcmp(argv[i], "--output") == 0)) {
            if (i + 1 >= argc) {
                printf("Option -o/--output requires a filename.\n");
                return 1;
            }
            output_filename = argv[++i];
        } else if (argv[i][0] == '-') {
            printf("Unknown option: %s\n", argv[i]);
            return 1;
        } else if (!rom_filename) {
            rom_filename = argv[i];
        } else {
            printf("Unexpected argument: %s\n", argv[i]);
            return 1;
        }
    }

    if (!rom_filename) {
        print_usage(argv[0]);
        return 1;
    }

    const char *base_name = rom_filename;
    const char *slash = strrchr(rom_filename, '/');
    const char *backslash = strrchr(rom_filename, '\\');
    if (slash || backslash) {
        const char *sep = slash;
        if (!sep || (backslash && backslash > sep)) {
            sep = backslash;
        }
        base_name = sep + 1;
    }

    const char *extension = strrchr(base_name, '.');
    if (!extension || !equals_ignore_case(extension, ".rom")) {
        printf("Invalid ROM file. Please provide a .rom file.\n");
        return 1;
    }

    bool mapper_forced_by_tag = false;
    uint8_t mapper_from_tag = 0;
    const char *name_end = extension;
    const char *last_period = NULL;
    for (const char *p = base_name; p < extension; ++p) {
        if (*p == '.') {
            last_period = p;
        }
    }

    if (last_period) {
        const char *token_start = last_period + 1;
        size_t token_length = (size_t)(extension - token_start);
        if (token_length > 0) {
            char mapper_token[32];
            if (token_length >= sizeof(mapper_token)) {
                token_length = sizeof(mapper_token) - 1;
            }
            memcpy(mapper_token, token_start, token_length);
            mapper_token[token_length] = '\0';

            uint8_t candidate = mapper_number_from_description(mapper_token);
            if (candidate == 10) {
                printf("Ignoring SYSTEM mapper tag in %s (cannot be forced)\n", base_name);
            } else if (candidate != 0) {
                mapper_forced_by_tag = true;
                mapper_from_tag = candidate;
                name_end = last_period;
            }
        }
    }

    uint32_t rom_size = file_size(rom_filename);
    if (rom_size == 0 || rom_size > MAX_ROM_SIZE) {
        printf("Failed to get the size of the ROM file or size not supported.\n");
        return 1;
    }

    uint8_t rom_type = 0;
    const char *mapper_label = NULL;
    if (mapper_forced_by_tag) {
        rom_type = mapper_from_tag;
        mapper_label = "[Forced via filename tag]";
    } else {
        rom_type = detect_rom_type(rom_filename, rom_size);
        if (rom_type == 0) {
            printf("Failed to detect the ROM type. Please check the ROM file.\n");
            return 1;
        }
        mapper_label = "[Auto-detected]";
    }
    printf("ROM Type: %s %s\n", rom_types[rom_type], mapper_label);

    char rom_name[MAX_FILE_NAME_LENGTH] = {0};
    size_t raw_length = (size_t)(name_end - base_name);
    if (raw_length >= MAX_FILE_NAME_LENGTH) {
        raw_length = MAX_FILE_NAME_LENGTH - 1;
    }
    memcpy(rom_name, base_name, raw_length);
    rom_name[raw_length] = '\0';

    uint32_t base_offset = CONFIG_RECORD_SIZE; // The ROM is placed right after the configuration record

    printf("ROM Name: %s\n", rom_name);
    printf("ROM Size: %u bytes\n", rom_size);
    printf("Pico Offset: 0x%08X\n", base_offset);
    printf("UF2 Output: %s\n", output_filename);

    create_uf2_file(rom_filename, rom_size, rom_type, rom_name, base_offset, output_filename);
    return 0;
}