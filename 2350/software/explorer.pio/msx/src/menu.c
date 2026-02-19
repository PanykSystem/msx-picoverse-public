// MSX PICOVERSE PROJECT
// (c) 2025 Cristiano Goncalves
// The Retro Hacker
//
// menu.c - MSX ROM with the menu program for the MSX PICOVERSE 2350 project
// 
// This work is licensed  under a "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
// License". https://creativecommons.org/licenses/by-nc-sa/4.0/

#include <string.h>
#include <stdint.h>
#include "menu.h"
#include "menu_state.h"
#include "menu_ui.h"
#include "menu_input.h"
#include "screen_mp3.h"
#include "screen_rom.h"

#define SEARCH_MAX_LEN 24

// Control Variables (definitions)
int currentPage;
int totalPages;
int currentIndex;
unsigned int totalFiles;
unsigned long totalSize;
ROMRecord records[FILES_PER_PAGE];

// --- Forward declarations (UI + Pico protocol) ---
static int read_search_query(char *buffer, int max_len);
static void send_query_to_pico(const char *query);
static unsigned int read_match_index(void);
static unsigned int bios_calatr(void);
int record_is_folder(const ROMRecord *record);
int record_is_mp3(const ROMRecord *record);
void trim_name_to_buffer(const char *src, char *dst, int max_len);
static void build_menu_row_text(const ROMRecord *record, const char *name_override, char *out, unsigned char width);
static void enter_directory(const char *name);
static void refresh_menu_state(void);
void msx_wait(uint16_t times_jiffy);
void delay_ms(uint16_t milliseconds);

// --- ROM record helpers ---
// read_ulong - Read a 4-byte value from the memory area
// This function will read a 4-byte value from the memory area pointed by ptr and return the value as an unsigned long
// Parameters:
//   ptr - Pointer to the memory area to read the value from
// Returns:
//   The 4-byte value as an unsigned long 
unsigned long read_ulong(const unsigned char *ptr) {
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
int isEndOfData(const unsigned char *memory) {
    for (int i = 0; i < ROM_RECORD_SIZE; i++) {
        if (memory[i] != 0xFF) {
            return 0;
        }
    }
    return 1;
}

static unsigned int read_u16_le(const unsigned char *ptr) {
    return (unsigned int)ptr[0] | ((unsigned int)ptr[1] << 8);
}

static int buffer_has_magic(const unsigned char *memory) {
    return memory[DATA_HDR_MAGIC] == (unsigned char)DATA_MAGIC_0 &&
           memory[DATA_HDR_MAGIC + 1] == (unsigned char)DATA_MAGIC_1 &&
           memory[DATA_HDR_MAGIC + 2] == (unsigned char)DATA_MAGIC_2 &&
           memory[DATA_HDR_MAGIC + 3] == (unsigned char)DATA_MAGIC_3;
}

static void clear_record(ROMRecord *record) {
    record->Name[0] = '\0';
    record->Mapper = 0;
    record->Size = 0;
    record->Offset = 0;
}

static void read_string_from_pool(char *dst, const unsigned char *base,
                                  unsigned int pool_offset, unsigned int pool_size,
                                  unsigned int string_offset) {
    dst[0] = '\0';
    if (string_offset >= pool_size) {
        return;
    }
    const unsigned char *src = base + pool_offset + string_offset;
    unsigned int max_len = MAX_FILE_NAME_LENGTH;
    unsigned int remaining = pool_size - string_offset;
    if (remaining < max_len) {
        max_len = remaining;
    }
    unsigned int i = 0;
    while (i < max_len) {
        unsigned char ch = src[i];
        if (ch == '\0') {
            break;
        }
        dst[i] = (char)ch;
        i++;
    }
    dst[i] = '\0';
}

// --- Pico control protocol (menu <-> firmware) ---

// read_total_count - Get total ROM count from Pico control registers.
static unsigned int read_total_count() {
    return (unsigned int)(*((unsigned char *)CTRL_COUNT_L)) |
           ((unsigned int)(*((unsigned char *)CTRL_COUNT_H)) << 8);
}

// bios_calatr - Return VRAM attribute/color table address for current cursor
static unsigned int bios_calatr(void) __naked
{
    __asm
    ld      iy,(#BIOS_EXPTBL-1)
    push    ix
    ld      ix,#BIOS_CALATR
    call    BIOS_CALSLT
    pop     ix
    ret
    __endasm;

}

// send_query_to_pico - Write the search query buffer into Pico memory.
static void send_query_to_pico(const char *query) {
    unsigned int i = 0;
    unsigned int len = 0;
    if (query) {
        len = strlen(query);
    }
    if (len >= CTRL_QUERY_SIZE) {
        len = CTRL_QUERY_SIZE - 1;
    }
    for (i = 0; i < CTRL_QUERY_SIZE; i++) {
        unsigned char value = 0;
        if (i < len) {
            value = (unsigned char)query[i];
        }
        Poke(CTRL_QUERY_BASE + i, value);
    }
}

// read_match_index - Read the Pico-calculated match index.
static unsigned int read_match_index(void) {
    return (unsigned int)(*((unsigned char *)CTRL_MATCH_L)) |
           ((unsigned int)(*((unsigned char *)CTRL_MATCH_H)) << 8);
}

// record_is_folder - Check if a ROM record represents a folder.
int record_is_folder(const ROMRecord *record) {
    return (record->Mapper & FOLDER_FLAG) != 0;
}

int record_is_mp3(const ROMRecord *record) {
    return (record->Mapper & MP3_FLAG) != 0;
}

static void build_menu_row_text(const ROMRecord *record, const char *name_override, char *out, unsigned char width) {
    const char *source = (record->Mapper & SOURCE_SD_FLAG) ? "SD" : "FL";
    const char *type_label = " ROM";
    char size_text[8] = "";
    const char *name = name_override ? name_override : record->Name;
    unsigned long size_kb = record->Size / 1024u;

    if (record_is_folder(record)) {
        type_label = "<DIR>";
        source = "SD";
    } else if (record_is_mp3(record)) {
        type_label = " MP3";
    }

    if (!record_is_folder(record)) {
        if (size_kb <= 1024u) {
            sprintf(size_text, "%luKB", size_kb);
        } else {
            unsigned long size_mb = (size_kb + 1023u) / 1024u;
            sprintf(size_text, "%luMB", size_mb);
        }
    }

    if (use_80_columns) {
        sprintf(out, " %-61.61s %6s %-2s %-5s", name, size_text, source, type_label);
    } else {
        sprintf(out, " %-21.21s %6s %-2s %-5s", name, size_text, source, type_label);
    }

    int len = (int)strlen(out);
    for (int i = len; i < (int)width; i++) {
        out[i] = ' ';
    }
    out[width] = '\0';
}

int build_sliding_name_window(const char *str, int startPos, char *out, unsigned char width)
{
    size_t len = strlen(str);

    if (width == 0) {
        out[0] = '\0';
        return 0;
    }

    if (len == 0) {
        for (size_t i = 0; i < width; i++) {
            out[i] = ' ';
        }
        out[width] = '\0';
        return 0;
    }

    if (len <= width) {
        memcpy(out, str, len);
        for (size_t i = len; i < width; i++) {
            out[i] = ' ';
        }
        out[width] = '\0';
        return 1;
    }

    int cycle = (int)len + 1;  /* virtual space separator between repetitions */
    int base = startPos % cycle;
    if (base < 0) {
        base += cycle;
    }

    int hasVisible = 0;
    for (size_t i = 0; i < width; i++) {
        int idx = (base + (int)i) % cycle;
        unsigned char ch = (idx < (int)len) ? (unsigned char)str[idx] : ' ';
        out[i] = (char)ch;
        if (ch != ' ') {
            hasVisible = 1;
        }
    }
    out[width] = '\0';
    return hasVisible;
}

// trim_name_to_buffer - Trim trailing spaces from a name and copy to buffer.
void trim_name_to_buffer(const char *src, char *dst, int max_len) {
    int len = 0;
    while (len < max_len && src[len] != '\0') {
        dst[len] = src[len];
        len++;
    }
    while (len > 0 && dst[len - 1] == ' ') {
        len--;
    }
    dst[len] = '\0';
}

// enter_directory - Send command to Pico to enter a directory.
static void enter_directory(const char *name) {
    char folder[CTRL_QUERY_SIZE];
    if (name) {
        trim_name_to_buffer(name, folder, CTRL_QUERY_SIZE - 1);
    } else {
        folder[0] = '\0';
    }
    if (folder[0] == '.' && folder[1] == '.' && folder[2] == '\0') {
        folder[0] = '\0';
    }
    menu_ui_print_last_line_text("Loading...");
    int blink_state = 1;
    int blink_tick = 0;
    for (unsigned int wait = 0; wait < 100; wait++) {
        if (Peek(CTRL_CMD) == 0) {
            break;
        }
        delay_ms(10);
        if (++blink_tick >= 2) {
            blink_tick = 0;
            if (blink_state) {
                menu_ui_clear_last_line();
                blink_state = 0;
            } else {
                menu_ui_print_last_line_text("Loading...");
                blink_state = 1;
            }
        }
    }
    send_query_to_pico(folder);
    Poke(CTRL_CMD, CMD_ENTER_DIR);
    for (unsigned int wait = 0; wait < 500; wait++) {
        if (Peek(CTRL_CMD) == 0) {
            break;
        }
        delay_ms(10);
        if (++blink_tick >= 2) {
            blink_tick = 0;
            if (blink_state) {
                menu_ui_clear_last_line();
                blink_state = 0;
            } else {
                menu_ui_print_last_line_text("Loading...");
                blink_state = 1;
            }
        }
    }
}

// refresh_menu_state - Refresh the menu state by reading total files and pages from Pico.
static void refresh_menu_state(void) {
    unsigned int count = 0xFFFFu;
    unsigned int last = 0xFFFFu;
    unsigned int stable = 0;
    int blink_state = 1;
    int blink_tick = 0;
    for (unsigned int wait = 0; wait < 100; wait++) {
        count = read_total_count();
        if (count != 0xFFFFu && count == last) {
            stable++;
            if (stable >= 2) {
                break;
            }
        } else {
            stable = 0;
        }
        last = count;
        delay_ms(10);
        if (++blink_tick >= 2) {
            blink_tick = 0;
            if (blink_state) {
                menu_ui_clear_last_line();
                blink_state = 0;
            } else {
                menu_ui_print_last_line_text("Loading...");
                blink_state = 1;
            }
        }
    }
    readROMData(records, &totalFiles, &totalSize);
    totalPages = (int)((totalFiles + FILES_PER_PAGE - 1) / FILES_PER_PAGE);
    if (totalPages == 0) {
        totalPages = 1;
    }
    currentPage = 1;
    if (totalFiles == 0) {
        currentIndex = 0;
    } else if ((unsigned int)currentIndex >= totalFiles) {
        currentIndex = (int)(totalFiles - 1);
    }
    menu_ui_clear_last_line();
    displayMenu();
}

// load_page_records - Request a page from Pico and load it into the local records array.
static int parse_page_buffer(unsigned int page_index) {
    unsigned char *base = (unsigned char *)MEMORY_START;
    if (!buffer_has_magic(base)) {
        return 0;
    }

    unsigned int header_size = read_u16_le(base + DATA_HDR_SIZE);
    if (header_size < DATA_HEADER_SIZE) {
        return 0;
    }

    unsigned int page_count = read_u16_le(base + DATA_HDR_PAGE_COUNT);
    unsigned int table_offset = read_u16_le(base + DATA_HDR_TABLE_OFFSET);
    unsigned int table_size = read_u16_le(base + DATA_HDR_TABLE_SIZE);
    unsigned int string_offset = read_u16_le(base + DATA_HDR_STRING_OFFSET);
    unsigned int string_size = read_u16_le(base + DATA_HDR_STRING_SIZE);

    if (table_offset + table_size > DATA_BUFFER_SIZE) {
        return 0;
    }
    if (string_offset + string_size > DATA_BUFFER_SIZE) {
        return 0;
    }

    if (page_count > FILES_PER_PAGE) {
        page_count = FILES_PER_PAGE;
    }

    for (unsigned int i = 0; i < FILES_PER_PAGE; i++) {
        clear_record(&records[i]);
    }

    for (unsigned int i = 0; i < page_count; i++) {
        unsigned int entry_offset = table_offset + (i * DATA_RECORD_TABLE_ENTRY_SIZE);
        unsigned int record_offset = read_u16_le(base + entry_offset);
        unsigned int record_length = read_u16_le(base + entry_offset + 2);

        if (record_offset + record_length > DATA_BUFFER_SIZE) {
            continue;
        }

        unsigned char mapper = 0;
        unsigned long size = 0;
        char name[MAX_FILE_NAME_LENGTH + 1];
        name[0] = '\0';

        unsigned int pos = 0;
        unsigned char *rec = base + record_offset;
        while (pos + 2 <= record_length) {
            unsigned char type = rec[pos];
            unsigned char len = rec[pos + 1];
            pos += 2;
            if (pos + len > record_length) {
                break;
            }
            switch (type) {
                case TLV_NAME_OFFSET: {
                    if (len >= 2) {
                        unsigned int name_offset = read_u16_le(rec + pos);
                        if (name_offset != 0xFFFFu) {
                            read_string_from_pool(name, base, string_offset, string_size, name_offset);
                        }
                    }
                    break;
                }
                case TLV_MAPPER:
                    if (len >= 1) {
                        mapper = rec[pos];
                    }
                    break;
                case TLV_SIZE:
                    if (len >= 4) {
                        size = read_ulong(rec + pos);
                    }
                    break;
                default:
                    break;
            }
            pos += len;
        }

        strncpy(records[i].Name, name, MAX_FILE_NAME_LENGTH);
        records[i].Name[MAX_FILE_NAME_LENGTH] = '\0';
        records[i].Mapper = mapper;
        records[i].Size = size;
        records[i].Offset = 0;
    }

    (void)page_index;
    return 1;
}

static void load_page_records(unsigned int page_index) {
    unsigned char *memory = (unsigned char *)MEMORY_START;
    unsigned int i;

    Poke(CTRL_PAGE, (unsigned char)page_index);
    for (unsigned int wait = 0; wait < 1000; wait++) {
        if (*((unsigned char *)CTRL_PAGE) == (unsigned char)page_index) {
            break;
        }
    }

    if (parse_page_buffer(page_index)) {
        return;
    }

    for (i = 0; i < FILES_PER_PAGE; i++) {
        unsigned int recordIndex = (page_index * FILES_PER_PAGE) + i;
        if (recordIndex >= totalFiles || isEndOfData(memory)) {
            break;
        }
        MemCopy(records[i].Name, memory, MAX_FILE_NAME_LENGTH);
        for (int j = strlen(records[i].Name); j < MAX_FILE_NAME_LENGTH; j++) {
            records[i].Name[j] = ' ';
        }
        records[i].Name[MAX_FILE_NAME_LENGTH] = '\0';
        records[i].Mapper = memory[MAX_FILE_NAME_LENGTH];
        records[i].Size = read_ulong(&memory[MAX_FILE_NAME_LENGTH + 1]);
        records[i].Offset = read_ulong(&memory[MAX_FILE_NAME_LENGTH + 5]);
        memory += ROM_RECORD_SIZE;
    }

    for (; i < FILES_PER_PAGE; i++) {
        clear_record(&records[i]);
    }
}

// --- ROM list loading ---

// readROMData - Read the ROM records from the memory area or Pico paging buffer.
// This function will read the ROM records from the memory area pointed by memory and store them in the records array. The function will stop reading
// when it reaches the end of the data or the maximum number of records is reached.
// Parameters:
//   records - Pointer to the array of ROM records to store the data
//   recordCount - Pointer to the variable to store the number of records read
void readROMData(ROMRecord *records, unsigned int *recordCount, unsigned long *sizeTotal) {
    unsigned char *memory = (unsigned char *)MEMORY_START;
    unsigned int count;
    unsigned long total;

    unsigned char ctrl_status = *((unsigned char *)CTRL_STATUS);
    count = read_total_count();
    paging_enabled = (ctrl_status == CTRL_MAGIC);
    if (!paging_enabled) {
        if (count > FILES_PER_PAGE && count != 0xFFFFu) {
            paging_enabled = 1;
        }
    }

    if (paging_enabled) {
        *recordCount = count;
        *sizeTotal = 0;
        load_page_records(0);
        return;
    }

    count = 0;
    total = 0;
    while (count < MAX_ROM_RECORDS && !isEndOfData(memory)) {
        if (count < FILES_PER_PAGE) {
            // Copy Name (page buffer only)
            MemCopy(records[count].Name, memory, MAX_FILE_NAME_LENGTH);
            for (int i = strlen(records[count].Name); i < MAX_FILE_NAME_LENGTH; i++) {
                records[count].Name[i] = ' ';
            }
            records[count].Name[MAX_FILE_NAME_LENGTH] = '\0';
            records[count].Mapper = memory[MAX_FILE_NAME_LENGTH];
            records[count].Size = read_ulong(&memory[MAX_FILE_NAME_LENGTH + 1]);
            records[count].Offset = read_ulong(&memory[MAX_FILE_NAME_LENGTH + 5]);
        }
        total += read_ulong(&memory[MAX_FILE_NAME_LENGTH + 1]);
        memory += ROM_RECORD_SIZE;
        count++;
    }

    *recordCount = count;
    *sizeTotal = total;
}

// --- BIOS wrappers and timing ---

static int compare_names(const char *a, const char *b) {
    return strncmp(a, b, MAX_FILE_NAME_LENGTH);
}

// sort_records - Sort the ROM records by name using a simple bubble sort.
static void sort_records(ROMRecord *list, unsigned char count) {
    for (unsigned char i = 0; i + 1 < count; i++) {
        for (unsigned char j = i + 1; j < count; j++) {
            if (compare_names(list[i].Name, list[j].Name) > 0) {
                ROMRecord temp;
                temp = list[i];
                list[i] = list[j];
                list[j] = temp;
            }
        }
    }
}

// putchar - Print a character on the screen
// This function will override the putchar function to use the MSX BIOS routine to print characters on the screen
// This is to deal with the mess we have between the Fusion-C putchar and the SDCC Z80 library putchar
// Parameters:
//  character - The character to print
// Returns:
//  The character printed
int putchar (int character)
{
    __asm
    ld      hl, #2              ;Get the return address of the function
    add     hl, sp              ;Bypass the return address of the function 
    ld      a, (hl)              ;Get the character to print

    ld      iy,(#BIOS_EXPTBL-1)  ;BIOS slot in iyh
    push    ix                     ;save ix
    ld      ix,#BIOS_CHPUT       ;address of BIOS routine
    call    BIOS_CALSLT          ;interslot call
    pop ix                      ;restore ix
    __endasm;

    return character;
}

// execute_rst00 - Execute the RST 00h instruction to reset the MSX computer
void execute_rst00() {
    __asm
        rst 0x00
    __endasm;
}

// clear_screen_0 - Clear the screen in SCREEN 0 mode
void clear_screen_0() {
    __asm
    ld      a, #0            ; Set SCREEN 0 mode
    call    BIOS_CHGMOD    ; Call BIOS CHGMOD to change screen mode to SCREEN 0
    call    BIOS_CLS       ; Call BIOS CLS function to clear the screen
    __endasm;
}

// clear_fkeys - Clear the function key strings in the BIOS area
void clear_fkeys()
{
    __asm
    ld hl, #BIOS_FNKSTR    ; Load the starting address of function key strings into HL
    ld de, #0xF880    ; Load the next address into DE for block fill
    ld bc, #160       ; Set BC to 160, the number of bytes to clear
    ld (hl), #0       ; Initialize the first byte to 0

clear_loop:
    ldi              ; Load (HL) with (DE), increment HL and DE, decrement BC
    dec hl           ; Adjust HL back to the correct position
    ld (hl), #0      ; Set the current byte to 0
    inc hl           ; Move to the next byte
    dec bc           ; Decrement the byte counter
    ld a, b          ; Check if BC has reached zero
    or c
    jp nz, clear_loop ; Repeat until all bytes are cleared
    __endasm;
}

// print_str_inverted - Print a string using the inverted character table
// This function will print a string using the inverted character table. It will apply an offset to the characters to display them correctly.
// Used to display the game names in the inverted characters.
// Parameters:
//   str - The string to print
void print_str_inverted(const char *str) 
{
    char buffer[MAX_FILE_NAME_LENGTH + 1];
    unsigned char width = name_col_width;
    size_t len = strlen(str);

    if (len > width) {
        len = width; // keep on-screen width stable
    }

    memcpy(buffer, str, len);

    for (size_t i = len; i < width; i++) {
        buffer[i] = ' ';
    }

    buffer[width] = '\0';

    for (size_t i = 0; i < width; i++) {
        int modifiedChar = buffer[i] + 96; // Apply the offset to the character
        PrintChar((unsigned char)modifiedChar); // Print the modified character
    }

}

// print_str_inverted_sliding - Print a sliding string using the inverted character table
// This function will print a sliding string using the inverted character table. It will apply an offset
int print_str_inverted_sliding(const char *str, int startPos) 
{
    size_t len = strlen(str);
    unsigned char width = name_col_width;

    if (len == 0) {
        for (size_t i = 0; i < width; i++) {
            PrintChar((unsigned char)(' ' + 96)); // Keep the highlighted row blank when no name is present
        }
        return 0;
    }

    if (len <= width) {
        print_str_inverted(str);
        return 1;
    }

    int base = startPos % (int)len;
    if (base < 0) {
        base += (int)len;
    }

    unsigned char window[MAX_FILE_NAME_LENGTH];
    int hasVisible = 0;

    for (size_t i = 0; i < width; i++) {
        unsigned char ch = (unsigned char)str[(base + (int)i) % (int)len];
        window[i] = ch;
        if (ch != ' ') {
            hasVisible = 1;
        }
    }

    if (!hasVisible) {
        return 0;
    }

    for (size_t i = 0; i < width; i++) {
        PrintChar((unsigned char)((int)window[i] + 96)); // Apply the offset to the character
    }
    return 1;
}

// joystick_direction_to_key - Convert joystick direction to key code
// This function will convert the joystick direction to the corresponding key code.
static int joystick_direction_to_key(unsigned char dir)
{
    switch (dir) {
        case 1:
            return 30;
        case 3:
            return 28;
        case 5:
            return 31;
        case 7:
            return 29;
        default:
            return 0;
    }
}

// wait_for_key_with_scroll - Wait for a key press with scrolling effect
// This function will wait for a key press and return the key code. While waiting, it
static int wait_for_key_with_scroll(const char *name, unsigned int row)
{
    volatile unsigned int *jiffyPtr = (volatile unsigned int *)JIFFY;
    unsigned int lastTick = *jiffyPtr;
    int startPos = 0;
    size_t len = strlen(name);
    int shouldScroll = (!use_80_columns && len > name_col_width);
    const unsigned int scrollDelay = 30U; // 0.5 seconds at 60 Hz
    unsigned char row_width = menu_ui_row_width();
    unsigned char highlight_width = menu_ui_highlight_width();
    char row_text[81];
    char name_window[MAX_FILE_NAME_LENGTH + 1];

    while (1)
    {
        
        // check for key press
        unsigned char peek = bios_chsns();
        if (peek) {
            return (int)bios_chget();
        }

        /*
        //joystick is too fast and not precise for menu navigation
        //need to implement some kind of delay or repeat rate control
        //also Fusion-C triggers are not working properly, need to investigate further

        // check for joystick input
        unsigned char joy1 = JoystickRead(0x01);
        unsigned char joy2 = JoystickRead(0x02);
        
        // any joystick inputs are valid
        int joyKey1 = joystick_direction_to_key(joy1); 
        if (joyKey1) {
            return joyKey1;
        }
        int joyKey2 = joystick_direction_to_key(joy2);
        if (joyKey2) {
            return joyKey2;
        }

        int joyButton1 = TriggerRead(0x01);
        int joyButton2 = TriggerRead(0x02);
        int joyButton = joyButton1 | joyButton2;
        if (joyButton) {
            return 13; // Enter key
        }
        */

        //debug
        //Locate(0, 23);
        //printf("Joy1: %02X Joy2: %02X", joy1, joy2);
        //printf("TR1: %02X TR2: %02X", joyButton1, joyButton2);

        // handle scrolling
        if (shouldScroll) {
            unsigned int now = *jiffyPtr; // read current jiffy count
            if ((unsigned int)(now - lastTick) >= scrollDelay) {
                int attempts = 0;
                int printed = 0;
                int lenInt = (int)len;

                while (attempts < lenInt && !printed) {
                    printed = build_sliding_name_window(name, startPos, name_window, name_col_width);
                    if (printed) {
                        build_menu_row_text(&records[currentIndex % FILES_PER_PAGE], name_window, row_text, row_width);
                        Locate(0, row);
                        PrintChar('>');
                        if (highlight_width > 1) {
                            menu_ui_print_str_inverted_width(row_text + 1, (unsigned char)(highlight_width - 1));
                        }
                    }
                    startPos++;
                    if (startPos >= lenInt + 1) {
                        startPos = 0;
                    }
                    attempts++;
                }
                lastTick = now;
            }
        }


    }
    
}

// mapper_description - Get the description of the mapper type
// This function will return the description of the mapper type based on the mapper number.
char* mapper_description(int number) {
    // Array of strings for the descriptions
    const char *descriptions[] = {"PL-16", "PL-32", "KonSCC", "Linear", "ASC-08", "ASC-16", "Konami","NEO-8","NEO-16","SYSTEM"};	
    number &= ~(SOURCE_SD_FLAG | FOLDER_FLAG | OVERRIDE_FLAG);
    if (number & MP3_FLAG) {
        return "MP3";
    }
    if (number <= 0 || number > 10) {
        return "Unknown";
    }
    return descriptions[number - 1];
}

// --- Menu rendering ---

// displayMenu - Display the menu on the screen
// This function will display the menu on the screen. It will print the header, the files on the current page and the footer with the page number and options.
void displayMenu() {
    if (!frame_rendered) {
        menu_ui_render_menu_frame();
    }
    if (paging_enabled && currentPage > 0) {
        load_page_records((unsigned int)(currentPage - 1));
    }

    unsigned int startIndex = (currentPage - 1) * FILES_PER_PAGE;
    unsigned int endIndex = startIndex + FILES_PER_PAGE;

    if (endIndex > totalFiles) {
        endIndex = totalFiles;
    }

    unsigned int line = 0;
    for (unsigned int idx = startIndex; idx < endIndex; idx++, line++) {
        Locate(0, 2 + line); // Position on the screen, starting at line 2
        {
            unsigned char row_width = menu_ui_row_width();
            char row_text[81];
            build_menu_row_text(&records[line], NULL, row_text, row_width);
            printf("%s", row_text);
        }
    }

    {
        unsigned char start_row = (unsigned char)(2 + line);
        menu_ui_clear_rows(start_row, 21);
    }

    menu_ui_update_footer_page();
    if (totalFiles > 0) {
        unsigned char row = (unsigned char)((currentIndex % FILES_PER_PAGE) + 2);
        unsigned char row_width = menu_ui_row_width();
        unsigned char highlight_width = menu_ui_highlight_width();
        char row_text[81];
        build_menu_row_text(&records[currentIndex % FILES_PER_PAGE], NULL, row_text, row_width);
        Locate(0, row); // Position the cursor on the selected file
        PrintChar('>'); // Print the cursor
        if (highlight_width > 1) {
            menu_ui_print_str_inverted_width(row_text + 1, (unsigned char)(highlight_width - 1)); // Print shortened row inverted
        }

    }
}

// --- Menu screens ---

// helpMenu - Display the help menu on the screen
// This function will display the help menu on the screen. It will print the help information and the keys to navigate the menu.
void helpMenu()
{
    
    Cls(); // Clear the screen
    Locate(0,0);
    menu_ui_print_title_line();
    Locate(0, 1);
    menu_ui_print_delimiter_line();
    Locate(0, 2);
    printf("Use [UP]  [DOWN] to navigate the menu");
    Locate(0, 3);
    printf("Use [LEFT] [RIGHT] to navigate  pages");
    Locate(0, 4);
    printf("Press [ENTER] or [SPACE] to load the ");
    Locate(0, 5);
    printf("  selected rom file");
    Locate(0, 6);
    printf("Press [/] to search the ROM list");
    Locate(0, 7);
    printf("Type, then [ENTER] to jump to match");
    Locate(0, 8);
    printf("Press [H] to display the help screen");
    Locate(0, 9);
    printf("Press [C] to toggle 40/80 columns");
    Locate(0, 21);
    menu_ui_print_delimiter_line();
    Locate(0, 22);
    printf("Press any key to return to the menu!");
    InputChar();
    frame_rendered = 0;
    displayMenu();
    navigateMenu();
}


// loadGame - Load the game from the flash memory
// This function will load the game from the flash memory based on the index. 
void loadGame(int index) 
{
    ROMRecord *record = &records[index % FILES_PER_PAGE];
    if (record_is_folder(record))
    {
        return;
    }
    if ((record->Mapper & ~SOURCE_SD_FLAG) != 0)
    {
        Poke(ROM_SELECT_REGISTER, index); // Set the game index (absolute)
        execute_rst00(); // Execute RST 00h to reset the MSX computer and load the game
        execute_rst00();
    }
}

// --- Input/navigation ---

// navigateMenu - Navigate the menu
// This function will navigate the menu. It will wait for the user to press a key and then act based on the key pressed. The user can navigate the menu using the arrow keys
// to move up and down the files, left and right to move between pages, enter to load the game, H to display the help screen and C to display the config screen.
// The function will update the current page and current index based on the key pressed and display the menu again.
void navigateMenu() 
{
    char key;
    char search_query[SEARCH_MAX_LEN + 1];

    while (1) 
    {
        //debug
        Locate(0, 23);
        //printf("Key: %3d", key);
        //printf("Size: %05lu/15872", totalSize/1024);
        if (totalFiles == 0) {
            key = (char)bios_chget();
            if (key == 27) {
                enter_directory(NULL);
                refresh_menu_state();
            }
            continue;
        }

        unsigned int currentRow = (currentIndex%FILES_PER_PAGE) + 2;

        key = wait_for_key_with_scroll(records[currentIndex % FILES_PER_PAGE].Name, currentRow);
        //key = KeyboardRead();
        //key = InputChar();
        char fkey = Fkeys();
        (void)fkey;

        {
            unsigned char row_width = menu_ui_row_width();
            char row_text[81];
            build_menu_row_text(&records[currentIndex % FILES_PER_PAGE], NULL, row_text, row_width);
            Locate(0, currentRow); // Position the cursor on the previously selected file
            printf("%s", row_text); // Restore full row
        }
        switch (key) 
        {
            case 30: // Up arrow
                if (currentIndex > 0) // Check if we are not at the first file
                {
                    if (currentIndex%FILES_PER_PAGE >= 0) currentIndex--; // Move to the previous file
                    if (currentIndex < ((currentPage-1) * FILES_PER_PAGE))  // Check if we need to move to the previous page
                    {
                        currentPage--; // Move to the previous page
                        displayMenu(); // Display the menu
                    }
                }
                break;
            case 31: // Down arrow
                if ((currentIndex%FILES_PER_PAGE < FILES_PER_PAGE) && currentIndex < totalFiles-1) currentIndex++; // Move to the next file
                if (currentIndex >= (currentPage * FILES_PER_PAGE)) // Check if we need to move to the next page
                {
                    currentPage++; // Move to the next page
                    displayMenu(); // Display the menu
                }
                break;
            case 28: // Right arrow
                if (currentPage < totalPages) // Check if we are not on the last page
                {
                    currentPage++; // Move to the next page
                    currentIndex = (currentPage-1) * FILES_PER_PAGE; // Move to the first file of the page
                    displayMenu(); // Display the menu
                }
                break;
            case 29: // Left arrow
                if (currentPage > 1) // Check if we are not on the first page
                {
                    currentPage--; // Move to the previous page
                    currentIndex = (currentPage-1) * FILES_PER_PAGE; // Move to the first file of the page
                    displayMenu(); // Display the menu
                }
                break;
            case 27: // ESC
                enter_directory(NULL);
                refresh_menu_state();
                break;
            case 72: // H - Help (uppercase H)
            case 104: // h - Help (lowercase h)
                // Help
                helpMenu(); // Display the help menu
                break;
            case 99: // C 
            case 67: // c 
                if (menu_ui_try_toggle_columns()) {
                    frame_rendered = 0;
                    displayMenu();
                }
                break;
            case 47: // / - Search
                if (read_search_query(search_query, SEARCH_MAX_LEN)) {
                    send_query_to_pico(search_query);
                    Poke(CTRL_CMD, CMD_FIND_FIRST);
                    for (unsigned int wait = 0; wait < 1000; wait++) {
                        if (Peek(CTRL_CMD) == 0) {
                            break;
                        }
                    }
                    unsigned int match = read_match_index();
                    if (match == 0xFFFFu || match >= totalFiles) {
                        menu_ui_print_last_line_text("Not found. Press any key.");
                        (void)bios_chget();
                        menu_ui_clear_last_line();
                    } else {
                        currentIndex = (int)match;
                        currentPage = (currentIndex / FILES_PER_PAGE) + 1;
                        menu_ui_clear_last_line();
                    }
                }
                displayMenu();
                break;
            case 13: // Enter
            case 32: // Space
                if (record_is_folder(&records[currentIndex % FILES_PER_PAGE])) {
                    char selected_name[CTRL_QUERY_SIZE];
                    trim_name_to_buffer(records[currentIndex % FILES_PER_PAGE].Name, selected_name, CTRL_QUERY_SIZE - 1);
                    if (selected_name[0] == '.' && selected_name[1] == '.' && selected_name[2] == '\0') {
                        enter_directory(NULL);
                    } else {
                        enter_directory(records[currentIndex % FILES_PER_PAGE].Name);
                    }
                    refresh_menu_state();
                } else if (record_is_mp3(&records[currentIndex % FILES_PER_PAGE])) {
                    show_mp3_screen((unsigned int)currentIndex);
                } else {
                    show_rom_screen((unsigned int)currentIndex);
                }
                break;
        }
        if (totalFiles > 0) {
            unsigned char row = (unsigned char)((currentIndex % FILES_PER_PAGE) + 2);
            unsigned char row_width = menu_ui_row_width();
            unsigned char highlight_width = menu_ui_highlight_width();
            char row_text[81];
            build_menu_row_text(&records[currentIndex % FILES_PER_PAGE], NULL, row_text, row_width);
            Locate(0, row); // Position the cursor on the selected file
            PrintChar('>'); // Print the cursor
            if (highlight_width > 1) {
                menu_ui_print_str_inverted_width(row_text + 1, (unsigned char)(highlight_width - 1)); // Print shortened row inverted
            }
            Locate(0, row); // Position the cursor on the selected file
        }
    }
}

void main() {
    // Initialize the variables
    currentPage = 1; // Start on page 1
    currentIndex = 0; // Start at the first file - index 0
    
    readROMData(records, &totalFiles, &totalSize);
    totalPages = (int)((totalFiles + FILES_PER_PAGE - 1) / FILES_PER_PAGE);

    if (totalPages == 0) {
        totalPages = 1;
    }
    menu_ui_init_text_mode();
    invert_chars(32, 126); // Invert the characters from 32 to 126
    clear_fkeys(); // Clear the function keys
    //KillKeyBuffer(); // Clear the key buffer

    // Display the menu
    menu_ui_render_menu_frame();
    displayMenu();
    // Activate navigation
    navigateMenu();
}


static int read_search_query(char *buffer, int max_len) {
    int len = 0;
    buffer[0] = '\0';
    menu_ui_print_last_line_text("Search: ");
    for (int i = 0; i < 23; i++) {
        PrintChar(' ');
    }
    Locate(8, 23);

    while (1) {
        char ch = (char)bios_chget();
        if (ch == 13) {
            buffer[len] = '\0';
            return 1;
        }
        if (ch == 27) {
            buffer[0] = '\0';
            return 0;
        }
        if (ch == 8 || ch == 127) {
            if (len > 0) {
                len--;
                buffer[len] = '\0';
                Locate(8 + len, 23);
                printf(" ");
                Locate(8 + len, 23);
            }
            continue;
        }
        if (ch >= 32 && ch <= 126) {
            if (len < max_len) {
                buffer[len++] = ch;
                buffer[len] = '\0';
                PrintChar((unsigned char)ch);
            }
        }
    }
}

#pragma disable_warning 85
void msx_wait(uint16_t times_jiffy)
{
    __asm
    ei
    WAIT:
        halt
        dec hl
        ld a,h
        or l
        jr nz, WAIT
        ret
    __endasm;
}

void delay_ms(uint16_t milliseconds)
{
    msx_wait(milliseconds / 20);
}

