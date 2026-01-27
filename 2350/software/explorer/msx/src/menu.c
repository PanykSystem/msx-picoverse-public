// MSX PICOVERSE PROJECT
// (c) 2025 Cristiano Goncalves
// The Retro Hacker
//
// menu.c - MSX ROM with the menu program for the MSX PICOVERSE 2350 project
// 
// This work is licensed  under a "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
// License". https://creativecommons.org/licenses/by-nc-sa/4.0/

#include <string.h>
#include "menu.h"

#ifndef EXPLORER_VERSION
#define EXPLORER_VERSION "v1.00"
#endif

static int paging_enabled = 0;
#define SEARCH_MAX_LEN 24
#define SCREEN_WIDTH 40
static int use_80_columns = 0;
static unsigned char name_col_width = NAME_COL_WIDTH;

__at (BIOS_HWVER) unsigned char msx_version;
__at (BIOS_LINL40) unsigned char text_columns;
__at (BIOS_TXTCOL) unsigned char text_color;
__at (BIOS_T32ATR) unsigned int text2_attr_base;
__at (BIOS_T32COL) unsigned int text2_color_base;
__at (BIOS_RG4SAV) unsigned char vdp_reg4;
__at (BIOS_RG3SAV) unsigned char vdp_reg3;
__at (BIOS_RG7SAV) unsigned char vdp_reg7;
__at (BIOS_CSRX) unsigned char cursor_x;
__at (BIOS_CSRY) unsigned char cursor_y;

// --- Forward declarations (UI + Pico protocol) ---
static int read_search_query(char *buffer, int max_len);
static void clear_last_line(void);
static void print_last_line_text(const char *text);
static void send_query_to_pico(const char *query);
static unsigned int read_match_index(void);
static int supports_80_column_mode(void);
static void set_text_mode(int enable_80);
static void init_text_mode(void);
static void apply_text2_attr(unsigned char col, unsigned char row, unsigned char width, int inverted);
static unsigned int bios_calatr(void);
static void print_delimiter_line(void);
static void print_title_line(void);
static void print_footer_line(void);

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

// --- Pico control protocol (menu <-> firmware) ---

// read_total_count - Get total ROM count from Pico control registers.
static unsigned int read_total_count() {
    return (unsigned int)(*((unsigned char *)CTRL_COUNT_L)) |
           ((unsigned int)(*((unsigned char *)CTRL_COUNT_H)) << 8);
}

static int supports_80_column_mode(void) {
    return msx_version >= 1;
}

static void set_text_mode(int enable_80) {
    if (enable_80) {
        text_columns = 80;
        use_80_columns = 1;
        name_col_width = MAX_FILE_NAME_LENGTH;
    } else {
        text_columns = 40;
        use_80_columns = 0;
        name_col_width = NAME_COL_WIDTH;
    }
    __asm
    ld     iy,(#BIOS_EXPTBL-1)
    push   ix
    ld     ix,#BIOS_INITXT
    call   BIOS_CALSLT
    pop    ix
    __endasm;
}

static void init_text_mode(void) {
    int enable_80 = (!MENU_FORCE_40_COLUMNS && supports_80_column_mode());
    set_text_mode(enable_80);
}

static void apply_text2_attr(unsigned char col, unsigned char row, unsigned char width, int inverted) {
    if (!use_80_columns) {
        return;
    }
    unsigned char normal = text_color ? text_color : vdp_reg7;
    unsigned char inv = (unsigned char)((normal << 4) | (normal >> 4));
    unsigned char attr = inverted ? inv : normal;
    unsigned int addr = 0;
    unsigned char saved_x = cursor_x;
    unsigned char saved_y = cursor_y;
    cursor_x = col;
    cursor_y = row;
    addr = bios_calatr();
    cursor_x = saved_x;
    cursor_y = saved_y;

    if (addr == 0 || addr == 0xFFFF) {
        unsigned int base_color = text2_color_base;
        if (base_color == 0 || base_color == 0xFFFF) {
            base_color = ((unsigned int)vdp_reg3) << 6;
        }
        if (base_color == 0 || base_color == 0xFFFF) {
            base_color = 0x2000;
        }
        addr = base_color + ((unsigned int)row * 80u) + col;
    }
    for (unsigned char i = 0; i < width; i++) {
        Vpoke(addr + i, attr);
    }
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

// load_page_records - Request a page from Pico and load it into the local records array.
static void load_page_records(unsigned int page_index) {
    unsigned char *memory = (unsigned char *)MEMORY_START;
    unsigned int i;

    Poke(CTRL_PAGE, (unsigned char)page_index);
    for (unsigned int wait = 0; wait < 1000; wait++) {
        if (*((unsigned char *)CTRL_PAGE) == (unsigned char)page_index) {
            break;
        }
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
        records[i].Name[0] = '\0';
        records[i].Mapper = 0;
        records[i].Size = 0;
        records[i].Offset = 0;
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
        if (count > FILES_PER_PAGE && count <= MAX_ROM_RECORDS && count != 0xFFFFu) {
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

void clear_screen_0() {
    __asm
    ld      a, #0            ; Set SCREEN 0 mode
    call    BIOS_CHGMOD    ; Call BIOS CHGMOD to change screen mode to SCREEN 0
    call    BIOS_CLS       ; Call BIOS CLS function to clear the screen
    __endasm;
}

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

// invert_chars - Invert the characters in the character table
// This function will invert the characters from startChar to endChar in the character table. We use it to copy and invert the characters from the
// normal character table area to the inverted character table area. This is to display the game names in the inverted character table.
// Parameters:
//   startChar - The first character to invert
//   endChar - The last character to invert
void invert_chars(unsigned char startChar, unsigned char endChar)
{
    unsigned int srcAddress, dstAddress;
    unsigned char patternByte;
    unsigned char i, c;
    unsigned int base = ((unsigned int)vdp_reg4) << 11; // reg4 * 0x800

    for (c = startChar; c <= endChar; c++)
    {
        // Each character has 8 bytes in the pattern table.
        srcAddress  = base + ((unsigned int)c * 8);
        // Calculate destination address (shift by +95 bytes)
        dstAddress = srcAddress + (96*8);

        // Flip all 8 bytes that define this character.
        for (i = 0; i < 8; i++)
        {
            patternByte = Vpeek(srcAddress + i);
            patternByte = ~patternByte;           // CPL (bitwise NOT)
            Vpoke(dstAddress  + i, patternByte);
        }
    }
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

// bios_chsns - Check if a key has been pressed
// This function will check if a key has been pressed using the MSX BIOS routine CHSNS
static unsigned char bios_chsns(void) __naked
{
    __asm
    ld      iy,(#BIOS_EXPTBL-1)
    push    ix
    ld      ix,#BIOS_CHSNS
    call    BIOS_CALSLT
    pop     ix
    jr      z, bios_chsns_no_key
    ld      l,a
    ld      h,#0
    ret

bios_chsns_no_key:
    xor     a
    ld      l,a
    ld      h,a
    ret
    __endasm;
}

// bios_chget - Get a character from the keyboard buffer
// This function will get a character from the keyboard buffer using the MSX BIOS routine.
static unsigned char bios_chget(void) __naked
{
    __asm
    ld      iy,(#BIOS_EXPTBL-1)
    push    ix
    ld      ix,#BIOS_CHGET
    call    BIOS_CALSLT
    pop     ix
    ld      l,a
    ld      h,#0
    ret
    __endasm;
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
                    Locate(1, row);
                    printed = print_str_inverted_sliding(name, startPos);
                    startPos++;
                    if (startPos >= lenInt) {
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
    number &= ~SOURCE_SD_FLAG;
    if (number <= 0 || number > 10) {
        return "Unknown";
    }
    return descriptions[number - 1];
}

// --- Menu rendering ---

// displayMenu - Display the menu on the screen
// This function will display the menu on the screen. It will print the header, the files on the current page and the footer with the page number and options.
void displayMenu() {
    Locate(0, 0);
    print_title_line();
    Locate(0, 1);
    print_delimiter_line();
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
        const char *source = (records[line].Mapper & SOURCE_SD_FLAG) ? "SD" : "FL";
        if (use_80_columns) {
            printf(" %-62.62s %04lu %-2s %-7s", records[line].Name, records[line].Size/1024, source, mapper_description(records[line].Mapper));
        } else {
            printf(" %-22.22s %04lu %-2s %-7s", records[line].Name, records[line].Size/1024, source, mapper_description(records[line].Mapper));
        }
    }

    for (; line < FILES_PER_PAGE; line++) {
        Locate(0, 2 + line);
        printf("                                 "); // clear unused lines to avoid stale entries
    }
    // footer
    Locate(0, 21);
    print_delimiter_line();
    Locate(0, 22);
    print_footer_line();
    if (totalFiles > 0) {
        Locate(0, (currentIndex % FILES_PER_PAGE) + 2); // Position the cursor on the selected file
        printf(">"); // Print the cursor
        print_str_inverted(records[currentIndex % FILES_PER_PAGE].Name); // Print the selected file name inverted

    }
}

// --- Menu screens ---

// helpMenu - Display the help menu on the screen
// This function will display the help menu on the screen. It will print the help information and the keys to navigate the menu.
void helpMenu()
{
    
    Cls(); // Clear the screen
    Locate(0,0);
    print_title_line();
    Locate(0, 1);
    print_delimiter_line();
    Locate(0, 2);
    printf("Use [UP]  [DOWN] to navigate the menu");
    Locate(0, 3);
    printf("Use [LEFT] [RIGHT] to navigate  pages");
    Locate(0, 4);
    printf("Press [ENTER] or [SPACE] to load the ");
    Locate(0, 5);
    printf("  selected rom file");
    Locate(0, 6);
    printf("Press [F] to search the ROM list");
    Locate(0, 7);
    printf("Type, then [ENTER] to jump to match");
    Locate(0, 8);
    printf("Press [H] to display the help screen");
    Locate(0, 9);
    printf("Press [C] to toggle columns");
    Locate(0, 21);
    print_delimiter_line();
    Locate(0, 22);
    printf("Press any key to return to the menu!");
    InputChar();
    displayMenu();
    navigateMenu();
}

// loadGame - Load the game from the flash memory
// This function will load the game from the flash memory based on the index. 
void loadGame(int index) 
{
    ROMRecord *record = &records[index % FILES_PER_PAGE];
    if ((record->Mapper & ~SOURCE_SD_FLAG) != 0)
    {
        Poke(ROM_SELECT_REGISTER, index); // Set the game index (absolute)
        execute_rst00(); // Execute RST 00h to reset the MSX computer and load the game
        execute_rst00();
    }
}

static void toggle_column_mode(void) {
    int target_80 = !use_80_columns;
    if (target_80) {
        if (MENU_FORCE_40_COLUMNS) {
            print_last_line_text("80-column mode disabled. Press key.");
            (void)bios_chget();
            clear_last_line();
            return;
        }
        if (!supports_80_column_mode()) {
            print_last_line_text("MSX lacks 80-column mode. Press key.");
            (void)bios_chget();
            clear_last_line();
            return;
        }
    }
    set_text_mode(target_80);
    invert_chars(32, 126);
    displayMenu();
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
        //debug
        //Locate(20, 23);
        //printf("Memory Mapper: Off");
        //printf("CPage: %2d Index: %2d", currentPage, currentIndex);
        unsigned int currentRow = (currentIndex%FILES_PER_PAGE) + 2;

        key = wait_for_key_with_scroll(records[currentIndex % FILES_PER_PAGE].Name, currentRow);
        //key = KeyboardRead();
        //key = InputChar();
        char fkey = Fkeys();
        (void)fkey;

        Locate(0, currentRow); // Position the cursor on the previously selected file
        printf(" "); // Clear the cursor
        if (use_80_columns) {
            printf("%-60.60s", records[currentIndex % FILES_PER_PAGE].Name);
        } else {
            printf("%-22.22s", records[currentIndex % FILES_PER_PAGE].Name); // Print only the first name column width characters
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
                // load Nextor
                //loadGame(0); // Load the Nextor ROM
                break;
            case 72: // H - Help (uppercase H)
            case 104: // h - Help (lowercase h)
                // Help
                helpMenu(); // Display the help menu
                break;
            case 99: // C 
            case 67: // c 
                toggle_column_mode();
                break;
            case 70: // F - Search (uppercase F)
            case 102: // f - Search (lowercase f)
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
                        print_last_line_text("Not found. Press any key.");
                        (void)bios_chget();
                        clear_last_line();
                    } else {
                        currentIndex = (int)match;
                        currentPage = (currentIndex / FILES_PER_PAGE) + 1;
                        clear_last_line();
                    }
                }
                displayMenu();
                break;
            case 13: // Enter
            case 32: // Space
                // Load the game
                loadGame(currentIndex); // Load the selected game
                break;
        }
        Locate(0, (currentIndex%FILES_PER_PAGE) + 2); // Position the cursor on the selected file
        printf(">"); // Print the cursor
        print_str_inverted(records[currentIndex % FILES_PER_PAGE].Name); // Print the selected file name
        Locate(0, (currentIndex%FILES_PER_PAGE) + 2); // Position the cursor on the selected file
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
    init_text_mode();
    invert_chars(32, 126); // Invert the characters from 32 to 126
    clear_fkeys(); // Clear the function keys
    //KillKeyBuffer(); // Clear the key buffer

    // Display the menu
    displayMenu();
    // Activate navigation
    navigateMenu();
}


static int read_search_query(char *buffer, int max_len) {
    int len = 0;
    buffer[0] = '\0';
    print_last_line_text("Search: ");
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

static void clear_last_line(void) {
    Locate(0, 23);
    for (int i = 0; i < 28; i++) {
        PrintChar(' ');
    }
    Locate(0, 23);
}

static void print_last_line_text(const char *text) {
    Locate(0, 23);
    int col = 0;
    while (*text && col < (SCREEN_WIDTH - 1)) {
        PrintChar((unsigned char)*text++);
        col++;
    }
}

static void print_delimiter_line(void) {
    unsigned char width = use_80_columns ? 78 : 38;
    for (unsigned char i = 0; i < width; i++) {
        PrintChar('-');
    }
}

static void print_title_line(void) {
    if (use_80_columns) {
        printf("MSX PICOVERSE 2350%44s[EXPLORER %s]", "", EXPLORER_VERSION);
    } else {
        printf("MSX PICOVERSE 2350%4s[EXPLORER %s]", "", EXPLORER_VERSION);
    }
}

static void print_footer_line(void) {
    if (use_80_columns) {
        printf("Page: %02d/%02d%46s[F - Find] [H - Help]", currentPage, totalPages, "");
    } else {
        printf("Page: %02d/%02d%6s[F - Find] [H - Help]", currentPage, totalPages, "");
    }
}