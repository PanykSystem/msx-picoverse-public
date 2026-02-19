#include <string.h>
#include <stdint.h>
#include "menu_ui.h"
#include "menu.h"
#include "menu_input.h"

__at (BIOS_HWVER) unsigned char msx_version;
__at (BIOS_LINL40) unsigned char text_columns;
__at (BIOS_RG4SAV) unsigned char vdp_reg4;

static void clear_rows_vram(unsigned char start_row, unsigned char end_row, unsigned char width) __naked
{
    (void)start_row;
    (void)end_row;
    (void)width;
    __asm
    ld      hl, #2
    add     hl, sp
    ld      e, (hl)
    inc     hl
    ld      d, (hl)
    inc     hl
    ld      c, (hl)

    ld      a, d
    sub     e
    jr      z, clear_rows_done
    ld      b, a

    ld      hl, (#BIOS_TXTNAM)
    ld      a, e
    ld      e, c
    ld      d, #0
    or      a
    jr      z, clear_rows_calc_done

clear_rows_calc_loop:
    add     hl, de
    dec     a
    jr      nz, clear_rows_calc_loop

clear_rows_calc_done:
clear_rows_loop:
    push    bc
    push    de
    push    hl
    ld      a, #32
    ld      b, #0
    ld      iy,(#BIOS_EXPTBL-1)
    push    ix
    ld      ix,#BIOS_FILVRM
    call    BIOS_CALSLT
    pop     ix
    pop     hl
    pop     de
    pop     bc
    add     hl, de
    djnz    clear_rows_loop

clear_rows_done:
    ret
    __endasm;
}

int menu_ui_supports_80_column_mode(void) {
    return msx_version >= 1;
}

void menu_ui_set_text_mode(int enable_80) {
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

void menu_ui_init_text_mode(void) {
    int enable_80 = (!MENU_FORCE_40_COLUMNS && menu_ui_supports_80_column_mode());
    menu_ui_set_text_mode(enable_80);
}

void invert_chars(unsigned char startChar, unsigned char endChar)
{
    unsigned int srcAddress, dstAddress;
    unsigned char patternByte;
    unsigned char i, c;
    unsigned int base = ((unsigned int)vdp_reg4) << 11;

    for (c = startChar; c <= endChar; c++)
    {
        srcAddress  = base + ((unsigned int)c * 8);
        dstAddress = srcAddress + (96 * 8);

        for (i = 0; i < 8; i++)
        {
            patternByte = Vpeek(srcAddress + i);
            patternByte = ~patternByte;
            Vpoke(dstAddress  + i, patternByte);
        }
    }
}

int menu_ui_try_toggle_columns(void) {
    int target_80 = !use_80_columns;
    if (target_80) {
        if (MENU_FORCE_40_COLUMNS) {
            menu_ui_print_last_line_text("80-column mode disabled. Press key.");
            (void)bios_chget();
            menu_ui_clear_last_line();
            return 0;
        }
        if (!menu_ui_supports_80_column_mode()) {
            menu_ui_print_last_line_text("MSX lacks 80-column mode. Press key.");
            (void)bios_chget();
            menu_ui_clear_last_line();
            return 0;
        }
    }
    menu_ui_set_text_mode(target_80);
    invert_chars(32, 126);
    return 1;
}

unsigned char menu_ui_row_width(void) {
    return (unsigned char)(use_80_columns ? 80 : SCREEN_WIDTH);
}

unsigned char menu_ui_highlight_width(void) {
    unsigned char row_width = menu_ui_row_width();
    return (unsigned char)(row_width > 2 ? row_width - 2 : row_width);
}

void menu_ui_clear_rows(unsigned char start_row, unsigned char end_row) {
    if (start_row >= end_row) {
        return;
    }
    clear_rows_vram(start_row, end_row, menu_ui_row_width());
}

void menu_ui_print_title_line(void) {
    if (use_80_columns) {
        printf("MSX PICOVERSE 2350%44s[EXPLORER %s]", "", EXPLORER_VERSION);
    } else {
        printf("MSX PICOVERSE 2350%4s[EXPLORER %s]", "", EXPLORER_VERSION);
    }
}

void menu_ui_print_delimiter_line(void) {
    unsigned char width = use_80_columns ? 78 : 38;
    for (unsigned char i = 0; i < width; i++) {
        PrintChar('-');
    }
}

void menu_ui_print_footer_line(void) {
    if (use_80_columns) {
        printf("Page: %02d/%02d%35s[/ - Find] [H - Help] [ESC - UP]", currentPage, totalPages, "");
    } else {
        printf("Page: %02d/%02d%1s[/-Find] [H-Help] [ESC-UP]", currentPage, totalPages, "");
    }
}

void menu_ui_render_menu_frame(void) {
    Locate(0, 0);
    menu_ui_print_title_line();
    Locate(0, 1);
    menu_ui_print_delimiter_line();
    Locate(0, 21);
    menu_ui_print_delimiter_line();
    Locate(0, 22);
    menu_ui_print_footer_line();
    frame_rendered = 1;
}

void menu_ui_update_footer_page(void) {
    Locate(0, 22);
    menu_ui_print_footer_line();
}

void menu_ui_clear_last_line(void) {
    Locate(0, 23);
    unsigned char width = menu_ui_row_width();
    for (unsigned char i = 0; i < (unsigned char)(width - 1); i++) {
        PrintChar(' ');
    }
    Locate(0, 23);
}

void menu_ui_print_last_line_text(const char *text) {
    Locate(0, 23);
    unsigned char col = 0;
    unsigned char width = menu_ui_row_width();
    while (*text && col < (unsigned char)(width - 1)) {
        PrintChar((unsigned char)*text++);
        col++;
    }
}

void menu_ui_print_str_inverted_width(const char *str, unsigned char width)
{
    if (width == 0) {
        return;
    }
    unsigned char capped_width = width > 80 ? 80 : width;
    char buffer[81];
    size_t len = strlen(str);

    if (len > capped_width) {
        len = capped_width;
    }

    memcpy(buffer, str, len);

    for (size_t i = len; i < capped_width; i++) {
        buffer[i] = ' ';
    }

    buffer[capped_width] = '\0';

    for (size_t i = 0; i < capped_width; i++) {
        int modifiedChar = buffer[i] + 96;
        PrintChar((unsigned char)modifiedChar);
    }

}

void menu_ui_render_selectable_line(unsigned char row, const char *text, int selected) {
    unsigned char width = menu_ui_row_width();
    unsigned char content_width = width > 3 ? (unsigned char)(width - 3) : 0;
    char buffer[81];
    unsigned char len = 0;

    if (content_width > 80) {
        content_width = 80;
    }

    if (text) {
        len = (unsigned char)strlen(text);
        if (len > content_width) {
            len = content_width;
        }
        memcpy(buffer, text, len);
    }
    for (unsigned char i = len; i < content_width; i++) {
        buffer[i] = ' ';
    }
    buffer[content_width] = '\0';

    Locate(0, row);
    if (selected) {
        PrintChar('>');
        if (content_width > 0) {
            menu_ui_print_str_inverted_width(buffer, content_width);
        }
    } else {
        PrintChar(' ');
        for (unsigned char i = 0; i < content_width; i++) {
            PrintChar((unsigned char)buffer[i]);
        }
    }
}
