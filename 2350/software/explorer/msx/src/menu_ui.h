#ifndef MENU_UI_H
#define MENU_UI_H

#include "menu_state.h"

int menu_ui_supports_80_column_mode(void);
void menu_ui_set_text_mode(int enable_80);
void menu_ui_init_text_mode(void);
int menu_ui_try_toggle_columns(void);

unsigned char menu_ui_row_width(void);
unsigned char menu_ui_highlight_width(void);

void menu_ui_clear_rows(unsigned char start_row, unsigned char end_row);
void menu_ui_render_menu_frame(void);
void menu_ui_update_footer_page(void);
void menu_ui_print_title_line(void);
void menu_ui_print_delimiter_line(void);
void menu_ui_print_footer_line(void);
void menu_ui_clear_last_line(void);
void menu_ui_print_last_line_text(const char *text);
void menu_ui_print_str_inverted_width(const char *str, unsigned char width);
void menu_ui_render_selectable_line(unsigned char row, const char *text, int selected);

#endif
