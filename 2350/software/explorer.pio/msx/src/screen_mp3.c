#include <string.h>
#include "menu.h"
#include "menu_state.h"
#include "menu_ui.h"
#include "menu_input.h"
#include "screen_mp3.h"

static void render_mp3_screen(const ROMRecord *record, const char *file_name);
static void render_mp3_name_line(const char *text, int is_80);
static void render_mp3_visualizer(unsigned int frame, int is_80);
static void render_mp3_clear(int is_80);
static void render_mp3_status_line(unsigned char status, const char *elapsed);
static void build_mp3_action_text(int is_playing, char *out, size_t out_size);
static void build_mp3_mute_text(int is_muted, char *out, size_t out_size);
static void build_mp3_visualizer_text(int enabled, char *out, size_t out_size);
static void build_mp3_mode_text(int mode, char *out, size_t out_size);
static void format_time(unsigned int seconds, char *out);
static unsigned char mp3_read_status(void);
static unsigned int mp3_read_elapsed(void);
static void mp3_send_select(unsigned int index);
static void mp3_send_cmd(unsigned char cmd);
static void mp3_send_mode(unsigned char mode);
static unsigned int mp3_read_playing_index(void);
static void mp3_read_now_playing_name(char *out, int max_len);
static unsigned long mp3_read_now_playing_size(void);

static void mp3_send_select(unsigned int index) {
    Poke(MP3_CTRL_INDEX_L, (unsigned char)(index & 0xFFu));
    Poke(MP3_CTRL_INDEX_H, (unsigned char)((index >> 8) & 0xFFu));
    Poke(MP3_CTRL_CMD, MP3_CMD_SELECT);
}

static void mp3_send_cmd(unsigned char cmd) {
    Poke(MP3_CTRL_CMD, cmd);
}

static unsigned char mp3_read_status(void) {
    return *((unsigned char *)MP3_CTRL_STATUS);
}

static unsigned int mp3_read_elapsed(void) {
    return (unsigned int)(*((unsigned char *)MP3_CTRL_ELAPSED_L)) |
           ((unsigned int)(*((unsigned char *)MP3_CTRL_ELAPSED_H)) << 8);
}

static void mp3_send_mode(unsigned char mode) {
    Poke(MP3_CTRL_MODE, mode);
}

static unsigned int mp3_read_playing_index(void) {
    return (unsigned int)(*((unsigned char *)MP3_CTRL_INDEX_L)) |
           ((unsigned int)(*((unsigned char *)MP3_CTRL_INDEX_H)) << 8);
}

static void mp3_read_now_playing_name(char *out, int max_len) {
    unsigned char *src = (unsigned char *)MP3_NOW_PLAYING_BASE;
    int i;
    int end = max_len < MP3_NOW_PLAYING_NAME_LEN ? max_len : MP3_NOW_PLAYING_NAME_LEN;
    for (i = 0; i < end; i++) {
        out[i] = (char)src[i];
    }
    out[i] = '\0';
    /* Trim trailing spaces */
    while (i > 0 && out[i - 1] == ' ') {
        i--;
        out[i] = '\0';
    }
}

static unsigned long mp3_read_now_playing_size(void) {
    unsigned char *src = (unsigned char *)(MP3_NOW_PLAYING_BASE + MP3_NOW_PLAYING_SIZE_OFFSET);
    return (unsigned long)src[0] |
           ((unsigned long)src[1] << 8) |
           ((unsigned long)src[2] << 16) |
           ((unsigned long)src[3] << 24);
}

static void format_time(unsigned int seconds, char *out) {
    unsigned int m = seconds / 60u;
    unsigned int s = seconds % 60u;
    sprintf(out, "%02u:%02u", m, s);
}

static void build_mp3_action_text(int is_playing, char *out, size_t out_size) {
    const char *label = is_playing ? "Action: Stop" : "Action: Play";
    strncpy(out, label, out_size - 1);
    out[out_size - 1] = '\0';
}

static void build_mp3_mute_text(int is_muted, char *out, size_t out_size) {
    const char *label = is_muted ? "Mute: On" : "Mute: Off";
    strncpy(out, label, out_size - 1);
    out[out_size - 1] = '\0';
}

static void build_mp3_visualizer_text(int enabled, char *out, size_t out_size) {
    const char *label = enabled ? "Visualizer: On" : "Visualizer: Off";
    strncpy(out, label, out_size - 1);
    out[out_size - 1] = '\0';
}

static void build_mp3_mode_text(int mode, char *out, size_t out_size) {
    const char *label;
    switch (mode) {
        case MP3_PLAY_MODE_ALL:    label = "Mode: All"; break;
        case MP3_PLAY_MODE_RANDOM: label = "Mode: Random"; break;
        default:                   label = "Mode: Single"; break;
    }
    strncpy(out, label, out_size - 1);
    out[out_size - 1] = '\0';
}

#define MP3_NAME_PREFIX "    MP3: "
#define MP3_NAME_PREFIX_LEN 9
#define MP3_NAME_WIDTH_40 (SCREEN_WIDTH - 2 - MP3_NAME_PREFIX_LEN)
#define MP3_NAME_WIDTH_80 (80 - MP3_NAME_PREFIX_LEN)

static void render_mp3_name_line(const char *text, int is_80) {
    unsigned char width = is_80 ? MP3_NAME_WIDTH_80 : MP3_NAME_WIDTH_40;
    Locate(0, 3);
    printf(MP3_NAME_PREFIX);
    unsigned char len = (unsigned char)strlen(text);
    if (len > width) len = width;
    for (unsigned char i = 0; i < len; i++) {
        PrintChar((unsigned char)text[i]);
    }
    for (unsigned char i = len; i < width; i++) {
        PrintChar(' ');
    }
}

static void render_mp3_screen(const ROMRecord *record, const char *file_name) {
    Locate(0, 0);
    menu_ui_print_title_line();
    Locate(0, 1);
    menu_ui_print_delimiter_line();

    menu_ui_clear_rows(2, 21);

    {
        const char *source = (record->Mapper & SOURCE_SD_FLAG) ? "SD" : "FL";
        unsigned long size_kb = record->Size / 1024u;

        if (use_80_columns) {
            render_mp3_name_line(file_name, 1);
        } else {
            /* 40-col: initial static display, scrolling handled in main loop */
            char window[SCREEN_WIDTH + 1];
            unsigned char w = MP3_NAME_WIDTH_40;
            unsigned char len = (unsigned char)strlen(file_name);
            if (len <= w) {
                render_mp3_name_line(file_name, 0);
            } else {
                /* show the first window */
                memcpy(window, file_name, w);
                window[w] = '\0';
                render_mp3_name_line(window, 0);
            }
        }

        Locate(0, 4);
        if (use_80_columns) {
            printf("   Size: %lu KB", size_kb);
        } else {
            printf("   Size: %luK", size_kb);
        }

        Locate(0, 5);
        printf(" Source: %s", source);

        Locate(0, 6);
        printf(" ");
    }

    Locate(0, 21);
    menu_ui_print_delimiter_line();
}

static void render_mp3_status_line(unsigned char status, const char *elapsed) {
    char line[81];
    const char *state = (status & MP3_STATUS_PLAYING) ? "PLAY" : "STOP";
    const char *label_80 = "[ENTER-TOGGLE] [ESC-BACK]";
    const char *label_40 = "[ENTR-TOGGLE] [ESC-BACK]";
    if (use_80_columns) {
        sprintf(line, "STATUS: %-4s  EL %s  %s%s", state, elapsed,
            (status & MP3_STATUS_MUTED) ? "MUTE" : "", (status & MP3_STATUS_ERROR) ? " ERR" : "");
    } else {
        sprintf(line, "%-4s EL %s %s%s", state, elapsed,
            (status & MP3_STATUS_MUTED) ? "M" : "", (status & MP3_STATUS_ERROR) ? "E" : "");
    }
    unsigned char row_width = menu_ui_row_width();
    unsigned char len = (unsigned char)strlen(line);
    if (len > row_width) {
        len = row_width;
    }
    const char *label = use_80_columns ? label_80 : label_40;
    unsigned char label_len = (unsigned char)strlen(label);
    unsigned char label_col = 0;
    if (label_len + 2 < row_width) {
        label_col = (unsigned char)(row_width - label_len - 2);
    }
    if (len > label_col) {
        len = label_col;
    }
    Locate(0, 22);
    for (unsigned char i = 0; i < len; i++) {
        PrintChar((unsigned char)line[i]);
    }
    for (unsigned char i = len; i < label_col; i++) {
        PrintChar(' ');
    }
    Locate(label_col, 22);
    printf("%s", label);
}

static void render_mp3_visualizer(unsigned int frame, int is_80) {
    static const unsigned char pattern[] = {
        0, 1, 3, 5, 6, 5, 3, 1,
        0, 2, 4, 6, 4, 2, 1, 3,
        5, 6, 4, 2
    };
    static unsigned int lfsr = 0xACE1u;
    const unsigned int pattern_len = (unsigned int)(sizeof(pattern) / sizeof(pattern[0]));
    const int bar_count = 10;
    const int bar_width = 2;
    const int bar_gap = 1;
    const int vis_rows = 6;
    const int top_row = 14;
    const int total_width = (bar_count * bar_width) + ((bar_count - 1) * bar_gap);
    const int screen_width = is_80 ? 80 : SCREEN_WIDTH;
    int start_col = (screen_width - total_width) / 2;
    if (start_col < 0) {
        start_col = 0;
    }

    unsigned int phase = frame % pattern_len;
    for (int row = 0; row < vis_rows; row++) {
        Locate(start_col, (unsigned char)(top_row + row));
        for (int i = 0; i < bar_count; i++) {
            lfsr = (lfsr >> 1) ^ (0xB400u & (-(int)(lfsr & 1u)));
            unsigned int jitter = (unsigned int)(lfsr & 0x0Fu);
            unsigned int idx = (phase + (unsigned int)(i * 3) + jitter) % pattern_len;
            unsigned char height = pattern[idx];
            unsigned char level = (unsigned char)(vis_rows - row);
            char ch = (height >= level) ? '#' : ' ';
            PrintChar((unsigned char)ch);
            PrintChar((unsigned char)ch);
            if (i + 1 < bar_count) {
                PrintChar(' ');
            }
        }
    }
}

static void render_mp3_clear(int is_80) {
    const int vis_rows = 6;
    const int top_row = 14;
    (void)is_80;
    menu_ui_clear_rows((unsigned char)top_row, (unsigned char)(top_row + vis_rows));
}

void show_mp3_screen(unsigned int index) {
    ROMRecord *record = &records[index % FILES_PER_PAGE];
    char file_name[MAX_FILE_NAME_LENGTH + 1];
    char elapsed_str[8];
    char action_text[32];
    char mute_text[24];
    char visualizer_text[32];
    char mode_text[24];
    unsigned char selected_sent = 0;
    unsigned int viz_frame = 0;
    int selection = 0;
    int visualizer_enabled = 1;
    int mute_enabled = 0;
    int action_playing = 0;
    int play_mode = MP3_PLAY_MODE_SINGLE;
    int scroll_pos = 0;
    unsigned int last_playing_index = mp3_read_playing_index();
    unsigned long current_size_kb = record->Size / 1024u;
    char name_window[MAX_FILE_NAME_LENGTH + 1];

    trim_name_to_buffer(record->Name, file_name, MAX_FILE_NAME_LENGTH);
    render_mp3_screen(record, file_name);

    unsigned char status = mp3_read_status();
    mute_enabled = (status & MP3_STATUS_MUTED) != 0;
    build_mp3_action_text(action_playing, action_text, sizeof(action_text));
    build_mp3_mute_text(mute_enabled, mute_text, sizeof(mute_text));
    build_mp3_visualizer_text(visualizer_enabled, visualizer_text, sizeof(visualizer_text));
    build_mp3_mode_text(play_mode, mode_text, sizeof(mode_text));
    menu_ui_render_selectable_line(7, action_text, selection == 0);
    menu_ui_render_selectable_line(8, mute_text, selection == 1);
    menu_ui_render_selectable_line(9, mode_text, selection == 2);
    menu_ui_render_selectable_line(10, visualizer_text, selection == 3);

    mp3_send_mode((unsigned char)play_mode);

    volatile unsigned int *jiffyPtr = (volatile unsigned int *)JIFFY;
    unsigned int lastTick = *jiffyPtr;

    while (1) {
        if (bios_chsns()) {
            char key = (char)bios_chget();
            if (key == 27) {
                if (mp3_read_status() & MP3_STATUS_PLAYING) {
                    mp3_send_cmd(MP3_CMD_STOP);
                }
                break;
            }
            if (key == 13) {
                if (selection == 0) {
                    if (!selected_sent) {
                        mp3_send_select(index);
                        selected_sent = 1;
                        delay_ms(10);
                        mp3_send_cmd(MP3_CMD_PLAY);
                        action_playing = 1;
                    } else {
                        if (action_playing) {
                            mp3_send_cmd(MP3_CMD_STOP);
                            action_playing = 0;
                        } else {
                            mp3_send_cmd(MP3_CMD_PLAY);
                            action_playing = 1;
                        }
                    }
                } else if (selection == 1) {
                    if (!selected_sent) {
                        mp3_send_select(index);
                        selected_sent = 1;
                    }
                    mp3_send_cmd(MP3_CMD_TOGGLE_MUTE);
                } else if (selection == 2) {
                    play_mode = (play_mode + 1) % 3;
                    mp3_send_mode((unsigned char)play_mode);
                } else if (selection == 3) {
                    if (!selected_sent) {
                        mp3_send_select(index);
                        selected_sent = 1;
                    }
                    visualizer_enabled = !visualizer_enabled;
                    if (!visualizer_enabled) {
                        render_mp3_clear(use_80_columns);
                    }
                }
            } else if (key == 30 || key == 31) {
                int delta = (key == 30) ? -1 : 1;
                int next = selection + delta;
                if (next < 0) {
                    next = 0;
                } else if (next > 3) {
                    next = 3;
                }
                if (selection != next) {
                    selection = next;
                    menu_ui_render_selectable_line(7, action_text, selection == 0);
                    menu_ui_render_selectable_line(8, mute_text, selection == 1);
                    menu_ui_render_selectable_line(9, mode_text, selection == 2);
                    menu_ui_render_selectable_line(10, visualizer_text, selection == 3);
                }
            } else if (key == 'C' || key == 'c') {
                if (menu_ui_try_toggle_columns()) {
                    scroll_pos = 0;
                    render_mp3_screen(record, file_name);
                    menu_ui_render_selectable_line(7, action_text, selection == 0);
                    menu_ui_render_selectable_line(8, mute_text, selection == 1);
                    menu_ui_render_selectable_line(9, mode_text, selection == 2);
                    menu_ui_render_selectable_line(10, visualizer_text, selection == 3);
                    if (visualizer_enabled) {
                        render_mp3_visualizer(viz_frame, use_80_columns);
                    } else {
                        render_mp3_clear(use_80_columns);
                    }
                }
            }
        }

        unsigned int now = *jiffyPtr;
        if ((unsigned int)(now - lastTick) >= 10) {
            lastTick = now;

            status = mp3_read_status();
            unsigned int elapsed = mp3_read_elapsed();

            /* Detect track change from auto-advance */
            {
                unsigned int playing_idx = mp3_read_playing_index();
                if (playing_idx != last_playing_index) {
                    last_playing_index = playing_idx;
                    mp3_read_now_playing_name(file_name, MAX_FILE_NAME_LENGTH);
                    current_size_kb = mp3_read_now_playing_size() / 1024u;
                    scroll_pos = 0;
                    /* Update name line */
                    if (use_80_columns) {
                        render_mp3_name_line(file_name, 1);
                    } else {
                        size_t nl = strlen(file_name);
                        if (nl <= MP3_NAME_WIDTH_40) {
                            render_mp3_name_line(file_name, 0);
                        } else {
                            char w[SCREEN_WIDTH + 1];
                            memcpy(w, file_name, MP3_NAME_WIDTH_40);
                            w[MP3_NAME_WIDTH_40] = '\0';
                            render_mp3_name_line(w, 0);
                        }
                    }
                    /* Update size line */
                    Locate(0, 4);
                    if (use_80_columns) {
                        printf("   Size: %lu KB     ", current_size_kb);
                    } else {
                        printf("   Size: %luK     ", current_size_kb);
                    }
                    action_playing = 1;
                }
            }

            // When playback stops, reset action_playing.
            // In ALL/RANDOM modes, Pico auto-advances so only reset on
            // a genuine stop (not a brief EOF gap between tracks).
            if (action_playing && !(status & MP3_STATUS_PLAYING)) {
                if (play_mode == MP3_PLAY_MODE_SINGLE || !(status & MP3_STATUS_EOF)) {
                    action_playing = 0;
                }
            }

            format_time(elapsed, elapsed_str);

            build_mp3_action_text(action_playing, action_text, sizeof(action_text));
            mute_enabled = (status & MP3_STATUS_MUTED) != 0;
            build_mp3_mute_text(mute_enabled, mute_text, sizeof(mute_text));
            build_mp3_visualizer_text(visualizer_enabled, visualizer_text, sizeof(visualizer_text));
            build_mp3_mode_text(play_mode, mode_text, sizeof(mode_text));
            menu_ui_render_selectable_line(7, action_text, selection == 0);
            menu_ui_render_selectable_line(8, mute_text, selection == 1);
            menu_ui_render_selectable_line(9, mode_text, selection == 2);
            menu_ui_render_selectable_line(10, visualizer_text, selection == 3);
            render_mp3_status_line(status, elapsed_str);

            if (visualizer_enabled && (status & MP3_STATUS_PLAYING)) {
                viz_frame++;
                render_mp3_visualizer(viz_frame, use_80_columns);
            } else {
                render_mp3_clear(use_80_columns);
            }

            /* Sliding name effect in 40-column mode */
            if (!use_80_columns) {
                size_t name_len = strlen(file_name);
                if (name_len > MP3_NAME_WIDTH_40) {
                    int printed = build_sliding_name_window(
                        file_name, scroll_pos, name_window, MP3_NAME_WIDTH_40);
                    if (printed) {
                        render_mp3_name_line(name_window, 0);
                    }
                    scroll_pos++;
                    if (scroll_pos >= (int)(name_len + 1)) {
                        scroll_pos = 0;
                    }
                }
            }
        }
    }

    frame_rendered = 0;
    displayMenu();
}
