#include <string.h>
#include "menu.h"
#include "menu_state.h"
#include "menu_ui.h"
#include "menu_input.h"
#include "screen_rom.h"

static void send_detect_mapper(unsigned int index);
static unsigned char send_set_mapper(unsigned int index, unsigned char mapper);
static unsigned char read_mapper_value(void);
static unsigned char read_ack_value(void);
static void render_rom_screen(const ROMRecord *record);
static void render_rom_mapper_line(const char *mapper_text, int selected);
static void render_rom_audio_line(const char *audio_text, int selected);
static void render_rom_action_line(int selected);
static void render_rom_footer_line(const char *left_text);
static void build_mapper_text(const ROMRecord *record, int waiting_mapper, char *out, size_t out_size);
static void build_audio_text(const ROMRecord *record, unsigned char audio_index, char *out, size_t out_size);

static void send_detect_mapper(unsigned int index) {
    unsigned char low = (unsigned char)(index & 0xFFu);
    unsigned char high = (unsigned char)((index >> 8) & 0xFFu);
    Poke(CTRL_QUERY_BASE + 0, low);
    Poke(CTRL_QUERY_BASE + 1, high);
    for (unsigned int i = 2; i < CTRL_QUERY_SIZE; i++) {
        Poke(CTRL_QUERY_BASE + i, 0);
    }
    Poke(CTRL_CMD, CMD_DETECT_MAPPER);
}

static unsigned char send_set_mapper(unsigned int index, unsigned char mapper) {
    unsigned char low = (unsigned char)(index & 0xFFu);
    unsigned char high = (unsigned char)((index >> 8) & 0xFFu);
    Poke(CTRL_QUERY_BASE + 0, low);
    Poke(CTRL_QUERY_BASE + 1, high);
    Poke(CTRL_QUERY_BASE + 2, mapper);
    for (unsigned int i = 3; i < CTRL_QUERY_SIZE; i++) {
        Poke(CTRL_QUERY_BASE + i, 0);
    }
    Poke(CTRL_CMD, CMD_SET_MAPPER);
    for (unsigned int wait = 0; wait < 200; wait++) {
        if (Peek(CTRL_CMD) == 0) {
            break;
        }
        delay_ms(5);
    }
    return read_ack_value();
}

static unsigned char read_mapper_value(void) {
    return *((unsigned char *)CTRL_MAPPER);
}

static unsigned char read_ack_value(void) {
    return *((unsigned char *)CTRL_ACK);
}

static void render_rom_screen(const ROMRecord *record) {
    Locate(0, 0);
    menu_ui_print_title_line();
    Locate(0, 1);
    menu_ui_print_delimiter_line();

    menu_ui_clear_rows(2, 21);

    {
        char name[CTRL_QUERY_SIZE];
        const char *source = (record->Mapper & SOURCE_SD_FLAG) ? "SD" : "FL";
        unsigned long size_kb = record->Size / 1024u;

        trim_name_to_buffer(record->Name, name, CTRL_QUERY_SIZE - 1);

        Locate(0, 3);
        if (use_80_columns) {
            printf("    ROM: %-72.72s", name);
        } else {
            printf("    ROM: %-32.32s", name);
        }

        Locate(0, 4);
        if (use_80_columns) {
            printf("   Size: %lu KB", size_kb);
        } else {
            printf("   Size:%luK", size_kb);
        }

        Locate(0, 5);
        printf(" Source: %s", source);

        Locate(0, 6);
        printf(" ");
    }

    Locate(0, 21);
    menu_ui_print_delimiter_line();
}

void show_rom_screen(unsigned int index) {
    ROMRecord *record = &records[index % FILES_PER_PAGE];
    unsigned char waiting_mapper = 0;
    int selection = 2;
    char mapper_text[48];
    char audio_text[32];
    unsigned char audio_index = 0;

    render_rom_screen(record);

    if ((record->Mapper & SOURCE_SD_FLAG) && !record_is_folder(record) && !record_is_mp3(record)) {
        unsigned char mapper_code = record->Mapper & ~(SOURCE_SD_FLAG | FOLDER_FLAG | MP3_FLAG | OVERRIDE_FLAG);
        if (mapper_code == 0) {
            for (unsigned int wait = 0; wait < 200; wait++) {
                if (Peek(CTRL_CMD) == 0) {
                    break;
                }
                delay_ms(5);
            }
            send_detect_mapper(index);
            waiting_mapper = 1;
        }
    }

    build_mapper_text(record, waiting_mapper, mapper_text, sizeof(mapper_text));
    build_audio_text(record, audio_index, audio_text, sizeof(audio_text));
    render_rom_mapper_line(mapper_text, selection == 0);
    render_rom_audio_line(audio_text, selection == 1);
    render_rom_action_line(selection == 2);
    render_rom_footer_line(selection == 0 ? "LEFT/RIGHT: Mapper" : (selection == 1 ? "LEFT/RIGHT: Audio" : "[ENTER - RUN]"));

    while (1) {
        if (bios_chsns()) {
            char key = (char)bios_chget();
            if (key == 27) {
                break;
            }
            if (key == 13) {
                if (!waiting_mapper && selection == 2) {
                    loadGame((int)index);
                    return;
                }
            }
            if (key == 30 || key == 31) {
                int delta = (key == 30) ? -1 : 1;
                int next = selection + delta;
                if (next < 0) {
                    next = 0;
                } else if (next > 2) {
                    next = 2;
                }
                if (selection != next) {
                    selection = next;
                    build_mapper_text(record, waiting_mapper, mapper_text, sizeof(mapper_text));
                    build_audio_text(record, audio_index, audio_text, sizeof(audio_text));
                    render_rom_mapper_line(mapper_text, selection == 0);
                    render_rom_audio_line(audio_text, selection == 1);
                    render_rom_action_line(selection == 2);
                    render_rom_footer_line(selection == 0 ? "LEFT/RIGHT: Mapper" : (selection == 1 ? "LEFT/RIGHT: Audio" : "[ENTER - RUN]"));
                }
            }
            if ((key == 28 || key == 29) && selection == 0 && !waiting_mapper) {
                static const unsigned char mapper_cycle[] = {1,2,3,4,5,6,7,8,9};
                const unsigned int mapper_count = (unsigned int)(sizeof(mapper_cycle) / sizeof(mapper_cycle[0]));
                unsigned char mapper_code = record->Mapper & ~(SOURCE_SD_FLAG | FOLDER_FLAG | MP3_FLAG | OVERRIDE_FLAG);
                int dir = (key == 28) ? 1 : -1;
                int found = -1;
                for (unsigned int i = 0; i < mapper_count; i++) {
                    if (mapper_cycle[i] == mapper_code) {
                        found = (int)i;
                        break;
                    }
                }
                int next_index = 0;
                if (found < 0) {
                    next_index = (dir > 0) ? 0 : (int)(mapper_count - 1);
                } else {
                    next_index = found + dir;
                    if (next_index < 0) {
                        next_index = (int)(mapper_count - 1);
                    } else if ((unsigned int)next_index >= mapper_count) {
                        next_index = 0;
                    }
                }
                unsigned char next_mapper = mapper_cycle[next_index];
                if (send_set_mapper(index, next_mapper)) {
                    record->Mapper = (record->Mapper & (SOURCE_SD_FLAG | FOLDER_FLAG | MP3_FLAG)) | OVERRIDE_FLAG | next_mapper;
                    build_mapper_text(record, waiting_mapper, mapper_text, sizeof(mapper_text));
                    render_rom_mapper_line(mapper_text, selection == 0);
                }
            }
            if ((key == 28 || key == 29) && selection == 1) {
                static const unsigned char audio_cycle[] = {0, 1, 2, 3, 4};
                const unsigned int audio_count = (unsigned int)(sizeof(audio_cycle) / sizeof(audio_cycle[0]));
                int dir = (key == 28) ? 1 : -1;
                int next_index = (int)audio_index + dir;
                if (next_index < 0) {
                    next_index = (int)(audio_count - 1);
                } else if ((unsigned int)next_index >= audio_count) {
                    next_index = 0;
                }
                audio_index = audio_cycle[next_index];
                build_audio_text(record, audio_index, audio_text, sizeof(audio_text));
                render_rom_audio_line(audio_text, selection == 1);
            }
            if (key == 'C' || key == 'c') {
                if (menu_ui_try_toggle_columns()) {
                    render_rom_screen(record);
                    build_mapper_text(record, waiting_mapper, mapper_text, sizeof(mapper_text));
                    build_audio_text(record, audio_index, audio_text, sizeof(audio_text));
                    render_rom_mapper_line(mapper_text, selection == 0);
                    render_rom_audio_line(audio_text, selection == 1);
                    render_rom_action_line(selection == 2);
                    render_rom_footer_line(selection == 0 ? "LEFT/RIGHT: Mapper" : (selection == 1 ? "LEFT/RIGHT: Audio" : "[ENTER - RUN]"));
                }
            }
        }

        if (waiting_mapper && Peek(CTRL_CMD) == 0) {
            unsigned char mapper = read_mapper_value();
            if (mapper != 0) {
                record->Mapper = (record->Mapper & (SOURCE_SD_FLAG | FOLDER_FLAG | MP3_FLAG)) | mapper;
            } else {
                record->Mapper = (record->Mapper & (SOURCE_SD_FLAG | FOLDER_FLAG | MP3_FLAG));
            }
            waiting_mapper = 0;
            build_mapper_text(record, waiting_mapper, mapper_text, sizeof(mapper_text));
            render_rom_mapper_line(mapper_text, selection == 0);
            render_rom_footer_line(selection == 0 ? "LEFT/RIGHT: Mapper" : (selection == 1 ? "LEFT/RIGHT: Audio" : "[ENTER - RUN]"));
        }

        delay_ms(10);
    }

    frame_rendered = 0;
    displayMenu();
}

static void render_rom_mapper_line(const char *mapper_text, int selected) {
    char line[80];
    const char *prefix = "Mapper: ";
    size_t prefix_len = strlen(prefix);
    size_t out_len = 0;
    if (prefix_len >= sizeof(line)) {
        prefix_len = sizeof(line) - 1;
    }
    memcpy(line, prefix, prefix_len);
    out_len = prefix_len;
    line[out_len] = '\0';

    if (mapper_text && out_len < (sizeof(line) - 1)) {
        size_t remaining = sizeof(line) - 1 - out_len;
        size_t text_len = strlen(mapper_text);
        if (text_len > remaining) {
            text_len = remaining;
        }
        memcpy(line + out_len, mapper_text, text_len);
        out_len += text_len;
        line[out_len] = '\0';
    }

    menu_ui_render_selectable_line(7, line, selected);
}

static void render_rom_audio_line(const char *audio_text, int selected) {
    char line[80];
    const char *prefix = " Audio: ";
    size_t prefix_len = strlen(prefix);
    size_t out_len = 0;
    if (prefix_len >= sizeof(line)) {
        prefix_len = sizeof(line) - 1;
    }
    memcpy(line, prefix, prefix_len);
    out_len = prefix_len;
    line[out_len] = '\0';

    if (audio_text && out_len < (sizeof(line) - 1)) {
        size_t remaining = sizeof(line) - 1 - out_len;
        size_t text_len = strlen(audio_text);
        if (text_len > remaining) {
            text_len = remaining;
        }
        memcpy(line + out_len, audio_text, text_len);
        out_len += text_len;
        line[out_len] = '\0';
    }

    menu_ui_render_selectable_line(8, line, selected);
}

static void render_rom_action_line(int selected) {
    menu_ui_render_selectable_line(9, "Action: Run", selected);
}

static void render_rom_footer_line(const char *left_text) {
    Locate(0, 22);
    if (use_80_columns) {
        if (left_text && strcmp(left_text, "LEFT/RIGHT: Mapper") == 0) {
            printf("[LEFT/RIGHT - SELECT MAPPER]                                      [ESC - BACK]");
        } else if (left_text && strcmp(left_text, "LEFT/RIGHT: Audio") == 0) {
            printf("[LEFT/RIGHT - SELECT AUDIO]                                      [ESC - BACK]");
        } else if (left_text && strcmp(left_text, "[ENTER - RUN]") == 0) {
            printf("[ENTER - RUN]                                                     [ESC - BACK]");
        } else {
            printf("                                                                  [ESC - BACK]");
        }
    } else {
        if (left_text && strcmp(left_text, "LEFT/RIGHT: Mapper") == 0) {
            printf("[LEFT/RIGHT - MAPPER]     [ESC - BACK]");
        } else if (left_text && strcmp(left_text, "LEFT/RIGHT: Audio") == 0) {
            printf("[LEFT/RIGHT - AUDIO]      [ESC - BACK]");
        } else if (left_text && strcmp(left_text, "[ENTER - RUN]") == 0) {
            printf("[ENTER - RUN]             [ESC - BACK]");
        } else {
            printf("                          [ESC - BACK]");
        }
    }
}

static void build_mapper_text(const ROMRecord *record, int waiting_mapper, char *out, size_t out_size) {
    if (!out || out_size == 0) {
        return;
    }

    if (waiting_mapper) {
        strncpy(out, "Detecting...", out_size - 1);
        out[out_size - 1] = '\0';
        return;
    }

    unsigned char mapper_code = record->Mapper & ~(SOURCE_SD_FLAG | FOLDER_FLAG | MP3_FLAG | OVERRIDE_FLAG);
    if (mapper_code == 0) {
        strncpy(out, "Unknown mapper", out_size - 1);
        out[out_size - 1] = '\0';
        return;
    }

    const char *desc = mapper_description(record->Mapper);
    if (record->Mapper & OVERRIDE_FLAG) {
        strncpy(out, desc, out_size - 1);
        out[out_size - 1] = '\0';
    } else {
        size_t desc_len = strlen(desc);
        size_t suffix_len = strlen(" (detected)");
        size_t max_desc = out_size - 1;
        if (max_desc > suffix_len) {
            max_desc -= suffix_len;
        } else {
            max_desc = 0;
        }
        if (desc_len > max_desc) {
            desc_len = max_desc;
        }
        memcpy(out, desc, desc_len);
        out[desc_len] = '\0';
        if (out_size > 1 && desc_len + suffix_len < out_size) {
            memcpy(out + desc_len, " (detected)", suffix_len);
            out[desc_len + suffix_len] = '\0';
        }
    }
}

static void build_audio_text(const ROMRecord *record, unsigned char audio_index, char *out, size_t out_size) {
    (void)record;
    const char *audio_labels[] = {"None", "OPL4", "SCC", "SCC+", "Second PSG"};
    const unsigned int audio_count = (unsigned int)(sizeof(audio_labels) / sizeof(audio_labels[0]));
    if (!out || out_size == 0) {
        return;
    }
    if (audio_index >= audio_count) {
        audio_index = 0;
    }
    strncpy(out, audio_labels[audio_index], out_size - 1);
    out[out_size - 1] = '\0';
}
