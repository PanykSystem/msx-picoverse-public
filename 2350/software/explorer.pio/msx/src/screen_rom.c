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
static void render_rom_wifi_line(unsigned char wifi_enabled, int selected);
static void render_rom_action_line(unsigned char row, int selected);
static void render_rom_footer_line(const char *left_text);
static const char *footer_text_for_selection(int selection, unsigned char allow_mapper_override, int wifi_selection);
static void build_mapper_text(const ROMRecord *record, int waiting_mapper, char *out, size_t out_size);
static void build_audio_text(const ROMRecord *record, unsigned char audio_profile, char *out, size_t out_size);
static unsigned char sanitize_audio_profile(const ROMRecord *record, unsigned char audio_profile);
static unsigned char next_audio_profile(const ROMRecord *record, unsigned char audio_profile, int dir);

static unsigned char record_is_system_rom(const ROMRecord *record) {
    unsigned char mapper_code = record_mapper_code(record->Mapper);
    return mapper_code == 9 || mapper_code == 10 || mapper_code == 11 || mapper_code == 15 || mapper_code == 16;
}

static unsigned char record_is_wifi_capable_system_rom(const ROMRecord *record) {
    unsigned char mapper_code = record_mapper_code(record->Mapper);
    return mapper_code == 10 || mapper_code == 11 || mapper_code == 15 || mapper_code == 16;
}

static unsigned char record_supports_scc_audio(const ROMRecord *record) {
    unsigned char mapper_code = record_mapper_code(record->Mapper);
    return mapper_code == 3 || mapper_code == 14;
}

static unsigned char record_supports_dual_psg(const ROMRecord *record) {
    return !record_is_folder(record) && !record_is_system_rom(record) && !record_supports_scc_audio(record);
}

static unsigned char record_supports_msx_music(const ROMRecord *record) {
    return !record_is_folder(record) && !record_is_system_rom(record) && !record_supports_scc_audio(record);
}

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
            printf("    ROM: %-71.71s", name);
        } else {
            printf("    ROM: %-29.29s", name);
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

void show_rom_screen(unsigned int index) {
    ROMRecord *record = &records[index % FILES_PER_PAGE];
    unsigned char waiting_mapper = 0;
    char mapper_text[48];
    char audio_text[32];
    unsigned char audio_profile = AUDIO_PROFILE_NONE;
    unsigned char allow_mapper_override = !record_is_system_rom(record);
    unsigned char allow_wifi_support = record_is_wifi_capable_system_rom(record);
    unsigned char wifi_enabled = 0;
    int wifi_selection = allow_wifi_support ? 2 : -1;
    int action_selection = allow_wifi_support ? 3 : 2;
    unsigned char action_row = allow_wifi_support ? 10 : 9;
    int selection = action_selection;

    render_rom_screen(record);

    if ((record->Mapper & SOURCE_SD_FLAG) && !record_is_folder(record)) {
        unsigned char mapper_code = record_mapper_code(record->Mapper);
        if (mapper_code == 0) {
            for (unsigned int wait = 0; wait < 200; wait++) {
                if (Peek(CTRL_CMD) == 0) {
                    break;
                }
                delay_ms(5);
            }
            send_detect_mapper(index);
            waiting_mapper = 1;
        } else if (record_supports_scc_audio(record)) {
            audio_profile = AUDIO_PROFILE_SCC;
        }
    } else {
        if (record_supports_scc_audio(record)) {
            audio_profile = AUDIO_PROFILE_SCC;
        }
    }

    audio_profile = sanitize_audio_profile(record, audio_profile);

    build_mapper_text(record, waiting_mapper, mapper_text, sizeof(mapper_text));
    build_audio_text(record, audio_profile, audio_text, sizeof(audio_text));
    render_rom_mapper_line(mapper_text, selection == 0);
    render_rom_audio_line(audio_text, selection == 1);
    if (allow_wifi_support) {
        render_rom_wifi_line(wifi_enabled, selection == wifi_selection);
    }
    render_rom_action_line(action_row, selection == action_selection);
    render_rom_footer_line(footer_text_for_selection(selection, allow_mapper_override, wifi_selection));

    while (1) {
        if (bios_chsns()) {
            char key = (char)bios_chget();
            if (key == MENU_KEY_F4_CONFIG) {
                launch_wifi_config();
                return;
            }
            if (key == 'h' || key == 'H') {
                helpMenu();
                render_rom_screen(record);
                build_mapper_text(record, waiting_mapper, mapper_text, sizeof(mapper_text));
                build_audio_text(record, audio_profile, audio_text, sizeof(audio_text));
                render_rom_mapper_line(mapper_text, selection == 0);
                render_rom_audio_line(audio_text, selection == 1);
                if (allow_wifi_support) {
                    render_rom_wifi_line(wifi_enabled, selection == wifi_selection);
                }
                render_rom_action_line(action_row, selection == action_selection);
                render_rom_footer_line(footer_text_for_selection(selection, allow_mapper_override, wifi_selection));
            }
            if (key == 27) {
                break;
            }
            if (key == 13 || key == 32) {
                if (!waiting_mapper && selection == action_selection) {
                    audio_profile = sanitize_audio_profile(record, audio_profile);
                    Poke(CTRL_AUDIO, audio_profile);
                    Poke(CTRL_WIFI_SUPPORT, allow_wifi_support ? wifi_enabled : 0);
                    loadGame((int)index);
                    return;
                }
            }
            if (key == 30 || key == 31) {
                int delta = (key == 30) ? -1 : 1;
                int next = selection + delta;
                if (next < 0) {
                    next = 0;
                } else if (next > action_selection) {
                    next = action_selection;
                }
                if (selection != next) {
                    selection = next;
                    build_mapper_text(record, waiting_mapper, mapper_text, sizeof(mapper_text));
                    build_audio_text(record, audio_profile, audio_text, sizeof(audio_text));
                    render_rom_mapper_line(mapper_text, selection == 0);
                    render_rom_audio_line(audio_text, selection == 1);
                    if (allow_wifi_support) {
                        render_rom_wifi_line(wifi_enabled, selection == wifi_selection);
                    }
                    render_rom_action_line(action_row, selection == action_selection);
                    render_rom_footer_line(footer_text_for_selection(selection, allow_mapper_override, wifi_selection));
                }
            }
            if ((key == 28 || key == 29) && selection == 0 && !waiting_mapper && allow_mapper_override) {
                static const unsigned char mapper_cycle[] = {1,2,3,4,5,6,7,8,9,12,13,14};
                const unsigned int mapper_count = (unsigned int)(sizeof(mapper_cycle) / sizeof(mapper_cycle[0]));
                unsigned char mapper_code = record_mapper_code(record->Mapper);
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
                    record->Mapper = (record->Mapper & (SOURCE_SD_FLAG | FOLDER_FLAG)) | OVERRIDE_FLAG | next_mapper;
                    audio_profile = sanitize_audio_profile(record, audio_profile);
                    build_mapper_text(record, waiting_mapper, mapper_text, sizeof(mapper_text));
                    build_audio_text(record, audio_profile, audio_text, sizeof(audio_text));
                    render_rom_mapper_line(mapper_text, selection == 0);
                    render_rom_audio_line(audio_text, selection == 1);
                }
            }
            if ((key == 28 || key == 29) && selection == 1) {
                int dir = (key == 28) ? 1 : -1;
                audio_profile = next_audio_profile(record, audio_profile, dir);
                build_audio_text(record, audio_profile, audio_text, sizeof(audio_text));
                render_rom_audio_line(audio_text, selection == 1);
            }
            if ((key == 28 || key == 29) && allow_wifi_support && selection == wifi_selection) {
                wifi_enabled = wifi_enabled ? 0 : 1;
                render_rom_wifi_line(wifi_enabled, 1);
            }
            if (key == 'C' || key == 'c') {
                if (menu_ui_try_toggle_columns()) {
                    render_rom_screen(record);
                    build_mapper_text(record, waiting_mapper, mapper_text, sizeof(mapper_text));
                    build_audio_text(record, audio_profile, audio_text, sizeof(audio_text));
                    render_rom_mapper_line(mapper_text, selection == 0);
                    render_rom_audio_line(audio_text, selection == 1);
                    if (allow_wifi_support) {
                        render_rom_wifi_line(wifi_enabled, selection == wifi_selection);
                    }
                    render_rom_action_line(action_row, selection == action_selection);
                    render_rom_footer_line(footer_text_for_selection(selection, allow_mapper_override, wifi_selection));
                }
            }
        }

        if (waiting_mapper && Peek(CTRL_CMD) == 0) {
            unsigned char mapper = read_mapper_value();
            if (mapper != 0) {
                record->Mapper = (record->Mapper & (SOURCE_SD_FLAG | FOLDER_FLAG)) | mapper;
            } else {
                record->Mapper = (record->Mapper & (SOURCE_SD_FLAG | FOLDER_FLAG));
            }
            if (audio_profile == AUDIO_PROFILE_NONE && record_supports_scc_audio(record)) {
                audio_profile = AUDIO_PROFILE_SCC;
            } else {
                audio_profile = sanitize_audio_profile(record, audio_profile);
            }
            build_audio_text(record, audio_profile, audio_text, sizeof(audio_text));
            render_rom_audio_line(audio_text, selection == 1);
            waiting_mapper = 0;
            build_mapper_text(record, waiting_mapper, mapper_text, sizeof(mapper_text));
            render_rom_mapper_line(mapper_text, selection == 0);
            render_rom_footer_line(footer_text_for_selection(selection, allow_mapper_override, wifi_selection));
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

static void render_rom_wifi_line(unsigned char wifi_enabled, int selected) {
    char line[80];
    sprintf(line, "  Wifi: %s", wifi_enabled ? "Yes" : "No");
    menu_ui_render_selectable_line(9, line, selected);
}

static void render_rom_action_line(unsigned char row, int selected) {
    menu_ui_render_selectable_line(row, "Action: Run", selected);
}

static const char *footer_text_for_selection(int selection, unsigned char allow_mapper_override, int wifi_selection) {
    if (selection == 0 && allow_mapper_override) {
        return "LEFT/RIGHT: Mapper";
    }
    if (selection == 1) {
        return "LEFT/RIGHT: Audio";
    }
    if (selection == wifi_selection) {
        return "LEFT/RIGHT: Wifi";
    }
    return "[ENTER - RUN]";
}

static void render_rom_footer_line(const char *left_text) {
    menu_ui_clear_rows(22, 22);
    Locate(0, 22);
    if (use_80_columns) {
        if (left_text && strcmp(left_text, "LEFT/RIGHT: Mapper") == 0) {
            printf("[LEFT/RIGHT - SELECT MAPPER]                                      [ESC - BACK]");
        } else if (left_text && strcmp(left_text, "LEFT/RIGHT: Audio") == 0) {
            printf("[LEFT/RIGHT - SELECT AUDIO]                                       [ESC - BACK]");
        } else if (left_text && strcmp(left_text, "LEFT/RIGHT: Wifi") == 0) {
            printf("[LEFT/RIGHT - SELECT WIFI]                                        [ESC - BACK]");
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
        } else if (left_text && strcmp(left_text, "LEFT/RIGHT: Wifi") == 0) {
            printf("[LEFT/RIGHT - WIFI]       [ESC - BACK]");
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

    unsigned char mapper_code = record_mapper_code(record->Mapper);
    if (mapper_code == 0) {
        strncpy(out, "Unknown mapper", out_size - 1);
        out[out_size - 1] = '\0';
        return;
    }

    const char *desc = mapper_description(record->Mapper);
    if (record_mapper_is_override(record->Mapper)) {
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

static unsigned char audio_profile_is_supported(const ROMRecord *record, unsigned char audio_profile) {
    if (audio_profile == AUDIO_PROFILE_NONE) {
        return 1;
    }
    if (audio_profile == AUDIO_PROFILE_SCC || audio_profile == AUDIO_PROFILE_SCC_PLUS) {
        return record_supports_scc_audio(record);
    }
    if (audio_profile == AUDIO_PROFILE_DUAL_PSG) {
        return record_supports_dual_psg(record);
    }
    if (audio_profile == AUDIO_PROFILE_MSX_MUSIC) {
        return record_supports_msx_music(record);
    }
    return 0;
}

static unsigned char default_audio_profile(const ROMRecord *record) {
    return record_supports_scc_audio(record) ? AUDIO_PROFILE_SCC : AUDIO_PROFILE_NONE;
}

static unsigned char sanitize_audio_profile(const ROMRecord *record, unsigned char audio_profile) {
    if (audio_profile_is_supported(record, audio_profile)) {
        return audio_profile;
    }
    return default_audio_profile(record);
}

static unsigned char next_audio_profile(const ROMRecord *record, unsigned char audio_profile, int dir) {
    static const unsigned char audio_profiles[] = {
        AUDIO_PROFILE_NONE,
        AUDIO_PROFILE_SCC,
        AUDIO_PROFILE_SCC_PLUS,
        AUDIO_PROFILE_DUAL_PSG,
        AUDIO_PROFILE_MSX_MUSIC
    };
    const unsigned int audio_count = (unsigned int)(sizeof(audio_profiles) / sizeof(audio_profiles[0]));
    int current = -1;

    audio_profile = sanitize_audio_profile(record, audio_profile);

    for (unsigned int i = 0; i < audio_count; i++) {
        if (audio_profiles[i] == audio_profile) {
            current = (int)i;
            break;
        }
    }
    if (current < 0) {
        current = 0;
    }

    for (unsigned int step = 0; step < audio_count; step++) {
        current += dir;
        if (current < 0) {
            current = (int)(audio_count - 1);
        } else if ((unsigned int)current >= audio_count) {
            current = 0;
        }
        if (audio_profile_is_supported(record, audio_profiles[current])) {
            return audio_profiles[current];
        }
    }

    return AUDIO_PROFILE_NONE;
}

static void build_audio_text(const ROMRecord *record, unsigned char audio_profile, char *out, size_t out_size) {
    const char *audio_label = "None";
    if (!out || out_size == 0) {
        return;
    }
    audio_profile = sanitize_audio_profile(record, audio_profile);
    if (audio_profile == AUDIO_PROFILE_SCC) {
        audio_label = "SCC";
    } else if (audio_profile == AUDIO_PROFILE_SCC_PLUS) {
        audio_label = "SCC+";
    } else if (audio_profile == AUDIO_PROFILE_DUAL_PSG) {
        audio_label = "Dual PSG";
    } else if (audio_profile == AUDIO_PROFILE_MSX_MUSIC) {
        audio_label = "MSX-MUSIC";
    }
    strncpy(out, audio_label, out_size - 1);
    out[out_size - 1] = '\0';
}
