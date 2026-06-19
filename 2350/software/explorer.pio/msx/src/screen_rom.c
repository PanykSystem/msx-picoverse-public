#include <string.h>
#include "menu.h"
#include "menu_state.h"
#include "menu_ui.h"
#include "menu_input.h"
#include "screen_rom.h"

static void send_detect_mapper(unsigned int index);
static unsigned char send_set_mapper(unsigned int index, unsigned char mapper);
static unsigned char send_load_options(unsigned int index, unsigned char *audio_profile, unsigned char *psg_enabled, unsigned char *mapper, unsigned char *sd_partition, unsigned char *audio_volume);
static void send_save_options(unsigned int index, unsigned char audio_profile, unsigned char psg_enabled, unsigned char mapper, unsigned char sd_partition, unsigned char audio_volume);
static void wait_for_ctrl_cmd_clear(void);
static unsigned char read_mapper_value(void);
static unsigned char read_ack_value(void);
static void render_rom_screen(const ROMRecord *record);
static void render_rom_prefixed_line(unsigned char row, const char *prefix, const char *text, int selected);
static void render_rom_mapper_line(const char *mapper_text, int selected);
static void render_rom_audio_line(const char *audio_text, int selected);
static void render_rom_volume_line(unsigned char audio_volume, int selected);
static void render_rom_psg_line(unsigned char psg_enabled, int selected);
static void render_rom_partition_line(unsigned char row, unsigned char sd_partition, int selected);
static void render_rom_wifi_line(unsigned char row, unsigned char wifi_enabled, int selected);
static void render_rom_action_line(unsigned char row, int selected);
static void render_rom_options_block(const ROMRecord *record, int waiting_mapper, unsigned char audio_profile, unsigned char psg_enabled, unsigned char wifi_enabled, unsigned char allow_mapper_override, unsigned char allow_wifi_support, int selection);
static void render_rom_footer_line(unsigned char mode);
static void show_mp3_screen(unsigned int index);
static void render_mp3_screen(const ROMRecord *record);
static void render_mp3_counter_line(void);
static void render_mp3_footer_line(void);
static void send_mp3_select(unsigned int index);
static unsigned int read_mp3_elapsed(void);
static unsigned int read_mp3_selected_index(void);
static ROMRecord *load_mp3_detail_record(unsigned int index);
static unsigned char footer_mode_for_selection(int selection, unsigned char allow_mapper_override, int psg_selection, int partition_selection, int wifi_selection);
static void build_mapper_text(const ROMRecord *record, int waiting_mapper, char *out, size_t out_size);
static void build_audio_text(const ROMRecord *record, unsigned char audio_profile, char *out, size_t out_size);
static unsigned char sanitize_audio_profile(const ROMRecord *record, unsigned char audio_profile);
static unsigned char next_audio_profile(const ROMRecord *record, unsigned char audio_profile, int dir);
static void apply_detected_mapper(ROMRecord *record);
static unsigned char current_wavegame_rom = 0;

#define MP3_COUNTER_POLL_JIFFIES 5
#define MP3_COUNTER_FORCE_JIFFIES 50
#define ROM_FOOTER_DEFAULT 0
#define ROM_FOOTER_MAPPER  1
#define ROM_FOOTER_AUDIO   2
#define ROM_FOOTER_PSG     3
#define ROM_FOOTER_WIFI    4
#define ROM_FOOTER_RUN     5
#define ROM_FOOTER_SD      6

static unsigned char sd_partition_count = 0;
static unsigned char sd_partition_mask = 0;
static unsigned char rom_sd_partition = 0;
static unsigned char rom_audio_volume = AUDIO_VOLUME_DEFAULT;
static unsigned char rom_allow_sd_partition = 0;

static void write_index_query(unsigned int index) {
    Poke(CTRL_QUERY_BASE + 0, (unsigned char)(index & 0xFFu));
    Poke(CTRL_QUERY_BASE + 1, (unsigned char)((index >> 8) & 0xFFu));
}

static unsigned char record_is_system_rom(const ROMRecord *record) {
    unsigned char mapper_code = record_mapper_code(record->Mapper);
    return mapper_code == 9 || mapper_code == 10 || mapper_code == 11 || mapper_code == 15 || mapper_code == 16;
}

static unsigned char record_is_wifi_capable_system_rom(const ROMRecord *record) {
    unsigned char mapper_code = record_mapper_code(record->Mapper);
    return mapper_code == 10 || mapper_code == 11 || mapper_code == 15 || mapper_code == 16;
}

static unsigned char record_is_sunrise_system_rom(const ROMRecord *record) {
    return record_is_wifi_capable_system_rom(record);
}

static unsigned char record_is_sunrise_sd_system_rom(const ROMRecord *record) {
    unsigned char mapper_code = record_mapper_code(record->Mapper);
    return mapper_code == 15 || mapper_code == 16;
}

static unsigned char record_is_sunrise_mapper_system_rom(const ROMRecord *record) {
    unsigned char mapper_code = record_mapper_code(record->Mapper);
    return mapper_code == 11 || mapper_code == 16;
}

static unsigned char record_supports_scc_audio(const ROMRecord *record) {
    unsigned char mapper_code = record_mapper_code(record->Mapper);
    return mapper_code == 3 || mapper_code == 14;
}

static unsigned char record_supports_external_scc_audio(const ROMRecord *record) {
    return !record_is_folder(record) && (!record_is_system_rom(record) || record_is_sunrise_system_rom(record));
}

static unsigned char record_supports_dual_psg(const ROMRecord *record) {
    return record_supports_external_scc_audio(record) && !record_supports_scc_audio(record);
}

static unsigned char record_supports_msx_music(const ROMRecord *record) {
    /* Base support shared with YM2151/SFG. YM2413/FM-PAC (MSX-MUSIC) additionally
       excludes the Sunrise 1MB-mapper options in audio_profile_is_supported(). */
    return record_supports_external_scc_audio(record);
}

static void send_detect_mapper(unsigned int index) {
    write_index_query(index);
    for (unsigned int i = 2; i < CTRL_QUERY_SIZE; i++) {
        Poke(CTRL_QUERY_BASE + i, 0);
    }
    Poke(CTRL_CMD, CMD_DETECT_MAPPER);
}

static void wait_for_ctrl_cmd_clear(void) {
    for (unsigned int wait = 0; wait < 200; wait++) {
        if (Peek(CTRL_CMD) == 0) {
            break;
        }
        delay_ms(5);
    }
}

static unsigned char send_set_mapper(unsigned int index, unsigned char mapper) {
    write_index_query(index);
    Poke(CTRL_QUERY_BASE + 2, mapper);
    for (unsigned int i = 3; i < CTRL_QUERY_SIZE; i++) {
        Poke(CTRL_QUERY_BASE + i, 0);
    }
    Poke(CTRL_CMD, CMD_SET_MAPPER);
    wait_for_ctrl_cmd_clear();
    return read_ack_value();
}

static unsigned char send_load_options(unsigned int index, unsigned char *audio_profile, unsigned char *psg_enabled, unsigned char *mapper, unsigned char *sd_partition, unsigned char *audio_volume) {
    write_index_query(index);
    for (unsigned int i = 2; i < CTRL_QUERY_SIZE; i++) {
        Poke(CTRL_QUERY_BASE + i, 0);
    }
    Poke(CTRL_CMD, CMD_LOAD_OPTIONS);
    wait_for_ctrl_cmd_clear();
    current_wavegame_rom = Peek(CTRL_WAVEGAME_ROM) ? 1 : 0;
    if (!read_ack_value()) {
        return 0;
    }
    *audio_profile = Peek(CTRL_AUDIO);
    *psg_enabled = Peek(CTRL_PSG_EMULATION) ? 1 : 0;
    *mapper = Peek(CTRL_MAPPER);
    *sd_partition = Peek(CTRL_SD_PARTITION);
    *audio_volume = Peek(CTRL_AUDIO_VOLUME);
    sd_partition_count = Peek(CTRL_SD_PARTITION_INFO_BASE);
    sd_partition_mask = Peek(CTRL_SD_PARTITION_INFO_BASE + 1);
    return 1;
}

static void send_save_options(unsigned int index, unsigned char audio_profile, unsigned char psg_enabled, unsigned char mapper, unsigned char sd_partition, unsigned char audio_volume) {
    write_index_query(index);
    Poke(CTRL_QUERY_BASE + 2, audio_profile);
    Poke(CTRL_QUERY_BASE + 3, psg_enabled ? 1 : 0);
    Poke(CTRL_QUERY_BASE + 4, mapper);
    Poke(CTRL_QUERY_BASE + 5, sd_partition);
    Poke(CTRL_QUERY_BASE + 6, audio_volume);
    for (unsigned int i = 7; i < CTRL_QUERY_SIZE; i++) {
        Poke(CTRL_QUERY_BASE + i, 0);
    }
    Poke(CTRL_CMD, CMD_SAVE_OPTIONS);
    wait_for_ctrl_cmd_clear();
}

static unsigned char read_mapper_value(void) {
    return *((unsigned char *)CTRL_MAPPER);
}

static unsigned char read_ack_value(void) {
    return *((unsigned char *)CTRL_ACK);
}

static void apply_detected_mapper(ROMRecord *record) {
    unsigned char mapper = read_mapper_value();
    if (mapper != 0) {
        record->Mapper = (record->Mapper & (SOURCE_SD_FLAG | FOLDER_FLAG)) | mapper;
    } else {
        record->Mapper = (record->Mapper & (SOURCE_SD_FLAG | FOLDER_FLAG));
    }
}

void quick_run_rom(unsigned int index) {
    ROMRecord *record = &records[index % FILES_PER_PAGE];
    write_index_query(index);
    Poke(CTRL_CMD, CMD_PREPARE_QUICK_RUN);
    while (Peek(CTRL_CMD) != 0) {
        delay_ms(10);
    }
    apply_detected_mapper(record);
    loadGame((int)index);
}

static void render_rom_screen(const ROMRecord *record) {
    Locate(0, 0);
    menu_ui_print_title_line();
    Locate(0, 1);
    menu_ui_print_delimiter_line();

    menu_ui_clear_rows(2, 21);

    {
        char name[MAX_FILE_NAME_LENGTH + 1];
        const char *source = (record->Mapper & SOURCE_SD_FLAG) ? "SD" : "FL";
        unsigned long size_kb = record->Size / 1024u;

        trim_name_to_buffer(record->Name, name, use_80_columns ? 71 : 29);

        Locate(0, 3);
        if (use_80_columns) {
            printf("    ROM: %-71.71s", name);
        } else {
            printf("    ROM: %-29.29s", name);
        }

        Locate(0, 4);
        printf("   Size: %lu KB", size_kb);
        Locate(0, 5);
        printf(" Source: %s", source);
        Locate(0, 6);
        printf(" ");
    }

    Locate(0, 21);
    menu_ui_print_delimiter_line();
    menu_ui_clear_rows(23, 24);
}

void show_rom_screen(unsigned int index) {
    ROMRecord *record = &records[index % FILES_PER_PAGE];
    unsigned char waiting_mapper = 0;
    unsigned char audio_profile = AUDIO_PROFILE_NONE;
    unsigned char psg_enabled = 1;
    unsigned char saved_mapper = 0;
    unsigned char sd_partition = 0;
    unsigned char options_loaded = 0;
    unsigned char allow_mapper_override = !record_is_system_rom(record);
    unsigned char allow_wifi_support = record_is_wifi_capable_system_rom(record);
    unsigned char allow_sd_partition = record_is_sunrise_sd_system_rom(record);
    unsigned char allow_psg = !record_is_sunrise_mapper_system_rom(record);
    unsigned char wifi_enabled = 0;
    int volume_selection = 2;
    int psg_selection = 3;
    int partition_selection = allow_sd_partition ? 4 : -1;
    int wifi_selection = allow_wifi_support ? (allow_sd_partition ? 5 : 4) : -1;
    int action_selection = 4 + (allow_sd_partition ? 1 : 0) + (allow_wifi_support ? 1 : 0);
    int selection = action_selection;

    rom_allow_sd_partition = allow_sd_partition;
    rom_audio_volume = AUDIO_VOLUME_DEFAULT;

    if (record_is_mp3(record)) {
        show_mp3_screen(index);
        return;
    }

    render_rom_screen(record);

    if ((record->Mapper & SOURCE_SD_FLAG) && !record_is_folder(record)) {
        unsigned char mapper_code = record_mapper_code(record->Mapper);
        if (mapper_code == 0) {
            wait_for_ctrl_cmd_clear();
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

    if (!waiting_mapper) {
        options_loaded = send_load_options(index, &audio_profile, &psg_enabled, &saved_mapper, &sd_partition, &rom_audio_volume);
        if (options_loaded && saved_mapper != 0 && allow_mapper_override) {
            record->Mapper = (record->Mapper & (SOURCE_SD_FLAG | FOLDER_FLAG)) | OVERRIDE_FLAG | saved_mapper;
        }
    }
    audio_profile = sanitize_audio_profile(record, audio_profile);
    if (!allow_psg) {
        psg_enabled = 0; // PSG Mirror is disabled on Nextor + 1MB mapper (unstable)
    }

    rom_sd_partition = sd_partition;
    render_rom_options_block(record, waiting_mapper, audio_profile, psg_enabled, wifi_enabled, allow_mapper_override, allow_wifi_support, selection);

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
                render_rom_options_block(record, waiting_mapper, audio_profile, psg_enabled, wifi_enabled, allow_mapper_override, allow_wifi_support, selection);
            }
            if (key == 27) {
                break;
            }
            if (key == 13 || key == 32) {
                if (!waiting_mapper && selection == action_selection) {
                    audio_profile = sanitize_audio_profile(record, audio_profile);
                    Poke(CTRL_AUDIO, audio_profile);
                    Poke(CTRL_PSG_EMULATION, psg_enabled);
                    Poke(CTRL_WIFI_SUPPORT, allow_wifi_support ? wifi_enabled : 0);
                    Poke(CTRL_SD_PARTITION, allow_sd_partition ? sd_partition : 0);
                    Poke(CTRL_AUDIO_VOLUME, rom_audio_volume);
                    send_save_options(index, audio_profile, psg_enabled, record_mapper_code(record->Mapper), allow_sd_partition ? sd_partition : 0, rom_audio_volume);
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
                    render_rom_options_block(record, waiting_mapper, audio_profile, psg_enabled, wifi_enabled, allow_mapper_override, allow_wifi_support, selection);
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
                    render_rom_options_block(record, waiting_mapper, audio_profile, psg_enabled, wifi_enabled, allow_mapper_override, allow_wifi_support, selection);
                }
            }
            if ((key == 28 || key == 29) && selection == 1) {
                int dir = (key == 28) ? 1 : -1;
                audio_profile = next_audio_profile(record, audio_profile, dir);
                render_rom_options_block(record, waiting_mapper, audio_profile, psg_enabled, wifi_enabled, allow_mapper_override, allow_wifi_support, selection);
            }
            if ((key == 28 || key == 29) && selection == volume_selection) {
                if (key == 28) {
                    if (rom_audio_volume < AUDIO_VOLUME_MAX) rom_audio_volume += AUDIO_VOLUME_STEP;
                } else {
                    if (rom_audio_volume >= AUDIO_VOLUME_STEP) rom_audio_volume -= AUDIO_VOLUME_STEP;
                }
                render_rom_options_block(record, waiting_mapper, audio_profile, psg_enabled, wifi_enabled, allow_mapper_override, allow_wifi_support, selection);
            }
            if ((key == 28 || key == 29) && selection == psg_selection && allow_psg) {
                psg_enabled = psg_enabled ? 0 : 1;
                render_rom_options_block(record, waiting_mapper, audio_profile, psg_enabled, wifi_enabled, allow_mapper_override, allow_wifi_support, selection);
            }
            if ((key == 28 || key == 29) && allow_sd_partition && selection == partition_selection && sd_partition_count) {
                int dir = (key == 28) ? 1 : -1;
                unsigned char next_part = sd_partition;
                for (unsigned char i = 0; i < 4; i++) {
                    next_part = (unsigned char)(next_part + dir);
                    if (next_part < 1) {
                        next_part = 4;
                    } else if (next_part > 4) {
                        next_part = 1;
                    }
                    if (sd_partition_mask & (1 << (next_part - 1))) {
                        break;
                    }
                }
                sd_partition = next_part;
                rom_sd_partition = sd_partition;
                Poke(CTRL_SD_PARTITION, sd_partition);
                render_rom_options_block(record, waiting_mapper, audio_profile, psg_enabled, wifi_enabled, allow_mapper_override, allow_wifi_support, selection);
            }
            if ((key == 28 || key == 29) && allow_wifi_support && selection == wifi_selection) {
                wifi_enabled = wifi_enabled ? 0 : 1;
                render_rom_options_block(record, waiting_mapper, audio_profile, psg_enabled, wifi_enabled, allow_mapper_override, allow_wifi_support, selection);
            }
            if (key == 'C' || key == 'c') {
                if (menu_ui_try_toggle_columns()) {
                    render_rom_screen(record);
                    render_rom_options_block(record, waiting_mapper, audio_profile, psg_enabled, wifi_enabled, allow_mapper_override, allow_wifi_support, selection);
                }
            }
        }

        if (waiting_mapper && Peek(CTRL_CMD) == 0) {
            apply_detected_mapper(record);
            if (audio_profile == AUDIO_PROFILE_NONE && record_supports_scc_audio(record)) {
                audio_profile = AUDIO_PROFILE_SCC;
            } else {
                audio_profile = sanitize_audio_profile(record, audio_profile);
            }
            if (!options_loaded) {
                options_loaded = send_load_options(index, &audio_profile, &psg_enabled, &saved_mapper, &sd_partition, &rom_audio_volume);
                if (options_loaded && saved_mapper != 0 && allow_mapper_override) {
                    record->Mapper = (record->Mapper & (SOURCE_SD_FLAG | FOLDER_FLAG)) | OVERRIDE_FLAG | saved_mapper;
                }
            }
            audio_profile = sanitize_audio_profile(record, audio_profile);
            waiting_mapper = 0;
            rom_sd_partition = sd_partition;
            render_rom_options_block(record, waiting_mapper, audio_profile, psg_enabled, wifi_enabled, allow_mapper_override, allow_wifi_support, selection);
        }

        delay_ms(10);
    }

    frame_rendered = 0;
    displayMenu();
}

static void render_rom_prefixed_line(unsigned char row, const char *prefix, const char *text, int selected) {
    char line[80];
    size_t prefix_len = strlen(prefix);
    size_t out_len = 0;
    if (prefix_len >= sizeof(line)) {
        prefix_len = sizeof(line) - 1;
    }
    memcpy(line, prefix, prefix_len);
    out_len = prefix_len;
    line[out_len] = '\0';

    if (text && out_len < (sizeof(line) - 1)) {
        size_t remaining = sizeof(line) - 1 - out_len;
        size_t text_len = strlen(text);
        if (text_len > remaining) {
            text_len = remaining;
        }
        memcpy(line + out_len, text, text_len);
        out_len += text_len;
        line[out_len] = '\0';
    }

    menu_ui_render_selectable_line(row, line, selected);
}

static void render_rom_mapper_line(const char *mapper_text, int selected) {
    render_rom_prefixed_line(7, "    Mapper: ", mapper_text, selected);
}

static void render_rom_audio_line(const char *audio_text, int selected) {
    render_rom_prefixed_line(8, "     Audio: ", audio_text, selected);
}

static void render_rom_volume_line(unsigned char audio_volume, int selected) {
    char line[80];
    sprintf(line, "    Volume: %u%%", audio_volume);
    menu_ui_render_selectable_line(9, line, selected);
}

static void render_rom_psg_line(unsigned char psg_enabled, int selected) {
    char line[80];
    sprintf(line, "PSG Mirror: %s", psg_enabled ? "Yes" : "No");
    menu_ui_render_selectable_line(10, line, selected);
}

static void render_rom_partition_line(unsigned char row, unsigned char sd_partition, int selected) {
    char label[31];
    unsigned char i;
    (void)sd_partition;
    for (i = 0; i < sizeof(label) - 1; i++) {
        label[i] = (char)Peek(CTRL_SD_PARTITION_INFO_BASE + 2 + i);
        if (!label[i]) {
            break;
        }
    }
    label[sizeof(label) - 1] = '\0';
    render_rom_prefixed_line(row, "   SD Part: ", label, selected);
}

static void render_rom_wifi_line(unsigned char row, unsigned char wifi_enabled, int selected) {
    char line[80];
    sprintf(line, "      Wifi: %s", wifi_enabled ? "Yes" : "No");
    menu_ui_render_selectable_line(row, line, selected);
}

static void render_rom_action_line(unsigned char row, int selected) {
    menu_ui_render_selectable_line(row, "    Action: Run", selected);
}

static void render_rom_options_block(const ROMRecord *record, int waiting_mapper, unsigned char audio_profile, unsigned char psg_enabled, unsigned char wifi_enabled, unsigned char allow_mapper_override, unsigned char allow_wifi_support, int selection) {
    char mapper_text[48];
    char audio_text[32];
    int volume_selection = 2;
    int psg_selection = 3;
    int partition_selection = rom_allow_sd_partition ? 4 : -1;
    int wifi_selection = allow_wifi_support ? (rom_allow_sd_partition ? 5 : 4) : -1;
    int action_selection = 4 + (rom_allow_sd_partition ? 1 : 0) + (allow_wifi_support ? 1 : 0);
    unsigned char row = 11;

    build_mapper_text(record, waiting_mapper, mapper_text, sizeof(mapper_text));
    build_audio_text(record, audio_profile, audio_text, sizeof(audio_text));
    render_rom_mapper_line(mapper_text, selection == 0);
    render_rom_audio_line(audio_text, selection == 1);
    render_rom_volume_line(rom_audio_volume, selection == volume_selection);
    render_rom_psg_line(psg_enabled, selection == psg_selection);
    if (rom_allow_sd_partition) {
        render_rom_partition_line(row++, rom_sd_partition, selection == partition_selection);
    }
    if (allow_wifi_support) {
        render_rom_wifi_line(row++, wifi_enabled, selection == wifi_selection);
    }
    render_rom_action_line(row, selection == action_selection);
    render_rom_footer_line(footer_mode_for_selection(selection, allow_mapper_override, psg_selection, partition_selection, wifi_selection));
}

static unsigned char footer_mode_for_selection(int selection, unsigned char allow_mapper_override, int psg_selection, int partition_selection, int wifi_selection) {
    if (selection == 0 && allow_mapper_override) {
        return ROM_FOOTER_MAPPER;
    }
    if (selection == 1 || selection == 2) {
        return ROM_FOOTER_AUDIO;
    }
    if (selection == psg_selection) {
        return ROM_FOOTER_PSG;
    }
    if (selection == partition_selection) {
        return ROM_FOOTER_SD;
    }
    if (selection == wifi_selection) {
        return ROM_FOOTER_WIFI;
    }
    return ROM_FOOTER_RUN;
}

static void render_rom_footer_line(unsigned char mode) {
    menu_ui_clear_rows(22, 24);
    Locate(0, 22);

    if (use_80_columns) printf("[ESC - BACK] ");
    else printf("[ESC-BACK] ");

    if (mode == ROM_FOOTER_MAPPER) {
            printf("[LEFT/RIGHT - SELECT MAPPER]");
        } else if (mode == ROM_FOOTER_AUDIO) {
            printf("[LEFT/RIGHT - CHANGE]");
        } else if (mode == ROM_FOOTER_PSG) {
            printf("[LEFT/RIGHT - PSG MIRROR]");
        } else if (mode == ROM_FOOTER_WIFI) {
            printf("[LEFT/RIGHT - SELECT WIFI]");
        } else if (mode == ROM_FOOTER_SD) {
            printf("[LEFT/RIGHT - SD PARTITION]");
        } else if (mode == ROM_FOOTER_RUN) {
            printf("[ENTER - RUN]");
        } 


}
static void send_mp3_select(unsigned int index) {
    Poke(MP3_CTRL_INDEX_L, (unsigned char)(index & 0xFFu));
    Poke(MP3_CTRL_INDEX_H, (unsigned char)((index >> 8) & 0xFFu));
    Poke(MP3_CTRL_CMD, MP3_CMD_SELECT);
}

static unsigned int read_mp3_elapsed(void) {
    return (unsigned int)Peek(MP3_CTRL_ELAPSED_L) |
           ((unsigned int)Peek(MP3_CTRL_ELAPSED_H) << 8);
}

static unsigned int read_mp3_selected_index(void) {
    return (unsigned int)Peek(MP3_CTRL_INDEX_L) |
           ((unsigned int)Peek(MP3_CTRL_INDEX_H) << 8);
}

static ROMRecord *load_mp3_detail_record(unsigned int index) {
    currentIndex = (int)index;
    currentPage = (int)(index / FILES_PER_PAGE) + 1;
    if (paging_enabled) {
        load_page_records((unsigned int)(currentPage - 1));
    }
    return &records[index % FILES_PER_PAGE];
}

static void render_mp3_screen(const ROMRecord *record) {
    char name[MAX_FILE_NAME_LENGTH + 1];
    unsigned long size_kb = record->Size / 1024u;
    const char *type = ((record->Mapper & AUDIO_TYPE_MASK) == AUDIO_TYPE_WAV) ? "WAV" : "MP3";

    Locate(0, 0);
    menu_ui_print_title_line();
    Locate(0, 1);
    menu_ui_print_delimiter_line();
    menu_ui_clear_rows(2, 21);

    trim_name_to_buffer(record->Name, name, use_80_columns ? 71 : 29);

    Locate(0, 3);
    if (use_80_columns) {
        printf("    %s: %-71.71s", type, name);
    } else {
        printf("    %s: %-29.29s", type, name);
    }

    Locate(0, 4);
    printf("   Type: %s", type);

    Locate(0, 5);
    printf("   Size: %lu KB", size_kb);
    Locate(0, 6);
    printf(" Source: SD");

    Locate(0, 21);
    menu_ui_print_delimiter_line();
    menu_ui_clear_rows(23, 24);
}

static void render_mp3_counter_line(void) {
    unsigned int elapsed = read_mp3_elapsed();
    unsigned char status = Peek(MP3_CTRL_STATUS);
    const char *state = "Ready";

    if (status & MP3_STATUS_ERROR) {
        state = "Error";
    } else if (status & MP3_STATUS_PLAYING) {
        state = "Playing";
    } else if (status & MP3_STATUS_PAUSED) {
        state = "Paused";
    } else if (status & MP3_STATUS_EOF) {
        state = "Done";
    }

    menu_ui_clear_rows(8, 9);
    Locate(0, 8);
    printf("   Play: %02u:%02u   %-8.8s", elapsed / 60, elapsed % 60, state);

}

static unsigned char mp3_play_stop_command(unsigned char status) {
    return (status & (MP3_STATUS_PLAYING | MP3_STATUS_PAUSED)) ? MP3_CMD_STOP : MP3_CMD_PLAY;
}

static unsigned char mp3_pause_resume_command(unsigned char status) {
    return (status & MP3_STATUS_PAUSED) ? MP3_CMD_RESUME : MP3_CMD_PAUSE;
}

static void render_mp3_actions(int selection, unsigned char status) {
    unsigned char mode = Peek(MP3_CTRL_MODE);
    menu_ui_clear_rows(10, 14);
    menu_ui_render_selectable_line(10,
        (mp3_play_stop_command(status) == MP3_CMD_STOP) ? "Action: Stop" : "Action: Play",
        selection == 0);
    menu_ui_render_selectable_line(11,
        (mp3_pause_resume_command(status) == MP3_CMD_RESUME) ? "Action: Resume" : "Action: Pause",
        selection == 1);
    if (mode == MP3_PLAY_MODE_ALL) {
        menu_ui_render_selectable_line(12, "  Mode: All", selection == 2);
    } else if (mode == MP3_PLAY_MODE_RANDOM) {
        menu_ui_render_selectable_line(12, "  Mode: Random", selection == 2);
    } else {
        menu_ui_render_selectable_line(12, "  Mode: Single", selection == 2);
    }
}

static void render_mp3_footer_line(void) {
    unsigned char mode = Peek(MP3_CTRL_MODE);
    menu_ui_clear_rows(22, 22);
    Locate(0, 22);
    if (use_80_columns) {
        if (mode == MP3_PLAY_MODE_ALL || mode == MP3_PLAY_MODE_RANDOM) {
            printf("[ESC - BACK] [ENTER - ACTION] [N - NEXT SONG] [P - PREVIOUS SONG]             ");
        } else {
            printf("[ESC - BACK] [ENTER - ACTION]                                                 ");
        }
    } else {
        if (mode == MP3_PLAY_MODE_ALL || mode == MP3_PLAY_MODE_RANDOM) {
            printf("[ESC - BACK] [N - NEXT] [P - PREVIOUS]");
        } else {
            printf("[ESC - BACK] [ENT - ACTION]           ");
        }
    }
}

static void show_mp3_screen(unsigned int index) {
    unsigned int detail_index = index;
    unsigned int last_detail_index = index;
    ROMRecord *record = &records[detail_index % FILES_PER_PAGE];
    volatile unsigned int *jiffyPtr = (volatile unsigned int *)JIFFY;
    unsigned int last_counter_tick = *jiffyPtr;
    unsigned int last_force_tick = last_counter_tick;
    unsigned int last_elapsed = 0xFFFFu;
    unsigned char last_status = 0xFFu;
    int selection = 0;

    send_mp3_select(detail_index);
    render_mp3_screen(record);
    render_mp3_counter_line();
    last_elapsed = read_mp3_elapsed();
    last_status = Peek(MP3_CTRL_STATUS);
    render_mp3_actions(selection, last_status);
    render_mp3_footer_line();

    while (1) {
        if (bios_chsns()) {
            char key = (char)bios_chget();
            if (key == MENU_KEY_F4_CONFIG) {
                launch_wifi_config();
                return;
            }
            if (key == 'h' || key == 'H') {
                helpMenu();
                record = &records[detail_index % FILES_PER_PAGE];
                render_mp3_screen(record);
                render_mp3_counter_line();
                render_mp3_actions(selection, last_status);
                render_mp3_footer_line();
            }
            if (key == 27) {
                Poke(MP3_CTRL_CMD, MP3_CMD_STOP);
                break;
            }
            if (key == 13 || key == 32) {
                unsigned char status = Peek(MP3_CTRL_STATUS);
                unsigned char cmd = 0;
                if (selection == 0) {
                    cmd = mp3_play_stop_command(status);
                    if (cmd == MP3_CMD_PLAY) {
                        send_mp3_select(detail_index);
                    }
                } else if (selection == 1) {
                    cmd = mp3_pause_resume_command(status);
                } else if (selection == 2) {
                    unsigned char mode = Peek(MP3_CTRL_MODE);
                    mode = (mode >= MP3_PLAY_MODE_RANDOM) ? MP3_PLAY_MODE_SINGLE : (unsigned char)(mode + 1u);
                    Poke(MP3_CTRL_MODE, mode);
                    render_mp3_actions(selection, status);
                    render_mp3_footer_line();
                }
                if (cmd) {
                    Poke(MP3_CTRL_CMD, cmd);
                    status = Peek(MP3_CTRL_STATUS);
                    last_elapsed = read_mp3_elapsed();
                    render_mp3_counter_line();
                    render_mp3_actions(selection, status);
                    last_status = status;
                    last_force_tick = *jiffyPtr;
                }
            }
            if (key == 30 || key == 31) {
                int next = selection + ((key == 30) ? -1 : 1);
                if (next < 0) {
                    next = 0;
                } else if (next > 2) {
                    next = 2;
                }
                if (selection != next) {
                    selection = next;
                    render_mp3_actions(selection, last_status);
                }
            }
            if (key == 28 || key == 29) {
                unsigned char mode = Peek(MP3_CTRL_MODE);
                if (selection == 2) {
                    if (key == 28) {
                        mode = (mode >= MP3_PLAY_MODE_RANDOM) ? MP3_PLAY_MODE_SINGLE : (unsigned char)(mode + 1u);
                    } else {
                        mode = (mode == MP3_PLAY_MODE_SINGLE) ? MP3_PLAY_MODE_RANDOM : (unsigned char)(mode - 1u);
                    }
                    Poke(MP3_CTRL_MODE, mode);
                    render_mp3_actions(selection, last_status);
                    render_mp3_footer_line();
                }
            }
            if (key == 'n' || key == 'N' || key == 'p' || key == 'P') {
                unsigned char mode = Peek(MP3_CTRL_MODE);
                unsigned char status = Peek(MP3_CTRL_STATUS);
                if ((status & MP3_STATUS_PLAYING) && (mode == MP3_PLAY_MODE_ALL || mode == MP3_PLAY_MODE_RANDOM)) {
                    Poke(MP3_CTRL_CMD, (key == 'p' || key == 'P') ? MP3_CMD_PREVIOUS : MP3_CMD_NEXT);
                }
            }
            if (key == 'C' || key == 'c') {
                if (menu_ui_try_toggle_columns()) {
                    record = &records[detail_index % FILES_PER_PAGE];
                    render_mp3_screen(record);
                    render_mp3_counter_line();
                    render_mp3_actions(selection, last_status);
                    render_mp3_footer_line();
                }
            }
        }

        {
            unsigned int now = *jiffyPtr;
            if ((unsigned int)(now - last_counter_tick) >= MP3_COUNTER_POLL_JIFFIES) {
                unsigned int elapsed = read_mp3_elapsed();
                unsigned char status = Peek(MP3_CTRL_STATUS);
                unsigned int selected_index = read_mp3_selected_index();
                last_counter_tick = now;
                if (selected_index != last_detail_index && selected_index < totalFiles) {
                    detail_index = selected_index;
                    last_detail_index = selected_index;
                    record = load_mp3_detail_record(detail_index);
                    render_mp3_screen(record);
                    render_mp3_counter_line();
                    render_mp3_actions(selection, status);
                    render_mp3_footer_line();
                }
                if (elapsed != last_elapsed || status != last_status ||
                    (unsigned int)(now - last_force_tick) >= MP3_COUNTER_FORCE_JIFFIES) {
                    render_mp3_counter_line();
                    if (status != last_status) {
                        render_mp3_actions(selection, status);
                    }
                    last_elapsed = elapsed;
                    last_status = status;
                    last_force_tick = now;
                }
            }
        }
        delay_ms(10);
    }

    frame_rendered = 0;
    displayMenu();
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
    if (current_wavegame_rom) {
        return audio_profile == AUDIO_PROFILE_NONE;
    }
    if (audio_profile == AUDIO_PROFILE_NONE) {
        return 1;
    }
    if (audio_profile == AUDIO_PROFILE_SCC || audio_profile == AUDIO_PROFILE_SCC_PLUS) {
        return record_supports_scc_audio(record);
    }
    if (audio_profile == AUDIO_PROFILE_SCC_EXTERNAL || audio_profile == AUDIO_PROFILE_SCC_PLUS_EXTERNAL) {
        return record_supports_external_scc_audio(record);
    }
    if (audio_profile == AUDIO_PROFILE_YM2151_SFG05 || audio_profile == AUDIO_PROFILE_YM2151_SFG01) {
        return record_supports_msx_music(record);
    }
    if (audio_profile == AUDIO_PROFILE_DUAL_PSG) {
        return record_supports_dual_psg(record);
    }
    if (audio_profile == AUDIO_PROFILE_MSX_MUSIC) {
        return record_supports_msx_music(record) && !record_is_sunrise_mapper_system_rom(record);
    }
    return 0;
}

static unsigned char default_audio_profile(const ROMRecord *record) {
    if (current_wavegame_rom) {
        return AUDIO_PROFILE_NONE;
    }
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
        AUDIO_PROFILE_SCC_EXTERNAL,
        AUDIO_PROFILE_SCC_PLUS_EXTERNAL,
        AUDIO_PROFILE_YM2151_SFG05,
        AUDIO_PROFILE_YM2151_SFG01,
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
        audio_label = "YM2212 (SCC)";
    } else if (audio_profile == AUDIO_PROFILE_SCC_PLUS) {
        audio_label = "YM2212 (SCC+)";
    } else if (audio_profile == AUDIO_PROFILE_SCC_EXTERNAL) {
        audio_label = "YM2212 (SCC) - External";
    } else if (audio_profile == AUDIO_PROFILE_SCC_PLUS_EXTERNAL) {
        audio_label = "YM2212 (SCC+) - External";
    } else if (audio_profile == AUDIO_PROFILE_YM2151_SFG05) {
        audio_label = "YM2164 (SFG05/OPP)";
    } else if (audio_profile == AUDIO_PROFILE_YM2151_SFG01) {
        audio_label = "YM2151 (SFG01/OPM)";
    } else if (audio_profile == AUDIO_PROFILE_DUAL_PSG) {
        audio_label = "YM2149 (Dual PSG)";
    } else if (audio_profile == AUDIO_PROFILE_MSX_MUSIC) {
        audio_label = "YM2413 (FMPAC/MSX-MUSIC)";
    }
    strncpy(out, audio_label, out_size - 1);
    out[out_size - 1] = '\0';
}
