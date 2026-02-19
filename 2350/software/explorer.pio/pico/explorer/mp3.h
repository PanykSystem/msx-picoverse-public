#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define MP3_STATUS_PLAYING  0x01
#define MP3_STATUS_MUTED    0x02
#define MP3_STATUS_ERROR    0x04
#define MP3_STATUS_EOF      0x08
#define MP3_STATUS_BUSY     0x10
#define MP3_STATUS_READING  0x20

// Callback type for background work (called between MP3 frames on Core 1)
typedef void (*mp3_bg_callback_t)(void);

// Core 1 dedicated loop — call from multicore_launch_core1()
void mp3_core1_loop(void);
void mp3_stop_core1(void);
void mp3_set_bg_callback(mp3_bg_callback_t cb);

void mp3_deinit(void);

void mp3_set_external_buffer(uint8_t *buffer, size_t size);

// Cross-core command API (called from Core 0, executed on Core 1)
void mp3_select_file(const char *path, uint32_t file_size);
void mp3_send_cmd(uint8_t cmd);

#define MP3_CMD_PLAY        2
#define MP3_CMD_STOP        3
#define MP3_CMD_TOGGLE_MUTE 4

// Play modes for auto-advance
#define MP3_PLAY_MODE_SINGLE 0
#define MP3_PLAY_MODE_ALL    1
#define MP3_PLAY_MODE_RANDOM 2

// Direct select+play from Core 1 (used by auto-advance)
void mp3_auto_play(const char *path, uint32_t file_size);

uint8_t mp3_get_status(void);
uint16_t mp3_get_elapsed_seconds(void);
uint16_t mp3_get_total_seconds(void);
