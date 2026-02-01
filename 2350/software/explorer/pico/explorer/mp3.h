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
#define MP3_STATUS_TONE     0x40

void mp3_init(void);
void mp3_update(void);

void mp3_set_external_buffer(uint8_t *buffer, size_t size);

void mp3_select_file(const char *path, uint32_t file_size);
void mp3_play(void);
void mp3_stop(void);
void mp3_toggle_mute(void);
void mp3_play_tone(void);

uint8_t mp3_get_status(void);
uint16_t mp3_get_elapsed_seconds(void);
uint16_t mp3_get_total_seconds(void);
