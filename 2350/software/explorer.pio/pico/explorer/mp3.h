#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

struct audio_buffer_pool;

#define MP3_STATUS_PLAYING  0x01
#define MP3_STATUS_MUTED    0x02
#define MP3_STATUS_ERROR    0x04
#define MP3_STATUS_EOF      0x08
#define MP3_STATUS_BUSY     0x10
#define MP3_STATUS_READING  0x20
#define MP3_STATUS_PAUSED   0x40

// Callback type for background work (called between MP3 frames on Core 1)
typedef void (*mp3_bg_callback_t)(void);
typedef int16_t (*mp3_wavegame_psg_sample_callback_t)(void);
typedef void (*mp3_wavegame_psg_rate_callback_t)(uint32_t sample_rate);

// Core 1 dedicated loop — call from multicore_launch_core1()
void mp3_core1_loop(void);
void mp3_stop_core1(void);
void mp3_request_shutdown(void);
void mp3_set_bg_callback(mp3_bg_callback_t cb);

// True once Core 1 has finished mp3_init() and entered its command loop.
bool mp3_core1_is_ready(void);
bool mp3_core1_has_stopped(void);
struct audio_buffer_pool *mp3_take_i2s_audio_pool(void);
struct audio_buffer_pool *mp3_force_i2s_handoff_from_core0(void);

void mp3_deinit(void);

void mp3_set_external_buffer(uint8_t *buffer, size_t size);
void mp3_wavegame_set_psg_callbacks(mp3_wavegame_psg_sample_callback_t sample_cb,
									mp3_wavegame_psg_rate_callback_t rate_cb);

// Cross-core command API (called from Core 0, executed on Core 1)
void mp3_select_file(const char *path, uint32_t file_size);
void mp3_send_cmd(uint8_t cmd);
void mp3_wavegame_play(const char *path, bool loop, uint32_t start_sample, uint32_t loop_sample, bool fade_in);
void mp3_wavegame_play_index(const char *dir, uint8_t index, bool loop);
void mp3_wavegame_play_pause(const char *dir);
void mp3_wavegame_fade_out(uint8_t seconds);
void mp3_wavegame_toggle_pause(void);
void mp3_wavegame_resume_previous(void);

#define MP3_CMD_PLAY        2
#define MP3_CMD_STOP        3
#define MP3_CMD_PAUSE       4
#define MP3_CMD_RESUME      5
#define MP3_CMD_TOGGLE_MUTE 6
#define MP3_CMD_PLAY_LOOP   7

// Play modes for auto-advance
#define MP3_PLAY_MODE_SINGLE 0
#define MP3_PLAY_MODE_ALL    1
#define MP3_PLAY_MODE_RANDOM 2

// Direct select+play from Core 1 (used by auto-advance)
void mp3_auto_play(const char *path, uint32_t file_size);

uint8_t mp3_get_status(void);
uint16_t mp3_get_elapsed_seconds(void);
uint16_t mp3_get_total_seconds(void);
