#include "mp3.h"
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/audio_i2s.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "ff.h"
#include "mp3dec.h"

// I2S clock pins must be consecutive: clock_pin_base = BCLK, clock_pin_base+1 = LRCLK.
#define MP3_I2S_DATA_PIN 29
#define MP3_I2S_BCLK_PIN 30
#define MP3_I2S_WSEL_PIN 31
#define MP3_I2S_MUTE_PIN 37

#define MP3_BUFFER_FALLBACK 65536
#define MP3_READ_ERROR_LIMIT 8
#define MP3_I2S_BUFFER_SAMPLES 1152
#define MP3_MIN_BUFFER_SIZE 2048
#define MP3_MAX_SAMPLES_PER_CH 1152
#define MP3_PATH_MAX 96
#define TONE_FREQ_HZ 440
#define TONE_DURATION_MS 300
#define DEFAULT_SAMPLE_RATE 44100
#define ESTIMATE_BITRATE_KBPS 128

static struct audio_buffer_pool *audio_pool;
static struct audio_i2s_config i2s_config = {
    .data_pin = MP3_I2S_DATA_PIN,
    .clock_pin_base = MP3_I2S_BCLK_PIN,
    .dma_channel = 0,
    .pio_sm = 0,
};
static bool i2s_ready = false;
static audio_format_t audio_format = {
    .sample_freq = DEFAULT_SAMPLE_RATE,
    .format = AUDIO_BUFFER_FORMAT_PCM_S16,
    .channel_count = 2,
};
static struct audio_buffer_format producer_format = {
    .format = &audio_format,
    .sample_stride = 4,
};

static FIL mp3_file;
static bool file_open = false;
static char mp3_selected_path[MP3_PATH_MAX] = {0};

static volatile uint8_t status_flags = 0;
static volatile uint16_t elapsed_seconds = 0;
static volatile uint16_t total_seconds = 0;
static bool total_seconds_estimated = false;

static uint8_t mp3_buf_fallback[MP3_BUFFER_FALLBACK];
static uint8_t *mp3_buf = mp3_buf_fallback;
static size_t mp3_buf_capacity = MP3_BUFFER_FALLBACK;
static size_t mp3_buf_used = 0;
static size_t mp3_buf_pos = 0;
static HMP3Decoder mp3_decoder = NULL;

static uint32_t mp3_file_size = 0;
static uint32_t mp3_bytes_read = 0;
static uint8_t mp3_read_errors = 0;

static uint32_t sample_rate = DEFAULT_SAMPLE_RATE;
static uint64_t elapsed_samples = 0;

static bool playing = false;
static bool muted = false;
static bool eof = false;
static bool error_flag = false;

static bool tone_active = false;
static uint32_t tone_samples_left = 0;
static uint32_t tone_phase = 0;
static uint32_t tone_phase_inc = 0;
static uint8_t last_status_flags = 0xFF;
void mp3_set_external_buffer(uint8_t *buffer, size_t size) {
    if (!buffer || size < MP3_MIN_BUFFER_SIZE) {
        return;
    }
    mp3_buf = buffer;
    mp3_buf_capacity = size;
    printf("MP3: external buffer size=%lu\n", (unsigned long)mp3_buf_capacity);
}

static int mp3_dma_channel = -1;

static bool i2s_start(void) {
    const struct audio_format *output_format = audio_i2s_setup(&audio_format, &i2s_config);
    if (!output_format || !audio_i2s_connect(audio_pool)) {
        return false;
    }
    audio_i2s_set_enabled(true);
    i2s_ready = true;
    return true;
}

static void i2s_stop(void) {
    if (!i2s_ready) {
        return;
    }
    audio_i2s_set_enabled(false);
    i2s_ready = false;
}

static void update_status_flags(void) {
    uint8_t flags = 0;
    if (playing) flags |= MP3_STATUS_PLAYING;
    if (muted) flags |= MP3_STATUS_MUTED;
    if (error_flag) flags |= MP3_STATUS_ERROR;
    if (eof) flags |= MP3_STATUS_EOF;
    if (tone_active) flags |= MP3_STATUS_TONE;
    status_flags = flags;

    if (status_flags != last_status_flags) {
        printf("MP3: status=0x%02X\n", status_flags);
        last_status_flags = status_flags;
    }
}

static void set_mute(bool enable) {
    muted = enable;
    gpio_put(MP3_I2S_MUTE_PIN, muted ? 1 : 0);
    update_status_flags();
}

static void reset_decoder_state(void) {
    if (mp3_decoder) {
        MP3FreeDecoder(mp3_decoder);
        mp3_decoder = NULL;
    }
    mp3_decoder = MP3InitDecoder();
    mp3_buf_used = 0;
    mp3_buf_pos = 0;
    mp3_bytes_read = 0;
    mp3_read_errors = 0;
    elapsed_samples = 0;
    elapsed_seconds = 0;
    total_seconds = 0;
    total_seconds_estimated = false;
    eof = false;
    error_flag = false;
}

static void close_file(void) {
    if (file_open) {
        f_close(&mp3_file);
        file_open = false;
    }
}

static void fill_buffer_if_needed(void) {
    if (!file_open || eof) return;

    if (mp3_buf_pos > 0 && mp3_buf_pos < mp3_buf_used) {
        memmove(mp3_buf, mp3_buf + mp3_buf_pos, mp3_buf_used - mp3_buf_pos);
        mp3_buf_used -= mp3_buf_pos;
        mp3_buf_pos = 0;
    } else if (mp3_buf_pos >= mp3_buf_used) {
        mp3_buf_used = 0;
        mp3_buf_pos = 0;
    }

    if (mp3_buf_used >= mp3_buf_capacity) return;

    UINT br = 0;
    if (f_read(&mp3_file, mp3_buf + mp3_buf_used, (UINT)(mp3_buf_capacity - mp3_buf_used), &br) == FR_OK) {
        if (br > 0) {
            mp3_buf_used += br;
            mp3_bytes_read += br;
            status_flags |= MP3_STATUS_READING;
            mp3_read_errors = 0;
            if (error_flag) {
                error_flag = false;
            }
        } else {
            eof = true;
        }
    } else {
        if (++mp3_read_errors < MP3_READ_ERROR_LIMIT) {
            return;
        }
        error_flag = true;
        eof = true;
    }
}

static void update_time_counters(void) {
    if (sample_rate > 0) {
        elapsed_seconds = (uint16_t)(elapsed_samples / sample_rate);
    }
}

static int output_pcm_to_i2s(const int16_t *pcm, int output_samps, int channels) {
    if (output_samps <= 0 || channels <= 0) {
        return 0;
    }
    struct audio_buffer *buffer = take_audio_buffer(audio_pool, true);
    if (!buffer) {
        return 0;
    }

    int16_t *dst = (int16_t *)buffer->buffer->bytes;
    if (channels == 1) {
        int frames = output_samps;
        if (frames > MP3_I2S_BUFFER_SAMPLES) {
            frames = MP3_I2S_BUFFER_SAMPLES;
        }
        for (int i = 0; i < frames; i++) {
            int16_t s = pcm[i];
            dst[i * 2] = s;
            dst[i * 2 + 1] = s;
        }
        buffer->sample_count = frames;
    } else {
        int frames = output_samps / channels;
        if (frames > MP3_I2S_BUFFER_SAMPLES) {
            frames = MP3_I2S_BUFFER_SAMPLES;
        }
        memcpy(dst, pcm, (size_t)frames * 2u * sizeof(int16_t));
        buffer->sample_count = frames;
    }

    give_audio_buffer(audio_pool, buffer);
    return buffer->sample_count;
}

static void output_tone(void) {
    struct audio_buffer *buffer = take_audio_buffer(audio_pool, false);
    if (!buffer) return;

    int16_t *dst = (int16_t *)buffer->buffer->bytes;
    int samples = MP3_I2S_BUFFER_SAMPLES;
    if (tone_samples_left < (uint32_t)samples) {
        samples = (int)tone_samples_left;
    }
    for (int i = 0; i < samples; i++) {
        tone_phase += tone_phase_inc;
        int16_t s = (tone_phase & 0x80000000u) ? 12000 : -12000;
        dst[i * 2] = s;
        dst[i * 2 + 1] = s;
    }
    tone_samples_left -= (uint32_t)samples;
    buffer->sample_count = samples;
    give_audio_buffer(audio_pool, buffer);

    if (tone_samples_left == 0) {
        tone_active = false;
    }
}

void mp3_init(void) {
    printf("MP3: init\n");
    if (mp3_dma_channel < 0) {
        for (int ch = 0; ch < NUM_DMA_CHANNELS; ch++) {
            if (!dma_channel_is_claimed(ch)) {
                mp3_dma_channel = ch;
                break;
            }
        }
        if (mp3_dma_channel < 0) {
            printf("MP3: no free DMA channel\n");
            error_flag = true;
            update_status_flags();
            return;
        }
        printf("MP3: dma channel=%d\n", mp3_dma_channel);
    }
    gpio_init(MP3_I2S_MUTE_PIN);
    gpio_set_dir(MP3_I2S_MUTE_PIN, GPIO_OUT);
    set_mute(false);

    audio_pool = audio_new_producer_pool(&producer_format, 4, MP3_I2S_BUFFER_SAMPLES);
    if (!audio_pool) {
        printf("MP3: audio pool alloc failed\n");
        error_flag = true;
        update_status_flags();
        return;
    }

    i2s_config.dma_channel = (uint)mp3_dma_channel;
    if (!i2s_start()) {
        printf("MP3: i2s setup/connect failed\n");
        error_flag = true;
        update_status_flags();
        return;
    }
    reset_decoder_state();
    update_status_flags();
}

void mp3_select_file(const char *path, uint32_t file_size) {
    printf("MP3: select '%s' size=%lu\n", path ? path : "(null)", (unsigned long)file_size);
    playing = false;
    tone_active = false;
    close_file();

    reset_decoder_state();
    mp3_file_size = file_size;

    if (!path || !path[0]) {
        printf("MP3: select failed (empty path)\n");
        error_flag = true;
        update_status_flags();
        return;
    }

    strncpy(mp3_selected_path, path, sizeof(mp3_selected_path) - 1);
    mp3_selected_path[sizeof(mp3_selected_path) - 1] = '\0';
    file_open = false;
    update_status_flags();
}

void mp3_play(void) {
    printf("MP3: play\n");
    if (!file_open && mp3_selected_path[0] != '\0') {
        if (f_open(&mp3_file, mp3_selected_path, FA_READ) != FR_OK) {
            printf("MP3: file open failed\n");
            error_flag = true;
            update_status_flags();
            return;
        }
        UINT br = 0;
        uint8_t hdr[10];
        if (f_read(&mp3_file, hdr, sizeof(hdr), &br) == FR_OK && br == sizeof(hdr)) {
            if (hdr[0] == 'I' && hdr[1] == 'D' && hdr[2] == '3') {
                uint32_t id3_size = ((hdr[6] & 0x7F) << 21) | ((hdr[7] & 0x7F) << 14) | ((hdr[8] & 0x7F) << 7) | (hdr[9] & 0x7F);
                uint32_t skip = id3_size + 10;
                if (hdr[5] & 0x10) {
                    skip += 10;
                }
                if (skip < mp3_file_size) {
                    f_lseek(&mp3_file, skip);
                } else {
                    f_lseek(&mp3_file, 0);
                }
            } else {
                f_lseek(&mp3_file, 0);
            }
        } else {
            f_lseek(&mp3_file, 0);
        }
        file_open = true;
    }
    if (file_open && !error_flag) {
        playing = true;
        eof = false;
        if (total_seconds == 0 && mp3_file_size > 0) {
            uint64_t bits = (uint64_t)mp3_file_size * 8ull;
            uint64_t denom = (uint64_t)ESTIMATE_BITRATE_KBPS * 1000ull;
            total_seconds = (uint16_t)(bits / denom);
            total_seconds_estimated = true;
        }
    }
    update_status_flags();
}

void mp3_stop(void) {
    printf("MP3: stop\n");
    playing = false;
    tone_active = false;
    close_file();
    reset_decoder_state();
    update_status_flags();
}

void mp3_toggle_mute(void) {
    printf("MP3: toggle mute\n");
    set_mute(!muted);
}

void mp3_play_tone(void) {
    printf("MP3: tone\n");
    if (tone_phase_inc == 0 && sample_rate > 0) {
        tone_phase_inc = (uint32_t)(((uint64_t)TONE_FREQ_HZ << 32) / sample_rate);
    }
    tone_active = true;
    tone_samples_left = (DEFAULT_SAMPLE_RATE * TONE_DURATION_MS) / 1000;
    update_status_flags();
}

void mp3_update(void) {
    if (error_flag) {
        update_status_flags();
        return;
    }

    if (tone_active) {
        output_tone();
        update_status_flags();
        return;
    }

    if (!playing || !file_open) {
        update_status_flags();
        return;
    }

    fill_buffer_if_needed();
    size_t available = (mp3_buf_used > mp3_buf_pos) ? (mp3_buf_used - mp3_buf_pos) : 0;
    if (available == 0) {
        if (eof) playing = false;
        update_status_flags();
        return;
    }

    if (!mp3_decoder) {
        mp3_decoder = MP3InitDecoder();
        if (!mp3_decoder) {
            error_flag = true;
            update_status_flags();
            return;
        }
    }

    int attempts = 0;
    while (attempts < 6 && available > 0) {
        size_t base_pos = mp3_buf_pos;
        unsigned char *read_ptr = mp3_buf + mp3_buf_pos;
        int bytes_left = (int)available;
        int offset = MP3FindSyncWord(read_ptr, bytes_left);
        if (offset < 0) {
            mp3_buf_pos = mp3_buf_used;
            break;
        }
        read_ptr += offset;
        bytes_left -= offset;

        int16_t pcm[MP3_MAX_SAMPLES_PER_CH * 2];
        int err = MP3Decode(mp3_decoder, &read_ptr, &bytes_left, pcm, 0);
        if (read_ptr >= mp3_buf && read_ptr <= (mp3_buf + mp3_buf_used)) {
            mp3_buf_pos = (size_t)(read_ptr - mp3_buf);
        }

        if (err == ERR_MP3_INDATA_UNDERFLOW || err == ERR_MP3_MAINDATA_UNDERFLOW) {
            break;
        }
        if (err != ERR_MP3_NONE) {
            mp3_buf_pos = base_pos + (size_t)offset + 1;
            available = (mp3_buf_used > mp3_buf_pos) ? (mp3_buf_used - mp3_buf_pos) : 0;
            attempts++;
            continue;
        }

        MP3FrameInfo info;
        MP3GetLastFrameInfo(mp3_decoder, &info);
        if (info.samprate > 0 && sample_rate != (uint32_t)info.samprate) {
            sample_rate = (uint32_t)info.samprate;
            audio_format.sample_freq = sample_rate;
            if (i2s_ready) {
                i2s_stop();
                if (!i2s_start()) {
                    error_flag = true;
                    update_status_flags();
                    return;
                }
            }
        }
        if (info.bitrate > 0 && mp3_file_size > 0 && (total_seconds == 0 || total_seconds_estimated)) {
            uint64_t bits = (uint64_t)mp3_file_size * 8ull;
            uint64_t denom = (uint64_t)info.bitrate * 1000ull;
            total_seconds = (uint16_t)(bits / denom);
            total_seconds_estimated = false;
        }

        if (info.outputSamps > 0 && info.nChans > 0) {
            int frames_sent = output_pcm_to_i2s(pcm, info.outputSamps, info.nChans);
            if (frames_sent > 0) {
                elapsed_samples += (uint64_t)frames_sent;
                update_time_counters();
            }
        }
        break;
    }

    update_status_flags();
}

uint8_t mp3_get_status(void) {
    return status_flags;
}

uint16_t mp3_get_elapsed_seconds(void) {
    return elapsed_seconds;
}

uint16_t mp3_get_total_seconds(void) {
    return total_seconds;
}
