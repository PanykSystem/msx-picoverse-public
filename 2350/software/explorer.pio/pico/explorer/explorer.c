// MSX PICOVERSE PROJECT
// (c) 2025 Cristiano Goncalves
// The Retro Hacker
//
// explorer.c - PicoVerse 2350 Explorer firmware (ROM loader + SD support)
//
// This work is licensed  under a "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
// License". https://creativecommons.org/licenses/by-nc-sa/4.0/

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "ff.h"
#include "diskio.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/structs/qmi.h"
#include "hardware/regs/qmi.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "hardware/regs/uart.h"
#include "hardware/irq.h"
#include "hw_config.h"
#include "explorer.h"
#include "mapper_detect.h"
#include "mp3.h"
#include "emu2212.h"
#include "emu2149.h"
#include "emu2413.h"
#include "msx_bus.pio.h"
#include "pico/audio_i2s.h"
#include "sunrise_ide.h"
#include "sunrise_sd.h"

// config area and buffer for the ROM data
#define ROM_NAME_MAX    60          // Maximum size of the ROM name
#define MAX_ROM_RECORDS 1024        // Maximum ROM files supported (flash + SD)
#define MAX_FLASH_RECORDS 128       // Maximum ROM files stored in flash
#define ROM_RECORD_SIZE (ROM_NAME_MAX + 1 + (sizeof(uint32_t) * 2)) // Name + mapper + size + offset
#define MENU_ROM_SIZE   (32u * 1024u) // Full menu ROM size stored before config records
#define CONFIG_AREA_SIZE (16u * 1024u)
#define ROM_SELECT_MENU 0xFEu
#define ROM_SELECT_WIFI_CONFIG 0xFFF0u
#define WIFI_CONFIG_FLASH_OFFSET (MENU_ROM_SIZE + CONFIG_AREA_SIZE)
#define WIFI_CONFIG_ROM_SIZE (8u * 1024u)
#define WIFI_BIOS_FLASH_OFFSET (WIFI_CONFIG_FLASH_OFFSET + WIFI_CONFIG_ROM_SIZE)
#define WIFI_BIOS_ROM_SIZE (16u * 1024u)
#define FMPAC_BIOS_FLASH_OFFSET (WIFI_BIOS_FLASH_OFFSET + WIFI_BIOS_ROM_SIZE)
#define FMPAC_BIOS_ROM_SIZE (64u * 1024u)
#define WIFI_CONFIG_RETURN_MENU 0xE0u
#define MONITOR_ADDR    (0xBF7F)    // ROM select register just below the menu control window
#define CACHE_SIZE      (256u * 1024u)     // 256KB cache size for ROM data

#define SOURCE_SD_FLAG  0x80 // Flag in the mapper byte indicating the ROM is on SD
#define FOLDER_FLAG     0x40 // Flag in the mapper byte indicating the record is a folder
#define MP3_FLAG        0x20 // Flag in the mapper byte indicating the record is an MP3 file
#define OVERRIDE_FLAG   0x10 // Flag in the mapper byte indicating manual override

// Temporarily disable the in-menu MP3 player. The MP3 stack is being
// reworked into a dedicated MSX ROM (mp3player.rom) so the menu no
// longer needs to coexist with Core 1 / I2S / SD streaming. While
// that is not in place, MP3 files are skipped during SD enumeration
// and Core 1 is never launched, leaving Explorer free to focus on
// flash and SD ROM execution.
#define EXPLORER_MP3_DISABLED 1

#define FILES_PER_PAGE  19 // Maximum files per page on the menu
#define CTRL_BASE_ADDR  0xBFF0 // Control registers base address
#define CTRL_COUNT_L    (CTRL_BASE_ADDR + 0) // Control: total record count low byte
#define CTRL_COUNT_H    (CTRL_BASE_ADDR + 1) // Control: total record count high byte
#define CTRL_PAGE       (CTRL_BASE_ADDR + 2) // Control: current page index
#define CTRL_STATUS     (CTRL_BASE_ADDR + 3) // Control: status register
#define CTRL_CMD        (CTRL_BASE_ADDR + 4) // Control: command register
#define CTRL_MATCH_L    (CTRL_BASE_ADDR + 5) // Control: match index low byte
#define CTRL_MATCH_H    (CTRL_BASE_ADDR + 6) // Control: match index high byte
#define CTRL_MAPPER     (CTRL_BASE_ADDR + 7) // Control: mapper value
#define CTRL_ACK        (CTRL_BASE_ADDR + 8) // Control: command ack
#define CTRL_MAGIC      0xA5 // Control: magic value to indicate valid command
#define CTRL_QUERY_BASE 0xBFC0 // Control: query string base address
#define CTRL_QUERY_SIZE 32 // Control: query string size
#define CMD_APPLY_FILTER 0x01 // Command: apply filter
#define CMD_FIND_FIRST   0x02 // Command: find first match
#define CMD_ENTER_DIR    0x03 // Command: enter directory
#define CMD_DETECT_MAPPER 0x04 // Command: detect mapper on demand
#define CMD_SET_MAPPER    0x05 // Command: set mapper override
#define CMD_SET_SOURCE    0x06 // Command: switch visible source set
#define CMD_FH_LIST_PAGE   0x40 // Command: File Hunter list page from menu ROM
#define CMD_FH_DOWNLOAD    0x41 // Command: File Hunter download/save from menu ROM
#define CMD_FH_SEARCH      0x42 // Command: File Hunter search from menu ROM
#define CMD_FH_WIFI_STATUS 0x43 // Command: File Hunter WiFi status from menu ROM
#define CMD_FH_WIFI_CONFIG 0x44 // Command: File Hunter WiFi setup from menu ROM

#define FH_CTRL_BASE    0xBFF0
#define FH_COUNT_L      (FH_CTRL_BASE + 0)
#define FH_COUNT_H      (FH_CTRL_BASE + 1)
#define FH_PAGE         (FH_CTRL_BASE + 2)
#define FH_STATUS       (FH_CTRL_BASE + 3)
#define FH_CMD          (FH_CTRL_BASE + 4)
#define FH_SELECT_L     (FH_CTRL_BASE + 5)
#define FH_SELECT_H     (FH_CTRL_BASE + 6)
#define FH_PROGRESS_L   (FH_CTRL_BASE + 7)
#define FH_PROGRESS_H   (FH_CTRL_BASE + 8)
#define FH_RESULT       (FH_CTRL_BASE + 9)
#define FH_QUERY_BASE   0xBFC0
#define FH_QUERY_SIZE   32
#define FH_STATUS_TEXT_BASE 0xBF80
#define FH_STATUS_TEXT_SIZE 64
#define FH_DATA_BASE    0xB900
#define FH_RECORD_SIZE  64
#define FH_FILES_PER_PAGE 19
#define FH_MAX_RESULTS 384
#define FH_HTTP_BUFFER_SIZE (256u * 1024u)
#define FH_HTTP_CHUNK_SIZE 2048u
#define FH_HOST "api.file-hunter.com"
#define FH_DEFAULT_QUERY "1"
#define FH_WIFI_CONNECT_WAIT_US 30000000u
#define FH_STATUS_READY 0xA5
#define FH_STATUS_BUSY  0x5A
#define FH_RESULT_SAVING 0x02
#define FH_CMD_LIST_PAGE 0x01
#define FH_CMD_DOWNLOAD  0x02
#define FH_CMD_EXIT      0x03
#define FH_CMD_SEARCH    0x04
#define FH_CMD_WIFI_STATUS 0x05

#define SOURCE_MODE_ALL   0x00
#define SOURCE_MODE_FLASH 0x01
#define SOURCE_MODE_SD    0x02
#define SOURCE_MODE_FILEHUNTER 0x03

#define CTRL_AUDIO      (CTRL_BASE_ADDR + 9) // Control: audio profile selection
#define CTRL_WIFI_SUPPORT (CTRL_BASE_ADDR + 10) // Control: Sunrise WiFi support (0=No, 1=Yes)

#define AUDIO_PROFILE_NONE       0u
#define AUDIO_PROFILE_SCC        1u
#define AUDIO_PROFILE_SCC_PLUS   2u
#define AUDIO_PROFILE_DUAL_PSG 3u
#define AUDIO_PROFILE_MSX_MUSIC 4u

#define PSG_SAMPLE_RATE 44100
#define PSG_CLOCK       1789773
#define PSG_VOLUME_SHIFT 2
#define PSG_PORT_REG    0x10u
#define PSG_PORT_DATA   0x11u

#define MSX_MUSIC_SAMPLE_RATE 44100
#define MSX_MUSIC_CLOCK       3579545
#define MSX_MUSIC_VOLUME_SHIFT 2
#define MSX_MUSIC_PORT_REG    0x7Cu
#define MSX_MUSIC_PORT_DATA   0x7Du
#define FMPAC_BIOS_SIZE       FMPAC_BIOS_ROM_SIZE

#define WIFI_MEM_F2_ADDR      0x7F05u
#define WIFI_MEM_CMD_ADDR     0x7F06u
#define WIFI_MEM_DATA_ADDR    0x7F07u
#define WIFI_F2_FORCE_SETUP   0xF1u
#define WIFI_RX_FIFO_SIZE     2080u
#define WIFI_RX_SERVICE_BUDGET 32u
#define WIFI_QUICK_WAIT_US    25000u
#define WIFI_UART_DEFAULT_BAUD 859372u
#define WIFI_STATUS_RX_READY  0x01u
#define WIFI_STATUS_TX_BUSY   0x02u
#define WIFI_STATUS_RX_FULL   0x04u
#define WIFI_STATUS_QUICK_RX  0x08u
#define WIFI_STATUS_UNDERRUN  0x10u
#define WIFI_STATUS_FREE_BITS 0x80u
#define WIFI_UART_INSTANCE    uart1

#define DATA_BASE_ADDR   0xB900 // Data buffer base address
#define DATA_BUFFER_SIZE (FH_STATUS_TEXT_BASE - DATA_BASE_ADDR) // Data buffer size
#define DATA_MAGIC_0     'P' // Data header magic bytes
#define DATA_MAGIC_1     'V' // Data header magic bytes
#define DATA_MAGIC_2     'E' // Data header magic bytes
#define DATA_MAGIC_3     'X' // Data header magic bytes
#define DATA_VERSION     1 // Data header version
#define DATA_HEADER_SIZE 24 // Data header size in bytes
#define DATA_RECORD_TABLE_ENTRY_SIZE 4 // Size of each record table entry
#define DATA_RECORD_PAYLOAD_SIZE 13 // Minimum size of each record payload
#define TLV_NAME_OFFSET  0x01 // Type-Length-Value: Name offset
#define TLV_MAPPER       0x02 // Type-Length-Value: Mapper
#define TLV_SIZE         0x03 // Type-Length-Value: Size


// MP3 control registers
#define MP3_CTRL_BASE      0xBFE0 // MP3 control registers base address
#define MP3_CTRL_CMD       (MP3_CTRL_BASE + 0)
#define MP3_CTRL_STATUS    (MP3_CTRL_BASE + 1)
#define MP3_CTRL_INDEX_L   (MP3_CTRL_BASE + 2)
#define MP3_CTRL_INDEX_H   (MP3_CTRL_BASE + 3)
#define MP3_CTRL_ELAPSED_L (MP3_CTRL_BASE + 4)
#define MP3_CTRL_ELAPSED_H (MP3_CTRL_BASE + 5)
#define MP3_CTRL_TOTAL_L   (MP3_CTRL_BASE + 6)
#define MP3_CTRL_TOTAL_H   (MP3_CTRL_BASE + 7)
#define MP3_CTRL_MODE      (MP3_CTRL_BASE + 8)

// "Now Playing" buffer: 60-byte name (space-padded) + 4-byte size (LE)
#define MP3_NOW_PLAYING_BASE 0xBFA0
#define MP3_NOW_PLAYING_SIZE 64

// MSX protocol command code for MP3 file selection (local to explorer.c)
#define MP3_CMD_SELECT      0x01

typedef struct {
    PIO pio;
    uint sm_read;
    uint sm_write;
    uint offset_read;
    uint offset_write;
} msx_pio_bus_t;

typedef struct {
    PIO pio_read;
    PIO pio_write;
    uint sm_io_read;
    uint sm_io_write;
    uint offset_io_read;
    uint offset_io_write;
} msx_pio_io_bus_t;

static msx_pio_bus_t msx_bus;
static bool msx_bus_programs_loaded = false;
static msx_pio_io_bus_t msx_io_bus;
static bool msx_io_bus_programs_loaded = false;

typedef struct {
    uint16_t rom_index;
    bool rom_selected;
} menu_select_ctx_t;

typedef struct {
    uint8_t *bank_regs;
} bank8_ctx_t;

typedef struct {
    uint16_t *bank_regs;
} bank16_ctx_t;

typedef struct {
    uint8_t page;
    uint8_t control;
    uint8_t sram_key_5ffe;
    uint8_t sram_key_5fff;
    uint8_t sram[8192];
} fmpac_state_t;

// This symbol marks the end of the main program in flash.
// Custom data starts right after it
extern const uint8_t __flash_binary_end[];

// PSRAM-backed buffer to cache ROM data
static uint8_t *rom_sram = NULL;
static uint32_t active_rom_size = 0;
static uint32_t rom_cached_size = 0; // Added for PIO implementation
// Effective cache capacity used by prepare_rom_source(). Some mappers (e.g.
// Manbow2) reduce this so the tail of rom_sram can be repurposed (writable
// flash sector emulation), matching multirom's behaviour.
static uint32_t rom_cache_capacity = CACHE_SIZE;
static uint8_t page_buffer[DATA_BUFFER_SIZE];

// pointer to the custom data
static const uint8_t *flash_rom = (const uint8_t *)&__flash_binary_end;
static const uint8_t *rom_data = (const uint8_t *)&__flash_binary_end;
static bool rom_data_in_ram = false;

static uint8_t wifi_rx_fifo[WIFI_RX_FIFO_SIZE];
static volatile uint16_t wifi_rx_head = 0u;
static volatile uint16_t wifi_rx_tail = 0u;
static volatile uint16_t wifi_rx_count = 0u;
static uint8_t wifi_f2_state = 0xFFu;
static bool wifi_uart_ready = false;
static uint32_t wifi_uart_baud = WIFI_UART_DEFAULT_BAUD;
static uint32_t wifi_tx_busy_deadline_us = 0u;
static bool wifi_rx_underrun = false;

static inline void __not_in_flash_func(wifi_reset_fifo)(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    wifi_rx_head = 0u;
    wifi_rx_tail = 0u;
    wifi_rx_count = 0u;
    wifi_rx_underrun = false;
    restore_interrupts(irq_state);
}

static inline bool __not_in_flash_func(wifi_push_rx_byte_locked)(uint8_t data)
{
    if (wifi_rx_count >= WIFI_RX_FIFO_SIZE) return false;
    wifi_rx_fifo[wifi_rx_head] = data;
    wifi_rx_head = (uint16_t)((wifi_rx_head + 1u) % WIFI_RX_FIFO_SIZE);
    ++wifi_rx_count;
    return true;
}

static inline bool __not_in_flash_func(wifi_push_rx_byte)(uint8_t data)
{
    uint32_t irq_state = save_and_disable_interrupts();
    bool pushed = wifi_push_rx_byte_locked(data);
    restore_interrupts(irq_state);
    return pushed;
}

static inline bool __not_in_flash_func(wifi_pop_rx_byte)(uint8_t *data_out)
{
    uint32_t irq_state = save_and_disable_interrupts();
    if (wifi_rx_count == 0u)
    {
        restore_interrupts(irq_state);
        return false;
    }
    *data_out = wifi_rx_fifo[wifi_rx_tail];
    wifi_rx_tail = (uint16_t)((wifi_rx_tail + 1u) % WIFI_RX_FIFO_SIZE);
    --wifi_rx_count;
    restore_interrupts(irq_state);
    return true;
}

static inline uint16_t __not_in_flash_func(wifi_rx_count_snapshot)(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    uint16_t count = wifi_rx_count;
    restore_interrupts(irq_state);
    return count;
}

static inline bool __not_in_flash_func(wifi_hw_tx_busy)(void)
{
    return (uart_get_hw(WIFI_UART_INSTANCE)->fr & UART_UARTFR_BUSY_BITS) != 0u;
}

static inline void __not_in_flash_func(wifi_hw_rx_drain)(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    while (uart_is_readable(WIFI_UART_INSTANCE))
    {
        (void)uart_getc(WIFI_UART_INSTANCE);
    }
    restore_interrupts(irq_state);
}

static inline void __not_in_flash_func(wifi_pio_rx_reset)(void)
{
    wifi_hw_rx_drain();
}

static inline void __not_in_flash_func(wifi_pio_tx_reset)(void)
{
    while (wifi_hw_tx_busy()) { }
    wifi_tx_busy_deadline_us = 0u;
}

static inline void __not_in_flash_func(wifi_service_rx)(void)
{
    uint16_t budget = WIFI_RX_SERVICE_BUDGET;
    uint32_t irq_state;

    if (!wifi_uart_ready) return;
    irq_state = save_and_disable_interrupts();
    while (budget-- != 0u && uart_is_readable(WIFI_UART_INSTANCE))
    {
        if (!wifi_push_rx_byte_locked((uint8_t)uart_getc(WIFI_UART_INSTANCE))) break;
    }
    restore_interrupts(irq_state);
}

static void __not_in_flash_func(wifi_uart_rx_irq)(void)
{
    while (uart_is_readable(WIFI_UART_INSTANCE))
    {
        uint8_t data = (uint8_t)uart_getc(WIFI_UART_INSTANCE);
        (void)wifi_push_rx_byte_locked(data);
    }
}

static inline uint32_t __not_in_flash_func(wifi_uart_frame_time_us)(void)
{
    return (10000000u + (wifi_uart_baud - 1u)) / wifi_uart_baud;
}

static inline void __not_in_flash_func(wifi_uart_init_once)(void)
{
    if (wifi_uart_ready) return;

    (void)uart_init(WIFI_UART_INSTANCE, wifi_uart_baud);
    gpio_set_function(PIN_ESP_UART_TX, GPIO_FUNC_UART_AUX);
    gpio_set_function(PIN_ESP_UART_RX, GPIO_FUNC_UART_AUX);
    uart_set_format(WIFI_UART_INSTANCE, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(WIFI_UART_INSTANCE, false, false);
    uart_set_fifo_enabled(WIFI_UART_INSTANCE, true);

    wifi_hw_rx_drain();
    wifi_f2_state = 0xFFu;
    wifi_reset_fifo();
    wifi_tx_busy_deadline_us = 0u;
    wifi_uart_ready = true;

    int uart_irq = (WIFI_UART_INSTANCE == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(uart_irq, wifi_uart_rx_irq);
    irq_set_enabled(uart_irq, true);
    uart_set_irq_enables(WIFI_UART_INSTANCE, true, false);
}

static inline void __not_in_flash_func(wifi_handle_cmd_write)(uint8_t cmd)
{
    wifi_uart_init_once();
    if (cmd == 20u)
    {
        wifi_reset_fifo();
        wifi_pio_tx_reset();
        wifi_pio_rx_reset();
    }
}

static inline uint8_t __not_in_flash_func(wifi_status_read)(void)
{
    wifi_uart_init_once();
    wifi_service_rx();
    uint8_t status = 0u;
    uint16_t rx_count = wifi_rx_count_snapshot();
    if (rx_count != 0u) status |= WIFI_STATUS_RX_READY;
    if (rx_count >= WIFI_RX_FIFO_SIZE) status |= WIFI_STATUS_RX_FULL;
    if (wifi_hw_tx_busy() || !uart_is_writable(WIFI_UART_INSTANCE) ||
        (int32_t)(time_us_32() - wifi_tx_busy_deadline_us) < 0)
    {
        status |= WIFI_STATUS_TX_BUSY;
    }
    if (wifi_rx_underrun)
    {
        status |= WIFI_STATUS_UNDERRUN;
        wifi_rx_underrun = false;
    }
    return status;
}

static inline uint8_t __not_in_flash_func(wifi_data_read)(void)
{
    wifi_uart_init_once();

    uint8_t data;
    wifi_service_rx();
    if (wifi_pop_rx_byte(&data)) return data;

    uint32_t deadline = time_us_32() + WIFI_QUICK_WAIT_US;
    while ((int32_t)(time_us_32() - deadline) < 0)
    {
        wifi_service_rx();
        if (wifi_pop_rx_byte(&data)) return data;
    }

    wifi_rx_underrun = true;
    return 0xFFu;
}

static inline bool __not_in_flash_func(wifi_handle_mem_write)(uint16_t addr, uint8_t data)
{
    if (addr == WIFI_MEM_F2_ADDR) { wifi_f2_state = data; return true; }
    if (addr == WIFI_MEM_CMD_ADDR) { wifi_handle_cmd_write(data); return true; }
    if (addr == WIFI_MEM_DATA_ADDR)
    {
        wifi_uart_init_once();
        uart_putc_raw(WIFI_UART_INSTANCE, (char)data);
        wifi_tx_busy_deadline_us = time_us_32() + wifi_uart_frame_time_us();
        return true;
    }
    return false;
}

static inline uint8_t __not_in_flash_func(wifi_handle_mem_read)(const uint8_t *wifi_rom_base, uint32_t wifi_rom_size, uint16_t addr)
{
    if (addr == WIFI_MEM_F2_ADDR) return wifi_f2_state;
    if (addr == WIFI_MEM_CMD_ADDR) return wifi_data_read();
    if (addr == WIFI_MEM_DATA_ADDR) return wifi_status_read();
    if (addr >= 0x4000u && addr <= 0x7FFFu)
    {
        uint32_t rel = (uint32_t)(addr - 0x4000u);
        if (rel < wifi_rom_size) return wifi_rom_base[rel];
    }
    return 0xFFu;
}

// External PSRAM region used to buffer SD-loaded ROMs. Defined later
// alongside the PSRAM bring-up code; declared here for use by earlier
// functions such as load_rom_from_sd().
static psram_region_t sd_rom_region;
static psram_region_t rom_cache_region;
static psram_region_t mapper_region;
static psram_region_t fh_list_region;
static psram_region_t fh_download_region;
static bool psram_bring_up_once(void);

uint8_t ctr_val = 0xFF;    
BYTE const pdrv = 0;  // Physical drive number
DSTATUS ds = 1; // Disk status (1 = not initialized)

// Structure to represent a ROM record
// The ROM record will contain the name of the ROM, the mapper code, the size of the ROM and the offset in the flash memory
// Name: ROM_NAME_MAX bytes
// Mapper: 1 byte
// Size: 4 bytes
// Offset: 4 bytes
typedef struct {
    char Name[ROM_NAME_MAX];
    unsigned char Mapper;
    unsigned long Size;
    unsigned long Offset;
} ROMRecord;

ROMRecord records[MAX_ROM_RECORDS]; // Array to store the ROM records

#define SD_PATH_MAX        96
#define SD_PATH_BUFFER_SIZE 19000
#define MIN_ROM_SIZE       8192
#define MAX_ROM_SIZE       (15u * 1024u * 1024u)
#define SD_ROM_MAX_SIZE    (4u * 1024u * 1024u) // PSRAM region capacity for SD-loaded ROMs
#define MAPPER_SUNRISE_USB        10
#define MAPPER_SUNRISE_MAPPER_USB 11
#define MAPPER_SYSTEM             MAPPER_SUNRISE_USB
#define MAPPER_SUNRISE_SD         15
#define MAPPER_SUNRISE_MAPPER_SD  16

static const char *MAPPER_DESCRIPTIONS[] = {
    "PLA-16", "PLA-32", "KonSCC", "PLN-48", "ASC-08",
    "ASC-16", "Konami", "NEO-8", "NEO-16", "SYSTEM",
    "SYSTEM", "ASC16X", "PLN-64", "MANBW2"
};

#define MAPPER_DESCRIPTION_COUNT (sizeof(MAPPER_DESCRIPTIONS) / sizeof(MAPPER_DESCRIPTIONS[0]))

static char sd_path_buffer[SD_PATH_BUFFER_SIZE];
static uint16_t sd_path_offsets[MAX_ROM_RECORDS];
static uint16_t sd_path_buffer_used = 0;
static uint16_t sd_record_count = 0;
static bool sd_mounted = false;
static sd_card_t *sd_card = NULL;
static uint16_t total_record_count = 0;
static uint16_t full_record_count = 0;
static uint8_t current_page = 0;
static uint16_t filtered_indices[MAX_ROM_RECORDS];
static char filter_query[CTRL_QUERY_SIZE];
static volatile uint8_t ctrl_cmd_state = 0;
static volatile uint8_t ctrl_mapper_value = 0;
static volatile uint8_t ctrl_ack_value = 0;
static uint16_t match_index = 0xFFFF;
static ROMRecord flash_records[MAX_FLASH_RECORDS];
static uint16_t flash_record_count = 0;
static char sd_current_path[SD_PATH_MAX] = "/";
static uint8_t browse_source_mode = SOURCE_MODE_FLASH;
static volatile bool refresh_requested = false;
static volatile bool refresh_in_progress = false;
static bool refresh_worker_started = false;
static volatile bool detect_mapper_pending = false;

// Chunked directory scan state machine (yields back to MP3 loop between chunks)
#define REFRESH_CHUNK_SIZE 8
typedef enum {
    REFRESH_IDLE = 0,
    REFRESH_INIT,
    REFRESH_SCAN_FOLDERS,
    REFRESH_SORT_FOLDERS,
    REFRESH_SCAN_FILES,
    REFRESH_FINALIZE
} refresh_state_t;
static refresh_state_t refresh_state = REFRESH_IDLE;
static DIR refresh_dir;
static uint16_t refresh_record_index = 0;
static uint16_t refresh_folder_count = 0;
static bool refresh_has_parent = false;
static volatile uint16_t detect_mapper_index = 0;
static volatile uint16_t mp3_selected_index = 0;
static volatile bool mp3_pending_select = false;
static volatile uint16_t mp3_pending_index = 0;
static volatile uint8_t mp3_pending_cmd = 0;
static volatile uint8_t mp3_play_mode = MP3_PLAY_MODE_SINGLE;
static uint16_t mp3_playing_filtered_index = 0xFFFF;
static uint8_t mp3_now_playing_buf[MP3_NOW_PLAYING_SIZE];
static volatile uint8_t ctrl_audio_selection = AUDIO_PROFILE_NONE;
static volatile uint8_t ctrl_wifi_support = 0; // 0=disabled, 1=enabled for Sunrise Nextor launches

static uint8_t fh_page_index = 0;
static uint8_t fh_status = FH_STATUS_READY;
static uint8_t fh_result = 0;
static uint16_t fh_progress_percent = 0;
static uint8_t fh_query[FH_QUERY_SIZE];
static uint16_t fh_selected_index = 0;
static char fh_wifi_status_text[FH_STATUS_TEXT_SIZE];
static uint8_t fh_tcp_last_error = 0;
static uint32_t fh_download_size = 0;
static char fh_download_name[ROM_NAME_MAX + 1];

#define FH_SAVE_IDLE 0u
#define FH_SAVE_RUNNING 1u
#define FH_SAVE_DONE 2u
#define FH_SAVE_FAILED 3u

static uint8_t fh_sd_save_chunk[1024];
static volatile uint8_t fh_save_state = FH_SAVE_IDLE;
static FIL fh_save_file;
static uint32_t fh_save_offset = 0;
static bool fh_save_file_open = false;
static bool fh_save_close_pending = false;
static volatile bool fh_menu_window_active = false;
static volatile bool fh_service_menu_window = false;

typedef struct {
    char name[ROM_NAME_MAX + 1];
    uint16_t size_kb;
} fh_catalog_record_t;

static fh_catalog_record_t fh_catalog[FH_MAX_RESULTS];
static uint16_t fh_catalog_count = 0;
static bool fh_catalog_loaded = false;
static uint8_t fh_tcp_buffer[FH_HTTP_CHUNK_SIZE + 2u];
static char fh_active_query[FH_QUERY_SIZE];

// SCC emulation state + I2S audio
#define SCC_VOLUME_SHIFT 2  // Left-shift SCC output for volume boost (4x)
#define SCC_AUDIO_BUFFER_SAMPLES 256
static SCC scc_instance;
static struct audio_buffer_pool *scc_audio_pool;

static PSG psg_instance;
static struct audio_buffer_pool *dual_psg_audio_pool;
static bool dual_psg_ready = false;
static bool dual_psg_audio_started = false;

static OPLL *msx_music_instance = NULL;
static spin_lock_t *msx_music_lock = NULL;
static struct audio_buffer_pool *msx_music_audio_pool;
static bool msx_music_ready = false;
static bool msx_music_audio_started = false;

typedef enum {
    AUDIO_MODE_NONE = 0,
    AUDIO_MODE_SCC,
    AUDIO_MODE_SCC_PLUS,
    AUDIO_MODE_DUAL_PSG,
    AUDIO_MODE_MSX_MUSIC,
} audio_mode_t;

static const char *EXCLUDED_SD_FOLDERS[] = {
    "System Volume Information"
};
static const size_t EXCLUDED_SD_FOLDER_COUNT = sizeof(EXCLUDED_SD_FOLDERS) / sizeof(EXCLUDED_SD_FOLDERS[0]);

static void write_u32_le(uint8_t *ptr, uint32_t value);
static void write_u16_le(uint8_t *ptr, uint16_t value);
static void build_page_buffer(uint8_t page_index);
static void refresh_records_for_current_path(void);

static bool is_system_mapper(uint8_t mapper) {
    return mapper == MAPPER_SUNRISE_USB ||
           mapper == MAPPER_SUNRISE_MAPPER_USB ||
           mapper == MAPPER_SUNRISE_SD ||
           mapper == MAPPER_SUNRISE_MAPPER_SD;
}

static bool is_audio_system_mapper(uint8_t mapper) {
    return mapper == 9u || is_system_mapper(mapper);
}

static bool mapper_supports_scc_audio(uint8_t mapper) {
    return mapper == 3u || mapper == 14u;
}

static audio_mode_t resolve_audio_mode(uint8_t mapper, uint8_t requested_profile) {
    if (is_audio_system_mapper(mapper)) {
        return AUDIO_MODE_NONE;
    }
    if (mapper_supports_scc_audio(mapper)) {
        if (requested_profile == AUDIO_PROFILE_SCC_PLUS) {
            return AUDIO_MODE_SCC_PLUS;
        }
        if (requested_profile == AUDIO_PROFILE_SCC) {
            return AUDIO_MODE_SCC;
        }
        return AUDIO_MODE_NONE;
    }
    if (requested_profile == AUDIO_PROFILE_DUAL_PSG) {
        return AUDIO_MODE_DUAL_PSG;
    }
    if (requested_profile == AUDIO_PROFILE_MSX_MUSIC) {
        return AUDIO_MODE_MSX_MUSIC;
    }
    return AUDIO_MODE_NONE;
}

static uint8_t mapper_code_from_record_byte(uint8_t mapper) {
    uint8_t raw = (uint8_t)(mapper & ~(SOURCE_SD_FLAG | FOLDER_FLAG | MP3_FLAG));

    if (raw == OVERRIDE_FLAG) {
        return MAPPER_SUNRISE_MAPPER_SD;
    }

    if (raw & OVERRIDE_FLAG) {
        return (uint8_t)(raw & (uint8_t)~OVERRIDE_FLAG);
    }

    return raw;
}

static int compare_record_names(const ROMRecord *a, const ROMRecord *b) {
    return strncmp(a->Name, b->Name, ROM_NAME_MAX);
}

static bool is_system_record(const ROMRecord *record) {
    return is_system_mapper(mapper_code_from_record_byte(record->Mapper));
}

static bool is_folder_record(const ROMRecord *record) {
    return (record->Mapper & FOLDER_FLAG) != 0;
}

static bool record_matches_source_mode(const ROMRecord *record) {
    switch (browse_source_mode) {
        case SOURCE_MODE_FLASH:
            return !is_folder_record(record) && ((record->Mapper & SOURCE_SD_FLAG) == 0);
        case SOURCE_MODE_SD:
            return is_folder_record(record) || ((record->Mapper & SOURCE_SD_FLAG) != 0);
        default:
            return true;
    }
}

static size_t trim_name_copy(char *dest, const char *src) {
    size_t len = 0;
    for (size_t i = 0; i < ROM_NAME_MAX; i++) {
        dest[i] = src[i];
    }
    dest[ROM_NAME_MAX] = '\0';
    len = ROM_NAME_MAX;
    while (len > 0 && dest[len - 1] == ' ') {
        dest[len - 1] = '\0';
        len--;
    }
    return len;
}

static bool contains_ignore_case(const char *text, const char *query) {
    if (!query || query[0] == '\0') {
        return true;
    }
    size_t text_len = strlen(text);
    size_t query_len = strlen(query);
    if (query_len == 0 || query_len > text_len) {
        return false;
    }
    for (size_t i = 0; i + query_len <= text_len; i++) {
        size_t j = 0;
        while (j < query_len) {
            char a = text[i + j];
            char b = query[j];
            if (toupper((unsigned char)a) != toupper((unsigned char)b)) {
                break;
            }
            j++;
        }
        if (j == query_len) {
            return true;
        }
    }
    return false;
}

static void apply_filter(void) {
    total_record_count = 0;
    if (filter_query[0] == '\0') {
        for (uint16_t i = 0; i < full_record_count; i++) {
            if (record_matches_source_mode(&records[i])) {
                filtered_indices[total_record_count++] = i;
            }
        }
        return;
    }

    char name_buf[ROM_NAME_MAX + 1];
    for (uint16_t i = 0; i < full_record_count; i++) {
        if (!record_matches_source_mode(&records[i])) {
            continue;
        }
        trim_name_copy(name_buf, records[i].Name);
        if (contains_ignore_case(name_buf, filter_query)) {
            filtered_indices[total_record_count++] = i;
        }
    }
}

static void find_first_match(void) {
    match_index = 0xFFFF;
    if (filter_query[0] == '\0') {
        return;
    }

    char name_buf[ROM_NAME_MAX + 1];
    for (uint16_t i = 0; i < total_record_count; i++) {
        uint16_t record_index = filtered_indices[i];
        if (record_index >= full_record_count) {
            continue;
        }
        trim_name_copy(name_buf, records[record_index].Name);
        if (contains_ignore_case(name_buf, filter_query)) {
            match_index = i;
            return;
        }
    }
}

static void swap_records(uint16_t a, uint16_t b) {
    if (a == b) {
        return;
    }
    ROMRecord temp = records[a];
    records[a] = records[b];
    records[b] = temp;
    uint16_t path_temp = sd_path_offsets[a];
    sd_path_offsets[a] = sd_path_offsets[b];
    sd_path_offsets[b] = path_temp;
}

static void sort_records_range(uint16_t start, uint16_t count) {
    if (count < 2) {
        return;
    }
    uint16_t end = (uint16_t)(start + count);
    for (uint16_t i = start; i + 1 < end; i++) {
        for (uint16_t j = i + 1; j < end; j++) {
            if (compare_record_names(&records[i], &records[j]) > 0) {
                swap_records(i, j);
            }
        }
    }
}

static void write_config_area(uint8_t *config_area, const ROMRecord *list, uint16_t count) {
    for (uint16_t i = 0; i < count; i++) {
        uint8_t *entry = config_area + (i * ROM_RECORD_SIZE);
        memset(entry, 0xFF, ROM_RECORD_SIZE);
        memset(entry, ' ', ROM_NAME_MAX);
        size_t name_len = strlen(list[i].Name);
        if (name_len > ROM_NAME_MAX) {
            name_len = ROM_NAME_MAX;
        }
        memcpy(entry, list[i].Name, name_len);
        entry[ROM_NAME_MAX] = list[i].Mapper;
        write_u32_le(entry + ROM_NAME_MAX + 1, (uint32_t)list[i].Size);
        write_u32_le(entry + ROM_NAME_MAX + 1 + sizeof(uint32_t), (uint32_t)list[i].Offset);
    }

    if (count < MAX_ROM_RECORDS) {
        memset(config_area + (count * ROM_RECORD_SIZE), 0xFF,
               (MAX_ROM_RECORDS - count) * ROM_RECORD_SIZE);
    }
}

static void build_page_buffer(uint8_t page_index) {
    uint8_t *buffer = page_buffer;
    const uint16_t buffer_size = DATA_BUFFER_SIZE;
    uint16_t start_index = (uint16_t)page_index * FILES_PER_PAGE;
    uint16_t record_count = 0;

    if (start_index < total_record_count) {
        uint16_t remaining = (uint16_t)(total_record_count - start_index);
        record_count = (remaining > FILES_PER_PAGE) ? FILES_PER_PAGE : remaining;
    }

    memset(buffer, 0xFF, buffer_size);

    uint16_t table_offset = DATA_HEADER_SIZE;
    uint16_t table_size = (uint16_t)(record_count * DATA_RECORD_TABLE_ENTRY_SIZE);
    uint16_t payload_size = (uint16_t)(record_count * DATA_RECORD_PAYLOAD_SIZE);
    uint16_t string_pool_offset = (uint16_t)(table_offset + table_size);
    uint16_t max_string_pool = 0;
    if ((uint32_t)string_pool_offset + payload_size <= buffer_size) {
        max_string_pool = (uint16_t)(buffer_size - (string_pool_offset + payload_size));
    }

    uint16_t name_offsets[FILES_PER_PAGE];
    uint16_t string_cursor = 0;
    for (uint16_t i = 0; i < record_count; i++) {
        uint16_t filtered_index = (uint16_t)(start_index + i);
        uint16_t record_index = filtered_indices[filtered_index];
        char name_buf[ROM_NAME_MAX + 1];
        size_t name_len = trim_name_copy(name_buf, records[record_index].Name);
        if (name_len + 1 <= (size_t)(max_string_pool - string_cursor)) {
            name_offsets[i] = string_cursor;
            memcpy(buffer + string_pool_offset + string_cursor, name_buf, name_len);
            buffer[string_pool_offset + string_cursor + name_len] = '\0';
            string_cursor = (uint16_t)(string_cursor + name_len + 1);
        } else {
            name_offsets[i] = 0xFFFF;
        }
    }

    uint16_t string_pool_size = string_cursor;
    uint16_t payload_offset = (uint16_t)(string_pool_offset + string_pool_size);
    uint16_t payload_cursor = 0;

    for (uint16_t i = 0; i < record_count; i++) {
        uint16_t filtered_index = (uint16_t)(start_index + i);
        uint16_t record_index = filtered_indices[filtered_index];
        uint8_t *entry = buffer + table_offset + (i * DATA_RECORD_TABLE_ENTRY_SIZE);
        uint16_t record_offset = (uint16_t)(payload_offset + payload_cursor);
        uint16_t record_length = DATA_RECORD_PAYLOAD_SIZE;

        write_u16_le(entry, record_offset);
        write_u16_le(entry + 2, record_length);

        uint8_t *rec = buffer + record_offset;
        rec[0] = TLV_NAME_OFFSET;
        rec[1] = 2;
        write_u16_le(rec + 2, name_offsets[i]);
        rec[4] = TLV_MAPPER;
        rec[5] = 1;
        rec[6] = records[record_index].Mapper;
        rec[7] = TLV_SIZE;
        rec[8] = 4;
        write_u32_le(rec + 9, (uint32_t)records[record_index].Size);

        payload_cursor = (uint16_t)(payload_cursor + record_length);
    }

    buffer[0] = DATA_MAGIC_0;
    buffer[1] = DATA_MAGIC_1;
    buffer[2] = DATA_MAGIC_2;
    buffer[3] = DATA_MAGIC_3;
    buffer[4] = DATA_VERSION;
    buffer[5] = 0;
    write_u16_le(buffer + 6, DATA_HEADER_SIZE);
    write_u16_le(buffer + 8, total_record_count);
    write_u16_le(buffer + 10, page_index);
    write_u16_le(buffer + 12, record_count);
    write_u16_le(buffer + 14, table_offset);
    write_u16_le(buffer + 16, table_size);
    write_u16_le(buffer + 18, string_pool_offset);
    write_u16_le(buffer + 20, string_pool_size);
    write_u16_le(buffer + 22, payload_offset);
}

static bool equals_ignore_case(const char *a, const char *b) {
    if (!a || !b) {
        return false;
    }
    while (*a && *b) {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

static bool is_excluded_folder(const char *name) {
    if (!name || name[0] == '\0') {
        return true;
    }
    if (equals_ignore_case(name, ".") || equals_ignore_case(name, "..")) {
        return true;
    }
    for (size_t i = 0; i < EXCLUDED_SD_FOLDER_COUNT; i++) {
        if (equals_ignore_case(name, EXCLUDED_SD_FOLDERS[i])) {
            return true;
        }
    }
    return false;
}

static bool is_root_path(const char *path) {
    if (!path || path[0] == '\0') {
        return true;
    }
    if (path[0] == '/' && path[1] == '\0') {
        return true;
    }
    return false;
}

static void set_root_path(void) {
    sd_current_path[0] = '/';
    sd_current_path[1] = '\0';
}

static void go_up_one_level(void) {
    if (is_root_path(sd_current_path)) {
        set_root_path();
        return;
    }

    size_t len = strlen(sd_current_path);
    if (len == 0) {
        set_root_path();
        return;
    }

    while (len > 1 && sd_current_path[len - 1] == '/') {
        sd_current_path[len - 1] = '\0';
        len--;
    }

    char *slash = strrchr(sd_current_path, '/');
    if (!slash || slash == sd_current_path) {
        set_root_path();
    } else {
        *slash = '\0';
    }
}

static void append_folder_to_path(const char *folder) {
    if (!folder || folder[0] == '\0') {
        return;
    }
    if (equals_ignore_case(folder, "..")) {
        go_up_one_level();
        return;
    }

    size_t base_len = strlen(sd_current_path);
    size_t folder_len = strlen(folder);
    if (base_len + folder_len + 2 >= sizeof(sd_current_path)) {
        return;
    }

    if (!is_root_path(sd_current_path)) {
        sd_current_path[base_len] = '/';
        base_len++;
    }
    memcpy(sd_current_path + base_len, folder, folder_len + 1);
}

static void write_u32_le(uint8_t *ptr, uint32_t value) {
    ptr[0] = (uint8_t)(value & 0xFFu);
    ptr[1] = (uint8_t)((value >> 8) & 0xFFu);
    ptr[2] = (uint8_t)((value >> 16) & 0xFFu);
    ptr[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static void write_u16_le(uint8_t *ptr, uint16_t value) {
    ptr[0] = (uint8_t)(value & 0xFFu);
    ptr[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static const char *basename_from_path(const char *path) {
    if (!path) {
        return "";
    }
    const char *slash = strrchr(path, '/');
    return slash ? (slash + 1) : path;
}

static uint8_t mapper_number_from_filename(const char *filename) {
    char name_copy[SD_PATH_MAX];
    strncpy(name_copy, filename, sizeof(name_copy));
    name_copy[sizeof(name_copy) - 1] = '\0';

    char *dot = strrchr(name_copy, '.');
    if (dot) {
        *dot = '\0';
    }

    size_t name_len = strlen(name_copy);
    for (size_t i = 0; i + 1 < MAPPER_DESCRIPTION_COUNT; ++i) {
        const char *tag = MAPPER_DESCRIPTIONS[i];
        size_t tag_len = strlen(tag);
        if (name_len > tag_len + 1 && name_copy[name_len - tag_len - 1] == '.') {
            if (equals_ignore_case(name_copy + name_len - tag_len, tag)) {
                return (uint8_t)(i + 1);
            }
        }
    }
    return 0;
}

static void build_display_name(const char *filename, char *out, size_t out_size) {
    char name_copy[SD_PATH_MAX];
    strncpy(name_copy, filename, sizeof(name_copy));
    name_copy[sizeof(name_copy) - 1] = '\0';

    char *dot = strrchr(name_copy, '.');
    if (dot) {
        *dot = '\0';
    }

    size_t name_len = strlen(name_copy);
    for (size_t i = 0; i + 1 < MAPPER_DESCRIPTION_COUNT; ++i) {
        const char *tag = MAPPER_DESCRIPTIONS[i];
        size_t tag_len = strlen(tag);
        if (name_len > tag_len + 1 && name_copy[name_len - tag_len - 1] == '.') {
            if (equals_ignore_case(name_copy + name_len - tag_len, tag)) {
                name_copy[name_len - tag_len - 1] = '\0';
                break;
            }
        }
    }

    strncpy(out, name_copy, out_size);
    out[out_size - 1] = '\0';
}

static bool has_rom_extension(const char *filename) {
    const char *dot = strrchr(filename, '.');
    if (!dot || dot[1] == '\0') {
        return false;
    }
    return equals_ignore_case(dot + 1, "ROM");
}

static bool has_mp3_extension(const char *filename) {
    const char *dot = strrchr(filename, '.');
    if (!dot || dot[1] == '\0') {
        return false;
    }
    return equals_ignore_case(dot + 1, "MP3");
}

// FatFS-backed reader callback for the shared streaming detector.
typedef struct {
    FIL *fil;
    uint32_t cur_offset; // current file pointer (best-effort cache to avoid redundant f_lseek)
    bool failed;
} fatfs_reader_ctx_t;

static uint32_t fatfs_reader_cb(void *user, uint32_t offset, void *buf, uint32_t len) {
    fatfs_reader_ctx_t *ctx = (fatfs_reader_ctx_t *)user;
    if (ctx->failed || !ctx->fil) {
        return 0;
    }
    if (offset != ctx->cur_offset) {
        if (f_lseek(ctx->fil, offset) != FR_OK) {
            ctx->failed = true;
            return 0;
        }
        ctx->cur_offset = offset;
    }
    UINT br = 0;
    if (f_read(ctx->fil, buf, (UINT)len, &br) != FR_OK) {
        ctx->failed = true;
        return 0;
    }
    ctx->cur_offset += br;
    return (uint32_t)br;
}

// Stream a ROM file from the SD card through the shared mapper detector.
// Uses the same SHA1 + heuristic algorithm as the PC tool so that the firmware
// produces identical mapper guesses for ROMs executed straight from microSD.
static uint8_t detect_rom_type_from_file(const char *path, uint32_t size) {
    if (size > MAX_ROM_SIZE || size < MIN_ROM_SIZE) {
        return 0;
    }

    FIL fil;
    FRESULT fr = f_open(&fil, path, FA_READ);
    if (fr != FR_OK) {
        return 0;
    }

    fatfs_reader_ctx_t ctx = { .fil = &fil, .cur_offset = 0, .failed = false };
    uint8_t mapper = mapper_detect_stream(size, fatfs_reader_cb, &ctx);
    f_close(&fil);
    if (ctx.failed) {
        return 0;
    }
    return mapper;
}

static bool sd_mount_card(void) {
    if (sd_mounted) {
        return true;
    }

    if (!sd_init_driver()) {
        return false;
    }

    sd_card = sd_get_by_num(0);
    if (!sd_card) {
        return false;
    }

    FRESULT fr = f_mount(&sd_card->state.fatfs, "", 1);
    if (fr != FR_OK) {
        return false;
    }

    sd_mounted = true;
    return true;
}

static bool sd_is_mounted(void) {
    return sd_mounted && sd_card != NULL;
}

static uint16_t add_folder_record(uint16_t record_index, const char *name) {
    if (record_index >= MAX_ROM_RECORDS) {
        return record_index;
    }
    memset(records[record_index].Name, 0, sizeof(records[record_index].Name));
    strncpy(records[record_index].Name, name, sizeof(records[record_index].Name) - 1);
    records[record_index].Mapper = (unsigned char)(FOLDER_FLAG | SOURCE_SD_FLAG);
    records[record_index].Size = 0;
    records[record_index].Offset = 0;
    sd_path_offsets[record_index] = 0xFFFF;
    return (uint16_t)(record_index + 1);
}

static uint16_t add_sd_rom_record(uint16_t record_index, const char *path, const char *filename, uint32_t size, uint8_t mapper) {
    if (record_index >= MAX_ROM_RECORDS) {
        return record_index;
    }
    size_t path_len = strlen(path);
    if (sd_path_buffer_used + path_len + 1 > SD_PATH_BUFFER_SIZE) {
        return record_index;
    }
    sd_path_offsets[record_index] = sd_path_buffer_used;
    memcpy(sd_path_buffer + sd_path_buffer_used, path, path_len + 1);
    sd_path_buffer_used = (uint16_t)(sd_path_buffer_used + path_len + 1);

    char display_name[ROM_NAME_MAX + 1];
    build_display_name(filename, display_name, sizeof(display_name));

    memset(records[record_index].Name, 0, sizeof(records[record_index].Name));
    strncpy(records[record_index].Name, display_name, sizeof(records[record_index].Name) - 1);
    records[record_index].Mapper = (unsigned char)(mapper | SOURCE_SD_FLAG);
    records[record_index].Size = (unsigned long)size;
    records[record_index].Offset = (unsigned long)record_index;
    return (uint16_t)(record_index + 1);
}

static uint16_t add_sd_mp3_record(uint16_t record_index, const char *path, const char *filename, uint32_t size) {
    if (record_index >= MAX_ROM_RECORDS) {
        return record_index;
    }
    size_t path_len = strlen(path);
    if (sd_path_buffer_used + path_len + 1 > SD_PATH_BUFFER_SIZE) {
        return record_index;
    }
    sd_path_offsets[record_index] = sd_path_buffer_used;
    memcpy(sd_path_buffer + sd_path_buffer_used, path, path_len + 1);
    sd_path_buffer_used = (uint16_t)(sd_path_buffer_used + path_len + 1);

    char display_name[ROM_NAME_MAX + 1];
    build_display_name(filename, display_name, sizeof(display_name));

    memset(records[record_index].Name, 0, sizeof(records[record_index].Name));
    strncpy(records[record_index].Name, display_name, sizeof(records[record_index].Name) - 1);
    records[record_index].Mapper = (unsigned char)(MP3_FLAG | SOURCE_SD_FLAG);
    records[record_index].Size = (unsigned long)size;
    records[record_index].Offset = (unsigned long)record_index;
    return (uint16_t)(record_index + 1);
}

static void refresh_records_for_current_path(void) {
    sd_record_count = 0;
    sd_path_buffer_used = 0;
    for (uint16_t i = 0; i < MAX_ROM_RECORDS; i++) {
        sd_path_offsets[i] = 0xFFFF;
    }

    uint16_t record_index = 0;
    bool has_parent = !is_root_path(sd_current_path);

    if (has_parent) {
        record_index = add_folder_record(record_index, "..");
    }

    uint16_t folder_count = record_index;
    if (sd_mount_card()) {
        DIR dir;
        FILINFO fno;
        FRESULT fr = f_opendir(&dir, sd_current_path);
        if (fr == FR_OK) {
            while (record_index < MAX_ROM_RECORDS) {
                fr = f_readdir(&dir, &fno);
                if (fr != FR_OK || fno.fname[0] == '\0') {
                    break;
                }
                if (!(fno.fattrib & AM_DIR)) {
                    continue;
                }
                if (is_excluded_folder(fno.fname)) {
                    continue;
                }
                record_index = add_folder_record(record_index, fno.fname);
            }
        }
        f_closedir(&dir);

        folder_count = record_index;
        uint16_t folder_sort_start = has_parent ? 1u : 0u;
        if (folder_count > folder_sort_start) {
            sort_records_range(folder_sort_start, (uint16_t)(folder_count - folder_sort_start));
        }

        fr = f_opendir(&dir, sd_current_path);
        if (fr == FR_OK) {
            while (record_index < MAX_ROM_RECORDS) {
                fr = f_readdir(&dir, &fno);
                if (fr != FR_OK || fno.fname[0] == '\0') {
                    break;
                }
                if (fno.fattrib & AM_DIR) {
                    continue;
                }
                bool is_mp3 = has_mp3_extension(fno.fname);
                bool is_rom = has_rom_extension(fno.fname);
#if EXPLORER_MP3_DISABLED
                // MP3 player temporarily disabled — skip MP3 entries.
                if (is_mp3) continue;
#endif

                if (!is_mp3 && !is_rom) {
                    continue;
                }

                char path[SD_PATH_MAX];
                if (is_root_path(sd_current_path)) {
                    snprintf(path, sizeof(path), "/%s", fno.fname);
                } else {
                    snprintf(path, sizeof(path), "%s/%s", sd_current_path, fno.fname);
                }

                if (is_mp3) {
                    if (fno.fsize == 0) {
                        continue;
                    }
                    record_index = add_sd_mp3_record(record_index, path, fno.fname, (uint32_t)fno.fsize);
                    sd_record_count++;
                    continue;
                }

                if (fno.fsize < MIN_ROM_SIZE || fno.fsize > MAX_ROM_SIZE) {
                    continue;
                }
                if (fno.fsize > SD_ROM_MAX_SIZE) {
                    continue;
                }

                uint8_t mapper = mapper_number_from_filename(fno.fname);
                record_index = add_sd_rom_record(record_index, path, fno.fname, (uint32_t)fno.fsize, mapper);
                sd_record_count++;
            }
        }
        f_closedir(&dir);
    }

    if (is_root_path(sd_current_path) && flash_record_count > 0) {
        for (uint16_t i = 0; i < flash_record_count && record_index < MAX_ROM_RECORDS; i++) {
            records[record_index] = flash_records[i];
            sd_path_offsets[record_index] = 0xFFFF;
            record_index++;
        }
    }

    uint16_t rom_count = (uint16_t)(record_index - folder_count);
    if (rom_count > 1) {
        uint16_t system_insert = folder_count;
        for (uint16_t i = folder_count; i < record_index; i++) {
            if (is_system_record(&records[i])) {
                swap_records(i, system_insert);
                system_insert++;
            }
        }

        uint16_t system_count = (uint16_t)(system_insert - folder_count);
        uint16_t non_system_count = (uint16_t)(record_index - system_insert);
        if (non_system_count > 1) {
            sort_records_range(system_insert, non_system_count);
        }
    }

    full_record_count = record_index;
    memset(filter_query, 0, sizeof(filter_query));
    apply_filter();
    current_page = 0;
    build_page_buffer(current_page);
}

// Chunked version of refresh_records_for_current_path.
// Processes up to REFRESH_CHUNK_SIZE SD reads per call, then returns false
// to yield back to the MP3 decode loop. Returns true when done (or idle).
static bool refresh_records_chunked(void) {
    FILINFO fno;
    FRESULT fr;

    switch (refresh_state) {
    case REFRESH_IDLE:
        return true;

    case REFRESH_INIT:
        sd_record_count = 0;
        sd_path_buffer_used = 0;
        for (uint16_t i = 0; i < MAX_ROM_RECORDS; i++) {
            sd_path_offsets[i] = 0xFFFF;
        }
        refresh_record_index = 0;
        refresh_has_parent = !is_root_path(sd_current_path);
        if (refresh_has_parent) {
            refresh_record_index = add_folder_record(refresh_record_index, "..");
        }
        refresh_folder_count = refresh_record_index;
        if (!sd_mount_card()) {
            refresh_state = REFRESH_FINALIZE;
            return false;
        }
        fr = f_opendir(&refresh_dir, sd_current_path);
        if (fr != FR_OK) {
            refresh_state = REFRESH_FINALIZE;
            return false;
        }
        refresh_state = REFRESH_SCAN_FOLDERS;
        return false;

    case REFRESH_SCAN_FOLDERS: {
        int reads = 0;
        while (reads < REFRESH_CHUNK_SIZE && refresh_record_index < MAX_ROM_RECORDS) {
            fr = f_readdir(&refresh_dir, &fno);
            reads++;
            if (fr != FR_OK || fno.fname[0] == '\0') {
                f_closedir(&refresh_dir);
                refresh_state = REFRESH_SORT_FOLDERS;
                return false;
            }
            if (!(fno.fattrib & AM_DIR)) continue;
            if (is_excluded_folder(fno.fname)) continue;
            refresh_record_index = add_folder_record(refresh_record_index, fno.fname);
        }
        if (refresh_record_index >= MAX_ROM_RECORDS) {
            f_closedir(&refresh_dir);
            refresh_state = REFRESH_SORT_FOLDERS;
        }
        return false;
    }

    case REFRESH_SORT_FOLDERS: {
        refresh_folder_count = refresh_record_index;
        uint16_t folder_sort_start = refresh_has_parent ? 1u : 0u;
        if (refresh_folder_count > folder_sort_start) {
            sort_records_range(folder_sort_start, (uint16_t)(refresh_folder_count - folder_sort_start));
        }
        fr = f_opendir(&refresh_dir, sd_current_path);
        if (fr != FR_OK) {
            refresh_state = REFRESH_FINALIZE;
            return false;
        }
        refresh_state = REFRESH_SCAN_FILES;
        return false;
    }

    case REFRESH_SCAN_FILES: {
        int reads = 0;
        while (reads < REFRESH_CHUNK_SIZE && refresh_record_index < MAX_ROM_RECORDS) {
            fr = f_readdir(&refresh_dir, &fno);
            reads++;
            if (fr != FR_OK || fno.fname[0] == '\0') {
                f_closedir(&refresh_dir);
                refresh_state = REFRESH_FINALIZE;
                return false;
            }
            if (fno.fattrib & AM_DIR) continue;
            bool is_mp3 = has_mp3_extension(fno.fname);
            bool is_rom = has_rom_extension(fno.fname);
#if EXPLORER_MP3_DISABLED
            // MP3 player temporarily disabled — skip MP3 entries.
            if (is_mp3) continue;
#endif
            if (!is_mp3 && !is_rom) continue;

            char path[SD_PATH_MAX];
            if (is_root_path(sd_current_path)) {
                snprintf(path, sizeof(path), "/%s", fno.fname);
            } else {
                snprintf(path, sizeof(path), "%s/%s", sd_current_path, fno.fname);
            }

            if (is_mp3) {
                if (fno.fsize == 0) continue;
                refresh_record_index = add_sd_mp3_record(refresh_record_index, path, fno.fname, (uint32_t)fno.fsize);
                sd_record_count++;
                continue;
            }

            if (fno.fsize < MIN_ROM_SIZE || fno.fsize > MAX_ROM_SIZE) continue;
            if (fno.fsize > SD_ROM_MAX_SIZE) continue;

            uint8_t mapper = mapper_number_from_filename(fno.fname);
            refresh_record_index = add_sd_rom_record(refresh_record_index, path, fno.fname, (uint32_t)fno.fsize, mapper);
            sd_record_count++;
        }
        if (refresh_record_index >= MAX_ROM_RECORDS) {
            f_closedir(&refresh_dir);
            refresh_state = REFRESH_FINALIZE;
        }
        return false;
    }

    case REFRESH_FINALIZE: {
        if (is_root_path(sd_current_path) && flash_record_count > 0) {
            for (uint16_t i = 0; i < flash_record_count && refresh_record_index < MAX_ROM_RECORDS; i++) {
                records[refresh_record_index] = flash_records[i];
                sd_path_offsets[refresh_record_index] = 0xFFFF;
                refresh_record_index++;
            }
        }

        uint16_t rom_count = (uint16_t)(refresh_record_index - refresh_folder_count);
        if (rom_count > 1) {
            uint16_t system_insert = refresh_folder_count;
            for (uint16_t i = refresh_folder_count; i < refresh_record_index; i++) {
                if (is_system_record(&records[i])) {
                    swap_records(i, system_insert);
                    system_insert++;
                }
            }
            uint16_t system_count = (uint16_t)(system_insert - refresh_folder_count);
            uint16_t non_system_count = (uint16_t)(refresh_record_index - system_insert);
            if (non_system_count > 1) {
                sort_records_range(system_insert, non_system_count);
            }
        }

        full_record_count = refresh_record_index;
        memset(filter_query, 0, sizeof(filter_query));
        apply_filter();
        current_page = 0;
        build_page_buffer(current_page);
        refresh_state = REFRESH_IDLE;
        return true;
    }

    default:
        refresh_state = REFRESH_IDLE;
        return true;
    }
}

static void process_detect_mapper_request(uint16_t filtered_index) {
    ctrl_mapper_value = 0;

    if (!sd_mount_card()) {
        ctrl_cmd_state = 0;
        return;
    }

    if (filtered_index >= total_record_count) {
        ctrl_cmd_state = 0;
        return;
    }

    uint16_t record_index = filtered_indices[filtered_index];
    if (record_index >= full_record_count) {
        ctrl_cmd_state = 0;
        return;
    }

    ROMRecord *rec = &records[record_index];
    uint8_t flags = rec->Mapper & (SOURCE_SD_FLAG | FOLDER_FLAG | MP3_FLAG);

    if (!(flags & SOURCE_SD_FLAG) || (flags & (FOLDER_FLAG | MP3_FLAG)) || sd_path_offsets[record_index] == 0xFFFF) {
        ctrl_mapper_value = mapper_code_from_record_byte(rec->Mapper);
        ctrl_cmd_state = 0;
        return;
    }

    const char *path = sd_path_buffer + sd_path_offsets[record_index];
    const char *filename = basename_from_path(path);

    uint8_t mapper = mapper_number_from_filename(filename);
    if (mapper == 0) {
        mapper = detect_rom_type_from_file(path, (uint32_t)rec->Size);
    }

    ctrl_mapper_value = mapper;
    if (mapper != 0) {
        rec->Mapper = (uint8_t)(flags | mapper);
    } else {
        rec->Mapper = flags;
    }

    ctrl_cmd_state = 0;
}

// Populate the now-playing buffer with name + size from a record.
static void update_mp3_now_playing(uint16_t record_index) {
    if (record_index >= full_record_count) return;
    const ROMRecord *rec = &records[record_index];
    // Copy name (space-padded to ROM_NAME_MAX)
    memcpy(mp3_now_playing_buf, rec->Name, ROM_NAME_MAX);
    // Write size as 4-byte LE at offset 60
    uint32_t sz = (uint32_t)rec->Size;
    mp3_now_playing_buf[ROM_NAME_MAX + 0] = (uint8_t)(sz & 0xFFu);
    mp3_now_playing_buf[ROM_NAME_MAX + 1] = (uint8_t)((sz >> 8) & 0xFFu);
    mp3_now_playing_buf[ROM_NAME_MAX + 2] = (uint8_t)((sz >> 16) & 0xFFu);
    mp3_now_playing_buf[ROM_NAME_MAX + 3] = (uint8_t)((sz >> 24) & 0xFFu);
}

// Find the next MP3 filtered index for auto-advance.
// mode: MP3_PLAY_MODE_ALL = sequential wrap, MP3_PLAY_MODE_RANDOM = random pick.
static uint16_t find_next_mp3_filtered_index(uint16_t current, uint8_t mode) {
    if (total_record_count == 0) return 0xFFFF;

    if (mode == MP3_PLAY_MODE_RANDOM) {
        // Count MP3 entries
        uint16_t mp3_count = 0;
        for (uint16_t i = 0; i < total_record_count; i++) {
            uint16_t ri = filtered_indices[i];
            if (ri < full_record_count && (records[ri].Mapper & MP3_FLAG))
                mp3_count++;
        }
        if (mp3_count == 0) return 0xFFFF;

        // Use microsecond timer as entropy for random pick
        uint16_t target = (uint16_t)(time_us_32() % mp3_count);
        uint16_t ord = 0;
        for (uint16_t i = 0; i < total_record_count; i++) {
            uint16_t ri = filtered_indices[i];
            if (ri < full_record_count && (records[ri].Mapper & MP3_FLAG)) {
                if (ord == target) return i;
                ord++;
            }
        }
        return 0xFFFF;
    }

    // ALL mode: find next MP3 after current, wrapping around
    uint16_t start = (current != 0xFFFF && current + 1 < total_record_count)
                     ? (current + 1) : 0;
    for (uint16_t count = 0; count < total_record_count; count++) {
        uint16_t i = (start + count) % total_record_count;
        uint16_t ri = filtered_indices[i];
        if (ri < full_record_count && (records[ri].Mapper & MP3_FLAG)) {
            return i;
        }
    }
    return 0xFFFF;
}

// Lazy non-blocking launch of Core 1 + MP3 stack. Called from
// core1_bg_work() the first time the MSX dispatches an MP3 command.
// We deliberately do NOT spin Core 0 waiting for Core 1 to finish
// mp3_init() — Core 0 is servicing the MSX PIO bus and any block
// here freezes the cartridge. The cross-core command queue is a
// small ring buffer so SELECT and PLAY (issued ~10 ms apart) both
// fit while Core 1 is still initialising.
static bool mp3_core1_started = false;
static void ensure_mp3_core1_started(void) {
#if EXPLORER_MP3_DISABLED
    // MP3 player disabled: never launch Core 1 from the menu, so the
    // SD card stays single-owner on Core 0 with no risk of cross-core
    // FatFS / I2S / DMA contention.
    return;
#else
    if (mp3_core1_started) return;
    mp3_core1_started = true;
    multicore_launch_core1(mp3_core1_loop);
#endif
}

// Background work pump. Runs on Core 0 from the menu loop idle branch
// (no PIO traffic from the MSX). Single-core ownership of the SD card
// avoids FatFS reentrancy issues and removes the freeze observed when
// folder navigation raced with MP3 work on Core 1.
static void fh_save_background_work(void);
static void core1_bg_work(void) {
    if (fh_save_state == FH_SAVE_RUNNING) {
        fh_save_background_work();
        return;
    }

    // Bridge MP3 commands queued by handle_menu_write_explorer() to the
    // mp3.c command queue. Lazy-launch Core 1 the first time an MP3
    // command actually arrives.
    if (mp3_pending_select) {
        uint16_t pending_index = mp3_pending_index;
        mp3_pending_select = false;
        if (pending_index < total_record_count) {
            uint16_t record_index = filtered_indices[pending_index];
            if (record_index < full_record_count) {
                ROMRecord const *rec = &records[record_index];
                if ((rec->Mapper & MP3_FLAG) && (sd_path_offsets[record_index] != 0xFFFF)) {
                    const char *path = sd_path_buffer + sd_path_offsets[record_index];
                    printf("MP3: select path=%s size=%lu\n", path, (unsigned long)rec->Size);
                    ensure_mp3_core1_started();
                    mp3_select_file(path, (uint32_t)rec->Size);
                    mp3_playing_filtered_index = pending_index;
                    update_mp3_now_playing(record_index);
                } else {
                    printf("MP3: select ignored (not mp3 or no path)\n");
                }
            } else {
                printf("MP3: select index out of range\n");
            }
        } else {
            printf("MP3: select index invalid (total=%u)\n", (unsigned int)total_record_count);
        }
    }
    if (mp3_pending_cmd != 0) {
        uint8_t cmd = mp3_pending_cmd;
        mp3_pending_cmd = 0;
        ensure_mp3_core1_started();
        mp3_send_cmd(cmd);
    }

    // Auto-advance: when current track ends and mode is ALL or RANDOM,
    // automatically select and play the next track.
    if (mp3_play_mode != MP3_PLAY_MODE_SINGLE) {
        uint8_t st = mp3_get_status();
        if ((st & MP3_STATUS_EOF) && !(st & MP3_STATUS_PLAYING)) {
            uint16_t next = find_next_mp3_filtered_index(
                mp3_playing_filtered_index, mp3_play_mode);
            if (next != 0xFFFF && next < total_record_count) {
                uint16_t ri = filtered_indices[next];
                if (ri < full_record_count &&
                    (records[ri].Mapper & MP3_FLAG) &&
                    sd_path_offsets[ri] != 0xFFFF) {
                    const char *path = sd_path_buffer + sd_path_offsets[ri];
                    printf("MP3: auto-advance to index %u path=%s\n",
                           (unsigned)next, path);
                    mp3_auto_play(path, (uint32_t)records[ri].Size);
                    mp3_playing_filtered_index = next;
                    mp3_selected_index = next;
                    update_mp3_now_playing(ri);
                }
            }
        }
    }

    // Defer SD-heavy work while MP3 is actively playing to avoid
    // SPI bus contention between directory reads and MP3 file reads.
    if (mp3_get_status() & MP3_STATUS_PLAYING) {
        return;
    }

    // Chunked directory refresh — process one chunk then yield
    if (refresh_state != REFRESH_IDLE) {
        bool done = refresh_records_chunked();
        if (done) {
            refresh_in_progress = false;
            ctrl_cmd_state = 0;
        }
        return;
    }

    if (refresh_requested) {
        refresh_requested = false;
        refresh_in_progress = true;
        refresh_state = REFRESH_INIT;
        return;
    }

    if (detect_mapper_pending && !refresh_in_progress) {
        uint16_t pending_index = detect_mapper_index;
        detect_mapper_pending = false;
        process_detect_mapper_request(pending_index);
    }
}

static bool load_rom_from_sd(uint16_t record_index, uint32_t size) {
    if (!sd_mount_card()) {
        return false;
    }
    if (record_index >= MAX_ROM_RECORDS) {
        return false;
    }
    if (sd_path_offsets[record_index] == 0xFFFF) {
        return false;
    }

    // SD ROMs are streamed into the 4MB PSRAM region so rom_sram remains
    // available for flash-resident mapper caching. Bring PSRAM up lazily.
    if (!psram_bring_up_once()) {
        return false;
    }
    if (size > sd_rom_region.size) {
        return false;
    }

    const char *path = sd_path_buffer + sd_path_offsets[record_index];

    FIL fil;
    FRESULT fr = f_open(&fil, path, FA_READ);
    if (fr != FR_OK) {
        return false;
    }

    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 0);

    uint8_t *dst = sd_rom_region.ptr;
    memset(dst, 0, size);

    UINT total = 0;
    while (total < size) {
        UINT to_read = (UINT)((size - total) > 4096 ? 4096 : (size - total));
        UINT br = 0;
        fr = f_read(&fil, dst + total, to_read, &br);
        if (fr != FR_OK || br == 0) {
            break;
        }
        total += br;
    }

    f_close(&fil);
    gpio_put(PIN_WAIT, 1);

    return total == size;
}

// Initialize GPIO pins
static inline void setup_gpio()
{

    for (int i = 0; i <= 23; i++) {
        gpio_init(i);
        gpio_set_input_hysteresis_enabled(i, true);
    }

    for (int i = 0; i <= 15; i++) {
        gpio_set_dir(i, GPIO_IN);
    }

    // Initialize control pins as input
    gpio_init(PIN_RD); gpio_set_dir(PIN_RD, GPIO_IN);    
    gpio_init(PIN_WR); gpio_set_dir(PIN_WR, GPIO_IN); 
    gpio_init(PIN_IORQ); gpio_set_dir(PIN_IORQ, GPIO_IN); 
    gpio_init(PIN_SLTSL); gpio_set_dir(PIN_SLTSL, GPIO_IN); 
    gpio_init(PIN_BUSSDIR); gpio_set_dir(PIN_BUSSDIR, GPIO_IN); 
}

// read_ulong - Read a 4-byte value from the memory area
// This function will read a 4-byte value from the memory area pointed by ptr and return the value as an unsigned long
// Parameters:
//   ptr - Pointer to the memory area to read the value from
// Returns:
//   The 4-byte value as an unsigned long 
unsigned long __no_inline_not_in_flash_func(read_ulong)(const unsigned char *ptr) {
    return (unsigned long)ptr[0] |
           ((unsigned long)ptr[1] << 8) |
           ((unsigned long)ptr[2] << 16) |
           ((unsigned long)ptr[3] << 24);
}

// isEndOfData - Check if the memory area is the end of the data
// This function will check if the memory area pointed by memory is the end of the data. The end of the data is defined by all bytes being 0xFF.
// Parameters:
//   memory - Pointer to the memory area to check
// Returns:
//   1 if the memory area is the end of the data, 0 otherwise
int __no_inline_not_in_flash_func(isEndOfData)(const unsigned char *memory) {
    for (int i = 0; i < ROM_RECORD_SIZE; i++) {
        if (memory[i] != 0xFF) {
            return 0;
        }
    }
    return 1;
}

// -----------------------------------------------------------------------
// External PSRAM (QMI CS1) - hardware init + bump allocator.
// Mirrors the implementation in multirom so the same code paths work
// across the two firmwares. Used for SD-cached ROM storage so that
// rom_sram remains fully available for flash-resident mapper caching.
// -----------------------------------------------------------------------
static inline void __not_in_flash_func(psram_delay_cycles)(uint32_t cycles)
{
    for (volatile uint32_t cycle = 0; cycle < cycles; ++cycle)
    {
        __asm volatile ("nop");
    }
}

static inline void __not_in_flash_func(psram_wait_direct_done)(void)
{
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_TXEMPTY_BITS) == 0) { }
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) { }
}

static inline void __not_in_flash_func(psram_send_direct_cmd)(uint8_t cmd, bool quad_width)
{
    qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
    if (quad_width)
    {
        qmi_hw->direct_tx = QMI_DIRECT_TX_OE_BITS |
                            (QMI_DIRECT_TX_IWIDTH_VALUE_Q << QMI_DIRECT_TX_IWIDTH_LSB) |
                            cmd;
    }
    else
    {
        qmi_hw->direct_tx = cmd;
    }
    psram_wait_direct_done();
    qmi_hw->direct_csr &= ~QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
    (void)qmi_hw->direct_rx;
}

static bool __no_inline_not_in_flash_func(psram_init)(void)
{
    gpio_set_function(PIN_PSRAM, GPIO_FUNC_XIP_CS1);

    uint32_t irq_state = save_and_disable_interrupts();

    qmi_hw->direct_csr = (30u << QMI_DIRECT_CSR_CLKDIV_LSB) | QMI_DIRECT_CSR_EN_BITS;
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) { }

    // Warm restart path: if PSRAM remained powered it may still be in QPI mode.
    psram_send_direct_cmd(0xF5u, true);
    psram_delay_cycles(128u);

    psram_send_direct_cmd(0x66u, false);
    psram_delay_cycles(128u);
    psram_send_direct_cmd(0x99u, false);
    psram_delay_cycles(50000u);
    psram_send_direct_cmd(0x35u, false);
    psram_delay_cycles(128u);
    psram_send_direct_cmd(0xC0u, false);
    psram_delay_cycles(128u);

    qmi_hw->direct_csr &= ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS | QMI_DIRECT_CSR_EN_BITS);

    // PSRAM QMI timing - tuned for 210 MHz system clock (CLKDIV=2 -> ~52.5 MHz).
    qmi_hw->m[1].timing =
        (QMI_M0_TIMING_PAGEBREAK_VALUE_1024 << QMI_M0_TIMING_PAGEBREAK_LSB) |
        (1u << QMI_M0_TIMING_SELECT_HOLD_LSB) |
        (1u << QMI_M0_TIMING_COOLDOWN_LSB) |
        (2u << QMI_M0_TIMING_RXDELAY_LSB) |
        (26u << QMI_M0_TIMING_MAX_SELECT_LSB) |
        (5u << QMI_M0_TIMING_MIN_DESELECT_LSB) |
        (2u << QMI_M0_TIMING_CLKDIV_LSB);
    qmi_hw->m[1].rfmt =
        (QMI_M0_RFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_PREFIX_WIDTH_LSB) |
        (QMI_M0_RFMT_ADDR_WIDTH_VALUE_Q << QMI_M0_RFMT_ADDR_WIDTH_LSB) |
        (QMI_M0_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_SUFFIX_WIDTH_LSB) |
        (QMI_M0_RFMT_DUMMY_WIDTH_VALUE_Q << QMI_M0_RFMT_DUMMY_WIDTH_LSB) |
        (QMI_M0_RFMT_DUMMY_LEN_VALUE_24 << QMI_M0_RFMT_DUMMY_LEN_LSB) |
        (QMI_M0_RFMT_DATA_WIDTH_VALUE_Q << QMI_M0_RFMT_DATA_WIDTH_LSB) |
        (QMI_M0_RFMT_PREFIX_LEN_VALUE_8 << QMI_M0_RFMT_PREFIX_LEN_LSB) |
        (QMI_M0_RFMT_SUFFIX_LEN_VALUE_NONE << QMI_M0_RFMT_SUFFIX_LEN_LSB);
    qmi_hw->m[1].rcmd = (0xEBu << QMI_M0_RCMD_PREFIX_LSB);
    qmi_hw->m[1].wfmt =
        (QMI_M0_WFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_PREFIX_WIDTH_LSB) |
        (QMI_M0_WFMT_ADDR_WIDTH_VALUE_Q << QMI_M0_WFMT_ADDR_WIDTH_LSB) |
        (QMI_M0_WFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_SUFFIX_WIDTH_LSB) |
        (QMI_M0_WFMT_DUMMY_WIDTH_VALUE_Q << QMI_M0_WFMT_DUMMY_WIDTH_LSB) |
        (QMI_M0_WFMT_DUMMY_LEN_VALUE_NONE << QMI_M0_WFMT_DUMMY_LEN_LSB) |
        (QMI_M0_WFMT_DATA_WIDTH_VALUE_Q << QMI_M0_WFMT_DATA_WIDTH_LSB) |
        (QMI_M0_WFMT_PREFIX_LEN_VALUE_8 << QMI_M0_WFMT_PREFIX_LEN_LSB) |
        (QMI_M0_WFMT_SUFFIX_LEN_VALUE_NONE << QMI_M0_WFMT_SUFFIX_LEN_LSB);
    qmi_hw->m[1].wcmd = (0x38u << QMI_M0_WCMD_PREFIX_LSB);

    xip_ctrl_hw->ctrl |= XIP_CTRL_WRITABLE_M1_BITS;

    restore_interrupts(irq_state);

    // Verify PSRAM via uncached window (bypass XIP cache).
    volatile uint32_t *psram_test = (volatile uint32_t *)0x15000000u;
    psram_test[0] = 0x12345678u;
    return psram_test[0] == 0x12345678u;
}

// -----------------------------------------------------------------------
// PSRAM Memory Manager - bump allocator for 8MB external PSRAM.
// -----------------------------------------------------------------------
static struct {
    uint32_t next_free;
    bool initialised;
} psram_mgr;

static void psram_mem_init(void)
{
    psram_mgr.next_free = 0;
    memset(&sd_rom_region, 0, sizeof(sd_rom_region));
    memset(&rom_cache_region, 0, sizeof(rom_cache_region));
    rom_sram = NULL;
    memset(&mapper_region, 0, sizeof(mapper_region));
    memset(&fh_list_region, 0, sizeof(fh_list_region));
    memset(&fh_download_region, 0, sizeof(fh_download_region));
    fh_download_size = 0;
    memset(fh_download_name, 0, sizeof(fh_download_name));
}

static bool psram_alloc(uint32_t size, psram_region_t *region)
{
    if (psram_mgr.next_free + size > PSRAM_TOTAL_SIZE)
        return false;
    region->offset = psram_mgr.next_free;
    region->size   = size;
    region->ptr    = (uint8_t *)(PSRAM_BASE_ADDR + psram_mgr.next_free);
    psram_mgr.next_free += size;
    return true;
}

// One-time PSRAM bring-up + region carve-out. Idempotent. Returns false if
// the hardware probe fails or any allocation cannot be satisfied.
static bool psram_bring_up_once(void)
{
    if (psram_mgr.initialised) return true;
    if (!psram_init()) return false;
    psram_mem_init();
    // 4MB region for SD-loaded ROMs.
    if (!psram_alloc(SD_ROM_MAX_SIZE, &sd_rom_region)) return false;
    psram_mgr.initialised = true;
    return true;
}

static bool psram_prepare_mapper_region(void)
{
    if (!psram_bring_up_once()) return false;
    if (mapper_region.size == MAPPER_SIZE) return true;
    return psram_alloc(MAPPER_SIZE, &mapper_region);
}

static bool psram_prepare_rom_cache(void)
{
    if (!psram_bring_up_once()) return false;
    if (rom_cache_region.size == CACHE_SIZE)
    {
        rom_sram = rom_cache_region.ptr;
        return true;
    }
    if (!psram_alloc(CACHE_SIZE, &rom_cache_region)) return false;
    rom_sram = rom_cache_region.ptr;
    return true;
}

static bool fh_prepare_list_region(void)
{
    if (!psram_bring_up_once()) return false;
    if (fh_list_region.ptr && fh_list_region.size >= FH_HTTP_BUFFER_SIZE) return true;
    return psram_alloc(FH_HTTP_BUFFER_SIZE, &fh_list_region);
}

static inline void __not_in_flash_func(prepare_rom_source)(
    uint32_t offset,
    bool cache_enable,
    uint32_t preferred_size,
    const uint8_t **rom_base_out,
    uint32_t *available_length_out)
{
    const uint8_t *rom_base = rom_data + offset;
    uint32_t available_length = active_rom_size;

    // For SD-loaded ROMs rom_data points at PSRAM (sd_rom_region.ptr) with
    // offset 0; for flash-resident ROMs it points at QSPI flash. In both
    // cases we copy the leading rom_cache_capacity bytes into the PSRAM ROM
    // cache, leaving the tail to be served from the source via XIP.
    if (preferred_size != 0u && (available_length == 0u || available_length > preferred_size))
    {
        available_length = preferred_size;
    }

    if (cache_enable && available_length > 0u && psram_prepare_rom_cache())
    {
        uint32_t cap = rom_cache_capacity;
        if (cap > CACHE_SIZE) cap = CACHE_SIZE;
        uint32_t bytes_to_cache = (available_length > cap) ? cap : available_length;

        gpio_init(PIN_WAIT);
        gpio_set_dir(PIN_WAIT, GPIO_OUT);
        gpio_put(PIN_WAIT, 0);

        // DMA bulk copy from flash XIP to SRAM (matches multirom). A
        // CPU memcpy() can stall or get truncated for large transfers
        // when this function runs in the PSRAM/QMI-active path.
        int dma_chan = dma_claim_unused_channel(true);
        dma_channel_config dma_cfg = dma_channel_get_default_config(dma_chan);
        channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
        channel_config_set_read_increment(&dma_cfg, true);
        channel_config_set_write_increment(&dma_cfg, true);
        dma_channel_configure(dma_chan, &dma_cfg,
            rom_sram,
            rom_base,
            bytes_to_cache,
            true);
        dma_channel_wait_for_finish_blocking(dma_chan);
        dma_channel_unclaim(dma_chan);
        gpio_put(PIN_WAIT, 1);

        rom_cached_size = bytes_to_cache;
        // If we cached everything, we can just point to SRAM
        if (active_rom_size <= cap)
        {
            rom_base = rom_sram;
        }
    }
    else
    {
        rom_cached_size = 0;
    }

    *rom_base_out = rom_base;
    *available_length_out = available_length;
}

static inline void __not_in_flash_func(prepare_sunrise_mapper_rom_source)(
    uint32_t offset,
    bool cache_enable,
    const uint8_t **rom_base_out,
    uint32_t *available_length_out)
{
    const uint8_t *rom_base = rom_data + offset;
    uint32_t available_length = active_rom_size;

    if (cache_enable && available_length > 0u && psram_prepare_rom_cache())
    {
        uint32_t bytes_to_cache = (available_length > CACHE_SIZE)
                                  ? CACHE_SIZE
                                  : available_length;

        int dma_chan = dma_claim_unused_channel(true);
        dma_channel_config dma_cfg = dma_channel_get_default_config(dma_chan);
        channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
        channel_config_set_read_increment(&dma_cfg, true);
        channel_config_set_write_increment(&dma_cfg, true);
        dma_channel_configure(dma_chan, &dma_cfg,
            rom_sram,
            rom_base,
            bytes_to_cache,
            true);
        dma_channel_wait_for_finish_blocking(dma_chan);
        dma_channel_unclaim(dma_chan);

        rom_cached_size = bytes_to_cache;
        if (available_length <= CACHE_SIZE)
        {
            rom_base = rom_sram;
        }
    }
    else
    {
        rom_cached_size = 0;
    }

    *rom_base_out = rom_base;
    *available_length_out = available_length;
}


static void msx_pio_bus_init(void)
{
    msx_bus.pio = pio0;
    msx_bus.sm_read  = 0;
    msx_bus.sm_write = 1;

    if (!msx_bus_programs_loaded)
    {
        msx_bus.offset_read  = pio_add_program(msx_bus.pio, &msx_read_responder_program);
        msx_bus.offset_write = pio_add_program(msx_bus.pio, &msx_write_captor_program);
        msx_bus_programs_loaded = true;
    }

    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_read, false);
    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_write, false);
    pio_sm_clear_fifos(msx_bus.pio, msx_bus.sm_read);
    pio_sm_clear_fifos(msx_bus.pio, msx_bus.sm_write);
    pio_sm_restart(msx_bus.pio, msx_bus.sm_read);
    pio_sm_restart(msx_bus.pio, msx_bus.sm_write);

    pio_sm_config cfg_read = msx_read_responder_program_get_default_config(msx_bus.offset_read);
    sm_config_set_in_pins(&cfg_read, PIN_A0);
    sm_config_set_in_shift(&cfg_read, false, false, 16);
    sm_config_set_out_pins(&cfg_read, PIN_D0, 8);
    sm_config_set_out_shift(&cfg_read, true, false, 32);
    sm_config_set_sideset_pins(&cfg_read, PIN_WAIT);
    sm_config_set_jmp_pin(&cfg_read, PIN_RD);
    sm_config_set_clkdiv(&cfg_read, 1.0f);
    pio_sm_init(msx_bus.pio, msx_bus.sm_read, msx_bus.offset_read, &cfg_read);

    pio_sm_config cfg_write = msx_write_captor_program_get_default_config(msx_bus.offset_write);
    sm_config_set_in_pins(&cfg_write, PIN_A0);
    sm_config_set_in_shift(&cfg_write, false, false, 32);
    sm_config_set_fifo_join(&cfg_write, PIO_FIFO_JOIN_RX);
    sm_config_set_jmp_pin(&cfg_write, PIN_WR);
    sm_config_set_clkdiv(&cfg_write, 1.0f);
    pio_sm_init(msx_bus.pio, msx_bus.sm_write, msx_bus.offset_write, &cfg_write);

    pio_gpio_init(msx_bus.pio, PIN_WAIT);
    pio_sm_set_consecutive_pindirs(msx_bus.pio, msx_bus.sm_read, PIN_WAIT, 1, true);

    for (uint pin = PIN_D0; pin <= PIN_D7; ++pin)
    {
        pio_gpio_init(msx_bus.pio, pin);
    }

    pio_sm_set_consecutive_pindirs(msx_bus.pio, msx_bus.sm_read, PIN_D0, 8, false);
    pio_sm_set_consecutive_pindirs(msx_bus.pio, msx_bus.sm_write, PIN_D0, 8, false);

    gpio_put(PIN_WAIT, 1);
    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_read, true);
    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_write, true);
}

static void msx_pio_io_bus_init(void)
{
    msx_io_bus.pio_read = pio1;
    msx_io_bus.pio_write = pio1;
    msx_io_bus.sm_io_read  = 0;
    msx_io_bus.sm_io_write = 1;

    if (!msx_io_bus_programs_loaded)
    {
        msx_io_bus.offset_io_read = pio_add_program(msx_io_bus.pio_read, &msx_io_read_responder_program);
        msx_io_bus.offset_io_write = pio_add_program(msx_io_bus.pio_write, &msx_io_write_captor_program);
        msx_io_bus_programs_loaded = true;
    }

    pio_sm_set_enabled(msx_io_bus.pio_read, msx_io_bus.sm_io_read, false);
    pio_sm_set_enabled(msx_io_bus.pio_write, msx_io_bus.sm_io_write, false);
    pio_sm_clear_fifos(msx_io_bus.pio_read, msx_io_bus.sm_io_read);
    pio_sm_clear_fifos(msx_io_bus.pio_write, msx_io_bus.sm_io_write);
    pio_sm_restart(msx_io_bus.pio_read, msx_io_bus.sm_io_read);
    pio_sm_restart(msx_io_bus.pio_write, msx_io_bus.sm_io_write);

    pio_sm_config cfg_io_read = msx_io_read_responder_program_get_default_config(msx_io_bus.offset_io_read);
    sm_config_set_in_pins(&cfg_io_read, PIN_A0);
    sm_config_set_in_shift(&cfg_io_read, false, false, 16);
    sm_config_set_out_pins(&cfg_io_read, PIN_D0, 8);
    sm_config_set_out_shift(&cfg_io_read, true, false, 32);
    sm_config_set_jmp_pin(&cfg_io_read, PIN_RD);
    sm_config_set_clkdiv(&cfg_io_read, 1.0f);
    pio_sm_init(msx_io_bus.pio_read, msx_io_bus.sm_io_read, msx_io_bus.offset_io_read, &cfg_io_read);

    pio_sm_config cfg_io_write = msx_io_write_captor_program_get_default_config(msx_io_bus.offset_io_write);
    sm_config_set_in_pins(&cfg_io_write, PIN_A0);
    sm_config_set_in_shift(&cfg_io_write, false, false, 32);
    sm_config_set_fifo_join(&cfg_io_write, PIO_FIFO_JOIN_RX);
    sm_config_set_jmp_pin(&cfg_io_write, PIN_WR);
    sm_config_set_clkdiv(&cfg_io_write, 1.0f);
    pio_sm_init(msx_io_bus.pio_write, msx_io_bus.sm_io_write, msx_io_bus.offset_io_write, &cfg_io_write);

    pio_sm_set_consecutive_pindirs(msx_io_bus.pio_read, msx_io_bus.sm_io_read, PIN_D0, 8, false);

    pio_sm_set_enabled(msx_io_bus.pio_read, msx_io_bus.sm_io_read, true);
    pio_sm_set_enabled(msx_io_bus.pio_write, msx_io_bus.sm_io_write, true);
}

static inline uint8_t __not_in_flash_func(read_rom_byte)(const uint8_t *rom_base, uint32_t rel)
{
    return (rel < rom_cached_size) ? rom_sram[rel] : rom_base[rel];
}

static inline uint16_t __not_in_flash_func(pio_build_token)(bool drive, uint8_t data)
{
    uint8_t dir_mask = drive ? 0xFFu : 0x00u;
    return (uint16_t)data | ((uint16_t)dir_mask << 8);
}

static inline bool __not_in_flash_func(pio_try_get_write)(uint16_t *addr_out, uint8_t *data_out)
{
    if (pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_write))
        return false;

    uint32_t sample = pio_sm_get(msx_bus.pio, msx_bus.sm_write);
    *addr_out = (uint16_t)(sample & 0xFFFFu);
    *data_out = (uint8_t)((sample >> 16) & 0xFFu);
    return true;
}

static inline bool __not_in_flash_func(pio_try_get_io_write)(uint16_t *addr_out, uint8_t *data_out)
{
    if (pio_sm_is_rx_fifo_empty(msx_io_bus.pio_write, msx_io_bus.sm_io_write))
        return false;

    uint32_t sample = pio_sm_get(msx_io_bus.pio_write, msx_io_bus.sm_io_write);
    *addr_out = (uint16_t)(sample & 0xFFFFu);
    *data_out = (uint8_t)((sample >> 16) & 0xFFu);
    return true;
}

static inline bool __not_in_flash_func(pio_try_get_io_read)(uint16_t *addr_out)
{
    if (pio_sm_is_rx_fifo_empty(msx_io_bus.pio_read, msx_io_bus.sm_io_read))
        return false;

    *addr_out = (uint16_t)pio_sm_get(msx_io_bus.pio_read, msx_io_bus.sm_io_read);
    return true;
}

static inline int16_t __not_in_flash_func(clamp_i16)(int32_t sample)
{
    if (sample > 32767) return 32767;
    if (sample < -32768) return -32768;
    return (int16_t)sample;
}

static void __not_in_flash_func(dual_psg_init)(void)
{
    if (dual_psg_ready)
        return;

    memset(&psg_instance, 0, sizeof(psg_instance));
    psg_instance.rate = PSG_SAMPLE_RATE;
    PSG_setVolumeMode(&psg_instance, 2);
    PSG_setClock(&psg_instance, PSG_CLOCK);
    PSG_setQuality(&psg_instance, 1);
    PSG_reset(&psg_instance);
    msx_pio_io_bus_init();
    dual_psg_ready = true;
}

static inline void __not_in_flash_func(dual_psg_service_io)(void)
{
    if (!dual_psg_ready)
        return;

    uint16_t io_addr;
    uint8_t io_data;
    while (pio_try_get_io_write(&io_addr, &io_data))
    {
        uint8_t port = io_addr & 0xFFu;
        if (port == PSG_PORT_REG || port == PSG_PORT_DATA)
            PSG_writeIO(&psg_instance, port, io_data);
    }

    while (pio_try_get_io_read(&io_addr))
    {
        pio_sm_put_blocking(msx_io_bus.pio_read, msx_io_bus.sm_io_read,
                            pio_build_token(false, 0xFFu));
    }
}

static inline int16_t __not_in_flash_func(dual_psg_calc_sample)(void)
{
    if (!dual_psg_ready)
        return 0;

    return clamp_i16((int32_t)PSG_calc(&psg_instance) << PSG_VOLUME_SHIFT);
}

static void __no_inline_not_in_flash_func(core1_dual_psg_audio)(void)
{
    while (true)
    {
        while (true)
        {
            dual_psg_service_io();
            struct audio_buffer *buffer = take_audio_buffer(dual_psg_audio_pool, false);
            if (buffer)
            {
                int16_t *samples = (int16_t *)buffer->buffer->bytes;
                for (int i = 0; i < SCC_AUDIO_BUFFER_SAMPLES; i++)
                {
                    dual_psg_service_io();
                    int16_t sample = dual_psg_calc_sample();
                    samples[i * 2] = sample;
                    samples[i * 2 + 1] = sample;
                }
                buffer->sample_count = SCC_AUDIO_BUFFER_SAMPLES;
                give_audio_buffer(dual_psg_audio_pool, buffer);
                break;
            }
            tight_loop_contents();
        }
    }
}

static void dual_psg_audio_init(void)
{
    if (dual_psg_audio_started)
        return;

    gpio_init(I2S_MUTE_PIN);
    gpio_set_dir(I2S_MUTE_PIN, GPIO_OUT);
    gpio_put(I2S_MUTE_PIN, 0);

    static audio_format_t dual_psg_audio_format = {
        .sample_freq = PSG_SAMPLE_RATE,
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .channel_count = 2,
    };

    static struct audio_buffer_format dual_psg_producer_format = {
        .format = &dual_psg_audio_format,
        .sample_stride = 4,
    };

    dual_psg_audio_pool = audio_new_producer_pool(&dual_psg_producer_format, 3, SCC_AUDIO_BUFFER_SAMPLES);

    int dma_channel = -1;
    for (int ch = 0; ch < NUM_DMA_CHANNELS; ch++) {
        if (!dma_channel_is_claimed(ch)) {
            dma_channel = ch;
            break;
        }
    }
    if (dma_channel < 0) {
        dual_psg_audio_pool = NULL;
        return;
    }

    static struct audio_i2s_config dual_psg_i2s_config = {
        .data_pin = I2S_DATA_PIN,
        .clock_pin_base = I2S_BCLK_PIN,
        .dma_channel = 0,
        .pio_sm = 2,
    };
    dual_psg_i2s_config.dma_channel = (uint)dma_channel;

    audio_i2s_setup(&dual_psg_audio_format, &dual_psg_i2s_config);
    audio_i2s_connect(dual_psg_audio_pool);
    audio_i2s_set_enabled(true);
    dual_psg_audio_started = true;
}

static void start_dual_psg_audio(void)
{
    dual_psg_init();
    dual_psg_audio_init();
    if (dual_psg_audio_pool)
        multicore_launch_core1(core1_dual_psg_audio);
}

static void __not_in_flash_func(msx_music_init)(void)
{
    if (msx_music_ready)
        return;

    msx_music_instance = OPLL_new(MSX_MUSIC_CLOCK, MSX_MUSIC_SAMPLE_RATE);
    if (!msx_music_instance)
        return;

    if (!msx_music_lock)
        msx_music_lock = spin_lock_instance(spin_lock_claim_unused(true));

    OPLL_reset(msx_music_instance);
    OPLL_setChipType(msx_music_instance, OPLL_2413_TONE);
    OPLL_resetPatch(msx_music_instance, OPLL_2413_TONE);
    msx_pio_io_bus_init();
    msx_music_ready = true;
}

static inline int16_t __not_in_flash_func(msx_music_calc_sample)(void)
{
    if (!msx_music_ready)
        return 0;

    uint32_t save = spin_lock_blocking(msx_music_lock);
    int16_t sample = OPLL_calc(msx_music_instance);
    spin_unlock(msx_music_lock, save);
    return clamp_i16((int32_t)sample << MSX_MUSIC_VOLUME_SHIFT);
}

static inline void __not_in_flash_func(msx_music_write_io)(uint8_t port, uint8_t data)
{
    if (!msx_music_ready)
        return;

    uint32_t save = spin_lock_blocking(msx_music_lock);
    OPLL_writeIO(msx_music_instance, port, data);
    spin_unlock(msx_music_lock, save);
}

static inline void __not_in_flash_func(msx_music_service_io)(void)
{
    if (!msx_music_ready)
        return;

    uint16_t io_addr;
    uint8_t io_data;
    while (pio_try_get_io_write(&io_addr, &io_data))
    {
        uint8_t port = io_addr & 0xFFu;
        if (port == MSX_MUSIC_PORT_REG || port == MSX_MUSIC_PORT_DATA)
            msx_music_write_io(port, io_data);
    }

    while (pio_try_get_io_read(&io_addr))
    {
        pio_sm_put_blocking(msx_io_bus.pio_read, msx_io_bus.sm_io_read,
                            pio_build_token(false, 0xFFu));
    }
}

static void __no_inline_not_in_flash_func(core1_msx_music_audio)(void)
{
    while (true)
    {
        while (true)
        {
            msx_music_service_io();
            struct audio_buffer *buffer = take_audio_buffer(msx_music_audio_pool, false);
            if (buffer)
            {
                int16_t *samples = (int16_t *)buffer->buffer->bytes;
                for (int i = 0; i < SCC_AUDIO_BUFFER_SAMPLES; i++)
                {
                    msx_music_service_io();
                    int16_t sample = msx_music_calc_sample();
                    samples[i * 2] = sample;
                    samples[i * 2 + 1] = sample;
                }
                buffer->sample_count = SCC_AUDIO_BUFFER_SAMPLES;
                give_audio_buffer(msx_music_audio_pool, buffer);
                break;
            }
            tight_loop_contents();
        }
    }
}

static void msx_music_audio_init(void)
{
    if (msx_music_audio_started)
        return;

    gpio_init(I2S_MUTE_PIN);
    gpio_set_dir(I2S_MUTE_PIN, GPIO_OUT);
    gpio_put(I2S_MUTE_PIN, 0);

    static audio_format_t msx_music_audio_format = {
        .sample_freq = MSX_MUSIC_SAMPLE_RATE,
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .channel_count = 2,
    };

    static struct audio_buffer_format msx_music_producer_format = {
        .format = &msx_music_audio_format,
        .sample_stride = 4,
    };

    msx_music_audio_pool = audio_new_producer_pool(&msx_music_producer_format, 3, SCC_AUDIO_BUFFER_SAMPLES);

    int dma_channel = -1;
    for (int ch = 0; ch < NUM_DMA_CHANNELS; ch++) {
        if (!dma_channel_is_claimed(ch)) {
            dma_channel = ch;
            break;
        }
    }
    if (dma_channel < 0) {
        msx_music_audio_pool = NULL;
        return;
    }

    static struct audio_i2s_config msx_music_i2s_config = {
        .data_pin = I2S_DATA_PIN,
        .clock_pin_base = I2S_BCLK_PIN,
        .dma_channel = 0,
        .pio_sm = 2,
    };
    msx_music_i2s_config.dma_channel = (uint)dma_channel;

    audio_i2s_setup(&msx_music_audio_format, &msx_music_i2s_config);
    audio_i2s_connect(msx_music_audio_pool);
    audio_i2s_set_enabled(true);
    msx_music_audio_started = true;
}

static void start_msx_music_audio(void)
{
    msx_music_init();
    msx_music_audio_init();
    if (msx_music_audio_pool)
        multicore_launch_core1(core1_msx_music_audio);
}

static inline uint8_t __not_in_flash_func(mapper_page_from_reg)(uint8_t reg)
{
    return (uint8_t)(reg % MAPPER_PAGES);
}

static inline void __not_in_flash_func(pio_drain_writes)(void (*handler)(uint16_t addr, uint8_t data, void *ctx), void *ctx)
{
    uint16_t addr;
    uint8_t data;
    while (pio_try_get_write(&addr, &data))
    {
        handler(addr, data, ctx);
    }
}

static void __not_in_flash_func(mapper_fill_ff)(void)
{
    uint32_t *words = (uint32_t *)mapper_region.ptr;
    for (uint32_t index = 0; index < (mapper_region.size / sizeof(uint32_t)); ++index)
    {
        words[index] = 0xFFFFFFFFu;
    }
}

static inline void __not_in_flash_func(mapper_write_byte)(uint32_t offset, uint8_t data)
{
    mapper_region.ptr[offset] = data;
}

static inline uint8_t __not_in_flash_func(mapper_read_byte)(uint32_t offset)
{
    return mapper_region.ptr[offset];
}

typedef struct {
    sunrise_ide_t *ide;
} sunrise_ctx_t;

static inline void __not_in_flash_func(handle_sunrise_write)(uint16_t addr, uint8_t data, void *ctx)
{
    sunrise_ctx_t *sctx = (sunrise_ctx_t *)ctx;
    if (addr >= 0x4000u && addr <= 0x7FFFu)
    {
        sunrise_ide_handle_write(sctx->ide, addr, data);
    }
}

static inline bool __not_in_flash_func(fmpac_sram_enabled)(const fmpac_state_t *fmpac)
{
    return fmpac->sram_key_5ffe == 0x4Du && fmpac->sram_key_5fff == 0x69u;
}

static inline void __not_in_flash_func(fmpac_handle_write)(fmpac_state_t *fmpac, uint16_t addr, uint8_t data)
{
    if (addr == 0x7FF4u)
    {
        msx_music_write_io(MSX_MUSIC_PORT_REG, data);
    }
    else if (addr == 0x7FF5u)
    {
        msx_music_write_io(MSX_MUSIC_PORT_DATA, data);
    }
    else if (addr == 0x7FF6u)
    {
        fmpac->control = data;
    }
    else if (addr == 0x7FF7u)
    {
        fmpac->page = data & 0x03u;
    }
    else if (addr == 0x5FFEu)
    {
        fmpac->sram_key_5ffe = data;
    }
    else if (addr == 0x5FFFu)
    {
        fmpac->sram_key_5fff = data;
    }
    else if (fmpac_sram_enabled(fmpac) && addr >= 0x4000u && addr <= 0x5FFFu)
    {
        fmpac->sram[addr - 0x4000u] = data;
    }
}

static inline uint8_t __not_in_flash_func(fmpac_handle_read)(const fmpac_state_t *fmpac, const uint8_t *bios_base, uint16_t addr)
{
    if (addr == 0x7FF6u)
        return fmpac->control;
    if (addr == 0x7FF7u)
        return fmpac->page;
    if (fmpac_sram_enabled(fmpac) && addr >= 0x4000u && addr <= 0x5FFFu)
        return fmpac->sram[addr - 0x4000u];

    if (addr >= 0x4000u && addr <= 0x7FFFu)
    {
        uint32_t rel = ((uint32_t)(fmpac->page & 0x03u) << 14) | (addr & 0x3FFFu);
        if (rel < FMPAC_BIOS_SIZE)
            return bios_base[rel];
    }
    return 0xFFu;
}

static inline void __not_in_flash_func(handle_menu_write)(uint16_t addr, uint8_t data, void *ctx)
{
    menu_select_ctx_t *menu_ctx = (menu_select_ctx_t *)ctx;
    
    // Original explorer menu handling logic needs to be adapted here
    // However, the original menu loop was complex handling many registers.
    // The handle_menu_write in multirom only checks MONITOR_ADDR.
    // Explorer needs to handle CTRL registers and MP3 registers too.

    // For now, I'll only put the basic MONITOR_ADDR check here 
    // AND I will have to implement a more complex handler or just use the main loop 
    // inside loadrom_msx_menu which will be refactored anyway.
    
    if (addr == MONITOR_ADDR)
    {
        if (data == ROM_SELECT_MENU) {
            menu_ctx->rom_selected = false;
            return;
        }
        menu_ctx->rom_index = data;
        menu_ctx->rom_selected = true;
    }
}

static inline void __not_in_flash_func(handle_konamiscc_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint8_t *regs = ((bank8_ctx_t *)ctx)->bank_regs;
    if      (addr >= 0x5000u && addr <= 0x57FFu) regs[0] = data;
    else if (addr >= 0x7000u && addr <= 0x77FFu) regs[1] = data;
    else if (addr >= 0x9000u && addr <= 0x97FFu) regs[2] = data;
    else if (addr >= 0xB000u && addr <= 0xB7FFu) regs[3] = data;
}

static inline void __not_in_flash_func(handle_konami_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint8_t *regs = ((bank8_ctx_t *)ctx)->bank_regs;
    if      (addr >= 0x6000u && addr <= 0x67FFu) regs[1] = data;
    else if (addr >= 0x8000u && addr <= 0x87FFu) regs[2] = data;
    else if (addr >= 0xA000u && addr <= 0xA7FFu) regs[3] = data;
}

static inline void __not_in_flash_func(handle_ascii8_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint8_t *regs = ((bank8_ctx_t *)ctx)->bank_regs;
    if      (addr >= 0x6000u && addr <= 0x67FFu) regs[0] = data;
    else if (addr >= 0x6800u && addr <= 0x6FFFu) regs[1] = data;
    else if (addr >= 0x7000u && addr <= 0x77FFu) regs[2] = data;
    else if (addr >= 0x7800u && addr <= 0x7FFFu) regs[3] = data;
}

static inline void __not_in_flash_func(handle_ascii16_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint8_t *regs = ((bank8_ctx_t *)ctx)->bank_regs;
    if      (addr >= 0x6000u && addr <= 0x67FFu) regs[0] = data;
    else if (addr >= 0x7000u && addr <= 0x77FFu) regs[1] = data;
}

static inline void __not_in_flash_func(handle_neo8_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint16_t *regs = ((bank16_ctx_t *)ctx)->bank_regs;
    uint16_t base_addr = addr & 0xF800u;
    uint8_t bank_index = 6;

    switch (base_addr)
    {
        case 0x5000: case 0x1000: case 0x9000: case 0xD000: bank_index = 0; break;
        case 0x5800: case 0x1800: case 0x9800: case 0xD800: bank_index = 1; break;
        case 0x6000: case 0x2000: case 0xA000: case 0xE000: bank_index = 2; break;
        case 0x6800: case 0x2800: case 0xA800: case 0xE800: bank_index = 3; break;
        case 0x7000: case 0x3000: case 0xB000: case 0xF000: bank_index = 4; break;
        case 0x7800: case 0x3800: case 0xB800: case 0xF800: bank_index = 5; break;
    }

    if (bank_index < 6u)
    {
        if (addr & 0x01u)
            regs[bank_index] = (regs[bank_index] & 0x00FFu) | ((uint16_t)data << 8);
        else
            regs[bank_index] = (regs[bank_index] & 0xFF00u) | data;
        regs[bank_index] &= 0x0FFFu;
    }
}

static inline void __not_in_flash_func(handle_neo16_write)(uint16_t addr, uint8_t data, void *ctx)
{
    uint16_t *regs = ((bank16_ctx_t *)ctx)->bank_regs;
    uint16_t base_addr = addr & 0xF800u;
    uint8_t bank_index = 3;

    switch (base_addr)
    {
        case 0x5000: case 0x1000: case 0x9000: case 0xD000: bank_index = 0; break;
        case 0x6000: case 0x2000: case 0xA000: case 0xE000: bank_index = 1; break;
        case 0x7000: case 0x3000: case 0xB000: case 0xF000: bank_index = 2; break;
    }

    if (bank_index < 3u)
    {
        if (addr & 0x01u)
            regs[bank_index] = (regs[bank_index] & 0x00FFu) | ((uint16_t)data << 8);
        else
            regs[bank_index] = (regs[bank_index] & 0xFF00u) | data;
        regs[bank_index] &= 0x0FFFu;
    }
}

static void __no_inline_not_in_flash_func(banked8_loop)(
    const uint8_t *rom_base,
    uint32_t available_length,
    uint8_t *bank_regs,
    void (*write_handler)(uint16_t, uint8_t, void *))
{
    bank8_ctx_t ctx = { .bank_regs = bank_regs };

    while (true)
    {
        pio_drain_writes(write_handler, &ctx);

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        pio_drain_writes(write_handler, &ctx);

        bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint32_t rel = ((uint32_t)bank_regs[(addr - 0x4000u) >> 13] * 0x2000u) + (addr & 0x1FFFu);
            if (available_length == 0u || rel < available_length)
                data = read_rom_byte(rom_base, rel);
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

typedef struct {
    bool rom_selected;
    uint16_t rom_index;
} explorer_menu_ctx_t;

static void fh_copy_query_from_buffer(char *out, size_t out_size);
static bool fh_fetch_catalog(const char *query);
static void fh_set_catalog_message(const char *message);
static void fh_build_page_buffer(void);
static bool fh_download_selected(void);
static void fh_update_wifi_status_text(void);
static void fh_save_background_work(void);
static bool fh_start_save_job(void);

static void __not_in_flash_func(handle_menu_fh_command)(uint8_t data, explorer_menu_ctx_t *menu_ctx)
{
    fh_menu_window_active = true;
    fh_service_menu_window = true;
    fh_status = FH_STATUS_BUSY;

    if (data == CMD_FH_LIST_PAGE || data == CMD_FH_SEARCH)
    {
        if (data == CMD_FH_SEARCH || !fh_catalog_loaded)
        {
            fh_copy_query_from_buffer(fh_active_query, sizeof(fh_active_query));
            if (!fh_fetch_catalog(fh_active_query))
                fh_set_catalog_message("Offline or File Hunter unavailable");
            else if (fh_catalog_count == 0u)
                fh_set_catalog_message("No File Hunter ROMs found");
        }
        fh_build_page_buffer();
        fh_result = fh_catalog_count ? 1u : 0u;
    }
    else if (data == CMD_FH_DOWNLOAD)
    {
        fh_selected_index = (uint16_t)((uint8_t)fh_query[0]) |
                            (uint16_t)(((uint8_t)fh_query[1]) << 8);
        fh_progress_percent = 0;
        fh_result = 0;
        if (!fh_download_selected())
            fh_result = 0u;
    }
    else if (data == CMD_FH_WIFI_STATUS)
    {
        fh_update_wifi_status_text();
        fh_result = 1u;
    }
    else if (data == CMD_FH_WIFI_CONFIG)
    {
        wifi_f2_state = WIFI_F2_FORCE_SETUP;
        fh_result = 1u;
        if (menu_ctx)
        {
            menu_ctx->rom_index = ROM_SELECT_WIFI_CONFIG;
            menu_ctx->rom_selected = true;
        }
    }

    fh_status = FH_STATUS_READY;
    fh_service_menu_window = false;
}

static inline void __not_in_flash_func(handle_menu_write_explorer)(uint16_t addr, uint8_t data, void *ctx)
{
    explorer_menu_ctx_t *menu_ctx = (explorer_menu_ctx_t *)ctx;

    if (addr >= CTRL_QUERY_BASE && addr < (CTRL_QUERY_BASE + CTRL_QUERY_SIZE))
    {
        uint16_t query_offset = addr - CTRL_QUERY_BASE;
        filter_query[query_offset] = (char)data;
        if (query_offset < FH_QUERY_SIZE)
            fh_query[query_offset] = data;
        return;
    }

    if (addr == CTRL_CMD)
    {
        ctrl_cmd_state = data;
        if (data == CMD_APPLY_FILTER)
        {
            filter_query[CTRL_QUERY_SIZE - 1] = '\0';
            apply_filter();
            current_page = 0;
            build_page_buffer(current_page);
        }
        else if (data == CMD_FIND_FIRST)
        {
            filter_query[CTRL_QUERY_SIZE - 1] = '\0';
            find_first_match();
        }
        else if (data == CMD_ENTER_DIR)
        {
            filter_query[CTRL_QUERY_SIZE - 1] = '\0';
            if (filter_query[0] == '\0') {
                go_up_one_level();
            } else {
                append_folder_to_path(filter_query);
            }
            refresh_requested = true;
            // Keep ctrl_cmd_state as CMD_ENTER_DIR to signal busy/done?
            // Original code: ctrl_cmd_state = CMD_ENTER_DIR;
            // Then it checked if (cmd != CMD_ENTER_DIR...) ctrl_cmd_state = 0;
            // Here data is CMD_ENTER_DIR.
        }
        else if (data == CMD_DETECT_MAPPER)
        {
             uint16_t index = (uint16_t)((uint8_t)filter_query[0]) |
                                         (uint16_t)(((uint8_t)filter_query[1]) << 8);
             detect_mapper_index = index;
             detect_mapper_pending = true;
             ctrl_mapper_value = 0;
        }
        else if (data == CMD_SET_MAPPER)
        {
             uint16_t index = (uint16_t)((uint8_t)filter_query[0]) |
                                         (uint16_t)(((uint8_t)filter_query[1]) << 8);
             uint8_t mapper = (uint8_t)filter_query[2];
             
             ctrl_ack_value = 0;
             if (index < total_record_count && mapper != 0 && !is_system_mapper(mapper) && mapper < MAPPER_DESCRIPTION_COUNT) {
                 uint16_t record_index = filtered_indices[index];
                 if (record_index < full_record_count) {
                    ROMRecord *rec = &records[record_index];
                    uint8_t flags = rec->Mapper & (SOURCE_SD_FLAG | FOLDER_FLAG | MP3_FLAG);
                    if ((flags & (FOLDER_FLAG | MP3_FLAG)) == 0) {
                        rec->Mapper = (uint8_t)(flags | OVERRIDE_FLAG | mapper);
                        ctrl_ack_value = 1;
                    }
                 }
             }
        }
        else if (data == CMD_SET_SOURCE)
        {
            uint8_t mode = (uint8_t)filter_query[0];
            if (mode > SOURCE_MODE_SD) {
                mode = SOURCE_MODE_ALL;
            }
            fh_menu_window_active = false;
            browse_source_mode = mode;
            sd_current_path[0] = '/';
            sd_current_path[1] = '\0';
            memset(filter_query, 0, sizeof(filter_query));
            refresh_requested = true;
        }
        else if (data == CMD_FH_LIST_PAGE || data == CMD_FH_SEARCH ||
                 data == CMD_FH_DOWNLOAD || data == CMD_FH_WIFI_STATUS ||
                 data == CMD_FH_WIFI_CONFIG)
        {
            handle_menu_fh_command(data, menu_ctx);
        }
        
        if (data != CMD_ENTER_DIR && data != CMD_DETECT_MAPPER) {
             ctrl_cmd_state = 0;
        }
        return;
    }

    if (addr == MP3_CTRL_INDEX_L) {
        mp3_selected_index = (uint16_t)((mp3_selected_index & 0xFF00u) | data);
        return;
    }
    if (addr == MP3_CTRL_INDEX_H) {
        mp3_selected_index = (uint16_t)((mp3_selected_index & 0x00FFu) | ((uint16_t)data << 8));
        return;
    }

    if (addr == MP3_CTRL_CMD) {
        if (data == MP3_CMD_SELECT) {
             if (mp3_selected_index < total_record_count) {
                 mp3_pending_index = mp3_selected_index;
                 mp3_pending_select = true;
             }
        } else if (data == MP3_CMD_PLAY || data == MP3_CMD_STOP || data == MP3_CMD_TOGGLE_MUTE) {
             mp3_pending_cmd = data;
        }
        return;
    }

    if (addr == MP3_CTRL_MODE) {
        if (data <= MP3_PLAY_MODE_RANDOM) {
            mp3_play_mode = data;
        }
        return;
    }

    if (addr == CTRL_AUDIO) {
        ctrl_audio_selection = data;
        return;
    }

    if (addr == CTRL_WIFI_SUPPORT) {
        ctrl_wifi_support = data ? 1u : 0u;
        return;
    }

    if (addr == CTRL_PAGE) {
        if (fh_menu_window_active) {
            fh_page_index = data;
        } else if (data != current_page) {
            current_page = data;
            build_page_buffer(current_page);
        }
        return;
    }

    if (addr == MONITOR_ADDR) {
        uint8_t selected_index = data;
        if (selected_index == ROM_SELECT_MENU) {
            menu_ctx->rom_selected = false;
            return;
        }
        if (selected_index < total_record_count) {
            menu_ctx->rom_index = (uint8_t)filtered_indices[selected_index];
            menu_ctx->rom_selected = true;
        }
        return;
    }
}

static uint16_t fh_read_le16(const uint8_t *ptr)
{
    return (uint16_t)ptr[0] | ((uint16_t)ptr[1] << 8);
}

static void fh_copy_status_prefix(const char *prefix);
static void fh_set_offline_status(void);
static bool fh_wifi_wait_byte(uint8_t *data_out, uint32_t deadline);
static bool fh_wifi_wait_connected(uint32_t timeout_us);

static uint8_t __not_in_flash_func(fh_read_window_byte)(uint16_t addr, bool *in_window)
{
    uint8_t data = 0xFFu;
    *in_window = false;

    if (addr >= 0x4000u && addr <= 0xBFFFu)
    {
        *in_window = true;
        if (addr >= FH_CTRL_BASE && addr <= (FH_CTRL_BASE + 0x0Fu))
        {
            switch (addr)
            {
                case FH_COUNT_L: data = (uint8_t)(fh_catalog_count & 0xFFu); break;
                case FH_COUNT_H: data = (uint8_t)((fh_catalog_count >> 8) & 0xFFu); break;
                case FH_PAGE: data = fh_page_index; break;
                case FH_STATUS: data = fh_status; break;
                case FH_PROGRESS_L: data = (uint8_t)(fh_progress_percent & 0xFFu); break;
                case FH_PROGRESS_H: data = (uint8_t)((fh_progress_percent >> 8) & 0xFFu); break;
                case FH_RESULT: data = fh_result; break;
            }
        }
        else if (addr >= FH_DATA_BASE && addr < (FH_DATA_BASE + (FH_RECORD_SIZE * FH_FILES_PER_PAGE)))
        {
            data = page_buffer[addr - FH_DATA_BASE];
        }
        else if (addr >= FH_STATUS_TEXT_BASE && addr < (FH_STATUS_TEXT_BASE + FH_STATUS_TEXT_SIZE))
        {
            data = (uint8_t)fh_wifi_status_text[addr - FH_STATUS_TEXT_BASE];
        }
    }

    return data;
}

static uint8_t __not_in_flash_func(fh_read_menu_window_byte)(uint16_t addr, bool *in_window)
{
    uint8_t data = 0xFFu;
    *in_window = false;

    if (addr >= 0x4000u && addr <= 0xBFFFu)
    {
        *in_window = true;
        if (addr >= FH_CTRL_BASE && addr <= (FH_CTRL_BASE + 0x0Fu))
        {
            switch (addr)
            {
                case FH_COUNT_L: data = (uint8_t)(fh_catalog_count & 0xFFu); break;
                case FH_COUNT_H: data = (uint8_t)((fh_catalog_count >> 8) & 0xFFu); break;
                case FH_PAGE: data = fh_page_index; break;
                case FH_STATUS: data = fh_status; break;
                case FH_CMD: data = ctrl_cmd_state; break;
                case FH_PROGRESS_L: data = (uint8_t)(fh_progress_percent & 0xFFu); break;
                case FH_PROGRESS_H: data = (uint8_t)((fh_progress_percent >> 8) & 0xFFu); break;
                case FH_RESULT: data = fh_result; break;
            }
        }
        else if (addr >= FH_STATUS_TEXT_BASE && addr < (FH_STATUS_TEXT_BASE + FH_STATUS_TEXT_SIZE))
        {
            data = (uint8_t)fh_wifi_status_text[addr - FH_STATUS_TEXT_BASE];
        }
        else if (addr >= FH_DATA_BASE && addr < (FH_DATA_BASE + (FH_RECORD_SIZE * FH_FILES_PER_PAGE)))
        {
            data = page_buffer[addr - FH_DATA_BASE];
        }
        else
        {
            uint32_t rel = addr - 0x4000u;
            if (rel < MENU_ROM_SIZE)
                data = rom_sram[rel];
        }
    }

    return data;
}

static void __not_in_flash_func(fh_service_msx_reads)(void)
{
    while (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
    {
        uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
        bool in_window;
        uint8_t data = fh_service_menu_window ?
            fh_read_menu_window_byte(addr, &in_window) :
            fh_read_window_byte(addr, &in_window);
        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

static uint32_t fh_read_le32(const uint8_t *ptr)
{
    return (uint32_t)ptr[0] |
           ((uint32_t)ptr[1] << 8) |
           ((uint32_t)ptr[2] << 16) |
           ((uint32_t)ptr[3] << 24);
}

static void fh_set_catalog_message(const char *message)
{
    memset(fh_catalog, 0, sizeof(fh_catalog));
    fh_catalog_loaded = false;
    if (message && *message)
    {
        strncpy(fh_catalog[0].name, message, ROM_NAME_MAX);
        fh_catalog[0].name[ROM_NAME_MAX] = '\0';
        fh_catalog_count = 1;
    }
    else
    {
        fh_catalog_count = 0;
    }
}

static bool fh_wifi_wait_response(uint8_t command, uint8_t *response, uint16_t response_cap,
                                  uint16_t *response_len, uint32_t timeout_us)
{
    uint8_t data;
    uint8_t rc;
    uint8_t size_hi;
    uint8_t size_lo;
    uint16_t size;
    uint16_t keep;
    uint32_t deadline;

    deadline = time_us_32() + timeout_us;
    do {
        if (!fh_wifi_wait_byte(&data, deadline)) return false;
    } while (data != command);

    if (!fh_wifi_wait_byte(&rc, deadline) || rc != 0u) return false;
    if (!fh_wifi_wait_byte(&size_hi, deadline)) return false;
    if (!fh_wifi_wait_byte(&size_lo, deadline)) return false;
    size = (uint16_t)(((uint16_t)size_hi << 8) | size_lo);
    keep = size;
    if (keep > response_cap) keep = response_cap;
    for (uint16_t i = 0; i < size; ++i)
    {
        if (!fh_wifi_wait_byte(&data, deadline)) return false;
        if (i < keep && response) response[i] = data;
    }
    if (response_len) *response_len = keep;
    return true;
}

static bool fh_wifi_command(uint8_t command, const uint8_t *payload, uint16_t payload_len,
                            uint8_t *response, uint16_t response_cap, uint16_t *response_len,
                            uint32_t timeout_us)
{
    wifi_uart_init_once();
    wifi_hw_rx_drain();
    wifi_reset_fifo();

    uart_putc_raw(WIFI_UART_INSTANCE, (char)command);
    uart_putc_raw(WIFI_UART_INSTANCE, (char)(payload_len >> 8));
    uart_putc_raw(WIFI_UART_INSTANCE, (char)(payload_len & 0xFFu));
    for (uint16_t i = 0; i < payload_len; ++i)
        uart_putc_raw(WIFI_UART_INSTANCE, (char)payload[i]);
    wifi_tx_busy_deadline_us = time_us_32() + wifi_uart_frame_time_us();

    return fh_wifi_wait_response(command, response, response_cap, response_len, timeout_us);
}

static bool fh_wifi_dns(const char *host, uint8_t ip[4])
{
    uint8_t payload[1 + 64];
    uint16_t host_len = (uint16_t)strlen(host);
    uint16_t response_len = 0;

    if (host_len > 63u) return false;
    payload[0] = 0;
    memcpy(&payload[1], host, host_len);
    if (!fh_wifi_command(206u, payload, (uint16_t)(host_len + 1u), ip, 4, &response_len, 15000000u))
        return false;
    return response_len == 4u;
}

static bool fh_wifi_tcp_open_ex(const char *host, uint16_t port, bool tls, uint8_t *connection)
{
    uint8_t ip[4];
    uint8_t params[11 + 64];
    uint16_t params_len = 11u;
    uint8_t response[1];
    uint16_t response_len = 0;
    size_t host_len = strlen(host);

    if (host_len > 63u) return false;
    if (!fh_wifi_dns(host, ip)) return false;
    memset(params, 0, sizeof(params));
    memcpy(params, ip, 4);
    params[4] = (uint8_t)(port & 0xFFu);
    params[5] = (uint8_t)(port >> 8);
    params[6] = 0xFF;
    params[7] = 0xFF;
    params[8] = 120;
    params[9] = 0;
    params[10] = tls ? 0x04u : 0u;
    if (tls)
    {
        memcpy(params + params_len, host, host_len);
        params_len = (uint16_t)(params_len + host_len);
    }

    if (!fh_wifi_command(13u, params, params_len, response, sizeof(response), &response_len, 60000000u))
        return false;
    if (response_len != 1u) return false;
    *connection = response[0];
    return true;
}

static bool fh_wifi_tcp_open(const char *host, uint8_t *connection)
{
    return fh_wifi_tcp_open_ex(host, 80u, false, connection);
}

static bool fh_wifi_tcp_state(uint8_t connection, uint16_t *available, uint8_t *state)
{
    uint8_t response[16];
    uint16_t response_len = 0;

    if (!fh_wifi_command(16u, &connection, 1u, response, sizeof(response), &response_len, 2000000u))
        return false;
    if (response_len < 4u) return false;
    if (state) *state = response[1];
    if (available) *available = (uint16_t)response[2] | ((uint16_t)response[3] << 8);
    return true;
}

static bool fh_wifi_tcp_send(uint8_t connection, const char *data, uint16_t len)
{
    uint8_t command = 17u;
    uint16_t payload_len = (uint16_t)(len + 2u);
    uint8_t response_dummy;
    uint16_t response_len = 0;

    wifi_uart_init_once();
    wifi_hw_rx_drain();
    wifi_reset_fifo();
    uart_putc_raw(WIFI_UART_INSTANCE, (char)command);
    uart_putc_raw(WIFI_UART_INSTANCE, (char)(payload_len >> 8));
    uart_putc_raw(WIFI_UART_INSTANCE, (char)(payload_len & 0xFFu));
    uart_putc_raw(WIFI_UART_INSTANCE, (char)connection);
    uart_putc_raw(WIFI_UART_INSTANCE, 1);
    for (uint16_t i = 0; i < len; ++i)
        uart_putc_raw(WIFI_UART_INSTANCE, data[i]);
    wifi_tx_busy_deadline_us = time_us_32() + wifi_uart_frame_time_us();

    return fh_wifi_wait_response(command, &response_dummy, 0, &response_len, 10000000u);
}

static bool fh_wifi_tcp_receive(uint8_t connection, uint8_t *buffer, uint16_t cap, uint16_t *bytes_read)
{
    uint8_t command = 18u;
    uint8_t payload[3];
    uint8_t data;
    uint8_t rc;
    uint8_t size_hi;
    uint8_t size_lo;
    uint16_t response_size;
    uint16_t response_len = 0;
    uint16_t keep;
    uint32_t deadline;

    if (cap > FH_HTTP_CHUNK_SIZE) cap = FH_HTTP_CHUNK_SIZE;
    fh_tcp_last_error = 0;
    payload[0] = connection;
    payload[1] = (uint8_t)(cap & 0xFFu);
    payload[2] = (uint8_t)(cap >> 8);

    wifi_uart_init_once();
    wifi_hw_rx_drain();
    wifi_reset_fifo();
    uart_putc_raw(WIFI_UART_INSTANCE, (char)command);
    uart_putc_raw(WIFI_UART_INSTANCE, 0);
    uart_putc_raw(WIFI_UART_INSTANCE, (char)sizeof(payload));
    for (uint16_t i = 0; i < sizeof(payload); ++i)
        uart_putc_raw(WIFI_UART_INSTANCE, (char)payload[i]);
    wifi_tx_busy_deadline_us = time_us_32() + wifi_uart_frame_time_us();

    deadline = time_us_32() + 20000000u;
    do {
        if (!fh_wifi_wait_byte(&data, deadline)) {
            fh_tcp_last_error = 0xFEu;
            *bytes_read = 0;
            return false;
        }
    } while (data != command);

    if (!fh_wifi_wait_byte(&rc, deadline))
    {
        fh_tcp_last_error = 0xFEu;
        *bytes_read = 0;
        return false;
    }

    if (rc == 3u)
    {
        *bytes_read = 0;
        return true;
    }
    if (rc != 0u)
    {
        fh_tcp_last_error = rc;
        *bytes_read = 0;
        return false;
    }

    if (!fh_wifi_wait_byte(&size_hi, deadline) ||
        !fh_wifi_wait_byte(&size_lo, deadline))
    {
        fh_tcp_last_error = 0xFEu;
        *bytes_read = 0;
        return false;
    }

    response_size = (uint16_t)(((uint16_t)size_hi << 8) | size_lo);
    keep = response_size;
    if (keep > cap + 2u)
    {
        fh_tcp_last_error = 0xFDu;
        keep = 0u;
    }
    for (uint16_t i = 0; i < response_size; ++i)
    {
        if (!fh_wifi_wait_byte(&data, deadline)) {
            fh_tcp_last_error = 0xFEu;
            *bytes_read = 0;
            return false;
        }
        if (i < keep) fh_tcp_buffer[i] = data;
        if ((i & 0x1Fu) == 0x1Fu) fh_service_msx_reads();
    }
    if (fh_tcp_last_error == 0xFDu)
    {
        *bytes_read = 0;
        return false;
    }
    response_len = keep;

    if (response_len <= 2u)
    {
        uint16_t available = 0;
        uint8_t state = 0;

        *bytes_read = 0;
        if (fh_wifi_tcp_state(connection, &available, &state) && available == 0u && state == 0u)
            return false;
        return true;
    }
    *bytes_read = (uint16_t)(response_len - 2u);
    memcpy(buffer, &fh_tcp_buffer[2], *bytes_read);
    return true;
}

static void fh_wifi_tcp_close(uint8_t connection)
{
    uint8_t payload = connection;
    (void)fh_wifi_command(14u, &payload, 1, NULL, 0, NULL, 2000000u);
}

typedef bool (*fh_http_body_callback_t)(const uint8_t *data, uint16_t len, void *ctx);

typedef struct {
    bool chunked;
    bool done;
    bool has_content_length;
    uint32_t content_remaining;
    uint32_t remaining;
    char size_line[12];
    uint8_t size_len;
    uint8_t crlf_to_skip;
} fh_http_decoder_t;

typedef struct {
    char host[64];
    char path[192];
    uint16_t port;
    bool tls;
} fh_url_t;

static bool fh_starts_with(const char *text, const char *prefix)
{
    while (*prefix)
    {
        if (*text++ != *prefix++) return false;
    }
    return true;
}

static bool fh_parse_url(const char *url, fh_url_t *out)
{
    const char *host_start;
    const char *path_start;
    const char *host_end;
    const char *port_start = NULL;
    size_t host_len;
    size_t path_len;

    memset(out, 0, sizeof(*out));
    if (fh_starts_with(url, "https://"))
    {
        out->tls = true;
        out->port = 443u;
        host_start = url + 8;
    }
    else if (fh_starts_with(url, "http://"))
    {
        out->tls = false;
        out->port = 80u;
        host_start = url + 7;
    }
    else
    {
        out->tls = false;
        out->port = 80u;
        host_start = FH_HOST;
        strncpy(out->host, FH_HOST, sizeof(out->host) - 1u);
        strncpy(out->path, url && *url ? url : "/", sizeof(out->path) - 1u);
        return true;
    }

    path_start = strchr(host_start, '/');
    host_end = path_start ? path_start : (host_start + strlen(host_start));
    for (const char *scan = host_start; scan < host_end; ++scan)
    {
        if (*scan == ':')
        {
            port_start = scan + 1;
            host_end = scan;
            break;
        }
    }

    host_len = (size_t)(host_end - host_start);
    if (host_len == 0u || host_len >= sizeof(out->host)) return false;
    memcpy(out->host, host_start, host_len);
    out->host[host_len] = '\0';

    if (port_start)
    {
        unsigned long parsed_port = strtoul(port_start, NULL, 10);
        if (parsed_port == 0u || parsed_port > 65535u) return false;
        out->port = (uint16_t)parsed_port;
    }

    if (!path_start) path_start = "/";
    path_len = strlen(path_start);
    if (path_len >= sizeof(out->path)) return false;
    memcpy(out->path, path_start, path_len + 1u);
    return true;
}

static const char *fh_find_header_value(const char *headers, const char *name)
{
    size_t name_len = strlen(name);
    const char *line = headers;

    while (line && *line)
    {
        const char *line_end = strstr(line, "\r\n");
        const char *value;
        if (!line_end) line_end = line + strlen(line);
        if ((size_t)(line_end - line) > name_len && line[name_len] == ':')
        {
            bool match = true;
            for (size_t i = 0; i < name_len; ++i)
            {
                if (tolower((unsigned char)line[i]) != tolower((unsigned char)name[i]))
                {
                    match = false;
                    break;
                }
            }
            if (match)
            {
                value = line + name_len + 1u;
                while (*value == ' ' || *value == '\t') value++;
                return value;
            }
        }
        if (*line_end == '\0') break;
        line = line_end + 2;
    }
    return NULL;
}

static bool fh_header_value_is_chunked(const char *value)
{
    if (!value) return false;
    while (*value && *value != '\r' && *value != '\n')
    {
        if (tolower((unsigned char)value[0]) == 'c' &&
            tolower((unsigned char)value[1]) == 'h' &&
            tolower((unsigned char)value[2]) == 'u' &&
            tolower((unsigned char)value[3]) == 'n' &&
            tolower((unsigned char)value[4]) == 'k' &&
            tolower((unsigned char)value[5]) == 'e' &&
            tolower((unsigned char)value[6]) == 'd')
            return true;
        value++;
    }
    return false;
}

static bool fh_parse_header_u32(const char *value, uint32_t *out)
{
    unsigned long parsed;

    if (!value || !out) return false;
    while (*value == ' ' || *value == '\t') value++;
    if (!isdigit((unsigned char)*value)) return false;
    parsed = strtoul(value, NULL, 10);
    if (parsed > 0xFFFFFFFFul) return false;
    *out = (uint32_t)parsed;
    return true;
}

static void fh_service_msx_reads_for(uint32_t duration_us)
{
    uint32_t start = time_us_32();
    while ((uint32_t)(time_us_32() - start) < duration_us)
    {
        fh_service_msx_reads();
        tight_loop_contents();
    }
}

static bool fh_deliver_http_body(const uint8_t *data, uint16_t len, fh_http_decoder_t *decoder,
                                 fh_http_body_callback_t callback, void *ctx)
{
    if (!decoder->chunked)
    {
        uint16_t take = len;
        if (decoder->has_content_length)
        {
            if (decoder->content_remaining == 0u)
            {
                decoder->done = true;
                return true;
            }
            if ((uint32_t)take > decoder->content_remaining)
                take = (uint16_t)decoder->content_remaining;
        }
        if (take != 0u && !callback(data, take, ctx))
            return false;
        if (decoder->has_content_length)
        {
            decoder->content_remaining -= take;
            if (decoder->content_remaining == 0u)
                decoder->done = true;
        }
        return true;
    }

    while (len > 0u && !decoder->done)
    {
        if (decoder->crlf_to_skip != 0u)
        {
            data++;
            len--;
            decoder->crlf_to_skip--;
            continue;
        }

        if (decoder->remaining == 0u)
        {
            uint8_t ch = *data++;
            len--;
            if (ch == '\n')
            {
                decoder->size_line[decoder->size_len] = '\0';
                decoder->remaining = (uint32_t)strtoul(decoder->size_line, NULL, 16);
                decoder->size_len = 0;
                if (decoder->remaining == 0u)
                    decoder->done = true;
            }
            else if (ch != '\r' && ch != ';' && decoder->size_len + 1u < sizeof(decoder->size_line))
            {
                decoder->size_line[decoder->size_len++] = (char)ch;
            }
            else if (ch == ';')
            {
                while (len > 0u && *data != '\n')
                {
                    data++;
                    len--;
                }
            }
            continue;
        }

        uint16_t take = len;
        if ((uint32_t)take > decoder->remaining)
            take = (uint16_t)decoder->remaining;
        if (take != 0u && !callback(data, take, ctx))
            return false;
        data += take;
        len = (uint16_t)(len - take);
        decoder->remaining -= take;
        if (decoder->remaining == 0u)
            decoder->crlf_to_skip = 2u;
    }

    return true;
}

static void fh_build_http_path(char *out, size_t out_size, const char *query, int download_index)
{
    size_t pos = 0;
    const char *prefix = "/index4.php?base=1BA0&type=rom&msx=&char=";
    const char *suffix = "&download=";

    pos += snprintf(out + pos, out_size - pos, "%s", prefix);
    if (!query || !*query) query = FH_DEFAULT_QUERY;
    for (; *query && pos + 4u < out_size; ++query)
    {
        unsigned char ch = (unsigned char)*query;
        if (isalnum(ch) || ch == '-' || ch == '_' || ch == '.')
            out[pos++] = (char)ch;
        else if (ch == ' ')
            out[pos++] = '+';
    }
    pos += snprintf(out + pos, out_size - pos, "%s", suffix);
    if (download_index >= 0)
        snprintf(out + pos, out_size - pos, "%d", download_index);
    else if (pos < out_size)
        out[pos] = '\0';
}

static bool fh_http_get_url(const char *url, fh_http_body_callback_t callback, void *ctx,
                            uint32_t idle_timeout_us, uint8_t redirect_depth)
{
    fh_url_t parsed;
    uint8_t connection;
    char request[384];
    uint8_t chunk[FH_HTTP_CHUNK_SIZE];
    char headers[768];
    uint16_t header_len = 0;
    uint16_t bytes_read;
    uint32_t last_data;
    uint8_t header_state = 0;
    bool body_started = false;
    bool got_body = false;
    bool tcp_closed = false;
    fh_http_decoder_t decoder;
    int status_code = 0;

    memset(&decoder, 0, sizeof(decoder));
    if (!fh_parse_url(url, &parsed)) return false;

    if (!fh_wifi_tcp_open_ex(parsed.host, parsed.port, parsed.tls, &connection)) return false;
    int request_len = snprintf(request, sizeof(request),
        "GET %s HTTP/1.1\r\nHost: %s\r\nUser-Agent: FHBrowser (MSX PicoVerse)\r\nConnection: close\r\n\r\n",
        parsed.path, parsed.host);
    if (request_len <= 0 || request_len >= (int)sizeof(request))
    {
        fh_wifi_tcp_close(connection);
        return false;
    }
    if (!fh_wifi_tcp_send(connection, request, (uint16_t)request_len))
    {
        fh_wifi_tcp_close(connection);
        return false;
    }

    last_data = time_us_32();
    while ((uint32_t)(time_us_32() - last_data) < idle_timeout_us)
    {
        if (!fh_wifi_tcp_receive(connection, chunk, sizeof(chunk), &bytes_read))
        {
            tcp_closed = true;
            if (got_body || body_started) break;
            continue;
        }
        if (bytes_read == 0)
        {
            fh_service_msx_reads_for(10000u);
            continue;
        }
        last_data = time_us_32();
        if (body_started)
        {
            if (!fh_deliver_http_body(chunk, bytes_read, &decoder, callback, ctx))
            {
                fh_wifi_tcp_close(connection);
                return false;
            }
            got_body = true;
            if (decoder.done) break;
            continue;
        }
        for (uint16_t i = 0; i < bytes_read; ++i)
        {
            uint8_t ch = chunk[i];
            if (!body_started)
            {
                if (header_len + 1u < sizeof(headers))
                    headers[header_len++] = (char)ch;
                if ((header_state == 0 && ch == '\r') ||
                    (header_state == 1 && ch == '\n') ||
                    (header_state == 2 && ch == '\r') ||
                    (header_state == 3 && ch == '\n'))
                {
                    header_state++;
                    if (header_state == 4)
                    {
                        headers[header_len] = '\0';
                        if (fh_starts_with(headers, "HTTP/"))
                        {
                            char *space = strchr(headers, ' ');
                            if (space) status_code = atoi(space + 1);
                        }
                        if (status_code >= 300 && status_code < 400 && redirect_depth < 4u)
                        {
                            const char *location = fh_find_header_value(headers, "Location");
                            if (location)
                            {
                                char redirect_url[256];
                                uint16_t pos = 0;
                                while (*location && *location != '\r' && *location != '\n' && pos + 1u < sizeof(redirect_url))
                                    redirect_url[pos++] = *location++;
                                redirect_url[pos] = '\0';
                                fh_wifi_tcp_close(connection);
                                if (redirect_url[0] == '/')
                                {
                                    char absolute_url[320];
                                    snprintf(absolute_url, sizeof(absolute_url), "%s://%s%s",
                                             parsed.tls ? "https" : "http", parsed.host, redirect_url);
                                    return fh_http_get_url(absolute_url, callback, ctx, idle_timeout_us, (uint8_t)(redirect_depth + 1u));
                                }
                                return fh_http_get_url(redirect_url, callback, ctx, idle_timeout_us, (uint8_t)(redirect_depth + 1u));
                            }
                        }
                        decoder.chunked = fh_header_value_is_chunked(fh_find_header_value(headers, "Transfer-Encoding"));
                        if (!decoder.chunked && fh_parse_header_u32(fh_find_header_value(headers, "Content-Length"), &decoder.content_remaining))
                            decoder.has_content_length = true;
                        body_started = true;
                        if (i + 1u < bytes_read)
                        {
                            uint16_t body_len = (uint16_t)(bytes_read - i - 1u);
                            if (!fh_deliver_http_body(&chunk[i + 1u], body_len, &decoder, callback, ctx))
                            {
                                fh_wifi_tcp_close(connection);
                                return false;
                            }
                            got_body = true;
                        }
                        break;
                    }
                }
                else
                {
                    header_state = (ch == '\r') ? 1u : 0u;
                }
            }
        }
        if (body_started && !got_body)
            got_body = true;
        if (decoder.done) break;
    }

    fh_wifi_tcp_close(connection);
    if (!body_started) return false;
    if (decoder.chunked || decoder.has_content_length) return decoder.done;
    return got_body && tcp_closed;
}

static bool fh_http_get(const char *path, fh_http_body_callback_t callback, void *ctx, uint32_t idle_timeout_us)
{
    char url[256];

    if (!fh_wifi_wait_connected(FH_WIFI_CONNECT_WAIT_US))
    {
        fh_set_offline_status();
        return false;
    }

    if (fh_starts_with(path, "http://") || fh_starts_with(path, "https://"))
        return fh_http_get_url(path, callback, ctx, idle_timeout_us, 0);
    snprintf(url, sizeof(url), "http://" FH_HOST "%s", path);
    return fh_http_get_url(url, callback, ctx, idle_timeout_us, 0);
}

typedef struct {
    uint8_t *data;
    uint32_t len;
    uint32_t cap;
    bool overflowed;
} fh_list_download_t;

static bool fh_list_body_callback(const uint8_t *data, uint16_t len, void *ctx)
{
    fh_list_download_t *download = (fh_list_download_t *)ctx;
    if (download->len + len > download->cap)
    {
        download->overflowed = true;
        return false;
    }
    memcpy(download->data + download->len, data, len);
    download->len += len;
    return true;
}

static bool fh_parse_catalog(const uint8_t *data, uint32_t len)
{
    uint16_t raw_count = 0;
    uint32_t pos = 0;
    uint32_t base_ptr;
    uint32_t string_start;

    fh_catalog_count = 0;
    if (len < 4u) return false;

    while (pos + 4u <= len)
    {
        uint32_t name_ptr = fh_read_le32(data + pos);
        if (name_ptr == 0u) break;
        if (pos + 7u > len) return false;
        raw_count++;
        pos += 7u;
    }
    if (pos + 4u > len) return false;
    if (raw_count == 0u)
    {
        fh_catalog_loaded = true;
        return true;
    }

    base_ptr = fh_read_le32(data);
    string_start = pos + 4u;
    pos = 0;
    for (uint16_t i = 0; i < raw_count && fh_catalog_count < FH_MAX_RESULTS; ++i, pos += 7u)
    {
        uint32_t name_ptr = fh_read_le32(data + pos);
        uint32_t name_offset;
        uint32_t source;
        uint16_t out = 0;

        if (name_ptr < base_ptr) continue;
        name_offset = string_start + (name_ptr - base_ptr);
        if (name_offset >= len) continue;

        memset(&fh_catalog[fh_catalog_count], 0, sizeof(fh_catalog[fh_catalog_count]));
        source = name_offset;
        while (source < len && data[source] != 0u && out < ROM_NAME_MAX)
            fh_catalog[fh_catalog_count].name[out++] = (char)data[source++];
        fh_catalog[fh_catalog_count].name[out] = '\0';
        fh_catalog[fh_catalog_count].size_kb = fh_read_le16(data + pos + 4u);
        if (fh_catalog[fh_catalog_count].name[0] != '\0')
            fh_catalog_count++;
    }

    fh_catalog_loaded = true;
    return true;
}

static void fh_copy_query_from_buffer(char *out, size_t out_size)
{
    size_t i;
    if (out_size == 0) return;
    for (i = 0; i + 1u < out_size && i < FH_QUERY_SIZE; ++i)
    {
        char ch = (char)fh_query[i];
        if (ch == '\0') break;
        out[i] = ch;
    }
    out[i] = '\0';
    if (out[0] == '\0')
        strncpy(out, FH_DEFAULT_QUERY, out_size - 1u);
    out[out_size - 1u] = '\0';
}

static bool fh_fetch_catalog(const char *query)
{
    char path[192];

    if (!fh_prepare_list_region()) return false;

    fh_list_download_t download = { fh_list_region.ptr, 0, fh_list_region.size, false };

    fh_build_http_path(path, sizeof(path), query, -1);
    if (!fh_http_get(path, fh_list_body_callback, &download, 3000000u))
    {
        if (download.overflowed)
        {
            fh_set_catalog_message("Too many results. Refine search");
            return true;
        }
        return false;
    }
    return fh_parse_catalog(download.data, download.len);
}

typedef struct {
    bool metadata_done;
    char metadata_line[128];
    uint8_t metadata_len;
    uint32_t expected_size;
    uint32_t written;
    uint16_t fallback_size_kb;
    bool failed;
} fh_psram_download_t;

static uint32_t fh_parse_download_size(const char *line, uint16_t fallback_size_kb)
{
    const char *ptr = strstr(line, "size:");
    uint32_t fallback_size = (uint32_t)fallback_size_kb * 1024u;
    uint32_t value = 0u;

    if (ptr)
    {
        value = (uint32_t)strtoul(ptr + 5, NULL, 10);
    }

    if (value == 0u)
    {
        ptr = line;
        while (*ptr == ' ' || *ptr == '\t') ptr++;
        if (tolower((unsigned char)ptr[0]) == 's' &&
            tolower((unsigned char)ptr[1]) == 'i' &&
            tolower((unsigned char)ptr[2]) == 'z' &&
            tolower((unsigned char)ptr[3]) == 'e' && ptr[4] == ':')
        {
            value = (uint32_t)strtoul(ptr + 5, NULL, 10);
        }
    }

    if (value < fallback_size) value = fallback_size;
    return value;
}

static bool fh_prepare_download_region(uint32_t size)
{
    if (size == 0u || size > PSRAM_TOTAL_SIZE) return false;
    if (!psram_bring_up_once()) return false;

    if (size <= sd_rom_region.size)
    {
        fh_download_region = sd_rom_region;
        return true;
    }

    if (fh_download_region.ptr && fh_download_region.size >= size) return true;
    return psram_alloc(size, &fh_download_region);
}

static bool fh_psram_download_body_callback(const uint8_t *data, uint16_t len, void *ctx)
{
    fh_psram_download_t *download = (fh_psram_download_t *)ctx;
    const uint8_t *ptr = data;
    uint16_t remaining = len;

    while (!download->metadata_done && remaining > 0u)
    {
        uint8_t ch = *ptr++;
        remaining--;
        if (ch == '\n')
        {
            download->metadata_line[download->metadata_len] = '\0';
            download->expected_size = fh_parse_download_size(download->metadata_line, download->fallback_size_kb);
            if (!fh_prepare_download_region(download->expected_size))
            {
                download->failed = true;
                return false;
            }
            download->metadata_done = true;
            fh_progress_percent = 0;
            break;
        }
        if (ch != '\r' && download->metadata_len + 1u < sizeof(download->metadata_line))
            download->metadata_line[download->metadata_len++] = (char)ch;
    }

    if (!download->metadata_done)
        return true;

    if (remaining > 0u)
    {
        uint32_t to_copy = remaining;

        if (!fh_download_region.ptr || download->written > fh_download_region.size ||
            to_copy > (fh_download_region.size - download->written))
        {
            download->failed = true;
            return false;
        }

        memcpy(fh_download_region.ptr + download->written, ptr, to_copy);
        download->written += to_copy;
    }

    if (download->expected_size != 0u)
    {
        uint32_t percent = (download->written * 100u) / download->expected_size;
        if (percent > 100u) percent = 100u;
        fh_progress_percent = (uint16_t)percent;
    }

    fh_service_msx_reads();
    return true;
}

static bool fh_filename_has_rom_extension(const char *filename)
{
    size_t len = strlen(filename);
    const char *ext;
    if (len < 4u) return false;
    ext = filename + len - 4u;
    return ext[0] == '.' &&
           tolower((unsigned char)ext[1]) == 'r' &&
           tolower((unsigned char)ext[2]) == 'o' &&
           tolower((unsigned char)ext[3]) == 'm';
}

static bool fh_build_download_path(const char *name, char *path, size_t path_size)
{
    char filename[ROM_NAME_MAX + 5u];
    size_t out = 0;

    if (!path || path_size == 0u) return false;
    if (!name || *name == '\0') name = "FILEHUNT";

    for (size_t i = 0; name[i] != '\0' && out + 1u < sizeof(filename); ++i)
    {
        unsigned char ch = (unsigned char)name[i];
        if (ch < 32u || ch == '/' || ch == '\\' || ch == ':' || ch == '*' ||
            ch == '?' || ch == '"' || ch == '<' || ch == '>' || ch == '|')
        {
            ch = '_';
        }
        filename[out++] = (char)ch;
    }

    while (out > 0u && (filename[out - 1u] == ' ' || filename[out - 1u] == '.'))
        --out;

    if (out == 0u)
    {
        memcpy(filename, "FILEHUNT", 8u);
        out = 8u;
    }

    filename[out] = '\0';
    if (!fh_filename_has_rom_extension(filename) && out + 4u < sizeof(filename))
    {
        memcpy(filename + out, ".ROM", 5u);
    }

    return snprintf(path, path_size, "%s", filename) < (int)path_size;
}

static void fh_save_fail(const char *message)
{
    if (message)
        fh_copy_status_prefix(message);
    if (fh_save_file_open)
    {
        f_close(&fh_save_file);
        fh_save_file_open = false;
    }
    fh_save_close_pending = false;
    fh_result = 0u;
    fh_save_state = FH_SAVE_FAILED;
}

static void fh_save_background_work(void)
{
    FRESULT fr;

    if (fh_save_state != FH_SAVE_RUNNING) return;

    if (!fh_save_file_open && !fh_save_close_pending)
    {
        char path[SD_PATH_MAX];

        if (!fh_download_region.ptr || fh_download_size == 0u || fh_download_size > fh_download_region.size)
        {
            fh_save_fail("Save failed: buffer");
            return;
        }
        if (!sd_is_mounted())
        {
            fh_save_fail("Use F2 microSD first");
            return;
        }
        if (!fh_build_download_path(fh_download_name, path, sizeof(path)))
        {
            fh_save_fail("Save failed: path");
            return;
        }
        fr = f_open(&fh_save_file, path, FA_CREATE_ALWAYS | FA_WRITE);
        if (fr != FR_OK)
        {
            snprintf(fh_wifi_status_text, sizeof(fh_wifi_status_text), "Save failed: open %u", (unsigned int)fr);
            fh_save_fail(NULL);
            return;
        }
        fh_save_file_open = true;
        return;
    }

    if (fh_save_close_pending)
    {
        fr = f_close(&fh_save_file);
        fh_save_file_open = false;
        fh_save_close_pending = false;
        if (fr != FR_OK)
        {
            snprintf(fh_wifi_status_text, sizeof(fh_wifi_status_text), "Save failed: close %u", (unsigned int)fr);
            fh_result = 0u;
            fh_save_state = FH_SAVE_FAILED;
            return;
        }
        fh_copy_status_prefix("Saved to microSD");
        fh_result = 1u;
        fh_save_state = FH_SAVE_DONE;
        set_root_path();
        refresh_requested = true;
        return;
    }

    if (fh_save_offset < fh_download_size)
    {
        UINT to_write = (UINT)((fh_download_size - fh_save_offset) > 512u ? 512u : (fh_download_size - fh_save_offset));
        UINT written = 0;

        memcpy(fh_sd_save_chunk, fh_download_region.ptr + fh_save_offset, to_write);
        fr = f_write(&fh_save_file, fh_sd_save_chunk, to_write, &written);
        if (fr != FR_OK || written != to_write)
        {
            snprintf(fh_wifi_status_text, sizeof(fh_wifi_status_text), "Save failed: write %u", (unsigned int)fr);
            fh_save_fail(NULL);
            return;
        }
        fh_save_offset += written;
        if (fh_save_offset >= fh_download_size)
            fh_save_close_pending = true;
    }
}

static bool fh_start_save_job(void)
{
    if (fh_save_state == FH_SAVE_RUNNING) return false;
    if (mp3_core1_started)
    {
        fh_copy_status_prefix("Save failed: audio busy");
        fh_save_state = FH_SAVE_FAILED;
        fh_result = 0u;
        return false;
    }

    fh_save_offset = 0;
    fh_save_file_open = false;
    fh_save_close_pending = false;
    fh_result = FH_RESULT_SAVING;
    fh_save_state = FH_SAVE_RUNNING;
    return true;
}

static bool fh_download_selected(void)
{
    char path[192];
    fh_psram_download_t download;
    bool http_ok;
    bool ok;

    if (fh_selected_index >= fh_catalog_count) return false;

    fh_build_http_path(path, sizeof(path), fh_active_query, (int)fh_selected_index);

    memset(&download, 0, sizeof(download));
    download.fallback_size_kb = fh_catalog[fh_selected_index].size_kb;
    fh_progress_percent = 0;
    http_ok = fh_http_get(path, fh_psram_download_body_callback, &download, 20000000u);
    ok = http_ok &&
         download.metadata_done && !download.failed &&
            download.written != 0u &&
            (download.expected_size == 0u || download.written >= download.expected_size);
    if (ok)
    {
        fh_progress_percent = 100u;
        fh_download_size = download.written;
        strncpy(fh_download_name, fh_catalog[fh_selected_index].name, sizeof(fh_download_name) - 1u);
        fh_download_name[sizeof(fh_download_name) - 1u] = '\0';
        fh_result = FH_RESULT_SAVING;
        fh_copy_status_prefix("Saving to microSD card...");
        fh_service_msx_reads_for(250000u);
        ok = fh_start_save_job();
    }
    else
    {
        uint32_t percent = 0;
        if (download.expected_size != 0u)
        {
            percent = (download.written * 100u) / download.expected_size;
            if (percent > 100u) percent = 100u;
        }
        if (!http_ok)
            snprintf(fh_wifi_status_text, sizeof(fh_wifi_status_text), "Download failed: TCP %u %lu%%", (unsigned int)fh_tcp_last_error, (unsigned long)percent);
        else if (!download.metadata_done)
            fh_copy_status_prefix("Download failed: no size");
        else if (download.failed)
            fh_copy_status_prefix("Download failed: data");
        else
            snprintf(fh_wifi_status_text, sizeof(fh_wifi_status_text), "Download failed: %lu%%", (unsigned long)percent);
    }
    return ok;
}

static void __not_in_flash_func(fh_build_page_buffer)(void)
{
    memset(page_buffer, 0, FH_RECORD_SIZE * FH_FILES_PER_PAGE);
    uint16_t start = (uint16_t)fh_page_index * FH_FILES_PER_PAGE;
    for (uint16_t row = 0; row < FH_FILES_PER_PAGE; row++)
    {
        uint16_t idx = start + row;
        if (idx >= fh_catalog_count)
            break;
        uint8_t *dst = &page_buffer[row * FH_RECORD_SIZE];
        strncpy((char *)dst, fh_catalog[idx].name, ROM_NAME_MAX);
        dst[60] = 0;
        uint16_t size_kb = fh_catalog[idx].size_kb;
        dst[61] = (uint8_t)(size_kb & 0xFFu);
        dst[62] = (uint8_t)((size_kb >> 8) & 0xFFu);
        dst[63] = 0;
    }
}

static void __not_in_flash_func(fh_copy_status_prefix)(const char *prefix)
{
    uint16_t i = 0;
    while (i + 1u < FH_STATUS_TEXT_SIZE && prefix[i] != '\0')
    {
        fh_wifi_status_text[i] = prefix[i];
        ++i;
    }
    fh_wifi_status_text[i] = '\0';
}

static void __not_in_flash_func(fh_set_offline_status)(void)
{
    fh_copy_status_prefix("Offline");
}

static bool __not_in_flash_func(fh_wifi_wait_byte)(uint8_t *data_out, uint32_t deadline)
{
    while ((int32_t)(time_us_32() - deadline) < 0)
    {
        wifi_service_rx();
        fh_service_msx_reads();
        if (wifi_pop_rx_byte(data_out)) return true;
    }
    return false;
}

static bool __not_in_flash_func(fh_wifi_query_ap_status)(uint8_t *payload, uint16_t *payload_len)
{
    uint8_t data;
    uint8_t size_hi;
    uint8_t size_lo;
    uint16_t size;
    uint16_t keep;
    uint32_t deadline;

    wifi_uart_init_once();
    wifi_hw_rx_drain();
    wifi_reset_fifo();

    uart_putc_raw(WIFI_UART_INSTANCE, 'g');
    wifi_tx_busy_deadline_us = time_us_32() + wifi_uart_frame_time_us();
    deadline = time_us_32() + 1000000u;

    do {
        if (!fh_wifi_wait_byte(&data, deadline)) return false;
    } while (data != 'g');

    if (!fh_wifi_wait_byte(&data, deadline) || data != 0u) return false;
    if (!fh_wifi_wait_byte(&size_hi, deadline)) return false;
    if (!fh_wifi_wait_byte(&size_lo, deadline)) return false;

    size = (uint16_t)(((uint16_t)size_hi << 8) | size_lo);
    if (size == 0u) return false;

    keep = size;
    if (keep > 33u) keep = 33u;
    for (uint16_t i = 0; i < size; ++i)
    {
        if (!fh_wifi_wait_byte(&data, deadline)) return false;
        if (i < keep) payload[i] = data;
    }

    *payload_len = keep;
    return true;
}

static bool __not_in_flash_func(fh_wifi_ap_connected)(void)
{
    uint8_t payload[33];
    uint16_t payload_len = 0;

    return fh_wifi_query_ap_status(payload, &payload_len) &&
           payload_len >= 2u && payload[0] == 5u && payload[1] != 0u;
}

static bool __not_in_flash_func(fh_wifi_wait_connected)(uint32_t timeout_us)
{
    uint32_t deadline = time_us_32() + timeout_us;

    while ((int32_t)(time_us_32() - deadline) < 0)
    {
        if (fh_wifi_ap_connected()) return true;
        fh_copy_status_prefix("Waiting for Wi-Fi...");
        fh_service_msx_reads_for(250000u);
    }

    return fh_wifi_ap_connected();
}

static void __not_in_flash_func(fh_update_wifi_status_text)(void)
{
    uint8_t payload[33];
    uint16_t payload_len = 0;
    uint16_t out = 0;
    const char *prefix = "Connected to ";

    fh_set_offline_status();
    if (!fh_wifi_query_ap_status(payload, &payload_len)) return;
    if (payload_len < 2u || payload[0] != 5u || payload[1] == 0u) return;

    while (out + 1u < FH_STATUS_TEXT_SIZE && prefix[out] != '\0')
    {
        fh_wifi_status_text[out] = prefix[out];
        ++out;
    }

    for (uint16_t i = 1; i < payload_len && out + 1u < FH_STATUS_TEXT_SIZE; ++i)
    {
        uint8_t ch = payload[i];
        if (ch == 0u) break;
        fh_wifi_status_text[out++] = (char)ch;
    }
    fh_wifi_status_text[out] = '\0';
}

static inline void __not_in_flash_func(handle_fh_write)(uint16_t addr, uint8_t data, void *ctx)
{
    bool *exit_requested = (bool *)ctx;

    if (addr == MONITOR_ADDR && data == ROM_SELECT_MENU)
    {
        *exit_requested = true;
        fh_result = 1;
        fh_status = FH_STATUS_READY;
        return;
    }

    if (addr >= FH_QUERY_BASE && addr < (FH_QUERY_BASE + FH_QUERY_SIZE))
    {
        fh_query[addr - FH_QUERY_BASE] = data;
        return;
    }

    if (addr == FH_PAGE)
    {
        fh_page_index = data;
        return;
    }
    if (addr == FH_SELECT_L)
    {
        fh_selected_index = (uint16_t)((fh_selected_index & 0xFF00u) | data);
        return;
    }
    if (addr == FH_SELECT_H)
    {
        fh_selected_index = (uint16_t)((fh_selected_index & 0x00FFu) | ((uint16_t)data << 8));
        return;
    }
    if (addr == FH_CMD)
    {
        fh_status = FH_STATUS_BUSY;
        if (data == FH_CMD_LIST_PAGE || data == FH_CMD_SEARCH)
        {
            if (data == FH_CMD_SEARCH || !fh_catalog_loaded)
            {
                fh_copy_query_from_buffer(fh_active_query, sizeof(fh_active_query));
                if (!fh_fetch_catalog(fh_active_query))
                    fh_set_catalog_message("Offline or File Hunter unavailable");
                else if (fh_catalog_count == 0u)
                    fh_set_catalog_message("No File Hunter ROMs found");
            }
            fh_build_page_buffer();
            fh_result = fh_catalog_count ? 1u : 0u;
        }
        else if (data == FH_CMD_DOWNLOAD)
        {
            fh_progress_percent = 0;
            fh_result = 0;
            if (!fh_download_selected())
                fh_result = 0u;
        }
        else if (data == FH_CMD_EXIT)
        {
            *exit_requested = true;
            fh_result = 1;
        }
        else if (data == FH_CMD_WIFI_STATUS)
        {
            fh_update_wifi_status_text();
            fh_result = 1;
        }
        fh_status = FH_STATUS_READY;
        return;
    }
}

static int __no_inline_not_in_flash_func(loadrom_filehunter)(void)
{
    msx_pio_bus_init();

    bool exit_requested = false;
    fh_page_index = 0;
    fh_result = 0;
    fh_status = FH_STATUS_READY;
    memset(fh_query, 0, sizeof(fh_query));
    strncpy((char *)fh_query, FH_DEFAULT_QUERY, sizeof(fh_query) - 1u);
    strncpy(fh_active_query, FH_DEFAULT_QUERY, sizeof(fh_active_query) - 1u);
    fh_catalog_count = 0;
    fh_catalog_loaded = false;
    fh_set_catalog_message("Retrieving File Hunter ROMs...");
    fh_set_offline_status();
    wifi_uart_init_once();

    while (true)
    {
        wifi_service_rx();

        uint16_t waddr;
        uint8_t wdata;
        while (pio_try_get_write(&waddr, &wdata))
        {
            handle_fh_write(waddr, wdata, &exit_requested);
        }

        if (pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            if (exit_requested &&
                !gpio_get(PIN_RD) &&
                ((gpio_get_all() & 0xFFFFu) == 0x0000u))
            {
                return ROM_SELECT_MENU;
            }
            tight_loop_contents();
            continue;
        }

        uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
        bool in_window = false;
        uint8_t data = 0xFFu;

        if (exit_requested && addr == 0x0000u)
        {
            pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(false, 0xFFu));
            return ROM_SELECT_MENU;
        }

        data = fh_read_window_byte(addr, &in_window);

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }

    return ROM_SELECT_MENU;
}

//load the MSX Menu ROM into the MSX
int __no_inline_not_in_flash_func(loadrom_msx_menu)(uint32_t offset)
{
    if (!psram_prepare_rom_cache())
    {
        return 0xFFFF;
    }

    //setup the rom_sram buffer for the 32KB ROM
    gpio_init(PIN_WAIT); // Init wait signal pin
    gpio_set_dir(PIN_WAIT, GPIO_OUT); // Set the WAIT signal as output
    gpio_put(PIN_WAIT, 0); // Wait until we are ready to read the ROM
    memset(rom_sram, 0, MENU_ROM_SIZE); // Clear the SRAM buffer
    memcpy(rom_sram, flash_rom + offset, MENU_ROM_SIZE); // Load full 32KB menu ROM
    gpio_put(PIN_WAIT, 1); // Lets go!

    int record_count = 0; // Record count
    const uint8_t *record_ptr = flash_rom + offset + MENU_ROM_SIZE; // Pointer to the ROM records
    for (int i = 0; i < MAX_FLASH_RECORDS; i++)      // Read the ROMs from the configuration area
    {
        if (isEndOfData(record_ptr)) {
            break; // Stop if end of data is reached
        }
        char flash_name[ROM_NAME_MAX + 1];
        memcpy(flash_name, record_ptr, ROM_NAME_MAX);
        flash_name[ROM_NAME_MAX] = '\0';
        memset(flash_records[record_count].Name, 0, sizeof(flash_records[record_count].Name));
        trim_name_copy(flash_records[record_count].Name, flash_name);
        record_ptr += ROM_NAME_MAX; // Move the pointer to the next field
        flash_records[record_count].Mapper = *record_ptr++; // Read the mapper code
        flash_records[record_count].Size = read_ulong(record_ptr); // Read the ROM size
        record_ptr += sizeof(unsigned long); // Move the pointer to the next field
        flash_records[record_count].Offset = read_ulong(record_ptr); // Read the ROM offset
        record_ptr += sizeof(unsigned long); // Move the pointer to the next record
        record_count++; // Increment the record count
    }
    flash_record_count = (uint16_t)record_count;
    set_root_path();
    refresh_records_for_current_path();

    // Core 1 is intentionally NOT launched here. Folder navigation,
    // mapper detection, and SD enumeration all run on Core 0 from the
    // menu loop's idle branch, so the SD card has a single owner and
    // Core 1 cannot contend on shared buses. Core 1 is lazy-started
    // by ensure_mp3_core1_started() the first time an MP3 command is
    // dispatched. The cross-core command queue is a small FIFO so
    // SELECT+PLAY (issued ~10 ms apart by the menu) both fit while
    // mp3_init() is still running on Core 1.

    uint16_t rom_index = 0;
    gpio_set_dir_in_masked(0xFF << 16); // Set data bus to input mode
    bool rom_selected = false; // ROM selected flag
    rom_cached_size = MENU_ROM_SIZE;
    msx_pio_bus_init();

    explorer_menu_ctx_t menu_ctx = {0};

    while (true)
    {
        pio_drain_writes(handle_menu_write_explorer, &menu_ctx);

        if (pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read)) {
            // PIO idle (SLTSL high).  On MSX1 the BIOS never selects
            // the cartridge at addr 0x0000 after rst 0x00, so detect
            // the reboot on the raw bus via GPIO.
            if (menu_ctx.rom_selected &&
                !gpio_get(PIN_RD) &&
                ((gpio_get_all() & 0xFFFFu) == 0x0000u))
            {
                return menu_ctx.rom_index;
            }
            // Background work runs on Core 0 during MSX bus idle: chunked
            // SD directory refresh, mapper detection, and bridging MP3
            // commands to Core 1 (lazy-launched on first MP3 command).
            core1_bg_work();
            tight_loop_contents();
            continue;
        }

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        pio_drain_writes(handle_menu_write_explorer, &menu_ctx);

        bool in_window = false;
        uint8_t data = 0xFFu;
        bool drive = false;

        if (menu_ctx.rom_selected && addr == 0x0000u)
        {
            pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(false, 0xFFu));
            return menu_ctx.rom_index;
        }

        if (addr >= 0x4000u && addr <= 0xBFFFu)
        {
            in_window = true;
            drive = true;

            if (addr >= CTRL_BASE_ADDR && addr <= (CTRL_BASE_ADDR + 0x0F))
            {
                if (fh_menu_window_active)
                {
                    switch (addr)
                    {
                        case CTRL_COUNT_L: data = (uint8_t)(fh_catalog_count & 0xFFu); break;
                        case CTRL_COUNT_H: data = (uint8_t)((fh_catalog_count >> 8) & 0xFFu); break;
                        case CTRL_PAGE:    data = fh_page_index; break;
                        case CTRL_STATUS:  data = fh_status; break;
                        case CTRL_CMD:     data = ctrl_cmd_state; break;
                        case CTRL_MATCH_L: data = (uint8_t)(fh_selected_index & 0xFFu); break;
                        case CTRL_MATCH_H: data = (uint8_t)((fh_selected_index >> 8) & 0xFFu); break;
                        case CTRL_MAPPER:  data = (uint8_t)(fh_progress_percent & 0xFFu); break;
                        case CTRL_ACK:     data = (uint8_t)((fh_progress_percent >> 8) & 0xFFu); break;
                        case CTRL_AUDIO:   data = fh_result; break;
                        case CTRL_WIFI_SUPPORT: data = 0; break;
                    }
                }
                else switch (addr)
                {
                    case CTRL_COUNT_L: data = (uint8_t)(total_record_count & 0xFFu); break;
                    case CTRL_COUNT_H: data = (uint8_t)((total_record_count >> 8) & 0xFFu); break;
                    case CTRL_PAGE:    data = current_page; break;
                    case CTRL_STATUS:  data = CTRL_MAGIC; break;
                    case CTRL_CMD:     data = ctrl_cmd_state; break;
                    case CTRL_MATCH_L: data = (uint8_t)(match_index & 0xFFu); break;
                    case CTRL_MATCH_H: data = (uint8_t)((match_index >> 8) & 0xFFu); break;
                    case CTRL_MAPPER:  data = ctrl_mapper_value; break;
                    case CTRL_ACK:     data = ctrl_ack_value; break;
                    case CTRL_AUDIO:   data = ctrl_audio_selection; break;
                    case CTRL_WIFI_SUPPORT: data = ctrl_wifi_support; break;
                }
            }
            else if (addr >= MP3_NOW_PLAYING_BASE && addr < (MP3_NOW_PLAYING_BASE + MP3_NOW_PLAYING_SIZE))
            {
                data = mp3_now_playing_buf[addr - MP3_NOW_PLAYING_BASE];
            }
            else if (addr >= MP3_CTRL_BASE && addr <= (MP3_CTRL_BASE + 0x0F))
            {
                uint16_t elapsed = mp3_get_elapsed_seconds();
                uint16_t total = mp3_get_total_seconds();
                switch (addr)
                {
                    case MP3_CTRL_STATUS:    data = mp3_get_status(); break;
                    case MP3_CTRL_ELAPSED_L: data = (uint8_t)(elapsed & 0xFFu); break;
                    case MP3_CTRL_ELAPSED_H: data = (uint8_t)((elapsed >> 8) & 0xFFu); break;
                    case MP3_CTRL_TOTAL_L:   data = (uint8_t)(total & 0xFFu); break;
                    case MP3_CTRL_TOTAL_H:   data = (uint8_t)((total >> 8) & 0xFFu); break;
                    case MP3_CTRL_INDEX_L:   data = (uint8_t)(mp3_selected_index & 0xFFu); break;
                    case MP3_CTRL_INDEX_H:   data = (uint8_t)((mp3_selected_index >> 8) & 0xFFu); break;
                    case MP3_CTRL_MODE:      data = mp3_play_mode; break;
                }
            }
            else if (fh_menu_window_active && addr >= FH_STATUS_TEXT_BASE && addr < (FH_STATUS_TEXT_BASE + FH_STATUS_TEXT_SIZE))
            {
                data = (uint8_t)fh_wifi_status_text[addr - FH_STATUS_TEXT_BASE];
            }
            else if (addr >= DATA_BASE_ADDR && addr < (DATA_BASE_ADDR + DATA_BUFFER_SIZE))
            {
                data = page_buffer[addr - DATA_BASE_ADDR];
            }
            else
            {
                uint32_t rel = addr - 0x4000u;
                if (rel < MENU_ROM_SIZE)
                {
                    data = rom_sram[rel];
                }
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(drive, data));
    }
    // Old implementation disabled
    /*
    while (true)
    {
        // Check control signals
        bool sltsl = !(gpio_get(PIN_SLTSL)); // Slot selected (active low)
        bool rd = !(gpio_get(PIN_RD));       // Read cycle (active low)

        uint16_t addr = gpio_get_all() & 0x00FFFF; // Read the address bus
        //uint16_t addr = sio_hw->gpio_in & 0x00FFFF;
        if (sltsl) 
        {
            bool wr = !(gpio_get(PIN_WR));       // Write cycle (active low, not used)

            if (wr && addr >= CTRL_QUERY_BASE && addr < (CTRL_QUERY_BASE + CTRL_QUERY_SIZE))
            {
                    uint8_t value = (gpio_get_all() >> 16) & 0xFF;
                    filter_query[addr - CTRL_QUERY_BASE] = (char)value;
                    while (!(gpio_get(PIN_WR))) {
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16);
            }

            if (wr && addr == CTRL_CMD)
            {
                    uint8_t cmd = (gpio_get_all() >> 16) & 0xFF;
                    ctrl_cmd_state = cmd;
                    if (cmd == CMD_APPLY_FILTER)
                    {
                        filter_query[CTRL_QUERY_SIZE - 1] = '\0';
                        apply_filter();
                        current_page = 0;
                        build_page_buffer(current_page);
                    }
                    else if (cmd == CMD_FIND_FIRST)
                    {
                        filter_query[CTRL_QUERY_SIZE - 1] = '\0';
                        find_first_match();
                    }
                    else if (cmd == CMD_ENTER_DIR)
                    {
                        filter_query[CTRL_QUERY_SIZE - 1] = '\0';
                        if (filter_query[0] == '\0') {
                            go_up_one_level();
                        } else {
                            append_folder_to_path(filter_query);
                        }
                        refresh_requested = true;
                        ctrl_cmd_state = CMD_ENTER_DIR;
                    }
                    else if (cmd == CMD_DETECT_MAPPER)
                    {
                        uint16_t index = (uint16_t)((uint8_t)filter_query[0]) |
                                         (uint16_t)(((uint8_t)filter_query[1]) << 8);
                        detect_mapper_index = index;
                        detect_mapper_pending = true;
                        ctrl_mapper_value = 0;
                        ctrl_cmd_state = CMD_DETECT_MAPPER;
                    }
                    else if (cmd == CMD_SET_MAPPER)
                    {
                        uint16_t index = (uint16_t)((uint8_t)filter_query[0]) |
                                         (uint16_t)(((uint8_t)filter_query[1]) << 8);
                        uint8_t mapper = (uint8_t)filter_query[2];
                        ctrl_ack_value = 0;

                        if (index < total_record_count && mapper != 0 && !is_system_mapper(mapper) && mapper < MAPPER_DESCRIPTION_COUNT) {
                            uint16_t record_index = filtered_indices[index];
                            if (record_index < full_record_count) {
                                ROMRecord *rec = &records[record_index];
                                uint8_t flags = rec->Mapper & (SOURCE_SD_FLAG | FOLDER_FLAG | MP3_FLAG);
                                if ((flags & (FOLDER_FLAG | MP3_FLAG)) == 0) {
                                    rec->Mapper = (uint8_t)(flags | OVERRIDE_FLAG | mapper);
                                    ctrl_ack_value = 1;
                                }
                            }
                        }
                        ctrl_cmd_state = 0;
                    }
                    else if (cmd == CMD_SET_SOURCE)
                    {
                        uint8_t mode = (uint8_t)filter_query[0];
                        if (mode > SOURCE_MODE_SD) {
                            mode = SOURCE_MODE_ALL;
                        }
                        browse_source_mode = mode;
                        sd_current_path[0] = '/';
                        sd_current_path[1] = '\0';
                        memset(filter_query, 0, sizeof(filter_query));
                        refresh_requested = true;
                        ctrl_cmd_state = 0;
                    }
                    if (cmd != CMD_ENTER_DIR && cmd != CMD_DETECT_MAPPER) {
                        ctrl_cmd_state = 0;
                    }
                    while (!(gpio_get(PIN_WR))) {
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16);
            }

            if (wr && addr == MP3_CTRL_INDEX_L)
            {
                    uint8_t value = (gpio_get_all() >> 16) & 0xFF;
                    mp3_selected_index = (uint16_t)((mp3_selected_index & 0xFF00u) | value);
                    while (!(gpio_get(PIN_WR))) {
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16);
            }

            if (wr && addr == MP3_CTRL_INDEX_H)
            {
                    uint8_t value = (gpio_get_all() >> 16) & 0xFF;
                    mp3_selected_index = (uint16_t)((mp3_selected_index & 0x00FFu) | ((uint16_t)value << 8));
                    while (!(gpio_get(PIN_WR))) {
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16);
            }

            if (wr && addr == MP3_CTRL_CMD)
            {
                    uint8_t cmd = (gpio_get_all() >> 16) & 0xFF;

                    if (cmd == MP3_CMD_SELECT)
                    {
                        if (mp3_selected_index < total_record_count)
                        {
                            mp3_pending_index = mp3_selected_index;
                            mp3_pending_select = true;
                        }
                    }
                    else if (cmd == MP3_CMD_PLAY || cmd == MP3_CMD_STOP || cmd == MP3_CMD_TOGGLE_MUTE)
                    {
                        mp3_pending_cmd = cmd;
                    }

                    while (!(gpio_get(PIN_WR))) {
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16);
            }

            if (wr && addr == CTRL_PAGE)
            {
                    uint8_t page = (gpio_get_all() >> 16) & 0xFF;
                    if (page != current_page)
                    {
                        current_page = page;
                        build_page_buffer(current_page);
                    }
                    while (!(gpio_get(PIN_WR))) { // Wait until the write cycle completes (WR goes high)
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16); // Set data bus to input mode
            }

            if (wr && addr == MONITOR_ADDR) // Monitor ROM address to select the ROM
            {   
                    uint8_t selected_index = (gpio_get_all() >> 16) & 0xFF;
                    if (selected_index < total_record_count) {
                        rom_index = filtered_indices[selected_index];
                    } else {
                        rom_index = 0;
                    }
                    while (!(gpio_get(PIN_WR))) { // Wait until the write cycle completes (WR goes high){
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16); // Set data bus to input mode
                    rom_selected = true;    // ROM selected
            }

            if (addr >= 0x4000 && addr <= 0xBFFF) // Check if the address is within the ROM range
            {   
                if (rd)
                {
                    if (addr >= CTRL_BASE_ADDR && addr <= (CTRL_BASE_ADDR + 0x0F))
                    {
                        uint8_t ctrl_value = 0xFF;
                        switch (addr)
                        {
                            case CTRL_COUNT_L:
                                ctrl_value = (uint8_t)(total_record_count & 0xFFu);
                                break;
                            case CTRL_COUNT_H:
                                ctrl_value = (uint8_t)((total_record_count >> 8) & 0xFFu);
                                break;
                            case CTRL_PAGE:
                                ctrl_value = current_page;
                                break;
                            case CTRL_STATUS:
                                ctrl_value = CTRL_MAGIC;
                                break;
                            case CTRL_CMD:
                                ctrl_value = ctrl_cmd_state;
                                break;
                            case CTRL_MATCH_L:
                                ctrl_value = (uint8_t)(match_index & 0xFFu);
                                break;
                            case CTRL_MATCH_H:
                                ctrl_value = (uint8_t)((match_index >> 8) & 0xFFu);
                                break;
                            case CTRL_MAPPER:
                                ctrl_value = ctrl_mapper_value;
                                break;
                            case CTRL_ACK:
                                ctrl_value = ctrl_ack_value;
                                break;
                        }
                        gpio_set_dir_out_masked(0xFF << 16); // Set data bus to output mode
                        gpio_put_masked(0xFF0000, (uint32_t)ctrl_value << 16);
                        while (!(gpio_get(PIN_RD))) {
                            tight_loop_contents();
                        }
                        gpio_set_dir_in_masked(0xFF << 16);
                        continue;
                    }

                    if (addr >= MP3_CTRL_BASE && addr <= (MP3_CTRL_BASE + 0x0F))
                    {
                        uint8_t ctrl_value = 0xFF;
                        uint16_t elapsed = mp3_get_elapsed_seconds();
                        uint16_t total = mp3_get_total_seconds();

                        switch (addr)
                        {
                            case MP3_CTRL_STATUS:
                                ctrl_value = mp3_get_status();
                                break;
                            case MP3_CTRL_ELAPSED_L:
                                ctrl_value = (uint8_t)(elapsed & 0xFFu);
                                break;
                            case MP3_CTRL_ELAPSED_H:
                                ctrl_value = (uint8_t)((elapsed >> 8) & 0xFFu);
                                break;
                            case MP3_CTRL_TOTAL_L:
                                ctrl_value = (uint8_t)(total & 0xFFu);
                                break;
                            case MP3_CTRL_TOTAL_H:
                                ctrl_value = (uint8_t)((total >> 8) & 0xFFu);
                                break;
                            case MP3_CTRL_INDEX_L:
                                ctrl_value = (uint8_t)(mp3_selected_index & 0xFFu);
                                break;
                            case MP3_CTRL_INDEX_H:
                                ctrl_value = (uint8_t)((mp3_selected_index >> 8) & 0xFFu);
                                break;
                        }
                        gpio_set_dir_out_masked(0xFF << 16);
                        gpio_put_masked(0xFF0000, (uint32_t)ctrl_value << 16);
                        while (!(gpio_get(PIN_RD))) {
                            tight_loop_contents();
                        }
                        gpio_set_dir_in_masked(0xFF << 16);
                        continue;
                    }

                    gpio_set_dir_out_masked(0xFF << 16); // Set data bus to output mode
                    uint8_t data;
                    if (addr >= DATA_BASE_ADDR && addr < (DATA_BASE_ADDR + DATA_BUFFER_SIZE)) {
                        data = page_buffer[addr - DATA_BASE_ADDR];
                    } else {
                        uint32_t rom_addr = (uint32_t)(addr - 0x4000); // Offset within 32KB menu ROM
                        data = rom_sram[rom_addr];
                    }
                    gpio_put_masked(0xFF0000, (uint32_t)data << 16); // Write the data to the data bus
                    while (!(gpio_get(PIN_RD))) { // Wait until the read cycle completes (RD goes high)
                        tight_loop_contents();
                    }
                    gpio_set_dir_in_masked(0xFF << 16); // Set data bus to input mode
                }
            } 
        }

        if (rd && addr == 0x0000 && rom_selected)   // lets return the rom_index and load the selected ROM
        {
            return rom_index;
        }
    }
    */
}

// loadrom_plain32 - Load a simple 32KB (or less) ROM into the MSX directly from the pico flash
// 32KB ROMS have two pages of 16Kb each in the following areas:
// 0x4000-0x7FFF and 0x8000-0xBFFF
// AB is on 0x0000, 0x0001
// 16KB ROMS have only one page in the 0x4000-0x7FFF area
// AB is on 0x0000, 0x0001
void __no_inline_not_in_flash_func(loadrom_plain32)(uint32_t offset, bool cache_enable)
{
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 32768u, &rom_base, &available_length);

    msx_pio_bus_init();

    while (true)
    {
        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint32_t rel = addr - 0x4000u;
            if (available_length == 0u || rel < available_length)
                data = read_rom_byte(rom_base, rel);
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// loadrom_linear48 - Load a simple 48KB Linear0 ROM into the MSX directly from the pico flash
// Those ROMs have three pages of 16Kb each in the following areas:
// 0x0000-0x3FFF, 0x4000-0x7FFF and 0x8000-0xBFFF
// AB is on 0x4000, 0x4001
void __no_inline_not_in_flash_func(loadrom_linear48)(uint32_t offset, bool cache_enable)
{
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 49152u, &rom_base, &available_length);

    msx_pio_bus_init();

    while (true)
    {
        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        bool in_window = (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;
        if (in_window && (available_length == 0u || addr < available_length))
            data = read_rom_byte(rom_base, addr);

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// loadrom_konamiscc - Load a any Konami SCC ROM into the MSX directly from the pico flash
// The KonamiSCC ROMs are divided into 8KB segments, managed by a memory mapper that allows dynamic switching of these segments 
// into the MSX's address space. Since the size of the mapper is 8Kb, the memory banks are:
// Bank 1: 4000h - 5FFFh , Bank 2: 6000h - 7FFFh, Bank 3: 8000h - 9FFFh, Bank 4: A000h - BFFFh
// And the address to change banks are:
// Bank 1: 5000h - 57FFh (5000h used), Bank 2: 7000h - 77FFh (7000h used), Bank 3: 9000h - 97FFh (9000h used), Bank 4: B000h - B7FFh (B000h used)
// AB is on 0x0000, 0x0001
void __no_inline_not_in_flash_func(loadrom_konamiscc)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    msx_pio_bus_init();
    banked8_loop(rom_base, available_length, bank_registers, handle_konamiscc_write);
}

// -----------------------------------------------------------------------
// Core 1 audio entry: generates SCC samples and pushes to I2S
// -----------------------------------------------------------------------
static void __no_inline_not_in_flash_func(core1_scc_audio)(void)
{
    while (true)
    {
        struct audio_buffer *buffer = take_audio_buffer(scc_audio_pool, true);
        int16_t *samples = (int16_t *)buffer->buffer->bytes;
        for (int i = 0; i < SCC_AUDIO_BUFFER_SAMPLES; i++)
        {
            int16_t raw = SCC_calc(&scc_instance);
            int32_t boosted = (int32_t)raw << SCC_VOLUME_SHIFT;
            if (boosted > 32767) boosted = 32767;
            else if (boosted < -32768) boosted = -32768;
            int16_t s = (int16_t)boosted;
            samples[i * 2]     = s;  // left
            samples[i * 2 + 1] = s;  // right
        }
        buffer->sample_count = SCC_AUDIO_BUFFER_SAMPLES;
        give_audio_buffer(scc_audio_pool, buffer);
    }
}

// -----------------------------------------------------------------------
// I2S audio initialisation for SCC emulation
// -----------------------------------------------------------------------
static void i2s_audio_init_scc(void)
{
    gpio_init(I2S_MUTE_PIN);
    gpio_set_dir(I2S_MUTE_PIN, GPIO_OUT);
    gpio_put(I2S_MUTE_PIN, 0);

    static audio_format_t scc_audio_format = {
        .sample_freq = SCC_SAMPLE_RATE,
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .channel_count = 2,
    };

    static struct audio_buffer_format scc_producer_format = {
        .format = &scc_audio_format,
        .sample_stride = 4,
    };

    scc_audio_pool = audio_new_producer_pool(&scc_producer_format, 3, SCC_AUDIO_BUFFER_SAMPLES);

    // Find a free DMA channel (SD SPI driver claims channels at init)
    int scc_dma_channel = -1;
    for (int ch = 0; ch < NUM_DMA_CHANNELS; ch++) {
        if (!dma_channel_is_claimed(ch)) {
            scc_dma_channel = ch;
            break;
        }
    }
    if (scc_dma_channel < 0) {
        return;
    }

    static struct audio_i2s_config scc_i2s_config = {
        .data_pin = I2S_DATA_PIN,
        .clock_pin_base = I2S_BCLK_PIN,
        .dma_channel = 0,
        // Audio I2S lives on PIO1 SM2 so it never collides with the MSX
        // memory bus PIO programs (pio0 SM0/SM1) or the I/O bus
        // responder (pio1 SM0/SM1). Matches multirom's layout.
        .pio_sm = 2,
    };
    scc_i2s_config.dma_channel = (uint)scc_dma_channel;

    audio_i2s_setup(&scc_audio_format, &scc_i2s_config);
    audio_i2s_connect(scc_audio_pool);
    audio_i2s_set_enabled(true);
}

// -----------------------------------------------------------------------
// Konami SCC mapper with SCC sound emulation via I2S
// -----------------------------------------------------------------------
void __no_inline_not_in_flash_func(loadrom_konamiscc_scc)(uint32_t offset, bool cache_enable, uint32_t scc_type)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    // Hold WAIT to freeze Z80 during audio subsystem initialization,
    // preventing BIOS from scanning the slot before we are ready.
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 0);

    // Initialise SCC emulator (static allocation, no malloc)
    memset(&scc_instance, 0, sizeof(SCC));
    scc_instance.clk  = SCC_CLOCK;
    scc_instance.rate = SCC_SAMPLE_RATE;
    SCC_set_quality(&scc_instance, 1);
    scc_instance.type = scc_type;
    SCC_reset(&scc_instance);

    // Start I2S audio subsystem, then launch audio on core 1
    i2s_audio_init_scc();
    multicore_launch_core1(core1_scc_audio);

    msx_pio_bus_init();

    while (true)
    {
        // --- drain pending writes ---
        uint16_t waddr;
        uint8_t  wdata;
        while (pio_try_get_write(&waddr, &wdata))
        {
            if      (waddr >= 0x5000u && waddr <= 0x57FFu) bank_registers[0] = wdata;
            else if (waddr >= 0x7000u && waddr <= 0x77FFu) bank_registers[1] = wdata;
            else if (waddr >= 0x9000u && waddr <= 0x97FFu) bank_registers[2] = wdata;
            else if (waddr >= 0xB000u && waddr <= 0xB7FFu) bank_registers[3] = wdata;

            // Forward to SCC emulator (handles enable + register writes)
            SCC_write(&scc_instance, waddr, wdata);
        }

        // --- handle read request ---
        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        // drain writes that arrived while waiting
        while (pio_try_get_write(&waddr, &wdata))
        {
            if      (waddr >= 0x5000u && waddr <= 0x57FFu) bank_registers[0] = wdata;
            else if (waddr >= 0x7000u && waddr <= 0x77FFu) bank_registers[1] = wdata;
            else if (waddr >= 0x9000u && waddr <= 0x97FFu) bank_registers[2] = wdata;
            else if (waddr >= 0xB000u && waddr <= 0xB7FFu) bank_registers[3] = wdata;
            SCC_write(&scc_instance, waddr, wdata);
        }

        bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            // Check if address falls in active SCC register read region.
            bool is_scc_read = false;
            if (scc_instance.active)
            {
                uint32_t scc_reg_start = scc_instance.base_adr + 0x800u;
                if (addr >= scc_reg_start && addr <= (scc_reg_start + 0xFFu))
                    is_scc_read = true;
            }
            // SCC+ mode register at 0xBFFE-0xBFFF
            if (scc_type == SCC_ENHANCED && (addr & 0xFFFEu) == 0xBFFEu)
                is_scc_read = true;

            if (is_scc_read)
            {
                data = (uint8_t)SCC_read(&scc_instance, addr);
            }
            else
            {
                uint32_t rel = ((uint32_t)bank_registers[(addr - 0x4000u) >> 13] * 0x2000u)
                             + (addr & 0x1FFFu);
                if (available_length == 0u || rel < available_length)
                    data = read_rom_byte(rom_base, rel);
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// loadrom_konami - Load a Konami (without SCC) ROM into the MSX directly from the pico flash
// The Konami (without SCC) ROM is divided into 8KB segments, managed by a memory mapper that allows dynamic switching of these segments into the MSX's address space
// Since the size of the mapper is 8Kb, the memory banks are:
//  Bank 1: 4000h - 5FFFh, Bank 2: 6000h - 7FFFh, Bank 3: 8000h - 9FFFh, Bank 4: A000h - BFFFh
// And the addresses to change banks are:
//	Bank 1: <none>, Bank 2: 6000h - 67FFh (6000h used), Bank 3: 8000h - 87FFh (8000h used), Bank 4: A000h - A7FFh (A000h used)
// AB is on 0x0000, 0x0001
void __no_inline_not_in_flash_func(loadrom_konami)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    msx_pio_bus_init();
    banked8_loop(rom_base, available_length, bank_registers, handle_konami_write);
}

// loadrom_ascii8 - Load an ASCII8 ROM into the MSX directly from the pico flash
// The ASCII8 ROM is divided into 8KB segments, managed by a memory mapper that allows dynamic switching of these segments into the MSX's address space
// Since the size of the mapper is 8Kb, the memory banks are:
// Bank 1: 4000h - 5FFFh , Bank 2: 6000h - 7FFFh, Bank 3: 8000h - 9FFFh, Bank 4: A000h - BFFFh
// And the address to change banks are:
// Bank 1: 6000h - 67FFh (6000h used), Bank 2: 6800h - 6FFFh (6800h used), Bank 3: 7000h - 77FFh (7000h used), Bank 4: 7800h - 7FFFh (7800h used)
// AB is on 0x0000, 0x0001
void __no_inline_not_in_flash_func(loadrom_ascii8)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    msx_pio_bus_init();
    banked8_loop(rom_base, available_length, bank_registers, handle_ascii8_write);
}

// loadrom_ascii16 - Load an ASCII16 ROM into the MSX directly from the pico flash
// The ASCII16 ROM is divided into 16KB segments, managed by a memory mapper that allows dynamic switching of these segments into the MSX's address space
// Since the size of the mapper is 16Kb, the memory banks are:
// Bank 1: 4000h - 7FFFh , Bank 2: 8000h - BFFFh
// And the address to change banks are:
// Bank 1: 6000h - 67FFh (6000h used), Bank 2: 7000h - 77FFh (7000h and 77FFh used)
void __no_inline_not_in_flash_func(loadrom_ascii16)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[2] = {0, 1};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    msx_pio_bus_init();

    bank8_ctx_t ctx = { .bank_regs = bank_registers };

    while (true)
    {
        pio_drain_writes(handle_ascii16_write, &ctx);

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        pio_drain_writes(handle_ascii16_write, &ctx);

        bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint8_t bank = (addr >> 15) & 1;
            uint32_t rel = ((uint32_t)bank_registers[bank] << 14) + (addr & 0x3FFFu);
            if (available_length == 0u || rel < available_length)
                data = read_rom_byte(rom_base, rel);
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}


// loadrom_neo8 - Load an NEO8 ROM into the MSX directly from the pico flash
// The NEO8 ROM is divided into 8KB segments, managed by a memory mapper that allows dynamic switching of these segments into the MSX's address space
// Size of a segment: 8 KB
// Segment switching addresses:
// Bank 0: 0000h~1FFFh, Bank 1: 2000h~3FFFh, Bank 2: 4000h~5FFFh, Bank 3: 6000h~7FFFh, Bank 4: 8000h~9FFFh, Bank 5: A000h~BFFFh
// Switching address: 
// 5000h (mirror at 1000h, 9000h and D000h), 
// 5800h (mirror at 1800h, 9800h and D800h), 
// 6000h (mirror at 2000h, A000h and E000h), 
// 6800h (mirror at 2800h, A800h and E800h), 
// 7000h (mirror at 3000h, B000h and F000h), 
// 7800h (mirror at 3800h, B800h and F800h)
void __no_inline_not_in_flash_func(loadrom_neo8)(uint32_t offset)
{
    uint16_t bank_registers[6] = {0};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, false, 0u, &rom_base, &available_length);

    msx_pio_bus_init();

    bank16_ctx_t ctx = { .bank_regs = bank_registers };

    while (true)
    {
        pio_drain_writes(handle_neo8_write, &ctx);

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        pio_drain_writes(handle_neo8_write, &ctx);

        bool in_window = (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint8_t bank_index = addr >> 13;
            if (bank_index < 6u)
            {
                uint32_t segment = bank_registers[bank_index] & 0x0FFFu;
                uint32_t rel = (segment << 13) + (addr & 0x1FFFu);
                if (available_length == 0u || rel < available_length)
                    data = read_rom_byte(rom_base, rel);
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// loadrom_neo16 - Load an NEO16 ROM into the MSX directly from the pico flash
// The NEO16 ROM is divided into 16KB segments, managed by a memory mapper that allows dynamic switching of these segments into the MSX's address space
// Size of a segment: 16 KB
// Segment switching addresses:
// Bank 0: 0000h~3FFFh, Bank 1: 4000h~7FFFh, Bank 2: 8000h~BFFFh
// Switching address:
// 5000h (mirror at 1000h, 9000h and D000h),
// 6000h (mirror at 2000h, A000h and E000h),
// 7000h (mirror at 3000h, B000h and F000h)
void __no_inline_not_in_flash_func(loadrom_neo16)(uint32_t offset)
{
    uint16_t bank_registers[3] = {0};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, false, 0u, &rom_base, &available_length);

    msx_pio_bus_init();

    bank16_ctx_t ctx = { .bank_regs = bank_registers };

    while (true)
    {
        pio_drain_writes(handle_neo16_write, &ctx);

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        pio_drain_writes(handle_neo16_write, &ctx);

        bool in_window = (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint8_t bank_index = addr >> 14;
            if (bank_index < 3u)
            {
                uint32_t segment = bank_registers[bank_index] & 0x0FFFu;
                uint32_t rel = (segment << 14) + (addr & 0x3FFFu);
                if (available_length == 0u || rel < available_length)
                    data = read_rom_byte(rom_base, rel);
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

void __not_in_flash_func(service_scc_audio)(void)
{
}

void __no_inline_not_in_flash_func(loadrom_sunrise)(uint32_t offset, bool cache_enable)
{
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    static sunrise_ide_t ide;
    sunrise_ide_init(&ide);

    sunrise_usb_set_ide_ctx(&ide);
    multicore_launch_core1(sunrise_usb_task);

    msx_pio_bus_init();

    sunrise_ctx_t ctx = { .ide = &ide };

    while (true)
    {
        uint16_t addr;
        while (true)
        {
            pio_drain_writes(handle_sunrise_write, &ctx);
            if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
            {
                addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
                break;
            }
        }

        pio_drain_writes(handle_sunrise_write, &ctx);

        bool in_window = (addr >= 0x4000u) && (addr <= 0x7FFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint8_t ide_data;
            if (sunrise_ide_handle_read(&ide, addr, &ide_data))
            {
                data = ide_data;
            }
            else
            {
                uint8_t seg = ide.segment;
                uint32_t rel = ((uint32_t)seg << 14) + (addr & 0x3FFFu);
                if (available_length == 0u || rel < available_length)
                    data = read_rom_byte(rom_base, rel);
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

void __no_inline_not_in_flash_func(loadrom_sunrise_mapper)(uint32_t offset, bool cache_enable)
{
    static const uint8_t bootstrap_rom[] = {
        0x41, 0x42, 0x0A, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xF3, 0xDB, 0xF4, 0xF6, 0x80, 0xD3, 0xF4, 0xC7
    };

    msx_pio_bus_init();

    bool restart_detected = false;
    bool init_called = false;

    while (!restart_detected)
    {
        uint16_t waddr;
        uint8_t wdata;
        while (pio_try_get_write(&waddr, &wdata))
        {
        }

        if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
            if (init_called && addr == 0x0000u)
            {
                pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(false, 0xFFu));
                restart_detected = true;
            }
            else
            {
                if (addr >= 0x400Au && addr <= 0x4011u) init_called = true;
                bool in_window = (addr >= 0x4000u && addr <= 0x7FFFu);
                uint8_t data = 0xFFu;
                if (in_window)
                {
                    uint32_t rel = addr - 0x4000u;
                    if (rel < sizeof(bootstrap_rom)) data = bootstrap_rom[rel];
                }
                pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
            }
        }
        else if (init_called && !gpio_get(PIN_RD) && ((gpio_get_all() & 0xFFFFu) == 0x0000u))
        {
            restart_detected = true;
        }
    }

    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_read, false);
    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_write, false);
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 0);

    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_sunrise_mapper_rom_source(offset, cache_enable, &rom_base, &available_length);

    static sunrise_ide_t ide;
    sunrise_ide_init(&ide);

    sunrise_usb_set_ide_ctx(&ide);
    multicore_launch_core1(sunrise_usb_task);

    if (!psram_prepare_mapper_region())
    {
        while (true) { tight_loop_contents(); }
    }

    uint8_t mapper_reg[4] = { 3, 2, 1, 0 };
    uint8_t subslot_reg = 0x10;

    mapper_fill_ff();

    msx_pio_io_bus_init();
    msx_pio_bus_init();

    while (true)
    {
        uint16_t waddr;
        uint8_t wdata;
        while (pio_try_get_write(&waddr, &wdata))
        {
            if (waddr == 0xFFFFu)
                subslot_reg = wdata;
            else
            {
                uint8_t page = (waddr >> 14) & 0x03u;
                uint8_t active_subslot = (subslot_reg >> (page * 2)) & 0x03u;
                if (active_subslot == 0)
                {
                    if (waddr >= 0x4000u && waddr <= 0x7FFFu)
                        sunrise_ide_handle_write(&ide, waddr, wdata);
                }
                else if (active_subslot == 1)
                {
                    uint8_t mapper_page = mapper_page_from_reg(mapper_reg[page]);
                    uint32_t mapper_offset = ((uint32_t)mapper_page << 14) | (waddr & 0x3FFFu);
                    mapper_write_byte(mapper_offset, wdata);
                }
            }
        }

        uint16_t io_addr;
        uint8_t io_data;
        while (pio_try_get_io_write(&io_addr, &io_data))
        {
            uint8_t port = io_addr & 0xFFu;
            if (port >= 0xFCu && port <= 0xFFu)
                mapper_reg[port - 0xFCu] = io_data & 0x3Fu;
        }

        while (pio_try_get_io_read(&io_addr))
        {
            uint8_t port = io_addr & 0xFFu;
            bool in_window = false;
            uint8_t data = 0xFFu;
            if (port >= 0xFCu && port <= 0xFFu)
            {
                in_window = true;
                data = (uint8_t)(0xC0u | (mapper_reg[port - 0xFCu] & 0x3Fu));
            }
            pio_sm_put_blocking(msx_io_bus.pio_read, msx_io_bus.sm_io_read, pio_build_token(in_window, data));
        }

        if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
            uint8_t data = 0xFFu;
            bool in_window = false;

            if (addr == 0xFFFFu)
            {
                in_window = true;
                data = (uint8_t)~subslot_reg;
            }
            else
            {
                uint8_t page = (addr >> 14) & 0x03u;
                uint8_t active_subslot = (subslot_reg >> (page * 2)) & 0x03u;
                if (active_subslot == 0)
                {
                    if (addr >= 0x4000u && addr <= 0x7FFFu)
                    {
                        in_window = true;
                        uint8_t ide_data;
                        if (sunrise_ide_handle_read(&ide, addr, &ide_data))
                            data = ide_data;
                        else
                        {
                            uint8_t seg = ide.segment;
                            uint32_t rel = ((uint32_t)seg << 14) + (addr & 0x3FFFu);
                            if (available_length == 0u || rel < available_length)
                                data = read_rom_byte(rom_base, rel);
                        }
                    }
                }
                else if (active_subslot == 1)
                {
                    in_window = true;
                    uint8_t mapper_page = mapper_page_from_reg(mapper_reg[page]);
                    uint32_t mapper_offset = ((uint32_t)mapper_page << 14) | (addr & 0x3FFFu);
                    data = mapper_read_byte(mapper_offset);
                }
            }

            pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
        }
    }
}

void __no_inline_not_in_flash_func(loadrom_sunrise_sd)(uint32_t offset, bool cache_enable)
{
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    static sunrise_ide_t ide;
    sunrise_ide_init(&ide);

    sunrise_sd_set_ide_ctx(&ide);
    multicore_launch_core1(sunrise_sd_task);

    msx_pio_bus_init();

    sunrise_ctx_t ctx = { .ide = &ide };

    while (true)
    {
        uint16_t addr;
        while (true)
        {
            pio_drain_writes(handle_sunrise_write, &ctx);
            if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
            {
                addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
                break;
            }
        }

        pio_drain_writes(handle_sunrise_write, &ctx);

        bool in_window = (addr >= 0x4000u) && (addr <= 0x7FFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint8_t ide_data;
            if (sunrise_ide_handle_read(&ide, addr, &ide_data))
            {
                data = ide_data;
            }
            else
            {
                uint8_t seg = ide.segment;
                uint32_t rel = ((uint32_t)seg << 14) + (addr & 0x3FFFu);
                if (available_length == 0u || rel < available_length)
                    data = read_rom_byte(rom_base, rel);
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

void __no_inline_not_in_flash_func(loadrom_sunrise_mapper_sd)(uint32_t offset, bool cache_enable)
{
    static const uint8_t bootstrap_rom[] = {
        0x41, 0x42, 0x0A, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xF3, 0xDB, 0xF4, 0xF6, 0x80, 0xD3, 0xF4, 0xC7
    };

    msx_pio_bus_init();

    bool restart_detected = false;
    bool init_called = false;

    while (!restart_detected)
    {
        uint16_t waddr;
        uint8_t wdata;
        while (pio_try_get_write(&waddr, &wdata))
        {
        }

        if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
            if (init_called && addr == 0x0000u)
            {
                pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(false, 0xFFu));
                restart_detected = true;
            }
            else
            {
                if (addr >= 0x400Au && addr <= 0x4011u) init_called = true;
                bool in_window = (addr >= 0x4000u && addr <= 0x7FFFu);
                uint8_t data = 0xFFu;
                if (in_window)
                {
                    uint32_t rel = addr - 0x4000u;
                    if (rel < sizeof(bootstrap_rom)) data = bootstrap_rom[rel];
                }
                pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
            }
        }
        else if (init_called && !gpio_get(PIN_RD) && ((gpio_get_all() & 0xFFFFu) == 0x0000u))
        {
            restart_detected = true;
        }
    }

    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_read, false);
    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_write, false);
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 0);

    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_sunrise_mapper_rom_source(offset, cache_enable, &rom_base, &available_length);

    // Bring up PSRAM and pre-fill the 1MB mapper region BEFORE launching
    // Core 1. psram_init() runs an interrupts-disabled QMI direct-mode
    // sequence and the subsequent 1MB fill saturates QMI/XIP traffic; if
    // Core 1's SD initialisation runs concurrently from XIP, SPI timing
    // is disrupted and `disk_initialize` fails ("Master device: not
    // found").
    if (!psram_prepare_mapper_region())
    {
        while (true) { tight_loop_contents(); }
    }

    mapper_fill_ff();

    static sunrise_ide_t ide;
    sunrise_ide_init(&ide);

    sunrise_sd_set_ide_ctx(&ide);
    multicore_launch_core1(sunrise_sd_task);

    uint8_t mapper_reg[4] = { 3, 2, 1, 0 };
    uint8_t subslot_reg = 0x10;

    msx_pio_io_bus_init();
    msx_pio_bus_init();

    while (true)
    {
        uint16_t waddr;
        uint8_t wdata;
        while (pio_try_get_write(&waddr, &wdata))
        {
            if (waddr == 0xFFFFu)
                subslot_reg = wdata;
            else
            {
                uint8_t page = (waddr >> 14) & 0x03u;
                uint8_t active_subslot = (subslot_reg >> (page * 2)) & 0x03u;
                if (active_subslot == 0)
                {
                    if (waddr >= 0x4000u && waddr <= 0x7FFFu)
                        sunrise_ide_handle_write(&ide, waddr, wdata);
                }
                else if (active_subslot == 1)
                {
                    uint8_t mapper_page = mapper_page_from_reg(mapper_reg[page]);
                    uint32_t mapper_offset = ((uint32_t)mapper_page << 14) | (waddr & 0x3FFFu);
                    mapper_write_byte(mapper_offset, wdata);
                }
            }
        }

        uint16_t io_addr;
        uint8_t io_data;
        while (pio_try_get_io_write(&io_addr, &io_data))
        {
            uint8_t port = io_addr & 0xFFu;
            if (port >= 0xFCu && port <= 0xFFu)
                mapper_reg[port - 0xFCu] = io_data & 0x3Fu;
        }

        while (pio_try_get_io_read(&io_addr))
        {
            uint8_t port = io_addr & 0xFFu;
            bool in_window = false;
            uint8_t data = 0xFFu;
            if (port >= 0xFCu && port <= 0xFFu)
            {
                in_window = true;
                data = (uint8_t)(0xC0u | (mapper_reg[port - 0xFCu] & 0x3Fu));
            }
            pio_sm_put_blocking(msx_io_bus.pio_read, msx_io_bus.sm_io_read, pio_build_token(in_window, data));
        }

        if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
            uint8_t data = 0xFFu;
            bool in_window = false;

            if (addr == 0xFFFFu)
            {
                in_window = true;
                data = (uint8_t)~subslot_reg;
            }
            else
            {
                uint8_t page = (addr >> 14) & 0x03u;
                uint8_t active_subslot = (subslot_reg >> (page * 2)) & 0x03u;
                if (active_subslot == 0)
                {
                    if (addr >= 0x4000u && addr <= 0x7FFFu)
                    {
                        in_window = true;
                        uint8_t ide_data;
                        if (sunrise_ide_handle_read(&ide, addr, &ide_data))
                            data = ide_data;
                        else
                        {
                            uint8_t seg = ide.segment;
                            uint32_t rel = ((uint32_t)seg << 14) + (addr & 0x3FFFu);
                            if (available_length == 0u || rel < available_length)
                                data = read_rom_byte(rom_base, rel);
                        }
                    }
                }
                else if (active_subslot == 1)
                {
                    in_window = true;
                    uint8_t mapper_page = mapper_page_from_reg(mapper_reg[page]);
                    uint32_t mapper_offset = ((uint32_t)mapper_page << 14) | (addr & 0x3FFFu);
                    data = mapper_read_byte(mapper_offset);
                }
            }

            pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
        }
    }
}

typedef void (*sunrise_backend_task_fn_t)(void);
typedef void (*sunrise_backend_attach_fn_t)(sunrise_ide_t *ide);

static void __no_inline_not_in_flash_func(loadrom_sunrise_wifi_common)(
    uint32_t offset,
    bool cache_enable,
    sunrise_backend_task_fn_t core1_task,
    sunrise_backend_attach_fn_t attach_ctx,
    bool mapper_enable)
{
    static const uint8_t bootstrap_rom[] = {
        0x41, 0x42, 0x0A, 0x40, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0xF3, 0xDB, 0xF4, 0xF6, 0x80, 0xD3,
        0xF4, 0xC7
    };

    msx_pio_bus_init();

    bool restart_detected = false;
    bool init_called = false;

    while (!restart_detected)
    {
        uint16_t waddr;
        uint8_t wdata;
        while (pio_try_get_write(&waddr, &wdata)) { }

        if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
            if (init_called && addr == 0x0000u)
            {
                pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(false, 0xFFu));
                restart_detected = true;
            }
            else
            {
                if (addr >= 0x400Au && addr <= 0x4011u) init_called = true;
                bool in_window = (addr >= 0x4000u && addr <= 0x7FFFu);
                uint8_t data = 0xFFu;
                if (in_window)
                {
                    uint32_t rel = addr - 0x4000u;
                    if (rel < sizeof(bootstrap_rom)) data = bootstrap_rom[rel];
                }
                pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
            }
        }
        else if (init_called && !gpio_get(PIN_RD) && ((gpio_get_all() & 0xFFFFu) == 0x0000u))
        {
            restart_detected = true;
        }
    }

    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_read, false);
    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_write, false);
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 0);

    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_sunrise_mapper_rom_source(offset, cache_enable, &rom_base, &available_length);
    const uint8_t *wifi_rom_base = rom_data + WIFI_BIOS_FLASH_OFFSET;

    if (mapper_enable)
    {
        if (!psram_prepare_mapper_region())
        {
            while (true) { tight_loop_contents(); }
        }
        mapper_fill_ff();
    }

    static sunrise_ide_t ide;
    sunrise_ide_init(&ide);

    attach_ctx(&ide);
    multicore_launch_core1(core1_task);
    wifi_uart_init_once();

    uint8_t mapper_reg[4] = { 3, 2, 1, 0 };
    uint8_t subslot_reg = mapper_enable ? 0x20u : 0x00u;

    if (mapper_enable)
    {
        msx_pio_io_bus_init();
    }

    msx_pio_bus_init();

    while (true)
    {
        wifi_service_rx();

        uint16_t waddr;
        uint8_t wdata;
        while (pio_try_get_write(&waddr, &wdata))
        {
            if (waddr == 0xFFFFu)
            {
                subslot_reg = wdata;
                continue;
            }

            uint8_t page = (waddr >> 14) & 0x03u;
            uint8_t active_subslot = (subslot_reg >> (page * 2)) & 0x03u;

            if (active_subslot == 0)
            {
                (void)wifi_handle_mem_write(waddr, wdata);
            }
            else if (active_subslot == 1)
            {
                if (waddr >= 0x4000u && waddr <= 0x7FFFu)
                    sunrise_ide_handle_write(&ide, waddr, wdata);
            }
            else if (mapper_enable && active_subslot == 2)
            {
                uint8_t mapper_page = mapper_page_from_reg(mapper_reg[page]);
                uint32_t mapper_offset = ((uint32_t)mapper_page << 14) | (waddr & 0x3FFFu);
                mapper_write_byte(mapper_offset, wdata);
            }
        }

        if (mapper_enable)
        {
            uint16_t io_addr;
            uint8_t io_data;
            while (pio_try_get_io_write(&io_addr, &io_data))
            {
                uint8_t port = io_addr & 0xFFu;
                if (port >= 0xFCu && port <= 0xFFu)
                    mapper_reg[port - 0xFCu] = io_data & 0x3Fu;
            }

            while (pio_try_get_io_read(&io_addr))
            {
                uint8_t port = io_addr & 0xFFu;
                bool in_window = false;
                uint8_t data = 0xFFu;
                if (port >= 0xFCu && port <= 0xFFu)
                {
                    in_window = true;
                    data = (uint8_t)(0xC0u | (mapper_reg[port - 0xFCu] & 0x3Fu));
                }
                pio_sm_put_blocking(msx_io_bus.pio_read, msx_io_bus.sm_io_read, pio_build_token(in_window, data));
            }
        }

        if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
            uint8_t data = 0xFFu;
            bool in_window = false;

            if (addr == 0xFFFFu)
            {
                in_window = true;
                data = (uint8_t)~subslot_reg;
            }
            else
            {
                uint8_t page = (addr >> 14) & 0x03u;
                uint8_t active_subslot = (subslot_reg >> (page * 2)) & 0x03u;

                if (active_subslot == 0)
                {
                    if (addr >= 0x4000u && addr <= 0x7FFFu)
                    {
                        in_window = true;
                        data = wifi_handle_mem_read(wifi_rom_base, WIFI_BIOS_ROM_SIZE, addr);
                    }
                }
                else if (active_subslot == 1)
                {
                    if (addr >= 0x4000u && addr <= 0x7FFFu)
                    {
                        in_window = true;
                        uint8_t ide_data;
                        if (sunrise_ide_handle_read(&ide, addr, &ide_data))
                        {
                            data = ide_data;
                        }
                        else
                        {
                            uint8_t seg = ide.segment;
                            uint32_t rel = ((uint32_t)seg << 14) + (addr & 0x3FFFu);
                            if (available_length == 0u || rel < available_length)
                                data = read_rom_byte(rom_base, rel);
                        }
                    }
                }
                else if (mapper_enable && active_subslot == 2)
                {
                    in_window = true;
                    uint8_t mapper_page = mapper_page_from_reg(mapper_reg[page]);
                    uint32_t mapper_offset = ((uint32_t)mapper_page << 14) | (addr & 0x3FFFu);
                    data = mapper_read_byte(mapper_offset);
                }
            }

            pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
        }
    }
}

static void __no_inline_not_in_flash_func(loadrom_sunrise_wifi)(uint32_t offset, bool cache_enable)
{
    loadrom_sunrise_wifi_common(offset, cache_enable, sunrise_usb_task, sunrise_usb_set_ide_ctx, false);
}

static void __no_inline_not_in_flash_func(loadrom_sunrise_wifi_sd)(uint32_t offset, bool cache_enable)
{
    loadrom_sunrise_wifi_common(offset, cache_enable, sunrise_sd_task, sunrise_sd_set_ide_ctx, false);
}

static void __no_inline_not_in_flash_func(loadrom_sunrise_mapper_wifi)(uint32_t offset, bool cache_enable)
{
    loadrom_sunrise_wifi_common(offset, cache_enable, sunrise_usb_task, sunrise_usb_set_ide_ctx, true);
}

static void __no_inline_not_in_flash_func(loadrom_sunrise_mapper_wifi_sd)(uint32_t offset, bool cache_enable)
{
    loadrom_sunrise_wifi_common(offset, cache_enable, sunrise_sd_task, sunrise_sd_set_ide_ctx, true);
}

// -----------------------------------------------------------------------
// Bank cache infrastructure for mapper-aware LRU caching
// Used by ASCII16-X to handle ROMs larger than the 128KB rom_sram cache.
// -----------------------------------------------------------------------
#define BANK_CACHE_MAX_SLOTS 16
#define BANK_CACHE_MAX_PINS   6
#define BANK_EMPTY           0xFFFFu

typedef struct {
    uint16_t      slot_bank[BANK_CACHE_MAX_SLOTS];
    uint8_t       slot_lru[BANK_CACHE_MAX_SLOTS];
    int8_t        page_slot[BANK_CACHE_MAX_PINS];
    const uint8_t *flash_base;
    uint32_t      rom_length;
    uint8_t       num_slots;
    uint8_t       num_pins;
    uint16_t      slot_size;
    uint8_t       slot_shift;
} bank_cache_t;

static inline void __not_in_flash_func(bcache_init)(
    bank_cache_t *c, uint16_t slot_size, uint8_t num_pins,
    const uint8_t *flash_base, uint32_t rom_length)
{
    uint8_t num_slots = (uint8_t)(CACHE_SIZE / slot_size);
    if (num_slots > BANK_CACHE_MAX_SLOTS) num_slots = BANK_CACHE_MAX_SLOTS;
    c->num_slots  = num_slots;
    c->num_pins   = num_pins;
    c->slot_size  = slot_size;
    c->slot_shift = (slot_size == 8192u) ? 13 : 14;
    c->flash_base = flash_base;
    c->rom_length = rom_length;
    for (int i = 0; i < BANK_CACHE_MAX_SLOTS; i++)
    {
        c->slot_bank[i] = BANK_EMPTY;
        c->slot_lru[i]  = (uint8_t)i;
    }
    for (int i = 0; i < BANK_CACHE_MAX_PINS; i++)
        c->page_slot[i] = -1;
}

static inline void __not_in_flash_func(bcache_touch)(bank_cache_t *c, int8_t slot)
{
    uint8_t prev = c->slot_lru[slot];
    if (prev == 0) return;
    uint8_t n = c->num_slots;
    for (uint8_t i = 0; i < n; i++)
        if (c->slot_lru[i] < prev) c->slot_lru[i]++;
    c->slot_lru[slot] = 0;
}

static inline int8_t __not_in_flash_func(bcache_find)(bank_cache_t *c, uint16_t bank)
{
    uint8_t n = c->num_slots;
    for (uint8_t i = 0; i < n; i++)
        if (c->slot_bank[i] == bank) return (int8_t)i;
    return -1;
}

static inline int8_t __not_in_flash_func(bcache_evict)(bank_cache_t *c)
{
    int8_t best = -1;
    uint8_t best_lru = 0;
    uint8_t n = c->num_slots;
    uint8_t np = c->num_pins;
    for (uint8_t i = 0; i < n; i++)
    {
        bool pinned = false;
        for (uint8_t p = 0; p < np; p++)
            if ((int8_t)i == c->page_slot[p]) { pinned = true; break; }
        if (pinned) continue;
        if (best < 0 || c->slot_lru[i] >= best_lru)
        {
            best = (int8_t)i;
            best_lru = c->slot_lru[i];
        }
    }
    return best;
}

static inline int8_t __not_in_flash_func(bcache_ensure)(bank_cache_t *c, uint16_t bank)
{
    if (c->rom_length > 0u)
    {
        uint16_t total_banks = (uint16_t)(c->rom_length >> c->slot_shift);
        if (total_banks > 0u)
            bank = bank % total_banks;
    }

    int8_t slot = bcache_find(c, bank);
    if (slot >= 0) { bcache_touch(c, slot); return slot; }

    slot = bcache_evict(c);
    uint32_t src_off = (uint32_t)(bank & 0x0FFFu) << c->slot_shift;
    uint8_t *dst = &rom_sram[(uint32_t)slot * c->slot_size];
    uint16_t ss = c->slot_size;

    if (c->rom_length == 0u || src_off < c->rom_length)
    {
        uint32_t n = ss;
        if (c->rom_length > 0u && src_off + n > c->rom_length)
            n = c->rom_length - src_off;
        memcpy(dst, c->flash_base + src_off, n);
        if (n < ss) memset(dst + n, 0xFFu, ss - n);
    }
    else
    {
        memset(dst, 0xFFu, ss);
    }

    c->slot_bank[slot] = bank;
    bcache_touch(c, slot);
    return slot;
}

static inline void __not_in_flash_func(bcache_prefill)(bank_cache_t *c)
{
    uint16_t max_banks = c->num_slots;
    if (c->rom_length > 0u)
    {
        uint16_t rom_banks = (uint16_t)((c->rom_length + c->slot_size - 1u) / c->slot_size);
        if (rom_banks < max_banks) max_banks = rom_banks;
    }
    for (uint16_t b = 0; b < max_banks; b++)
        bcache_ensure(c, b);
}

// -----------------------------------------------------------------------
// ASCII16-X mapper (Padial) - 16KB banks at 0x4000-0x7FFF / 0x8000-0xBFFF
// with extended addressing (12-bit bank number) and AMD-compatible flash
// command emulation (program/erase) on the cached SRAM banks.
// Bank-switch writes use Konami-like layout but the high nibble of the
// register address is OR'd into the bank number to allow >256 banks.
// -----------------------------------------------------------------------
typedef enum {
    FLASH_IDLE = 0,
    FLASH_UNLOCK1,
    FLASH_UNLOCK2,
    FLASH_BYTE_PGM,
    FLASH_ERASE_SETUP,
    FLASH_ERASE_UNLOCK1,
    FLASH_ERASE_UNLOCK2,
} flash_cmd_state_t;

typedef struct {
    uint16_t          bank_regs[2];
    bank_cache_t      cache;
    flash_cmd_state_t flash_state;
} ascii16x_state_t;

static inline void __not_in_flash_func(flash_process_write)(
    ascii16x_state_t *st, uint16_t addr, uint8_t data)
{
    uint16_t cmd_addr = addr & 0x0FFFu;

    switch (st->flash_state)
    {
        case FLASH_IDLE:
            if (cmd_addr == 0x0AAAu && data == 0xAAu)
                st->flash_state = FLASH_UNLOCK1;
            else if (data == 0xF0u)
                st->flash_state = FLASH_IDLE;
            break;
        case FLASH_UNLOCK1:
            if (cmd_addr == 0x0555u && data == 0x55u)
                st->flash_state = FLASH_UNLOCK2;
            else
                st->flash_state = FLASH_IDLE;
            break;
        case FLASH_UNLOCK2:
            if (cmd_addr == 0x0AAAu)
            {
                switch (data)
                {
                    case 0xA0u: st->flash_state = FLASH_BYTE_PGM;    break;
                    case 0x80u: st->flash_state = FLASH_ERASE_SETUP;  break;
                    default:    st->flash_state = FLASH_IDLE;         break;
                }
            }
            else
                st->flash_state = FLASH_IDLE;
            break;
        case FLASH_BYTE_PGM:
        {
            uint8_t page_idx = ((addr >> 14) & 0x01u) ? 0u : 1u;
            int8_t slot = st->cache.page_slot[page_idx];
            if (slot >= 0)
            {
                uint32_t off = (uint32_t)slot * st->cache.slot_size + (addr & 0x3FFFu);
                rom_sram[off] &= data;
            }
            st->flash_state = FLASH_IDLE;
            break;
        }
        case FLASH_ERASE_SETUP:
            if (cmd_addr == 0x0AAAu && data == 0xAAu)
                st->flash_state = FLASH_ERASE_UNLOCK1;
            else
                st->flash_state = FLASH_IDLE;
            break;
        case FLASH_ERASE_UNLOCK1:
            if (cmd_addr == 0x0555u && data == 0x55u)
                st->flash_state = FLASH_ERASE_UNLOCK2;
            else
                st->flash_state = FLASH_IDLE;
            break;
        case FLASH_ERASE_UNLOCK2:
            if (data == 0x30u)
            {
                uint8_t page_idx = ((addr >> 14) & 0x01u) ? 0u : 1u;
                int8_t slot = st->cache.page_slot[page_idx];
                if (slot >= 0)
                    memset(&rom_sram[(uint32_t)slot * st->cache.slot_size], 0xFFu, st->cache.slot_size);
            }
            st->flash_state = FLASH_IDLE;
            break;
    }
}

static inline void __not_in_flash_func(handle_ascii16x_write_cached)(uint16_t addr, uint8_t data, void *ctx)
{
    ascii16x_state_t *st = (ascii16x_state_t *)ctx;
    flash_process_write(st, addr, data);

    uint8_t high_nibble = (uint8_t)((addr >> 8) & 0x0Fu);
    uint16_t bank = ((uint16_t)high_nibble << 8) | data;
    uint8_t page;

    switch (addr & 0xF000u)
    {
        case 0x2000u: case 0x6000u: case 0xA000u: case 0xE000u: page = 0; break;
        case 0x3000u: case 0x7000u: case 0xB000u: case 0xF000u: page = 1; break;
        default: return;
    }

    st->bank_regs[page] = bank;
    st->cache.page_slot[page] = bcache_ensure(&st->cache, bank);
}

void __no_inline_not_in_flash_func(loadrom_ascii16x)(uint32_t offset, bool cache_enable)
{
    (void)cache_enable;

    if (!psram_prepare_rom_cache())
    {
        return;
    }

    const uint8_t *rom_base;
    uint32_t available_length;
    // Don't preload via prepare_rom_source — bcache manages SRAM directly.
    prepare_rom_source(offset, false, 0u, &rom_base, &available_length);

    ascii16x_state_t state;
    memset(&state, 0, sizeof(state));

    bcache_init(&state.cache, 16384u, 2, rom_data + offset, active_rom_size);
    bcache_prefill(&state.cache);

    // Initial bank 0 visible at both windows.
    state.cache.page_slot[0] = bcache_find(&state.cache, 0);
    state.cache.page_slot[1] = state.cache.page_slot[0];

    // bcache reads from rom_sram via slot index, bypassing read_rom_byte.
    rom_cached_size = 0;

    msx_pio_bus_init();

    while (true)
    {
        pio_drain_writes(handle_ascii16x_write_cached, &state);

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        pio_drain_writes(handle_ascii16x_write_cached, &state);

        bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint8_t page_idx = ((addr >> 14) & 0x01u) ? 0u : 1u;
            int8_t slot = state.cache.page_slot[page_idx];
            if (slot >= 0)
                data = rom_sram[(uint32_t)slot * state.cache.slot_size + (addr & 0x3FFFu)];
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// -----------------------------------------------------------------------
// loadrom_planar64 - 64KB Planar ROM (no mapper)
// Linear 64KB ROM mapped at 0x0000-0xFFFF (typical MSX2 boot games like
// Game Master 2). AB header is at 0x4000.
// -----------------------------------------------------------------------
void __no_inline_not_in_flash_func(loadrom_planar64)(uint32_t offset, bool cache_enable)
{
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 65536u, &rom_base, &available_length);

    msx_pio_bus_init();

    while (true)
    {
        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        bool in_window = true; // 64KB linear: drive all reads
        uint8_t data = 0xFFu;
        if (available_length == 0u || addr < available_length)
            data = read_rom_byte(rom_base, addr);

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// -----------------------------------------------------------------------
// Manbow2 mapper - Konami SCC banking + AMD AM29F040B flash emulation
// 8KB banks at 0x4000-0x5FFF / 0x6000-0x7FFF / 0x8000-0x9FFF / 0xA000-0xBFFF.
// Bank-switch writes land at 0x?000-0x?7FF for each page (Konami SCC layout).
// The last 64KB of the ROM (flash offset 0x70000-0x7FFFF) is treated as a
// writable sector emulating the AMD AM29F040B flash chip used by Manbow2 to
// persist save data via the standard JEDEC unlock+program/erase sequence.
// -----------------------------------------------------------------------
enum manbow2_flash_state {
    MBW2_READ = 0,
    MBW2_CMD1,
    MBW2_CMD2,
    MBW2_AUTOSELECT,
    MBW2_PROGRAM,
    MBW2_ERASE_CMD1,
    MBW2_ERASE_CMD2,
    MBW2_ERASE_CMD3
};

typedef struct {
    uint8_t bank_regs[4];
    enum manbow2_flash_state state;
    uint8_t *writable_sram;
    uint32_t writable_offset;
    uint32_t writable_size;
} manbow2_ctx_t;

static inline void __not_in_flash_func(handle_manbow2_write)(uint16_t addr, uint8_t data, void *ctx)
{
    manbow2_ctx_t *mb = (manbow2_ctx_t *)ctx;

    if (addr < 0x4000u || addr > 0xBFFFu)
        return;

    uint8_t page = (uint8_t)((addr - 0x4000u) >> 13);
    uint32_t flash_addr = (uint32_t)mb->bank_regs[page] * 0x2000u + (addr & 0x1FFFu);

    // Konami SCC-style bank latches at 0x?000-0x?7FF of each 8KB page.
    if ((addr & 0x1800u) == 0x1000u)
        mb->bank_regs[page] = data & 0x3Fu;

    // AMD reset
    if (data == 0xF0u)
    {
        mb->state = MBW2_READ;
        return;
    }

    switch (mb->state)
    {
        case MBW2_READ:
            if ((flash_addr & 0x7FFu) == 0x555u && data == 0xAAu)
                mb->state = MBW2_CMD1;
            break;
        case MBW2_CMD1:
            if ((flash_addr & 0x7FFu) == 0x2AAu && data == 0x55u)
                mb->state = MBW2_CMD2;
            else
                mb->state = MBW2_READ;
            break;
        case MBW2_CMD2:
            if      (data == 0xA0u) mb->state = MBW2_PROGRAM;
            else if (data == 0x90u) mb->state = MBW2_AUTOSELECT;
            else if (data == 0x80u) mb->state = MBW2_ERASE_CMD1;
            else                     mb->state = MBW2_READ;
            break;
        case MBW2_PROGRAM:
            if (flash_addr >= mb->writable_offset &&
                flash_addr <  mb->writable_offset + mb->writable_size)
            {
                uint32_t sram_off = flash_addr - mb->writable_offset;
                mb->writable_sram[sram_off] &= data;
            }
            mb->state = MBW2_READ;
            break;
        case MBW2_ERASE_CMD1:
            if ((flash_addr & 0x7FFu) == 0x555u && data == 0xAAu)
                mb->state = MBW2_ERASE_CMD2;
            else
                mb->state = MBW2_READ;
            break;
        case MBW2_ERASE_CMD2:
            if ((flash_addr & 0x7FFu) == 0x2AAu && data == 0x55u)
                mb->state = MBW2_ERASE_CMD3;
            else
                mb->state = MBW2_READ;
            break;
        case MBW2_ERASE_CMD3:
            if (data == 0x30u)
            {
                if (flash_addr >= mb->writable_offset &&
                    flash_addr <  mb->writable_offset + mb->writable_size)
                    memset(mb->writable_sram, 0xFF, mb->writable_size);
            }
            mb->state = MBW2_READ;
            break;
        case MBW2_AUTOSELECT:
            break;
    }
}

// Common Manbow2 setup: caches the first (CACHE_SIZE - writable_size) bytes of
// the ROM into rom_sram (mirrors multirom's strategy) and reserves the tail of
// rom_sram for the emulated 64KB writable AMD flash sector. Returns rom_base /
// writable_sram / available_length to the caller.
static inline void __not_in_flash_func(loadrom_manbow2_setup)(
    uint32_t offset,
    const uint8_t **rom_base_out,
    uint32_t *available_length_out,
    uint8_t **writable_sram_out,
    uint32_t writable_offset,
    uint32_t writable_size)
{
    uint32_t reduced_cache = CACHE_SIZE - writable_size;

    // Limit prepare_rom_source's caching to `reduced_cache` so the tail of
    // rom_sram is left untouched for the writable flash sector.
    rom_cache_capacity = reduced_cache;
    prepare_rom_source(offset, true, 0u, rom_base_out, available_length_out);
    rom_cache_capacity = CACHE_SIZE; // restore default for next ROM type

    uint8_t *writable_sram = &rom_sram[reduced_cache];

    // Seed the writable AMD flash sector mirror from the ROM payload.
    // rom_data points to flash for flash-resident ROMs and to PSRAM for
    // SD-resident ROMs, so a single source path works for both.
    {
        const uint8_t *src = rom_data + offset;

        if (*available_length_out >= writable_offset + writable_size)
        {
            memcpy(writable_sram, src + writable_offset, writable_size);
        }
        else if (*available_length_out > writable_offset)
        {
            uint32_t partial = *available_length_out - writable_offset;
            memcpy(writable_sram, src + writable_offset, partial);
            memset(writable_sram + partial, 0xFF, writable_size - partial);
        }
        else
        {
            memset(writable_sram, 0xFF, writable_size);
        }
    }

    *writable_sram_out = writable_sram;
}

static inline void __not_in_flash_func(handle_ascii16x_write_simple)(uint16_t addr, uint8_t data, uint16_t *bank_regs)
{
    if (addr >= 0x6000u && addr <= 0x67FFu)
        bank_regs[0] = data;
    else if (addr >= 0x7000u && addr <= 0x77FFu)
        bank_regs[1] = data;
}

static void __not_in_flash_func(fmpac_wait_for_expanded_bootstrap)(void)
{
    static const uint8_t bootstrap_rom[] = {
        0x41, 0x42, 0x0A, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xF3, 0xDB, 0xF4, 0xF6, 0x80, 0xD3, 0xF4, 0xC7
    };

    msx_pio_bus_init();

    bool restart_detected = false;
    bool init_called = false;

    while (!restart_detected)
    {
        uint16_t waddr;
        uint8_t wdata;
        while (pio_try_get_write(&waddr, &wdata))
        {
        }

        if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
            if (init_called && addr == 0x0000u)
            {
                pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(false, 0xFFu));
                restart_detected = true;
            }
            else
            {
                if (addr >= 0x400Au && addr <= 0x4011u) init_called = true;
                bool in_window = (addr >= 0x4000u && addr <= 0x7FFFu);
                uint8_t data = 0xFFu;
                if (in_window)
                {
                    uint32_t rel = addr - 0x4000u;
                    if (rel < sizeof(bootstrap_rom)) data = bootstrap_rom[rel];
                }
                pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
            }
        }
        else if (init_called && !gpio_get(PIN_RD) && ((gpio_get_all() & 0xFFFFu) == 0x0000u))
        {
            restart_detected = true;
        }
    }

    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_read, false);
    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_write, false);
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 0);
}

void __no_inline_not_in_flash_func(loadrom_fmpac)(uint32_t offset, bool cache_enable, uint8_t mapper)
{
    fmpac_wait_for_expanded_bootstrap();

    uint8_t bank8_regs[4] = {0, 1, 2, 3};
    uint8_t ascii16_regs[2] = {0, 1};
    uint16_t neo8_regs[6] = {0, 1, 2, 3, 4, 5};
    uint16_t neo16_regs[3] = {0, 1, 2};
    uint16_t ascii16x_regs[2] = {0, 0};

    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);
    const uint8_t *fmpac_bios_base = flash_rom + FMPAC_BIOS_FLASH_OFFSET;

    uint8_t subslot_reg = 0x00u;
    static fmpac_state_t fmpac;
    memset(&fmpac, 0, sizeof(fmpac));
    fmpac.control = 0x10u;

    bank8_ctx_t bank8_ctx = { .bank_regs = bank8_regs };
    bank8_ctx_t ascii16_ctx = { .bank_regs = ascii16_regs };
    bank16_ctx_t neo8_ctx = { .bank_regs = neo8_regs };
    bank16_ctx_t neo16_ctx = { .bank_regs = neo16_regs };

    msx_pio_bus_init();

    while (true)
    {
        uint16_t waddr;
        uint8_t wdata;
        while (pio_try_get_write(&waddr, &wdata))
        {
            if (waddr == 0xFFFFu)
            {
                subslot_reg = wdata;
                continue;
            }

            uint8_t page = (waddr >> 14) & 0x03u;
            uint8_t active_subslot = (subslot_reg >> (page * 2)) & 0x03u;
            if (active_subslot == 0)
            {
                switch (mapper)
                {
                    case 5:  handle_ascii8_write(waddr, wdata, &bank8_ctx); break;
                    case 6:  handle_ascii16_write(waddr, wdata, &ascii16_ctx); break;
                    case 7:  handle_konami_write(waddr, wdata, &bank8_ctx); break;
                    case 8:  handle_neo8_write(waddr, wdata, &neo8_ctx); break;
                    case 9:  handle_neo16_write(waddr, wdata, &neo16_ctx); break;
                    case 12: handle_ascii16x_write_simple(waddr, wdata, ascii16x_regs); break;
                }
            }
            else if (active_subslot == 3)
            {
                fmpac_handle_write(&fmpac, waddr, wdata);
            }
        }

        if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
            uint8_t data = 0xFFu;
            bool in_window = true;

            if (addr == 0xFFFFu)
            {
                data = (uint8_t)~subslot_reg;
            }
            else
            {
                uint8_t page = (addr >> 14) & 0x03u;
                uint8_t active_subslot = (subslot_reg >> (page * 2)) & 0x03u;
                if (active_subslot == 0)
                {
                    uint32_t rel = 0;
                    bool mapped = false;

                    switch (mapper)
                    {
                        case 1:
                        case 2:
                            mapped = (addr >= 0x4000u && addr <= 0xBFFFu);
                            rel = addr - 0x4000u;
                            break;
                        case 5:
                        case 7:
                            mapped = (addr >= 0x4000u && addr <= 0xBFFFu);
                            if (mapped)
                                rel = ((uint32_t)bank8_regs[(addr - 0x4000u) >> 13] << 13) | (addr & 0x1FFFu);
                            break;
                        case 4:
                            mapped = (addr <= 0xBFFFu);
                            rel = addr;
                            break;
                        case 6:
                            mapped = (addr >= 0x4000u && addr <= 0xBFFFu);
                            if (mapped)
                                rel = ((uint32_t)ascii16_regs[(addr >> 15) & 1u] << 14) | (addr & 0x3FFFu);
                            break;
                        case 8:
                            mapped = (addr <= 0xBFFFu);
                            if (mapped && (addr >> 13) < 6u)
                                rel = ((uint32_t)(neo8_regs[addr >> 13] & 0x0FFFu) << 13) | (addr & 0x1FFFu);
                            break;
                        case 9:
                            mapped = (addr <= 0xBFFFu);
                            if (mapped && (addr >> 14) < 3u)
                                rel = ((uint32_t)(neo16_regs[addr >> 14] & 0x0FFFu) << 14) | (addr & 0x3FFFu);
                            break;
                        case 12:
                            mapped = true;
                            rel = ((uint32_t)ascii16x_regs[((addr >> 14) & 1u) ? 0u : 1u] << 14) | (addr & 0x3FFFu);
                            break;
                        case 13:
                            mapped = true;
                            rel = addr;
                            break;
                    }

                    if (mapped && (available_length == 0u || rel < available_length))
                        data = read_rom_byte(rom_base, rel);
                }
                else if (active_subslot == 3 && addr >= 0x4000u && addr <= 0x7FFFu)
                {
                    data = fmpac_handle_read(&fmpac, fmpac_bios_base, addr);
                }
            }

            pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
        }

        tight_loop_contents();
    }
}

// loadrom_manbow2 - Manbow2 (Konami SCC-banked + AMD flash) without SCC audio
void __no_inline_not_in_flash_func(loadrom_manbow2)(uint32_t offset, bool cache_enable)
{
    (void)cache_enable;

    static const uint32_t WRITABLE_SECTOR_SIZE   = 0x10000u;  // 64KB
    static const uint32_t WRITABLE_SECTOR_OFFSET = 0x70000u;  // 448KB

    const uint8_t *rom_base;
    uint32_t available_length;
    uint8_t *writable_sram;
    loadrom_manbow2_setup(offset, &rom_base, &available_length, &writable_sram,
                           WRITABLE_SECTOR_OFFSET, WRITABLE_SECTOR_SIZE);

    manbow2_ctx_t mb = {
        .bank_regs       = {0, 1, 2, 3},
        .state           = MBW2_READ,
        .writable_sram   = writable_sram,
        .writable_offset = WRITABLE_SECTOR_OFFSET,
        .writable_size   = WRITABLE_SECTOR_SIZE,
    };

    msx_pio_bus_init();

    while (true)
    {
        // Busy-poll write FIFO + read FIFO together. Manbow2's AMD flash
        // command sequences burst multiple writes (0x555=0xAA, 0x2AA=0x55,
        // 0xA0/0x80, then data) without intervening reads — using a blocking
        // read here would let the write FIFO overflow and stall the bus.
        uint16_t addr;
        while (true)
        {
            uint16_t waddr; uint8_t wdata;
            while (pio_try_get_write(&waddr, &wdata))
                handle_manbow2_write(waddr, wdata, &mb);
            if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
            {
                addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
                break;
            }
        }

        { uint16_t waddr; uint8_t wdata;
          while (pio_try_get_write(&waddr, &wdata))
              handle_manbow2_write(waddr, wdata, &mb); }

        bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            uint8_t page = (uint8_t)((addr - 0x4000u) >> 13);
            uint32_t rel = (uint32_t)mb.bank_regs[page] * 0x2000u + (addr & 0x1FFFu);

            if (mb.state == MBW2_AUTOSELECT)
            {
                // AM29F040B device IDs (manufacturer 0x01 AMD, device 0xA4)
                uint32_t id_addr = rel & 0x03u;
                if      (id_addr == 0x00u) data = 0x01u;
                else if (id_addr == 0x01u) data = 0xA4u;
                else if (id_addr == 0x02u)
                    data = (rel >= mb.writable_offset && rel < mb.writable_offset + mb.writable_size) ? 0x00u : 0x01u;
                else                       data = 0x00u;
            }
            else
            {
                if (rel >= mb.writable_offset && rel < mb.writable_offset + mb.writable_size)
                    data = writable_sram[rel - mb.writable_offset];
                else if (available_length == 0u || rel < available_length)
                    data = read_rom_byte(rom_base, rel); // SRAM cache up to rom_cached_size, then flash
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// loadrom_manbow2_scc - Manbow2 mapper with SCC/SCC+ audio emulation
void __no_inline_not_in_flash_func(loadrom_manbow2_scc)(uint32_t offset, bool cache_enable, uint32_t scc_type)
{
    (void)cache_enable;

    static const uint32_t WRITABLE_SECTOR_SIZE   = 0x10000u;
    static const uint32_t WRITABLE_SECTOR_OFFSET = 0x70000u;

    const uint8_t *rom_base;
    uint32_t available_length;
    uint8_t *writable_sram;
    loadrom_manbow2_setup(offset, &rom_base, &available_length, &writable_sram,
                           WRITABLE_SECTOR_OFFSET, WRITABLE_SECTOR_SIZE);

    manbow2_ctx_t mb = {
        .bank_regs       = {0, 1, 2, 3},
        .state           = MBW2_READ,
        .writable_sram   = writable_sram,
        .writable_offset = WRITABLE_SECTOR_OFFSET,
        .writable_size   = WRITABLE_SECTOR_SIZE,
    };

    // Hold WAIT to freeze Z80 during audio subsystem init.
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 0);

    memset(&scc_instance, 0, sizeof(SCC));
    scc_instance.clk  = SCC_CLOCK;
    scc_instance.rate = SCC_SAMPLE_RATE;
    SCC_set_quality(&scc_instance, 1);
    scc_instance.type = scc_type;
    SCC_reset(&scc_instance);

    i2s_audio_init_scc();
    multicore_launch_core1(core1_scc_audio);

    msx_pio_bus_init();

    while (true)
    {
        // Busy-poll write FIFO + read FIFO together (see loadrom_manbow2 for
        // rationale — AMD flash command sequences burst writes without reads).
        uint16_t addr;
        while (true)
        {
            uint16_t waddr; uint8_t wdata;
            while (pio_try_get_write(&waddr, &wdata))
            {
                handle_manbow2_write(waddr, wdata, &mb);
                SCC_write(&scc_instance, waddr, wdata);
            }
            if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
            {
                addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
                break;
            }
        }

        { uint16_t waddr; uint8_t wdata;
          while (pio_try_get_write(&waddr, &wdata))
          {
              handle_manbow2_write(waddr, wdata, &mb);
              SCC_write(&scc_instance, waddr, wdata);
          } }

        bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
        uint8_t data = 0xFFu;

        if (in_window)
        {
            // SCC register read window
            bool is_scc_read = false;
            if (scc_instance.active)
            {
                uint32_t scc_reg_start = scc_instance.base_adr + 0x800u;
                if (addr >= scc_reg_start && addr <= (scc_reg_start + 0xFFu))
                    is_scc_read = true;
            }
            if (scc_type == SCC_ENHANCED && (addr & 0xFFFEu) == 0xBFFEu)
                is_scc_read = true;

            if (is_scc_read)
            {
                data = (uint8_t)SCC_read(&scc_instance, addr);
            }
            else
            {
                uint8_t page = (uint8_t)((addr - 0x4000u) >> 13);
                uint32_t rel = (uint32_t)mb.bank_regs[page] * 0x2000u + (addr & 0x1FFFu);

                if (mb.state == MBW2_AUTOSELECT)
                {
                    uint32_t id_addr = rel & 0x03u;
                    if      (id_addr == 0x00u) data = 0x01u;
                    else if (id_addr == 0x01u) data = 0xA4u;
                    else if (id_addr == 0x02u)
                        data = (rel >= mb.writable_offset && rel < mb.writable_offset + mb.writable_size) ? 0x00u : 0x01u;
                    else                       data = 0x00u;
                }
                else
                {
                    if (rel >= mb.writable_offset && rel < mb.writable_offset + mb.writable_size)
                        data = writable_sram[rel - mb.writable_offset];
                    else if (available_length == 0u || rel < available_length)
                        data = read_rom_byte(rom_base, rel);
                }
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

void __no_inline_not_in_flash_func(loadrom_wifi_config_setup)(uint32_t offset)
{
    const uint8_t *wifi_rom_base = flash_rom + offset;
    bool return_requested = false;

    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 0);

    wifi_uart_init_once();
    wifi_f2_state = WIFI_F2_FORCE_SETUP;
    msx_pio_bus_init();

    gpio_put(PIN_WAIT, 1);

    while (true)
    {
        wifi_service_rx();

        uint16_t waddr;
        uint8_t wdata;
        while (pio_try_get_write(&waddr, &wdata))
        {
            (void)wifi_handle_mem_write(waddr, wdata);
        }

        if (wifi_f2_state == WIFI_CONFIG_RETURN_MENU)
            return_requested = true;

        if (pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            if (return_requested &&
                !gpio_get(PIN_RD) &&
                ((gpio_get_all() & 0xFFFFu) == 0x0000u))
            {
                return;
            }
            tight_loop_contents();
            continue;
        }

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        while (pio_try_get_write(&waddr, &wdata))
        {
            (void)wifi_handle_mem_write(waddr, wdata);
        }

        if (wifi_f2_state == WIFI_CONFIG_RETURN_MENU)
            return_requested = true;

        if (return_requested && addr == 0x0000u)
        {
            pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(false, 0xFFu));
            return;
        }

        bool in_window = (addr >= 0x4000u && addr <= 0x7FFFu) ||
                         (addr >= WIFI_MEM_F2_ADDR && addr <= WIFI_MEM_DATA_ADDR);
        uint8_t data = wifi_handle_mem_read(wifi_rom_base, WIFI_CONFIG_ROM_SIZE, addr);

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}


// Main function running on core 0
int __no_inline_not_in_flash_func(main)()
{
    qmi_hw->m[0].timing = 0x40000202; // Set the QMI timing for the MSX bus
    set_sys_clock_khz(210000, true);     // Set system clock to 210Mhz

    stdio_init_all();     // Initialize stdio
    setup_gpio();     // Initialize GPIO

    while (true) {
    int rom_index = loadrom_msx_menu(0x0000); //load the first 32KB ROM into the MSX (The MSX PICOVERSE MENU)

    if ((uint16_t)rom_index == ROM_SELECT_WIFI_CONFIG) {
        active_rom_size = WIFI_CONFIG_ROM_SIZE;
        rom_data = flash_rom;
        rom_data_in_ram = false;
        loadrom_wifi_config_setup(WIFI_CONFIG_FLASH_OFFSET);
        continue;
    }

    ROMRecord const *selected = &records[rom_index];
    active_rom_size = selected->Size;

    uint32_t rom_offset = selected->Offset;
    rom_data = flash_rom;
    rom_data_in_ram = false;

    bool is_sd_rom = (selected->Mapper & SOURCE_SD_FLAG) != 0;
    if (is_sd_rom) {
        if (!load_rom_from_sd((uint16_t)rom_index, (uint32_t)selected->Size)) {
            printf("Debug: Failed to load ROM from SD card\n");
            while (true) { tight_loop_contents(); }
        }
        rom_data = sd_rom_region.ptr;
        rom_data_in_ram = true;
        rom_offset = 0;
    }

    // Cache the leading window into rom_sram for both flash- and PSRAM-resident
    // ROMs (SD ROMs are staged to PSRAM, so caching them into SRAM mirrors the
    // flash path and keeps the mapper hot loop fast).
    bool cache_enable = true;
    uint8_t mapper = mapper_code_from_record_byte(selected->Mapper);
    audio_mode_t audio_mode = resolve_audio_mode(mapper, ctrl_audio_selection);
    bool scc_audio = (audio_mode == AUDIO_MODE_SCC || audio_mode == AUDIO_MODE_SCC_PLUS);
    if (audio_mode == AUDIO_MODE_DUAL_PSG) {
        start_dual_psg_audio();
    } else if (audio_mode == AUDIO_MODE_MSX_MUSIC) {
        start_msx_music_audio();
    }
    bool wifi_support = (ctrl_wifi_support != 0u) && is_system_mapper(mapper);

    if (audio_mode == AUDIO_MODE_MSX_MUSIC) {
        loadrom_fmpac(rom_offset, cache_enable, mapper);
        continue;
    }

    // Load the selected ROM into the MSX according to the mapper
    switch (mapper) {
       
        case 1:
        case 2:
            loadrom_plain32(rom_offset, cache_enable);
            break;
        case 3:
            if (scc_audio)
                loadrom_konamiscc_scc(rom_offset, cache_enable, audio_mode == AUDIO_MODE_SCC_PLUS ? SCC_ENHANCED : SCC_STANDARD);
            else
                loadrom_konamiscc(rom_offset, cache_enable);
            break;
        case 4:
            loadrom_linear48(rom_offset, cache_enable);
            break;
        case 5:
            loadrom_ascii8(rom_offset, cache_enable); 
            break;
        case 6:
            loadrom_ascii16(rom_offset, cache_enable); 
            break;
        case 7:
            loadrom_konami(rom_offset, cache_enable); 
            break;
        case 8:
            loadrom_neo8(rom_offset); 
            break;
        case 9:
            loadrom_neo16(rom_offset); 
            break;
        case MAPPER_SUNRISE_USB:
            if (wifi_support)
                loadrom_sunrise_wifi(rom_offset, cache_enable);
            else
                loadrom_sunrise(rom_offset, cache_enable);
            break;
        case MAPPER_SUNRISE_MAPPER_USB:
            if (wifi_support)
                loadrom_sunrise_mapper_wifi(rom_offset, cache_enable);
            else
                loadrom_sunrise_mapper(rom_offset, cache_enable);
            break;
        case MAPPER_SUNRISE_SD:
            if (wifi_support)
                loadrom_sunrise_wifi_sd(rom_offset, cache_enable);
            else
                loadrom_sunrise_sd(rom_offset, cache_enable);
            break;
        case MAPPER_SUNRISE_MAPPER_SD:
            if (wifi_support)
                loadrom_sunrise_mapper_wifi_sd(rom_offset, cache_enable);
            else
                loadrom_sunrise_mapper_sd(rom_offset, cache_enable);
            break;
        case 12:
            loadrom_ascii16x(rom_offset, cache_enable);
            break;
        case 13:
            loadrom_planar64(rom_offset, cache_enable);
            break;
        case 14:
            if (scc_audio)
                loadrom_manbow2_scc(rom_offset, cache_enable, audio_mode == AUDIO_MODE_SCC_PLUS ? SCC_ENHANCED : SCC_STANDARD);
            else
                loadrom_manbow2(rom_offset, cache_enable);
            break;
        default:
                printf("Debug: Unsupported ROM mapper: %d\n", mapper);
            break;
    }
    }
    
}
