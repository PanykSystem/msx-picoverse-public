// MSX PICOVERSE PROJECT
// (c) 2025 Cristiano Goncalves
// The Retro Hacker
//
// multirom.c - This is the Raspberry Pico firmware that will be used to load ROMs into the MSX
//
// This firmware is responsible for loading the multirom menu and the ROMs selected by the user into the MSX. When flashed through the
// multirom tool, it will be stored on the pico flash memory followed by the MSX MENU ROM (with the config) and all the ROMs processed by the
// multirom tool. The software in this firmware will load the first 32KB ROM that contains the menu into the MSX and it will allow the user
// to select a ROM to be loaded into the MSX. The selected ROM will be loaded into the MSX and the MSX will be reset to run the selected ROM.
//
// This work is licensed under a "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
// License". https://creativecommons.org/licenses/by-nc-sa/4.0/

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/audio_i2s.h"
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
#include "multirom.h"
#include "sunrise_ide.h"
#include "sunrise_sd.h"
#include "emu2212.h"
#include "c2_emu.h"
#include "msx_bus.pio.h"

// config area and buffer for the ROM data
#define ROM_NAME_MAX    50
#define MAX_ROM_RECORDS 128
#define ROM_RECORD_SIZE (ROM_NAME_MAX + 1 + (sizeof(uint32_t) * 2))
#define MONITOR_ADDR    (0x8000 + (ROM_RECORD_SIZE * MAX_ROM_RECORDS) + 1)

extern unsigned char __flash_binary_end;

// Optionally copy the ROM into this SRAM buffer for faster access.
// All modes use the full 256KB as ROM cache; the 1MB mapper RAM lives
// in external PSRAM and is allocated through the PSRAM memory manager.
static uint8_t rom_sram[CACHE_SIZE];

static uint32_t active_rom_size = 0;
static uint32_t rom_cached_size = 0;

// Effective SRAM cache capacity (may be reduced by mappers that reserve
// part of the SRAM pool for writable flash sectors, e.g. Manbow2).
static uint32_t rom_cache_capacity = CACHE_SIZE;

#define SCC_VOLUME_SHIFT 2
#define SCC_AUDIO_BUFFER_SAMPLES 256

static SCC scc_instance;
static struct audio_buffer_pool *audio_pool;
static bool scc_audio_ready = false;
static int scc_dma_channel = -1;

// -----------------------------------------------------------------------
// Sunrise WiFi memio backend (mirrors loadrom.pio implementation)
// -----------------------------------------------------------------------
#define WIFI_MEM_F2_ADDR      0x7F05u
#define WIFI_MEM_CMD_ADDR     0x7F06u
#define WIFI_MEM_DATA_ADDR    0x7F07u
#define WIFI_RX_FIFO_SIZE     2080u
#define WIFI_QUICK_WAIT_US    25000u
#define WIFI_UART_DEFAULT_BAUD 859372u
#define WIFI_STATUS_RX_READY  0x01u
#define WIFI_STATUS_TX_BUSY   0x02u
#define WIFI_STATUS_RX_FULL   0x04u
#define WIFI_STATUS_QUICK_RX  0x08u
#define WIFI_STATUS_UNDERRUN  0x10u
#define WIFI_STATUS_FREE_BITS 0x80u

// Hardware UART1 backs the ESP-01 link on Core2350B GPIO38/39.
// On RP2350B these are UART1 TX/RX on FUNCSEL=11 (GPIO_FUNC_UART_AUX).
// FUNCSEL=2 on these pins maps to UART1 CTS/RTS, NOT data.
#define WIFI_UART_INSTANCE uart1
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

static inline void __not_in_flash_func(wifi_push_rx_byte)(uint8_t data)
{
    // Producer side: called from UART RX IRQ on core0. The pop side
    // disables IRQs while it mutates head/tail/count, so we don't race.
    if (wifi_rx_count >= WIFI_RX_FIFO_SIZE) return;
    wifi_rx_fifo[wifi_rx_head] = data;
    wifi_rx_head = (uint16_t)((wifi_rx_head + 1u) % WIFI_RX_FIFO_SIZE);
    ++wifi_rx_count;
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

static inline bool __not_in_flash_func(wifi_hw_tx_busy)(void)
{
    return (uart_get_hw(WIFI_UART_INSTANCE)->fr & UART_UARTFR_BUSY_BITS) != 0u;
}

static inline void __not_in_flash_func(wifi_hw_rx_drain)(void)
{
    while (uart_is_readable(WIFI_UART_INSTANCE))
    {
        (void)uart_getc(WIFI_UART_INSTANCE);
    }
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
    if (!wifi_uart_ready) return;
    while (wifi_rx_count < WIFI_RX_FIFO_SIZE && uart_is_readable(WIFI_UART_INSTANCE))
    {
        wifi_push_rx_byte((uint8_t)uart_getc(WIFI_UART_INSTANCE));
    }
}

// IRQ handler: drains UART1 RX hardware FIFO into the SW FIFO. Runs on
// core0 (the core that calls wifi_uart_init_once). Must be in RAM so it
// is not blocked by XIP cache misses while the bus loop is busy.
static void __not_in_flash_func(wifi_uart_rx_irq)(void)
{
    while (uart_is_readable(WIFI_UART_INSTANCE))
    {
        uint8_t b = (uint8_t)uart_getc(WIFI_UART_INSTANCE);
        if (wifi_rx_count < WIFI_RX_FIFO_SIZE)
        {
            wifi_rx_fifo[wifi_rx_head] = b;
            wifi_rx_head = (uint16_t)((wifi_rx_head + 1u) % WIFI_RX_FIFO_SIZE);
            ++wifi_rx_count;
        }
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

    // IRQ-driven RX: at 859372 baud the 32-byte HW FIFO fills in ~370us.
    // Polling from the bus service loop is not reliable when the loop
    // stalls on pio_sm_put_blocking, so route the RX/RT interrupts to a
    // dedicated handler that drains the FIFO immediately. RT (receive
    // timeout) ensures partial bursts don't sit in the FIFO.
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

    uint8_t status = WIFI_STATUS_QUICK_RX | WIFI_STATUS_FREE_BITS;
    if (wifi_rx_count != 0u) status |= WIFI_STATUS_RX_READY;
    if (wifi_rx_count >= WIFI_RX_FIFO_SIZE) status |= WIFI_STATUS_RX_FULL;
    if (wifi_hw_tx_busy() ||
        !uart_is_writable(WIFI_UART_INSTANCE) ||
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

static inline uint8_t __not_in_flash_func(wifi_read_rom_byte)(const uint8_t *wifi_rom_base, uint16_t addr)
{
    uint32_t rel = (uint32_t)(addr - 0x4000u);
    if (rel < WIFI_ROM_SIZE) return wifi_rom_base[rel];
    return 0xFFu;
}

static inline uint8_t __not_in_flash_func(wifi_handle_mem_read)(const uint8_t *wifi_rom_base, uint16_t addr)
{
    if (addr == WIFI_MEM_F2_ADDR) return wifi_f2_state;
    if (addr == WIFI_MEM_CMD_ADDR) return wifi_data_read();
    if (addr == WIFI_MEM_DATA_ADDR) return wifi_status_read();
    return wifi_read_rom_byte(wifi_rom_base, addr);
}

const uint8_t *rom = (const uint8_t *)&__flash_binary_end;

typedef struct {
    char Name[ROM_NAME_MAX];
    unsigned char Mapper;
    unsigned long Size;
    unsigned long Offset;
} ROMRecord;

typedef struct {
    PIO pio;
    uint sm_read;
    uint sm_write;
    uint offset_read;
    uint offset_write;
} msx_pio_bus_t;

typedef struct {
    uint8_t rom_index;
    bool rom_selected;
} menu_select_ctx_t;

typedef struct {
    uint8_t *bank_regs;
} bank8_ctx_t;

typedef struct {
    uint16_t *bank_regs;
} bank16_ctx_t;

static ROMRecord records[MAX_ROM_RECORDS];
static msx_pio_bus_t msx_bus;
static bool msx_bus_programs_loaded = false;

// I/O bus context (PIO1) for memory mapper port access
typedef struct {
    PIO pio_read;
    PIO pio_write;
    uint sm_io_read;
    uint sm_io_write;
    uint offset_io_read;
    uint offset_io_write;
} msx_pio_io_bus_t;

static msx_pio_io_bus_t msx_io_bus;
static bool msx_io_bus_programs_loaded = false;

static inline void setup_gpio()
{
    for (uint pin = PIN_A0; pin <= PIN_A15; ++pin)
    {
        gpio_init(pin);
        gpio_set_input_hysteresis_enabled(pin, true);
        gpio_set_dir(pin, GPIO_IN);
    }

    for (uint pin = PIN_D0; pin <= PIN_D7; ++pin)
    {
        gpio_init(pin);
        gpio_set_input_hysteresis_enabled(pin, true);
    }

    gpio_init(PIN_RD);      gpio_set_dir(PIN_RD, GPIO_IN);
    gpio_init(PIN_WR);      gpio_set_dir(PIN_WR, GPIO_IN);
    gpio_init(PIN_IORQ);    gpio_set_dir(PIN_IORQ, GPIO_IN);
    gpio_init(PIN_SLTSL);   gpio_set_dir(PIN_SLTSL, GPIO_IN);
    gpio_init(PIN_BUSSDIR); gpio_set_dir(PIN_BUSSDIR, GPIO_IN);
    gpio_init(PIN_PSRAM);   gpio_set_dir(PIN_PSRAM, GPIO_IN);
}

unsigned long __no_inline_not_in_flash_func(read_ulong)(const unsigned char *ptr)
{
    return (unsigned long)ptr[0] |
           ((unsigned long)ptr[1] << 8) |
           ((unsigned long)ptr[2] << 16) |
           ((unsigned long)ptr[3] << 24);
}

int __no_inline_not_in_flash_func(isEndOfData)(const unsigned char *memory)
{
    for (int i = 0; i < ROM_RECORD_SIZE; i++)
    {
        if (memory[i] != 0xFF)
        {
            return 0;
        }
    }
    return 1;
}

static inline void __not_in_flash_func(prepare_rom_source)(
    uint32_t offset,
    bool cache_enable,
    uint32_t preferred_size,
    const uint8_t **rom_base_out,
    uint32_t *available_length_out)
{
    const uint8_t *rom_base = rom + offset;
    uint32_t available_length = active_rom_size;

    if (preferred_size != 0u && (available_length == 0u || available_length > preferred_size))
    {
        available_length = preferred_size;
    }

    if (cache_enable && available_length > 0u)
    {
        uint32_t bytes_to_cache = (available_length > rom_cache_capacity)
                                  ? rom_cache_capacity
                                  : available_length;

        gpio_init(PIN_WAIT);
        gpio_set_dir(PIN_WAIT, GPIO_OUT);
        gpio_put(PIN_WAIT, 0);

        // DMA bulk copy from flash XIP to SRAM.
        int dma_chan = dma_claim_unused_channel(true);
        dma_channel_config dma_cfg = dma_channel_get_default_config(dma_chan);
        channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
        channel_config_set_read_increment(&dma_cfg, true);
        channel_config_set_write_increment(&dma_cfg, true);
        dma_channel_configure(dma_chan, &dma_cfg,
            rom_sram,                        // write address (SRAM)
            rom_base,                        // read address (flash XIP)
            bytes_to_cache,                  // transfer count (bytes)
            true);                           // start immediately
        dma_channel_wait_for_finish_blocking(dma_chan);
        dma_channel_unclaim(dma_chan);
        gpio_set_dir(PIN_WAIT, GPIO_IN);

        rom_cached_size = bytes_to_cache;
        if (available_length <= rom_cache_capacity)
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

// -----------------------------------------------------------------------
// Variant of prepare_rom_source for the Sunrise+Mapper paths.
//
// Caller has already frozen the MSX bus by driving PIN_WAIT low via GPIO,
// so this helper must NOT toggle /WAIT while the cache DMA is running
// (the Z80 would otherwise resume before the mapper is allocated and the
// PIO read responder is reinstated).
// -----------------------------------------------------------------------
static inline void __not_in_flash_func(prepare_sunrise_mapper_rom_source)(
    uint32_t offset,
    bool cache_enable,
    const uint8_t **rom_base_out,
    uint32_t *available_length_out)
{
    const uint8_t *rom_base = rom + offset;
    uint32_t available_length = active_rom_size;

    if (cache_enable && available_length > 0u)
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
    sm_config_set_set_pins(&cfg_read, PIN_WAIT, 1);
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

    pio_sm_set_pins_with_mask(msx_bus.pio, msx_bus.sm_read, 0u, (1u << PIN_WAIT));
    pio_gpio_init(msx_bus.pio, PIN_WAIT);
    pio_sm_set_consecutive_pindirs(msx_bus.pio, msx_bus.sm_read, PIN_WAIT, 1, false);

    for (uint pin = PIN_D0; pin <= PIN_D7; ++pin)
    {
        pio_gpio_init(msx_bus.pio, pin);
    }

    pio_sm_set_consecutive_pindirs(msx_bus.pio, msx_bus.sm_read, PIN_D0, 8, false);
    pio_sm_set_consecutive_pindirs(msx_bus.pio, msx_bus.sm_write, PIN_D0, 8, false);

    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_read, true);
    pio_sm_set_enabled(msx_bus.pio, msx_bus.sm_write, true);
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

static inline void __not_in_flash_func(pio_drain_writes)(void (*handler)(uint16_t addr, uint8_t data, void *ctx), void *ctx)
{
    uint16_t addr;
    uint8_t data;
    while (pio_try_get_write(&addr, &data))
    {
        handler(addr, data, ctx);
    }
}

// -----------------------------------------------------------------------
// PIO I/O bus initialisation (for memory mapper port access on PIO1)
// -----------------------------------------------------------------------
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

// I/O bus helpers
static inline bool __not_in_flash_func(pio_try_get_io_write)(uint16_t *addr_out, uint8_t *data_out)
{
    if (pio_sm_is_rx_fifo_empty(msx_io_bus.pio_write, msx_io_bus.sm_io_write))
        return false;

    uint32_t sample = pio_sm_get(msx_io_bus.pio_write, msx_io_bus.sm_io_write);
    *addr_out = (uint16_t)(sample & 0xFFFFu);
    *data_out = (uint8_t)((sample >> 16) & 0xFFu);
    return true;
}

static inline uint8_t __not_in_flash_func(mapper_page_from_reg)(uint8_t reg)
{
    return (uint8_t)(reg % MAPPER_PAGES);
}

// -----------------------------------------------------------------------
// External PSRAM mapper backing (QMI CS1)
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
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_TXEMPTY_BITS) == 0)
    {
    }
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0)
    {
    }
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
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0)
    {
    }

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

    // PSRAM QMI timing — tuned for 210 MHz system clock.
    // CLKDIV=2 → SCK = 52.5 MHz (safe for all APS6404L/IPS6404L grades).
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

    // Verify PSRAM is functional via uncached window (bypass XIP cache
    // to guarantee the write reaches the device and the read comes back
    // from the device, not from a stale cache line).
    volatile uint32_t *psram_test = (volatile uint32_t *)0x15000000u;
    psram_test[0] = 0x12345678u;
    return psram_test[0] == 0x12345678u;
}

// -----------------------------------------------------------------------
// PSRAM Memory Manager — bump allocator for 8MB external PSRAM
// -----------------------------------------------------------------------
static psram_region_t mapper_region;
static psram_region_t c2_rom_region;

static struct {
    uint32_t next_free;
} psram_mgr;

static void psram_mem_init(void)
{
    psram_mgr.next_free = 0;
    memset(&mapper_region, 0, sizeof(mapper_region));
    memset(&c2_rom_region, 0, sizeof(c2_rom_region));
}

static bool psram_alloc(uint32_t size, psram_region_t *region)
{
    if (psram_mgr.next_free + size > PSRAM_TOTAL_SIZE)
        return false;
    region->offset = psram_mgr.next_free;
    region->size = size;
    region->ptr = (uint8_t *)(PSRAM_BASE_ADDR + psram_mgr.next_free);
    psram_mgr.next_free += size;
    return true;
}

static void psram_free_all(void)
{
    psram_mgr.next_free = 0;
    memset(&mapper_region, 0, sizeof(mapper_region));
    memset(&c2_rom_region, 0, sizeof(c2_rom_region));
}

// -----------------------------------------------------------------------
// PSRAM region helpers — mapper RAM
// -----------------------------------------------------------------------
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

static inline bool __not_in_flash_func(pio_try_get_io_read)(uint16_t *addr_out)
{
    if (pio_sm_is_rx_fifo_empty(msx_io_bus.pio_read, msx_io_bus.sm_io_read))
        return false;

    *addr_out = (uint16_t)pio_sm_get(msx_io_bus.pio_read, msx_io_bus.sm_io_read);
    return true;
}

// -----------------------------------------------------------------------
// Sunrise IDE write handler
// -----------------------------------------------------------------------
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

static inline void __not_in_flash_func(handle_menu_write)(uint16_t addr, uint8_t data, void *ctx)
{
    menu_select_ctx_t *menu_ctx = (menu_select_ctx_t *)ctx;
    if (addr == MONITOR_ADDR)
    {
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

int __no_inline_not_in_flash_func(loadrom_msx_menu)(uint32_t offset)
{
    active_rom_size = 32768u;
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, true, 32768u, &rom_base, &available_length);

    int record_count = 0;
    const uint8_t *record_ptr = rom + offset + 0x4000;
    for (int i = 0; i < MAX_ROM_RECORDS; i++)
    {
        if (isEndOfData(record_ptr))
        {
            break;
        }

        memcpy(records[record_count].Name, record_ptr, ROM_NAME_MAX);
        record_ptr += ROM_NAME_MAX;
        records[record_count].Mapper = *record_ptr++;
        records[record_count].Size = read_ulong(record_ptr);
        record_ptr += sizeof(unsigned long);
        records[record_count].Offset = read_ulong(record_ptr);
        record_ptr += sizeof(unsigned long);
        record_count++;
    }

    menu_select_ctx_t menu_ctx = { .rom_index = 0, .rom_selected = false };

    msx_pio_bus_init();

    while (true)
    {
        pio_drain_writes(handle_menu_write, &menu_ctx);

        if (menu_ctx.rom_selected)
        {
            // ROM selected — switch to non-blocking PIO reads.
            // After the MSX executes rst 0x00, the cartridge slot is
            // deselected.  On MSX2 the BIOS rescans via expanded slots
            // and reads addr 0x0000 through the cartridge (PIO catches
            // it).  On MSX1 the BIOS never selects the cartridge at
            // addr 0x0000, so detect the reboot on the raw bus via GPIO
            // instead (same approach as the non-PIO firmware).
            if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
            {
                uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
                pio_drain_writes(handle_menu_write, &menu_ctx);

                // MSX2 path: expanded-slot scan reaches addr 0x0000
                if (addr == 0x0000u)
                {
                    pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read,
                                        pio_build_token(false, 0xFFu));
                    return menu_ctx.rom_index;
                }

                // Serve the remaining menu ROM reads before the reset
                bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
                uint8_t data = 0xFFu;
                if (in_window)
                {
                    uint32_t rel = addr - 0x4000u;
                    if (available_length == 0u || rel < available_length)
                        data = read_rom_byte(rom_base, rel);
                }
                pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read,
                                    pio_build_token(in_window, data));
            }
            else
            {
                // PIO idle (SLTSL high) — the cartridge slot is not
                // selected.  MSX1 path: detect the reboot by watching
                // for addr 0x0000 on the address bus with RD active,
                // regardless of SLTSL.
                if (!gpio_get(PIN_RD) &&
                    ((gpio_get_all() & 0xFFFFu) == 0x0000u))
                {
                    return menu_ctx.rom_index;
                }
            }
        }
        else
        {
            // Normal operation — blocking PIO read
            uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio,
                                                           msx_bus.sm_read);

            pio_drain_writes(handle_menu_write, &menu_ctx);

            if (menu_ctx.rom_selected && addr == 0x0000u)
            {
                pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read,
                                    pio_build_token(false, 0xFFu));
                return menu_ctx.rom_index;
            }

            bool in_window = (addr >= 0x4000u) && (addr <= 0xBFFFu);
            uint8_t data = 0xFFu;

            if (in_window)
            {
                uint32_t rel = addr - 0x4000u;
                if (available_length == 0u || rel < available_length)
                    data = read_rom_byte(rom_base, rel);
            }

            pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read,
                                pio_build_token(in_window, data));
        }
    }
}

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

void __no_inline_not_in_flash_func(loadrom_konamiscc)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    msx_pio_bus_init();
    banked8_loop(rom_base, available_length, bank_registers, handle_konamiscc_write);
}

static void __no_inline_not_in_flash_func(core1_scc_audio)(void)
{
    while (true)
    {
        struct audio_buffer *buffer = take_audio_buffer(audio_pool, true);
        int16_t *samples = (int16_t *)buffer->buffer->bytes;
        for (int i = 0; i < SCC_AUDIO_BUFFER_SAMPLES; i++)
        {
            int16_t raw = SCC_calc(&scc_instance);
            int32_t boosted = (int32_t)raw << SCC_VOLUME_SHIFT;
            if (boosted > 32767) boosted = 32767;
            else if (boosted < -32768) boosted = -32768;
            int16_t s = (int16_t)boosted;
            samples[i * 2]     = s;
            samples[i * 2 + 1] = s;
        }
        buffer->sample_count = SCC_AUDIO_BUFFER_SAMPLES;
        give_audio_buffer(audio_pool, buffer);
    }
}

static void i2s_audio_init(void)
{
    if (scc_audio_ready)
        return;

    gpio_init(I2S_MUTE_PIN);
    gpio_set_dir(I2S_MUTE_PIN, GPIO_OUT);
    gpio_put(I2S_MUTE_PIN, 0);

    static audio_format_t audio_format = {
        .sample_freq = SCC_SAMPLE_RATE,
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .channel_count = 2,
    };

    static struct audio_buffer_format producer_format = {
        .format = &audio_format,
        .sample_stride = 4,
    };

    audio_pool = audio_new_producer_pool(&producer_format, 3, SCC_AUDIO_BUFFER_SAMPLES);

    if (scc_dma_channel < 0)
    {
        for (int ch = 0; ch < NUM_DMA_CHANNELS; ++ch)
        {
            if (!dma_channel_is_claimed((uint)ch))
            {
                scc_dma_channel = ch;
                break;
            }
        }
    }
    if (scc_dma_channel < 0)
    {
        audio_pool = NULL;
        return;
    }

    // Audio I2S lives on PIO1 SM2 so it never collides with the mapper
    // I/O responder (PIO1 SM0/SM1 used by msx_pio_io_bus_init).
    static struct audio_i2s_config i2s_config = {
        .data_pin = I2S_DATA_PIN,
        .clock_pin_base = I2S_BCLK_PIN,
        .dma_channel = 0,
        .pio_sm = 2,
    };

    i2s_config.dma_channel = (uint)scc_dma_channel;

    audio_i2s_setup(&audio_format, &i2s_config);
    audio_i2s_connect(audio_pool);
    audio_i2s_set_enabled(true);
    scc_audio_ready = true;
}

// -----------------------------------------------------------------------
// Polled SCC audio service — fills one I2S buffer if one is available.
// Used by Sunrise-mapper / Carnivore2 paths where Core 1 is occupied by
// the storage backend and cannot run a dedicated audio worker.
// -----------------------------------------------------------------------
void __not_in_flash_func(service_scc_audio)(void)
{
    if (!scc_audio_ready || !audio_pool)
        return;

    struct audio_buffer *buffer = take_audio_buffer(audio_pool, false);
    if (!buffer)
        return;

    int16_t *samples = (int16_t *)buffer->buffer->bytes;
    for (int i = 0; i < SCC_AUDIO_BUFFER_SAMPLES; i++)
    {
        int16_t raw = SCC_calc(&scc_instance);
        int32_t boosted = (int32_t)raw << SCC_VOLUME_SHIFT;
        if (boosted > 32767) boosted = 32767;
        else if (boosted < -32768) boosted = -32768;
        int16_t s = (int16_t)boosted;
        samples[i * 2]     = s;
        samples[i * 2 + 1] = s;
    }
    buffer->sample_count = SCC_AUDIO_BUFFER_SAMPLES;
    give_audio_buffer(audio_pool, buffer);
}

void __no_inline_not_in_flash_func(loadrom_konamiscc_scc)(uint32_t offset, bool cache_enable, uint32_t scc_type)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    memset(&scc_instance, 0, sizeof(SCC));
    scc_instance.clk  = SCC_CLOCK;
    scc_instance.rate = SCC_SAMPLE_RATE;
    SCC_set_quality(&scc_instance, 1);
    scc_instance.type = scc_type;
    SCC_reset(&scc_instance);

    i2s_audio_init();
    multicore_launch_core1(core1_scc_audio);

    msx_pio_bus_init();

    while (true)
    {
        uint16_t waddr;
        uint8_t  wdata;
        while (pio_try_get_write(&waddr, &wdata))
        {
            if      (waddr >= 0x5000u && waddr <= 0x57FFu) bank_registers[0] = wdata;
            else if (waddr >= 0x7000u && waddr <= 0x77FFu) bank_registers[1] = wdata;
            else if (waddr >= 0x9000u && waddr <= 0x97FFu) bank_registers[2] = wdata;
            else if (waddr >= 0xB000u && waddr <= 0xB7FFu) bank_registers[3] = wdata;

            SCC_write(&scc_instance, waddr, wdata);
        }

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

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
                uint32_t rel = ((uint32_t)bank_registers[(addr - 0x4000u) >> 13] * 0x2000u)
                             + (addr & 0x1FFFu);
                if (available_length == 0u || rel < available_length)
                    data = read_rom_byte(rom_base, rel);
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

void __no_inline_not_in_flash_func(loadrom_konami)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    msx_pio_bus_init();
    banked8_loop(rom_base, available_length, bank_registers, handle_konami_write);
}

void __no_inline_not_in_flash_func(loadrom_ascii8)(uint32_t offset, bool cache_enable)
{
    uint8_t bank_registers[4] = {0, 1, 2, 3};
    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    msx_pio_bus_init();
    banked8_loop(rom_base, available_length, bank_registers, handle_ascii8_write);
}

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

// -----------------------------------------------------------------------
// loadrom_sunrise - Sunrise IDE Nextor ROM with emulated IDE interface (USB)
// -----------------------------------------------------------------------
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

// -----------------------------------------------------------------------
// loadrom_sunrise_mapper - Sunrise IDE Nextor + 256KB Memory Mapper (USB)
// -----------------------------------------------------------------------
void __no_inline_not_in_flash_func(loadrom_sunrise_mapper)(uint32_t offset, bool cache_enable)
{
    (void)cache_enable;

    // Phase 1 — Bootstrap: minimal ROM that restarts the MSX
    static const uint8_t bootstrap_rom[] = {
        0x41, 0x42,            // 'AB' ROM header
        0x0A, 0x40,            // INIT = 0x400A
        0x00, 0x00,            // STATEMENT = none
        0x00, 0x00,            // DEVICE = none
        0x00, 0x00,            // TEXT = none
        0xF3,                  // DI
        0xDB, 0xF4,            // IN A, (0xF4)
        0xF6, 0x80,            // OR 0x80       (set bit 7 = cold boot)
        0xD3, 0xF4,            // OUT (0xF4), A
        0xC7                   // RST 0x00      (cold restart)
    };

    msx_pio_bus_init();

    bool restart_detected = false;
    bool init_called = false;

    while (!restart_detected)
    {
        { uint16_t waddr; uint8_t wdata; while (pio_try_get_write(&waddr, &wdata)) { } }

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
        else
        {
            if (init_called && !gpio_get(PIN_RD) && ((gpio_get_all() & 0xFFFFu) == 0x0000u))
                restart_detected = true;
        }
    }

    // Phase 2 — Initialise expanded-slot mapper
    pio_sm_set_enabled(pio0, 0, false);
    pio_sm_set_enabled(pio0, 1, false);
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

    // Allocate 1MB mapper RAM from the PSRAM memory manager
    psram_mem_init();
    if (!psram_alloc(MAPPER_SIZE, &mapper_region))
    {
        while (true) { tight_loop_contents(); }
    }

    uint8_t mapper_reg[4] = { 3, 2, 1, 0 };
    uint8_t subslot_reg = 0x10;

    mapper_fill_ff();

    msx_pio_io_bus_init();
    msx_pio_bus_init();

    sunrise_ctx_t ctx = { .ide = &ide };

    while (true)
    {
        // Drain memory writes
        {
            uint16_t waddr; uint8_t wdata;
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
        }

        // Drain I/O writes (mapper page registers FC-FF)
        {
            uint16_t io_addr; uint8_t io_data;
            while (pio_try_get_io_write(&io_addr, &io_data))
            {
                uint8_t port = io_addr & 0xFFu;
                if (port >= 0xFCu && port <= 0xFFu)
                    mapper_reg[port - 0xFCu] = io_data & 0x3Fu;
            }
        }

        // Handle I/O reads (mapper page registers FC-FF)
        {
            uint16_t io_addr;
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

        // Handle memory reads
        if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
            uint8_t data = 0xFFu;
            bool in_window = false;

            if (addr == 0xFFFFu)
            {
                in_window = true;
                data = ~subslot_reg;
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

// -----------------------------------------------------------------------
// loadrom_sunrise_sd - Sunrise IDE Nextor ROM via microSD (no mapper)
// -----------------------------------------------------------------------
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

// -----------------------------------------------------------------------
// loadrom_sunrise_mapper_sd - Sunrise IDE + 256KB mapper via microSD
// -----------------------------------------------------------------------
void __no_inline_not_in_flash_func(loadrom_sunrise_mapper_sd)(uint32_t offset, bool cache_enable)
{
    (void)cache_enable;

    // Phase 1 — Bootstrap: minimal ROM that restarts the MSX
    static const uint8_t bootstrap_rom[] = {
        0x41, 0x42, 0x0A, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xF3, 0xDB, 0xF4, 0xF6, 0x80, 0xD3, 0xF4, 0xC7
    };

    msx_pio_bus_init();

    bool restart_detected = false;
    bool init_called = false;

    while (!restart_detected)
    {
        { uint16_t waddr; uint8_t wdata; while (pio_try_get_write(&waddr, &wdata)) { } }

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
        else
        {
            if (init_called && !gpio_get(PIN_RD) && ((gpio_get_all() & 0xFFFFu) == 0x0000u))
                restart_detected = true;
        }
    }

    // Phase 2 — Initialise expanded-slot mapper
    pio_sm_set_enabled(pio0, 0, false);
    pio_sm_set_enabled(pio0, 1, false);
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 0);

    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_sunrise_mapper_rom_source(offset, cache_enable, &rom_base, &available_length);

    static sunrise_ide_t ide;
    sunrise_ide_init(&ide);

    sunrise_sd_set_ide_ctx(&ide);
    multicore_launch_core1(sunrise_sd_task);

    // Allocate 1MB mapper RAM from the PSRAM memory manager
    psram_mem_init();
    if (!psram_alloc(MAPPER_SIZE, &mapper_region))
    {
        while (true) { tight_loop_contents(); }
    }

    uint8_t mapper_reg[4] = { 3, 2, 1, 0 };
    uint8_t subslot_reg = 0x10;

    mapper_fill_ff();

    msx_pio_io_bus_init();
    msx_pio_bus_init();

    sunrise_ctx_t ctx = { .ide = &ide };

    while (true)
    {
        // Drain memory writes
        {
            uint16_t waddr; uint8_t wdata;
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
        }

        // Drain I/O writes (mapper page registers FC-FF)
        {
            uint16_t io_addr; uint8_t io_data;
            while (pio_try_get_io_write(&io_addr, &io_data))
            {
                uint8_t port = io_addr & 0xFFu;
                if (port >= 0xFCu && port <= 0xFFu)
                    mapper_reg[port - 0xFCu] = io_data & 0x3Fu;
            }
        }

        // Handle I/O reads (mapper page registers FC-FF)
        {
            uint16_t io_addr;
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

        // Handle memory reads
        if (!pio_sm_is_rx_fifo_empty(msx_bus.pio, msx_bus.sm_read))
        {
            uint16_t addr = (uint16_t)pio_sm_get(msx_bus.pio, msx_bus.sm_read);
            uint8_t data = 0xFFu;
            bool in_window = false;

            if (addr == 0xFFFFu)
            {
                in_window = true;
                data = ~subslot_reg;
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

// -----------------------------------------------------------------------
// loadrom_planar64 - 64KB Planar ROM (no mapper)
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

        bool in_window = true;
        uint8_t data = 0xFFu;

        if (available_length == 0u || (uint32_t)addr < available_length)
            data = read_rom_byte(rom_base, (uint32_t)addr);

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// -----------------------------------------------------------------------
// Bank cache infrastructure for mapper-aware LRU caching
// -----------------------------------------------------------------------
#define BANK_CACHE_MAX_SLOTS 32
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
// AMD-compatible flash command emulation for ASCII16-X
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
    uint16_t         bank_regs[2];
    bank_cache_t     cache;
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

// -----------------------------------------------------------------------
// loadrom_ascii16x - ASCII16-X mapper
// -----------------------------------------------------------------------
void __no_inline_not_in_flash_func(loadrom_ascii16x)(uint32_t offset, bool cache_enable)
{
    (void)cache_enable;

    ascii16x_state_t state;
    memset(&state, 0, sizeof(state));

    bcache_init(&state.cache, 16384u, 2, rom + offset, active_rom_size);
    bcache_prefill(&state.cache);

    state.cache.page_slot[0] = bcache_find(&state.cache, 0);
    state.cache.page_slot[1] = state.cache.page_slot[0];

    msx_pio_bus_init();

    while (true)
    {
        pio_drain_writes(handle_ascii16x_write_cached, &state);

        uint16_t addr = (uint16_t)pio_sm_get_blocking(msx_bus.pio, msx_bus.sm_read);

        pio_drain_writes(handle_ascii16x_write_cached, &state);

        uint8_t data = 0xFFu;
        uint8_t page_idx = ((addr >> 14) & 0x01u) ? 0u : 1u;
        int8_t slot = state.cache.page_slot[page_idx];

        if (slot >= 0)
            data = rom_sram[(uint32_t)slot * state.cache.slot_size + (addr & 0x3FFFu)];

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(true, data));
    }
}

// -----------------------------------------------------------------------
// Manbow2 mapper — Konami SCC banking + AMD AM29F040B flash emulation
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
    uint32_t rom_size_mask;
} manbow2_ctx_t;

static inline void __not_in_flash_func(handle_manbow2_write)(uint16_t addr, uint8_t data, void *ctx)
{
    manbow2_ctx_t *mb = (manbow2_ctx_t *)ctx;

    if (addr < 0x4000u || addr > 0xBFFFu)
        return;

    uint8_t page = (uint8_t)((addr - 0x4000u) >> 13);
    uint32_t flash_addr = (uint32_t)mb->bank_regs[page] * 0x2000u + (addr & 0x1FFFu);

    if ((addr & 0x1800u) == 0x1000u)
        mb->bank_regs[page] = data & 0x3Fu;

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
            if (data == 0xA0u) mb->state = MBW2_PROGRAM;
            else if (data == 0x90u) mb->state = MBW2_AUTOSELECT;
            else if (data == 0x80u) mb->state = MBW2_ERASE_CMD1;
            else mb->state = MBW2_READ;
            break;
        case MBW2_PROGRAM:
            if (flash_addr >= mb->writable_offset &&
                flash_addr < mb->writable_offset + mb->writable_size)
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
                    flash_addr < mb->writable_offset + mb->writable_size)
                    memset(mb->writable_sram, 0xFF, mb->writable_size);
            }
            mb->state = MBW2_READ;
            break;
        case MBW2_AUTOSELECT:
            break;
    }
}

// -----------------------------------------------------------------------
// loadrom_manbow2 - Manbow2 (Konami SCC + AMD flash) mapper
// -----------------------------------------------------------------------
void __no_inline_not_in_flash_func(loadrom_manbow2)(uint32_t offset, bool cache_enable)
{
    const uint8_t *rom_base;
    uint32_t available_length;

    static const uint32_t WRITABLE_SECTOR_SIZE  = 0x10000u;
    static const uint32_t WRITABLE_SECTOR_OFFSET = 0x70000u;
    uint32_t reduced_cache = CACHE_SIZE - WRITABLE_SECTOR_SIZE;

    rom_cache_capacity = reduced_cache;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    uint8_t *writable_sram = &rom_sram[reduced_cache];

    if (available_length >= WRITABLE_SECTOR_OFFSET + WRITABLE_SECTOR_SIZE)
    {
        const uint8_t *flash_rom = rom + offset;
        memcpy(writable_sram, flash_rom + WRITABLE_SECTOR_OFFSET, WRITABLE_SECTOR_SIZE);
    }
    else if (available_length > WRITABLE_SECTOR_OFFSET)
    {
        const uint8_t *flash_rom = rom + offset;
        uint32_t partial = available_length - WRITABLE_SECTOR_OFFSET;
        memcpy(writable_sram, flash_rom + WRITABLE_SECTOR_OFFSET, partial);
        memset(writable_sram + partial, 0xFF, WRITABLE_SECTOR_SIZE - partial);
    }
    else
    {
        memset(writable_sram, 0xFF, WRITABLE_SECTOR_SIZE);
    }

    manbow2_ctx_t mb = {
        .bank_regs = {0, 1, 2, 3},
        .state = MBW2_READ,
        .writable_sram = writable_sram,
        .writable_offset = WRITABLE_SECTOR_OFFSET,
        .writable_size = WRITABLE_SECTOR_SIZE,
        .rom_size_mask = 0
    };

    msx_pio_bus_init();

    while (true)
    {
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
                uint32_t id_addr = rel & 0x03u;
                if (id_addr == 0x00u) data = 0x01u;
                else if (id_addr == 0x01u) data = 0xA4u;
                else if (id_addr == 0x02u)
                    data = (rel >= mb.writable_offset && rel < mb.writable_offset + mb.writable_size) ? 0x00u : 0x01u;
                else data = 0x00u;
            }
            else
            {
                if (rel >= mb.writable_offset && rel < mb.writable_offset + mb.writable_size)
                    data = writable_sram[rel - mb.writable_offset];
                else if (available_length == 0u || rel < available_length)
                    data = read_rom_byte(rom_base, rel);
            }
        }

        pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
    }
}

// -----------------------------------------------------------------------
// loadrom_manbow2_scc - Manbow2 mapper with SCC/SCC+ sound emulation
// -----------------------------------------------------------------------
void __no_inline_not_in_flash_func(loadrom_manbow2_scc)(uint32_t offset, bool cache_enable, uint32_t scc_type)
{
    const uint8_t *rom_base;
    uint32_t available_length;

    static const uint32_t WRITABLE_SECTOR_SIZE  = 0x10000u;
    static const uint32_t WRITABLE_SECTOR_OFFSET = 0x70000u;
    uint32_t reduced_cache = CACHE_SIZE - WRITABLE_SECTOR_SIZE;

    rom_cache_capacity = reduced_cache;
    prepare_rom_source(offset, cache_enable, 0u, &rom_base, &available_length);

    uint8_t *writable_sram = &rom_sram[reduced_cache];

    if (available_length >= WRITABLE_SECTOR_OFFSET + WRITABLE_SECTOR_SIZE)
    {
        const uint8_t *flash_rom = rom + offset;
        memcpy(writable_sram, flash_rom + WRITABLE_SECTOR_OFFSET, WRITABLE_SECTOR_SIZE);
    }
    else if (available_length > WRITABLE_SECTOR_OFFSET)
    {
        const uint8_t *flash_rom = rom + offset;
        uint32_t partial = available_length - WRITABLE_SECTOR_OFFSET;
        memcpy(writable_sram, flash_rom + WRITABLE_SECTOR_OFFSET, partial);
        memset(writable_sram + partial, 0xFF, WRITABLE_SECTOR_SIZE - partial);
    }
    else
    {
        memset(writable_sram, 0xFF, WRITABLE_SECTOR_SIZE);
    }

    manbow2_ctx_t mb = {
        .bank_regs = {0, 1, 2, 3},
        .state = MBW2_READ,
        .writable_sram = writable_sram,
        .writable_offset = WRITABLE_SECTOR_OFFSET,
        .writable_size = WRITABLE_SECTOR_SIZE,
        .rom_size_mask = 0
    };

    memset(&scc_instance, 0, sizeof(SCC));
    scc_instance.clk  = SCC_CLOCK;
    scc_instance.rate = SCC_SAMPLE_RATE;
    SCC_set_quality(&scc_instance, 1);
    scc_instance.type = scc_type;
    SCC_reset(&scc_instance);

    i2s_audio_init();
    multicore_launch_core1(core1_scc_audio);

    msx_pio_bus_init();

    while (true)
    {
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
                    if (id_addr == 0x00u) data = 0x01u;
                    else if (id_addr == 0x01u) data = 0xA4u;
                    else if (id_addr == 0x02u)
                        data = (rel >= mb.writable_offset && rel < mb.writable_offset + mb.writable_size) ? 0x00u : 0x01u;
                    else data = 0x00u;
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

// -----------------------------------------------------------------------
// loadrom_c2_common - Carnivore2 RAM-mode emulation with SCC support
// Sub-slot layout (Carnivore2-compatible):
//   sub-slot 0 = C2 register window + bank windows (SROM upload target)
//   sub-slot 1 = Nextor (Sunrise IDE BIOS, SD or USB backend)
//   sub-slot 2 = MSX 1MB memory mapper RAM
// -----------------------------------------------------------------------

typedef void (*c2_core1_task_t)(void);

static void __no_inline_not_in_flash_func(loadrom_c2_common)(uint32_t offset,
                                                             bool cache_enable,
                                                             bool scc_emulation,
                                                             bool scc_plus,
                                                             c2_core1_task_t core1_task,
                                                             void (*attach_ctx)(sunrise_ide_t *))
{
    bool c2_scc_enabled = scc_emulation || scc_plus;

    // Phase 1 — Bootstrap: tiny ROM that cold-restarts the MSX so the
    // BIOS sees the final expanded-slot layout on its next probe.
    static const uint8_t bootstrap_rom[] = {
        0x41, 0x42,            // 'AB' ROM header
        0x0A, 0x40,            // INIT = 0x400A
        0x00, 0x00,            // STATEMENT = none
        0x00, 0x00,            // DEVICE = none
        0x00, 0x00,            // TEXT = none
        0xF3,                  // DI
        0xDB, 0xF4,            // IN A, (0xF4)
        0xF6, 0x80,            // OR 0x80
        0xD3, 0xF4,            // OUT (0xF4), A
        0xC7                   // RST 0x00
    };

    msx_pio_bus_init();

    bool restart_detected = false;
    bool init_called = false;

    while (!restart_detected)
    {
        { uint16_t waddr; uint8_t wdata; while (pio_try_get_write(&waddr, &wdata)) { } }

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
        else
        {
            if (init_called && !gpio_get(PIN_RD) && ((gpio_get_all() & 0xFFFFu) == 0x0000u))
                restart_detected = true;
        }
    }

    // Phase 2 — freeze MSX before it probes slots again, then wire
    // expanded slot + Nextor + mapper RAM + Carnivore2 emulation.
    pio_sm_set_enabled(pio0, 0, false);
    pio_sm_set_enabled(pio0, 1, false);
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 0);

    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_sunrise_mapper_rom_source(offset, cache_enable, &rom_base, &available_length);

    static sunrise_ide_t ide;
    sunrise_ide_init(&ide);

    attach_ctx(&ide);
    multicore_launch_core1(core1_task);

    // PSRAM regions: 1 MB mapper RAM + 1 MB Carnivore2 RAM.
    psram_mem_init();
    if (!psram_alloc(MAPPER_SIZE, &mapper_region) ||
        !psram_alloc(MAPPER_SIZE, &c2_rom_region))
    {
        while (true) { tight_loop_contents(); }
    }

    uint8_t mapper_reg[4] = { 3, 2, 1, 0 };
    uint8_t subslot_reg = 0x10;

    mapper_fill_ff();

    // Initialise Carnivore2 state.
    static c2_state_t c2;
    c2_init(&c2, c2_rom_region.ptr, c2_rom_region.size, 0x01u);

    static const uint8_t c2_descr[8] = {
        'C', 'M', 'F', 'C', 'C', 'F', 'R', 'C'
    };
    bool c2_signature_overlay_armed = false;
    uint8_t c2_signature_overlay_index = 0u;

    if (c2_scc_enabled)
    {
        memset(&scc_instance, 0, sizeof(SCC));
        scc_instance.clk = SCC_CLOCK;
        scc_instance.rate = SCC_SAMPLE_RATE;
        SCC_set_quality(&scc_instance, 1);
        scc_instance.type = scc_plus ? SCC_ENHANCED : SCC_STANDARD;
        SCC_reset(&scc_instance);
        i2s_audio_init();
    }

    msx_pio_io_bus_init();
    msx_pio_bus_init();

    while (true)
    {
        // --- Drain memory writes ---
        {
            uint16_t waddr; uint8_t wdata;
            while (pio_try_get_write(&waddr, &wdata))
            {
                if (waddr == 0xFFFFu)
                {
                    subslot_reg = wdata;
                    continue;
                }

                uint8_t page = (waddr >> 14) & 0x03u;
                uint8_t active_subslot = (subslot_reg >> (page * 2)) & 0x03u;

                if (active_subslot == 1)
                {
                    if (waddr >= 0x4000u && waddr <= 0x7FFFu)
                        sunrise_ide_handle_write(&ide, waddr, wdata);
                }
                else if (active_subslot == 0)
                {
                    if (c2_addr_is_regwin(&c2, waddr))
                    {
                        c2_reg_write(&c2, waddr, wdata);
                    }
                    else
                    {
                        if (c2_scc_enabled)
                            SCC_write(&scc_instance, waddr, wdata);

                        c2_bank_switch_write(&c2, waddr, wdata);

                        uint8_t bank_idx;
                        uint32_t linear;
                        if (c2_decode_addr(&c2, waddr, &bank_idx, &linear))
                        {
                            bool is_ram = c2_bank_is_ram(&c2, bank_idx);
                            bool is_we  = c2_bank_is_we (&c2, bank_idx);
                            if (is_ram && is_we)
                            {
                                if (linear < c2.ram_size)
                                {
                                    c2.ram_ptr[linear] = wdata;
                                    if (linear + 1u > c2.max_written) c2.max_written = linear + 1u;
                                }
                            }
                            else if (is_we && !is_ram)
                            {
                                c2_flash_cmd_write(&c2, waddr, wdata);
                            }
                        }
                    }
                }
                else if (active_subslot == 2)
                {
                    uint8_t mapper_page = mapper_page_from_reg(mapper_reg[page]);
                    uint32_t mapper_offset = ((uint32_t)mapper_page << 14) | (waddr & 0x3FFFu);
                    mapper_write_byte(mapper_offset, wdata);
                }
            }
        }

        // --- Drain I/O writes (mapper page registers FC-FF + Carnivore2 port F0+PFXN) ---
        {
            uint16_t io_addr; uint8_t io_data;
            while (pio_try_get_io_write(&io_addr, &io_data))
            {
                uint8_t port = io_addr & 0xFFu;
                if (port >= 0xFCu && port <= 0xFFu)
                {
                    mapper_reg[port - 0xFCu] = io_data & 0x3Fu;
                }
                else if ((port & 0xFCu) == 0xF0u &&
                         (port & 0x03u) == (c2.pfxn & 0x03u))
                {
                    if (io_data == 'C' || io_data == 'S')
                    {
                        c2_signature_overlay_armed = true;
                        c2_signature_overlay_index = 0u;
                    }
                    c2_port_write(&c2, io_data);
                }
            }
        }

        // --- Handle I/O reads ---
        {
            uint16_t io_addr;
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
                else if ((port & 0xFCu) == 0xF0u &&
                         (port & 0x03u) == (c2.pfxn & 0x03u) &&
                         c2.pf0_state != C2_PF0_IDLE)
                {
                    in_window = true;
                    data = c2_port_read(&c2);
                }
                pio_sm_put_blocking(msx_io_bus.pio_read, msx_io_bus.sm_io_read, pio_build_token(in_window, data));
            }
        }

        // --- Handle memory reads ---
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

                if (c2_signature_overlay_armed &&
                    c2_signature_overlay_index < sizeof(c2_descr) &&
                    addr == (uint16_t)(0x4010u + c2_signature_overlay_index) &&
                    (c2.cardmdr & C2_CARDMDR_REGS_DISABLE) == 0u)
                {
                    in_window = true;
                    data = c2_descr[c2_signature_overlay_index++];
                    if (c2_signature_overlay_index >= sizeof(c2_descr))
                    {
                        c2_signature_overlay_armed = false;
                    }
                    pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read,
                                        pio_build_token(in_window, data));
                    continue;
                }
                else if (c2_signature_overlay_armed &&
                         addr >= 0x4010u && addr <= 0x4017u &&
                         (c2.cardmdr & C2_CARDMDR_REGS_DISABLE) == 0u)
                {
                    c2_signature_overlay_armed = false;
                    c2_signature_overlay_index = 0u;
                }

                if (active_subslot == 1)
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
                else if (active_subslot == 0)
                {
                    bool regwin_read_active =
                        ((c2.cardmdr & C2_CARDMDR_REGS_DISABLE) == 0u) &&
                        ((c2.cardmdr & C2_CARDMDR_REG_RD_OFF) == 0u) &&
                        c2_addr_is_regwin(&c2, addr);

                    if (regwin_read_active)
                    {
                        in_window = true;
                        data = c2_reg_read(&c2, addr);
                    }
                    else
                    {
                        bool is_scc_read = false;
                        if (c2_scc_enabled)
                        {
                            if (scc_instance.active)
                            {
                                uint32_t scc_reg_start = scc_instance.base_adr + 0x800u;
                                if (addr >= scc_reg_start && addr <= (scc_reg_start + 0xFFu))
                                    is_scc_read = true;
                            }
                            if (scc_plus && (addr & 0xFFFEu) == 0xBFFEu)
                                is_scc_read = true;
                        }

                        if (is_scc_read)
                        {
                            in_window = true;
                            data = (uint8_t)SCC_read(&scc_instance, addr);
                        }
                        else
                        {
                            uint8_t bank_idx;
                            uint32_t linear;
                            if (c2_decode_addr(&c2, addr, &bank_idx, &linear))
                            {
                                in_window = true;
                                if (!c2_bank_is_ram(&c2, bank_idx))
                                {
                                    data = c2_flash_read(&c2, addr, linear);
                                }
                                else
                                {
                                    data = (linear < c2.ram_size) ? c2.ram_ptr[linear] : 0xFFu;
                                }
                            }
                        }
                    }
                }
                else if (active_subslot == 2)
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

void __no_inline_not_in_flash_func(loadrom_c2_sd)(uint32_t offset, bool cache_enable,
                                                  bool scc_emulation, bool scc_plus)
{
    loadrom_c2_common(offset, cache_enable, scc_emulation, scc_plus,
                      sunrise_sd_task, sunrise_sd_set_ide_ctx);
}

void __no_inline_not_in_flash_func(loadrom_c2_usb)(uint32_t offset, bool cache_enable,
                                                   bool scc_emulation, bool scc_plus)
{
    loadrom_c2_common(offset, cache_enable, scc_emulation, scc_plus,
                      sunrise_usb_task, sunrise_usb_set_ide_ctx);
}

// -----------------------------------------------------------------------
// loadrom_sunrise_wifi_common - Sunrise IDE Nextor + ESP-01 WiFi (memio UART),
// optional 1MB MSX memory mapper. Mirrors loadrom.pio implementation.
// Sub-slot layout:
//   sub-slot 0 = WiFi BIOS + UART memio (0x4000-0x7FFF)
//   sub-slot 1 = Sunrise IDE Nextor BIOS (0x4000-0x7FFF)
//   sub-slot 2 = MSX 1MB mapper RAM (when mapper_enable)
// -----------------------------------------------------------------------
typedef void (*sunrise_backend_task_fn_t)(void);
typedef void (*sunrise_backend_attach_fn_t)(sunrise_ide_t *ide);

static void __no_inline_not_in_flash_func(loadrom_sunrise_wifi_common)(
    uint32_t offset,
    bool cache_enable,
    sunrise_backend_task_fn_t core1_task,
    sunrise_backend_attach_fn_t attach_ctx,
    bool mapper_enable)
{
    (void)cache_enable;

    // Phase 1 — Bootstrap: tiny ROM that cold-restarts the MSX so the
    // BIOS sees the final expanded-slot layout on its next probe.
    static const uint8_t bootstrap_rom[] = {
        0x41, 0x42,
        0x0A, 0x40,
        0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00,
        0xF3,
        0xDB, 0xF4,
        0xF6, 0x80,
        0xD3, 0xF4,
        0xC7
    };

    msx_pio_bus_init();

    bool restart_detected = false;
    bool init_called = false;

    while (!restart_detected)
    {
        { uint16_t waddr; uint8_t wdata; while (pio_try_get_write(&waddr, &wdata)) { } }

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
        else
        {
            if (init_called && !gpio_get(PIN_RD) && ((gpio_get_all() & 0xFFFFu) == 0x0000u))
                restart_detected = true;
        }
    }

    // Phase 2 — Initialise expanded-slot WiFi + Nextor (+ optional mapper)
    pio_sm_set_enabled(pio0, 0, false);
    pio_sm_set_enabled(pio0, 1, false);
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 0);

    const uint8_t *rom_base;
    uint32_t available_length;
    prepare_sunrise_mapper_rom_source(offset, cache_enable, &rom_base, &available_length);
    const uint8_t *wifi_rom_base = rom + offset + available_length;

    static sunrise_ide_t ide;
    sunrise_ide_init(&ide);

    attach_ctx(&ide);
    multicore_launch_core1(core1_task);
    wifi_uart_init_once();

    uint8_t mapper_reg[4] = { 3, 2, 1, 0 };
    uint8_t subslot_reg = mapper_enable ? 0x20u : 0x00u;

    if (mapper_enable)
    {
        psram_mem_init();
        if (!psram_alloc(MAPPER_SIZE, &mapper_region))
        {
            while (true) { tight_loop_contents(); }
        }
        mapper_fill_ff();
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
                {
                    sunrise_ide_handle_write(&ide, waddr, wdata);
                }
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
                {
                    mapper_reg[port - 0xFCu] = io_data & 0x3Fu;
                }
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
                    if (addr >= 0x4000u && addr <= 0x7FFFu)
                    {
                        data = wifi_handle_mem_read(wifi_rom_base, addr);
                    }
                }
                else if (active_subslot == 1)
                {
                    if (addr >= 0x4000u && addr <= 0x7FFFu)
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
                            {
                                data = read_rom_byte(rom_base, rel);
                            }
                        }
                    }
                }
                else if (mapper_enable && active_subslot == 2)
                {
                    uint8_t mapper_page = mapper_page_from_reg(mapper_reg[page]);
                    uint32_t mapper_offset = ((uint32_t)mapper_page << 14) | (addr & 0x3FFFu);
                    data = mapper_read_byte(mapper_offset);
                }
            }

            pio_sm_put_blocking(msx_bus.pio, msx_bus.sm_read, pio_build_token(in_window, data));
        }
    }
}

void __no_inline_not_in_flash_func(loadrom_sunrise_wifi)(uint32_t offset, bool cache_enable)
{
    loadrom_sunrise_wifi_common(offset, cache_enable, sunrise_usb_task, sunrise_usb_set_ide_ctx, false);
}

void __no_inline_not_in_flash_func(loadrom_sunrise_wifi_sd)(uint32_t offset, bool cache_enable)
{
    loadrom_sunrise_wifi_common(offset, cache_enable, sunrise_sd_task, sunrise_sd_set_ide_ctx, false);
}

void __no_inline_not_in_flash_func(loadrom_sunrise_mapper_wifi)(uint32_t offset, bool cache_enable)
{
    loadrom_sunrise_wifi_common(offset, cache_enable, sunrise_usb_task, sunrise_usb_set_ide_ctx, true);
}

void __no_inline_not_in_flash_func(loadrom_sunrise_mapper_wifi_sd)(uint32_t offset, bool cache_enable)
{
    loadrom_sunrise_wifi_common(offset, cache_enable, sunrise_sd_task, sunrise_sd_set_ide_ctx, true);
}

int __no_inline_not_in_flash_func(main)()
{
    qmi_hw->m[0].timing = 0x40000202;
    set_sys_clock_khz(210000, true);

    stdio_init_all();
    setup_gpio();

    int rom_index = loadrom_msx_menu(0x0000);
    active_rom_size = (uint32_t)records[rom_index].Size;

    uint8_t mapper = records[rom_index].Mapper;
    bool scc_emulation = (mapper & SCC_FLAG) != 0;
    bool scc_plus = (mapper & SCC_PLUS_FLAG) != 0;
    bool wifi_enabled = (mapper & WIFI_FLAG) != 0;
    uint8_t base_mapper = mapper & ~(SCC_FLAG | SCC_PLUS_FLAG | WIFI_FLAG);

    if ((base_mapper == 11u || base_mapper == 16u ||
         base_mapper == 17u || base_mapper == 18u) && !psram_init())
    {
        while (true)
        {
            tight_loop_contents();
        }
    }

    switch (base_mapper)
    {
        case 1:
        case 2:
            loadrom_plain32(records[rom_index].Offset, true);
            break;
        case 3:
            loadrom_konamiscc_scc(records[rom_index].Offset, true, scc_plus ? SCC_ENHANCED : SCC_STANDARD);
            break;
        case 4:
            loadrom_linear48(records[rom_index].Offset, true);
            break;
        case 5:
            loadrom_ascii8(records[rom_index].Offset, true);
            break;
        case 6:
            loadrom_ascii16(records[rom_index].Offset, true);
            break;
        case 7:
            loadrom_konami(records[rom_index].Offset, true);
            break;
        case 8:
            loadrom_neo8(records[rom_index].Offset);
            break;
        case 9:
            loadrom_neo16(records[rom_index].Offset);
            break;
        case 10:
            if (wifi_enabled)
                loadrom_sunrise_wifi(records[rom_index].Offset, true);
            else
                loadrom_sunrise(records[rom_index].Offset, true);
            break;
        case 11:
            if (wifi_enabled)
                loadrom_sunrise_mapper_wifi(records[rom_index].Offset, true);
            else
                loadrom_sunrise_mapper(records[rom_index].Offset, true);
            break;
        case 12:
            loadrom_ascii16x(records[rom_index].Offset, true);
            break;
        case 13:
            loadrom_planar64(records[rom_index].Offset, true);
            break;
        case 14:
            if (scc_plus || (mapper & SCC_FLAG))
                loadrom_manbow2_scc(records[rom_index].Offset, true, scc_plus ? SCC_ENHANCED : SCC_STANDARD);
            else
                loadrom_manbow2(records[rom_index].Offset, true);
            break;
        case 15:
            if (wifi_enabled)
                loadrom_sunrise_wifi_sd(records[rom_index].Offset, true);
            else
                loadrom_sunrise_sd(records[rom_index].Offset, true);
            break;
        case 16:
            if (wifi_enabled)
                loadrom_sunrise_mapper_wifi_sd(records[rom_index].Offset, true);
            else
                loadrom_sunrise_mapper_sd(records[rom_index].Offset, true);
            break;
        case 17:
            loadrom_c2_sd(records[rom_index].Offset, true, scc_emulation, scc_plus);
            break;
        case 18:
            loadrom_c2_usb(records[rom_index].Offset, true, scc_emulation, scc_plus);
            break;
        default:
            printf("Debug: Unsupported ROM mapper: %d\n", mapper);
            break;
    }

    return 0;
}