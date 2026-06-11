// MSX PICOVERSE PROJECT
// (c) 2026 Cristiano Goncalves
// The Retro Hacker
//
// midi_main.c - Standalone USB MIDI → MSX-MIDI interface firmware
//
// Turns the PicoVerse 2040 cartridge into a dedicated MSX-MIDI interface.
// MSX software (MIDI players) writes MIDI data through standard MSX-MIDI
// I/O ports (0xE8-0xEF, 8251 USART + 8253 timer) and the firmware
// forwards the data to a USB MIDI device (e.g., Roland SoundCanvas)
// connected to the Pico's USB port via a USB-MIDI cable.
//
// Emulates the MSX-MIDI interface as found in the Panasonic FS-A1GT
// turbo R and the ESEMSX3 FPGA implementation:
//   0xE8 (R/W) — 8251 data register: MIDI TX and RX
//   0xE9 (R/W) — 8251 status (read) / command-mode (write)
//   0xEA-0xEB  — Timer interrupt acknowledge (stub)
//   0xEC-0xEF  — 8253 timer counters and control (stub)
//
// Architecture:
//   Core 0: PIO1 IRQ handler services MSX I/O bus requests.
//           Captures MIDI data writes from port 0xE8 into a ring buffer.
//           Returns status/data for I/O reads. Main loop idles (wfi).
//   Core 1: TinyUSB host task polls USB MIDI device. Drains the TX ring
//           buffer, parses MIDI bytes into USB-MIDI event packets, and
//           sends them to the USB MIDI device. Received MIDI data from
//           the device is placed in an RX ring buffer for the MSX to read.
//
// This work is licensed under a "Creative Commons Attribution-NonCommercial-
// ShareAlike 4.0 International License".
// https://creativecommons.org/licenses/by-nc-sa/4.0/

#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "tusb.h"
#include "midi.h"
#include "usb_midi_host.h"

#define MIDI_WIRE_BYTE_US 320
#include "msx_midi.pio.h"

// -----------------------------------------------------------------------
// TX ring buffer: Core 0 (IRQ) writes, Core 1 reads
// Single-producer single-consumer — safe without locks on RP2040
// -----------------------------------------------------------------------
static volatile uint8_t tx_ring_buf[MIDI_TX_BUFSIZE];
static volatile uint32_t tx_ring_head;   // Written by Core 0
static volatile uint32_t tx_ring_tail;   // Written by Core 1

static inline bool __not_in_flash_func(tx_ring_put)(uint8_t byte) {
    uint32_t next = (tx_ring_head + 1) & (MIDI_TX_BUFSIZE - 1);
    if (next == tx_ring_tail) return false;  // Full
    tx_ring_buf[tx_ring_head] = byte;
    __dmb();
    tx_ring_head = next;
    return true;
}

static inline bool tx_ring_get(uint8_t *byte) {
    if (tx_ring_head == tx_ring_tail) return false;  // Empty
    *byte = tx_ring_buf[tx_ring_tail];
    __dmb();
    tx_ring_tail = (tx_ring_tail + 1) & (MIDI_TX_BUFSIZE - 1);
    return true;
}

static inline bool __not_in_flash_func(tx_ring_is_full)(void) {
    return ((tx_ring_head + 1) & (MIDI_TX_BUFSIZE - 1)) == tx_ring_tail;
}

static inline bool tx_ring_is_empty(void) {
    return tx_ring_head == tx_ring_tail;
}

// -----------------------------------------------------------------------
// 8251 emulation state
// -----------------------------------------------------------------------
// The 8251 USART has a two-phase initialization:
//   1. After reset: first write to port $E9 is the Mode instruction
//   2. Subsequent writes to port $E9 are Command instructions
//   3. Reset sequence: write $00, $00, $00, $40 to port $E9
// We track this to properly handle the init sequence.

static volatile bool midi_tx_enabled;
static volatile bool midi_rx_enabled;

// -----------------------------------------------------------------------
// PIO1 context
// -----------------------------------------------------------------------
#define IO_PIO      pio1
#define IO_SM_READ  0
#define IO_SM_WRITE 1

// -----------------------------------------------------------------------
// Build a 16-bit response token for the I/O read PIO:
//   bits[7:0]  = data byte
//   bits[15:8] = pindirs (0xFF = drive, 0x00 = float)
// -----------------------------------------------------------------------
static inline uint16_t __not_in_flash_func(build_token)(bool drive, uint8_t data) {
    return drive ? ((uint16_t)0xFF00u | data) : (uint16_t)data;
}

// -----------------------------------------------------------------------
// PIO1 IRQ handler — services MSX I/O reads and writes for MIDI ports
// -----------------------------------------------------------------------
static void __not_in_flash_func(midi_pio1_irq_handler)(void) {
    // --- Handle I/O writes first ---
    while (!pio_sm_is_rx_fifo_empty(IO_PIO, IO_SM_WRITE)) {
        uint32_t sample = pio_sm_get(IO_PIO, IO_SM_WRITE);
        uint8_t port = (uint8_t)(sample & 0xFFu);
        uint8_t data = (uint8_t)((sample >> 16) & 0xFFu);

        switch (port) {
            case MIDI_PORT_DATA:
                // 8251 TX data — queue byte for USB MIDI transmission
                if (midi_tx_enabled) {
                    tx_ring_put(data);
                }
                break;

            case MIDI_PORT_STATUS:
                // 8251 command/mode register write
                // Bit 0: TEN (transmit enable)
                // Bit 2: RE (receive enable)
                // Bit 6: Internal reset (if set, next write is Mode instruction)
                if (data & 0x40u) {
                    // Internal reset — reinitialize
                    midi_tx_enabled = false;
                    midi_rx_enabled = false;
                } else {
                    midi_tx_enabled = (data & 0x01u) != 0;
                    midi_rx_enabled = (data & 0x04u) != 0;
                }
                break;

            case MIDI_PORT_TIMERACK:
            case MIDI_PORT_TIMERACK2:
            case MIDI_PORT_COUNTER0:
            case MIDI_PORT_COUNTER1:
            case MIDI_PORT_COUNTER2:
            case MIDI_PORT_TIMERCTRL:
                // 8253 timer writes — ignored (USB MIDI handles baud rate)
                break;

            default:
                break;
        }
    }

    // --- Handle I/O reads ---
    while (!pio_sm_is_rx_fifo_empty(IO_PIO, IO_SM_READ)) {
        uint16_t addr = (uint16_t)pio_sm_get(IO_PIO, IO_SM_READ);
        uint8_t port = (uint8_t)(addr & 0xFFu);

        switch (port) {
            case MIDI_PORT_DATA: {
                // 8251 RX data — return received MIDI byte or 0xFF
                uint8_t rx_byte;
                if (usb_midi_host_receive_byte(&rx_byte)) {
                    pio_sm_put(IO_PIO, IO_SM_READ, build_token(true, rx_byte));
                } else {
                    pio_sm_put(IO_PIO, IO_SM_READ, build_token(true, 0xFFu));
                }
                break;
            }

            case MIDI_PORT_STATUS: {
                // 8251 status register
                uint8_t status = 0;

                // TxRDY (bit 0): ready to accept data when buffer is not full
                if (!tx_ring_is_full()) {
                    status |= MIDI_STATUS_TXRDY;
                }

                // RRDY (bit 1): received data available
                if (usb_midi_host_rx_available()) {
                    status |= MIDI_STATUS_RRDY;
                }

                // TxEM (bit 2): transmitter empty (buffer fully drained)
                if (tx_ring_is_empty()) {
                    status |= MIDI_STATUS_TXEM;
                }

                pio_sm_put(IO_PIO, IO_SM_READ, build_token(true, status));
                break;
            }

            case MIDI_PORT_TIMERACK:
            case MIDI_PORT_TIMERACK2:
            case MIDI_PORT_COUNTER0:
            case MIDI_PORT_COUNTER1:
            case MIDI_PORT_COUNTER2:
            case MIDI_PORT_TIMERCTRL:
                // 8253 timer reads — return 0x00
                pio_sm_put(IO_PIO, IO_SM_READ, build_token(true, 0x00u));
                break;

            default:
                // Not our port — release with tri-state
                pio_sm_put(IO_PIO, IO_SM_READ, build_token(false, 0xFFu));
                break;
        }
    }
}

// -----------------------------------------------------------------------
// GPIO initialisation
// -----------------------------------------------------------------------
static void setup_gpio(void) {
    // Address bus A0-A15 as inputs
    for (uint pin = PIN_A0; pin <= PIN_A15; ++pin) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
    }

    // Data bus D0-D7 — managed by PIO later, start as input
    for (uint pin = PIN_D0; pin <= PIN_D7; ++pin) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
    }

    // Control signals as inputs
    gpio_init(PIN_RD);      gpio_set_dir(PIN_RD, GPIO_IN);
    gpio_init(PIN_WR);      gpio_set_dir(PIN_WR, GPIO_IN);
    gpio_init(PIN_IORQ);    gpio_set_dir(PIN_IORQ, GPIO_IN);
    gpio_init(PIN_SLTSL);   gpio_set_dir(PIN_SLTSL, GPIO_IN);
    gpio_init(PIN_BUSSDIR); gpio_set_dir(PIN_BUSSDIR, GPIO_IN);

    // /WAIT — open-drain: output latch low, released as input until PIO asserts it
    gpio_init(PIN_WAIT);
    gpio_put(PIN_WAIT, 0);
    gpio_set_dir(PIN_WAIT, GPIO_IN);
}

// -----------------------------------------------------------------------
// I/O bus PIO initialisation
// -----------------------------------------------------------------------
static void io_bus_init(void) {
    // Load PIO programs
    uint offset_read  = pio_add_program(IO_PIO, &msx_midi_io_read_program);
    uint offset_write = pio_add_program(IO_PIO, &msx_midi_io_write_program);

    // --- SM0: I/O read responder (with /WAIT) ---
    pio_sm_set_enabled(IO_PIO, IO_SM_READ, false);
    pio_sm_clear_fifos(IO_PIO, IO_SM_READ);
    pio_sm_restart(IO_PIO, IO_SM_READ);

    pio_sm_config cfg_r = msx_midi_io_read_program_get_default_config(offset_read);
    sm_config_set_in_pins(&cfg_r, PIN_A0);
    sm_config_set_in_shift(&cfg_r, false, false, 16);
    sm_config_set_out_pins(&cfg_r, PIN_D0, 8);
    sm_config_set_out_shift(&cfg_r, true, false, 32);
    sm_config_set_set_pins(&cfg_r, PIN_WAIT, 1);
    sm_config_set_jmp_pin(&cfg_r, PIN_RD);
    sm_config_set_clkdiv(&cfg_r, 1.0f);
    pio_sm_init(IO_PIO, IO_SM_READ, offset_read, &cfg_r);

    // Preload /WAIT low; the PIO releases it by switching direction to input.
    pio_sm_set_pins_with_mask(IO_PIO, IO_SM_READ, 0u, (1u << PIN_WAIT));

    // Hand /WAIT to PIO1
    pio_gpio_init(IO_PIO, PIN_WAIT);
    pio_sm_set_consecutive_pindirs(IO_PIO, IO_SM_READ, PIN_WAIT, 1, false);

    // Data bus D0-D7: hand to PIO1 for bidirectional control
    for (int i = PIN_D0; i <= PIN_D7; i++)
        pio_gpio_init(IO_PIO, i);
    pio_sm_set_consecutive_pindirs(IO_PIO, IO_SM_READ, PIN_D0, 8, false);

    // --- SM1: I/O write captor ---
    pio_sm_set_enabled(IO_PIO, IO_SM_WRITE, false);
    pio_sm_clear_fifos(IO_PIO, IO_SM_WRITE);
    pio_sm_restart(IO_PIO, IO_SM_WRITE);

    pio_sm_config cfg_w = msx_midi_io_write_program_get_default_config(offset_write);
    sm_config_set_in_pins(&cfg_w, PIN_A0);
    sm_config_set_in_shift(&cfg_w, false, false, 32);
    sm_config_set_fifo_join(&cfg_w, PIO_FIFO_JOIN_RX);
    sm_config_set_jmp_pin(&cfg_w, PIN_WR);
    sm_config_set_clkdiv(&cfg_w, 1.0f);
    pio_sm_init(IO_PIO, IO_SM_WRITE, offset_write, &cfg_w);

    // --- Enable both state machines ---
    pio_sm_set_enabled(IO_PIO, IO_SM_READ,  true);
    pio_sm_set_enabled(IO_PIO, IO_SM_WRITE, true);

    // --- Route SM0+SM1 RX-not-empty to PIO1_IRQ_0 → Core 0 ---
    pio_set_irq0_source_enabled(IO_PIO, pis_sm0_rx_fifo_not_empty, true);
    pio_set_irq0_source_enabled(IO_PIO, pis_sm1_rx_fifo_not_empty, true);
    irq_set_exclusive_handler(PIO1_IRQ_0, midi_pio1_irq_handler);
    irq_set_priority(PIO1_IRQ_0, 0);  // Highest priority
    irq_set_enabled(PIO1_IRQ_0, true);
}

// -----------------------------------------------------------------------
// Core 1 — TinyUSB host task + MIDI TX processing
// -----------------------------------------------------------------------
static void core1_entry(void) {
    tusb_init();
    tuh_init(0);

    absolute_time_t next_midi_tx_tick = get_absolute_time();

    while (true) {
        tuh_task();
        absolute_time_t now = get_absolute_time();

        // Feed raw MIDI bytes at standard DIN MIDI wire speed.
        if (usb_midi_host_mounted() && absolute_time_diff_us(next_midi_tx_tick, now) >= 0) {
            uint8_t byte;
            if (usb_midi_host_can_accept_byte() && tx_ring_get(&byte)) {
                usb_midi_host_send_byte(byte);
                next_midi_tx_tick = delayed_by_us(now, MIDI_WIRE_BYTE_US);
            } else {
                next_midi_tx_tick = now;
            }
            usb_midi_host_flush();
        }
    }
}

// -----------------------------------------------------------------------
// Main — Core 0
// -----------------------------------------------------------------------
int main(void) {
    set_sys_clock_khz(230000, true);

    // Initialize state
    tx_ring_head = 0;
    tx_ring_tail = 0;
    midi_tx_enabled = false;
    midi_rx_enabled = false;

    setup_gpio();
    io_bus_init();

    multicore_launch_core1(core1_entry);

    while (true)
        __wfi();

    return 0;
}
