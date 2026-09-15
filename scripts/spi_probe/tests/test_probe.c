/* Offline pin-permission/command-trace test of the actual diagnostic source. */
#include <assert.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define SPI_PROBE_TEST 1
#ifndef SPI_PROBE_COMPARE_PULLS
#define SPI_PROBE_COMPARE_PULLS 1
#endif
#define main probe_main
#include "../probe.c"
#undef main

static spi_hw_t fake_spi;
spi_hw_t *const spi1 = &fake_spi;
static mock_io_bank_t fake_bank;
mock_io_bank_t *const io_bank0_hw = &fake_bank;
static bool outputs[30], latches[30], forced_low[30];
static uint functions[30];
static uint32_t clock_baud, mode;
static pull_t current_pull;
static bool input_bit;
static uint64_t clock_us;
static uint8_t frame[3];
static size_t frame_bytes, bit_count, frame_count;
static unsigned sdk_calls, watchdog_disables;
static bool pumping, hold_worker, stall_rx, follows_pull;
static bool connected, blocked_usb, irq_enabled, running_main;
static unsigned wait_count, idle_count;
static char transcript[32768];
static size_t transcript_length;
static jmp_buf main_done;

static bool motor(uint pin) {
    return (pin >= 6 && pin <= 9) || (pin >= 18 && pin <= 21);
}
static bool permitted(uint pin) {
    return motor(pin) || (pin >= 10 && pin <= 13);
}
static void assert_idle(void) {
    for (uint pin = 0; pin < 30; ++pin) {
        if (motor(pin)) {
            assert(outputs[pin] && forced_low[pin] && !latches[pin]);
        } else if (pin != PIN_SCK && pin != PIN_MOSI && pin != PIN_CS) {
            assert(!outputs[pin]);
        }
    }
    assert(latches[PIN_CS] && !latches[PIN_SCK] && !latches[PIN_MOSI]);
    assert(functions[PIN_MISO] == GPIO_FUNC_SIO && !outputs[PIN_MISO]);
}
static void pump_worker(void) {
    if (!pumping && !hold_worker) {
        pumping = true;
        (void)service_transfer_request();
        pumping = false;
    }
}
static void refresh_pads(void) {
    for (uint pin = 0; pin < 30; ++pin) {
        if (permitted(pin)) {
            fake_bank.io[pin].status = gpio_get_pad(pin) ? IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS : 0;
        }
    }
}
uint64_t time_us_64(void) {
    clock_us++;
    pump_worker();
    refresh_pads();
    return clock_us;
}
void busy_wait_us_32(uint32_t us) {
    clock_us += us;
}
void sleep_us(uint64_t us) {
    clock_us += us;
}
void sleep_ms(uint32_t ms) {
    clock_us += (uint64_t)ms * 1000;
    if (running_main && ms == 10) {
        assert(frame_count == 0);
        assert_idle();
        if (++wait_count == 3) {
            connected = true;
        }
    }
    if (running_main && ms == 100) {
        assert_idle();
        assert(frame_count == VARIANT_COUNT * REPEATS * 2);
        /* Disconnect/reconnect must not start another experiment. */
        connected = ++idle_count != 1;
        if (idle_count == 3) {
            longjmp(main_done, 1);
        }
    }
}
void tight_loop_contents(void) {
    pump_worker();
}
void __wfe(void) {
    pump_worker();
}
void __sev(void) {
    pump_worker();
}
void gpio_init(uint pin) {
    assert(permitted(pin));
    outputs[pin] = false;
    latches[pin] = false;
    functions[pin] = GPIO_FUNC_SIO;
}
void gpio_put(uint pin, bool value) {
    assert(permitted(pin) && pin != PIN_MISO);
    if (motor(pin)) {
        assert(!value);
    }
    if (pin == PIN_CS && !value) {
        assert(connected && latches[pin]);
        frame_bytes = bit_count = 0;
        memset(frame, 0, sizeof(frame));
    }
    if (pin == PIN_CS && value && !latches[pin] && outputs[pin]) {
        if (!load_shared(&worker_stopped) &&
            (functions[PIN_SCK] == GPIO_FUNC_SIO || (fake_spi.cr1 & SPI_SSPCR1_SSE_BITS))) {
            assert(frame_bytes == 3 && memcmp(frame, read_command, 3) == 0);
        }
        frame_count++;
    }
    if (pin == PIN_SCK && value && !latches[pin] && !latches[PIN_CS] &&
        functions[PIN_SCK] == GPIO_FUNC_SIO) {
        assert(bit_count < 24);
        frame[bit_count / 8] = (uint8_t)((frame[bit_count / 8] << 1u) | latches[PIN_MOSI]);
        const uint8_t reply[3] = {0xa5, 0x5a, 0x24};
        input_bit = (reply[bit_count / 8] & (0x80u >> (bit_count % 8))) != 0;
        bit_count++;
        frame_bytes = bit_count / 8;
    }
    latches[pin] = value;
}
void gpio_set_dir(uint pin, bool output) {
    assert(permitted(pin) && !(pin == PIN_MISO && output));
    if (motor(pin) && output) {
        assert(!latches[pin] && forced_low[pin]);
    }
    outputs[pin] = output;
}
void gpio_set_function(uint pin, uint function) {
    assert(permitted(pin));
    assert(function == GPIO_FUNC_SIO ||
           (function == GPIO_FUNC_SPI && pin >= PIN_SCK && pin <= PIN_MISO));
    functions[pin] = function;
}
uint gpio_get_function(uint pin) {
    refresh_pads();
    return functions[pin];
}
void gpio_set_outover(uint pin, uint value) {
    assert(motor(pin) && value == GPIO_OVERRIDE_LOW);
    forced_low[pin] = true;
}
void gpio_pull_down(uint pin) {
    assert(pin == PIN_MISO);
    current_pull = PULL_DOWN;
}
void gpio_pull_up(uint pin) {
    assert(pin == PIN_MISO);
    current_pull = PULL_UP;
}
void gpio_disable_pulls(uint pin) {
    assert(pin == PIN_MISO);
}
bool gpio_get_pad(uint pin) {
    assert(permitted(pin));
    if (pin == PIN_MISO) {
        bool high = current_pull == PULL_UP;
        fake_bank.io[pin].status = high ? (1u << 17u) : 0;
        return high;
    }
    if (pin == PIN_SCK && functions[pin] == GPIO_FUNC_SPI) {
        return mode == 3;
    }
    return forced_low[pin] ? false : latches[pin];
}
bool gpio_get(uint pin) {
    assert(pin == PIN_MISO && !outputs[pin] && !latches[PIN_CS]);
    return input_bit;
}
uint32_t gpio_get_all(void) {
    uint32_t result = 0;
    for (uint pin = 0; pin < 30; ++pin) {
        if (permitted(pin) && gpio_get_pad(pin)) {
            result |= 1u << pin;
        }
    }
    return result;
}
uint spi_init(spi_hw_t *spi, uint baud) {
    assert(spi == spi1 && latches[PIN_CS]);
    assert(baud == 4000000 || baud == 1000000 || baud == 100000);
    fake_spi.cr1 = SPI_SSPCR1_SSE_BITS;
    clock_baud = baud;
    return baud;
}
void spi_set_format(spi_hw_t *spi, uint bits, uint polarity, uint phase, uint order) {
    assert(spi == spi1 && bits == 8 && order == SPI_MSB_FIRST && polarity == phase);
    assert(polarity == 0 || polarity == 1);
    mode = polarity == 1 ? 3 : 0;
}
uint spi_get_baudrate(spi_hw_t *spi) {
    assert(spi == spi1);
    return clock_baud;
}
spi_hw_t *spi_get_hw(spi_hw_t *spi) {
    assert(spi == spi1);
    return spi;
}
bool spi_is_writable(spi_hw_t *spi) {
    assert(spi == spi1);
    return true;
}
static uint8_t transfer_byte(uint8_t output) {
    assert(!latches[PIN_CS] && frame_bytes < 3 && connected);
    assert(functions[PIN_SCK] == GPIO_FUNC_SPI && functions[PIN_MISO] == GPIO_FUNC_SPI);
    assert(output == read_command[frame_bytes]); /* No write-register command can pass. */
    frame[frame_bytes] = output;
    const uint8_t reply[3] = {0xa5, 0x5a, 0x24};
    uint8_t input = follows_pull ? (current_pull == PULL_UP ? 0xff : 0) : reply[frame_bytes];
    frame_bytes++;
    clock_us += 8u * 1000000u / clock_baud;
    return input;
}
bool spi_is_readable(spi_hw_t *spi) {
    assert(spi == spi1);
    if (stall_rx) {
        return false;
    }
    fake_spi.dr = transfer_byte((uint8_t)fake_spi.dr);
    return true;
}
bool spi_is_busy(spi_hw_t *spi) {
    assert(spi == spi1);
    return false;
}
int spi_write_read_blocking(spi_hw_t *spi, const uint8_t *tx, uint8_t *rx, size_t length) {
    assert(spi == spi1 && length == 3 && memcmp(tx, read_command, 3) == 0);
    sdk_calls++;
    for (size_t i = 0; i < length; ++i) {
        rx[i] = transfer_byte(tx[i]);
    }
    return 3;
}
void multicore_launch_core1(void (*entry)(void)) {
    assert(entry == spi_worker);
}
void watchdog_disable(void) {
    watchdog_disables++;
}
bool stdio_usb_init(void) {
    assert_idle();
    return true;
}
uint32_t save_and_disable_interrupts(void) {
    uint32_t previous = irq_enabled ? 0u : 1u;
    irq_enabled = false;
    return previous;
}
void restore_interrupts(uint32_t state) {
    irq_enabled = state == 0;
}
bool tud_cdc_connected(void) {
    assert(!irq_enabled);
    return connected;
}
uint32_t tud_cdc_write_available(void) {
    assert(!irq_enabled);
    return blocked_usb ? 0 : 64;
}
uint32_t tud_cdc_write(const void *data, uint32_t length) {
    assert(!irq_enabled && connected && length <= 64);
    assert(transcript_length + length < sizeof(transcript));
    memcpy(transcript + transcript_length, data, length);
    transcript_length += length;
    transcript[transcript_length] = '\0';
    return length;
}
uint32_t tud_cdc_write_flush(void) {
    return 0;
}

static void reset_fixture(void) {
    memset(outputs, 0, sizeof(outputs));
    memset(latches, 0, sizeof(latches));
    memset(forced_low, 0, sizeof(forced_low));
    memset(functions, 0, sizeof(functions));
    memset(observations, 0, sizeof(observations));
    fake_spi = (spi_hw_t){0};
    fake_bank = (mock_io_bank_t){0};
    request_sequence = complete_sequence = worker_stopped = 0;
    observation_count = 0;
    peripheral_initialized = run_complete = false;
    clock_us = last_cs_high = 0;
    frame_count = frame_bytes = bit_count = 0;
    sdk_calls = watchdog_disables = wait_count = idle_count = 0;
    pumping = hold_worker = stall_rx = follows_pull = running_main = blocked_usb = false;
    irq_enabled = connected = true;
    current_pull = PULL_DOWN;
    transcript_length = 0;
    transcript[0] = '\0';
}

static void test_full_run(void) {
    reset_fixture();
    connected = false;
    running_main = true;
    if (setjmp(main_done) == 0) {
        (void)probe_main();
        assert(!"Main must remain idle instead of returning/resetting");
    }
    running_main = false;
    assert(run_complete && observation_count == VARIANT_COUNT * REPEATS * 2);
    assert(watchdog_disables == 1 && wait_count == 3 && idle_count == 3);
    assert(sdk_calls >= 24 && strstr(transcript, "END_SPI_PROBE") != NULL);
    for (size_t v = 0; v < VARIANT_COUNT; ++v) {
        for (size_t repeat = 0; repeat < REPEATS; ++repeat) {
            for (size_t phase = 0; phase < 2; ++phase) {
                const observation_t *r = &observations[v][repeat][phase];
                const uint8_t expected[3] = {0xa5, 0x5a, 0x24};
                assert(r->complete && memcmp(r->rx, expected, 3) == 0);
                assert(r->motor_levels == 0 && r->cs_high_us >= 450);
                assert(r->mux[3] == GPIO_FUNC_SIO);
                assert(((r->pad_before >> 2u) & 1u) == ((r->miso_status_before >> 17u) & 1u));
            }
        }
    }
    assert_idle();
}

static void test_timeout_cleanup(void) {
    for (unsigned fault = 0; fault < 2; ++fault) {
        reset_fixture();
        motor_low_forever();
        hold_worker = fault == 0;
        stall_rx = fault == 1;
        run_probe();
        assert(!run_complete && observation_count == 1);
        assert(!observations[0][0][0].complete && clock_us < 5000);
        assert(load_shared(&worker_stopped) && !(fake_spi.cr1 & SPI_SSPCR1_SSE_BITS));
        assert_idle();
        report_probe();
        assert(strstr(transcript, "complete=0") != NULL);
    }
    reset_fixture();
    motor_low_forever();
    run_probe();
    blocked_usb = true;
    uint64_t start = clock_us;
    report_probe();
    assert(clock_us - start <= USB_TIMEOUT_US + 1100u);
    assert(irq_enabled && transcript_length == 0);
    assert_idle();
}

static void test_pull_results_are_raw(void) {
    reset_fixture();
    motor_low_forever();
    follows_pull = true;
    run_probe();
    assert(run_complete); /* Complete means transfer completed, NOT healthy sensor. */
    for (size_t v = 0; v < VARIANT_COUNT; ++v) {
        if (variants[v].method == METHOD_SIO) {
            continue;
        }
        uint8_t expected = variants[v].pull == PULL_UP ? 0xff : 0;
        assert(observations[v][0][1].rx[2] == expected);
    }
    assert_idle();
}

int main(void) {
    test_full_run();
    test_timeout_cleanup();
    test_pull_results_are_raw();
    puts("SPI probe: read-only traces, pin allowlist, motor-low, DTR, bounded abort/USB and raw "
         "pulls pass");
    return 0;
}
