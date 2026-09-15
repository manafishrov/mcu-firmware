#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef SPI_PROBE_TEST
#include "tests/mock_sdk.h"
#else
#include "probe_build.h"
#include <class/cdc/cdc_device.h>
#include <hardware/gpio.h>
#include <hardware/regs/io_bank0.h>
#include <hardware/regs/spi.h>
#include <hardware/spi.h>
#include <hardware/structs/io_bank0.h>
#include <hardware/sync.h>
#include <hardware/timer.h>
#include <hardware/watchdog.h>
#include <pico/multicore.h>
#include <pico/platform.h>
#include <pico/stdio_usb.h>
#include <pico/time.h>
#include <pico/types.h>
#endif

#define PIN_SCK 10u
#define PIN_MOSI 11u
#define PIN_MISO 12u
#define PIN_CS 13u
#define MOTOR_MASK ((0x0fu << 6u) | (0x0fu << 18u))
#define BYTEWISE_TIMEOUT_US 500u
#define REQUEST_TIMEOUT_US 2000u
#define RUN_TIMEOUT_US 1000000u
#define USB_TIMEOUT_US 5000000u
#define REPEATS 3u
#define BIT_HALF_US 10u

typedef enum { METHOD_BYTEWISE, METHOD_SDK, METHOD_SIO } method_t;
typedef enum { PULL_DOWN, PULL_UP } pull_t;
typedef struct {
    const char *name;
    method_t method;
    uint32_t baud;
    uint8_t mode;
    pull_t pull;
} variant_t;

static const variant_t variants[] = {
    {"current", METHOD_BYTEWISE, 4000000, 0, PULL_DOWN},
    {"sdk", METHOD_SDK, 4000000, 0, PULL_DOWN},
    {"current_100k", METHOD_BYTEWISE, 100000, 0, PULL_DOWN},
    {"sdk_100k", METHOD_SDK, 100000, 0, PULL_DOWN},
    {"current_1m", METHOD_BYTEWISE, 1000000, 0, PULL_DOWN},
    {"sdk_1m", METHOD_SDK, 1000000, 0, PULL_DOWN},
    {"current_mode3", METHOD_BYTEWISE, 4000000, 3, PULL_DOWN},
    {"sdk_mode3", METHOD_SDK, 4000000, 3, PULL_DOWN},
    {"sio_mode0", METHOD_SIO, 50000, 0, PULL_DOWN},
#if SPI_PROBE_COMPARE_PULLS
    {"weak_pull_up", METHOD_SDK, 4000000, 0, PULL_UP},
    {"weak_pull_down_repeat", METHOD_SDK, 4000000, 0, PULL_DOWN},
#endif
};
#define VARIANT_COUNT (sizeof(variants) / sizeof(variants[0]))

typedef struct {
    uint8_t rx[3];
    uint8_t mux[4];
    uint8_t pad_before, pad_after;
    uint32_t miso_status_before, miso_status_after;
    uint32_t actual_baud, duration_us, cs_high_us, motor_levels;
    bool complete;
} observation_t;
static observation_t observations[VARIANT_COUNT][REPEATS][2];
static size_t observation_count;
static bool run_complete;
static uint64_t last_cs_high;
static bool peripheral_initialized;

/* Only the SPI worker accesses the SPI FIFO while a request is outstanding.
 * Core 0 owns CS/pinmux and can make the pins idle even if the unmodified SDK
 * blocking call hangs. No reset/watchdog action is used to recover a timeout.
 */
static uint32_t request_sequence, complete_sequence, worker_stopped;
static method_t requested_method;
static uint8_t worker_rx[3];
static bool worker_success;
static const uint8_t read_command[3] = {0x80, 0x00, 0x00};

static uint32_t load_shared(const uint32_t *value) {
    return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}
static void store_shared(uint32_t *value, uint32_t next) {
    __atomic_store_n(value, next, __ATOMIC_RELEASE);
}

/* Same FIFO register algorithm as src/imu/bmi270_sensor.c at b817d1d;
 * unlike the production wrapper, retain the command-phase RX byte as well.
 */
static bool current_exchange_byte(uint8_t output, uint8_t *input, uint64_t deadline) {
    while (!spi_is_writable(spi1)) {
        if (time_us_64() >= deadline) {
            return false;
        }
    }
    if (time_us_64() >= deadline) {
        return false;
    }
    spi_get_hw(spi1)->dr = output;
    while (!spi_is_readable(spi1)) {
        if (time_us_64() >= deadline) {
            return false;
        }
    }
    *input = (uint8_t)spi_get_hw(spi1)->dr;
    return true;
}

static bool current_transfer(uint8_t rx[3]) {
    const uint64_t deadline = time_us_64() + BYTEWISE_TIMEOUT_US;
    for (size_t i = 0; i < sizeof(read_command); ++i) {
        if (!current_exchange_byte(read_command[i], &rx[i], deadline)) {
            return false;
        }
    }
    while (spi_is_busy(spi1)) {
        if (time_us_64() >= deadline) {
            return false;
        }
    }
    return time_us_64() < deadline;
}

static bool service_transfer_request(void) {
    const uint32_t sequence = load_shared(&request_sequence);
    if (load_shared(&worker_stopped) || sequence == load_shared(&complete_sequence)) {
        return false;
    }
    memset(worker_rx, 0xee, sizeof(worker_rx));
    if (requested_method == METHOD_BYTEWISE) {
        worker_success = current_transfer(worker_rx);
    } else {
        worker_success = spi_write_read_blocking(spi1, read_command, worker_rx, 3) == 3;
    }
    store_shared(&complete_sequence, sequence);
    return true;
}

static void spi_worker(void) {
    for (;;) {
        (void)service_transfer_request();
        __wfe();
    }
}

static void motor_low_forever(void) {
    const uint8_t pins[8] = {6, 7, 8, 9, 18, 19, 20, 21};
    for (size_t i = 0; i < sizeof(pins); ++i) {
        gpio_init(pins[i]);
        gpio_put(pins[i], false);
        gpio_set_outover(pins[i], GPIO_OVERRIDE_LOW);
        gpio_set_dir(pins[i], GPIO_OUT);
    }
}

static void idle_spi_pins(void) {
    if (peripheral_initialized) {
        spi_get_hw(spi1)->cr1 &= ~SPI_SSPCR1_SSE_BITS;
    }
    /* Prepare idle latches before switching away from the peripheral. */
    gpio_put(PIN_CS, true);
    gpio_put(PIN_SCK, false);
    gpio_put(PIN_MOSI, false);
    gpio_set_function(PIN_CS, GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SIO);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SIO);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_set_dir(PIN_SCK, GPIO_OUT);
    gpio_set_dir(PIN_MOSI, GPIO_OUT);
    gpio_set_dir(PIN_MISO, GPIO_IN);
    last_cs_high = time_us_64();
}

static uint32_t configure_variant(const variant_t *variant) {
    idle_spi_pins();
    if (variant->pull == PULL_UP) {
        gpio_pull_up(PIN_MISO);
    } else {
        gpio_pull_down(PIN_MISO);
    }
    uint32_t actual = 0;
    if (variant->method != METHOD_SIO) {
        (void)spi_init(spi1, variant->baud);
        peripheral_initialized = true;
        spi_set_format(spi1, 8, variant->mode == 3 ? SPI_CPOL_1 : SPI_CPOL_0,
                       variant->mode == 3 ? SPI_CPHA_1 : SPI_CPHA_0, SPI_MSB_FIRST);
        gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
        gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
        gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
        actual = spi_get_baudrate(spi1);
    }
    busy_wait_us_32(500); /* Includes time for CS-rising SPI selection and pull settling. */
    return actual;
}

static uint8_t read_pads(void) {
    uint8_t levels = 0;
    for (uint pin = PIN_SCK; pin <= PIN_CS; ++pin) {
        if (io_bank0_hw->io[pin].status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS) {
            levels |= (uint8_t)(1u << (pin - PIN_SCK));
        }
    }
    return levels;
}

static void sio_transfer(uint8_t rx[3]) {
    for (size_t byte = 0; byte < sizeof(read_command); ++byte) {
        uint8_t input = 0;
        for (unsigned bit = 0; bit < 8; ++bit) {
            gpio_put(PIN_MOSI, (read_command[byte] & (0x80u >> bit)) != 0);
            busy_wait_us_32(BIT_HALF_US);
            gpio_put(PIN_SCK, true);
            busy_wait_us_32(BIT_HALF_US);
            input = (uint8_t)((input << 1u) | (gpio_get(PIN_MISO) ? 1u : 0u));
            gpio_put(PIN_SCK, false);
        }
        rx[byte] = input;
    }
}

static bool transfer_once(method_t method, observation_t *result) {
    memset(result->rx, 0xee, sizeof(result->rx)); /* Timeout sentinel, NOT received bytes. */
    for (uint pin = PIN_SCK; pin <= PIN_CS; ++pin) {
        result->mux[pin - PIN_SCK] = (uint8_t)gpio_get_function(pin);
    }
    result->pad_before = read_pads();
    result->miso_status_before = io_bank0_hw->io[PIN_MISO].status;
    result->cs_high_us = (uint32_t)(time_us_64() - last_cs_high);
    gpio_put(PIN_CS, false);
    busy_wait_us_32(1);
    const uint64_t start = time_us_64();
    bool complete = true;
    if (method == METHOD_SIO) {
        sio_transfer(result->rx);
    } else {
        requested_method = method;
        const uint32_t next = load_shared(&request_sequence) + 1u;
        store_shared(&request_sequence, next);
        __sev();
        while (load_shared(&complete_sequence) != next) {
            if (time_us_64() - start >= REQUEST_TIMEOUT_US) {
                store_shared(&worker_stopped, 1);
                complete = false;
                break;
            }
            tight_loop_contents();
        }
        if (complete) {
            memcpy(result->rx, worker_rx, sizeof(result->rx));
            complete = worker_success;
        }
    }
    result->duration_us = (uint32_t)(time_us_64() - start);
    if (!complete && peripheral_initialized) {
        spi_get_hw(spi1)->cr1 &= ~SPI_SSPCR1_SSE_BITS;
    }
    busy_wait_us_32(1);
    gpio_put(PIN_CS, true);
    last_cs_high = time_us_64();
    result->pad_after = read_pads();
    result->miso_status_after = io_bank0_hw->io[PIN_MISO].status;
    result->motor_levels = gpio_get_all() & MOTOR_MASK;
    result->complete = complete;
    return complete;
}

static void run_probe(void) {
    const uint64_t deadline = time_us_64() + RUN_TIMEOUT_US;
    run_complete = false;
    for (size_t v = 0; v < VARIANT_COUNT; ++v) {
        const uint32_t actual = configure_variant(&variants[v]);
        for (unsigned repeat = 0; repeat < REPEATS; ++repeat) {
            for (unsigned phase = 0; phase < 2; ++phase) {
                if (time_us_64() >= deadline) {
                    goto finished;
                }
                observation_t *result = &observations[v][repeat][phase];
                result->actual_baud = actual;
                observation_count++;
                if (!transfer_once(variants[v].method, result)) {
                    goto finished;
                }
                busy_wait_us_32(500); /* At least 450 us CS-high between EVERY burst. */
            }
        }
    }
    run_complete = true;
finished:
    store_shared(&worker_stopped, 1);
    idle_spi_pins();
    gpio_disable_pulls(PIN_MISO);
}

static bool usb_connected(void) {
    const uint32_t irq = save_and_disable_interrupts();
    const bool connected = tud_cdc_connected(); /* Includes DTR, not merely enumeration. */
    restore_interrupts(irq);
    return connected;
}

static bool send_line(const char *line, uint64_t deadline) {
    const size_t length = strlen(line);
    size_t offset = 0;
    while (offset < length) {
        if (time_us_64() >= deadline) {
            return false;
        }
        const uint32_t irq = save_and_disable_interrupts();
        const bool connected = tud_cdc_connected();
        uint32_t accepted = 0;
        if (connected) {
            uint32_t count = tud_cdc_write_available();
            if (count > length - offset) {
                count = (uint32_t)(length - offset);
            }
            if (count > 64u) {
                count = 64u;
            }
            if (count != 0) {
                accepted = tud_cdc_write(line + offset, count);
                (void)tud_cdc_write_flush();
            }
        }
        restore_interrupts(irq);
        if (!connected) {
            return false;
        }
        offset += accepted;
        if (accepted == 0) {
            sleep_us(1000);
        }
    }
    return true;
}

static void report_probe(void) {
    const uint64_t deadline = time_us_64() + USB_TIMEOUT_US;
    char line[512];
    (void)snprintf(
        line, sizeof(line),
        "SPI_PROBE diagnostic-1 source=%s records=%u complete=%u pulls=%u\n"
        "READ_ONLY tx=80,00,00; select phase discarded; id is rx[2] on id phase\n"
        "pad bits: SCK10=0 MOSI11=1 MISO12=2 CS13=3; pad snapshots are idle, NOT waveforms\n"
        "SIO actual_baud=0: nominal50k only; hardware actual_baud is SDK divider readback\n",
        SPI_PROBE_SOURCE_SHA256, (unsigned)observation_count, run_complete,
        SPI_PROBE_COMPARE_PULLS);
    if (!send_line(line, deadline)) {
        return;
    }
    size_t index = 0;
    for (size_t v = 0; v < VARIANT_COUNT; ++v) {
        for (unsigned repeat = 0; repeat < REPEATS; ++repeat) {
            for (unsigned phase = 0; phase < 2; ++phase) {
                if (index++ >= observation_count) {
                    goto report_end;
                }
                const observation_t *r = &observations[v][repeat][phase];
                (void)snprintf(
                    line, sizeof(line),
                    "%s repeat=%u phase=%s method=%u requested=%lu actual_baud=%lu mode=%u "
                    "pull=%s rx=%02x,%02x,%02x complete=%u us=%lu high_us=%lu "
                    "mux=%u,%u,%u,%u pads=%x/%x miso_status=%08lx/%08lx motors=%08lx\n",
                    variants[v].name, repeat, phase == 0 ? "select" : "id",
                    (unsigned)variants[v].method, (unsigned long)variants[v].baud,
                    (unsigned long)r->actual_baud, variants[v].mode,
                    variants[v].pull == PULL_UP ? "weak_up" : "weak_down", r->rx[0], r->rx[1],
                    r->rx[2], r->complete, (unsigned long)r->duration_us,
                    (unsigned long)r->cs_high_us, r->mux[0], r->mux[1], r->mux[2], r->mux[3],
                    r->pad_before, r->pad_after, (unsigned long)r->miso_status_before,
                    (unsigned long)r->miso_status_after, (unsigned long)r->motor_levels);
                if (!send_line(line, deadline)) {
                    return;
                }
            }
        }
    }
report_end:
    (void)send_line(
        "END_SPI_PROBE CS=high SCK=MOSI=low RX=input motors=low; no rerun or automatic reset\n",
        deadline);
}

int main(void) {
    motor_low_forever();
    watchdog_disable();
    idle_spi_pins();
    gpio_disable_pulls(PIN_MISO);
    if (!stdio_usb_init()) {
        for (;;) {
            tight_loop_contents();
        }
    }
    multicore_launch_core1(spi_worker);
    while (!usb_connected()) {
        sleep_ms(10);
    }
    run_probe();
    report_probe();
    for (;;) {
        sleep_ms(100); /* Keep the SDK USB reset interface alive; never run again. */
    }
}
