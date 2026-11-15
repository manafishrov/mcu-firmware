/* Standalone test of the real AM32 bank flasher against simulated ESC
 * bootloaders on a virtual one-wire UART, without the Pico SDK:
 * cc -std=c11 -Wall -Wextra -Werror -DAM32_BOOTLOADER_TEST \
 *   -Itests/am32_bootloader_mocks -Isrc tests/test_am32_bootloader.c \
 *   -o /tmp/test_am32_bootloader
 * /tmp/test_am32_bootloader
 * The guard keeps this translation unit inert in the Unity wildcard build.
 */
#ifdef AM32_BOOTLOADER_TEST
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "am32/bootloader.c"

#define PIN_COUNT 30
#define TEST_IMAGE_SIZE 600u
#define UART_BYTE_US (10u * AM32_BIT_TIME_US)
#define TURNAROUND_US 20u
#define MAX_STATUSES 256

typedef enum { ESC_ABSENT, ESC_PRESENT, ESC_WRONG_TARGET } esc_kind_t;

typedef struct {
    esc_kind_t kind;
    int rx_bit; /* -1 idle, 0..7 data bits, 8 stop bit */
    uint8_t rx_value;
    uint8_t packet[AM32_PROGRAM_CHUNK_SIZE + 2];
    size_t packet_length;
    uint8_t response[AM32_PROGRAM_CHUNK_SIZE + 3];
    size_t response_length;
    uint64_t response_start;
    bool expect_payload;
    uint16_t buffer_length;
    uint8_t buffer[AM32_PROGRAM_CHUNK_SIZE];
    uint16_t address;
    uint8_t flash[TEST_IMAGE_SIZE];
    unsigned writes;
    bool running;
} sim_esc_t;

typedef struct {
    esc_firmware_update_status_t status;
    uint8_t motor;
    esc_firmware_update_error_t error;
    uint32_t value;
} status_record_t;

static uint64_t now_us;
static bool pin_output[PIN_COUNT];
static sim_esc_t escs[PIN_COUNT];
static status_record_t statuses[MAX_STATUSES];
static size_t status_count;

void esc_firmware_update_send_status(esc_firmware_update_status_t status, uint8_t motor,
                                     esc_firmware_update_error_t error, uint32_t value) {
    assert(status_count < MAX_STATUSES);
    statuses[status_count++] = (status_record_t){status, motor, error, value};
}

uint32_t save_and_disable_interrupts(void) {
    return 0;
}
void restore_interrupts(uint32_t state) {
    (void)state;
}
/* Every clock read advances time so the flasher's polling loops terminate. */
uint32_t time_us_32(void) {
    return (uint32_t)now_us++;
}
void busy_wait_us_32(uint32_t delay_us) {
    now_us += delay_us;
}
void sleep_ms(uint32_t ms) {
    now_us += (uint64_t)ms * 1000u;
}
void gpio_pull_up(uint pin) {
    (void)pin;
}
void gpio_disable_pulls(uint pin) {
    (void)pin;
}

static void respond(sim_esc_t *esc, const uint8_t *data, size_t length) {
    memcpy(esc->response, data, length);
    esc->response_length = length;
    esc->response_start = now_us + TURNAROUND_US;
}

static void respond_ack(sim_esc_t *esc) {
    const uint8_t ack = AM32_ACK;
    respond(esc, &ack, 1);
}

static bool packet_crc_ok(const sim_esc_t *esc) {
    size_t payload = esc->packet_length - 2;
    uint16_t crc = crc16(esc->packet, payload);
    return esc->packet[payload] == (uint8_t)crc && esc->packet[payload + 1] == (uint8_t)(crc >> 8);
}

static void process_packet(sim_esc_t *esc) {
    const uint8_t *packet = esc->packet;
    if (esc->packet_length == 17) {
        assert(packet[8] == 13 && packet[9] == 66 && packet[16] == 0x7D);
        uint8_t info[AM32_DEVICE_INFO_SIZE] = {
            '4', '7', '1', AM32_EXPECTED_PIN_CODE, AM32_EXPECTED_FLASH_SIZE, 0, 0, 1, AM32_ACK,
        };
        if (esc->kind == ESC_WRONG_TARGET) {
            info[3] = AM32_EXPECTED_PIN_CODE + 1;
        }
        respond(esc, info, sizeof(info));
        return;
    }
    assert(esc->packet_length >= 4 && packet_crc_ok(esc));
    if (esc->expect_payload) {
        assert(esc->packet_length == (size_t)esc->buffer_length + 2);
        memcpy(esc->buffer, packet, esc->buffer_length);
        esc->expect_payload = false;
        respond_ack(esc);
        return;
    }
    switch (packet[0]) {
    case AM32_COMMAND_SET_ADDRESS:
        esc->address = (uint16_t)((packet[2] << 8) | packet[3]);
        respond_ack(esc);
        break;
    case AM32_COMMAND_SET_BUFFER:
        esc->buffer_length = packet[2] == 1 ? AM32_PROGRAM_CHUNK_SIZE : packet[3];
        esc->expect_payload = true;
        break;
    case AM32_COMMAND_PROGRAM_FLASH: {
        size_t offset = (size_t)esc->address - 0x1000u;
        assert(offset + esc->buffer_length <= TEST_IMAGE_SIZE);
        memcpy(&esc->flash[offset], esc->buffer, esc->buffer_length);
        esc->writes++;
        respond_ack(esc);
        break;
    }
    case AM32_COMMAND_READ_FLASH: {
        size_t length = packet[1] == 0 ? AM32_PROGRAM_CHUNK_SIZE : packet[1];
        size_t offset = (size_t)esc->address - 0x1000u;
        uint8_t response[AM32_PROGRAM_CHUNK_SIZE + 3];
        memcpy(response, &esc->flash[offset], length);
        append_crc(response, length);
        response[length + 2] = AM32_ACK;
        respond(esc, response, length + 3);
        break;
    }
    case AM32_COMMAND_RUN:
        esc->running = true;
        break;
    default:
        assert(!"unexpected bootloader command");
    }
}

void gpio_init(uint pin) {
    pin_output[pin] = false;
}

void gpio_set_dir(uint pin, bool out) {
    sim_esc_t *esc = &escs[pin];
    if (out) {
        esc->response_length = 0;
    } else if (pin_output[pin] && esc->packet_length > 0) {
        assert(esc->rx_bit == -1);
        if (esc->kind != ESC_ABSENT) {
            process_packet(esc);
        }
        esc->packet_length = 0;
    }
    pin_output[pin] = out;
}

/* Decode the Pico's bit-banged frames: start bit, eight LSB-first data
 * bits, stop bit. Idle-high writes outside a frame are ignored. */
void gpio_put(uint pin, bool value) {
    sim_esc_t *esc = &escs[pin];
    if (!pin_output[pin]) {
        return;
    }
    if (esc->rx_bit == -1) {
        if (!value) {
            esc->rx_bit = 0;
            esc->rx_value = 0;
        }
    } else if (esc->rx_bit < 8) {
        esc->rx_value |= (uint8_t)((value ? 1u : 0u) << esc->rx_bit);
        esc->rx_bit++;
    } else {
        assert(value); /* stop bit */
        assert(esc->packet_length < sizeof(esc->packet));
        esc->packet[esc->packet_length++] = esc->rx_value;
        esc->rx_bit = -1;
    }
}

bool gpio_get(uint pin) {
    const sim_esc_t *esc = &escs[pin];
    if (pin_output[pin] || esc->response_length == 0 || now_us < esc->response_start) {
        return true;
    }
    uint64_t elapsed = now_us - esc->response_start;
    size_t byte = (size_t)(elapsed / UART_BYTE_US);
    if (byte >= esc->response_length) {
        return true;
    }
    uint32_t slot = (uint32_t)(elapsed % UART_BYTE_US) / AM32_BIT_TIME_US;
    if (slot == 0) {
        return false;
    }
    if (slot > 8) {
        return true;
    }
    return ((esc->response[byte] >> (slot - 1)) & 1u) != 0;
}

static uint8_t image[TEST_IMAGE_SIZE];

static void reset_bench(const esc_kind_t kinds[NUM_MOTORS]) {
    memset(escs, 0, sizeof(escs));
    memset(pin_output, 0, sizeof(pin_output));
    status_count = 0;
    now_us = 0;
    for (uint pin = 0; pin < PIN_COUNT; ++pin) {
        escs[pin].rx_bit = -1;
        memset(escs[pin].flash, 0xFF, sizeof(escs[pin].flash));
    }
    for (uint8_t motor = 0; motor < NUM_MOTORS; ++motor) {
        escs[motor_pins[motor]].kind = kinds[motor];
    }
}

static bool flash(esc_firmware_update_error_t *error, uint8_t *failed_motor, bool *modified) {
    return am32_bootloader_flash_all(image, TEST_IMAGE_SIZE, error, failed_motor, modified);
}

static size_t count_status(esc_firmware_update_status_t status, uint8_t motor) {
    size_t count = 0;
    for (size_t i = 0; i < status_count; ++i) {
        if (statuses[i].status == status && statuses[i].motor == motor) {
            count++;
        }
    }
    return count;
}

static size_t first_status(esc_firmware_update_status_t status) {
    for (size_t i = 0; i < status_count; ++i) {
        if (statuses[i].status == status) {
            return i;
        }
    }
    return status_count;
}

static void test_flashes_only_the_escs_that_answer(void) {
    esc_kind_t kinds[NUM_MOTORS] = {ESC_ABSENT, ESC_ABSENT, ESC_ABSENT, ESC_ABSENT,
                                    ESC_ABSENT, ESC_ABSENT, ESC_ABSENT, ESC_PRESENT};
    reset_bench(kinds);
    esc_firmware_update_error_t error = ESC_FIRMWARE_UPDATE_ERROR_VERIFY;
    uint8_t failed_motor = 0;
    bool modified = false;

    assert(flash(&error, &failed_motor, &modified));
    assert(error == ESC_FIRMWARE_UPDATE_ERROR_NONE && failed_motor == UINT8_MAX && modified);
    assert(memcmp(escs[21].flash, image, TEST_IMAGE_SIZE) == 0 && escs[21].running);
    for (uint8_t motor = 0; motor < NUM_MOTORS - 1; ++motor) {
        assert(count_status(ESC_FIRMWARE_UPDATE_STATUS_MOTOR_SKIPPED, motor) == 1);
        assert(count_status(ESC_FIRMWARE_UPDATE_STATUS_MOTOR_BEGIN, motor) == 0);
        assert(count_status(ESC_FIRMWARE_UPDATE_STATUS_MOTOR_DONE, motor) == 0);
    }
    for (size_t i = 0; i < status_count; ++i) {
        if (statuses[i].status == ESC_FIRMWARE_UPDATE_STATUS_MOTOR_SKIPPED) {
            assert(statuses[i].error == ESC_FIRMWARE_UPDATE_ERROR_BOOTLOADER_CONNECT);
        }
    }
    assert(count_status(ESC_FIRMWARE_UPDATE_STATUS_MOTOR_SKIPPED, 7) == 0);
    assert(count_status(ESC_FIRMWARE_UPDATE_STATUS_MOTOR_DONE, 7) == 1);
    /* Every skip is reported during the pre-write check, before any write. */
    size_t first_begin = first_status(ESC_FIRMWARE_UPDATE_STATUS_MOTOR_BEGIN);
    for (size_t i = first_begin; i < status_count; ++i) {
        assert(statuses[i].status != ESC_FIRMWARE_UPDATE_STATUS_MOTOR_SKIPPED);
    }
}

static void test_fails_without_writing_when_no_esc_answers(void) {
    esc_kind_t kinds[NUM_MOTORS] = {ESC_ABSENT, ESC_ABSENT, ESC_ABSENT, ESC_ABSENT,
                                    ESC_ABSENT, ESC_ABSENT, ESC_ABSENT, ESC_ABSENT};
    reset_bench(kinds);
    esc_firmware_update_error_t error = ESC_FIRMWARE_UPDATE_ERROR_NONE;
    uint8_t failed_motor = 0;
    bool modified = true;

    assert(!flash(&error, &failed_motor, &modified));
    assert(error == ESC_FIRMWARE_UPDATE_ERROR_BOOTLOADER_CONNECT);
    assert(failed_motor == UINT8_MAX && !modified);
    assert(first_status(ESC_FIRMWARE_UPDATE_STATUS_MOTOR_BEGIN) == status_count);
    for (uint8_t motor = 0; motor < NUM_MOTORS; ++motor) {
        assert(count_status(ESC_FIRMWARE_UPDATE_STATUS_MOTOR_SKIPPED, motor) == 1);
    }
}

static void test_wrong_target_stops_the_bank_before_any_write(void) {
    esc_kind_t kinds[NUM_MOTORS] = {ESC_PRESENT, ESC_ABSENT, ESC_ABSENT, ESC_ABSENT,
                                    ESC_ABSENT,  ESC_ABSENT, ESC_ABSENT, ESC_WRONG_TARGET};
    reset_bench(kinds);
    esc_firmware_update_error_t error = ESC_FIRMWARE_UPDATE_ERROR_NONE;
    uint8_t failed_motor = 0;
    bool modified = true;

    assert(!flash(&error, &failed_motor, &modified));
    assert(error == ESC_FIRMWARE_UPDATE_ERROR_WRONG_TARGET);
    assert(failed_motor == 7 && !modified);
    assert(escs[motor_pins[0]].writes == 0 && escs[motor_pins[0]].running);
    assert(escs[motor_pins[7]].writes == 0);
    assert(first_status(ESC_FIRMWARE_UPDATE_STATUS_MOTOR_BEGIN) == status_count);
}

static void test_full_bank_flashes_every_esc_without_skips(void) {
    esc_kind_t kinds[NUM_MOTORS] = {ESC_PRESENT, ESC_PRESENT, ESC_PRESENT, ESC_PRESENT,
                                    ESC_PRESENT, ESC_PRESENT, ESC_PRESENT, ESC_PRESENT};
    reset_bench(kinds);
    esc_firmware_update_error_t error = ESC_FIRMWARE_UPDATE_ERROR_VERIFY;
    uint8_t failed_motor = 0;
    bool modified = false;

    assert(flash(&error, &failed_motor, &modified));
    assert(error == ESC_FIRMWARE_UPDATE_ERROR_NONE && modified);
    assert(first_status(ESC_FIRMWARE_UPDATE_STATUS_MOTOR_SKIPPED) == status_count);
    for (uint8_t motor = 0; motor < NUM_MOTORS; ++motor) {
        assert(memcmp(escs[motor_pins[motor]].flash, image, TEST_IMAGE_SIZE) == 0);
        assert(escs[motor_pins[motor]].running);
        assert(count_status(ESC_FIRMWARE_UPDATE_STATUS_MOTOR_DONE, motor) == 1);
    }
}

int main(void) {
    for (size_t i = 0; i < TEST_IMAGE_SIZE; ++i) {
        image[i] = (uint8_t)(i * 7u + 3u);
    }
    test_flashes_only_the_escs_that_answer();
    test_fails_without_writing_when_no_esc_answers();
    test_wrong_target_stops_the_bank_before_any_write();
    test_full_bank_flashes_every_esc_without_skips();
    puts("AM32 bootloader tests passed");
    return 0;
}
#endif
