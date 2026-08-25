#include "bootloader.h"
#include "esc_firmware/update.h"
#include "motors.h"
#include <hardware/gpio.h>
#include <hardware/sync.h>
#include <hardware/timer.h>
#include <pico/platform.h>
#include <pico/time.h>
#include <pico/types.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define AM32_BIT_TIME_US 52u
#define AM32_HALF_BIT_TIME_US 26u
#define AM32_ACK 0x30u
#define AM32_COMMAND_RUN 0x00u
#define AM32_COMMAND_PROGRAM_FLASH 0x01u
#define AM32_COMMAND_READ_FLASH 0x03u
#define AM32_COMMAND_SET_BUFFER 0xFEu
#define AM32_COMMAND_SET_ADDRESS 0xFFu
#define AM32_PROGRAM_CHUNK_SIZE 256u
#define AM32_BOOTLOADER_ENTRY_MS 2300u
#define AM32_RESPONSE_TIMEOUT_US 50000u
#define AM32_CONNECT_ATTEMPTS 30u
#define AM32_PROGRAM_ATTEMPTS 3u
#define AM32_DEVICE_INFO_SIZE 9u
#define AM32_BOOTLOADER_PROTOCOL_MIN 1u
#define AM32_EXPECTED_PIN_CODE 0x14u
#define AM32_EXPECTED_FLASH_SIZE 0x1Fu

static const uint motor_pins[NUM_MOTORS] = {
    MOTOR0_PIN_BASE, MOTOR0_PIN_BASE + 1, MOTOR0_PIN_BASE + 2, MOTOR0_PIN_BASE + 3,
    MOTOR1_PIN_BASE, MOTOR1_PIN_BASE + 1, MOTOR1_PIN_BASE + 2, MOTOR1_PIN_BASE + 3,
};

static uint16_t crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        uint8_t value = data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (((value & 1u) ^ (crc & 1u)) != 0) {
                crc = (crc >> 1) ^ 0xA001u;
            } else {
                crc >>= 1;
            }
            value >>= 1;
        }
    }
    return crc;
}

static void append_crc(uint8_t *packet, size_t payload_length) {
    uint16_t crc = crc16(packet, payload_length);
    packet[payload_length] = (uint8_t)crc;
    packet[payload_length + 1] = (uint8_t)(crc >> 8);
}

static void set_receive(uint pin) {
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

static void send_bytes(uint pin, const uint8_t *data, size_t length) {
    gpio_put(pin, true);
    gpio_set_dir(pin, GPIO_OUT);

    uint32_t interrupt_state = save_and_disable_interrupts();
    for (size_t i = 0; i < length; ++i) {
        uint8_t value = data[i];
        gpio_put(pin, false);
        busy_wait_us_32(AM32_BIT_TIME_US);
        for (uint8_t bit = 0; bit < 8; ++bit) {
            gpio_put(pin, (value & 1u) != 0);
            value >>= 1;
            busy_wait_us_32(AM32_BIT_TIME_US);
        }
        gpio_put(pin, true);
        busy_wait_us_32(AM32_BIT_TIME_US);
    }
    restore_interrupts(interrupt_state);
    set_receive(pin);
}

static bool receive_byte(uint pin, uint8_t *value, uint32_t timeout_us) {
    uint32_t start = time_us_32();
    while (gpio_get(pin)) {
        if (time_us_32() - start >= timeout_us) {
            return false;
        }
    }

    uint32_t interrupt_state = save_and_disable_interrupts();
    busy_wait_us_32(AM32_HALF_BIT_TIME_US);
    if (gpio_get(pin)) {
        restore_interrupts(interrupt_state);
        return false;
    }

    uint8_t received = 0;
    for (uint8_t bit = 0; bit < 8; ++bit) {
        busy_wait_us_32(AM32_BIT_TIME_US);
        if (gpio_get(pin)) {
            received |= (uint8_t)(1u << bit);
        }
    }
    busy_wait_us_32(AM32_BIT_TIME_US);
    bool stop_bit = gpio_get(pin);
    restore_interrupts(interrupt_state);
    if (!stop_bit) {
        return false;
    }

    *value = received;
    return true;
}

static bool receive_bytes(uint pin, uint8_t *data, size_t length, uint32_t first_timeout_us) {
    for (size_t i = 0; i < length; ++i) {
        uint32_t timeout = i == 0 ? first_timeout_us : AM32_RESPONSE_TIMEOUT_US;
        if (!receive_byte(pin, &data[i], timeout)) {
            return false;
        }
    }
    return true;
}

static bool receive_expected(uint pin, uint8_t expected, uint32_t timeout_us) {
    uint8_t response = 0;
    return receive_byte(pin, &response, timeout_us) && response == expected;
}

static bool set_address(uint pin, uint16_t address) {
    uint8_t packet[6] = {
        AM32_COMMAND_SET_ADDRESS, 0, (uint8_t)(address >> 8), (uint8_t)address, 0, 0,
    };
    append_crc(packet, 4);
    send_bytes(pin, packet, sizeof(packet));
    return receive_expected(pin, AM32_ACK, AM32_RESPONSE_TIMEOUT_US);
}

static bool set_buffer(uint pin, const uint8_t *data, uint16_t length) {
    uint8_t command[6] = {
        AM32_COMMAND_SET_BUFFER, 0, length == AM32_PROGRAM_CHUNK_SIZE ? 1u : 0u,
        (uint8_t)length,         0, 0,
    };
    append_crc(command, 4);
    send_bytes(pin, command, sizeof(command));

    // SET_BUFFER has no acknowledgement. The bootloader recognizes the end
    // of this command from the inter-packet gap before accepting the payload.
    busy_wait_us_32(500);

    uint8_t payload[AM32_PROGRAM_CHUNK_SIZE + 2];
    memcpy(payload, data, length);
    append_crc(payload, length);
    send_bytes(pin, payload, (size_t)length + 2);
    return receive_expected(pin, AM32_ACK, AM32_RESPONSE_TIMEOUT_US);
}

static bool program_buffer(uint pin) {
    uint8_t packet[4] = {AM32_COMMAND_PROGRAM_FLASH, 0, 0, 0};
    append_crc(packet, 2);
    send_bytes(pin, packet, sizeof(packet));
    return receive_expected(pin, AM32_ACK, AM32_RESPONSE_TIMEOUT_US);
}

static bool read_flash(uint pin, uint16_t address, uint8_t *data, uint16_t length) {
    if (!set_address(pin, address)) {
        return false;
    }

    uint8_t command[4] = {AM32_COMMAND_READ_FLASH, (uint8_t)length, 0, 0};
    append_crc(command, 2);
    send_bytes(pin, command, sizeof(command));

    uint8_t response[AM32_PROGRAM_CHUNK_SIZE + 3];
    if (!receive_bytes(pin, response, (size_t)length + 3, AM32_RESPONSE_TIMEOUT_US)) {
        return false;
    }
    uint16_t received_crc = (uint16_t)response[length] | ((uint16_t)response[length + 1] << 8);
    if (received_crc != crc16(response, length) || response[length + 2] != AM32_ACK) {
        return false;
    }
    memcpy(data, response, length);
    return true;
}

static bool connect_motor(uint pin, uint8_t *device_info) {
    uint8_t handshake[17] = {0};
    handshake[8] = 13;
    handshake[9] = 66;
    handshake[16] = 0x7D;
    send_bytes(pin, handshake, sizeof(handshake));
    return receive_bytes(pin, device_info, AM32_DEVICE_INFO_SIZE, AM32_RESPONSE_TIMEOUT_US);
}

static bool target_matches(const uint8_t *device_info) {
    return device_info[0] == '4' && device_info[1] == '7' && device_info[2] == '1' &&
           device_info[3] == AM32_EXPECTED_PIN_CODE && device_info[4] == AM32_EXPECTED_FLASH_SIZE &&
           device_info[7] >= AM32_BOOTLOADER_PROTOCOL_MIN && device_info[8] == AM32_ACK;
}

static void run_all(void) {
    const uint8_t run_command[4] = {AM32_COMMAND_RUN, 0, 0, 0};
    for (uint8_t motor = 0; motor < NUM_MOTORS; ++motor) {
        send_bytes(motor_pins[motor], run_command, sizeof(run_command));
        gpio_disable_pulls(motor_pins[motor]);
    }
}

static bool wait_for_motor(uint8_t motor, esc_firmware_update_error_t *error) {
    uint8_t device_info[AM32_DEVICE_INFO_SIZE];
    for (uint8_t attempt = 0; attempt < AM32_CONNECT_ATTEMPTS; ++attempt) {
        if (connect_motor(motor_pins[motor], device_info)) {
            if (!target_matches(device_info)) {
                *error = ESC_FIRMWARE_UPDATE_ERROR_WRONG_TARGET;
                return false;
            }
            return true;
        }
        sleep_ms(70);
    }
    *error = ESC_FIRMWARE_UPDATE_ERROR_BOOTLOADER_CONNECT;
    return false;
}

static bool program_chunk(uint8_t motor, const uint8_t *data, uint16_t offset, uint16_t length,
                          esc_firmware_update_error_t *error) {
    uint pin = motor_pins[motor];
    uint16_t address = (uint16_t)(0x1000u + offset);
    uint8_t verify[AM32_PROGRAM_CHUNK_SIZE];

    for (uint8_t attempt = 0; attempt < AM32_PROGRAM_ATTEMPTS; ++attempt) {
        if (!set_address(pin, address) || !set_buffer(pin, data, length) || !program_buffer(pin)) {
            *error = ESC_FIRMWARE_UPDATE_ERROR_PROGRAM;
            continue;
        }
        if (!read_flash(pin, address, verify, length) || memcmp(data, verify, length) != 0) {
            *error = ESC_FIRMWARE_UPDATE_ERROR_VERIFY;
            continue;
        }
        return true;
    }
    return false;
}

bool am32_bootloader_flash_all(const uint8_t *image, uint16_t image_size,
                               esc_firmware_update_error_t *error, uint8_t *failed_motor,
                               bool *modified) {
    *modified = false;
    for (uint8_t motor = 0; motor < NUM_MOTORS; ++motor) {
        gpio_init(motor_pins[motor]);
        gpio_put(motor_pins[motor], true);
        gpio_set_dir(motor_pins[motor], GPIO_OUT);
    }

    esc_firmware_update_send_status(ESC_FIRMWARE_UPDATE_STATUS_ENTERING_BOOTLOADER, UINT8_MAX,
                                    ESC_FIRMWARE_UPDATE_ERROR_NONE, 0);
    // The application resets after two seconds without DShot. Its bootloader
    // then waits indefinitely while the UART line remains idle high, so all
    // eight bootloaders can be entered together before programming begins.
    sleep_ms(AM32_BOOTLOADER_ENTRY_MS);

    // Prove that every connected ESC is the supported target before writing
    // the first byte. A missing or wrong target therefore remains a fully
    // recoverable pre-write failure for the entire bank.
    for (uint8_t motor = 0; motor < NUM_MOTORS; ++motor) {
        *failed_motor = motor;
        if (!wait_for_motor(motor, error)) {
            run_all();
            return false;
        }
    }

    for (uint8_t motor = 0; motor < NUM_MOTORS; ++motor) {
        *failed_motor = motor;
        esc_firmware_update_send_status(ESC_FIRMWARE_UPDATE_STATUS_MOTOR_BEGIN, motor,
                                        ESC_FIRMWARE_UPDATE_ERROR_NONE, 0);
        for (uint16_t offset = 0; offset < image_size; offset += AM32_PROGRAM_CHUNK_SIZE) {
            uint16_t remaining = image_size - offset;
            uint16_t length =
                remaining < AM32_PROGRAM_CHUNK_SIZE ? remaining : AM32_PROGRAM_CHUNK_SIZE;
            // A failed program command may still have changed flash. Mark the
            // bank unsafe before issuing the first write, not after its ACK.
            *modified = true;
            if (!program_chunk(motor, &image[offset], offset, length, error)) {
                return false;
            }
            esc_firmware_update_send_status(ESC_FIRMWARE_UPDATE_STATUS_MOTOR_BEGIN, motor,
                                            ESC_FIRMWARE_UPDATE_ERROR_NONE,
                                            (uint32_t)offset + length);
        }
        esc_firmware_update_send_status(ESC_FIRMWARE_UPDATE_STATUS_MOTOR_DONE, motor,
                                        ESC_FIRMWARE_UPDATE_ERROR_NONE, image_size);
    }

    run_all();
    *error = ESC_FIRMWARE_UPDATE_ERROR_NONE;
    *failed_motor = UINT8_MAX;
    return true;
}
