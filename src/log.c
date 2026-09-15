#include "log.h"
#include "usb_tx.h"
#include <pico/time.h>
#include <pico/types.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t log_count;
static absolute_time_t log_window_start;

void log_init(void) {
    log_count = 0;
    log_window_start = get_absolute_time();
}

static bool rate_limit_check(void) {
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(log_window_start, now) >= 1000000) {
        log_window_start = now;
        log_count = 0;
    }
    if (log_count >= LOG_MAX_PER_SECOND) {
        return false;
    }
    log_count++;
    return true;
}

static void send_log(enum log_level level, const char *message) {
    if (!rate_limit_check()) {
        return;
    }

    size_t msg_len = strlen(message);
    if (msg_len > LOG_MAX_MESSAGE_SIZE) {
        msg_len = LOG_MAX_MESSAGE_SIZE;
    }

    uint8_t header[3];
    header[0] = LOG_START_BYTE;
    header[1] = (uint8_t)level;
    header[2] = (uint8_t)msg_len;

    uint8_t checksum = 0;
    for (size_t i = 0; i < 3; ++i) {
        checksum ^= header[i];
    }
    for (size_t i = 0; i < msg_len; ++i) {
        checksum ^= (uint8_t)message[i];
    }

    uint8_t packet[LOG_MAX_MESSAGE_SIZE + 4];
    memcpy(packet, header, sizeof(header));
    memcpy(packet + sizeof(header), message, msg_len);
    packet[msg_len + sizeof(header)] = checksum;
    (void)usb_tx_packet(packet, msg_len + 4, false);
}

static void send_logf(enum log_level level, const char *format, va_list args) {
    char message[LOG_MAX_MESSAGE_SIZE + 1];
    int written = vsnprintf(message, sizeof(message), format, args);
    if (written < 0) {
        return;
    }
    send_log(level, message);
}

void log_info(const char *message) {
    send_log(LOG_LEVEL_INFO, message);
}

void log_warn(const char *message) {
    send_log(LOG_LEVEL_WARN, message);
}

void log_error(const char *message) {
    send_log(LOG_LEVEL_ERROR, message);
}

void log_infof(const char *format, ...) {
    va_list args;
    va_start(args, format);
    send_logf(LOG_LEVEL_INFO, format, args);
    va_end(args);
}

void log_warnf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    send_logf(LOG_LEVEL_WARN, format, args);
    va_end(args);
}

void log_errorf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    send_logf(LOG_LEVEL_ERROR, format, args);
    va_end(args);
}
