#include "protocol.h"

#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "controller.h"

#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128 || FLT_MIN_EXP != -125
#error "Control protocol requires IEEE 754 binary32 floats"
#endif
_Static_assert(sizeof(float) == sizeof(uint32_t), "Control protocol requires 32-bit floats");
_Static_assert(CONTROL_FRAME_HEADER_SIZE + sizeof(uint32_t) == CONTROL_FRAME_OVERHEAD,
               "Control frame overhead must include CRC32C");
_Static_assert(CONTROL_FRAME_OVERHEAD + CONTROL_MAX_PAYLOAD == CONTROL_FRAME_MAX,
               "Control frame maximum must include payload and overhead");

uint16_t control_get_u16(const uint8_t *wire) {
    return (uint16_t)((uint16_t)wire[0] | ((uint16_t)wire[1] << 8));
}

uint32_t control_get_u32(const uint8_t *wire) {
    return (uint32_t)wire[0] | ((uint32_t)wire[1] << 8) | ((uint32_t)wire[2] << 16) |
           ((uint32_t)wire[3] << 24);
}

uint64_t control_get_u64(const uint8_t *wire) {
    return (uint64_t)control_get_u32(wire) | ((uint64_t)control_get_u32(wire + 4) << 32);
}

float control_get_f32(const uint8_t *wire) {
    uint32_t bits = control_get_u32(wire);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

void control_put_u16(uint8_t *wire, uint16_t value) {
    wire[0] = (uint8_t)value;
    wire[1] = (uint8_t)(value >> 8);
}

void control_put_u32(uint8_t *wire, uint32_t value) {
    for (size_t i = 0; i < 4; ++i) {
        wire[i] = (uint8_t)(value >> (8 * i));
    }
}

void control_put_u64(uint8_t *wire, uint64_t value) {
    control_put_u32(wire, (uint32_t)value);
    control_put_u32(wire + 4, (uint32_t)(value >> 32));
}

void control_put_f32(uint8_t *wire, float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    control_put_u32(wire, bits);
}

uint32_t control_crc32c(const uint8_t *data, size_t len) {
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1u) ? 0x82F63B78u : 0u);
        }
    }
    return crc ^ UINT32_MAX;
}

size_t control_frame_encode(uint8_t *out, size_t cap, const control_frame_t *frame) {
    if (out == NULL || frame == NULL || frame->length > CONTROL_MAX_PAYLOAD) {
        return 0;
    }
    size_t size = CONTROL_FRAME_OVERHEAD + (size_t)frame->length;
    if (cap < size) {
        return 0;
    }
    out[0] = CONTROL_FRAME_START;
    out[1] = CONTROL_PROTOCOL_VERSION;
    out[2] = frame->type;
    out[3] = 0;
    control_put_u16(out + 4, frame->length);
    control_put_u32(out + 6, frame->session);
    control_put_u32(out + 10, frame->sequence);
    memcpy(out + CONTROL_FRAME_HEADER_SIZE, frame->payload, frame->length);
    control_put_u32(out + size - 4, control_crc32c(out, size - 4));
    return size;
}

bool control_frame_decode(const uint8_t *wire, size_t len, control_frame_t *out) {
    if (wire == NULL || out == NULL || len < CONTROL_FRAME_OVERHEAD || len > CONTROL_FRAME_MAX) {
        return false;
    }
    if (wire[0] != CONTROL_FRAME_START || wire[1] != CONTROL_PROTOCOL_VERSION || wire[3] != 0) {
        return false;
    }
    uint16_t length = control_get_u16(wire + 4);
    if (length > CONTROL_MAX_PAYLOAD || len != CONTROL_FRAME_OVERHEAD + (size_t)length ||
        control_get_u32(wire + len - 4) != control_crc32c(wire, len - 4)) {
        return false;
    }
    out->type = wire[2];
    out->session = control_get_u32(wire + 6);
    out->sequence = control_get_u32(wire + 10);
    out->length = length;
    memcpy(out->payload, wire + CONTROL_FRAME_HEADER_SIZE, length);
    memset(out->payload + length, 0, CONTROL_MAX_PAYLOAD - length);
    return true;
}

static control_axis_t decode_axis(const uint8_t *wire) {
    return (control_axis_t){.kp = control_get_f32(wire),
                            .ki = control_get_f32(wire + 4),
                            .kd = control_get_f32(wire + 8),
                            .rate = control_get_f32(wire + 12)};
}

static bool valid_speed(uint16_t speed) {
    switch (speed) {
    case 150:
    case 300:
    case 600:
#if PICO_RP2350
    case 1200:
#endif
        return true;
    default:
        return false;
    }
}

bool control_settings_decode(const uint8_t *wire, size_t len, control_settings_t *out,
                             uint16_t *protocol, uint16_t *speed) {
    if (wire == NULL || out == NULL || protocol == NULL || speed == NULL ||
        len != CONTROL_SETTINGS_WIRE_SIZE) {
        return false;
    }
    uint32_t fpv_mode = control_get_u32(wire + 64);
    uint32_t count = control_get_u32(wire + 364);
    uint16_t output_protocol = control_get_u16(wire + 624);
    uint16_t output_speed = control_get_u16(wire + 626);
    if (fpv_mode > 1 || count > 8 || output_protocol > 1 || !valid_speed(output_speed)) {
        return false;
    }

    control_settings_t candidate = {0};
    candidate.roll = decode_axis(wire);
    candidate.pitch = decode_axis(wire + 16);
    candidate.yaw = decode_axis(wire + 32);
    candidate.depth = decode_axis(wire + 48);
    candidate.fpv_mode = fpv_mode != 0;
    for (size_t i = 0; i < 3; ++i) {
        candidate.power[i] = control_get_f32(wire + 68 + (4 * i));
        candidate.coefficients[i] = control_get_f32(wire + 80 + (4 * i));
    }
    candidate.nullspace_count = count;
    for (size_t row = 0; row < 8; ++row) {
        candidate.identifiers[row] = wire[348 + row];
        uint8_t spin = wire[356 + row];
        if (spin != 1 && spin != UINT8_MAX) {
            return false;
        }
        candidate.spin[row] = spin == 1 ? 1 : -1;
        for (size_t col = 0; col < 8; ++col) {
            size_t index = (row * 8) + col;
            candidate.allocation[row][col] = control_get_f32(wire + 92 + (4 * index));
            candidate.nullspace[row][col] = control_get_f32(wire + 368 + (4 * index));
            if (row >= count && candidate.nullspace[row][col] != 0.0f) {
                return false;
            }
        }
    }
    if (!control_validate_settings(&candidate)) {
        return false;
    }
    *out = candidate;
    *protocol = output_protocol;
    *speed = output_speed;
    return true;
}
