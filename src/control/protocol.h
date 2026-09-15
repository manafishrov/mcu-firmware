#ifndef CONTROL_PROTOCOL_H
#define CONTROL_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CONTROL_FRAME_START 0xF0u
#define CONTROL_PROTOCOL_VERSION 1u
#define CONTROL_FRAME_HEADER_SIZE 14u
#define CONTROL_FRAME_OVERHEAD 18u
#define CONTROL_MAX_PAYLOAD 768u
#define CONTROL_FRAME_MAX 786u
#define CONTROL_SETTINGS_WIRE_SIZE 628u

typedef struct control_settings control_settings_t;

typedef struct {
    uint8_t type;
    uint32_t session;
    uint32_t sequence;
    uint16_t length;
    uint8_t payload[CONTROL_MAX_PAYLOAD];
} control_frame_t;

/* Unaligned little-endian access; callers provide the full field width. */
uint16_t control_get_u16(const uint8_t *wire);
uint32_t control_get_u32(const uint8_t *wire);
uint64_t control_get_u64(const uint8_t *wire);
float control_get_f32(const uint8_t *wire);
void control_put_u16(uint8_t *wire, uint16_t value);
void control_put_u32(uint8_t *wire, uint32_t value);
void control_put_u64(uint8_t *wire, uint64_t value);
void control_put_f32(uint8_t *wire, float value);

/* CRC32C/Castagnoli, initial and final XOR 0xffffffff. NULL is valid only for len=0. */
uint32_t control_crc32c(const uint8_t *data, size_t len);

/* No allocation, stream parsing, or command/session validation. Buffers must not overlap.
 * Encode returns bytes written, or zero without modifying out.
 * Decode requires exactly one complete v1 frame; failure leaves out unchanged.
 * Unused decoded payload bytes are zero. Unknown types remain available to the dispatcher. */
size_t control_frame_encode(uint8_t *out, size_t cap, const control_frame_t *frame);
bool control_frame_decode(const uint8_t *wire, size_t len, control_frame_t *out);

/* All output pointers are required and must not overlap wire or each other.
 * Validates the entire packed image, including the board-specific output suffix.
 * Failure leaves all outputs unchanged. No live settings are activated here. */
bool control_settings_decode(const uint8_t *wire, size_t len, control_settings_t *out,
                             uint16_t *protocol, uint16_t *speed);

#endif
