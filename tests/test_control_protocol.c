#include "control/controller.h"
#include "control/protocol.h"
#include "unity/unity.h"

#include <float.h>
#include <math.h>
#include <string.h>

/* Independently calculated with the non-reflected Castagnoli polynomial 0x1edc6f41. */
static const uint8_t golden_frame[] = {0xf0, 0x01, 0x10, 0x00, 0x06, 0x00, 0x78, 0x56,
                                       0x34, 0x12, 0xef, 0xcd, 0xab, 0x90, 0xf0, 0xc5,
                                       0xd5, 0x5a, 0x00, 0x01, 0x62, 0xbe, 0xea, 0x56};

static void test_endian_unaligned_fields(void) {
    uint8_t wire[18];
    memset(wire, 0xa5, sizeof(wire));
    const uint8_t expected[] = {0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x01};
    control_put_u64(wire + 1, UINT64_C(0x0123456789abcdef));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, wire + 1, sizeof(expected));
    TEST_ASSERT_EQUAL_HEX64(UINT64_C(0x0123456789abcdef), control_get_u64(wire + 1));
    TEST_ASSERT_EQUAL_HEX32(0x89abcdef, control_get_u32(wire + 1));
    TEST_ASSERT_EQUAL_HEX16(0xcdef, control_get_u16(wire + 1));
    control_put_u16(wire + 9, 0x8123);
    TEST_ASSERT_EQUAL_HEX8(0x23, wire[9]);
    TEST_ASSERT_EQUAL_HEX8(0x81, wire[10]);
    control_put_u32(wire + 11, 0x89abcdef);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, wire + 11, 4);
    TEST_ASSERT_EQUAL_HEX8(0xa5, wire[0]);
    TEST_ASSERT_EQUAL_HEX8(0xa5, wire[15]);
    control_put_u64(wire + 1, UINT64_MAX);
    TEST_ASSERT_EQUAL_HEX64(UINT64_MAX, control_get_u64(wire + 1));
    control_put_u64(wire + 1, 0);
    TEST_ASSERT_EQUAL_HEX64(0, control_get_u64(wire + 1));
}

static void test_float_wire_bits(void) {
    uint8_t wire[6] = {0xa5, 0, 0, 0, 0, 0xa5};
    const uint32_t patterns[] = {0,          0x80000000, 0x3f800000, 0xc0200000,
                                 0x00000001, 0x007fffff, 0x00800000, 0x7f7fffff,
                                 0x7f800000, 0xff800000, 0x7fc12345};
    for (size_t i = 0; i < sizeof(patterns) / sizeof(patterns[0]); ++i) {
        control_put_u32(wire + 1, patterns[i]);
        float value = control_get_f32(wire + 1);
        uint32_t bits;
        memcpy(&bits, &value, sizeof(bits));
        TEST_ASSERT_EQUAL_HEX32(patterns[i], bits);
        control_put_f32(wire + 1, value);
        TEST_ASSERT_EQUAL_HEX32(patterns[i], control_get_u32(wire + 1));
    }
    const uint8_t negative_two_and_half[] = {0, 0, 0x20, 0xc0};
    control_put_f32(wire + 1, -2.5f);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(negative_two_and_half, wire + 1, 4);
    TEST_ASSERT_EQUAL_FLOAT(-2.5f, control_get_f32(wire + 1));
    TEST_ASSERT_EQUAL_HEX8(0xa5, wire[0]);
    TEST_ASSERT_EQUAL_HEX8(0xa5, wire[5]);
}

static void test_crc_known_vectors(void) {
    const uint8_t zeros[32] = {0};
    TEST_ASSERT_EQUAL_HEX32(0, control_crc32c(NULL, 0));
    TEST_ASSERT_EQUAL_HEX32(0xe3069283, control_crc32c((const uint8_t *)"123456789", 9));
    TEST_ASSERT_EQUAL_HEX32(0x8a9136aa, control_crc32c(zeros, sizeof(zeros)));
    TEST_ASSERT_EQUAL_HEX32(0x56eabe62, control_crc32c(golden_frame, sizeof(golden_frame) - 4));
}

static void test_frame_golden_vector(void) {
    control_frame_t frame;
    TEST_ASSERT_TRUE(control_frame_decode(golden_frame, sizeof(golden_frame), &frame));
    TEST_ASSERT_EQUAL_HEX8(0x10, frame.type);
    TEST_ASSERT_EQUAL_HEX32(0x12345678, frame.session);
    TEST_ASSERT_EQUAL_HEX32(0x90abcdef, frame.sequence);
    TEST_ASSERT_EQUAL_UINT16(6, frame.length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(golden_frame + 14, frame.payload, 6);
    for (size_t i = 6; i < CONTROL_MAX_PAYLOAD; ++i) {
        TEST_ASSERT_EQUAL_UINT8(0, frame.payload[i]);
    }
    uint8_t encoded[sizeof(golden_frame) + 2];
    memset(encoded, 0xa5, sizeof(encoded));
    TEST_ASSERT_EQUAL_UINT(sizeof(golden_frame),
                           control_frame_encode(encoded + 1, sizeof(golden_frame), &frame));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(golden_frame, encoded + 1, sizeof(golden_frame));
    TEST_ASSERT_EQUAL_HEX8(0xa5, encoded[0]);
    TEST_ASSERT_EQUAL_HEX8(0xa5, encoded[sizeof(encoded) - 1]);
}

static void test_frame_all_payload_lengths(void) {
    control_frame_t frame = {.type = 0xff, .session = 0, .sequence = UINT32_MAX};
    control_frame_t decoded;
    uint8_t encoded[CONTROL_FRAME_MAX + 2];
    for (size_t i = 0; i < CONTROL_MAX_PAYLOAD; ++i) {
        frame.payload[i] = (uint8_t)i;
    }
    /* The codec accepts unknown types, session zero and every u32 sequence. */
    for (size_t length = 0; length <= CONTROL_MAX_PAYLOAD; ++length) {
        frame.length = (uint16_t)length;
        memset(encoded, 0xa5, sizeof(encoded));
        size_t total = length + CONTROL_FRAME_OVERHEAD;
        TEST_ASSERT_EQUAL_UINT(total, control_frame_encode(encoded + 1, total, &frame));
        TEST_ASSERT_EQUAL_HEX8(0xa5, encoded[0]);
        TEST_ASSERT_EQUAL_HEX8(0xa5, encoded[total + 1]);
        TEST_ASSERT_TRUE(control_frame_decode(encoded + 1, total, &decoded));
        TEST_ASSERT_EQUAL_UINT16(length, decoded.length);
        TEST_ASSERT_EQUAL_HEX8(frame.type, decoded.type);
        TEST_ASSERT_EQUAL_HEX32(frame.session, decoded.session);
        TEST_ASSERT_EQUAL_HEX32(frame.sequence, decoded.sequence);
        if (length != 0) {
            TEST_ASSERT_EQUAL_HEX8_ARRAY(frame.payload, decoded.payload, length);
        }
        for (size_t i = length; i < CONTROL_MAX_PAYLOAD; ++i) {
            TEST_ASSERT_EQUAL_UINT8(0, decoded.payload[i]);
        }
    }
}

static void assert_frame_rejected(const uint8_t *wire, size_t len) {
    control_frame_t output;
    memset(&output, 0xa5, sizeof(output));
    uint8_t before[sizeof(output)];
    memcpy(before, &output, sizeof(output));
    TEST_ASSERT_FALSE(control_frame_decode(wire, len, &output));
    TEST_ASSERT_EQUAL_MEMORY(before, &output, sizeof(output));
}

static void test_frame_rejects_every_single_bit_corruption(void) {
    uint8_t wire[sizeof(golden_frame)];
    for (size_t i = 0; i < sizeof(wire); ++i) {
        for (unsigned bit = 0; bit < 8; ++bit) {
            memcpy(wire, golden_frame, sizeof(wire));
            wire[i] ^= (uint8_t)(1u << bit);
            assert_frame_rejected(wire, sizeof(wire));
        }
    }
}

static void test_frame_rejects_invalid_envelope_even_with_valid_crc(void) {
    uint8_t wire[sizeof(golden_frame)];
    const size_t offsets[] = {0, 1, 3, 4, 5};
    for (size_t i = 0; i < sizeof(offsets) / sizeof(offsets[0]); ++i) {
        for (unsigned value = 0; value <= UINT8_MAX; ++value) {
            if (value == golden_frame[offsets[i]]) {
                continue;
            }
            memcpy(wire, golden_frame, sizeof(wire));
            wire[offsets[i]] = (uint8_t)value;
            control_put_u32(wire + sizeof(wire) - 4, control_crc32c(wire, sizeof(wire) - 4));
            assert_frame_rejected(wire, sizeof(wire));
        }
    }
}

static void test_frame_rejects_truncation_trailing_bytes_and_null(void) {
    for (size_t len = 0; len < sizeof(golden_frame); ++len) {
        assert_frame_rejected(golden_frame, len);
    }
    uint8_t extended[CONTROL_FRAME_MAX + 1] = {0};
    memcpy(extended, golden_frame, sizeof(golden_frame));
    for (size_t len = sizeof(golden_frame) + 1; len <= sizeof(extended); ++len) {
        assert_frame_rejected(extended, len);
    }
    assert_frame_rejected(extended, SIZE_MAX);
    assert_frame_rejected(NULL, 0);
    assert_frame_rejected(NULL, sizeof(golden_frame));
    TEST_ASSERT_FALSE(control_frame_decode(golden_frame, sizeof(golden_frame), NULL));
}

static void test_frame_encode_rejects_without_writing(void) {
    control_frame_t frame = {.length = CONTROL_MAX_PAYLOAD};
    uint8_t wire[CONTROL_FRAME_MAX];
    uint8_t before[sizeof(wire)];
    memset(before, 0xa5, sizeof(before));
    for (size_t cap = 0; cap < sizeof(wire); ++cap) {
        memcpy(wire, before, sizeof(wire));
        TEST_ASSERT_EQUAL_UINT(0, control_frame_encode(wire, cap, &frame));
        TEST_ASSERT_EQUAL_MEMORY(before, wire, sizeof(wire));
    }
    const uint16_t invalid_lengths[] = {CONTROL_MAX_PAYLOAD + 1, UINT16_MAX};
    for (size_t i = 0; i < sizeof(invalid_lengths) / sizeof(invalid_lengths[0]); ++i) {
        frame.length = invalid_lengths[i];
        TEST_ASSERT_EQUAL_UINT(0, control_frame_encode(wire, SIZE_MAX, &frame));
        TEST_ASSERT_EQUAL_MEMORY(before, wire, sizeof(wire));
    }
    TEST_ASSERT_EQUAL_UINT(0, control_frame_encode(wire, sizeof(wire), NULL));
    TEST_ASSERT_EQUAL_MEMORY(before, wire, sizeof(wire));
    TEST_ASSERT_EQUAL_UINT(0, control_frame_encode(NULL, sizeof(wire), &frame));
}

static void make_settings(uint8_t wire[CONTROL_SETTINGS_WIRE_SIZE]) {
    memset(wire, 0, CONTROL_SETTINGS_WIRE_SIZE);
    for (size_t i = 0; i < 16; ++i) {
        control_put_f32(wire + 4 * i, (float)i + 0.25f);
    }
    control_put_u32(wire + 64, 1);
    for (size_t i = 0; i < 3; ++i) {
        control_put_f32(wire + 68 + 4 * i, (float)(25 * i));
        control_put_f32(wire + 80 + 4 * i, (float)i - 0.5f);
    }
    for (size_t i = 0; i < 64; ++i) {
        control_put_f32(wire + 92 + 4 * i, ((float)i - 32.0f) / 64.0f);
        control_put_f32(wire + 368 + 4 * i, ((float)i - 16.0f) / 32.0f);
    }
    for (size_t i = 0; i < 8; ++i) {
        wire[348 + i] = (uint8_t)(7 - i);
        wire[356 + i] = i % 2 == 0 ? 1 : 0xff;
    }
    control_put_u32(wire + 364, 8);
    control_put_u16(wire + 624, 1);
    control_put_u16(wire + 626, 600);
}

static void test_settings_decodes_every_offset_unaligned(void) {
    uint8_t storage[CONTROL_SETTINGS_WIRE_SIZE + 2];
    memset(storage, 0xa5, sizeof(storage));
    uint8_t *wire = storage + 1;
    make_settings(wire);
    control_settings_t settings;
    uint16_t protocol, speed;
    TEST_ASSERT_TRUE(
        control_settings_decode(wire, CONTROL_SETTINGS_WIRE_SIZE, &settings, &protocol, &speed));
    const control_axis_t *axes[] = {&settings.roll, &settings.pitch, &settings.yaw,
                                    &settings.depth};
    for (size_t i = 0; i < 4; ++i) {
        TEST_ASSERT_EQUAL_FLOAT((float)(4 * i) + 0.25f, axes[i]->kp);
        TEST_ASSERT_EQUAL_FLOAT((float)(4 * i) + 1.25f, axes[i]->ki);
        TEST_ASSERT_EQUAL_FLOAT((float)(4 * i) + 2.25f, axes[i]->kd);
        TEST_ASSERT_EQUAL_FLOAT((float)(4 * i) + 3.25f, axes[i]->rate);
    }
    TEST_ASSERT_TRUE(settings.fpv_mode);
    for (size_t i = 0; i < 3; ++i) {
        TEST_ASSERT_EQUAL_FLOAT((float)(25 * i), settings.power[i]);
        TEST_ASSERT_EQUAL_FLOAT((float)i - 0.5f, settings.coefficients[i]);
    }
    for (size_t row = 0; row < 8; ++row) {
        TEST_ASSERT_EQUAL_UINT8(7 - row, settings.identifiers[row]);
        TEST_ASSERT_EQUAL_INT8(row % 2 == 0 ? 1 : -1, settings.spin[row]);
        for (size_t col = 0; col < 8; ++col) {
            float index = (float)(row * 8 + col);
            TEST_ASSERT_EQUAL_FLOAT((index - 32.0f) / 64.0f, settings.allocation[row][col]);
            TEST_ASSERT_EQUAL_FLOAT((index - 16.0f) / 32.0f, settings.nullspace[row][col]);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(8, settings.nullspace_count);
    TEST_ASSERT_EQUAL_UINT16(1, protocol);
    TEST_ASSERT_EQUAL_UINT16(600, speed);
    TEST_ASSERT_EQUAL_HEX8(0xa5, storage[0]);
    TEST_ASSERT_EQUAL_HEX8(0xa5, storage[sizeof(storage) - 1]);
}

static void assert_settings_rejected(const uint8_t *wire, size_t len) {
    control_settings_t output;
    memset(&output, 0xa5, sizeof(output));
    uint8_t before[sizeof(output)];
    memcpy(before, &output, sizeof(output));
    uint16_t protocol = 0x1234, speed = 0x5678;
    TEST_ASSERT_FALSE(control_settings_decode(wire, len, &output, &protocol, &speed));
    TEST_ASSERT_EQUAL_MEMORY(before, &output, sizeof(output));
    TEST_ASSERT_EQUAL_HEX16(0x1234, protocol);
    TEST_ASSERT_EQUAL_HEX16(0x5678, speed);
}

static void test_settings_rejects_nonfinite_at_every_float_offset(void) {
    uint8_t wire[CONTROL_SETTINGS_WIRE_SIZE];
    const size_t ranges[][2] = {{0, 64}, {68, 348}, {368, 624}};
    const uint32_t nonfinite[] = {0x7f800000, 0xff800000, 0x7fc00000, 0xffc12345, 0x7f800001};
    for (size_t range = 0; range < sizeof(ranges) / sizeof(ranges[0]); ++range) {
        for (size_t offset = ranges[range][0]; offset < ranges[range][1]; offset += 4) {
            for (size_t i = 0; i < sizeof(nonfinite) / sizeof(nonfinite[0]); ++i) {
                make_settings(wire);
                control_put_u32(wire + offset, nonfinite[i]);
                assert_settings_rejected(wire, sizeof(wire));
            }
        }
    }
}

static void test_settings_rejects_invalid_discrete_fields(void) {
    uint8_t wire[CONTROL_SETTINGS_WIRE_SIZE];
    const uint32_t invalid_fpv[] = {2, 255, 256, 0x10000, 0x1000000, UINT32_MAX};
    for (size_t i = 0; i < sizeof(invalid_fpv) / sizeof(invalid_fpv[0]); ++i) {
        make_settings(wire);
        control_put_u32(wire + 64, invalid_fpv[i]);
        assert_settings_rejected(wire, sizeof(wire));
    }
    const uint32_t invalid_counts[] = {9, 256, 0x10000, 0x1000000, UINT32_MAX};
    for (size_t i = 0; i < sizeof(invalid_counts) / sizeof(invalid_counts[0]); ++i) {
        make_settings(wire);
        control_put_u32(wire + 364, invalid_counts[i]);
        assert_settings_rejected(wire, sizeof(wire));
    }
    for (size_t i = 0; i < 8; ++i) {
        for (unsigned value = 0; value <= UINT8_MAX; ++value) {
            if (value >= 8) {
                make_settings(wire);
                wire[348 + i] = (uint8_t)value;
                assert_settings_rejected(wire, sizeof(wire));
            }
            if (value != 1 && value != 0xff) {
                make_settings(wire);
                wire[356 + i] = (uint8_t)value;
                assert_settings_rejected(wire, sizeof(wire));
            }
        }
    }
}

static void test_settings_power_boundaries_and_duplicate_identifiers(void) {
    uint8_t wire[CONTROL_SETTINGS_WIRE_SIZE];
    control_settings_t settings;
    uint16_t protocol, speed;
    const float valid[] = {0.0f, -0.0f, 100.0f};
    const float invalid[] = {-0.001f, 100.001f, FLT_MAX, -FLT_MAX};
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < sizeof(valid) / sizeof(valid[0]); ++j) {
            make_settings(wire);
            memset(wire + 348, 3, 8);
            control_put_u32(wire + 64, 0);
            control_put_f32(wire + 68 + 4 * i, valid[j]);
            TEST_ASSERT_TRUE(
                control_settings_decode(wire, sizeof(wire), &settings, &protocol, &speed));
            TEST_ASSERT_FALSE(settings.fpv_mode);
            TEST_ASSERT_EQUAL_FLOAT(valid[j], settings.power[i]);
        }
        for (size_t j = 0; j < sizeof(invalid) / sizeof(invalid[0]); ++j) {
            make_settings(wire);
            control_put_f32(wire + 68 + 4 * i, invalid[j]);
            assert_settings_rejected(wire, sizeof(wire));
        }
    }
}

static void test_settings_nullspace_count_and_unused_rows(void) {
    uint8_t wire[CONTROL_SETTINGS_WIRE_SIZE];
    control_settings_t settings;
    uint16_t protocol, speed;
    for (uint32_t count = 0; count <= 8; ++count) {
        make_settings(wire);
        control_put_u32(wire + 364, count);
        memset(wire + 368 + count * 32, 0, (8 - count) * 32);
        TEST_ASSERT_TRUE(control_settings_decode(wire, sizeof(wire), &settings, &protocol, &speed));
        TEST_ASSERT_EQUAL_UINT32(count, settings.nullspace_count);
        for (size_t index = count * 8; index < 64; ++index) {
            control_put_f32(wire + 368 + 4 * index, -0.0f);
            TEST_ASSERT_TRUE(
                control_settings_decode(wire, sizeof(wire), &settings, &protocol, &speed));
            control_put_f32(wire + 368 + 4 * index, 0.001f);
            assert_settings_rejected(wire, sizeof(wire));
            control_put_f32(wire + 368 + 4 * index, NAN);
            assert_settings_rejected(wire, sizeof(wire));
            control_put_f32(wire + 368 + 4 * index, 0);
        }
    }
}

static void test_settings_output_suffix_and_board_speed(void) {
    uint8_t wire[CONTROL_SETTINGS_WIRE_SIZE];
    control_settings_t settings;
    uint16_t protocol, speed;
    const uint16_t speeds[] = {0, 1, 149, 150, 151, 300, 450, 600, 1200, 2400, UINT16_MAX};
    for (uint16_t mode = 0; mode < 2; ++mode) {
        for (size_t i = 0; i < sizeof(speeds) / sizeof(speeds[0]); ++i) {
            make_settings(wire);
            control_put_u16(wire + 624, mode);
            control_put_u16(wire + 626, speeds[i]);
            bool valid = speeds[i] == 150 || speeds[i] == 300 || speeds[i] == 600;
#if PICO_RP2350
            valid = valid || speeds[i] == 1200;
#endif
            if (valid) {
                TEST_ASSERT_TRUE(
                    control_settings_decode(wire, sizeof(wire), &settings, &protocol, &speed));
                TEST_ASSERT_EQUAL_UINT16(mode, protocol);
                TEST_ASSERT_EQUAL_UINT16(speeds[i], speed);
            } else {
                assert_settings_rejected(wire, sizeof(wire));
            }
        }
    }
    const uint16_t invalid_protocols[] = {2, 255, 256, UINT16_MAX};
    for (size_t i = 0; i < sizeof(invalid_protocols) / sizeof(invalid_protocols[0]); ++i) {
        make_settings(wire);
        control_put_u16(wire + 624, invalid_protocols[i]);
        assert_settings_rejected(wire, sizeof(wire));
    }
}

static void test_settings_rejects_wrong_size_and_null_arguments(void) {
    uint8_t wire[CONTROL_SETTINGS_WIRE_SIZE + 1];
    make_settings(wire);
    wire[CONTROL_SETTINGS_WIRE_SIZE] = 0;
    for (size_t len = 0; len < CONTROL_SETTINGS_WIRE_SIZE; ++len) {
        assert_settings_rejected(wire, len);
    }
    assert_settings_rejected(wire, sizeof(wire));
    assert_settings_rejected(wire, SIZE_MAX);
    assert_settings_rejected(NULL, CONTROL_SETTINGS_WIRE_SIZE);
    control_settings_t settings;
    memset(&settings, 0xa5, sizeof(settings));
    uint8_t before[sizeof(settings)];
    memcpy(before, &settings, sizeof(settings));
    uint16_t protocol = 0x1234, speed = 0x5678;
    TEST_ASSERT_FALSE(
        control_settings_decode(wire, CONTROL_SETTINGS_WIRE_SIZE, NULL, &protocol, &speed));
    TEST_ASSERT_FALSE(
        control_settings_decode(wire, CONTROL_SETTINGS_WIRE_SIZE, &settings, NULL, &speed));
    TEST_ASSERT_FALSE(
        control_settings_decode(wire, CONTROL_SETTINGS_WIRE_SIZE, &settings, &protocol, NULL));
    TEST_ASSERT_EQUAL_MEMORY(before, &settings, sizeof(settings));
    TEST_ASSERT_EQUAL_HEX16(0x1234, protocol);
    TEST_ASSERT_EQUAL_HEX16(0x5678, speed);
}

void test_control_protocol(void) {
    RUN_TEST(test_endian_unaligned_fields);
    RUN_TEST(test_float_wire_bits);
    RUN_TEST(test_crc_known_vectors);
    RUN_TEST(test_frame_golden_vector);
    RUN_TEST(test_frame_all_payload_lengths);
    RUN_TEST(test_frame_rejects_every_single_bit_corruption);
    RUN_TEST(test_frame_rejects_invalid_envelope_even_with_valid_crc);
    RUN_TEST(test_frame_rejects_truncation_trailing_bytes_and_null);
    RUN_TEST(test_frame_encode_rejects_without_writing);
    RUN_TEST(test_settings_decodes_every_offset_unaligned);
    RUN_TEST(test_settings_rejects_nonfinite_at_every_float_offset);
    RUN_TEST(test_settings_rejects_invalid_discrete_fields);
    RUN_TEST(test_settings_power_boundaries_and_duplicate_identifiers);
    RUN_TEST(test_settings_nullspace_count_and_unused_rows);
    RUN_TEST(test_settings_output_suffix_and_board_speed);
    RUN_TEST(test_settings_rejects_wrong_size_and_null_arguments);
}
