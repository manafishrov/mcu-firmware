#include "dshot/telemetry_usb.h"
#include "unity/unity.h"

static void test_esc_version_decoder_requires_signature_and_order(void) {
    esc_version_decoder_t decoder = {0};
    char version[ESC_FIRMWARE_VERSION_MAX_LENGTH + 1] = {0};
    const char expected[] = "2.20.0-rc.2";
    uint8_t crc = 0;

    crc ^= (uint8_t)(sizeof(expected) - 1);
    for (uint8_t bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x80u) != 0 ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
    }
    for (size_t i = 0; i < sizeof(expected) - 1; ++i) {
        crc ^= (uint8_t)expected[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80u) != 0 ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }

    TEST_ASSERT_FALSE(
        dshot_telemetry_usb_decode_esc_version(&decoder, DSHOT_TELEMETRY_TYPE_DEBUG2, 2, version));
    TEST_ASSERT_FALSE(dshot_telemetry_usb_decode_esc_version(&decoder, DSHOT_TELEMETRY_TYPE_DEBUG1,
                                                             0xA5, version));
    TEST_ASSERT_FALSE(dshot_telemetry_usb_decode_esc_version(&decoder, DSHOT_TELEMETRY_TYPE_DEBUG2,
                                                             sizeof(expected) - 1, version));
    for (size_t i = 0; i < sizeof(expected) - 1; ++i) {
        TEST_ASSERT_FALSE(dshot_telemetry_usb_decode_esc_version(
            &decoder, DSHOT_TELEMETRY_TYPE_DEBUG3, (uint8_t)expected[i], version));
    }
    TEST_ASSERT_TRUE(dshot_telemetry_usb_decode_esc_version(&decoder, DSHOT_TELEMETRY_TYPE_DEBUG2,
                                                            crc, version));

    TEST_ASSERT_EQUAL_STRING(expected, version);
}

void test_esc_version_telemetry(void) {
    RUN_TEST(test_esc_version_decoder_requires_signature_and_order);
}
