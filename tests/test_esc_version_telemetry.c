#include "dshot/telemetry_usb.h"
#include "unity/unity.h"

static void test_esc_version_decoder_requires_signature_and_order(void) {
    esc_version_decoder_t decoder = {0};
    char version[ESC_FIRMWARE_VERSION_MAX_LENGTH + 1] = {0};
    const char expected[] = "2.20.0-rc.2";
    const uint8_t expected_crc = 0x2D;

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
                                                            expected_crc, version));

    TEST_ASSERT_EQUAL_STRING(expected, version);
}

static void test_esc_version_decoder_rejects_invalid_payload_bytes(void) {
    esc_version_decoder_t decoder = {0};
    char version[ESC_FIRMWARE_VERSION_MAX_LENGTH + 1] = {0};

    TEST_ASSERT_FALSE(dshot_telemetry_usb_decode_esc_version(&decoder, DSHOT_TELEMETRY_TYPE_DEBUG1,
                                                             0xA5, version));
    TEST_ASSERT_FALSE(
        dshot_telemetry_usb_decode_esc_version(&decoder, DSHOT_TELEMETRY_TYPE_DEBUG2, 1, version));
    TEST_ASSERT_FALSE(
        dshot_telemetry_usb_decode_esc_version(&decoder, DSHOT_TELEMETRY_TYPE_DEBUG3, 0, version));
    TEST_ASSERT_EQUAL_UINT8(0, decoder.stage);

    TEST_ASSERT_FALSE(dshot_telemetry_usb_decode_esc_version(&decoder, DSHOT_TELEMETRY_TYPE_DEBUG1,
                                                             0xA5, version));
    TEST_ASSERT_FALSE(
        dshot_telemetry_usb_decode_esc_version(&decoder, DSHOT_TELEMETRY_TYPE_DEBUG2, 1, version));
    TEST_ASSERT_FALSE(dshot_telemetry_usb_decode_esc_version(&decoder, DSHOT_TELEMETRY_TYPE_DEBUG3,
                                                             0x100, version));
    TEST_ASSERT_EQUAL_UINT8(0, decoder.stage);
}

static void test_esc_version_decoder_rejects_bad_crc(void) {
    esc_version_decoder_t decoder = {0};
    char version[ESC_FIRMWARE_VERSION_MAX_LENGTH + 1] = {0};
    const char expected[] = "2.20.0-rc.2";

    TEST_ASSERT_FALSE(dshot_telemetry_usb_decode_esc_version(&decoder, DSHOT_TELEMETRY_TYPE_DEBUG1,
                                                             0xA5, version));
    TEST_ASSERT_FALSE(dshot_telemetry_usb_decode_esc_version(&decoder, DSHOT_TELEMETRY_TYPE_DEBUG2,
                                                             sizeof(expected) - 1, version));
    for (size_t i = 0; i < sizeof(expected) - 1; ++i) {
        TEST_ASSERT_FALSE(dshot_telemetry_usb_decode_esc_version(
            &decoder, DSHOT_TELEMETRY_TYPE_DEBUG3, (uint8_t)expected[i], version));
    }
    TEST_ASSERT_FALSE(dshot_telemetry_usb_decode_esc_version(&decoder, DSHOT_TELEMETRY_TYPE_DEBUG2,
                                                             0x2C, version));
    TEST_ASSERT_EQUAL_UINT8(0, decoder.stage);
}

void test_esc_version_telemetry(void) {
    RUN_TEST(test_esc_version_decoder_requires_signature_and_order);
    RUN_TEST(test_esc_version_decoder_rejects_invalid_payload_bytes);
    RUN_TEST(test_esc_version_decoder_rejects_bad_crc);
}
