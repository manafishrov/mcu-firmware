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

static void test_esc_version_report_count_tracks_completed_responses(void) {
    const char expected[] = "2.20.0-rc.2";
    const uint8_t expected_crc = 0x2D;
    dshot_telemetry_context_t context = {.controller_base_global_id = 0};

    dshot_telemetry_usb_init();
    TEST_ASSERT_EQUAL_UINT8(0, dshot_telemetry_usb_esc_versions_reported_count());

    dshot_telemetry_callback(&context, 0, DSHOT_TELEMETRY_TYPE_DEBUG1, 0xA5);
    dshot_telemetry_callback(&context, 0, DSHOT_TELEMETRY_TYPE_DEBUG2, sizeof(expected) - 1);
    for (size_t i = 0; i < sizeof(expected) - 1; ++i) {
        dshot_telemetry_callback(&context, 0, DSHOT_TELEMETRY_TYPE_DEBUG3, (uint8_t)expected[i]);
    }
    dshot_telemetry_callback(&context, 0, DSHOT_TELEMETRY_TYPE_DEBUG2, expected_crc);

    TEST_ASSERT_EQUAL_UINT8(1, dshot_telemetry_usb_esc_versions_reported_count());
    TEST_ASSERT_FALSE(dshot_telemetry_usb_all_esc_versions_reported());
}

void test_esc_version_telemetry(void) {
    RUN_TEST(test_esc_version_decoder_requires_signature_and_order);
    RUN_TEST(test_esc_version_decoder_rejects_invalid_payload_bytes);
    RUN_TEST(test_esc_version_decoder_rejects_bad_crc);
    RUN_TEST(test_esc_version_report_count_tracks_completed_responses);
}
