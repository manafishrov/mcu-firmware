#ifndef AM32_BOOTLOADER_H
#define AM32_BOOTLOADER_H

#include "esc_firmware/update.h"
#include <stdbool.h>
#include <stdint.h>

bool am32_bootloader_flash_all(const uint8_t *image, uint16_t image_size,
                               esc_firmware_update_error_t *error, uint8_t *failed_motor,
                               bool *modified);

#endif
