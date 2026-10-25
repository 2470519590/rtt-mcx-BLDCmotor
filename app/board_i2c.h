#ifndef BOARD_I2C_H
#define BOARD_I2C_H

#include <stddef.h>
#include <stdint.h>
#include "fsl_common.h"

#ifdef __cplusplus
extern "C" {
#endif

void board_i2c3_init(uint32_t baudrate_hz);
status_t board_i2c3_read_register(uint16_t address, uint8_t reg,
                                  uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif
