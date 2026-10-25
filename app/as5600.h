#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>

#define AS5600_I2C_ADDRESS (0x36U)

int as5600_init(void);
int as5600_read_raw(uint16_t *raw);
int as5600_read_status(uint8_t *status);
float as5600_raw_to_rad(uint16_t raw);

#endif
