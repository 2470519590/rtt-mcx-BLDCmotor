#include "as5600.h"

#include "board_i2c.h"

#define AS5600_REG_STATUS (0x0BU)
#define AS5600_REG_RAW_ANGLE (0x0CU)
volatile status_t g_as5600_last_i2c_status;
volatile status_t g_as5600_last_i2c_error;

int as5600_init(void)
{
    uint8_t status;
    return as5600_read_status(&status);
}

int as5600_read_status(uint8_t *status)
{
    if (status == NULL) return -1;
    return board_i2c3_read_register(AS5600_I2C_ADDRESS, AS5600_REG_STATUS,
                                    status, 1U) == kStatus_Success ? 0 : -1;
}

int as5600_read_raw(uint16_t *raw)
{
    uint8_t data[2];

    if (raw == NULL) return -1;
    g_as5600_last_i2c_status = board_i2c3_read_register(
        AS5600_I2C_ADDRESS, AS5600_REG_RAW_ANGLE, data, sizeof(data));
    if (g_as5600_last_i2c_status != kStatus_Success) {
        g_as5600_last_i2c_error = g_as5600_last_i2c_status;
        return -1;
    }
    *raw = (uint16_t)((((uint16_t)data[0] << 8U) | data[1]) & 0x0FFFU);
    return 0;
}

float as5600_raw_to_rad(uint16_t raw)
{
    return ((float)(raw & 0x0FFFU) * 6.28318530718f) / 4096.0f;
}
