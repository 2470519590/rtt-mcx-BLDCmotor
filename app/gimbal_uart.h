#ifndef GIMBAL_UART_H
#define GIMBAL_UART_H

#include <stddef.h>
#include <stdint.h>

#define GIMBAL_UART_BAUDRATE 115200U
#define GIMBAL_ID_X 0x01U
#define GIMBAL_ID_Y 0x02U

void gimbal_uart_init(void);
int gimbal_uart_send_frame(const uint8_t *frame, size_t length);
int gimbal_uart_receive_frame(uint8_t *frame, size_t capacity, uint32_t timeout_ms);

int gimbal_motor_enable(uint8_t id);
int gimbal_motor_disable(uint8_t id);
int gimbal_motor_set_mode(uint8_t id, uint16_t mode);
int gimbal_motor_set_speed(uint8_t id, int16_t rpm);
int gimbal_motor_set_multi_turn_angle(uint8_t id, int32_t angle_x10);
int gimbal_motor_set_single_turn_angle(uint8_t id, uint16_t angle_x10);
int gimbal_motor_set_acceleration(uint8_t id, uint16_t acceleration);
int gimbal_motor_save(uint8_t id);
int gimbal_motor_zero_multi_turn(uint8_t id);
int gimbal_motor_zero_single_turn(uint8_t id);
int gimbal_motor_set_address(uint8_t id, uint8_t new_id);
int gimbal_motor_request_feedback(uint8_t id, uint8_t type);

#endif
