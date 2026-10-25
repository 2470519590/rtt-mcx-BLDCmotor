#include "gimbal_uart.h"

#include "rtthread.h"
#include "fsl_clock.h"
#include "fsl_lpuart.h"
#include "fsl_port.h"
#include "fsl_reset.h"

#define GIMBAL_UART_BASE LPUART3
#define GIMBAL_FRAME_MAX 10U

static uint8_t gimbal_uart_ready;

static uint8_t gimbal_bcc(const uint8_t *data, size_t length)
{
    uint8_t bcc = 0U;
    for (size_t i = 0U; i < length; ++i)
    {
        bcc ^= data[i];
    }
    return bcc;
}

void gimbal_uart_init(void)
{
    lpuart_config_t config;

    /* FRDM-MCXA346 mikroBUS J5: pin 3=P4_2/LPUART3_RXD,
       pin 4=P4_5/LPUART3_TXD, pin 8=GND. */
    CLOCK_EnableClock(kCLOCK_GatePORT4);
    RESET_ReleasePeripheralReset(kPORT4_RST_SHIFT_RSTn);
    PORT_SetPinMux(PORT4, 2U, kPORT_MuxAlt3);
    PORT_SetPinMux(PORT4, 5U, kPORT_MuxAlt3);
    PORT4->PCR[2] |= PORT_PCR_IBE_MASK;
    PORT4->PCR[5] |= PORT_PCR_IBE_MASK;

    CLOCK_SetClockDiv(kCLOCK_DivLPUART3, 1U);
    CLOCK_AttachClk(kFRO_LF_DIV_to_LPUART3);
    RESET_PeripheralReset(kLPUART3_RST_SHIFT_RSTn);

    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = GIMBAL_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;
    LPUART_Init(GIMBAL_UART_BASE, &config, 12000000U);
    gimbal_uart_ready = 1U;
}

int gimbal_uart_send_frame(const uint8_t *frame, size_t length)
{
    if (!gimbal_uart_ready || frame == NULL || length == 0U || length > GIMBAL_FRAME_MAX)
    {
        return -1;
    }
    LPUART_WriteBlocking(GIMBAL_UART_BASE, frame, length);
    return 0;
}

int gimbal_uart_receive_frame(uint8_t *frame, size_t capacity, uint32_t timeout_ms)
{
    rt_tick_t deadline;
    size_t length = 0U;
    uint8_t byte;

    if (!gimbal_uart_ready || frame == NULL || capacity < 5U)
    {
        return -1;
    }

    deadline = rt_tick_get() + rt_tick_from_millisecond(timeout_ms);
    while ((rt_int32_t)(rt_tick_get() - deadline) < 0)
    {
        if ((LPUART_GetStatusFlags(GIMBAL_UART_BASE) & kLPUART_RxDataRegFullFlag) == 0U)
        {
            continue;
        }

        byte = LPUART_ReadByte(GIMBAL_UART_BASE);
        if (length == 0U)
        {
            if (byte != 0x7AU)
            {
                continue;
            }
        }
        else if (length >= capacity)
        {
            length = 0U;
            continue;
        }

        frame[length++] = byte;
        if (byte == 0x7BU && length >= 5U)
        {
            if (frame[length - 2U] == gimbal_bcc(frame, length - 2U))
            {
                return (int)length;
            }
            length = 0U;
        }
    }
    return 0;
}

static int gimbal_send_short(uint8_t id, uint8_t function)
{
    uint8_t frame[5] = {0x7AU, id, function, 0U, 0x7BU};
    frame[3] = gimbal_bcc(frame, 3U);
    return gimbal_uart_send_frame(frame, sizeof(frame));
}

int gimbal_motor_enable(uint8_t id) { return gimbal_send_short(id, 0x06U); }
int gimbal_motor_disable(uint8_t id) { return gimbal_send_short(id, 0x05U); }
int gimbal_motor_save(uint8_t id) { return gimbal_send_short(id, 0x08U); }
int gimbal_motor_zero_multi_turn(uint8_t id) { return gimbal_send_short(id, 0x09U); }
int gimbal_motor_zero_single_turn(uint8_t id) { return gimbal_send_short(id, 0x0AU); }

int gimbal_motor_set_mode(uint8_t id, uint16_t mode)
{
    uint8_t frame[7] = {0x7AU, id, 0x00U, (uint8_t)(mode >> 8), (uint8_t)mode, 0U, 0x7BU};
    frame[5] = gimbal_bcc(frame, 5U);
    return gimbal_uart_send_frame(frame, sizeof(frame));
}

int gimbal_motor_set_speed(uint8_t id, int16_t rpm)
{
    uint16_t value = (uint16_t)rpm;
    uint8_t frame[7] = {0x7AU, id, 0x01U, (uint8_t)(value >> 8), (uint8_t)value, 0U, 0x7BU};
    frame[5] = gimbal_bcc(frame, 5U);
    return gimbal_uart_send_frame(frame, sizeof(frame));
}

int gimbal_motor_set_single_turn_angle(uint8_t id, uint16_t angle_x10)
{
    if (angle_x10 > 3599U) return -2;
    uint8_t frame[7] = {0x7AU, id, 0x03U, (uint8_t)(angle_x10 >> 8), (uint8_t)angle_x10, 0U, 0x7BU};
    frame[5] = gimbal_bcc(frame, 5U);
    return gimbal_uart_send_frame(frame, sizeof(frame));
}

int gimbal_motor_set_multi_turn_angle(uint8_t id, int32_t angle_x10)
{
    uint32_t value = (uint32_t)angle_x10;
    uint8_t frame[9] = {0x7AU, id, 0x02U, (uint8_t)(value >> 24), (uint8_t)(value >> 16),
                        (uint8_t)(value >> 8), (uint8_t)value, 0U, 0x7BU};
    frame[7] = gimbal_bcc(frame, 7U);
    return gimbal_uart_send_frame(frame, sizeof(frame));
}

int gimbal_motor_set_acceleration(uint8_t id, uint16_t acceleration)
{
    uint8_t frame[7] = {0x7AU, id, 0x07U, (uint8_t)(acceleration >> 8), (uint8_t)acceleration, 0U, 0x7BU};
    frame[5] = gimbal_bcc(frame, 5U);
    return gimbal_uart_send_frame(frame, sizeof(frame));
}

int gimbal_motor_set_address(uint8_t id, uint8_t new_id)
{
    uint8_t frame[6] = {0x7AU, id, 0x0DU, new_id, 0U, 0x7BU};
    frame[4] = gimbal_bcc(frame, 4U);
    return gimbal_uart_send_frame(frame, sizeof(frame));
}

int gimbal_motor_request_feedback(uint8_t id, uint8_t type)
{
    uint8_t frame[6] = {0x7AU, id, 0x0EU, type, 0U, 0x7BU};
    frame[4] = gimbal_bcc(frame, 4U);
    return gimbal_uart_send_frame(frame, sizeof(frame));
}
