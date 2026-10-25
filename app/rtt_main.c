#include "rtthread.h"
#include "board_i2c.h"
#include "as5600.h"
#include "motor_control.h"
#include "host_cli.h"
#include "board_uart.h"

void rt_hw_board_init(void);

#if TESTFLAG
__attribute__((noinline)) void uart_test_send(void)
{
    static const uint8_t hello[] = "hello\r\n";
    ++g_host_uart_debug.tx_attempts;
    LPUART_Type *uart = rt_hw_debug_uart();
    g_host_uart_debug.tx_status = kStatus_Success;
    /* Byte-at-a-time test allows observing the physical TX input buffer while
       the transmitter runs. Never consume DATA or modify the RX path. */
    for (unsigned i = 0; i < sizeof(hello) - 1U; ++i) {
        LPUART_WriteByte(uart, hello[i]);
        uint32_t retries = 1000000U;
        while (!(LPUART_GetStatusFlags(uart) & kLPUART_TransmissionCompleteFlag)) {
            if (GPIO4->PDIR & (1UL << 5))
                ++g_host_uart_debug.tx_pad_high_samples;
            else
                ++g_host_uart_debug.tx_pad_low_samples;
            if (--retries == 0U) {
                g_host_uart_debug.tx_status = kStatus_LPUART_Timeout;
                break;
            }
        }
        if (g_host_uart_debug.tx_status != kStatus_Success) break;
    }
    if (g_host_uart_debug.tx_status == kStatus_Success)
        ++g_host_uart_debug.tx_successes;
    else
        ++g_host_uart_debug.tx_errors;
    rt_hw_debug_uart_snapshot();
}
#endif

int main(void)
{
    rt_hw_interrupt_disable();
    rt_hw_board_init();
#if TESTFLAG
    /* UART-only test: do not initialize I2C/PWM or start the scheduler. */
    while (1) {
        uart_test_send();
        SDK_DelayAtLeastUs(1000000U, SystemCoreClock);
    }
#endif
    board_i2c3_init(400000U);
    (void)as5600_init();
    rt_system_timer_init();
    rt_system_scheduler_init();

    motor_control_init();

    rt_thread_t motor = rt_thread_create("motor", motor_control_thread, RT_NULL,
                                          1536, 8, 1);
    RT_ASSERT(motor != RT_NULL);
    rt_thread_startup(motor);

    rt_thread_t cli = rt_thread_create("cli", host_cli_thread, RT_NULL,
                                       2048, 14, 10);
    RT_ASSERT(cli != RT_NULL);
    rt_thread_startup(cli);

    rt_system_timer_thread_init();
    rt_thread_idle_init();
    rt_system_scheduler_start();

    return 0;
}
