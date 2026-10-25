#ifndef BOARD_UART_H
#define BOARD_UART_H
#include "fsl_lpuart.h"
/* 1: send hello every second, without starting sensors or motor tasks.
   0: run the normal RT-Thread motor/telemetry application. */
#ifndef TESTFLAG
#define TESTFLAG 0
#endif
/* External 3.3V USB-UART: J5-4/P4_5 TX, J5-3/P4_2 RX. */
LPUART_Type *rt_hw_debug_uart(void);
typedef struct {
    uint32_t source_hz;
    uint32_t baud;
    uint32_t ctrl;
    uint32_t stat;
    uint32_t modir;
    uint32_t rx_pcr;
    uint32_t tx_pcr;
    uint32_t pad_inputs;
    int32_t tx_status;
    uint32_t tx_attempts;
    uint32_t tx_successes;
    uint32_t tx_errors;
    uint32_t global;
    uint32_t pincfg;
    uint32_t fifo;
    uint32_t water;
    uint32_t tx_pad_high_samples;
    uint32_t tx_pad_low_samples;
} host_uart_debug_t;
extern volatile host_uart_debug_t g_host_uart_debug;
void rt_hw_debug_uart_snapshot(void);
#endif
