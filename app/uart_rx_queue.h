#ifndef UART_RX_QUEUE_H
#define UART_RX_QUEUE_H

#include <stdint.h>
#define UART_RX_QUEUE_SIZE 256U
typedef struct {
    uint8_t data[UART_RX_QUEUE_SIZE];
    unsigned head, tail;
    uint32_t generation, overflows;
    uint8_t discard_line;
} uart_rx_queue_t;

/* Caller serializes consumer with the UART ISR. No heap/RTOS dependencies. */
void uart_rx_queue_reset(uart_rx_queue_t *queue);
void uart_rx_queue_error(uart_rx_queue_t *queue);
void uart_rx_queue_push(uart_rx_queue_t *queue, uint8_t byte);
/* -1: incomplete command invalidated, 0: empty, 1: byte returned. */
int uart_rx_queue_pop(uart_rx_queue_t *queue, uint32_t *generation, uint8_t *byte);
#endif
