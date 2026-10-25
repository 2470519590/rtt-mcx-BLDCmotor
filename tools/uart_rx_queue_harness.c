#include "uart_rx_queue.h"
#include <assert.h>

int main(void)
{
    uart_rx_queue_t queue;
    uint32_t generation = 0;
    uint8_t byte;
    uart_rx_queue_reset(&queue);
    for (unsigned turn = 0; turn < 1000; ++turn) {
        uart_rx_queue_push(&queue, 'D');
        assert(uart_rx_queue_pop(&queue, &generation, &byte) == 1 && byte == 'D');
    }
    for (unsigned i = 0; i < UART_RX_QUEUE_SIZE; ++i)
        uart_rx_queue_push(&queue, 'a');
    assert(queue.overflows == 1);
    assert(uart_rx_queue_pop(&queue, &generation, &byte) == -1);
    assert(uart_rx_queue_pop(&queue, &generation, &byte) == 0);
    uart_rx_queue_push(&queue, 'S'); /* Corrupt tail is never executed. */
    uart_rx_queue_push(&queue, '\n');
    uart_rx_queue_push(&queue, 'D');
    assert(uart_rx_queue_pop(&queue, &generation, &byte) == 1 && byte == 'D');
    uart_rx_queue_error(&queue);
    uart_rx_queue_push(&queue, '0');
    uart_rx_queue_push(&queue, '\n');
    uart_rx_queue_push(&queue, 'S');
    assert(uart_rx_queue_pop(&queue, &generation, &byte) == -1);
    assert(uart_rx_queue_pop(&queue, &generation, &byte) == 1 && byte == 'S');
    assert(uart_rx_queue_pop(&queue, &generation, &byte) == 0);
    return 0;
}
