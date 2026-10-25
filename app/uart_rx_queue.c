#include "uart_rx_queue.h"
#include <string.h>

void uart_rx_queue_reset(uart_rx_queue_t *queue)
{
    memset(queue, 0, sizeof(*queue));
}

void uart_rx_queue_error(uart_rx_queue_t *queue)
{
    queue->head = queue->tail = 0U;
    ++queue->generation;
    queue->discard_line = 1U;
}

void uart_rx_queue_push(uart_rx_queue_t *queue, uint8_t byte)
{
    if (queue->discard_line) {
        if (byte == '\n' || byte == '\r') queue->discard_line = 0U;
        return;
    }
    unsigned next = (queue->head + 1U) % UART_RX_QUEUE_SIZE;
    if (next == queue->tail) {
        ++queue->overflows;
        uart_rx_queue_error(queue);
        if (byte == '\n' || byte == '\r') queue->discard_line = 0U;
        return;
    }
    queue->data[queue->head] = byte;
    queue->head = next;
}

int uart_rx_queue_pop(uart_rx_queue_t *queue, uint32_t *generation, uint8_t *byte)
{
    if (*generation != queue->generation) {
        *generation = queue->generation;
        return -1;
    }
    if (queue->head == queue->tail) return 0;
    *byte = queue->data[queue->tail];
    queue->tail = (queue->tail + 1U) % UART_RX_QUEUE_SIZE;
    return 1;
}
