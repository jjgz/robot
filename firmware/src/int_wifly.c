#include "int_wifly.h"
#include "queue.h"

#define INT_WIFLY_QUEUE_LEN 1

QueueHandle_t queue;

CharBuffer current;
unsigned current_pos;

char *receive_buffer;
unsigned receive_buffer_pos;
unsigned receive_buffer_len;

void wifly_int_init() {
    current.buff = 0;
    receive_buffer = 0;
    queue = xQueueCreate(INT_WIFLY_QUEUE_LEN, sizeof(CharBuffer));
}

void wifly_int_send(CharBuffer buffer) {
    xQueueSendToBack(queue, &buffer, portMAX_DELAY);
}

WiflyIntCycle wifly_int_cycle() {
    WiflyIntCycle cycle;
    // If we are not currently sending a buffer.
    if (!current.buff) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        // Attempt to receive a new buffer to send.
        if (xQueueReceiveFromISR(queue, &current, &higher_priority_task_woken)) {
            cycle.sending = true;
            cycle.item = current.buff[0];
            current_pos = 0;
            portEND_SWITCHING_ISR(higher_priority_task_woken);
        // We didn't receive a buffer.
        } else {
            cycle.sending = false;
        }
    // We are still processing something.
    } else {
        cycle.sending = true;
        cycle.item = current.buff[current_pos];
    }
    return cycle;
}

void wifly_int_acknowledge() {
    current_pos++;
    if (current_pos == current.length) {
        free(current.buff);
        current.buff = 0;
    }
}

void wifly_int_recv_byte(char byte) {
    if (!receive_buffer) {
        receive_buffer_len = (unsigned)byte + 1;
        receive_buffer_pos = 0;
        receive_buffer = malloc(receive_buffer_len);
    } else {
        receive_buffer[receive_buffer_pos++] = byte;
        if (receive_buffer_pos == receive_buffer_len) {
            // TODO: Send buffer to wifly_recv.
            receive_buffer = 0;
        }
    }
}
