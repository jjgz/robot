#include "int_wifly.h"
#include "queue.h"
#include "network_recv.h"
#include "debug.h"

#define INT_WIFLY_QUEUE_LEN 32

QueueHandle_t wifly_queue;

CharBuffer send_buffer;
unsigned send_buffer_pos;

CharBuffer recv_buffer;
unsigned recv_buffer_pos;

void wifly_int_init() {
    send_buffer.buff = 0;
    recv_buffer.buff = 0;
    wifly_queue = xQueueCreate(INT_WIFLY_QUEUE_LEN, sizeof(CharBuffer));
}

void wifly_int_send(CharBuffer *buffer) {
    xQueueSendToBack(wifly_queue, buffer, portMAX_DELAY);
}

WiflyIntCycle wifly_int_cycle() {
    WiflyIntCycle cycle;
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (!send_buffer.buff) {
        // Attempt to receive a new buffer to send.
        if (xQueueReceiveFromISR(wifly_queue, &send_buffer, &higher_priority_task_woken)) {
            if (send_buffer.length == 0) {
                cycle.sending = false;
                buffer_free(&send_buffer);
            } else {
                cycle.sending = true;
                cycle.item = send_buffer.buff[0];
                send_buffer_pos = 0;
            }
            portEND_SWITCHING_ISR(higher_priority_task_woken);
        // We didn't receive a buffer.
        } else {
            cycle.sending = false;
        }
    } else {
        cycle.sending = true;
        cycle.item = send_buffer.buff[send_buffer_pos];
    }
    return cycle;
}

/// Acknowledge that a byte was sent out.
void wifly_int_acknowledge_send() {
    send_buffer_pos++;
    // If we reach the end of the message.
    if (send_buffer_pos == send_buffer.length) {
        // Clear the buffer (automatically nulls the pointer).
        buffer_free(&send_buffer);
    }
}

void wifly_int_recv_byte(char byte) {
    if (!recv_buffer.buff) {
        recv_buffer = buffer_new((unsigned)byte + 1);
        recv_buffer_pos = 0;
    } else {
        recv_buffer.buff[recv_buffer_pos++] = byte;
        if (recv_buffer_pos == recv_buffer.length) {
            network_recv_add_buffer_from_isr(&recv_buffer);
            recv_buffer.buff = 0;
        }
    }
}
