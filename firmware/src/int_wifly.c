#include "int_wifly.h"
#include "queue.h"
#include "network_recv.h"
#include "debug.h"

#define INT_WIFLY_QUEUE_LEN 1

QueueHandle_t queue;

CharBuffer send_buffer;
unsigned send_buffer_pos;

CharBuffer recv_buffer;
unsigned recv_buffer_pos;

void wifly_int_init() {
    send_buffer.buff = 0;
    recv_buffer.buff = 0;
    queue = xQueueCreate(INT_WIFLY_QUEUE_LEN, sizeof(CharBuffer));
}

void wifly_int_send(CharBuffer buffer) {
    xQueueSendToBack(queue, &buffer, portMAX_DELAY);
}

WiflyIntCycle wifly_int_cycle() {
    WiflyIntCycle cycle;
    // If we are not currently sending a buffer.
    if (!send_buffer.buff) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        // Attempt to receive a new buffer to send.
        if (xQueueReceiveFromISR(queue, &send_buffer, &higher_priority_task_woken)) {
            cycle.sending = true;
            cycle.item = send_buffer.buff[0];
            send_buffer_pos = 0;
            portEND_SWITCHING_ISR(higher_priority_task_woken);
        // We didn't receive a buffer.
        } else {
            cycle.sending = false;
        }
    // We are still processing something.
    } else {
        cycle.sending = true;
        cycle.item = send_buffer.buff[send_buffer_pos];
    }
    return cycle;
}

void wifly_int_acknowledge() {
    send_buffer_pos++;
    if (send_buffer_pos == send_buffer.length) {
        free(send_buffer.buff);
        send_buffer.buff = 0;
    }
}

void wifly_int_recv_byte(char byte) {
    if (!recv_buffer.buff) {
        recv_buffer.length = (unsigned)byte + 1;
        recv_buffer_pos = 0;
        recv_buffer.buff = malloc(recv_buffer.length);
    } else {
        recv_buffer.buff[recv_buffer_pos++] = byte;
        if (recv_buffer_pos == recv_buffer.length) {
            network_recv_add_buffer_from_isr(recv_buffer);
            recv_buffer.buff = 0;
        }
    }
}
