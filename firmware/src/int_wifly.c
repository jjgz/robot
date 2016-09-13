#include "int_wifly.h"
#include "queue.h"
#include "network_recv.h"
#include "debug.h"

#define INT_WIFLY_QUEUE_LEN 2

QueueHandle_t wifly_queue;

CharBuffer send_buffer;
unsigned send_buffer_pos;

CharBuffer recv_buffer;
unsigned recv_buffer_pos;

void wifly_int_init() {
    send_buffer_pos = 0;
    send_buffer.length = 0;
    recv_buffer_pos = 0;
    recv_buffer.length = 0;
    wifly_queue = xQueueCreate(INT_WIFLY_QUEUE_LEN, 1);
    if (!wifly_queue) {
        SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 1);
    }
}

void wifly_int_send(char a) {
    xQueueSendToBack(wifly_queue, &a, portMAX_DELAY);
}

WiflyIntCycle wifly_int_cycle() {
    WiflyIntCycle cycle;
    BaseType_t higher_priority_task_woken = pdFALSE;
    char a;
    // Attempt to receive a new buffer to send.
    if (xQueueReceiveFromISR(wifly_queue, &a, &higher_priority_task_woken)) {
        cycle.sending = true;
        cycle.item = a;
        portEND_SWITCHING_ISR(higher_priority_task_woken);
    // We didn't receive a buffer.
    } else {
        cycle.sending = false;
    }
    return cycle;
}

void wifly_int_acknowledge() {
    send_buffer_pos++;
}

void wifly_int_recv_byte(char byte) {
    if (recv_buffer.length == recv_buffer_pos) {
        recv_buffer.length = (unsigned)byte + 1;
        recv_buffer_pos = 0;
    } else {
        recv_buffer.buff[recv_buffer_pos++] = byte;
        if (recv_buffer_pos == recv_buffer.length) {
            network_recv_add_buffer_from_isr(&recv_buffer);
        }
    }
}
