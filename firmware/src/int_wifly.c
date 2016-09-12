#include "int_wifly.h"
#include "queue.h"
#include "system_definitions.h"

#define INT_WIFLY_QUEUE_LEN 1

typedef struct {
    char *buff;
    unsigned length;
} WiflyQueueItem;

QueueHandle_t queue;

WiflyQueueItem current;
unsigned current_pos;

char *receive_buffer;
unsigned receive_buffer_pos;

void wifly_int_init() {
    current.buff = 0;
    receive_buffer = 0;
    queue = xQueueCreate(INT_WIFLY_QUEUE_LEN, sizeof(WiflyQueueItem));
}

void wifly_int_send(char *buff, unsigned length) {
    WiflyQueueItem item;
    item.buff = buff;
    item.length = length;
    xQueueSendToBack(queue, &item, portMAX_DELAY);
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
            return cycle;
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
        current.buff = 0;
    }
}

void wifly_int_recv_byte(char byte) {
    
}
