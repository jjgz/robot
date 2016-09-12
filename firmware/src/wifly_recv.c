// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

#include "wifly_recv.h"
#include "network_recv.h"

#define WIFLY_RECV_QUEUE_LEN 32

QueueHandle_t queue;

void wifly_recv_add_buffer_from_isr(CharBuffer buffer) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    // Attempt add the buffer from the isr to the queue.
    if (xQueueSendToBackFromISR(queue, &buffer, &higher_priority_task_woken)) {
        // If a higher priority task was waiting for something on the queue, switch to it.
        portEND_SWITCHING_ISR(higher_priority_task_woken);
    // We didn't receive a buffer.
    } else {
        // Indicate on LD4 that we lost a packet.
        // NOTE: LD4 conflicts with SDA2 (I2C).
        SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
    }
}

void WIFLY_RECV_Initialize() {
    queue = xQueueCreate(WIFLY_RECV_QUEUE_LEN, sizeof(CharBuffer));
}

void WIFLY_RECV_Tasks() {
    CharBuffer buffer;
    while (1) {
        xQueueReceive(queue, &buffer, portMAX_DELAY);
        // TODO: Decide on UDP and TCP and add reliability handling for UDP or remove wifly_recv for TCP.
        // Forward buffer to network_recv.
        network_recv_add_buffer(buffer);
    }
}
