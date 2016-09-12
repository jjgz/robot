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

#include "wifly_send.h"
#include "int_wifly.h"

#define WIFLY_SEND_QUEUE_LEN 32

QueueHandle_t queue;

void wifly_send_add_buffer(CharBuffer buffer) {
    xQueueSendToBack(queue, &buffer, portMAX_DELAY);
}

void WIFLY_SEND_Initialize() {
    queue = xQueueCreate(WIFLY_SEND_QUEUE_LEN, sizeof(CharBuffer));
}

void WIFLY_SEND_Tasks() {
    while (1) {
        CharBuffer buffer;
        xQueueReceive(queue, &buffer, portMAX_DELAY);
        // TODO: Decide on UDP and TCP and add reliability handling for UDP or remove wifly_recv for TCP.
        // Forward buffer to int_wifly.
        wifly_int_send(buffer);
    }
}
