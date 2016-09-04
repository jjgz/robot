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

#include "debug.h"
#include "queue.h"

#define DEBUG_QUEUE_LEN 64

QueueHandle_t queue;

void debug_val(unsigned char val) {
    DebugQueueItem item;
    item.type = DEBUG_QUEUE_VAL;
    item.value.val = val;
    xQueueSendToBack(queue, &item, portMAX_DELAY);
}

void debug_val_isr(unsigned char val) {
    DebugQueueItem item;
    item.type = DEBUG_QUEUE_VAL;
    item.value.val = val;
    xQueueSendToBackFromISR(queue, &item, NULL);
}

void debug_loc(DebugLocation loc) {
    DebugQueueItem item;
    item.type = DEBUG_QUEUE_LOC;
    item.value.loc = loc;
    xQueueSendToBack(queue, &item, portMAX_DELAY);
}

void debug_loc_isr(DebugLocation loc) {
    DebugQueueItem item;
    item.type = DEBUG_QUEUE_LOC;
    item.value.loc = loc;
    xQueueSendToBackFromISR(queue, &item, NULL);
}

void debug_halt() {
    DebugQueueItem item;
    item.type = DEBUG_QUEUE_HALT;
    xQueueSendToBack(queue, &item, portMAX_DELAY);
}

void debug_halt_isr(unsigned char val) {
    DebugQueueItem item;
    item.type = DEBUG_QUEUE_HALT;
    xQueueSendToBackFromISR(queue, &item, NULL);
}

void DEBUG_Initialize() {
    queue = xQueueCreate(DEBUG_QUEUE_LEN, sizeof(DebugQueueItem));
    DRV_TMR0_Start();
}

void DEBUG_Tasks() {
    DebugQueueItem item;
    while (1) {
        xQueueReceive(queue, &item, portMAX_DELAY);
        //SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_1, ledtoggle);
        switch (item.type) {
            case DEBUG_QUEUE_VAL:
                // TODO: Write to pins A8-A15.
                break;
            case DEBUG_QUEUE_LOC:
                // TODO: Write to pins A0-A7.
                break;
            case DEBUG_QUEUE_HALT:
                // TODO: Write DEBUG_LOC_HALT to pins A0-A7.
                // Halt.
                while (1) {}
                break;
        }
    }
}
