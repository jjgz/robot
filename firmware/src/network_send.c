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

#include "network_send.h"
#include "int_wifly.h"
#include "int_adc.h"
#include "debug.h"
#include "cJSON/cJSON.h"
#include <stdio.h>

#define NETWORK_SEND_QUEUE_LEN 16
#define MESSAGE_BUF_SIZE 512
#define TOTAL_MESSAGE_BUFFS 4

QueueHandle_t network_send_queue;

char messagebuffs[TOTAL_MESSAGE_BUFFS][MESSAGE_BUF_SIZE];
char *messagebuff;
unsigned choose_buff;

void network_send_add_message(NSMessage *message) {
    xQueueSendToBack(network_send_queue, message, portMAX_DELAY);
}

void network_send_add_message_isr(NSMessage *message) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    // Attempt add the buffer from the isr to the queue.
    if (xQueueSendToBackFromISR(network_send_queue, message, &higher_priority_task_woken)) {
        // If a higher priority task was waiting for something on the queue, switch to it.
        portEND_SWITCHING_ISR(higher_priority_task_woken);
    // We didn't receive a buffer.
    } else {
        // Indicate on LD4 that we lost a packet.
        // NOTE: LD4 conflicts with SDA2 (I2C).
        SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
    }
}

void next_messagebuff() {
    messagebuff = messagebuffs[(choose_buff++) % TOTAL_MESSAGE_BUFFS];
}

void NETWORK_SEND_Initialize() {
    messagebuff = messagebuffs[0];
    choose_buff = 0;
    network_send_queue = xQueueCreate(NETWORK_SEND_QUEUE_LEN, sizeof(NSMessage));
    int_adc_init();
    DRV_ADC_Open();
}

void NETWORK_SEND_Tasks() {
    NSMessage message;
    CharBuffer buffer;
    while (1) {
        xQueueReceive(network_send_queue, &message, portMAX_DELAY);
        switch (message.type) {
            case NS_NETSTATS: {
                MSGNetstats *netstats = &message.data.netstats;
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"Netstats\":{\"myName\":\"Sensor\",\"numGoodMessagesRecved\":%d,\"numCommErrors\":%d,\"numJSONRequestsRecved\":%d,\"numJSONResponsesRecved\":%d,\"numJSONRequestsSent\":%d,\"numJSONResponsesSent\":%d}}",
                        netstats->numGoodMessagesRecved,
                        netstats->numCommErrors,
                        netstats->numJSONRequestsRecved,
                        netstats->numJSONResponsesRecved,
                        netstats->numJSONRequestsSent,
                        netstats->numJSONResponsesSent);
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            } break;
            case NS_ADC_READING: {
                MSGAdcReading *adc_reading = &message.data.adc_reading;
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"AdcReading\":{\"reading\":%d}}", adc_reading->reading);
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            } break;
        }
    }
    
}
