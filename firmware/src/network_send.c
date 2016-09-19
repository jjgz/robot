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
#include "debug.h"
#include "cJSON/cJSON.h"
#include <stdio.h>

#define NETWORK_SEND_QUEUE_LEN 16
#define MESSAGE_BUF_SIZE 512

QueueHandle_t network_send_queue;

const char *netmsg = "{\"Netstats\":{\"myName\":\"Sensor\",\"numGoodMessagesRecved\":0,\"numCommErrors\":0,\"numJSONRequestsRecved\":0,\"numJSONResponsesRecved\":0,\"numJSONRequestsSent\":0,\"numJSONResponsesSent\":0}}";

char messagebuff[MESSAGE_BUF_SIZE];

void network_send_add_message(NSMessage *message) {
    xQueueSendToBack(network_send_queue, message, portMAX_DELAY);
}

void NETWORK_SEND_Initialize() {
    network_send_queue = xQueueCreate(NETWORK_SEND_QUEUE_LEN, sizeof(NSMessage));
}

void NETWORK_SEND_Tasks() {
    NSMessage message;
    while (1) {
        xQueueReceive(network_send_queue, &message, portMAX_DELAY);
        switch (message.type) {
            case NS_NETSTATS: {
                MSGNetstats *netstats = &message.data.netstats;
                CharBuffer buffer;
                buffer.length = sprintf(messagebuff, "{\"Netstats\":{\"myName\":\"Sensor\",\"numGoodMessagesRecved\":%d,\"numCommErrors\":%d,\"numJSONRequestsRecved\":%d,\"numJSONResponsesRecved\":%d,\"numJSONRequestsSent\":%d,\"numJSONResponsesSent\":%d}}",
                        netstats->numGoodMessagesRecved,
                        netstats->numCommErrors,
                        netstats->numJSONRequestsRecved,
                        netstats->numJSONResponsesRecved,
                        netstats->numJSONRequestsSent,
                        netstats->numJSONResponsesSent);
                
                if (buffer.length > 0) {
                    buffer.buff = messagebuff;

                    int i;
                    for (i = 0; i < buffer.length; i++) {
                        while (1) {
                            if (!DRV_USART0_TransmitBufferIsFull()) {
                                DRV_USART0_WriteByte(buffer.buff[i]);
                                break;
                            }
                        }
                    }
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
                
                //free(cst);
            } break;
            case NS_ADC_READING: {
                MSGAdcReading *adc_reading = &message.data.adc_reading;
                CharBuffer buffer;
                buffer.length = sprintf(messagebuff, "{\"AdcReading\":{\"reading\":%d}}", adc_reading->reading);
                
                if (buffer.length > 0) {
                    buffer.buff = messagebuff;

                    int i;
                    for (i = 0; i < buffer.length; i++) {
                        while (1) {
                            if (!DRV_USART0_TransmitBufferIsFull()) {
                                DRV_USART0_WriteByte(buffer.buff[i]);
                                break;
                            }
                        }
                    }
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            } break;
        }
    }
    
}
