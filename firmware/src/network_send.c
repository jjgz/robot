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

#define NETWORK_SEND_QUEUE_LEN 2

QueueHandle_t network_send_queue;

char alpha;

const char *netmsg = "{\"Netstats\":{\"myName\":\"Sensor\",\"numGoodMessagesRecved\":0,\"numCommErrors\":0,\"numJSONRequestsRecved\":0,\"numJSONResponsesRecved\":0,\"numJSONRequestsSent\":0,\"numJSONResponsesSent\":0}}";

void network_send_add_message(NSMessage *message) {
    xQueueSendToBack(network_send_queue, message, portMAX_DELAY);
}

void NETWORK_SEND_Initialize() {
    network_send_queue = xQueueCreate(NETWORK_SEND_QUEUE_LEN, sizeof(NSMessage));
    alpha = 'a';
}

void NETWORK_SEND_Tasks() {
    debug_loc(DEBUG_NETSEND_ENTER);
    NSMessage message;
    debug_loc(DEBUG_NETSEND_WHILE);
    while (1) {
        debug_loc(DEBUG_NETSEND_BEFORE_RECV);
        xQueueReceive(network_send_queue, &message, portMAX_DELAY);
        debug_loc(DEBUG_NETSEND_AFTER_RECV);
        switch (message.type) {
            case NS_NETSTATS: {
                MSGNetstats *netstats = &message.data.netstats;
                /*debug_loc(DEBUG_NETSEND_BEFORE_PARSE);
                cJSON *root, *netstats_json;
                root = cJSON_CreateObject();
                cJSON_AddItemToObject(root, "Netstats", netstats_json = cJSON_CreateObject());
                cJSON_AddStringToObject(netstats_json, "myName", "Sensor");
                cJSON_AddNumberToObject(netstats_json, "numGoodMessagesRecved", netstats->numGoodMessagesRecved);
                cJSON_AddNumberToObject(netstats_json, "numCommErrors", netstats->numCommErrors);
                cJSON_AddNumberToObject(netstats_json, "numJSONRequestsRecved", netstats->numJSONRequestsRecved);
                cJSON_AddNumberToObject(netstats_json, "numJSONResponsesRecved", netstats->numJSONResponsesRecved);
                cJSON_AddNumberToObject(netstats_json, "numJSONRequestsSent", netstats->numJSONRequestsSent);
                cJSON_AddNumberToObject(netstats_json, "numJSONResponsesSent", netstats->numJSONResponsesSent);
                char *cst = cJSON_PrintUnformatted(root);
                cJSON_Delete(root);*/
                
                //CharBuffer buffer = buffer_new(strlen(netmsg));
                CharBuffer buffer;
                buffer.buff = (char*)netmsg;
                buffer.length = strlen(netmsg);
                //strcpy(buffer.buff, netmsg);
                
                int i;
                for (i = 0; i < buffer.length; i++) {
                    while (1) {
                        if (!DRV_USART0_TransmitBufferIsFull()) {
                            DRV_USART0_WriteByte(buffer.buff[i]);
                            break;
                        }
                    }
                }
            } break;
        }
    }
    
}
