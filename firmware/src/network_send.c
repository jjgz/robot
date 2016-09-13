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

#define NETWORK_SEND_QUEUE_LEN 2

QueueHandle_t network_send_queue;

char alpha;

void network_send_add_message(NSMessage *message) {
    xQueueSendToBack(network_send_queue, message, portMAX_DELAY);
}

void NETWORK_SEND_Initialize() {
    network_send_queue = xQueueCreate(NETWORK_SEND_QUEUE_LEN, sizeof(NSMessage));
    alpha = 'a';
}

unsigned count_length_json(char *str) {
    unsigned counter = 0;
    char *current = str;
    while (1) {
        if (*current == '{') {
            counter++;
        } else if (*current == '}') {
            if (counter == 1)
                return current - str + 1;
            if (counter == 0)
                return 0;
            counter--;
        }
        
        current++;
    }
}

void NETWORK_SEND_Tasks() {
    while (1) {
    wifly_int_send(alpha);
    alpha++;
            if (alpha == 'z' + 1)
                alpha = 'a';
    }
    /*
    debug_loc(DEBUG_NETSEND_ENTER);
    NSMessage message;
    debug_loc(DEBUG_NETSEND_WHILE);
    while (1) {
        debug_loc(DEBUG_NETSEND_BEFORE_RECV);
        xQueueReceive(queue, &message, portMAX_DELAY);
        debug_loc(DEBUG_NETSEND_AFTER_RECV);
        switch (message.type) {
            case NS_NETSTATS: {
                MSGNetstats *netstats = &message.data.netstats;
                
                CharBuffer buffer;
                MSGNetstats *netstats = &message.data.netstats;
                debug_loc(DEBUG_NETSEND_BEFORE_PARSE);
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
                debug_loc(DEBUG_NETSEND_BEFORE_STRING);
                char *s = cJSON_PrintUnformatted(root);
                debug_loc((unsigned)s);
                buffer.length = count_length_json(s);
                debug_loc(DEBUG_NETSEND_AFTER_STRLEN);
                buffer.buff = s;
                memcpy(buffer.buff, s, buffer.length);
                cJSON_Delete(root);

                debug_loc(DEBUG_NETSEND_AFTER_SEND);
                
                CharBuffer buffer;
                buffer.length = strlen(cst);
                strcpy(buffer.buff, cst);

                debug_loc(DEBUG_NETSEND_BEFORE_SEND);
                wifly_int_send(&buffer);
            } break;
        }
    }
    */
}
