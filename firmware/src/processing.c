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
#include <math.h>
#include "processing.h"
#include "network/send.h"
#include "debug.h"
#include "int_adc.h"
#include "world/world.h"

typedef enum {
    AS_WAIT_INIT,
    AS_SCANNING,
    AS_SUPPLY_GRID,
} AState;

QueueHandle_t processing_queue;
AState astate;

void processing_add_recvmsg(NRMessage *message) {
    PRMessage pr_message;
    pr_message.type = PR_NR;
    pr_message.data.nr_message = *message;
    xQueueSendToBack(processing_queue, &pr_message, portMAX_DELAY);
}

void processing_add_message(PRMessage *message) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    // Attempt add the buffer from the isr to the queue.
    if (xQueueSendToBackFromISR(processing_queue, message, &higher_priority_task_woken)) {
        // If a higher priority task was waiting for something on the queue, switch to it.
        portEND_SWITCHING_ISR(higher_priority_task_woken);
    // We didn't receive a buffer.
    } else {
        // Indicate on LD4 that we lost a packet.
        // NOTE: LD4 conflicts with SDA2 (I2C).
        SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
    }
}

void PROCESSING_Initialize() {
    astate = AS_WAIT_INIT;
    processing_queue = xQueueCreate(PROCESSING_QUEUE_LEN, sizeof(PRMessage));
    
    DRV_ADC_Open();
    DRV_TMR0_Start();
}

uint8_t grid[128 * 128];

void PROCESSING_Tasks() {
    PRMessage recv_message;
    NSMessage send_message;
    // Create netstats message.
    NSMessage netstats_message;
    netstats_message.type = NS_NETSTATS;
    MSGNetstats *netstats = &netstats_message.data.netstats;
    netstats->numGoodMessagesRecved = 0;
    netstats->numCommErrors = 0;
    netstats->numJSONRequestsRecved = 0;
    netstats->numJSONResponsesRecved = 0;
    netstats->numJSONRequestsSent = 0;
    netstats->numJSONResponsesSent = 0;
    
    unsigned i;
    for (i = 0; i < 128*128; i++)
        grid[i] = ((i / 128 % 2) + (i % 128 % 2) + i) % 100;
    
    
    network_send_add_message(&netstats_message);

    while (1) {
        // We responded to a request, so we increase the responses sent.
        xQueueReceive(processing_queue, &recv_message, portMAX_DELAY);
        
        switch (recv_message.type) {
            case PR_NR:{
                NRMessage *nr_message = &recv_message.data.nr_message;
                 switch (nr_message->type) {
                    case NR_QUERY_STATS: {
                        netstats->numGoodMessagesRecved++;
                        netstats->numJSONRequestsRecved++;
                        network_send_add_message(&netstats_message);

                        // We responded to a request, so we increase the responses sent.
                        netstats->numJSONResponsesSent++;
                    } break;
                     case NR_GD_REQ_PING:
                     {
                         send_message.type = NS_GD_PING;
                        network_send_add_message(&send_message);
                     } break;
                    case NR_INVALID_ERROR:
                    {
                        netstats->numCommErrors++;
                    } break;
                    case NR_REQ_NAME:
                    {
                        send_message.type = NS_SEND_NAME_GEO;
                        network_send_add_message(&send_message);
                    } break;
                    case NR_INITIALIZE:
                    {
                        astate = AS_SCANNING;
                        OrientPoint ra = {{{nr_message->data.initialization.ra.x, nr_message->data.initialization.ra.y}, 0.0f, {0, 0, 0, 0}}, 0.0f, 0.0f};
                        world_init(ra, nr_message->data.initialization.nt,
                                3.0f, (AbsolutePoint*)nr_message->data.initialization.points,
                                nr_message->data.initialization.nb);
                        // Send initialization confirmation after performing init.
                        send_message.type = NS_DEBUG_GEORDON_STR;
                        send_message.data.dbstr = "Got initialize";
                        network_send_add_message(&send_message);
                    } break;
                    case NR_GD_REQ_HALF_ROW:
                    {
                        for (i = 0; i < 64; i++)
                            send_message.data.w_array[i] = grid[recv_message.data.nr_message.data.half_row * 64 + i];
                        send_message.type = NS_GD_HALF_ROW;
                        network_send_add_message(&send_message);
                    } break;
                    default:
                        break;
                }
            } break;
            
            case PR_ADC_SAMPLES:
            {
                float ir_front_right = recv_message.data.adc_samples.ir_front_right;
                float ir_front_left = recv_message.data.adc_samples.ir_front_left;
                float ir_left = recv_message.data.adc_samples.ir_left;
                world_add_front_right_ir_sensor_reading(0.0833333 * (13.0619 - 0.043598 * ir_front_right +
                        0.0000604419 * powf(ir_front_right, 2) - 3.009494425621623e-8 * powf(ir_front_right, 3)));
                world_add_front_left_ir_sensor_reading(0.0833333 * (13.0619 - 0.043598 * ir_front_left +
                        0.0000604419 * powf(ir_front_left, 2) - 3.009494425621623e-8 * powf(ir_front_left, 3)));
                world_add_left_ir_sensor_reading(0.0833333 * (13.0619 - 0.043598 * ir_left +
                        0.0000604419 * powf(ir_left, 2) - 3.009494425621623e-8 * powf(ir_left, 3)));
            } break;
            
            default:
                break;
        }
    }
}
