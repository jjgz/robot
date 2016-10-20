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

#include "processing.h"
#include "network/send.h"
#include "debug.h"

QueueHandle_t processing_queue;

void processing_add_recvmsg(NRMessage *message) {
    PRMessage pr_message;
    pr_message.type = PR_NR;
    pr_message.data.nr_message = *message;
    xQueueSendToBack(processing_queue, &pr_message, portMAX_DELAY);
}

void processing_add_adc_reading(unsigned adc_sample){
    PRMessage pr_adc_message;
    pr_adc_message.type = PR_ADC;
    pr_adc_message.data.adc_sample = adc_sample;
    
    BaseType_t higher_priority_task_woken = pdFALSE;
    // Attempt add the buffer from the isr to the queue.
    if (xQueueSendToBackFromISR(processing_queue, &pr_adc_message, &higher_priority_task_woken)) {
        // If a higher priority task was waiting for something on the queue, switch to it.
        portEND_SWITCHING_ISR(higher_priority_task_woken);
    } 
    // We didn't receive a buffer.
    else {
        // Indicate on LD4 that we lost a packet.
        // NOTE: LD4 conflicts with SDA2 (I2C).
        SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
    }
}

void PROCESSING_Initialize() {
    processing_queue = xQueueCreate(PROCESSING_QUEUE_LEN, sizeof(NRMessage));
}

void PROCESSING_Tasks() {
    // Declares Network Variables
    PRMessage recv_message;
    NSMessage send_message;
    
    // Declares IR Sensor Variables
    bool ON_EDGE = false;
    bool CAUGHT_EDGE = false;
    bool IN_POSIT_GRAB = true;
    bool IN_POSIT_DROP = false;
    bool MAG_ON = false;
    double dist_read = 0.0;
    
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
    
    // ElectroMagnet Port Example
    // MAX32 Pin: 22 / ChipKit Pin: 7 / RC2
    // SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_2, 1);

//  --Start Processing Block
    //--IR Sensor
    // Request: STOPPED_FROM_JOSH
    // T: Start IR sensor readings
    // Send: Edge Detect
    // Send: Edge Drop
    
    //--ElectroMagnet
    //GRABBING
    // Send: Distance from Target
    // Start Electromagnet 
    // Received: REQ IF GRABBED FROM JOSH
    // wait certain time
    // Send: GRABBED
    //DROPPING
    // Received: REQ IF DROPPED FROM JOSH
    // Stop Electromagnet
    // wait certain time
    // Send: DROPPED
    
//  --END Processing Block
    
// We responded to a request, so we increase the responses sent.
    while(1) {  
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
                    case NR_REQ_EDGE_DETECT: {
                        // Detect Increase from IR Sensor
                        if(ON_EDGE) {
                            send_message.data.answer = true;
                        }
                        else {
                            send_message.data.answer = false;
                        }
                        // Send EDGE DETECT to JOSH
                        send_message.type = NS_EDGE_DETECT;
                        network_send_add_message(&send_message);
                    } break;
                    case NR_REQ_EDGE_DROPPED: {
                        // Detect Decrease from IR Sensor
                        if((CAUGHT_EDGE) && (!ON_EDGE)) {
                            send_message.data.answer = true;
                        }
                        else {
                            send_message.data.answer = false;
                        }
                        // Send EDGE DROP to JOSH
                        send_message.type = NS_EDGE_DROPPED;
                        network_send_add_message(&send_message);
                    } break;
                    case NR_REQ_DISTANCE: {               
                        // Send DISTANCE to JOSH
                        send_message.data.distance = dist_read;
                        send_message.type = NS_DISTANCE;
                        network_send_add_message(&send_message);
                    } break;
                    case NR_REQ_GRABBED: {
                        // Turn on ElectroMagnet
                        SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_2, 1);
                        MAG_ON = true;
                        
                        // Wait
                        // :::::::::::TODO::::::::::::
                                                
                        // Send GRABBED to JOSH
                        if(MAG_ON) {
                            send_message.data.answer = true;
                        }
                        else {
                            send_message.data.answer = false;
                        }
                        send_message.type = NS_GRABBED;
                        network_send_add_message(&send_message);
                    } break;
                    case NR_REQ_DROPPED: {
                        // Turn off ElectroMagnet
                        SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_2, 0);
                        MAG_ON = false;
                        
                        // Wait
                        // :::::::::::TODO::::::::::::
                        
                        // Send DROPPED to JOSH
                        if(MAG_ON) {
                            send_message.data.answer = false;
                        }
                        else {
                            send_message.data.answer = true;
                        }
                        send_message.type = NS_DROPPED;
                        network_send_add_message(&send_message);
                    } break;
                    case NR_INVALID_ERROR: {
                        netstats->numCommErrors++;
                    } break;
                    case NR_REQ_NAME: {
                        send_message.type = NS_SEND_NAME_ZACH;
                        network_send_add_message(&send_message);
                    } break;
                    default:
                        break;
                }
            } break;
            case PR_ADC: {
                // ANALOG PIN: A4 / AN4
                unsigned adc_sample = recv_message.data.adc_sample;
                // Check For DISTANCE Reading
                dist_read = adc_sample;
                
                /* DEBUG: SEND EVERY DISTANCE READING
                 * send_message.type = NS_DISTANCE;
                 * send_message.data.distance = adc_sample;
                 * network_send_add_message(&send_message); */
                
                // Testing: TURN ON ELECTROMAGNET IF IN RANGE
                // Testing: CHECK FOR EDGE DETECT / DROP
                if(adc_sample > 100) {
                    // Turn ON: MAG_ON = true;
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_3, 1);
                    // Edge Detected
                    ON_EDGE = true;
                    CAUGHT_EDGE = true;
                }
                else {
                    // Turn OFF: MAG_ON = false;
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_3, 0);
                    // Edge Dropped
                    ON_EDGE = false;
                }

                // Testing: Send EDGE DROP when lose of edge
                if(CAUGHT_EDGE && (!ON_EDGE)) {
                    send_message.type = NS_EDGE_DROPPED;
                    send_message.data.answer = true;
                    network_send_add_message(&send_message);
                    CAUGHT_EDGE = false;
                }
                // Testing: Send EDGE DETECT when within range
                if(ON_EDGE) {
                    send_message.type = NS_EDGE_DETECT;
                    send_message.data.answer = true;
                    network_send_add_message(&send_message);
                }
            } break;
            default:
              break;
        }

    }
}
/*******************************************************************************
 End of File
 */
