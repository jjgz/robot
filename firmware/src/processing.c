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
#include "pid/pid.h"
extern Pid controller_right;
extern Pid controller_left;
QueueHandle_t processing_queue;
extern QueueHandle_t interrupt_queue;
rover my_rover;
extern double wanted_speed_left;
extern double wanted_speed_right;
void enable_init()
{
    pid_start(&controller_left);
    pid_start(&controller_right);
   // PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_5);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_2);
    DRV_TMR0_Start();
    DRV_TMR1_Start();
    DRV_TMR2_Start();
    DRV_TMR3_Start();
    DRV_OC0_Enable();
    DRV_OC1_Enable();
    DRV_OC0_Start();
    DRV_OC1_Start();
    
    //THIS HAS THEM STOP/////
    wanted_speed_right = 2e-5;
    wanted_speed_left = 2e-5;
    //////////////////////////
    
    my_rover.ticks.prev_left = 0;
    my_rover.ticks.prev_right = 0;
    my_rover.ticks.tick_left = 0;
    my_rover.ticks.tick_right = 0;
    my_rover.rover_state = ROVER_INIT;
    my_rover.stop_left = false;
    my_rover.stop_right = false;
}
void processing_add_recvmsg(NRMessage *message) {
    PRMessage pr_message;
    pr_message.type = PR_NR;
    pr_message.data.nr_message = *message;
    xQueueSendToBack(processing_queue, &pr_message, portMAX_DELAY);
}
//tmr3 = right
//tmr4 = left
void processing_add_pwm_reading(uint32_t left_pwm, uint32_t right_pwm,uint32_t tmr3, uint32_t tmr4){
    PRMessage pr_adc_message;
    pr_adc_message.type = PR_PWM;
    pr_adc_message.data.timer.speed_left = left_pwm;
    pr_adc_message.data.timer.speed_right = right_pwm;
    pr_adc_message.data.timer.tmr3 = tmr3;
    pr_adc_message.data.timer.tmr4 =  tmr4;
    BaseType_t higher_priority_task_woken = pdFALSE;
    // Attempt add the buffer from the isr to the queue.
    if (xQueueSendToBackFromISR(processing_queue, &pr_adc_message, &higher_priority_task_woken)) {
        // If a higher priority task was waiting for something on the queue, switch to it.
        portEND_SWITCHING_ISR(higher_priority_task_woken);
    // We didn't receive a buffer.
    } else {
        // Indicate on LD4 that we lost a packet.
        // NOTE: LD4 conflicts with SDA2 (I2C).
        SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
    }
}

void interrupt_add_pwm(pwm_to_isr *pwm)
{
    pwm_to_isr pwm_val = *pwm;
    xQueueSendToBack(interrupt_queue, &pwm_val, portMAX_DELAY);
}
void PROCESSING_Initialize() {
    enable_init();
    processing_queue = xQueueCreate(PROCESSING_QUEUE_LEN, sizeof(NRMessage));
    interrupt_queue = xQueueCreate(PROCESSING_QUEUE_LEN, sizeof(pwm_to_isr));
}

void PROCESSING_Tasks() {
    PRMessage recv_message;
    NSMessage send_message;
    pwm_to_isr pwm;
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
    
    SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_14, 0);
    SYS_PORTS_PinWrite(0, PORT_CHANNEL_G, PORTS_BIT_POS_1, 0);
    
    
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
                    case NR_INVALID_ERROR:
                    {
                        netstats->numCommErrors++;
                    } break;
                    case NR_REQ_NAME:
                    {
                        send_message.type = NS_SEND_NAME_JOSH;
                        network_send_add_message(&send_message);
                    } break;
                     case NR_REQ_JOSH_POINTS:
                     {
                         send_message.type = NS_JOSH_REQ_POINTS;
                         send_message.data.point.x = 5;
                         send_message.data.point.y = 6;
                         network_send_add_message(&send_message);
                     }break;
                     case NR_REQ_IN_POS:
                     {
                         send_message.type = NS_IN_POS;
                         send_message.data.answer = true;
                         network_send_add_message(&send_message);
                     }
                    default:
                        break;
                }       
            } break;
            case PR_PWM:
            {
                send_message.type = NS_PWM;
                my_rover.ticks.tick_left += recv_message.data.timer.tmr4;
                my_rover.ticks.tick_right += recv_message.data.timer.tmr3;
                
                
                send_message.data.tmr.speed_left= recv_message.data.timer.speed_left;
                send_message.data.tmr.speed_right = recv_message.data.timer.speed_right;
                send_message.data.tmr.tmr3= my_rover.ticks.tick_right;
                send_message.data.tmr.tmr4 = my_rover.ticks.tick_left;
                network_send_add_message(&send_message);
            }break;
            default:
                break;
        }
        unsigned blocks = 0;
        switch(my_rover.rover_state)
        {
            case ROVER_INIT:
            {
                blocks = 0;
                my_rover.rover_state = ROVER_MOVE;
            }break;
            case ROVER_MOVE:
            {
                if(my_rover.stop_left && my_rover.stop_right)
                {
                    my_rover.stop_left = false;
                    my_rover.stop_right = false;
                    my_rover.ticks.tick_left = 0;
                    my_rover.ticks.tick_right = 0;
                }
                pwm.wanted_speed_right = 2e-1;
                pwm.wanted_speed_left = 2e-1;
                interrupt_add_pwm(&pwm);
                //my_rover.rover_state = ROVER_WAIT;
            }break;
            case ROVER_WAIT:
            {
//                pwm.wanted_speed_right = 2e-1;
//                pwm.wanted_speed_left = 2e-1;
                if(my_rover.ticks.tick_right >= 150)
                {
                    pwm.wanted_speed_right = 2e-3;
                }
                else if(my_rover.ticks.tick_right >= 200)
                {
                    my_rover.ticks.tick_right = 0;
                    pwm.wanted_speed_right = 9e-4;
                    my_rover.stop_right = true;
                }
                
                if(my_rover.ticks.tick_left >= 150)
                {
                    pwm.wanted_speed_left = 2e-3;
                }
                else if(my_rover.ticks.tick_left >= 200){
                    pwm.wanted_speed_left = 9e-4;            
                    my_rover.ticks.tick_left= 0;
                    my_rover.stop_left = true;
                }
                if(my_rover.stop_left && my_rover.stop_right){
                    blocks++;
                    my_rover.rover_state = ROVER_STOP;
                    if(blocks < 1)
                        my_rover.rover_state = ROVER_INIT;
                }

                interrupt_add_pwm(&pwm);
            }break;
            case ROVER_STOP:
            {
                
            }break;
            default:
                break;
        }
        
    }
}


/*******************************************************************************
 End of File
 */
