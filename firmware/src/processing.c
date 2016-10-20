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

#define TEN_CM(a) (750*a)
#define ROTATE(a) (650*a)
extern Pid controller_right;
extern Pid controller_left;
QueueHandle_t processing_queue;
extern QueueHandle_t interrupt_queue;
extern rover my_rover;
extern double wanted_speed_left;
extern double wanted_speed_right;
unsigned output_right_avg;
unsigned output_left_avg;
uint16_t processing_counter;
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
    output_left_avg = 0;
    output_right_avg = 0;
    processing_counter = 0;
    init_rover();
}
void init_rover()
{
    my_rover.ticks.prev_left = 0;
    my_rover.ticks.prev_right = 0;
    my_rover.ticks.tick_left = 0;
    my_rover.ticks.tick_right = 0;
    my_rover.bools.stop_left = false;
    my_rover.bools.stop_right = false;
    my_rover.bools.slow_left = false;
    my_rover.bools.slow_right = false;
    my_rover.bools.got_name = false;
    my_rover.bools.test_move = false;
    my_rover.bools.test_rotate = false;
}
void processing_add_recvmsg(NRMessage *message) {
    PRMessage pr_message;
    pr_message.type = PR_NR;
    pr_message.data.nr_message = *message;
    xQueueSendToBack(processing_queue, &pr_message, portMAX_DELAY);
}
//tmr3 = right
//tmr4 = left
void processing_add_pwm_reading(uint32_t left_pwm, uint32_t right_pwm,uint32_t tmr3, uint32_t tmr4, double left_error, double right_error){
    
    if(my_rover.bools.got_name)
    {
       processing_counter++;
       output_left_avg += left_pwm;
       output_right_avg += right_pwm;

       my_rover.ticks.tick_left += tmr4;
       my_rover.ticks.tick_right += tmr3;
       if(processing_counter == 50)
       {
           processing_counter = 0;
           output_left_avg = output_left_avg/50;
           output_right_avg = output_right_avg/50;
           PRMessage pr_adc_message;
           pr_adc_message.type = PR_PWM;
           pr_adc_message.data.timer.speed_left = output_left_avg;
           pr_adc_message.data.timer.speed_right = output_right_avg;
           pr_adc_message.data.timer.tmr3 = tmr3;
           pr_adc_message.data.timer.tmr4 =  tmr4;
           pr_adc_message.data.timer.left_error = left_error;
           pr_adc_message.data.timer.right_error = right_error;
           output_left_avg = 0;
           output_right_avg = 0;
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
    unsigned blocks = 0;
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
   move_wheels(0,0);
    
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
                        //my_rover.got_name = true;
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
                     }break;
                     case NR_TEST_ROTATE:
                     {
                         my_rover.bools.test_rotate = true;
                         my_rover.debug_test.test_rotate_val = recv_message.data.nr_message.data.rotate_val;
                     }break;
                     case NR_TEST_RESET:
                     {
                        SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 0);
                        init_rover();
                        my_rover.bools.got_name = true;
                        pwm.wanted_speed_right = 0;
                        pwm.wanted_speed_left = 0;
                        interrupt_add_pwm(&pwm);
                     }break;
                     case NR_TEST_MOVE:
                     {
                         my_rover.bools.test_move = true;
                         my_rover.debug_test.test_move_val = recv_message.data.nr_message.data.move_val;
                     }break;
                     //TODO: Add a NR_REQ_TEST_RESET case such that you can reset the values and everything 
                     //you could also send back a send message to get a confirmation that the command was sent..or do it down in the case statement
                     //and just push it in the queue. If you choose to do this part then add it into send.c
                     //TODO2: Eventually you need to add request (NR_REQ_TEST_MOVE or NR_REQ_TEST_ROT) to indicate that you want to run a move test or a reotation test
                     //just set the my_rover state to init and create boolean that you set in this condition and use in the bottom switch() statement.)
                    default:
                        break;
                }       
            } break;
            case PR_PWM:
            {
                if(my_rover.bools.got_name)
                {
                    send_message.type = NS_PWM;

                    send_message.data.tmr.speed_left= recv_message.data.timer.speed_left;
                    send_message.data.tmr.speed_right = recv_message.data.timer.speed_right;
                    send_message.data.tmr.tmr3= my_rover.ticks.tick_right;
                    send_message.data.tmr.tmr4 = my_rover.ticks.tick_left;
                    send_message.data.tmr.left_error = recv_message.data.timer.left_error;
                    send_message.data.tmr.right_error = recv_message.data.timer.right_error;
                    network_send_add_message(&send_message);   
                }
            }break;
            default:
                break;
        }
        switch(my_rover.rover_state)
        {
            case ROVER_INIT:
            {
                my_rover.ticks.tick_right = 0;
                my_rover.ticks.tick_left = 0;
                if(my_rover.bools.got_name && (my_rover.bools.test_rotate || my_rover.bools.test_move)){
                    blocks = 0;
                    my_rover.rover_state = ROVER_MOVE;
                }
            }break;
            case ROVER_MOVE:
            {   
                my_rover.ticks.tick_right = 0;
                my_rover.ticks.tick_left = 0;
                pwm.wanted_speed_right = 2e-1;
                pwm.wanted_speed_left = 2e-1;
                
                //move_wheels(0,1);//left rotate
                
                interrupt_add_pwm(&pwm);
                my_rover.rover_state = ROVER_WAIT;
                if(my_rover.bools.test_rotate){
                    move_wheels(1,0);//right rotate
                    my_rover.rover_state = ROVER_ROTATE_R;
                }
                
            }break;
            case ROVER_WAIT:
            {
//                pwm.wanted_speed_right = 2e-1;
//                pwm.wanted_speed_left = 2e-1;
                
                SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 1);
             
                if(my_rover.ticks.tick_right >= TEN_CM(my_rover.debug_test.test_move_val))
                {                 
                    my_rover.ticks.tick_right = 0;
                    my_rover.bools.stop_right = true;
                }
                else if(my_rover.ticks.tick_right == 130)
                {
                    my_rover.bools.slow_right = true;
                }
                
                if(my_rover.ticks.tick_left >= TEN_CM(my_rover.debug_test.test_move_val))
                {         
                    my_rover.ticks.tick_left= 0;
                    my_rover.bools.stop_left = true;
                }
                else if(my_rover.ticks.tick_left == 130)
                {
                    my_rover.bools.slow_left = true;
                }
                
                
                if(my_rover.bools.slow_right && my_rover.bools.slow_left)
                {
                    pwm.wanted_speed_left = 1.5e-1;
                    pwm.wanted_speed_right = 1.5e-1;
                    my_rover.bools.slow_left = false;
                    my_rover.bools.slow_right = false;
                }
                

                if(my_rover.bools.stop_left && my_rover.bools.stop_right){
                    blocks++;
                    my_rover.rover_state = ROVER_STOP;
                    pwm.wanted_speed_right =0;
                    pwm.wanted_speed_left = 0;   
                    if(blocks < 1)
                        my_rover.rover_state = ROVER_INIT;
                }

                interrupt_add_pwm(&pwm);
            }break;
            case ROVER_ROTATE_R:
            {
                //unsigned rotate = 60;
                if(my_rover.ticks.tick_right >= ROTATE(my_rover.debug_test.test_rotate_val))
                {                 
                    my_rover.ticks.tick_right = 0;
                    my_rover.bools.stop_right = true;
                }
                if(my_rover.ticks.tick_left >= ROTATE(my_rover.debug_test.test_rotate_val))
                {         
                    my_rover.ticks.tick_left= 0;
                    my_rover.bools.stop_left = true;
                }
                if(my_rover.bools.stop_left && my_rover.bools.stop_right){
                    blocks++;
                    my_rover.rover_state = ROVER_STOP;
                    pwm.wanted_speed_right =0;
                    pwm.wanted_speed_left = 0;   
//                    if(blocks < 1)
//                        my_rover.rover_state = ROVER_INIT;
                }
                else
                {
                    pwm.wanted_speed_right = 2e-1;
                    pwm.wanted_speed_left = 2e-1;
                }
                
                interrupt_add_pwm(&pwm);
            }break;
            case ROVER_ROTATE_L:
            {
                //unsigned rotate = 60;
                if(my_rover.ticks.tick_right >= ROTATE(1))
                {                 
                    my_rover.ticks.tick_right = 0;
                    my_rover.bools.stop_right = true;
                }
                if(my_rover.ticks.tick_left >= ROTATE(1))
                {         
                    my_rover.ticks.tick_left= 0;
                    my_rover.bools.stop_left = true;
                }
                if(my_rover.bools.stop_left && my_rover.bools.stop_right){
                    blocks++;
                    my_rover.rover_state = ROVER_STOP;
                    pwm.wanted_speed_right = 0;
                    pwm.wanted_speed_left = 0;   
                    if(blocks < 1)
                        my_rover.rover_state = ROVER_INIT;
                }
                
                interrupt_add_pwm(&pwm);
            }break;
            case ROVER_STOP:
            {
                pwm.wanted_speed_right = 0;
                pwm.wanted_speed_left = 0;
                interrupt_add_pwm(&pwm);
            }break;
            default:
                break;
        }
        
    }
}

void move_wheels(unsigned right, unsigned left)
{
    SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_14, right);
    SYS_PORTS_PinWrite(0, PORT_CHANNEL_G, PORTS_BIT_POS_1, left);
}
/*******************************************************************************
 End of File
 */
