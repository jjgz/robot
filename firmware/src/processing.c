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
#include "peripheral/oc/plib_oc.h"

#define TEN_CM(a) (60*a)
#define ROTATE(a) (52*a)
extern Pid pid_right;
extern Pid pid_left;
QueueHandle_t processing_queue;
extern QueueHandle_t interrupt_queue;
leader rover;
int processing_counter;
extern double target_right_spd;
extern double target_left_spd;
unsigned output_left_avg;
unsigned output_right_avg;

<<<<<<< HEAD
void rover_init(){
    rover.ticks.prev_left = 0;
    rover.ticks.prev_right = 0;
    rover.ticks.t_left = 0;
    rover.ticks.t_right = 0;
    rover.stop_left = false;
    rover.slow_right = false;
    rover.slow_left = false;
    rover.stop_right = false;
    rover.got_cmnd = false;
}
=======
bool map[128][128];

>>>>>>> parent of ac911b9... changed where my case block for rover movement was and made adjustments to speed of rover.  As of now rover moves with decent accuracy on straight movement and turns as well.
void enable_start(){
    pid_start(&pid_right);
    pid_start(&pid_left);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_2);
    DRV_TMR0_Start();
    DRV_TMR1_Start();
    DRV_TMR2_Start();
    DRV_TMR3_Start();
    DRV_OC0_Enable();
    DRV_OC1_Enable();
    DRV_OC0_Start();
    DRV_OC1_Start();
    rover.ticks.prev_left = 0;
    rover.ticks.prev_right = 0;
    rover.ticks.t_left = 0;
    rover.ticks.t_right = 0;
    rover.stop_left = false;
    rover.slow_right = false;
    rover.slow_left = false;
    rover.stop_right - false;
    //rover.last_rmotor = 0;
    //rover.time_l = 0;
    //rover.time_r = 0;
    target_right_spd = 2e-5;
    target_left_spd = 2e-5;
}

void processing_add_recvmsg(NRMessage *message) {
    PRMessage pr_message;
    pr_message.type = PR_NR;
    pr_message.data.nr_message = *message;
    xQueueSendToBack(processing_queue, &pr_message, portMAX_DELAY);
}

<<<<<<< HEAD
/*void processing_change_rover_state(uint32_t timer_state){
    PRMessage pr_pwm_message;
    pr_pwm_message.type = PR_TMR;
    pr_pwm_message.data.timer.tmr_state = timer_state;
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (xQueueSendToBackFromISR(processing_queue, &pr_pwm_message, &higher_priority_task_woken)) {
                portEND_SWITCHING_ISR(higher_priority_task_woken);
            }   
            else {
                SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
            }
}*/

void processing_add_pwm_reading(uint32_t left_pwm, uint32_t right_pwm, uint32_t tmr3, uint32_t tmr4){
    if(rover.got_cmnd)    {
        output_right_avg += right_pwm;
        output_left_avg += left_pwm;
        rover.ticks.t_left += tmr4;
        rover.ticks.t_right += tmr3;
        processing_counter++;
        if(processing_counter == 50){
            processing_counter = 0;
            PRMessage pr_pwm_message;
            output_right_avg = output_right_avg/50;
            output_left_avg = output_left_avg/50;
            pr_pwm_message.type = PR_PWM;
            pr_pwm_message.data.timer.r_spd = output_left_avg;
            pr_pwm_message.data.timer.l_spd = output_right_avg;
            pr_pwm_message.data.timer.tmr3 = tmr3;
            pr_pwm_message.data.timer.tmr4 = tmr4;
            output_left_avg = 0;
            output_right_avg = 0;
            BaseType_t higher_priority_task_woken = pdFALSE;
            if (xQueueSendToBackFromISR(processing_queue, &pr_pwm_message, &higher_priority_task_woken)) {
                portEND_SWITCHING_ISR(higher_priority_task_woken);
            }   
            else {
                SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
            }
        }
    }
=======
void processing_add_adc_reading(unsigned adc_sample){
    PRMessage pr_adc_message;
    pr_adc_message.type = PR_ADC;
    pr_adc_message.data.adc_sample = adc_sample;
    
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

void processing_add_pwm_reading(uint32_t left_pwm, uint32_t right_pwm, uint32_t tmr3, uint32_t tmr4){
    PRMessage pr_pwm_message;
    
    
    output_right_avg += right_pwm;
    output_left_avg += left_pwm;
    rover.ticks.t_left += tmr4;
    rover.ticks.t_right += tmr3;
    
    processing_counter++;
    if(processing_counter == 50){
        processing_counter = 0;
        output_right_avg = pr_pwm_message.data.timer.right_spd_avg/50;
        output_left_avg = pr_pwm_message.data.timer.left_spd_avg/50;
        pr_pwm_message.type = PR_PWM;
        pr_pwm_message.data.timer.r_spd = output_left_avg;
        pr_pwm_message.data.timer.l_spd = output_right_avg;
        pr_pwm_message.data.timer.tmr3 = tmr3;
        pr_pwm_message.data.timer.tmr4 = tmr4;
    
        BaseType_t higher_priority_task_woken = pdFALSE;
    
        if (xQueueSendToBackFromISR(processing_queue, &pr_pwm_message, &higher_priority_task_woken)) {
            portEND_SWITCHING_ISR(higher_priority_task_woken);
        }   
        else {
            SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
        }         
    }    
>>>>>>> parent of ac911b9... changed where my case block for rover movement was and made adjustments to speed of rover.  As of now rover moves with decent accuracy on straight movement and turns as well.
}
void interrupt_add_pwm(pwm_to_isr * pwm){
    pwm_to_isr p_val = *pwm;
    xQueueSendToBack(interrupt_queue, &p_val, portMAX_DELAY);
}

void leader_move(unsigned right, unsigned left){
    SYS_PORTS_PinWrite(0,PORT_CHANNEL_D, PORTS_BIT_POS_1,1);
    SYS_PORTS_PinWrite(0,PORT_CHANNEL_D, PORTS_BIT_POS_0,1);
    SYS_PORTS_PinWrite(0,PORT_CHANNEL_C, PORTS_BIT_POS_14, right);
    SYS_PORTS_PinWrite(0,PORT_CHANNEL_G, PORTS_BIT_POS_1, left);
}

void PROCESSING_Initialize() {
    enable_start();
    processing_queue = xQueueCreate(PROCESSING_QUEUE_LEN, sizeof(PRMessage));
    interrupt_queue = xQueueCreate(PROCESSING_QUEUE_LEN, sizeof(pwm_to_isr));
    
}

void PROCESSING_Tasks() {
    PRMessage recv_message;
    NSMessage send_message;
    pwm_to_isr pwm;
    int counter = 0;
    double prev_left_photo = 0;
    double prev_right_photo = 0;
    bool left;
    bool right;
    NSMessage netstats_message;
    netstats_message.type = NS_NETSTATS;
    MSGNetstats *netstats = &netstats_message.data.netstats;
    netstats->numGoodMessagesRecved = 0;
    netstats->numCommErrors = 0;
    netstats->numJSONRequestsRecved = 0;
    netstats->numJSONResponsesRecved = 0;
    netstats->numJSONRequestsSent = 0;
    netstats->numJSONResponsesSent = 0;
    //leader_move(0,0);
    
    
    //rover.lead_state = LEADER_MOVE;
    
    //SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_14, 1);
    //SYS_PORTS_PinWrite(0, PORT_CHANNEL_G, PORTS_BIT_POS_1, 1);
    
    network_send_add_message(&netstats_message);

    while (1) {
        // We responded to a request, so we increase the responses sent.
        xQueueReceive(processing_queue, &recv_message, portMAX_DELAY);
        //recv_message.type = PR_DEBUG;
        switch (recv_message.type) {
            case PR_NR:{
                NRMessage *nr_message = &recv_message.data.nr_message;
                 switch (nr_message->type) {
                    case NR_QUERY_STATS: {
                        //MSGQueryStats *stats = &nr_message->data.query_stats;
                        netstats->numGoodMessagesRecved++;
                        netstats->numJSONRequestsRecved++;
                        network_send_add_message(&netstats_message);
                        //SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 1);
                        

                        // We responded to a request, so we increase the responses sent.
                        netstats->numJSONResponsesSent++;
                    } break;
                    case NR_INVALID_ERROR:
                    {
                        netstats->numCommErrors++;
                    } break;
                    case NR_REQ_NAME:
                    {
                        send_message.type = NS_SEND_NAME_JOE;
                        network_send_add_message(&send_message);
                        //SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 1);
                        
                    } break;
                     /*case NR_REQ_MOVEMENT:
                     {
                         /*send_message.type = NS_MOVEMENT;
                         send_message.data.ldr_m.dist_x = rover.ldr_m.dist_x;
                         send_message.data.ldr_m.dist_y = rover.ldr_m.dist_y;
                         send_message.data.ldr_m.turn_angle = rover.ldr_m.turn_angle;
                         send_message.data.ldr_m.angle_var = rover.ldr_m.angle_var;
                         send_message.data.ldr_m.move_var = rover.ldr_m.move_var;
                         network_send_add_message(&send_message);
                         
                     }break;*/
                     /*case NR_SENSORS:
                     {
                        /*send_message.type = NS_MOVEMENT;
                        send_message.data.ldr_m.dist_x = rover.ldr_m.dist_x;
                        send_message.data.ldr_m.dist_y = rover.ldr_m.dist_y;
                        send_message.data.ldr_m.turn_angle = rover.ldr_m.turn_angle;
                        send_message.data.ldr_m.angle_var = rover.ldr_m.angle_var;
                        send_message.data.ldr_m.move_var = rover.ldr_m.move_var;
                        network_send_add_message(&send_message);
                        rover_init();
                        double ul = 3.0;
                        double lp = 30.0;
                        double rp = 30.0;
                        rover.lead_state = LEADER_INIT;
                        rover.sensors.prev_lphoto = prev_left_photo;
                        rover.sensors.prev_rphoto = prev_right_photo;
                        rover.sensors.ultra = nr_message->data.ult_photo.ultra;
                        rover.sensors.l_photo = nr_message->data.ult_photo.l_photo;
                        rover.sensors.r_photo = nr_message->data.ult_photo.r_photo;
                        //leader_state_change(ul, lp, rp,rover.sensors.prev_lphoto, rover.sensors.prev_rphoto, rover.thresholds.dist_thresh, rover.thresholds.border_thresh);
                        //leader_state_change(rover.sensors.ultra, rover.sensors.l_photo, rover.sensors.r_photo,rover.sensors.prev_lphoto, rover.sensors.prev_rphoto, rover.thresholds.dist_thresh, rover.thresholds.border_thresh);
                        rover.got_cmnd = true;
                     }break;*/
                    case NR_DEBUG_JOE_TREAD:
                    {
<<<<<<< HEAD
                        //left = nr_message->data.debug_joe_tread.left;
                        //right = nr_message->data.debug_joe_tread.right;
                        /*rover_init();
                        pwm.target_left_spd = 0;
                        pwm.target_right_spd = 0;
                        rover.got_cmnd = true;
                        rover.lead_state = LEADER_INIT;  
                        rover.ticks.t_right = 0;
                        rover.ticks.t_left = 0;
                        interrupt_add_pwm(&pwm);*/
=======
                        left = nr_message->data.debug_joe_tread.left;
                        right = nr_message->data.debug_joe_tread.right;
                        rover.lead_state = LEADER_INIT;
                        switch(rover.lead_state){
                            case(LEADER_INIT):
                            {
                                if(counter == 0){
                                    rover.ticks.t_right = 0;
                                    rover.ticks.t_left = 0;
                                    pwm.target_left_spd = 2e-1;
                                    pwm.target_right_spd = 2e-1;
                                } 
                                if((left == 0) && (right == 0)){
                                    rover.lead_state = LEADER_MOVE;
                                    leader_move(0,0);
                                }
                                if((left == 1) && (right == 0)){
                                    leader_move(1,0);
                                    rover.lead_state = LEADER_LEFT;
                                }
                                if((left == 0) && (right == 1)){
                                    leader_move(0,1);
                                    rover.lead_state = LEADER_RIGHT;
                                }
                                if((left == 1) && (right== 01)){
                                    leader_move(1,1);
                                    rover.lead_state = LEADER_BACK;
                                }
                                counter++;
                                interrupt_add_pwm(&pwm);
                            }break;
                            case(LEADER_MOVE):
                            {
                                rover.ticks.t_right = 0;
                                rover.ticks.t_left = 0;
                                pwm.target_left_spd = 2e-1;
                                pwm.target_right_spd = 2e-1;
                                interrupt_add_pwm(&pwm);
                                rover.lead_state = LEADER_WAIT;
                            }break;
                            case(LEADER_BACK):
                            {
                                rover.ticks.t_right = 0;
                                rover.ticks.t_left = 0;
                                pwm.target_left_spd = -(2e-1);
                                pwm.target_right_spd = -(2e-1);
                                interrupt_add_pwm(&pwm);
                                rover.lead_state = LEADER_WAIT;
                            }
                            case(LEADER_LEFT):
                            {
                                leader_move(0,1);
                                pwm.target_left_spd = -(2e-1);
                                pwm.target_right_spd = (2e-1);
                                if(rover.ticks.t_right == ROTATE(2))
                                {
                                    rover.ticks.t_right = 0;
                                    rover.stop_right = true;
                                }
                                if(rover.ticks.t_left == ROTATE(2))
                                {       
                                    rover.ticks.t_left = 0;
                                    rover.stop_left = true;
                                }
                                if(rover.stop_left && rover.stop_right)
                                {
                                    rover.lead_state = LEADER_STOP;
                                    pwm.target_left_spd = 0;
                                    pwm.target_right_spd = 0;
                                }
                                interrupt_add_pwm(&pwm);
                            }break;
                            case(LEADER_RIGHT):
                            {
                                leader_move(1,0);
                                pwm.target_left_spd = (2e-1);
                                pwm.target_right_spd = -(2e-1);
                                if(rover.ticks.t_right == ROTATE(2))
                                {
                                    rover.ticks.t_right = 0;
                                    rover.stop_right = true;
                                }
                                if(rover.ticks.t_left == ROTATE(2))
                                {       
                                    rover.ticks.t_left = 0;
                                    rover.stop_left = true;
                                }
                                if(rover.stop_left && rover.stop_right)
                                {
                                    rover.lead_state = LEADER_STOP;
                                    pwm.target_left_spd = 0;
                                    pwm.target_right_spd = 0;
                                }
                                interrupt_add_pwm(&pwm);
                            }break;
                            case(LEADER_WAIT):
                            {               
                                if(rover.ticks.t_right == TEN_CM(2))
                                {
                                    rover.ticks.t_right = 0;
                                    rover.stop_right = true;
                                }
                                if(rover.ticks.t_left == TEN_CM(2))
                                {
                                    rover.ticks.t_left = 0;
                                    rover.stop_left = true;
                                }
                                if(rover.stop_left && rover.stop_right)
                                {
                                    rover.lead_state = LEADER_STOP;
                                    pwm.target_left_spd = 0;
                                    pwm.target_right_spd = 0;
                                }
                                interrupt_add_pwm(&pwm);
                            }break;
                            case(LEADER_STOP):
                            {
                                pwm.target_left_spd = 0;
                                pwm.target_right_spd = 0;
                                interrupt_add_pwm(&pwm);
                            }break;
                        }
                        //send_message.data.tmr.l_spd = left;
                        //send_message.data.tmr.r_spd = right;
                        //send_message.type = NS_DEBUG_OC;
                        //network_send_add_message(&send_message);
                        //left = 0;
                        //right = 0;
                        //rover.ticks.t_right = 0;
                        //rover.ticks.t_left = 0;
>>>>>>> parent of ac911b9... changed where my case block for rover movement was and made adjustments to speed of rover.  As of now rover moves with decent accuracy on straight movement and turns as well.
                        //pwm.target_left_spd = 2e-1;
                        //pwm.target_right_spd = 2e-1;
                        //interrupt_add_pwm(&pwm);
                        //leader_move(1,1);
                        //pwm.target_left_spd = 2e-1;
                        //pwm.target_right_spd = 2e-1;
                        //interrupt_add_pwm(&pwm);              
                        /*if((left == 0) &&(right == 0))
                        {
                            pwm.target_left_spd = 2e-1;
                            pwm.target_right_spd = 2e-1;
                            leader_move(0,0);
                            if()
                            {
                                rover.ticks.t_right = 0;
                                rover.stop_right = true;
                            }
                            if(rover.ticks.t_left == TEN_CM(2))
                            {
                                rover.ticks.t_left = 0;
                                rover.stop_left = true;
                            }
                            interrupt_add_pwm(&pwm);
                        }
                        else if((left == 1)&&(right == 1))
                        {
                            pwm.target_left_spd = -(2e-1);
                            pwm.target_right_spd = -(2e-1);
                            if(rover.ticks.t_right == TEN_CM(2))
                            {
                                rover.ticks.t_right = 0;
                                rover.stop_right = true;
                            }
                            if(rover.ticks.t_left == TEN_CM(2))
                            {
                                rover.ticks.t_left = 0;
                                rover.stop_left = true;
                            }
                            SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 1);                            
                            interrupt_add_pwm(&pwm);
                        }
                        else if((left == 0)&&(right == 1))
                        {
                            leader_move(0,1);
                            if(rover.ticks.t_right == ROTATE(2))
                            {
                                rover.ticks.t_right = 0;
                                rover.stop_right = true;
                            }
                            if(rover.ticks.t_left == ROTATE(2))
                            {       
                                rover.ticks.t_left = 0;
                                rover.stop_left = true;
                            }
                            interrupt_add_pwm(&pwm);
                        }
                        else if((left == 1) && (right == 0))
                        {
                            leader_move(1,0);
                            if(rover.ticks.t_right == ROTATE(2))
                            {
                                rover.ticks.t_right = 0;
                                rover.stop_right = true;
                            }
                            if(rover.ticks.t_left == ROTATE(2))
                            {
                                rover.ticks.t_left = 0;
                                rover.stop_left = true;
                            }    
                            interrupt_add_pwm(&pwm);
                        }
                        else if(rover.stop_left && rover.stop_right)
                        {
                            pwm.target_left_spd = 0;
                            pwm.target_right_spd = 0;
                            interrupt_add_pwm(&pwm);
                        }*/
                    }break;
                     
                    default:
                        break;
                }       
            } break;
            case PR_PWM:
<<<<<<< HEAD
            { 
                
                send_message.type = NS_DEBUG_OC;               
                send_message.data.tm3r.l_spd = recv_message.data.timer.l_spd;
                send_message.data.tm3r.r_spd = recv_message.data.timer.r_spd;
                send_message.data.tm3r.tmr3 = recv_message.data.timer.tmr3;
                send_message.data.tm3r.tmr4 = recv_message.data.timer.tmr4;
                //send_message.data.tmr.r_spd = processing_counter;
                network_send_add_message(&send_message);
=======
            {
                // PLIB_OC_PulseWidth16BitSet(OC_ID_1, recv_message.data.timer.tmr3);
                //PLIB_OC_PulseWidth16BitSet(OC_ID_2, recv_message.data.timer.tmr4);
                send_message.type = NS_DEBUG_OC;
                //rover.ticks.t_left += recv_message.data.timer.tmr4;
                //rover.ticks.t_right += recv_message.data.timer.tmr3;
                
                send_message.data.tmr.l_spd = recv_message.data.timer.l_spd;
                send_message.data.tmr.r_spd = recv_message.data.timer.r_spd;
>>>>>>> parent of ac911b9... changed where my case block for rover movement was and made adjustments to speed of rover.  As of now rover moves with decent accuracy on straight movement and turns as well.
                //send_message.data.tmr.l_spd = 100.50;
                //send_message.data.tmr.r_spd = 100.50;
                send_message.data.tmr.tmr3 = rover.ticks.t_right;
                send_message.data.tmr.tmr3 = rover.ticks.t_left;
                network_send_add_message(&send_message);
                
            } break;
            case PR_TMR:
            {
<<<<<<< HEAD
                
            } break;            
            default:
                break;               
        }
        if(rover.got_cmnd){
            switch(rover.lead_state){
                case(LEADER_INIT):
                {
                    
                    //pwm.target_left_spd = 2e-1;
                    //pwm.target_right_spd = 2e-1;
                    /*if((left == 0) && (right == 0)){
                        rover.lead_state = LEADER_MOVE;
                        //leader_move(0,0);
                    }
                    if((left == 1) && (right == 0)){
                        //leader_move(1,0);
                        rover.lead_state = LEADER_LEFT;
                    }
                    if((left == 0) && (right == 1)){
                        //leader_move(0,1);
                        rover.lead_state = LEADER_RIGHT;
                    }
                    if((left == 1) && (right== 01)){
                        //leader_move(1,1);
                        rover.lead_state = LEADER_BACK;
                    }
                    //counter++;
                    //interrupt_add_pwm(&pwm);*/
                }break;
                case(LEADER_MOVE):
                {
                    leader_move(0,0);
                    pwm.target_left_spd = 3e-1;
                    pwm.target_right_spd = 3e-1;
                    interrupt_add_pwm(&pwm);
                    rover.lead_state = LEADER_WAIT;
                    //SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 1);
                }break;
                case(LEADER_BACK):
                {
                    leader_move(1,1);                    
                    pwm.target_left_spd = 3e-1;
                    pwm.target_right_spd = 3e-1;
                    interrupt_add_pwm(&pwm);
                    rover.lead_state = LEADER_WAIT;
                }break;
                case(LEADER_TURN):
                {
                    leader_move(0,1);
                    pwm.target_left_spd = 3e-1;
                    pwm.target_right_spd = 3e-1;
                    rover.lead_state = LEADER_WAIT;
                    interrupt_add_pwm(&pwm);
                }break;
                case(LEADER_WAIT):
                {     
                    if(rover.ticks.t_right >= TEN_CM(1))
                    {
                        rover.ticks.t_right = 0;
                        rover.stop_right = true;
                    }
                    if(rover.ticks.t_left >= TEN_CM(1))
                    {
                        rover.ticks.t_left = 0;
                        rover.stop_left = true;
                    }
                    if(rover.ticks.t_right >= ROTATE(1))
                    {
                        rover.ticks.t_right = 0;
                        rover.stop_right = true;
                    }
                    if(rover.ticks.t_left >= ROTATE(1))
                    {       
                        rover.ticks.t_left = 0;
                        rover.stop_left = true;
                    }   
                    if(rover.stop_left && rover.stop_right)
                    {
                        rover.lead_state = LEADER_STOP;                        
                        pwm.target_left_spd = 0;
                        pwm.target_right_spd = 0;
                        interrupt_add_pwm(&pwm);
                    }                    
                }break;
                /*case(LEADER_BORDER):
                {
                    /*switch(rover.border){
                        case(RIGHT_FIRST):
                        {
                            leader_move(1,0);
                            pwm.target_left_spd = 1.8e-1;
                            pwm.target_right_spd = 1.8e-1;
                            rover.border = BORDER_WAIT;
                            interrupt_add_pwm(&pwm);
                        }break;
                        case(LEFT_FIRST):
                        {
                            leader_move(1,0);
                            pwm.target_left_spd = 1.8e-1;
                            pwm.target_right_spd = 1.8e-1;
                            rover.border = BORDER_WAIT;
                            interrupt_add_pwm(&pwm);
                            
                        }break;
                        case(BACK_UP):
                        {
                            leader_move(1,1);
                            pwm.target_left_spd = 1.8e-1;
                            pwm.target_right_spd = 1.8e-1;
                            rover.border = BORDER_WAIT;
                            interrupt_add_pwm(&pwm);
                            
                        }break;
                        case(FORWARD):
                        {
                            leader_move(0,0);
                            pwm.target_left_spd = 1.8e-1;
                            pwm.target_right_spd = 1.8e-1;
                            rover.border = BORDER_WAIT;
                            interrupt_add_pwm(&pwm);                            
                        }break;
                        case(BORDER_WAIT):
                        {
                            if(rover.ticks.t_right >= TEN_CM(1))
                            {
                                rover.ticks.t_right = 0;
                                rover.stop_right = true;
                            }
                            if(rover.ticks.t_left >= TEN_CM(1))
                            {
                                rover.ticks.t_left = 0;
                                rover.stop_left = true;
                            }
                            if(rover.ticks.t_right >= ROTATE(1)/4)
                            {
                                rover.ticks.t_right = 0;
                                rover.stop_right = true;
                            }
                            if(rover.ticks.t_left >= ROTATE(1)/4)
                            {       
                                rover.ticks.t_left = 0;
                                rover.stop_left = true;
                            }   
                            if(rover.stop_left && rover.stop_right)
                            {
                                rover.lead_state = FINISHED;
                            }
                        }break;
                        case(FINISHED):
                        {
                            pwm.target_left_spd = 0;
                            pwm.target_right_spd = 0;
                            interrupt_add_pwm(&pwm);
                            rover.lead_state = LEADER_STOP;
                        }break;
                        default:
                            break;                        
                    }
                }break; */               
                case(LEADER_STOP):
                {                    
                    pwm.target_left_spd = 0;
                    pwm.target_right_spd = 0;
                    interrupt_add_pwm(&pwm);
                    //counter = 0;
                    //rover.lead_state = LEADER_INIT;
                    //send_message.type = NS_MOVEMENT;
                    //send_message.data.ldr_m.dist_x = rover.ldr_m.dist_x;
                    //send_message.data.ldr_m.dist_y = rover.ldr_m.dist_y;
                    //send_message.data.ldr_m.turn_angle = rover.ldr_m.turn_angle;
                    //send_message.data.ldr_m.angle_var = rover.ldr_m.angle_var;
                   // send_message.data.ldr_m.move_var = rover.ldr_m.move_var;
                   // network_send_add_message(&send_message);
                    rover.got_cmnd = false;                    
                }break;
                case(LEADER_STALL):
                {
                    pwm.target_left_spd = 1.5e-1;
                    pwm.target_right_spd = 1.5e-1;
                    interrupt_add_pwm(&pwm);
                }break;
                default:
                    break;
            } 
        }      
=======
                /*send_message.type = NS_DEBUG_OC;
                if(recv_message.data.timer.tmr4 > 10000){
                    pwm.target_right_spd = 2e-5;
                    pwm.target_left_spd = 2e-5;
                    interrupt_add_pwm(&pwm);
                }*/
            } break;
            case PR_ADC:
            {
                //TODO:: ADC DATA
                //send_message.type = NS_ADC_READING;
                //send_message.data.adc_reading.reading = recv_message.data.adc_sample;
                //network_send_add_message(&send_message);
            } break;
            default:
                break;               
        }
        
//        switch(rover.lead_state){
//            case LEADER_INIT:
//            {
//                rover.ticks.t_right = 0;
//                rover.ticks.t_left = 0;
//            }break;
//            case LEADER_MOVE:
//            {
//                pwm.target_right_spd = 2e-1;
//                pwm.target_left_spd = 2e-1;
//                leader_move(0,0);
//                
//                interrupt_add_pwm(&pwm);
//            }break;
//        }
>>>>>>> parent of ac911b9... changed where my case block for rover movement was and made adjustments to speed of rover.  As of now rover moves with decent accuracy on straight movement and turns as well.
    }
}


/*******************************************************************************
 End of File
 */
