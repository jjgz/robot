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
#include "network/recv.h"
#include "debug.h"
#include "pid/pid.h"
#include "peripheral/oc/plib_oc.h"
#include "timers.h"
#include <math.h>

#define TEN_CM(a) (650*a)
#define ROTATE(a) (625*a)
#define PI 3.14159265

TimerHandle_t sensorTimer;
BaseType_t sensorStart = 1;
double cm_1tick = 0.153846/100;
double degree_1tick = 0.16/50;
double val = PI/180;
double cm_to_ft = 0.0328084;
double ft_to_cm = 30.48;
double x_tot = 0;
double y_tot  = 0;    
double angle_tot = 0;
extern Pid pid_right;
extern Pid pid_left;
//int tmr_cnt;
QueueHandle_t processing_queue;
extern QueueHandle_t interrupt_queue;
leader rover;
uint32_t processing_counter;
extern double target_right_spd;
extern double target_left_spd;
double output_left_avg;
double output_right_avg;

void rover_init(){
    rover.ticks.prev_left = 0;
    rover.ticks.prev_right = 0;
    rover.ticks.t_left = 0;
    rover.ticks.t_right = 0;
    rover.stop_left = false;
    rover.slow_right = false;
    rover.slow_left = false;
    rover.stop_right = false;    
    rover.ldr_m.dist_x = 0;
    rover.ldr_m.dist_y = 0;
    rover.ldr_m.turn_angle = 0;
    rover.ldr_m.angle_var = 0;
    rover.ldr_m.move_var = 0;
    rover.sensors.ultra = 0;
    rover.sensors.l_photo =0;
    rover.sensors.r_photo = 0;
    rover.sensors.prev_lphoto = 0;
    rover.sensors.prev_rphoto = 0;
    rover.thresholds.dist_thresh = 0.3;
    rover.thresholds.border_thresh = 50.0;
    rover.lead_state = LEADER_MOVE;
    rover.sense_state = CHARGE;
    rover.nextSense = CHARGE;
    rover.senseArray[0] = 0;
    rover.senseArray[1] = 0;
    rover.senseArray[2] = 0;
    rover.senseArray[3] = 0;
    rover.senseArray[4] = 0;
    rover.senseArray[5] = 0;
    rover.senseArray[6] = 0;
    rover.senseArray[7] = 0;
}
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
    rover_init();
    processing_counter = 0;
    output_left_avg = 0;
    output_right_avg = 0;
    target_right_spd = 2e-5;
    target_left_spd = 2e-5;
}
 
void processing_add_recvmsg(NRMessage *message) {
    PRMessage pr_message;
    pr_message.type = PR_NR;
    pr_message.data.nr_message = *message;
    xQueueSendToBack(processing_queue, &pr_message, portMAX_DELAY);
}

void processing_add_8c_reading( bool boolArray[8]){
    PRMessage pr_message;
    pr_message.type = PR_8C;
    int i;
    for(i = 0; i < 8; i ++)
    {
        pr_message.data.senseArray[i] = boolArray[i];
    }
    /*BaseType_t higher_priority_task_woken = pdFALSE;
    if (xQueueSendToBack(processing_queue, &pr_pwm_message, &higher_priority_task_woken)) {
        portEND_SWITCHING_ISR(higher_priority_task_woken);
    }
    else {
        SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
    }*/
    xQueueSendToBack(processing_queue, &pr_message, portMAX_DELAY);
}

void processing_add_pwm_reading(uint32_t left_pwm, uint32_t right_pwm, double tmr3, double tmr4, bool right, bool left){
    if(rover.got_cmnd) {
        output_right_avg += tmr3;
        output_left_avg += tmr4;
        rover.ticks.t_left += tmr4;
        rover.ticks.t_right += tmr3;
        processing_counter++;
        if(processing_counter == 50){
            processing_counter = 0;
            PRMessage pr_pwm_message;
            //output_right_avg = output_right_avg/50;
            //output_left_avg = output_left_avg/50;
            
            rover.ldr_m.dist_x = cm_to_ft*((output_right_avg + output_left_avg)*cm_1tick*cos((angle_tot)*val));
            rover.ldr_m.dist_y = cm_to_ft*((output_right_avg + output_left_avg)*cm_1tick*sin((angle_tot)*val));
            rover.ldr_m.move_var = 0.4;
            x_tot += rover.ldr_m.dist_x;
            y_tot += rover.ldr_m.dist_y;
            rover.current_pos.x += rover.ldr_m.dist_x;
            rover.current_pos.y += rover.ldr_m.dist_y;
                
           
            angle_tot += (output_right_avg - output_left_avg)*degree_1tick*val;
            
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
}
void interrupt_add_pwm(pwm_to_isr * pwm){
    pwm_to_isr p_val = *pwm;
    xQueueSendToBack(interrupt_queue, &p_val, portMAX_DELAY);
}
/*void leader_state_change(double ultrasonic, double left_photo, double right_photo, double prev_left, double prev_right,double dist_thresh, double border_thresh){
    if((ultrasonic <= dist_thresh)&&(!(left_photo >= border_thresh)) && (!(right_photo >= border_thresh))){
        rover.lead_state = LEADER_TURN;
    }
    else if (!(ultrasonic <= dist_thresh)&&(left_photo >= border_thresh) && !(right_photo >= border_thresh)){
        rover.lead_state = LEADER_BORDER;
        if(!(prev_left >= border_thresh)&&(prev_right >= border_thresh)&&!(right_photo >= border_thresh)){
            rover.border = BACK_UP;
        }
        else{
            rover.border = LEFT_FIRST;
        }
    }
    else if (!(ultrasonic <= dist_thresh)&&!(left_photo >= border_thresh) && (right_photo >= border_thresh)){
        rover.lead_state = LEADER_BORDER;
        if((prev_left >= border_thresh)&&!(prev_right >= border_thresh)&&!(left_photo >= border_thresh)){
            rover.border = BACK_UP;
        }
        else{
            rover.border = RIGHT_FIRST;
        }
    }
    else if (!(ultrasonic <= dist_thresh)&&(left_photo >= border_thresh) &&(right_photo >= border_thresh)){
        rover.border = FINISHED;
        rover.lead_state = LEADER_STOP;
    }
    else if ((!(ultrasonic <= dist_thresh))&&(!(left_photo >= border_thresh)) && (!(right_photo >= border_thresh))){
        rover.lead_state = LEADER_MOVE;
    }
    
}*/

void leader_move(unsigned right, unsigned left){
    SYS_PORTS_PinWrite(0,PORT_CHANNEL_D, PORTS_BIT_POS_1,1);
    SYS_PORTS_PinWrite(0,PORT_CHANNEL_D, PORTS_BIT_POS_0,1);
    SYS_PORTS_PinWrite(0,PORT_CHANNEL_C, PORTS_BIT_POS_14, right);
    SYS_PORTS_PinWrite(0,PORT_CHANNEL_G, PORTS_BIT_POS_1, left);
}

void senseTimerCallback(TimerHandle_t xTimer){
    //tmr_cnt++;
    //vTimerSetTimerID( xTimer, ( void * ) tmr_cnt );
    rover.sense_state = rover.nextSense;
}

void PROCESSING_Initialize() {   
    sensorTimer = xTimerCreate("Timer",pdMS_TO_TICKS(2),pdTRUE, (void *) 0, senseTimerCallback);
    sensorStart = xTimerStart(sensorTimer,0);    
    processing_queue = xQueueCreate(PROCESSING_QUEUE_LEN, sizeof(PRMessage));
    interrupt_queue = xQueueCreate(PROCESSING_QUEUE_LEN, sizeof(pwm_to_isr));
    enable_start();   
}

void PROCESSING_Tasks() {    
    PRMessage recv_message;
    NSMessage send_message;
    pwm_to_isr pwm;
    int counter = 0;
    rover.got_cmnd = false;
    int turn_counter = 0;
    x_tot = 0;
    y_tot  = 0;    
    angle_tot = 0;
    rover.current_pos.x = 0;
    rover.current_pos.y = 0;
    double prev_left_photo = 0;
    double prev_right_photo = 0;
    bool prevSensor[8] = {0};
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
    
    //DebugState debug_s = DEBUGS_NOTHING;
    network_send_add_message(&netstats_message);

    while (1) {
        // We responded to a request, so we increase the responses sent.
        xQueueReceive(processing_queue, &recv_message, portMAX_DELAY);        
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
                     case NR_INITIALIZE:
                     {
                         rover.ntargets = nr_message->data.initialization.nt;
                         rover.start_pos.x = nr_message->data.initialization.ra.x;
                         rover.start_pos.y = nr_message->data.initialization.ra.y;
                         rover.nvertices = nr_message->data.initialization.nb;
                         int i;
                         for( i = 0; i < rover.nvertices; i++)
                         {
                             rover.vertices[i].x = nr_message->data.initialization.points[i].x;
                             rover.vertices[i].y = nr_message->data.initialization.points[i].y;
                         }
                         angle_tot = 0;
                         rover.current_pos.x = rover.start_pos.x;
                         rover.current_pos.y = rover.start_pos.y;
                         rover.got_cmnd = true;
                     }break;
                     case NR_REQ_MOVEMENT:
                     {
                         send_message.type = NS_MOVEMENT;
                         send_message.data.movement.x = rover.current_pos.x;
                         send_message.data.movement.y = rover.current_pos.y;
                         send_message.data.movement.angle = angle_tot;
                         send_message.data.movement.av = rover.ldr_m.angle_var;
                         send_message.data.movement.v = rover.ldr_m.move_var;
                         network_send_add_message(&send_message);
                         
                     }break;
                     case NR_PROXIMITY:
                     {
                         rover_init();                        
                         rover.lead_state = LEADER_INIT;
                         rover.sensors.l_ir = nr_message->data.proximity.left_ir;
                         rover.sensors.r_ir = nr_message->data.proximity.right_ir;
                     }break;
                     case NR_SENSORS:
                     {
                        /*send_message.type = NS_MOVEMENT;
                        send_message.data.ldr_m.dist_x = rover.ldr_m.dist_x;
                        send_message.data.ldr_m.dist_y = rover.ldr_m.dist_y;
                        send_message.data.ldr_m.turn_angle = rover.ldr_m.turn_angle;
                        send_message.data.ldr_m.angle_var = rover.ldr_m.angle_var;
                        send_message.data.ldr_m.move_var = rover.ldr_m.move_var;
                        network_send_add_message(&send_message);*/
                        /*turn_counter++;
                        rover.sensors.ultra = 2.5;
                        if(turn_counter >= 3)
                        {
                            rover.sensors.ultra = 5.0;
                            turn_counter = 0;
                        }*/
                        
                        rover.sensors.prev_lphoto = prev_left_photo;
                        rover.sensors.prev_rphoto = prev_right_photo;
                        rover.sensors.ultra = nr_message->data.ult_photo.ultra;
                        rover.sensors.l_photo = nr_message->data.ult_photo.l_photo;
                        rover.sensors.r_photo = nr_message->data.ult_photo.r_photo;
                        //leader_state_change(ul, lp, rp,rover.sensors.prev_lphoto, rover.sensors.prev_rphoto, rover.thresholds.dist_thresh, rover.thresholds.border_thresh);
                        //leader_state_change(rover.sensors.ultra, rover.sensors.l_photo, rover.sensors.r_photo,rover.sensors.prev_lphoto, rover.sensors.prev_rphoto, rover.thresholds.dist_thresh, rover.thresholds.border_thresh);
                        
                     }break;
                    case NR_DEBUG_JOE_TREAD:
                    {
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
                        //pwm.target_left_spd = 2e-1;
                        //pwm.target_right_spd = 2e-1;
                        //interrupt_add_pwm(&pwm);
                        /*if((left == 0) && (right == 0)){
                        debug_s = DEBUGS_MOVE;
                        //leader_move(0,0);
                        }
                        if((left == 1) && (right == 0)){
                            //leader_move(1,0);
                            debug_s = DEBUGS_TURNL;
                        }
                        if((left == 0) && (right == 1)){
                            //leader_move(0,1);
                            debug_s = DEBUGS_TURNR;
                        }
                        if((left == 1) && (right== 01)){
                            //leader_move(1,1);
                            debug_s = DEBUGS_BACK;
                        }*/
                    }break;
                     
                    default:
                        break;
                }       
            } break;
            case PR_8C:
            {
                int j;
                for(j = 0; j < 8; j++)
                {                
                    rover.senseArray[j] = recv_message.data.senseArray[j];
                }
                rover.lead_state = LEADER_INIT;
            }
            case PR_PWM:
            { 
                //SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 1);
                send_message.type = NS_DEBUG_OC;               
                send_message.data.tm3r.l_spd = recv_message.data.timer.l_spd;
                send_message.data.tm3r.r_spd = recv_message.data.timer.r_spd;
                send_message.data.tm3r.tmr3 = recv_message.data.timer.tmr3;
                send_message.data.tm3r.tmr4 = recv_message.data.timer.tmr4;
                //send_message.data.tmr.r_spd = processing_counter;
                network_send_add_message(&send_message);
                //send_message.data.tmr.l_spd = 100.50;
                //send_message.data.tmr.r_spd = 100.50;
                //send_message.data.tmr.tmr3 = rover.ticks.t_right;
                //send_message.data.tmr.tmr3 = rover.ticks.t_left;
                
                
            } break;
            case PR_TMR:
            {
                
            } break;            
            default:
                break;               
        }
        
        switch(rover.sense_state){
            bool tempArray[8];
            
            case CHARGE:
            {
                
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_7 );
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_6 );
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_5 );
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_4 );
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_3 );
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_2 );
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_1 );
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_0 );
                
                SYS_PORTS_PinWrite(0, PORT_CHANNEL_E, PORTS_BIT_POS_7, 1 );
                SYS_PORTS_PinWrite(0, PORT_CHANNEL_E, PORTS_BIT_POS_6, 1 );
                SYS_PORTS_PinWrite(0, PORT_CHANNEL_E, PORTS_BIT_POS_5, 1 );
                SYS_PORTS_PinWrite(0, PORT_CHANNEL_E, PORTS_BIT_POS_4, 1 );
                SYS_PORTS_PinWrite(0, PORT_CHANNEL_E, PORTS_BIT_POS_3, 1 );
                SYS_PORTS_PinWrite(0, PORT_CHANNEL_E, PORTS_BIT_POS_2, 1 );
                SYS_PORTS_PinWrite(0, PORT_CHANNEL_E, PORTS_BIT_POS_1, 1 );
                SYS_PORTS_PinWrite(0, PORT_CHANNEL_E, PORTS_BIT_POS_0, 1 );
                
                rover.nextSense = INPUT;
            }break;
            case INPUT:
            {
                SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 1);
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_7 );
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_6 );
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_5 );
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_4 );
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_3 );
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_2 );
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_1 );
                SYS_PORTS_PinDirectionSelect ( 0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_0 );
                
                rover.nextSense = READ;
                        
            }break;
            case READ:
            {
                
                tempArray[0] = SYS_PORTS_PinRead(0, PORT_CHANNEL_E, PORTS_BIT_POS_7);
                tempArray[1] = SYS_PORTS_PinRead(0, PORT_CHANNEL_E, PORTS_BIT_POS_6);
                tempArray[2] = SYS_PORTS_PinRead(0, PORT_CHANNEL_E, PORTS_BIT_POS_5);
                tempArray[3] = SYS_PORTS_PinRead(0, PORT_CHANNEL_E, PORTS_BIT_POS_4);
                tempArray[4] = SYS_PORTS_PinRead(0, PORT_CHANNEL_E, PORTS_BIT_POS_3);
                tempArray[5] = SYS_PORTS_PinRead(0, PORT_CHANNEL_E, PORTS_BIT_POS_2);
                tempArray[6] = SYS_PORTS_PinRead(0, PORT_CHANNEL_E, PORTS_BIT_POS_1);
                tempArray[7] = SYS_PORTS_PinRead(0, PORT_CHANNEL_E, PORTS_BIT_POS_0);
                
                processing_add_8c_reading(tempArray);
                rover.nextSense = CHARGE;
            }break;
            default:
                break;
        }
        if(rover.got_cmnd){
            switch(rover.lead_state){
                case(LEADER_INIT):
                {                    
                    if((rover.sensors.l_ir < rover.thresholds.dist_thresh)||(rover.sensors.r_ir < rover.thresholds.dist_thresh))
                    {
                        rover.lead_state = LEADER_TURN;
                    }
                    /*if(((rover.senseArray[0])&&(rover.senseArray[1])&&(rover.senseArray[2])&&(rover.senseArray[3])&&
                            (rover.senseArray[4])&&(rover.senseArray[5])&&(rover.senseArray[6])&&(rover.senseArray[7])) == 0)
                    {
                        rover.lead_state = LEADER_BORDER;
                        pwm.target_left_spd = 0;
                        pwm.target_right_spd = 0;
                        interrupt_add_pwm(&pwm);
                    }*/
                    else{
                        rover.lead_state = LEADER_MOVE;
                    }
                }break;
                case(LEADER_MOVE):
                {
                    leader_move(0,0);
                    pwm.target_left_spd = 3e-1;
                    pwm.target_right_spd = 3e-1;
                    pwm.left_dir = 0;
                    pwm.right_dir = 0;
                    interrupt_add_pwm(&pwm);
                    //rover.lead_state = LEADER_WAIT;
                    //SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 1);
                    if(rover.ticks.t_right >= (TEN_CM(1)/2))
                    {
                        rover.ticks.t_right = 0;
                        rover.stop_right = true;
                    }
                    if(rover.ticks.t_left >= (TEN_CM(1)/2))
                    {
                        rover.ticks.t_left = 0;
                        rover.stop_left = true;
                    }
                    
                    //if(rover.stop_left && rover.stop_right)
                    //{
                        //rover.lead_state = LEADER_STOP;                        
                        //pwm.target_left_spd = 0;
                        //pwm.target_right_spd = 0;
                        //interrupt_add_pwm(&pwm);                       
                        /*rover.ldr_m.dist_x += cm_to_ft*(0.01*cos((90-angle_tot)*val));
                        rover.ldr_m.dist_y += cm_to_ft*(0.01*sin((90-angle_tot)*val));
                        rover.ldr_m.move_var = 0.4;
                        x_tot += rover.ldr_m.dist_x;
                        y_tot += rover.ldr_m.dist_y;
                        rover.current_pos.x += rover.ldr_m.dist_x;
                        rover.current_pos.y += rover.ldr_m.dist_y;*/
                        //send_message.data.ldr_m.turn_angle = 24.0;
                    //}                    
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
                    pwm.target_left_spd = 1.5e-1;
                    pwm.target_right_spd = 1.5e-1;
                    pwm.right_dir = 0;
                    pwm.left_dir = 1;
                    //rover.lead_state = LEADER_WAIT;
                    interrupt_add_pwm(&pwm);
                    if(rover.ticks.t_right >= ROTATE(1)/12)
                    {
                        rover.ticks.t_right = 0;
                        rover.stop_right = true;
                    }
                    if(rover.ticks.t_left >= ROTATE(1)/12)
                    {       
                        rover.ticks.t_left = 0;
                        rover.stop_left = true;
                    }
                     /*if(rover.stop_left && rover.stop_right)
                    {
                        //rover.lead_state = LEADER_STOP;                        
                        //pwm.target_left_spd = 0;
                        //pwm.target_right_spd = 0;
                        //interrupt_add_pwm(&pwm);
                        rover.ldr_m.turn_angle += 12.5;
                        rover.ldr_m.angle_var = 1.5*1.5;
                        angle_tot += val*rover.ldr_m.turn_angle;
                        //send_message.data.ldr_m.turn_angle = 24.0;
                    }*/
                    if((rover.sensors.l_ir > (2.0*rover.thresholds.dist_thresh))&&(rover.sensors.r_ir > (2.0*rover.thresholds.dist_thresh)))
                    {
                        rover.lead_state = LEADER_MOVE;
                    }
                }break;
                case(LEADER_WAIT):
                {     
                    if(rover.ticks.t_right >= (TEN_CM(1)/2))
                    {
                        rover.ticks.t_right = 0;
                        rover.stop_right = true;
                    }
                    if(rover.ticks.t_left >= (TEN_CM(1)/2))
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
                        //send_message.data.ldr_m.turn_angle = 24.0;
                    }                    
                }break;
                /*case(LEADER_BORDER):
                {
                    int k;
                    for(k = 0; k < 4; k++)
                    {
                        if(rover.senseArray[k] == 0)
                            left = 1;
                        if(rover.senseArray[k+4] == 0)
                            right = 1;
                    }        
                    rover.border = B_BACK;
                    switch(rover.border)
                    {
                        case(B_BACK):
                        {
                            leader_move(1,1); 
                            pwm.target_left_spd = 1.5e-1;
                            pwm.target_right_spd = 1.5e-1;
                            interrupt_add_pwm(&pwm);
                            rover.border = B_BWAIT;                            
                        }break;
                        case(B_FORWARD):
                        {
                            leader_move(0,0); 
                            pwm.target_left_spd = 1.5e-1;
                            pwm.target_right_spd = 1.5e-1;
                            interrupt_add_pwm(&pwm);
                            rover.border = B_FWAIT;
                        }break;
                        case(B_RIGHT):
                        {
                            leader_move(1,0); 
                            pwm.target_left_spd = 1.5e-1;
                            pwm.target_right_spd = 1.5e-1;
                            interrupt_add_pwm(&pwm);
                            rover.border = B_TWAIT;
                        }break;
                        case(B_LEFT):
                        {
                            leader_move(0,1); 
                            pwm.target_left_spd = 1.5e-1;
                            pwm.target_right_spd = 1.5e-1;
                            interrupt_add_pwm(&pwm);
                            rover.border = B_TWAIT;
                        }break;
                        case(B_ALIGNED):
                        {
                            bool align;
                            int c;
                            for(c = 0; c < 8; c++)
                            {
                                if(rover.senseArray[c] == 0)
                                {
                                    align = 1;
                                }
                                else
                                {
                                    align = 0;
                                }
                            }
                            if(align)
                            {
                                leader_move(0,1); 
                                pwm.target_left_spd = 1.5e-1;
                                pwm.target_right_spd = 1.5e-1;
                                interrupt_add_pwm(&pwm);
                                rover.border = B_AWAIT;
                            }
                            else
                            {
                                rover.border = B_BACK;
                            }
                        }break;
                        case(B_AWAIT):
                        {
                            if(rover.ticks.t_right >= 2*ROTATE(1))
                            {
                                rover.ticks.t_right = 0;
                                rover.stop_right = true;
                            }
                            if(rover.ticks.t_left >= 2*ROTATE(1))
                            {  
                                rover.ticks.t_left = 0;
                                rover.stop_left = true;
                            }
                            if(rover.stop_left && rover.stop_right)
                            {
                                rover.lead_state = B_ASTOP;                                
                            }
                        }break;
                        case(B_TWAIT):
                        {
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
                                rover.lead_state = B_TSTOP;                                
                            }
                        }break;                        
                        case(B_BWAIT):
                        {
                            if(rover.ticks.t_right >= TEN_CM(1)/5)
                            {
                                rover.ticks.t_right = 0;
                                rover.stop_right = true;
                            }
                            if(rover.ticks.t_left >= TEN_CM(1)/5)
                            {
                                rover.ticks.t_left = 0;
                                rover.stop_left = true;
                            }
                            if(rover.stop_left && rover.stop_right)
                            {
                                rover.lead_state = B_BSTOP;                                
                            }                      
                        }break;
                        case(B_FWAIT):
                        {
                            if(rover.ticks.t_right >= TEN_CM(1)/5)
                            {
                                rover.ticks.t_right = 0;
                                rover.stop_right = true;
                            }
                            if(rover.ticks.t_left >= TEN_CM(1)/5)
                            {
                                rover.ticks.t_left = 0;
                                rover.stop_left = true;
                            }
                            if(rover.stop_left && rover.stop_right)
                            {
                                rover.lead_state = B_FSTOP;                                
                            }                      
                        }break;
                        case(B_TSTOP):
                        {
                            pwm.target_left_spd = 0;
                            pwm.target_right_spd = 0;
                            interrupt_add_pwm(&pwm);
                            rover.border = B_FORWARD;
                        }break;
                        case(B_ASTOP):
                        {
                            pwm.target_left_spd = 0;
                            pwm.target_right_spd = 0;
                            interrupt_add_pwm(&pwm);
                            //rover.border = B_INIT;
                            rover.lead_state = LEADER_STOP;
                        }break;
                         case(B_FSTOP):
                        {
                            pwm.target_left_spd = 0;
                            pwm.target_right_spd = 0;
                            interrupt_add_pwm(&pwm);
                            if (right && left)
                                rover.border = B_ALIGNED;
                            else
                                rover.border = B_BACK;
                        }break;
                         case(B_BSTOP):
                        {
                            pwm.target_left_spd = 0;
                            pwm.target_right_spd = 0;
                            interrupt_add_pwm(&pwm);
                            if(left)
                                rover.border = B_LEFT;
                            else if(right)
                                rover.border = B_RIGHT;
                            else if (right && left)
                                rover.border = B_FORWARD;
                        }break;
                    }
                            
                }break;*/                
                case(LEADER_STOP):
                {     
                    //uint32_t tmp_timer = (uint32_t) pvTimerGetTimerID(sensorTimer);
                    pwm.target_left_spd = 0;
                    pwm.target_right_spd = 0;
                    interrupt_add_pwm(&pwm);
                    //counter = 0;
                    /*send_message.type = NS_MOVEMENT;
                    send_message.data.ldr_m.dist_x = x_tot;
                    send_message.data.ldr_m.dist_y = y_tot;
                    if(sensorTimer == NULL)
                    {
                        send_message.data.ldr_m.turn_angle = 0.0;
                    }
                    else
                    {
                        send_message.data.ldr_m.turn_angle = 1.0;
                    }                    
                    send_message.data.ldr_m.angle_var = (double)sensorStart;  //rover.ldr_m.angle_var;
                    send_message.data.ldr_m.move_var =  rover.ldr_m.move_var;
                    network_send_add_message(&send_message);*/
                    rover.lead_state = LEADER_INIT;
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
    }
}


/*******************************************************************************
 End of File
 */
