/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <xc.h>
#include <sys/attribs.h>
#include "network/int_wifly.h"
#include "int_adc.h"
#include "system_definitions.h"
#include "framework/driver/adc/drv_adc_static.h"
#include "pid/pid.h"
#include "peripheral/oc/plib_oc.h"
#include "math.h"
#include "processing.h"
#include "system_config.h"
#include "portmacro.h"
#include "network/send.h"
Pid pid_right;
Pid pid_left;
bool alter;
double  target_right_spd;
double  target_left_spd;
double prev_left_tspd = 0;
double prev_right_tspd = 0;
double ramp_spd_r = 0;
double ramp_spd_l = 0;
bool left_back;
bool right_back;
extern leader rover;
const double desired_speed = 2e-1;
QueueHandle_t interrupt_queue;
NSMessage req_sensors;
//unsigned counter = 0;
#define SPEED_TICKS 1000

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

double clamp(double val, double min, double max){
    return min(max, max(min, val));
}
//void IntHandlerDrvAdc(void)
//{
    //debug_loc(DEBUG_INTADC_ENTER);
    //if (DRV_ADC_SamplesAvailable()) {
    //    int_adc_sample(PLIB_ADC_ResultGetByIndex(DRV_ADC_ID_1, 0));
    //}
    //PLIB_ADC_SampleAutoStartEnable(DRV_ADC_ID_1);
    /* Clear ADC Interrupt Flag */
   // PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1);
    //debug_loc(DEBUG_INTADC_LEAVE);
//}

void IntHandlerDrvTmrInstance0(void)
{
    
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_3);
}
 
void IntHandlerDrvTmrInstance1(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_4);
}
void IntHandlerDrvTmrInstance2(void)
{   
    switch(rover.lead_state)
    {
        case LEADER_WAIT_ASSUMED:
            req_sensors.type = NS_REQ_ASSUMED;
            network_send_add_message_isr(&req_sensors);
            break;
        default:
            req_sensors.type = NS_REQ_PROXIMITY;
            network_send_add_message_isr(&req_sensors);
            break;
    }
    
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_5); 
}

void IntHandlerDrvTmrInstance3(void)
{    
    pwm_to_isr recv_pwm;
        if(!xQueueIsQueueEmptyFromISR(interrupt_queue))
        {       
            xQueueReceiveFromISR(interrupt_queue, &recv_pwm, 0);        
            target_right_spd = recv_pwm.target_right_spd;
            target_left_spd = recv_pwm.target_left_spd;
            left_back = recv_pwm.left_dir;
            right_back = recv_pwm.right_dir;
        }
        const double scaling_left = 1.0;
        const double scaling_right = 1.009;
        double right_ticks = scaling_right * DRV_TMR0_CounterValueGet();
        double left_ticks = scaling_left * DRV_TMR1_CounterValueGet();
        
        if (target_left_spd < ramp_spd_l) {
            ramp_spd_l -= min(0.001, ramp_spd_l - target_left_spd);
        } else {
            ramp_spd_l += min(0.001, target_left_spd - ramp_spd_l);
        }
        
        if (target_right_spd < ramp_spd_r) {
            ramp_spd_r -= min(0.001, ramp_spd_r - target_right_spd);
        } else {
            ramp_spd_r += min(0.001, target_right_spd - ramp_spd_r);
        }
        
        uint16_t output_r = clamp(pid_output(&pid_right, ramp_spd_r - right_ticks,1e1,1e3,0),0,65535);
        uint16_t output_l = clamp(pid_output(&pid_left,  ramp_spd_l - left_ticks,1e1,1e3,0),0,65535);
        if(left_back){
            left_ticks = -left_ticks;
        }
        if(right_back){
            right_ticks = -right_ticks;
        }       
        processing_add_pwm_reading(output_l, output_r, right_ticks, left_ticks, right_back, left_back);
        DRV_TMR0_CounterClear();
        DRV_TMR1_CounterClear();
        leader_move(right_back, left_back);
        PLIB_OC_PulseWidth16BitSet(OC_ID_2, output_r);
        PLIB_OC_PulseWidth16BitSet(OC_ID_1, output_l);
        PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}

void IntHandlerDrvUsartInstance0(void)
{
    if (SYS_INT_SourceStatusGet(INT_SOURCE_USART_1_RECEIVE)) {
        wifly_int_recv_byte(DRV_USART0_ReadByte());
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_RECEIVE);
    }
    
    if (SYS_INT_SourceStatusGet(INT_SOURCE_USART_1_TRANSMIT)) {
        while (!DRV_USART0_TransmitBufferIsFull()) {
            WiflyIntCycle cycle = wifly_int_cycle();
            if (cycle.sending) {
                DRV_USART0_WriteByte(cycle.item);
                wifly_int_confirm_sent();
            } else {
                break;
            }
        }
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_TRANSMIT);
    }
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_TRANSMIT);
} 
/*******************************************************************************
 End of File
*/


