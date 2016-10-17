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
Pid controller_left;
Pid controller_right;
QueueHandle_t interrupt_queue;
double wanted_speed_left;
double wanted_speed_right;
const double desired_speed = 2e-1;
#define SPEED_TICKS 1000
// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
double clamp(double val, double min, double max)
{
    return min(max, max(min, val));
}

void IntHandlerDrvTmrInstance0(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_3);
}

void IntHandlerDrvTmrInstance1(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_4);
}

//good wanted value is 2e-1
    
void IntHandlerDrvTmrInstance2(void)
{
    pwm_to_isr recv_pwm;
    if(!xQueueIsQueueEmptyFromISR(interrupt_queue))
    {
        xQueueReceiveFromISR( interrupt_queue, &recv_pwm, portMAX_DELAY);
        wanted_speed_left = recv_pwm.wanted_speed_left;
        wanted_speed_right = recv_pwm.wanted_speed_right;
    }
 
    const double left_scale = 1.0;
    const double right_scale = 1.015;
    uint16_t output_right = clamp(pid_output(&controller_right, wanted_speed_right - right_scale * (double)DRV_TMR0_CounterValueGet(), 1e1, 1e3, 0), 0, 65535);
    uint16_t output_left = clamp(pid_output(&controller_left, wanted_speed_left - left_scale * (double)DRV_TMR1_CounterValueGet(), 1e1, 1e3, 0), 0, 65535);
    
    processing_add_pwm_reading(output_left,output_right,DRV_TMR0_CounterValueGet(),DRV_TMR1_CounterValueGet());
    DRV_TMR0_CounterClear();
    DRV_TMR1_CounterClear();
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, output_right);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, output_left);
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_5);
}
    
void IntHandlerDrvTmrInstance3(void)
{
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

