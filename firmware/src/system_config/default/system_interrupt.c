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
#include "network/send.h"

NSMessage req_movement_msg;

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
void IntHandlerDrvAdc(void)
{
    PLIB_ADC_SampleAutoStartDisable(ADC_ID_1);
    if (DRV_ADC_SamplesAvailable()) {
        int_adc_samples(DRV_ADC_SamplesRead(0), DRV_ADC_SamplesRead(1), DRV_ADC_SamplesRead(2));
    }
    /* Clear ADC Interrupt Flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1);
}
    
void IntHandlerDrvTmrInstance0(void)
{
    req_movement_msg.type = NS_REQ_MOVEMENT;
    network_send_add_message_isr(&req_movement_msg);
    PLIB_ADC_SampleAutoStartEnable(ADC_ID_1);
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
}
  
/*******************************************************************************
 End of File
*/

