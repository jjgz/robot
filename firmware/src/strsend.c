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

#include "strsend.h"
#include "debug.h"
#include "queue.h"

#define STRSEND_QUEUE_LEN 64

QueueHandle_t queue;

void strsend_update_isr() {
    char item;
    debug_loc(DEBUG_LOC_STRSEND_BEFORE_SEND);
    BaseType_t higher_priority_task_woken = pdFALSE;
    xQueueSendToBackFromISR(queue, &item, &higher_priority_task_woken);
    portEND_SWITCHING_ISR(higher_priority_task_woken);
    debug_loc(DEBUG_LOC_STRSEND_AFTER_SEND);
}

void STRSEND_Initialize() {
    queue = xQueueCreate(STRSEND_QUEUE_LEN, 1);
    DRV_TMR0_Start();
}

void STRSEND_Tasks() {
    const char *teamstr = "Team 1";
    int teamstrlen = strlen(teamstr);
    int teamstrpos;
    debug_loc(DEBUG_LOC_STRSEND_ENTER);
    char item;
    debug_loc(DEBUG_LOC_STRSEND_WHILE);
    while (1) {
        debug_loc(DEBUG_LOC_STRSEND_BEFORE_RECV);
        xQueueReceive(queue, &item, portMAX_DELAY);
        debug_loc(DEBUG_LOC_STRSEND_AFTER_RECV);
        //make sure the transmit buffer is not full before trying to write
      /*  if(!(DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL & DRV_USART0_TransferStatus()))
        {
             DRV_USART0_WriteByte('a');
        }*/
        debug_val(teamstr[teamstrpos++]);
        if (teamstrpos == teamstrlen)
            teamstrpos = 0;
    }
}
