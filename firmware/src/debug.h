/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    debug.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _DEBUG_H
#define _DEBUG_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

typedef enum {
    DEBUG_LOC_HALT,
            
    DEBUG_LOC_STRSEND_ENTER,
    DEBUG_LOC_STRSEND_WHILE,
    DEBUG_LOC_STRSEND_BEFORE_SEND,
    DEBUG_LOC_STRSEND_AFTER_SEND,
    DEBUG_LOC_STRSEND_BEFORE_RECV,
    DEBUG_LOC_STRSEND_AFTER_RECV,
        
    DEBUG_LOC_TMR2_ENTER,
    DEBUG_LOC_TMR2_LEAVE,
    DEBUG_LOC_TMR2_BEFORE_SEND,
    DEBUG_LOC_TMR2_AFTER_SEND,
    DEBUG_LOC_TMR2_BEFORE_RECV,
    DEBUG_LOC_TMR2_AFTER_RECV,
} DebugLocation;

typedef enum {
    DEBUG_QUEUE_VAL,
    DEBUG_QUEUE_LOC,
    DEBUG_QUEUE_HALT
} DebugQueueType;

typedef union {
    unsigned char val;
    DebugLocation loc;
} DebugQueueUnion;

typedef struct {
    DebugQueueType type;
    DebugQueueUnion value;
} DebugQueueItem;

void debug_val(unsigned char val);
void debug_val_isr(unsigned char val);

void debug_loc(DebugLocation loc);
void debug_loc_isr(DebugLocation loc);

void debug_halt();
void debug_halt_isr();

void DEBUG_Initialize ( void );

void DEBUG_Tasks( void );


#endif /* _DEBUG_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

