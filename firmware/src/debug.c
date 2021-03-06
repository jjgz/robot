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

#include "debug.h"

typedef struct {
    PORTS_CHANNEL reg;
    PORTS_BIT_POS bit;
} Pin;

const Pin loc_pins[] = {
    {PORT_CHANNEL_E, PORTS_BIT_POS_7}, //port 30
    {PORT_CHANNEL_E, PORTS_BIT_POS_6}, //port 31
    {PORT_CHANNEL_E, PORTS_BIT_POS_5}, //port 32
    {PORT_CHANNEL_E, PORTS_BIT_POS_4}, //port 33
    {PORT_CHANNEL_E, PORTS_BIT_POS_3}, //port 34
    {PORT_CHANNEL_E, PORTS_BIT_POS_2}, //port 35
    {PORT_CHANNEL_E, PORTS_BIT_POS_1}, //port 36
    {PORT_CHANNEL_E, PORTS_BIT_POS_0} //port 37
};

const Pin val_pins[] = {
    {PORT_CHANNEL_D, PORTS_BIT_POS_1}, //port 0
    {PORT_CHANNEL_D, PORTS_BIT_POS_2}, //port 1
    {PORT_CHANNEL_A, PORTS_BIT_POS_6}, //port 80
    {PORT_CHANNEL_A, PORTS_BIT_POS_7}, //port 81
    {PORT_CHANNEL_G, PORTS_BIT_POS_14}, //port 82
    {PORT_CHANNEL_G, PORTS_BIT_POS_12}, //port 83
    {PORT_CHANNEL_G, PORTS_BIT_POS_13}, //port 84
    {PORT_CHANNEL_D, PORTS_BIT_POS_3} //port 9
};

void debug_val(unsigned char val) {
    int i;
    for (i = 0; i < 8; i++) {
        SYS_PORTS_PinWrite(0, val_pins[i].reg, val_pins[i].bit, (val >> i) & 1);
    }
}

void debug_loc(DebugLocation loc) {
    int i;
    //SYS_PORTS_PinWrite(0, PORT_CHANNEL_E, PORTS_BIT_POS_0, 1);
    for (i = 0; i < 8; i++) {
        SYS_PORTS_PinWrite(0, loc_pins[i].reg, loc_pins[i].bit, (loc >> i) & 1);
    }
}

void debug_halt() {
    int i;
    for (i = 0; i < 8; i++) {
        SYS_PORTS_PinWrite(0, loc_pins[i].reg, loc_pins[i].bit, (DEBUG_LOC_HALT >> i) & 1);
    }
    // Halt.
    while (1) {}
}
