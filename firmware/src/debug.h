#ifndef _DEBUG_H
#define _DEBUG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

typedef enum {
    // 00
    DEBUG_LOC_HALT,
    
    // 01
    DEBUG_LOC_STRSEND_ENTER,
    // 02
    DEBUG_LOC_STRSEND_WHILE,
    // 03
    DEBUG_LOC_STRSEND_BEFORE_SEND,
    // 04
    DEBUG_LOC_STRSEND_AFTER_SEND,
    // 05
    DEBUG_LOC_STRSEND_BEFORE_RECV,
    // 06
    DEBUG_LOC_STRSEND_AFTER_RECV,
        
    // 07
    DEBUG_LOC_TMR2_ENTER,
    // 08
    DEBUG_LOC_TMR2_LEAVE,
    // 09
    DEBUG_LOC_TMR2_BEFORE_SEND,
    // 0A
    DEBUG_LOC_TMR2_AFTER_SEND,
    // 0B
    DEBUG_LOC_TMR2_BEFORE_RECV,
    // 0C
    DEBUG_LOC_TMR2_AFTER_RECV,
} DebugLocation;

void debug_val(unsigned char val);

void debug_loc(DebugLocation loc);

void debug_halt();


#endif /* _DEBUG_H */

