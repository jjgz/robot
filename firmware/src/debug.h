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
    DEBUG_NETRECV_ENTER,
    // 02
    DEBUG_NETRECV_WHILE,
    // 03
    DEBUG_NETRECV_BEFORE_SEND,
    // 04
    DEBUG_NETRECV_AFTER_SEND,
    // 05
    DEBUG_NETRECV_BEFORE_RECV,
    // 06
    DEBUG_NETRECV_AFTER_RECV,
        
    // 07
    DEBUG_NETSEND_ENTER,
    // 08
    DEBUG_NETSEND_WHILE,
    // 09
    DEBUG_NETSEND_BEFORE_SEND,
    // 0A
    DEBUG_NETSEND_AFTER_SEND,
    // 0B
    DEBUG_NETSEND_BEFORE_RECV,
    // 0C
    DEBUG_NETSEND_AFTER_RECV,
        
    // 0D
    DEBUG_PROCESSING_ENTER,
    // 0E
    DEBUG_PROCESSING_WHILE,
    // 0F
    DEBUG_PROCESSING_BEFORE_SEND,
    // 10
    DEBUG_PROCESSING_AFTER_SEND,
    // 11
    DEBUG_PROCESSING_BEFORE_RECV,
    // 12
    DEBUG_PROCESSING_AFTER_RECV,
        
    // 13
    DEBUG_INTWIFLY_ENTER,
    // 14
    DEBUG_INTWIFLY_LEAVE,
    // 15
    DEBUG_INTWIFLY_WRITEBYTE,
    // 16
    DEBUG_INTWIFLY_READBYTE,
            
    // 17
    DEBUG_NETSEND_BEFORE_PARSE,
    // 18
    DEBUG_NETSEND_BEFORE_STRING,
    // 19
    DEBUG_NETSEND_AFTER_STRING,
    // 1A
    DEBUG_NETSEND_AFTER_STRLEN,
} DebugLocation;

void debug_val(unsigned char val);

void debug_loc(DebugLocation loc);

void debug_halt();


#endif /* _DEBUG_H */

