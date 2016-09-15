#ifndef INT_WIFLY_H
#define	INT_WIFLY_H

#include "system_definitions.h"
#include "buffer.h"

// Functions to call from outside of the ISR.
void wifly_int_init();

// Functions to call from the ISR.
void wifly_int_recv_byte(char byte);

#endif	/* INT_WIFLY_H */
