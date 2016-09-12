#ifndef INT_WIFLY_H
#define	INT_WIFLY_H

typedef struct {
    bool sending;
    char item;
} WiflyIntCycle;

// Functions to call from outside of the ISR.
void wifly_int_init();
void wifly_int_send(char *buff, unsigned length);

// Functions to call from the ISR.
WiflyIntCycle wifly_int_cycle();
void wifly_int_acknowledge();
void wifly_int_recv_byte(char byte);

#endif	/* INT_WIFLY_H */

