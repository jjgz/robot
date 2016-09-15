#include "int_wifly.h"
#include "queue.h"
#include "network_recv.h"
#include "debug.h"

#define INT_WIFLY_QUEUE_LEN 2

QueueHandle_t wifly_queue;

CharBuffer send_buffer;
unsigned send_buffer_pos;

CharBuffer recv_buffer;
unsigned recv_buffer_pos;

unsigned sequence_counter;
const char sequence_buffer[] = {128, 37, 35, 36};
uint8_t crc;
typedef enum {INIT, CRC, LENGTH, MSG} SEQUENCE_STATE;
SEQUENCE_STATE sequence_state;

void wifly_int_crc(uint8_t byte);
void wifly_int_init() {
    sequence_state = CRC;
    crc = 0;
    sequence_counter = 0;
    send_buffer_pos = 0;
    send_buffer.length = 0;
    recv_buffer_pos = 0;
    recv_buffer.length = 0;
    wifly_queue = xQueueCreate(INT_WIFLY_QUEUE_LEN, 1);
    if (!wifly_queue) {
        SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 1);
    }
}

void wifly_int_send(char a) {
    xQueueSendToBack(wifly_queue, &a, portMAX_DELAY);
}

WiflyIntCycle wifly_int_cycle() {
    WiflyIntCycle cycle;
    BaseType_t higher_priority_task_woken = pdFALSE;
    char a;
    // Attempt to receive a new buffer to send.
    if (xQueueReceiveFromISR(wifly_queue, &a, &higher_priority_task_woken)) {
        cycle.sending = true;
        cycle.item = a;
        portEND_SWITCHING_ISR(higher_priority_task_woken);
    // We didn't receive a buffer.
    } else {
        cycle.sending = false;
    }
    return cycle;
}

void wifly_int_acknowledge() {
    send_buffer_pos++;
}

void wifly_int_recv_byte(char byte) {
    //check if the sequence bytes match...should count up to 4
    if(sequence_buffer[sequence_counter] == byte){
        sequence_counter++;
    }
    else{
        sequence_counter = 0;
        sequence_state = INIT;
    }
    if(sequence_counter == 4){
        sequence_state = CRC;
        sequence_counter = 0;
    }
    
    switch(sequence_state){
        case INIT:
            sequence_state = INIT;
            break;
        case CRC:
            crc = 0; 
            wifly_int_crc(byte);
            sequence_state = LENGTH;
            break;
        case LENGTH:            
            
            sequence_state = MSG;
            break;
        case MSG:
            
            break;
    }
    //if all the sequenced match then the counter should be 4
    if (recv_buffer.length == recv_buffer_pos) {
        recv_buffer.length = (unsigned)byte + 1;
        recv_buffer_pos = 0;
    } else {
        recv_buffer.buff[recv_buffer_pos++] = byte;
        if (recv_buffer_pos == recv_buffer.length) {
            network_recv_add_buffer_from_isr(&recv_buffer);
        }
    }
}

void wifly_int_crc(uint8_t byte){
	uint8_t i;
    crc ^= (byte << 8);
	for(i = 8; i; i--) {
		if (crc & 0x8000)
			crc ^= (0x1070 << 3);
		crc <<= 1;
	}
	
}