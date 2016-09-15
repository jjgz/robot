#include "int_wifly.h"
#include "queue.h"
#include "network_recv.h"
#include "debug.h"

CharBuffer send_buffer;
unsigned send_buffer_pos;

CharBuffer recv_buffer;
unsigned recv_buffer_pos;

unsigned sequence_counter;
const char sequence_buffer[] = {128, 37, 35, 36};
uint16_t crc, crc_byte;
typedef enum {SEQ_INIT, SEQ_CRC, SEQ_LENGTH, SEQ_MSG} SEQUENCE_STATE;
SEQUENCE_STATE sequence_state;

void wifly_int_crc(uint8_t byte);
void wifly_int_init() {
    sequence_state = SEQ_CRC;
    crc = 0;
    sequence_counter = 0;
    recv_buffer.buff = 0;
}

void wifly_int_recv_byte(char byte) {
    //check if the sequence bytes match...should count up to 4
    if (sequence_buffer[sequence_counter] == byte) {
        if (sequence_state == SEQ_MSG)
            buffer_free(&recv_buffer);
        sequence_counter++;
    } else if (sequence_buffer[0] == byte) {
        sequence_counter = 1;
    } else {
        sequence_counter = 0;
    }
    
    if (sequence_counter == 4) {
        sequence_state = SEQ_CRC;
        sequence_counter = 0;
    }
    
    switch (sequence_state) {
        case SEQ_INIT:
            sequence_state = SEQ_INIT;
            break;
        case SEQ_CRC:
            debug_loc(DEBUG_INTWIFLY_IN_CRC);
            crc = 0; 
            crc_byte = byte;
            sequence_state = SEQ_LENGTH;
            break;
        case SEQ_LENGTH:
            debug_loc(DEBUG_INTWIFLY_DO_CRC_LENGTH);
            wifly_int_crc(byte);
            recv_buffer = buffer_new((unsigned)byte + 1);
            recv_buffer_pos = 0;
            sequence_state = SEQ_MSG;
            break;
        case SEQ_MSG:
            debug_loc(DEBUG_INTWIFLY_DO_CRC_MESSAGE);
            wifly_int_crc(byte);
            recv_buffer.buff[recv_buffer_pos++] = byte;
            if (recv_buffer_pos == recv_buffer.length) {
                if ((crc >> 8) == crc_byte) {
                    debug_loc(DEBUG_INTWIFLY_ADD_MESSAGE);
                    network_recv_add_buffer_from_isr(&recv_buffer);
                // We received bad data.
                } else {
                    // TODO: Send a message to indicate a packet loss.
                    buffer_free(&recv_buffer);
                }
                sequence_state = SEQ_INIT;
            }
            break;
        default:
            //SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 1);
            break;
    }
}

// TODO: Credit: https://chromium.googlesource.com/chromiumos/platform/vboot_reference/+/master/firmware/lib/crc8.c
void wifly_int_crc(uint8_t byte) {
	uint8_t i;
    uint16_t ibyte = byte;
    crc ^= (ibyte << 8);
	for (i = 8; i; i--) {
		if (crc & 0x8000)
			crc ^= (0x1070 << 3);
		crc <<= 1;
	}
}