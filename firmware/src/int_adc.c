#include "int_adc.h"
#include "network_send.h"
#include "debug.h"

#define NUM_SAMPLES_BEFORE_SEND 50

unsigned samples_collected;
unsigned sample_accumulator;

void int_adc_init() {
    samples_collected = 0;
    sample_accumulator = 0;
}

void int_adc_sample(unsigned sample) {
    sample_accumulator += sample;
    samples_collected++;
    if (samples_collected == NUM_SAMPLES_BEFORE_SEND) {
        samples_collected = 0;
        //debug_loc(sample_accumulator / NUM_SAMPLES_BEFORE_SEND);
        NSMessage message;
        message.type = NS_ADC_READING;
        message.data.adc_reading.reading = PLIB_ADC_ResultGetByIndex(DRV_ADC_ID_1, 0);
        //debug_loc(message.data.adc_reading.reading / NUM_SAMPLES_BEFORE_SEND);
        network_send_add_message_isr(&message);
        sample_accumulator = 0;
    }
}
