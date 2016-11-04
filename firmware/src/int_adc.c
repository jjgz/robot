#include "int_adc.h"
#include "processing.h"

PRMessage message;

void int_adc_samples(unsigned ir_front_right, unsigned ir_front_left, unsigned ir_left) {
    message.type = PR_ADC_SAMPLES;
    message.data.adc_samples.ir_front_right = ir_front_right;
    message.data.adc_samples.ir_front_left = ir_front_left;
    message.data.adc_samples.ir_left = ir_left;
    processing_add_message(&message);
}
