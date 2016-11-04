#include "int_adc.h"
#include "processing.h"

PRMessage message;

void int_adc_samples(unsigned ultra_front, unsigned ir_front, unsigned ir_left, unsigned ir_right) {
    message.type = PR_ADC_SAMPLES;
    message.data.adc_samples.ultra_front = ultra_front;
    message.data.adc_samples.ir_front = ir_front;
    message.data.adc_samples.ir_left = ir_left;
    message.data.adc_samples.ir_right = ir_right;
    processing_add_message(&message);
}
