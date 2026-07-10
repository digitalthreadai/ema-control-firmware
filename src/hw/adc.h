#ifndef HW_ADC_H
#define HW_ADC_H

#include <stdint.h>

/* Minimal ADC front-end surface consumed by resolver_iface.c. The concrete
 * implementation lives in the board support package for the target MCU
 * (not part of this repository) -- this header is the contract the DAL-A
 * application code links against. */

typedef enum {
    ADC_CH_RDC_A_SIN = 0,
    ADC_CH_RDC_A_COS,
    ADC_CH_RDC_B_SIN,
    ADC_CH_RDC_B_COS,
    ADC_CH_MOTOR_A_I,
    ADC_CH_MOTOR_B_I,
} adc_channel_t;

void     adc_configure_channel(adc_channel_t ch);
uint16_t adc_read_raw(adc_channel_t ch);

#endif /* HW_ADC_H */
