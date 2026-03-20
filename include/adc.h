#ifndef ADC_H
#define ADC_H

#include "esp_adc/adc_oneshot.h"

void adc_handle_init();
adc_oneshot_unit_handle_t adc_unit_to_handle(adc_unit_t adc_unit);

#endif