#include "adc.h"

static adc_oneshot_unit_handle_t adc_handle_unit1 = NULL;
static adc_oneshot_unit_handle_t adc_handle_unit2 = NULL;

void adc_handle_init(){

    adc_oneshot_unit_init_cfg_t init_config = {0, 0, 0};

    if(adc_handle_unit1 == NULL){
        init_config.unit_id = ADC_UNIT_1;
        adc_oneshot_new_unit(&init_config, &adc_handle_unit1);
    }

    if(adc_handle_unit2 == NULL){
        init_config.unit_id = ADC_UNIT_2;
        adc_oneshot_new_unit(&init_config, &adc_handle_unit2);
    }
}

adc_oneshot_unit_handle_t adc_unit_to_handle(adc_unit_t adc_unit){
    
    switch(adc_unit){
        case ADC_UNIT_1: return adc_handle_unit1;
        case ADC_UNIT_2: return adc_handle_unit2; 
        default        : return NULL; 
    }
}