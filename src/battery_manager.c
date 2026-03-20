#include <math.h>

#include "driver/gpio.h"
#include "driver/adc.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_adc_cal.h"
#include "constants.h"
#include "adc.h"
#include "battery_manager.h"

static adc_unit_t    gpio_to_adc_unit   (gpio_num_t gpio);
static adc_channel_t gpio_to_adc_channel(gpio_num_t gpio);

void battery_manager_init(battery_manager_t* battery_manager, uint8_t pin_battery, uint8_t pin_indicator);
void battery_manager_read(battery_manager_t* battery_manager);
void battery_manager_indicate(battery_manager_t* battery_manager);

void battery_manager_init(battery_manager_t* battery_manager, uint8_t pin_battery, uint8_t pin_indicator){
    battery_manager->pin_battery   = pin_battery;
    battery_manager->pin_indicator = pin_indicator;

    battery_manager->adc_handle = adc_unit_to_handle(gpio_to_adc_unit(battery_manager->pin_battery));

    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_12  
    };

    battery_manager->adc_channel = gpio_to_adc_channel(battery_manager->pin_battery);

    adc_oneshot_config_channel(battery_manager->adc_handle, battery_manager->adc_channel, &channel_config);

    gpio_config_t gpio_configuration = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1 << pin_indicator),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE
    };

    gpio_config(&gpio_configuration);

    battery_manager->battery_voltage = 0;
    battery_manager->battery_status  = BATTERY_STATUS_CRITICAL;
}

void battery_manager_read(battery_manager_t* battery_manager){
    int voltage_raw;

    adc_oneshot_read(battery_manager->adc_handle, battery_manager->adc_channel, &voltage_raw);

    battery_manager->battery_voltage = ((float)voltage_raw / ADC_MAXIMUM) * VOLTAGE_REFERENCE * RESISTOR_RATIO;

    if(battery_manager->battery_voltage > VOLTAGE_LEVEL_LOW)
    {      
        battery_manager->battery_status = BATTERY_STATUS_NORMAL; 
        return;
    }
    if(battery_manager->battery_voltage > VOLTAGE_LEVEL_CRITICAL)
    {      
        battery_manager->battery_status = BATTERY_STATUS_LOW; 
        return;
    }
    if(battery_manager->battery_voltage <= VOLTAGE_LEVEL_CRITICAL)
    {      
        battery_manager->battery_status = BATTERY_STATUS_CRITICAL; 
        return;
    }
}

void battery_manager_indicate(battery_manager_t* battery_manager){
    static uint64_t time_us_current  = 0;
    static uint64_t time_us_previous = 0;
    static uint32_t time_us_delay    = 1 * SECOND_TO_MICROSECOND;

    static bool blink_indicator = false;

    time_us_current = esp_timer_get_time();

    if(blink_indicator && time_us_current - time_us_previous > time_us_delay)
    {
        static bool state_indicator = 0;

        state_indicator = !state_indicator;

        gpio_set_level(battery_manager->pin_indicator, state_indicator);
    }

    switch(battery_manager->battery_status)
    {
        case BATTERY_STATUS_NORMAL : 
            blink_indicator = false; 
            break; 
        case BATTERY_STATUS_LOW : 
            blink_indicator = true;
            time_us_delay = (uint32_t)(1.50 * SECOND_TO_MICROSECOND); 
            break;
        case BATTERY_STATUS_CRITICAL : 
            blink_indicator = true;
            time_us_delay = (uint32_t)(0.35 * SECOND_TO_MICROSECOND); 
            break;
        default :
            blink_indicator = true;
            time_us_delay = (uint32_t)(0.15 * SECOND_TO_MICROSECOND);
            break;
    }
}

//for esp32 s3
static adc_unit_t gpio_to_adc_unit(gpio_num_t gpio){
    switch (gpio){
        case GPIO_NUM_1: 
        case GPIO_NUM_2: 
        case GPIO_NUM_3: 
        case GPIO_NUM_4:
        case GPIO_NUM_5: 
        case GPIO_NUM_6: 
        case GPIO_NUM_7: 
        case GPIO_NUM_8:
        case GPIO_NUM_9:
        case GPIO_NUM_10:
            return ADC_UNIT_1;

        case GPIO_NUM_11: 
        case GPIO_NUM_12: 
        case GPIO_NUM_13: 
        case GPIO_NUM_14:
        case GPIO_NUM_15:
        case GPIO_NUM_16: 
        case GPIO_NUM_17:
        case GPIO_NUM_18: 
        case GPIO_NUM_19: 
        case GPIO_NUM_20:
            return ADC_UNIT_2;

        default: return -1; 
    }
}

static adc_channel_t gpio_to_adc_channel(gpio_num_t gpio){
    switch(gpio){

        case GPIO_NUM_1 : return ADC_CHANNEL_0;
        case GPIO_NUM_2 : return ADC_CHANNEL_1;
        case GPIO_NUM_3 : return ADC_CHANNEL_2;
        case GPIO_NUM_4 : return ADC_CHANNEL_3;
        case GPIO_NUM_5 : return ADC_CHANNEL_4;
        case GPIO_NUM_6 : return ADC_CHANNEL_5;
        case GPIO_NUM_7 : return ADC_CHANNEL_6;
        case GPIO_NUM_8 : return ADC_CHANNEL_7;
        case GPIO_NUM_9 : return ADC_CHANNEL_8;
        case GPIO_NUM_10: return ADC_CHANNEL_9;

        case GPIO_NUM_11: return ADC_CHANNEL_0;
        case GPIO_NUM_12: return ADC_CHANNEL_1;
        case GPIO_NUM_13: return ADC_CHANNEL_2;
        case GPIO_NUM_14: return ADC_CHANNEL_3;
        case GPIO_NUM_15: return ADC_CHANNEL_4;
        case GPIO_NUM_16: return ADC_CHANNEL_5;
        case GPIO_NUM_17: return ADC_CHANNEL_6;
        case GPIO_NUM_18: return ADC_CHANNEL_7;
        case GPIO_NUM_19: return ADC_CHANNEL_8;
        case GPIO_NUM_20: return ADC_CHANNEL_9;

        default: return -1; 
    }
}

/*
For normal esp32 modules (used for wroom 32e)

static adc_unit_t gpio_to_adc_unit(gpio_num_t gpio){
    switch (gpio){
        case GPIO_NUM_36: 
        case GPIO_NUM_37: 
        case GPIO_NUM_38: 
        case GPIO_NUM_39:
        case GPIO_NUM_32: 
        case GPIO_NUM_33: 
        case GPIO_NUM_34: 
        case GPIO_NUM_35: return ADC_UNIT_1;

        case GPIO_NUM_0: 
        case GPIO_NUM_2: 
        case GPIO_NUM_4: 
        case GPIO_NUM_12:
        case GPIO_NUM_13:
        case GPIO_NUM_14: 
        case GPIO_NUM_15:
        case GPIO_NUM_25: 
        case GPIO_NUM_26: 
        case GPIO_NUM_27: return ADC_UNIT_2;

        default: return -1; 
    }
}

static adc_channel_t gpio_to_adc_channel(gpio_num_t gpio){
    switch(gpio){
        case GPIO_NUM_36: return ADC_CHANNEL_0;
        case GPIO_NUM_37: return ADC_CHANNEL_1;
        case GPIO_NUM_38: return ADC_CHANNEL_2;
        case GPIO_NUM_39: return ADC_CHANNEL_3;
        case GPIO_NUM_32: return ADC_CHANNEL_4;
        case GPIO_NUM_33: return ADC_CHANNEL_5;
        case GPIO_NUM_34: return ADC_CHANNEL_6;
        case GPIO_NUM_35: return ADC_CHANNEL_7;
        case GPIO_NUM_0 : return ADC_CHANNEL_1;
        case GPIO_NUM_2 : return ADC_CHANNEL_2;
        case GPIO_NUM_4 : return ADC_CHANNEL_0;
        case GPIO_NUM_12: return ADC_CHANNEL_5;
        case GPIO_NUM_13: return ADC_CHANNEL_4;
        case GPIO_NUM_14: return ADC_CHANNEL_6;
        case GPIO_NUM_15: return ADC_CHANNEL_3;
        case GPIO_NUM_25: return ADC_CHANNEL_8;
        case GPIO_NUM_26: return ADC_CHANNEL_9;
        case GPIO_NUM_27: return ADC_CHANNEL_7;
        default: return -1; 
    }
}
*/