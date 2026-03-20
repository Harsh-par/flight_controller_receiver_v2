#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

#include <stdint.h>
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc_cal.h"

typedef enum{
    BATTERY_STATUS_NORMAL,
    BATTERY_STATUS_LOW,
    BATTERY_STATUS_CRITICAL
} battery_status_t;

typedef struct{
    adc_channel_t                 adc_channel;
    adc_oneshot_unit_handle_t     adc_handle;
    
    float            battery_voltage;
    battery_status_t battery_status;
    uint8_t          pin_battery;
    uint8_t          pin_indicator;

} battery_manager_t;

void battery_manager_init(battery_manager_t* battery_manager, uint8_t pin_battery, uint8_t pin_indicator);
void battery_manager_read(battery_manager_t* battery_manager);
void battery_manager_indicate(battery_manager_t* battery_manager);

#endif