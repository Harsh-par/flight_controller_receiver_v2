#ifndef RECEIVER_H
#define RECEIVER_H

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "constants.h"
#include "globals.h"
#include "battery_manager.h"
#include "motor.h"

typedef struct{
    int16_t joystick_lx, joystick_ly, button_l;
    int16_t joystick_mx, joystick_my, button_m;
    int16_t joystick_rx, joystick_ry, button_r;
} controller_data_t;

extern volatile controller_data_t transmitter;

void receiver_init();
void receiver_callback(const esp_now_recv_info_t *receive_info, const uint8_t *data, int data_length);

void receiver_check_connection(bool* receiver_connected, bool* receiver_armed, int64_t time_us_current);
void receiver_check_arming(bool* receiver_armed, volatile controller_data_t* transmitter, motor_t* motor_a, motor_t* motor_b, motor_t* motor_c, motor_t* motor_d);
void receiver_check_voltage(bool* receiver_voltage, bool* receiver_armed, battery_manager_t* battery_manager);

#endif