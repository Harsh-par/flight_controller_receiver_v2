#include "motor.h"

void motor_config(uint32_t timer, uint32_t resolution, uint32_t frequency);

void motor_init(motor_t* motor, uint8_t pin, uint8_t channel,uint32_t timer);

void motor_set(motor_t* motor, uint32_t duty);

void motor_config(uint32_t timer, uint32_t resolution, uint32_t frequency){
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = timer,
        .duty_resolution  = resolution, 
        .freq_hz          = frequency,               
        .clk_cfg          = LEDC_AUTO_CLK
    };

    ledc_timer_config(&ledc_timer);
}

void motor_init(motor_t* motor, uint8_t pin, uint8_t channel, uint32_t timer){
    motor->pin     = pin;
    motor->channel = channel;

    ledc_channel_config_t ledc_channel = {
        .gpio_num   = motor->pin,                       
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = motor->channel,
        .timer_sel  = timer,
        .duty       = 0,
        .hpoint     = 0
    };

    ledc_channel_config(&ledc_channel);
}

void motor_set(motor_t* motor, uint32_t duty){
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
}

void motor_stop(motor_t* motor){
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, MINIMUM_PWM_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
}
