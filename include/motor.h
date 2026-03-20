#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#include "driver/ledc.h"

#include "constants.h"

typedef struct {
    uint8_t pin;
    uint8_t channel;
} motor_t;

void motor_config(uint32_t timer, uint32_t resolution, uint32_t frequency);
void motor_init(motor_t* motor, uint8_t pin, uint8_t channel,uint32_t timer);
void motor_set(motor_t* motor, uint32_t duty);
void motor_stop(motor_t* motor);

#endif