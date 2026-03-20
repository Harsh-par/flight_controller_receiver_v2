#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float error_previous;
    float measurement_previous;
    float integral_previous;
    float integral_windup;
    float minimum_output, maximum_output;
} pid_t;

void  pid_init(pid_t* pid, float kp, float ki, float kd, float integral_windup, float minimum_output, float maximum_output);
float pid_compute(pid_t* pid, float setpoint, float measurement, float dt);
float pid_compute_rate(pid_t* pid, float setpoint, float measurement, float dt);
float pid_clamp(float clamp_value, const float minimum_value, const float maximum_value);

#endif