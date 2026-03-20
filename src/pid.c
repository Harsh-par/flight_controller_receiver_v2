#include "pid.h"

void  pid_init(pid_t* pid, float kp, float ki, float kd, float integral_windup, float minimum_output, float maximum_output);

float pid_compute(pid_t* pid, float setpoint, float measurement, float dt);

float pid_compute_rate(pid_t* pid, float setpoint, float measurement, float dt);

float pid_clamp(float clamp_value, const float minimum_value, const float maximum_value);

void pid_init(pid_t* pid, float kp, float ki, float kd, float integral_windup, float minimum_output, float maximum_output){
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral_previous    = 0;
    pid->measurement_previous = 0;

    pid->integral_windup = integral_windup;
    pid->minimum_output  = minimum_output;
    pid->maximum_output  = maximum_output;
}

float pid_compute(pid_t* pid, float setpoint, float measurement, float dt){
    if(dt == 0){ dt = 0.002; }
    
    float error = setpoint - measurement;

    //float derivative = (error - pid->error_previous) / dt;

    float integral = pid->integral_previous + error * dt;

    integral = pid_clamp(integral, -pid->integral_windup, pid->integral_windup); 

    float output =  pid->kp * error + pid->ki * integral; //+ pid->kd * derivative;

    pid->error_previous    = error;
    pid->integral_previous = integral;

    return pid_clamp(output, pid->minimum_output, pid->maximum_output);
}

float pid_compute_rate(pid_t* pid, float setpoint, float measurement, float dt){
    if(dt == 0){ dt = 0.002; }
    
    float error = setpoint - measurement;

    float derivative = -(measurement - pid->measurement_previous) / dt;

    float integral = pid->integral_previous + error * dt;

    integral = pid_clamp(integral, -pid->integral_windup, pid->integral_windup); 

    float output =  pid->kp * error + pid->ki * integral + pid->kd * derivative;

    pid->error_previous       = error;
    pid->integral_previous    = integral;
    pid->measurement_previous = measurement;

    return pid_clamp(output, pid->minimum_output, pid->maximum_output);
}

float pid_clamp(float clamp_value, const float minimum_value, const float maximum_value){
    if(clamp_value > maximum_value){
        return maximum_value;
    }
    else if(clamp_value < minimum_value){
        return minimum_value;
    }
    else return clamp_value;
}