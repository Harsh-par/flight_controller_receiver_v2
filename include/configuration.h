#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/*
When enabled battery manager will be used to read voltage of battery and disarm if voltage too low
*/
//#define ENABLE_FC_BATTERY_MANAGER 1

/*
When enabled Receiver must be connected & armed by Transmiter
When disabled debugging logs will print to serial monitor
*/
#define ENABLE_FC_TRANSMITTER 1

typedef enum{
    FC_ACRO_MODE,
    FC_ANGLE_MODE
} flight_mode_t;

/*
When enabled can be used to debug on serial monitor
NOTE : ENABLE_FC_TRANSMITTER must not be defined
NOTE : Must enable DEBUG_IMU aswell as specific DEBUG_IMU values you want to see
*/
#ifndef ENABLE_FC_TRANSMITTER
    #define ENABLE_FC_DEBUG_PID       1
    //#define ENABLE_FC_DEBUG_IMU       1
    #define ENABLE_FC_DEBUG_IMU_GYRO  1
    #define ENABLE_FC_DEBUG_IMU_ACCEL 1
    #define ENABLE_FC_DEBUG_IMU_ANGLE 1
#endif

#endif