#ifndef CONSTANTS_H
#define CONSTANTS_H

#define RADIAN_TO_DEGREE      57.2957795
#define MICROSECOND_TO_SECOND 1e-6f
#define SECOND_TO_MICROSECOND 1e+6f

//serial baut rate
#define RATE_SERIAL_BAUD   115200
//i2c master port number
#define I2C_MASTER_NUM     I2C_NUM_0
//i2c master sda gpio number
#define I2C_MASTER_SDA     5 //D4
//i2c master scl gpio number
#define I2C_MASTER_SCL     6 //D5
//i2c master clock speed (Hz)
#define I2C_MASTER_CLK_HZ  400000
//i2c read/write timeout delay ms
#define I2C_TIMEOUT_MS     1000

#define IMU_CALIBRATION_SAMPLE_NUM 6000

#define IMU_CALIBRATION_THROTTLE_THRESHOLD 30

//voltage reference for adc reading (changes based on adc attenuation)
#define VOLTAGE_REFERENCE 3.9f 
#define VOLTAGE_REFERENCE_DEFAULT_MV 1100
#define RESISTOR_TOP      100e3f //33e3f
#define RESISTOR_BOTTOM   10e3f
#define RESISTOR_RATIO    (RESISTOR_TOP + RESISTOR_BOTTOM) / (RESISTOR_BOTTOM)
#define ADC_MAXIMUM       4095.f
#define MILLIVOLT_TO_VOLT 1e-3f
#define VOLTAGE_LEVEL_NORMAL   4.2f
#define VOLTAGE_LEVEL_LOW      3.5f
#define VOLTAGE_LEVEL_CRITICAL 3.3f

//ledc pwm frequency
#define PWM_FREQUENCY 5000 //CHANGE TO 20kHz

#define MOTOR_PIN_A         1 //D0
#define MOTOR_PIN_B         2 //D1
#define MOTOR_PIN_C         3 //D2 
#define MOTOR_PIN_D         4 //D3

#define PIN_VOLTAGE_DIVIDER 7 //D8
#define PIN_INDICATOR_LED   8 //D9
#define PIN_ONBOARD_LED     21

//mpu6050 axis offsets
#define OFFSET_AX    -0.05f
#define OFFSET_AY    -0.04f
#define OFFSET_AZ    -0.04f
#define OFFSET_GX    -4.90f
#define OFFSET_GY     2.12f
#define OFFSET_GZ    -0.52f
#define OFFSET_ROLL  0.0f
#define OFFSET_PITCH 0.0f

#define PACKET_TIMEOUT_US  (0.2f * SECOND_TO_MICROSECOND)
#define BUTTON_DEBOUNCE_US (0.3f * SECOND_TO_MICROSECOND)

//pid roll & pid pitch controller values
#define KP_RATE_ROLL  0.2f
#define KI_RATE_ROLL  0
#define KD_RATE_ROLL  0

#define KP_RATE_PITCH 0.2f
#define KI_RATE_PITCH 0
#define KD_RATE_PITCH 0

#define KP_ANGLE_ROLL  2.5f
#define KI_ANGLE_ROLL  0.0f 
#define KD_ANGLE_ROLL  2.0f 

#define KP_ANGLE_PITCH 2.5f //6.0f
#define KI_ANGLE_PITCH 0.0f
#define KD_ANGLE_PITCH 2.0f

#define INTEGRAL_WINDUP 100.f

#define MINIMUM_ANGLE_RATE -250.f
#define MAXIMUM_ANGLE_RATE  250.f

#define MINIMUM_OUTPUT -1000.f 
#define MAXIMUM_OUTPUT  1000.f

#define MIN_JOYSTICK -100.f
#define MAX_JOYSTICK  100.f

#define MINIMUM_THROTTLE -2500.f
#define MAXIMUM_THROTTLE  2500.f
#define BASE_THROTTLE     2580.f
#define BASE_ANGLE        30.f

#define MINIMUM_PWM_DUTY 0
#define MAXIMUM_PWM_DUTY 4095

#define ESP_NOW_CHANNEL 1
#define RX_SSID        "RX_1"
#define RX_PASSWORD    "RX_1_Password"

//esp32 core 0
#define ESP_CORE_0 0 
//esp32 core 1
#define ESP_CORE_1 1 
//freertos task stack size
#define TASK_STACK_SIZE 2048*2
//freertos task delay microseconds (periodic)
#define TASK_DELAY_MS 10
#define TASK_FREQUENCY_DIVIDER       4
#define TASK_CORE0_SENSOR_DELAY_MS   20
#define TASK_CORE0_RECEIVER_DELAY_MS 100
#define TASK_CORE1_PID_DELAY_MS      10

#endif