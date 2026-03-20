#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"

#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "configuration.h"

#include "constants.h"
#include "globals.h"
#include "adc.h"
#include "battery_manager.h"
#include "mpu6050.h"
#include "pid.h"
#include "motor.h"
#include "receiver.h"

void task_core0_sensor(void *pvParameters);
void task_core0_receiver(void *pvParameters);
void task_core1_pid_angle(void *pvParameters);
void task_core1_pid_acro(void *pvParameters);

TaskHandle_t handle_core0_sensor    = NULL;
TaskHandle_t handle_core0_receiver  = NULL;
TaskHandle_t handle_core1_pid_angle = NULL;
TaskHandle_t handle_core1_pid_acro  = NULL;

battery_manager_t battery_manager;
flight_mode_t flight_mode = FC_ACRO_MODE;

extern mpu6050_t mpu6050;

pid_t pid_rate_roll;
pid_t pid_rate_pitch;
pid_t pid_angle_roll;
pid_t pid_angle_pitch;

motor_t motor_a;
motor_t motor_b;
motor_t motor_c;
motor_t motor_d;

volatile controller_data_t transmitter;

float ax, ay, az, gx, gy, gz;
float angle_roll, angle_pitch;

float kalman_roll  = 0, kalman_roll_uncertainty  = 2*2;
float kalman_pitch = 0, kalman_pitch_uncertainty = 2*2;

bool receiver_connected   = false;
bool receiver_armed       = false;  
bool receiver_voltage     = false;
bool receiver_calibrated  = false;

int64_t time_us_last_packet;

void app_main(void){
    adc_handle_init();
    receiver_init();

    motor_config(LEDC_TIMER_0, LEDC_TIMER_12_BIT, PWM_FREQUENCY);

    motor_init(&motor_a, MOTOR_PIN_A, LEDC_CHANNEL_0, LEDC_TIMER_0);
    motor_init(&motor_b, MOTOR_PIN_B, LEDC_CHANNEL_1, LEDC_TIMER_0);
    motor_init(&motor_c, MOTOR_PIN_C, LEDC_CHANNEL_2, LEDC_TIMER_0);
    motor_init(&motor_d, MOTOR_PIN_D, LEDC_CHANNEL_3, LEDC_TIMER_0);

    xTaskCreatePinnedToCore(task_core0_sensor,   "core0_sensor",   TASK_STACK_SIZE, NULL, 2, &handle_core0_sensor,    ESP_CORE_0);
    xTaskCreatePinnedToCore(task_core0_receiver, "core0_receiver", TASK_STACK_SIZE, NULL, 2, &handle_core0_receiver,  ESP_CORE_0);
    xTaskCreatePinnedToCore(task_core1_pid_acro, "core1_pid_acro", TASK_STACK_SIZE, NULL, 2, &handle_core1_pid_acro,  ESP_CORE_1);
    //xTaskCreatePinnedToCore(task_core1_pid_angle,"core1_pid_angle",TASK_STACK_SIZE, NULL, 2, &handle_core1_pid_angle, ESP_CORE_1);
}

void task_core0_sensor(void *pvParameters){
    int64_t time_us_current  = 0;
    int64_t time_us_previous = esp_timer_get_time(); 
    float   time_us_delta    = 0; 

    int64_t time_us_previous_calibration = 0; 

    float offset_accelerometer[3] = {OFFSET_AX, OFFSET_AY, OFFSET_AZ};
    float offset_gyroscope[3]     = {OFFSET_GX, OFFSET_GY, OFFSET_GZ};
    //float offset_angle[2]         = {OFFSET_ROLL, OFFSET_PITCH};

    mpu6050_init(I2C_MASTER_SDA, I2C_MASTER_SCL, I2C_MASTER_CLK_HZ);
    mpu6050_calibrate(offset_accelerometer, offset_gyroscope);
    mpu6050_reorientate(&mpu6050, IMU_CALIBRATION_SAMPLE_NUM);
    receiver_calibrated = true;

    while(true)
    {
        time_us_current = esp_timer_get_time();
        time_us_delta   = (time_us_current - time_us_previous) * MICROSECOND_TO_SECOND;

        mpu6050_read(&ax, &ay, &az, &gx, &gy, &gz);
        mpu6050_compute(ax, ay, az, &angle_roll, &angle_pitch);
        mpu6050_filter(&kalman_roll , &kalman_roll_uncertainty , gx, angle_roll , time_us_delta);
        mpu6050_filter(&kalman_pitch, &kalman_pitch_uncertainty, gy, angle_pitch, time_us_delta); 

        if(!receiver_armed && transmitter.joystick_my > IMU_CALIBRATION_THROTTLE_THRESHOLD && time_us_current - time_us_previous_calibration > BUTTON_DEBOUNCE_US)
        {
            receiver_calibrated = false;

            mpu6050_reorientate(&mpu6050, IMU_CALIBRATION_SAMPLE_NUM);
            receiver_calibrated = true;

            time_us_previous_calibration = time_us_current;
        }

        time_us_previous = time_us_current;
           
        #ifdef ENABLE_FC_DEBUG_IMU
            static uint32_t counter = 0;
            counter = (counter + 1) % 25;
            if(counter == 0)
            {
                #ifdef ENABLE_FC_DEBUG_IMU_GYRO 
                    ESP_LOGI("GYRO",  "gx : %.2f gy : %.2f gz : %.2f\n", gx, gy, gz);
                #endif
                #ifdef ENABLE_FC_DEBUG_IMU_ACCEL
                    ESP_LOGI("ACCEL", "ax : %.2f ay : %.2f az : %.2f\n", ax, ay, az);
                #endif
                #ifdef ENABLE_FC_DEBUG_IMU_ANGLE
                    ESP_LOGI("ANGLE", "pitch : %.2f roll : %.2f\n", kalman_pitch, kalman_roll);
                #endif
            }
        #endif

        vTaskDelay(TASK_CORE0_SENSOR_DELAY_MS / portTICK_PERIOD_MS);
    }
}

void task_core0_receiver(void *pvParameters){

    int64_t time_us_current  = 0;

    #ifdef ENABLE_FC_BATTERY_MANAGER
        battery_manager_init(&battery_manager, PIN_VOLTAGE_DIVIDER, PIN_INDICATOR_LED);
    #endif

    while(true)
    {
        time_us_current = esp_timer_get_time();

        /*
        Will use adc to read voltage of battery and get a battery status, status will affect the indicator led
        */
        #ifdef ENABLE_FC_BATTERY_MANAGER
            battery_manager_read(&battery_manager);
            battery_manager_indicate(&battery_manager);
        #endif

        /*
        System & Safety checks done to ensure receiver is still connected to transmitter, armed and battery level is not critical
        */
        receiver_check_connection(&receiver_connected, &receiver_armed, time_us_current);
        receiver_check_arming(&receiver_armed, &transmitter, &motor_a, &motor_b, &motor_c, &motor_d);
        receiver_check_voltage(&receiver_voltage, &receiver_armed, &battery_manager);
        
        vTaskDelay(TASK_CORE0_RECEIVER_DELAY_MS / portTICK_PERIOD_MS);
    }
}

void task_core1_pid_angle(void *pvParameters){

    /*
    Outer and Inner timer variables for the angle and rate pid controllers since they have different frequencies
    */
    int64_t outer_time_us_current  = 0;
    int64_t outer_time_us_previous = esp_timer_get_time(); 
    float   outer_time_us_delta    = 0; 

    int64_t inner_time_us_current  = 0;
    int64_t inner_time_us_previous = esp_timer_get_time(); 
    float   inner_time_us_delta    = 0; 
    
    pid_init(&pid_angle_roll,  KP_ANGLE_ROLL,  KI_ANGLE_ROLL,  KD_ANGLE_ROLL,  INTEGRAL_WINDUP, MINIMUM_ANGLE_RATE, MAXIMUM_ANGLE_RATE);
    pid_init(&pid_angle_pitch, KP_ANGLE_PITCH, KI_ANGLE_PITCH, KD_ANGLE_PITCH, INTEGRAL_WINDUP, MINIMUM_ANGLE_RATE, MAXIMUM_ANGLE_RATE);

    pid_init(&pid_rate_roll,  KP_RATE_ROLL,  KI_RATE_ROLL,  KD_RATE_ROLL,  INTEGRAL_WINDUP, MINIMUM_OUTPUT, MAXIMUM_OUTPUT);
    pid_init(&pid_rate_pitch, KP_RATE_PITCH, KI_RATE_PITCH, KD_RATE_PITCH, INTEGRAL_WINDUP, MINIMUM_OUTPUT, MAXIMUM_OUTPUT);

    float setpoint_rate_roll  = 0;
    float setpoint_rate_pitch = 0;

    while(true)
    {
        #ifdef ENABLE_FC_TRANSMITTER
        if(receiver_armed && receiver_connected && receiver_voltage && receiver_calibrated)
        #endif
        {
            outer_time_us_current = esp_timer_get_time();
            outer_time_us_delta   = (outer_time_us_current - outer_time_us_previous) * MICROSECOND_TO_SECOND; 

            static uint8_t inner_loop_counter = 0;
            
            /*
            Map transmitter joystick values to setpoints for PID controllers and base throttle
            */
            float motor_output_base    = map(transmitter.joystick_ly, -MAX_JOYSTICK, MAX_JOYSTICK, -BASE_THROTTLE, BASE_THROTTLE*0.70);
            float setpoint_angle_roll  = map(transmitter.joystick_rx, MIN_JOYSTICK, MAX_JOYSTICK, -BASE_ANGLE, BASE_ANGLE);
            float setpoint_angle_pitch = map(transmitter.joystick_ry, MIN_JOYSTICK, MAX_JOYSTICK, -BASE_ANGLE, BASE_ANGLE);

            /*
            Using cascaded PID controllers to calculate the motor output 

            setpoint_angle => [ANGLE_CONTROLLER] => setpoint_rate => [RATE_CONTROLLER] => motor_output
            ANGLE_CONTROLLER running at 250 Hz and RATE_CONTROLLER running at 1kHz

            To run angle controller at 250 Hz inside the pid task inner_loop_counter is used
            this allows the angle pid controller to run once every 4 loops, while the rate runs every loop
            */
            if(inner_loop_counter == TASK_FREQUENCY_DIVIDER)
            {
                inner_time_us_current = esp_timer_get_time();
                inner_time_us_delta   = (inner_time_us_current - inner_time_us_previous) * MICROSECOND_TO_SECOND;

                setpoint_rate_roll  = pid_compute(&pid_angle_roll,  setpoint_angle_roll,  kalman_roll,  inner_time_us_delta);
                setpoint_rate_pitch = pid_compute(&pid_angle_pitch, setpoint_angle_pitch, -kalman_pitch, inner_time_us_delta);

                inner_loop_counter = 0;

                inner_time_us_previous = inner_time_us_current;
            }

            float motor_output_roll  = pid_compute_rate(&pid_rate_roll,  setpoint_rate_roll, -gx, outer_time_us_delta);
            float motor_output_pitch = pid_compute_rate(&pid_rate_pitch, setpoint_rate_pitch, gy, outer_time_us_delta);

            /*
            Compute motor output for each motor based on outputs from PID controllers then map each output to pwm duty
            */
            float motor_output_a = motor_output_base + motor_output_pitch + motor_output_roll; 
            float motor_output_b = motor_output_base + motor_output_pitch - motor_output_roll; 
            float motor_output_c = motor_output_base - motor_output_pitch - motor_output_roll; 
            float motor_output_d = motor_output_base - motor_output_pitch + motor_output_roll;

            motor_output_a = MAPF(motor_output_a, MINIMUM_THROTTLE, MAXIMUM_THROTTLE, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);
            motor_output_b = MAPF(motor_output_b, MINIMUM_THROTTLE, MAXIMUM_THROTTLE, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);
            motor_output_c = MAPF(motor_output_c, MINIMUM_THROTTLE, MAXIMUM_THROTTLE, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);
            motor_output_d = MAPF(motor_output_d, MINIMUM_THROTTLE, MAXIMUM_THROTTLE, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);

            motor_output_a = CONSTRAIN(motor_output_a, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);
            motor_output_b = CONSTRAIN(motor_output_b, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);
            motor_output_c = CONSTRAIN(motor_output_c, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);
            motor_output_d = CONSTRAIN(motor_output_d, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);

            #ifdef ENABLE_FC_DEBUG_PID
                static uint32_t counter = 0;
                counter = (counter + 1) % 20;
                if(counter == 0) 
                    ESP_LOGI("MOTOR", "A : %.2f B : %.2f C : %.2f D : %.2f\n", motor_output_a, motor_output_b, motor_output_c, motor_output_d);
            #endif

            #ifdef ENABLE_FC_TRANSMITTER
                motor_set(&motor_a, motor_output_a);
                motor_set(&motor_b, motor_output_b);
                motor_set(&motor_c, motor_output_c);
                motor_set(&motor_d, motor_output_d);
            #endif

            inner_loop_counter++;

            outer_time_us_previous = outer_time_us_current;
        }
        
        vTaskDelay(TASK_CORE1_PID_DELAY_MS / portTICK_PERIOD_MS);
    }
}


void task_core1_pid_acro(void *pvParameters){

    int64_t outer_time_us_current  = 0;
    int64_t outer_time_us_previous = esp_timer_get_time(); 
    float   outer_time_us_delta    = 0; 
    
    pid_init(&pid_rate_roll,  KP_RATE_ROLL,  KI_RATE_ROLL,  KD_RATE_ROLL,  INTEGRAL_WINDUP, MINIMUM_OUTPUT, MAXIMUM_OUTPUT);
    pid_init(&pid_rate_pitch, KP_RATE_PITCH, KI_RATE_PITCH, KD_RATE_PITCH, INTEGRAL_WINDUP, MINIMUM_OUTPUT, MAXIMUM_OUTPUT);

    while(true)
    {
        #ifdef ENABLE_FC_TRANSMITTER
        if(receiver_armed && receiver_connected && receiver_voltage && receiver_calibrated)
        #endif
        {
            outer_time_us_current = esp_timer_get_time();
            outer_time_us_delta   = (outer_time_us_current - outer_time_us_previous) * MICROSECOND_TO_SECOND; 
            /*
            Map transmitter joystick values to setpoints for PID controllers and base throttle
            */
            float motor_output_base   = map(transmitter.joystick_ly, -MAX_JOYSTICK, MAX_JOYSTICK, -BASE_THROTTLE, BASE_THROTTLE*0.70);
            float setpoint_rate_roll  = map(transmitter.joystick_rx, MIN_JOYSTICK, MAX_JOYSTICK, -BASE_ANGLE*8, BASE_ANGLE*8);
            float setpoint_rate_pitch = map(transmitter.joystick_ry, MIN_JOYSTICK, MAX_JOYSTICK, -BASE_ANGLE*8, BASE_ANGLE*8);

            float motor_output_roll  = pid_compute_rate(&pid_rate_roll,  setpoint_rate_roll, -gx, outer_time_us_delta);
            float motor_output_pitch = pid_compute_rate(&pid_rate_pitch, setpoint_rate_pitch, gy, outer_time_us_delta);

            /*
            Compute motor output for each motor based on outputs from PID controllers then map each output to pwm duty
            */
            float motor_output_a = motor_output_base + motor_output_pitch + motor_output_roll; 
            float motor_output_b = motor_output_base + motor_output_pitch - motor_output_roll; 
            float motor_output_c = motor_output_base - motor_output_pitch - motor_output_roll; 
            float motor_output_d = motor_output_base - motor_output_pitch + motor_output_roll;

            motor_output_a = MAPF(motor_output_a, MINIMUM_THROTTLE, MAXIMUM_THROTTLE, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);
            motor_output_b = MAPF(motor_output_b, MINIMUM_THROTTLE, MAXIMUM_THROTTLE, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);
            motor_output_c = MAPF(motor_output_c, MINIMUM_THROTTLE, MAXIMUM_THROTTLE, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);
            motor_output_d = MAPF(motor_output_d, MINIMUM_THROTTLE, MAXIMUM_THROTTLE, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);

            motor_output_a = CONSTRAIN(motor_output_a, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);
            motor_output_b = CONSTRAIN(motor_output_b, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);
            motor_output_c = CONSTRAIN(motor_output_c, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);
            motor_output_d = CONSTRAIN(motor_output_d, MINIMUM_PWM_DUTY, MAXIMUM_PWM_DUTY);

            #ifdef ENABLE_FC_DEBUG_PID
                static uint32_t counter = 0;
                counter = (counter + 1) % 15;
                if(counter == 0) 
                    ESP_LOGI("MOTOR", "A : %.2f B : %.2f C : %.2f D : %.2f\n", motor_output_a, motor_output_b, motor_output_c, motor_output_d);
            #endif

            #ifdef ENABLE_FC_TRANSMITTER
                motor_set(&motor_a, motor_output_a);
                motor_set(&motor_b, motor_output_b);
                motor_set(&motor_c, motor_output_c);
                motor_set(&motor_d, motor_output_d);
            #endif

            outer_time_us_previous = outer_time_us_current;
        }
        
        vTaskDelay(TASK_CORE1_PID_DELAY_MS / portTICK_PERIOD_MS);
    }
}
