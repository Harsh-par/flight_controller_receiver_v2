#include <math.h>

#include "driver/i2c.h"

#include "mpu6050.h"
#include "constants.h"

void mpu6050_init(const uint8_t pin_sda, const uint8_t pin_scl, const uint32_t clock_speed);

void mpu6050_calibrate(float offset_accelerometer[3], float offset_gyroscope[3]);

void mpu6050_reorientate(mpu6050_t* mpu6050, uint32_t sample_size);

void mpu6050_read(float* ax, float* ay, float* az, float* gx, float* gy, float* gz);

void mpu6050_compute(const float ax, const float ay, const float az, float* angle_roll, float* angle_pitch);

void mpu6050_filter(float* kalman_state, float* kalman_uncertainty, const float kalman_input, const float kalman_measurement, const float dt);

mpu6050_t mpu6050 = {
    .init      = mpu6050_init,
    .calibrate = mpu6050_calibrate,
    .read      = mpu6050_read,
    .compute   = mpu6050_compute,
    .filter    = mpu6050_filter
};

void mpu6050_init(const uint8_t pin_sda, const uint8_t pin_scl, const uint32_t clock_speed){
     i2c_config_t i2c_config = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = pin_sda,
        .scl_io_num       = pin_scl,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clock_speed,
    };

    i2c_param_config(I2C_MASTER_NUM, &i2c_config);
    i2c_driver_install(I2C_MASTER_NUM, i2c_config.mode, 0, 0, 0);

    uint8_t buffer[2];

    buffer[0] = ADDR_REG_PWR_MGMT_1; 
    buffer[1] = 0x00;
    i2c_master_write_to_device(I2C_MASTER_NUM, ADDR_I2C_MPU6050, buffer, sizeof(buffer), I2C_TIMEOUT_MS/portTICK_PERIOD_MS);

    buffer[0] = ADDR_REG_PWR_MGMT_2; 
    buffer[1] = 0x00;
    i2c_master_write_to_device(I2C_MASTER_NUM, ADDR_I2C_MPU6050, buffer, sizeof(buffer), I2C_TIMEOUT_MS/portTICK_PERIOD_MS);

    buffer[0] = ADDR_REG_CONFIG; 
    buffer[1] = 0x05;
    i2c_master_write_to_device(I2C_MASTER_NUM, ADDR_I2C_MPU6050, buffer, sizeof(buffer), I2C_TIMEOUT_MS/portTICK_PERIOD_MS);

    buffer[0] = ADDR_REG_CONFIG_GYRO; 
    buffer[1] = 0x08;
    i2c_master_write_to_device(I2C_MASTER_NUM, ADDR_I2C_MPU6050, buffer, sizeof(buffer), I2C_TIMEOUT_MS/portTICK_PERIOD_MS);

    buffer[0] = ADDR_REG_CONFIG_ACCEL; 
    buffer[1] = 0x10;
    i2c_master_write_to_device(I2C_MASTER_NUM, ADDR_I2C_MPU6050, buffer, sizeof(buffer), I2C_TIMEOUT_MS/portTICK_PERIOD_MS);

    //init the onboard led (used to show when in calibration mode)
    gpio_config_t gpio_configuration = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1 << PIN_ONBOARD_LED),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE
    };

    gpio_config(&gpio_configuration);
}

void mpu6050_calibrate(float offset_accelerometer[3], float offset_gyroscope[3]){
    gpio_set_level(PIN_ONBOARD_LED, false);

    mpu6050.offset_ax = offset_accelerometer[0];
    mpu6050.offset_ay = offset_accelerometer[1];
    mpu6050.offset_az = offset_accelerometer[2];

    mpu6050.offset_gx = offset_gyroscope[0];
    mpu6050.offset_gy = offset_gyroscope[1];
    mpu6050.offset_gz = offset_gyroscope[2];

    mpu6050.offset_angle_roll  = 0.0f;
    mpu6050.offset_angle_pitch = 0.0f;

    gpio_set_level(PIN_ONBOARD_LED, true);
}

void mpu6050_reorientate(mpu6050_t* mpu6050, uint32_t sample_size){
    float ax, ay, az;
    float gx, gy, gz;
    float angle_roll, angle_pitch;
    float total_angle_roll = 0, total_angle_pitch = 0;

    for(uint32_t samples = 0; samples < sample_size; samples++)
    {   
        mpu6050_read(&ax, &ay, &az, &gx, &gy, &gz);
        mpu6050_compute(ax, ay, az, &angle_roll, &angle_pitch); 
        total_angle_roll  += angle_roll;
        total_angle_pitch += angle_pitch;
    }

    mpu6050->offset_angle_roll  = total_angle_roll / sample_size;
    mpu6050->offset_angle_pitch = total_angle_pitch / sample_size;
}

void mpu6050_read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz){
    uint8_t buffer[14]; 
    uint8_t reg = ADDR_REG_OUT_ACCEL;

    i2c_master_write_read_device(I2C_NUM_0, ADDR_I2C_MPU6050, &reg, sizeof(reg), buffer, sizeof(buffer), I2C_TIMEOUT_MS/portTICK_PERIOD_MS);
    
    int16_t raw_ax = (buffer[0] << 8) | buffer[1];
    int16_t raw_ay = (buffer[2] << 8) | buffer[3];
    int16_t raw_az = (buffer[4] << 8) | buffer[5];

    int16_t raw_gx = (buffer[8]  << 8) | buffer[9];
    int16_t raw_gy = (buffer[10] << 8) | buffer[11];
    int16_t raw_gz = (buffer[12] << 8) | buffer[13];

    *ax = ((float)raw_ax) / 4096.0f - mpu6050.offset_ax;
    *ay = ((float)raw_ay) / 4096.0f - mpu6050.offset_ay;
    *az = ((float)raw_az) / 4096.0f - mpu6050.offset_az;

    *gx = ((float)raw_gx) / 65.5f - mpu6050.offset_gx;
    *gy = ((float)raw_gy) / 65.5f - mpu6050.offset_gy;
    *gz = ((float)raw_gz) / 65.5f - mpu6050.offset_gz;
}

void mpu6050_compute(const float ax, const float ay, const float az, float* angle_roll, float* angle_pitch){
  *angle_roll  = atanf( ay / sqrtf(ax * ax + az * az)) * (RADIAN_TO_DEGREE) - mpu6050.offset_angle_roll;
  *angle_pitch = atanf(-ax / sqrtf(ay * ay + az * az)) * (RADIAN_TO_DEGREE) - mpu6050.offset_angle_pitch;
}

void mpu6050_filter(float* kalman_state, float* kalman_uncertainty, const float kalman_input, const float kalman_measurement, const float dt){
    float state       = *kalman_state; 
    float uncertainty = *kalman_uncertainty;

    state       = state + kalman_input * dt;
    uncertainty = uncertainty + 4*4 * dt * dt;

    float kalman_gain = uncertainty / (uncertainty + 3*3);

    state       = state + kalman_gain * (kalman_measurement - state);
    uncertainty = (1 - kalman_gain) * uncertainty;

    *kalman_state       = state;
    *kalman_uncertainty = uncertainty;
}