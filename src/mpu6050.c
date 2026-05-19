#include <math.h>

#include "driver/i2c.h"
#include "mpu6050.h"
#include "constants.h"


void mpu6050_init(volatile mpu6050_t *mpu6050, uint32_t i2c_clk, gpio_num_t i2c_sda, gpio_num_t i2c_scl)
{
    mpu6050->i2c_sda = i2c_sda;
    mpu6050->i2c_scl = i2c_scl;

    mpu6050->kalman_pitch_uncertainty = 2 * 2;
    mpu6050->kalman_roll_uncertainty  = 2 * 2;

    i2c_config_t i2c_config = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = i2c_sda,
        .scl_io_num       = i2c_scl,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_clk,
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
    buffer[1] = 0x00;
    i2c_master_write_to_device(I2C_MASTER_NUM, ADDR_I2C_MPU6050, buffer, sizeof(buffer), I2C_TIMEOUT_MS/portTICK_PERIOD_MS);

    buffer[0] = ADDR_REG_CONFIG_ACCEL; 
    buffer[1] = 0x10;
    i2c_master_write_to_device(I2C_MASTER_NUM, ADDR_I2C_MPU6050, buffer, sizeof(buffer), I2C_TIMEOUT_MS/portTICK_PERIOD_MS);
}

void mpu6050_read(volatile mpu6050_t *mpu6050)
{
    uint8_t buffer[14]; 
    uint8_t reg = ADDR_REG_OUT_ACCEL;

    i2c_master_write_read_device(I2C_NUM_0, ADDR_I2C_MPU6050, &reg, sizeof(reg), buffer, sizeof(buffer), I2C_TIMEOUT_MS/portTICK_PERIOD_MS);
    
    int16_t raw_ax = (buffer[0] << 8) | buffer[1];
    int16_t raw_ay = (buffer[2] << 8) | buffer[3];
    int16_t raw_az = (buffer[4] << 8) | buffer[5];

    int16_t raw_gx = (buffer[8]  << 8) | buffer[9];
    int16_t raw_gy = (buffer[10] << 8) | buffer[11];
    int16_t raw_gz = (buffer[12] << 8) | buffer[13];

    mpu6050->ax = ((float)raw_ax) / 4096.0f - mpu6050->ax_offset;
    mpu6050->ay = ((float)raw_ay) / 4096.0f - mpu6050->ay_offset;
    mpu6050->az = ((float)raw_az) / 4096.0f - mpu6050->az_offset;

    mpu6050->gx = ((float)raw_gx) / 131.0f - mpu6050->gx_offset;
    mpu6050->gy = ((float)raw_gy) / 131.0f - mpu6050->gy_offset;
    mpu6050->gz = ((float)raw_gz) / 131.0f - mpu6050->gz_offset;
}

void mpu6050_compute(volatile mpu6050_t *mpu6050)
{
    mpu6050->angle_roll  = atan( mpu6050->ay / sqrt( (mpu6050->ax * mpu6050->ax) + (mpu6050->az * mpu6050->az) )) * (RADIAN_TO_DEGREE);
    mpu6050->angle_pitch = atan(-mpu6050->ax / sqrt( (mpu6050->ay * mpu6050->ay) + (mpu6050->az * mpu6050->az) )) * (RADIAN_TO_DEGREE);
}

void mpu6050_calibrate(volatile mpu6050_t *mpu6050)
{
    float accelerometer_values[3] = {0, 0, 0};
    float gyroscope_values[3] = {0, 0, 0};

    for(int i = 0; i < IMU_CALIBRATION_ITERATIONS; i++)
    {
        mpu6050_read(mpu6050);

        accelerometer_values[0] = accelerometer_values[0] + mpu6050->ax;
        accelerometer_values[1] = accelerometer_values[1] + mpu6050->ay;
        accelerometer_values[2] = accelerometer_values[2] + mpu6050->az;

        gyroscope_values[0] = gyroscope_values[0] + mpu6050->gx;
        gyroscope_values[1] = gyroscope_values[1] + mpu6050->gy;
        gyroscope_values[2] = gyroscope_values[2] + mpu6050->gz;
    }

    mpu6050->ax_offset = (accelerometer_values[0] / IMU_CALIBRATION_ITERATIONS);
    mpu6050->ay_offset = (accelerometer_values[1] / IMU_CALIBRATION_ITERATIONS);
    mpu6050->az_offset = (accelerometer_values[2] / IMU_CALIBRATION_ITERATIONS);

    mpu6050->gx_offset = (gyroscope_values[0] / IMU_CALIBRATION_ITERATIONS);
    mpu6050->gy_offset = (gyroscope_values[1] / IMU_CALIBRATION_ITERATIONS);
    mpu6050->gz_offset = (gyroscope_values[2] / IMU_CALIBRATION_ITERATIONS);
}

void mpu6050_filter_pitch(volatile mpu6050_t *mpu6050, const float dt)
{
    float state       = mpu6050->kalman_pitch; 
    float uncertainty = mpu6050->kalman_pitch_uncertainty;
    float input       = mpu6050->gy;
    float measurement = mpu6050->angle_pitch;

    state = state + input * dt;
    uncertainty = uncertainty + 4*4 * dt * dt;

    float kalman_gain = uncertainty / (uncertainty + 3*3);

    state = state + kalman_gain * (measurement - state);
    uncertainty = (1 - kalman_gain) * uncertainty;

    mpu6050->kalman_pitch             = state;
    mpu6050->kalman_pitch_uncertainty = uncertainty;
}

void mpu6050_filter_roll(volatile mpu6050_t *mpu6050, const float dt)
{
    float state       = mpu6050->kalman_roll; 
    float uncertainty = mpu6050->kalman_roll_uncertainty;
    float input       = mpu6050->gx;
    float measurement = mpu6050->angle_roll;

    state = state + input * dt;
    uncertainty = uncertainty + 4*4 * dt * dt;

    float kalman_gain = uncertainty / (uncertainty + 3*3);

    state = state + kalman_gain * (measurement - state);
    uncertainty = (1 - kalman_gain) * uncertainty;

    mpu6050->kalman_roll             = state;
    mpu6050->kalman_roll_uncertainty = uncertainty;
}