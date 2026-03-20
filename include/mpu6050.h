#ifndef MPU6050_H
#define MPU6050_H

#define ADDR_I2C_MPU6050      0x68
#define ADDR_REG_PWR_MGMT_1   0x6B
#define ADDR_REG_PWR_MGMT_2   0x6C
#define ADDR_REG_CONFIG       0x1A
#define ADDR_REG_CONFIG_GYRO  0x1B
#define ADDR_REG_CONFIG_ACCEL 0x1C
#define ADDR_REG_OUT_GYRO     0x43
#define ADDR_REG_OUT_TEMP     0x41
#define ADDR_REG_OUT_ACCEL    0x3B

typedef struct{
    float offset_ax, offset_ay, offset_az;
    float offset_gx, offset_gy, offset_gz;
    float offset_angle_pitch, offset_angle_roll;

    void (*init)(const uint8_t pin_sda, const uint8_t pin_scl, const uint32_t clock_speed);

    void (*calibrate)(float offset_accelerometer[3], float offset_gyroscope[3]);

    void (*read)(float* ax, float* ay, float* az, float* gx, float* gy, float* gz);

    void (*compute)(const float ax, const float ay, const float az, float* angle_roll, float* angle_pitch);

    void (*filter)(float* kalman_state, float* kalman_uncertainty, const float kalman_input, const float kalman_measurement, const float dt);
} mpu6050_t;

extern mpu6050_t mpu6050;

void mpu6050_init(const uint8_t pin_sda, const uint8_t pin_scl, const uint32_t clock_speed);
void mpu6050_calibrate(float offset_accelerometer[3], float offset_gyroscope[3]);
void mpu6050_reorientate(mpu6050_t* mpu6050, uint32_t sample_size);
void mpu6050_read(float* ax, float* ay, float* az, float* gx, float* gy, float* gz);
void mpu6050_compute(const float ax, const float ay, const float az, float* angle_roll, float* angle_pitch);
void mpu6050_filter(float* kalman_state, float* kalman_uncertainty, const float kalman_input, const float kalman_measurement, const float dt);

#endif