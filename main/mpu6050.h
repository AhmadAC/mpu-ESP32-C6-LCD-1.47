// main\mpu6050.h
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "node_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_SDA_PIN          BOARD_MPU_SDA_PIN
#define MPU6050_SCL_PIN          BOARD_MPU_SCL_PIN
#define MPU6050_I2C_FREQ_HZ      400000

#define MPU6050_ADDR_LOW         0x68
#define MPU6050_ADDR_HIGH        0x69

#define MPU6050_ACCEL_LSB_PER_G  16384.0f
#define MPU6050_GYRO_LSB_PER_DPS 65.5f

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} mpu6050_raw_t;

bool mpu6050_init(void);
bool mpu6050_is_present(void);
uint8_t mpu6050_address(void);
bool mpu6050_read_raw(mpu6050_raw_t *out);

#ifdef __cplusplus
}
#endif