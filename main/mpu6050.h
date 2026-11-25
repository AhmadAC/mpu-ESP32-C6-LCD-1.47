#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// MPU-6050 wiring on Waveshare ESP32-C6-LCD-1.47 (I2C0):
//   VCC -> 3.3V
//   GND -> GND
//   SDA -> GPIO 18 (Safe, unshared header pin)
//   SCL -> GPIO 19 (Safe, unshared header pin)
//   AD0 -> GND (address 0x68) or 3.3V (address 0x69)

#define MPU6050_SDA_PIN          GPIO_NUM_18
#define MPU6050_SCL_PIN          GPIO_NUM_19
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