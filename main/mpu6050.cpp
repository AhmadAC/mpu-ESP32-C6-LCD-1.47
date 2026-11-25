#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPU6050";

#define I2C_PORT             I2C_NUM_0
#define I2C_TIMEOUT_MS       50

#define REG_SMPLRT_DIV       0x19
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_ACCEL_XOUT_H     0x3B
#define REG_PWR_MGMT_1       0x6B
#define REG_WHO_AM_I         0x75

static bool    s_driver_installed = false;
static bool    s_present          = false;
static uint8_t s_addr             = MPU6050_ADDR_LOW;

static esp_err_t write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_master_write_to_device(I2C_PORT, s_addr, buf, sizeof(buf), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t read_regs(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_PORT, s_addr, &reg, 1, data, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static bool install_driver(void) {
    if (s_driver_installed) return true;

    i2c_config_t conf = {};
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = MPU6050_SDA_PIN;
    conf.scl_io_num       = MPU6050_SCL_PIN;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = MPU6050_I2C_FREQ_HZ;

    esp_err_t err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return false;
    }
    err = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return false;
    }
    s_driver_installed = true;
    return true;
}

static bool probe_address(uint8_t addr) {
    s_addr = addr;
    uint8_t who = 0;
    if (read_regs(REG_WHO_AM_I, &who, 1) != ESP_OK) return false;
    bool known = (who == 0x68) || (who == 0x69) || (who >= 0x70 && who <= 0x73) || (who == 0x98);
    if (!known) {
        ESP_LOGW(TAG, "Device at 0x%02X returned unexpected WHO_AM_I 0x%02X", addr, who);
        return false;
    }
    ESP_LOGI(TAG, "MPU-6050 found at 0x%02X (WHO_AM_I=0x%02X)", addr, who);
    return true;
}

bool mpu6050_init(void) {
    s_present = false;
    if (!install_driver()) return false;

    if (!probe_address(MPU6050_ADDR_LOW) && !probe_address(MPU6050_ADDR_HIGH)) {
        return false;
    }

    if (write_reg(REG_PWR_MGMT_1, 0x80) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(100));
    if (write_reg(REG_PWR_MGMT_1, 0x01) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(50));

    bool ok = true;
    ok &= (write_reg(REG_SMPLRT_DIV, 4) == ESP_OK);     // 200 Hz sampling
    ok &= (write_reg(REG_CONFIG, 0x03) == ESP_OK);      // DLPF ~44 Hz
    ok &= (write_reg(REG_GYRO_CONFIG, 0x08) == ESP_OK); // +/-500 deg/s
    ok &= (write_reg(REG_ACCEL_CONFIG, 0x00) == ESP_OK);// +/-2 g
    if (!ok) {
        ESP_LOGE(TAG, "Failed to configure MPU-6050 registers");
        return false;
    }

    s_present = true;
    return true;
}

bool mpu6050_is_present(void) {
    return s_present;
}

uint8_t mpu6050_address(void) {
    return s_addr;
}

bool mpu6050_read_raw(mpu6050_raw_t *out) {
    if (!s_present || out == NULL) return false;

    uint8_t d[14];
    if (read_regs(REG_ACCEL_XOUT_H, d, sizeof(d)) != ESP_OK) return false;

    out->ax   = (int16_t)((d[0]  << 8) | d[1]);
    out->ay   = (int16_t)((d[2]  << 8) | d[3]);
    out->az   = (int16_t)((d[4]  << 8) | d[5]);
    out->temp = (int16_t)((d[6]  << 8) | d[7]);
    out->gx   = (int16_t)((d[8]  << 8) | d[9]);
    out->gy   = (int16_t)((d[10] << 8) | d[11]);
    out->gz   = (int16_t)((d[12] << 8) | d[13]);
    return true;
}