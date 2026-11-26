// main\node_config.h
#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    #define TARGET_IS_ESP32S3        1
    #define TARGET_IS_ESP32C6        0
    #define BOARD_NAME               "YD-ESP32-S3 v1.3"
    #define HAS_ONBOARD_LCD          0
    // Safe unshared I2C pins for YD-ESP32-S3 (GPIO 18-20 are reserved for USB JTAG)
    #define BOARD_MPU_SDA_PIN        GPIO_NUM_4
    #define BOARD_MPU_SCL_PIN        GPIO_NUM_5
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
    #define TARGET_IS_ESP32S3        0
    #define TARGET_IS_ESP32C6        1
    #define BOARD_NAME               "Waveshare ESP32-C6-LCD-1.47"
    #define HAS_ONBOARD_LCD          1
    // Safe unshared I2C pins for Waveshare ESP32-C6
    #define BOARD_MPU_SDA_PIN        GPIO_NUM_18
    #define BOARD_MPU_SCL_PIN        GPIO_NUM_19
#else
    #define TARGET_IS_ESP32S3        0
    #define TARGET_IS_ESP32C6        0
    #define BOARD_NAME               "Generic ESP32"
    #define HAS_ONBOARD_LCD          0
    #define BOARD_MPU_SDA_PIN        GPIO_NUM_4
    #define BOARD_MPU_SCL_PIN        GPIO_NUM_5
#endif

#if defined(CONFIG_NODE_ROLE_SENDER)
    #define NODE_IS_SENDER           1
    #define NODE_IS_RECEIVER         0
    #define NODE_ROLE_NAME           "MPU-Sender"
    #if TARGET_IS_ESP32S3
        #define BLE_DEVICE_NAME      "ESP32S3-MPU-Sender"
    #else
        #define BLE_DEVICE_NAME      "ESP32C6-MPU-Sender"
    #endif
#elif defined(CONFIG_NODE_ROLE_RECEIVER)
    #define NODE_IS_SENDER           0
    #define NODE_IS_RECEIVER         1
    #define NODE_ROLE_NAME           "Arm-Receiver"
    #define BLE_DEVICE_NAME          "ESP32C6-Arm-Receiver"
#else
    #define NODE_IS_SENDER           0
    #define NODE_IS_RECEIVER         1
    #define NODE_ROLE_NAME           "Arm-Receiver"
    #define BLE_DEVICE_NAME          "ESP32C6-Arm-Receiver"
#endif