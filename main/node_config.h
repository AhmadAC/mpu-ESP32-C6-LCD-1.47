#pragma once

#include "sdkconfig.h"

#if defined(CONFIG_NODE_ROLE_SENDER)
#define NODE_IS_SENDER    1
#define NODE_IS_RECEIVER  0
#define NODE_ROLE_NAME    "MPU-Sender"
#define BLE_DEVICE_NAME   "ESP32C6-MPU-Sender"
#elif defined(CONFIG_NODE_ROLE_RECEIVER)
#define NODE_IS_SENDER    0
#define NODE_IS_RECEIVER  1
#define NODE_ROLE_NAME    "Arm-Receiver"
#define BLE_DEVICE_NAME   "ESP32C6-Arm-Receiver"
#else
#define NODE_IS_SENDER    0
#define NODE_IS_RECEIVER  1
#define NODE_ROLE_NAME    "Arm-Receiver"
#define BLE_DEVICE_NAME   "ESP32C6-Arm-Receiver"
#endif