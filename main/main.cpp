#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#include "node_config.h"
#include "servo_controller.h"
#include "ble_server.h"
#include "console_serial.h"
#include "command_processor.h"
#include "imu.h"
#include "mpu_control.h"
#include "espnow_sync.h"
#include "lcd_display.h"

static const char *TAG = "APP_MAIN";

#if NODE_IS_RECEIVER
static void on_espnow_command_reply(const char *reply) {
    if (reply && strlen(reply) > 0) {
        espnow_sync_send_reply(reply);
        ble_server_send_notify(reply);
    }
}

static void on_espnow_command(const char *cmd_str) {
    command_process(cmd_str, on_espnow_command_reply);
}
#endif

extern "C" void app_main(void) {
    esp_log_level_set("NimBLE", ESP_LOG_WARN);
    esp_log_level_set("wifi", ESP_LOG_WARN);

#if NODE_IS_RECEIVER
    const gpio_num_t default_servo_pins[SERVO_COUNT] = {
        SERVO_1_PIN_DEFAULT,
        SERVO_2_PIN_DEFAULT,
        SERVO_3_PIN_DEFAULT,
        SERVO_4_PIN_DEFAULT
    };
    for (int i = 0; i < SERVO_COUNT; i++) {
        gpio_reset_pin(default_servo_pins[i]);
        gpio_set_direction(default_servo_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(default_servo_pins[i], 0);
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(200));

    esp_reset_reason_t rst = esp_reset_reason();
    if (rst == ESP_RST_BROWNOUT) {
        ESP_LOGW(TAG, "WARNING: Previous reset was caused by BROWNOUT (power rail sag)!");
    }

    ESP_LOGI(TAG, "Booting Waveshare ESP32-C6-LCD-1.47 [%s] (Reset Reason: %d)...", NODE_ROLE_NAME, (int)rst);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    lcd_display_init();

    command_processor_init();

    ble_server_init(command_on_ble_received);

#if NODE_IS_RECEIVER
    servo_controller_init();
    espnow_sync_init(ESPNOW_ROLE_RECEIVER);
    espnow_sync_register_cmd_handler(on_espnow_command);
#endif

#if NODE_IS_SENDER
    imu_init();
    mpu_control_init();
    espnow_sync_init(ESPNOW_ROLE_SENDER);
#endif

    console_serial_init();

    ESP_LOGI(TAG, "Waveshare ESP32-C6 [%s] systems online. Zero-intervention ESP-NOW mesh active.", NODE_ROLE_NAME);
}