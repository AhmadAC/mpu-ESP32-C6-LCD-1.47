#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "node_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if HAS_ONBOARD_LCD
// Waveshare ESP32-C6-LCD-1.47 Hardware Display Mapping:
// 1.47" ST7789 IPS LCD (172 x 320 resolution)
#define LCD_PIN_MOSI      GPIO_NUM_6
#define LCD_PIN_SCLK      GPIO_NUM_7
#define LCD_PIN_CS        GPIO_NUM_14
#define LCD_PIN_DC        GPIO_NUM_15
#define LCD_PIN_RST       GPIO_NUM_21
#define LCD_PIN_BL        GPIO_NUM_22

#define LCD_WIDTH         172
#define LCD_HEIGHT        320
#endif

void lcd_display_init(void);
void lcd_display_update(void);

#ifdef __cplusplus
}
#endif