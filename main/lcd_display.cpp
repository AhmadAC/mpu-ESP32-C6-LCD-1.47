#include "lcd_display.h"
#include "node_config.h"
#include "servo_controller.h"
#include "imu.h"
#include "mpu_control.h"
#include "espnow_sync.h"
#include "ble_server.h"

#if HAS_ONBOARD_LCD

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "LCD_147";

static spi_device_handle_t s_spi = NULL;

#define COLOR_BLACK       0x0000
#define COLOR_NAVY        0x0810
#define COLOR_DARKCYAN    0x03E0
#define COLOR_WHITE       0xFFFF
#define COLOR_CYAN        0x07FF
#define COLOR_GREEN       0x07E0
#define COLOR_AMBER       0xFD20
#define COLOR_RED         0xF800
#define COLOR_GRAY        0x7BEF
#define COLOR_PURPLE      0x915C

// 8x8 Basic ASCII Font Bitmap (characters 32 to 127)
static const uint8_t FONT8x8[][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 32 ' '
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // 33 '!'
    {0x66,0x66,0x24,0x00,0x00,0x00,0x00,0x00}, // 34 '"'
    {0x6C,0x6C,0xFE,0x6C,0xFE,0x6C,0x6C,0x00}, // 35 '#'
    {0x18,0x3E,0x60,0x3C,0x06,0x7C,0x18,0x00}, // 36 '$'
    {0x00,0x63,0x66,0x0C,0x18,0x33,0x63,0x00}, // 37 '%'
    {0x38,0x6C,0x38,0x76,0xDC,0xCC,0x76,0x00}, // 38 '&'
    {0x30,0x30,0x10,0x20,0x00,0x00,0x00,0x00}, // 39 '''
    {0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00}, // 40 '('
    {0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00}, // 41 ')'
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // 42 '*'
    {0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00}, // 43 '+'
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30}, // 44 ','
    {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, // 45 '-'
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, // 46 '.'
    {0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00}, // 47 '/'
    {0x3C,0x66,0x6E,0x76,0x66,0x66,0x3C,0x00}, // 48 '0'
    {0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00}, // 49 '1'
    {0x3C,0x66,0x06,0x1C,0x30,0x66,0x7E,0x00}, // 50 '2'
    {0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00}, // 51 '3'
    {0x0C,0x1C,0x34,0x6C,0x7E,0x0C,0x0C,0x00}, // 52 '4'
    {0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0x00}, // 53 '5'
    {0x1C,0x30,0x60,0x7C,0x66,0x66,0x3C,0x00}, // 54 '6'
    {0x7E,0x66,0x0C,0x18,0x18,0x18,0x18,0x00}, // 55 '7'
    {0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00}, // 56 '8'
    {0x3C,0x66,0x66,0x3E,0x06,0x0C,0x38,0x00}, // 57 '9'
    {0x00,0x18,0x18,0x00,0x18,0x18,0x00,0x00}, // 58 ':'
    {0x00,0x18,0x18,0x00,0x18,0x18,0x30,0x00}, // 59 ';'
    {0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x00}, // 60 '<'
    {0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00}, // 61 '='
    {0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x00}, // 62 '>'
    {0x3C,0x66,0x0C,0x18,0x18,0x00,0x18,0x00}, // 63 '?'
    {0x3C,0x66,0x6E,0x6A,0x6E,0x60,0x3C,0x00}, // 64 '@'
    {0x18,0x3C,0x66,0x66,0x7E,0x66,0x66,0x00}, // 65 'A'
    {0x7C,0x66,0x66,0x7C,0x66,0x66,0x7C,0x00}, // 66 'B'
    {0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00}, // 67 'C'
    {0x78,0x6C,0x66,0x66,0x66,0x6C,0x78,0x00}, // 68 'D'
    {0x7E,0x60,0x60,0x7C,0x60,0x60,0x7E,0x00}, // 69 'E'
    {0x7E,0x60,0x60,0x7C,0x60,0x60,0x60,0x00}, // 70 'F'
    {0x3C,0x66,0x60,0x6E,0x66,0x66,0x3E,0x00}, // 71 'G'
    {0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x00}, // 72 'H'
    {0x3C,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, // 73 'I'
    {0x06,0x06,0x06,0x06,0x06,0x66,0x3C,0x00}, // 74 'J'
    {0x66,0x6C,0x78,0x70,0x78,0x6C,0x66,0x00}, // 75 'K'
    {0x60,0x60,0x60,0x60,0x60,0x60,0x7E,0x00}, // 76 'L'
    {0x63,0x77,0x7F,0x6B,0x63,0x63,0x63,0x00}, // 77 'M'
    {0x66,0x76,0x7E,0x7E,0x6E,0x66,0x66,0x00}, // 78 'N'
    {0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, // 79 'O'
    {0x7C,0x66,0x66,0x7C,0x60,0x60,0x60,0x00}, // 80 'P'
    {0x3C,0x66,0x66,0x66,0x6A,0x6C,0x36,0x00}, // 81 'Q'
    {0x7C,0x66,0x66,0x7C,0x6C,0x66,0x66,0x00}, // 82 'R'
    {0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0x00}, // 83 'S'
    {0x7E,0x5A,0x18,0x18,0x18,0x18,0x18,0x00}, // 84 'T'
    {0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, // 85 'U'
    {0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00}, // 86 'V'
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, // 87 'W'
    {0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00}, // 88 'X'
    {0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x00}, // 89 'Y'
    {0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00}, // 90 'Z'
    {0x3C,0x30,0x30,0x30,0x30,0x30,0x3C,0x00}, // 91 '['
    {0xC0,0x60,0x30,0x18,0x0C,0x06,0x02,0x00}, // 92 '\'
    {0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00}, // 93 ']'
    {0x10,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00}, // 94 '^'
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, // 95 '_'
    {0x30,0x18,0x0C,0x00,0x00,0x00,0x00,0x00}, // 96 '`'
    {0x00,0x00,0x3C,0x06,0x3E,0x66,0x3E,0x00}, // 97 'a'
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x7C,0x00}, // 98 'b'
    {0x00,0x00,0x3C,0x66,0x60,0x66,0x3C,0x00}, // 99 'c'
    {0x06,0x06,0x3E,0x66,0x66,0x66,0x3E,0x00}, // 100 'd'
    {0x00,0x00,0x3C,0x66,0x7E,0x60,0x3C,0x00}, // 101 'e'
    {0x1C,0x30,0x30,0x78,0x30,0x30,0x30,0x00}, // 102 'f'
    {0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x7C}, // 103 'g'
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x00}, // 104 'h'
    {0x18,0x00,0x38,0x18,0x18,0x18,0x3C,0x00}, // 105 'i'
    {0x06,0x00,0x0E,0x06,0x06,0x66,0x3C,0x00}, // 106 'j'
    {0x60,0x60,0x66,0x6C,0x78,0x6C,0x66,0x00}, // 107 'k'
    {0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, // 108 'l'
    {0x00,0x00,0x66,0x7F,0x7F,0x6B,0x63,0x00}, // 109 'm'
    {0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x00}, // 110 'n'
    {0x00,0x00,0x3C,0x66,0x66,0x66,0x3C,0x00}, // 111 'o'
    {0x00,0x00,0x7C,0x66,0x66,0x7C,0x60,0x60}, // 112 'p'
    {0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x07}, // 113 'q'
    {0x00,0x00,0x7C,0x66,0x60,0x60,0x60,0x00}, // 114 'r'
    {0x00,0x00,0x3E,0x60,0x3C,0x06,0x7C,0x00}, // 115 's'
    {0x18,0x18,0x7E,0x18,0x18,0x18,0x0E,0x00}, // 116 't'
    {0x00,0x00,0x66,0x66,0x66,0x66,0x3E,0x00}, // 117 'u'
    {0x00,0x00,0x66,0x66,0x66,0x3C,0x18,0x00}, // 118 'v'
    {0x00,0x00,0x63,0x6B,0x7F,0x3E,0x36,0x00}, // 119 'w'
    {0x00,0x00,0x66,0x3C,0x18,0x3C,0x66,0x00}, // 120 'x'
    {0x00,0x00,0x66,0x66,0x66,0x3E,0x06,0x7C}, // 121 'y'
    {0x00,0x00,0x7E,0x0C,0x18,0x30,0x7E,0x00}, // 122 'z'
    {0x0E,0x18,0x18,0x70,0x18,0x18,0x0E,0x00}, // 123 '{'
    {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00}, // 124 '|'
    {0x70,0x18,0x18,0x0E,0x18,0x18,0x70,0x00}, // 125 '}'
    {0x76,0xDC,0x00,0x00,0x00,0x00,0x00,0x00}  // 126 '~'
};

// Line buffer for fast SPI transfers
static uint16_t s_line_buf[LCD_WIDTH];

static void lcd_cmd(uint8_t cmd) {
    gpio_set_level(LCD_PIN_DC, 0);
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &cmd;
    spi_device_polling_transmit(s_spi, &t);
}

static void lcd_data(const uint8_t *data, size_t len) {
    if (len == 0) return;
    gpio_set_level(LCD_PIN_DC, 1);
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = data;
    spi_device_polling_transmit(s_spi, &t);
}

static void lcd_data_byte(uint8_t d) {
    lcd_data(&d, 1);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    // 172x320 ST7789 display controller horizontal offset is 34 px
    uint16_t col_offset = 34;
    uint16_t row_offset = 0;

    uint8_t col_data[4] = {
        (uint8_t)((x0 + col_offset) >> 8), (uint8_t)(x0 + col_offset),
        (uint8_t)((x1 + col_offset) >> 8), (uint8_t)(x1 + col_offset)
    };
    uint8_t row_data[4] = {
        (uint8_t)((y0 + row_offset) >> 8), (uint8_t)(y0 + row_offset),
        (uint8_t)((y1 + row_offset) >> 8), (uint8_t)(y1 + row_offset)
    };

    lcd_cmd(0x2A); // CASET
    lcd_data(col_data, 4);

    lcd_cmd(0x2B); // RASET
    lcd_data(row_data, 4);

    lcd_cmd(0x2C); // RAMWR
}

static void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT || w == 0 || h == 0) return;
    if (x + w > LCD_WIDTH) w = LCD_WIDTH - x;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;

    lcd_set_window(x, y, x + w - 1, y + h - 1);

    uint16_t be_color = (uint16_t)((color << 8) | (color >> 8));
    for (int i = 0; i < w; i++) s_line_buf[i] = be_color;

    gpio_set_level(LCD_PIN_DC, 1);
    for (int row = 0; row < h; row++) {
        spi_transaction_t t = {};
        t.length = w * 16;
        t.tx_buffer = s_line_buf;
        spi_device_polling_transmit(s_spi, &t);
    }
}

static void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale) {
    if (c < 32 || c > 126) c = '?';
    const uint8_t *glyph = FONT8x8[c - 32];

    for (uint8_t row = 0; row < 8; row++) {
        uint8_t bits = glyph[row];
        for (uint8_t col = 0; col < 8; col++) {
            bool on = (bits & (1 << (7 - col))) != 0;
            uint16_t col_val = on ? fg : bg;
            if (scale == 1) {
                if (on || bg != fg) {
                    lcd_fill_rect(x + col, y + row, 1, 1, col_val);
                }
            } else {
                if (on || bg != fg) {
                    lcd_fill_rect(x + col * scale, y + row * scale, scale, scale, col_val);
                }
            }
        }
    }
}

static void lcd_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg, uint8_t scale) {
    if (!str) return;
    uint16_t cx = x;
    while (*str) {
        if (*str == '\n') {
            y += 8 * scale + 2;
            cx = x;
        } else {
            if (cx + 8 * scale <= LCD_WIDTH) {
                lcd_draw_char(cx, y, *str, fg, bg, scale);
            }
            cx += 8 * scale;
        }
        str++;
    }
}

void lcd_display_update(void) {
    if (!s_spi) return;

    // 1. Header Banner
    lcd_fill_rect(0, 0, LCD_WIDTH, 26, COLOR_NAVY);
    lcd_draw_string(8, 7, NODE_IS_SENDER ? "MPU-SENDER" : "ARM-RECEIVER", COLOR_CYAN, COLOR_NAVY, 2);

    // 2. Wireless Connectivity Status Section
    bool paired = espnow_sync_is_paired();
    int8_t rssi = espnow_sync_get_last_rssi();
    bool ble_conn = ble_server_is_connected();

    lcd_fill_rect(0, 28, LCD_WIDTH, 48, COLOR_BLACK);
    lcd_draw_string(6, 32, "ESP-NOW MESH:", COLOR_WHITE, COLOR_BLACK, 1);
    if (paired) {
        char link_str[32];
        snprintf(link_str, sizeof(link_str), "PAIRED (%ddBm)", (int)rssi);
        lcd_draw_string(82, 32, link_str, COLOR_GREEN, COLOR_BLACK, 1);

        uint8_t pm[6] = {0};
        espnow_sync_get_peer_mac(pm);
        char mac_str[24];
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X", pm[0], pm[1], pm[2], pm[3], pm[4], pm[5]);
        lcd_draw_string(6, 44, mac_str, COLOR_AMBER, COLOR_BLACK, 1);
    } else {
        lcd_draw_string(82, 32, "DISCOVERING", COLOR_AMBER, COLOR_BLACK, 1);
        lcd_draw_string(6, 44, "Beaconing Ch 1...", COLOR_GRAY, COLOR_BLACK, 1);
    }

    lcd_draw_string(6, 58, "BLE APP LINK:", COLOR_WHITE, COLOR_BLACK, 1);
    lcd_draw_string(84, 58, ble_conn ? "CONNECTED" : "ADVERTISING", ble_conn ? COLOR_GREEN : COLOR_GRAY, COLOR_BLACK, 1);

    // Separator rule
    lcd_fill_rect(6, 78, LCD_WIDTH - 12, 1, COLOR_DARKCYAN);

    // 3. Telemetry Block
#if NODE_IS_RECEIVER
    int s1 = 90, s2 = 90, s3 = 90, s4 = 90;
    servo_get_all_angles(&s1, &s2, &s3, &s4);
    int pct = servo_get_scissor_percent();
    bool auto_idle = servo_get_auto_idle();
    bool seq_run = servo_is_sequence_running();

    lcd_fill_rect(0, 84, LCD_WIDTH, 170, COLOR_BLACK);

    lcd_draw_string(6, 86, "SERVO JOINTS:", COLOR_CYAN, COLOR_BLACK, 1);

    char line1[32], line2[32], line3[32], line4[32];
    snprintf(line1, sizeof(line1), "S1 Claw:     %3d deg", s1);
    snprintf(line2, sizeof(line2), "S2 Arm:      %3d deg", s2);
    snprintf(line3, sizeof(line3), "S3 Shoulder: %3d deg", s3);
    snprintf(line4, sizeof(line4), "S4 Scissor:  %3d%% (%d)", pct, s4);

    lcd_draw_string(6, 102, line1, COLOR_WHITE, COLOR_BLACK, 1);
    lcd_draw_string(6, 120, line2, COLOR_WHITE, COLOR_BLACK, 1);
    lcd_draw_string(6, 138, line3, COLOR_WHITE, COLOR_BLACK, 1);
    lcd_draw_string(6, 156, line4, COLOR_AMBER, COLOR_BLACK, 1);

    lcd_draw_string(6, 180, "MOTION STATUS:", COLOR_CYAN, COLOR_BLACK, 1);
    lcd_draw_string(6, 196, seq_run ? "RUNNING ACTION..." : "MANUAL IDLE", seq_run ? COLOR_GREEN : COLOR_GRAY, COLOR_BLACK, 1);

    char idle_str[32];
    snprintf(idle_str, sizeof(idle_str), "Auto-Idle:   %s", auto_idle ? "ENABLED" : "HOLD TORQUE");
    lcd_draw_string(6, 212, idle_str, auto_idle ? COLOR_GREEN : COLOR_AMBER, COLOR_BLACK, 1);

#elif NODE_IS_SENDER
    imu_pose_t pose;
    bool has_imu = imu_get(&pose);
    bool mpu_ctrl = mpu_control_is_enabled();
    float gain = mpu_control_get_gain();

    lcd_fill_rect(0, 84, LCD_WIDTH, 170, COLOR_BLACK);

    lcd_draw_string(6, 86, "IMU ATTITUDE:", COLOR_CYAN, COLOR_BLACK, 1);

    if (has_imu) {
        char r_str[32], p_str[32], y_str[32], t_str[32];
        snprintf(r_str, sizeof(r_str), "Roll  (X): %+.1f deg", pose.roll);
        snprintf(p_str, sizeof(p_str), "Pitch (Y): %+.1f deg", pose.pitch);
        snprintf(y_str, sizeof(y_str), "Yaw   (Z): %+.1f deg", pose.yaw);
        snprintf(t_str, sizeof(t_str), "Temp:      %.1f C", pose.temp);

        lcd_draw_string(6, 102, r_str, COLOR_WHITE, COLOR_BLACK, 1);
        lcd_draw_string(6, 120, p_str, COLOR_WHITE, COLOR_BLACK, 1);
        lcd_draw_string(6, 138, y_str, COLOR_WHITE, COLOR_BLACK, 1);
        lcd_draw_string(6, 156, t_str, COLOR_GRAY, COLOR_BLACK, 1);
    } else {
        lcd_draw_string(6, 102, "Scanning MPU-6050...", COLOR_AMBER, COLOR_BLACK, 1);
        lcd_draw_string(6, 118, "I2C SDA:18 SCL:19", COLOR_GRAY, COLOR_BLACK, 1);
    }

    lcd_draw_string(6, 180, "MOTION CONTROL:", COLOR_CYAN, COLOR_BLACK, 1);
    lcd_draw_string(6, 196, mpu_ctrl ? "TRACKING ACTIVE" : "TRACKING DISABLED", mpu_ctrl ? COLOR_GREEN : COLOR_GRAY, COLOR_BLACK, 1);

    char g_str[32];
    snprintf(g_str, sizeof(g_str), "Sensitivity: %.2fx", gain);
    lcd_draw_string(6, 212, g_str, COLOR_AMBER, COLOR_BLACK, 1);
#endif

    // Separator rule
    lcd_fill_rect(6, 258, LCD_WIDTH - 12, 1, COLOR_DARKCYAN);

    // 4. Hardware Footer Bar
    lcd_fill_rect(0, 264, LCD_WIDTH, 56, COLOR_BLACK);
#if NODE_IS_RECEIVER
    lcd_draw_string(6, 268, "HARDWARE ASSIGNMENT", COLOR_GRAY, COLOR_BLACK, 1);
    lcd_draw_string(6, 282, "S1:GP0 S2:GP1", COLOR_WHITE, COLOR_BLACK, 1);
    lcd_draw_string(6, 296, "S3:GP2 S4:GP3", COLOR_AMBER, COLOR_BLACK, 1);
#else
    lcd_draw_string(6, 268, "HARDWARE ASSIGNMENT", COLOR_GRAY, COLOR_BLACK, 1);
    lcd_draw_string(6, 282, "SDA:GP18 SCL:GP19", COLOR_WHITE, COLOR_BLACK, 1);
    lcd_draw_string(6, 296, "I2C0 400kHz MPU-6050", COLOR_AMBER, COLOR_BLACK, 1);
#endif
}

static void lcd_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(150));
        lcd_display_update();
    }
}

void lcd_display_init(void) {
    gpio_config_t dc_rst_cfg = {};
    dc_rst_cfg.mode = GPIO_MODE_OUTPUT;
    dc_rst_cfg.pin_bit_mask = (1ULL << LCD_PIN_DC) | (1ULL << LCD_PIN_RST) | (1ULL << LCD_PIN_CS);
    gpio_config(&dc_rst_cfg);

    gpio_set_level(LCD_PIN_CS, 1);
    gpio_set_level(LCD_PIN_DC, 1);

    // Hardware Reset
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    // Initialize SPI bus for ST7789 display controller
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = LCD_PIN_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = LCD_PIN_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = LCD_WIDTH * 16 * sizeof(uint16_t);

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 40 * 1000 * 1000; // 40 MHz SPI
    devcfg.mode = 0;
    devcfg.spics_io_num = LCD_PIN_CS;
    devcfg.queue_size = 6;

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi));

    // ST7789 Initialization Sequence
    lcd_cmd(0x01); // Software Reset
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_cmd(0x11); // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_cmd(0x3A); // Interface Pixel Format: 16-bit RGB565
    lcd_data_byte(0x55);

    lcd_cmd(0x36); // Memory Data Access Control (Orientation)
    lcd_data_byte(0x00);

    lcd_cmd(0x21); // Display Inversion On
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_cmd(0x13); // Normal Display Mode On
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_cmd(0x29); // Display On
    vTaskDelay(pdMS_TO_TICKS(20));

    // Clear display to solid black
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, COLOR_BLACK);

    // Configure Backlight via LEDC PWM (Safe 45% brightness per Waveshare specifications)
    ledc_timer_config_t bl_timer = {};
    bl_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    bl_timer.timer_num = LEDC_TIMER_1;
    bl_timer.duty_resolution = LEDC_TIMER_10_BIT;
    bl_timer.freq_hz = 1000;
    bl_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&bl_timer);

    ledc_channel_config_t bl_ch = {};
    bl_ch.channel = LEDC_CHANNEL_4;
    bl_ch.duty = 460; // ~45% of 1023
    bl_ch.gpio_num = LCD_PIN_BL;
    bl_ch.speed_mode = LEDC_LOW_SPEED_MODE;
    bl_ch.timer_sel = LEDC_TIMER_1;
    ledc_channel_config(&bl_ch);

    xTaskCreatePinnedToCore(lcd_task, "lcd_hud_task", 4096, NULL, 3, NULL, 0);

    ESP_LOGI(TAG, "Waveshare 1.47-inch ST7789 LCD Initialized (MOSI:%d SCLK:%d CS:%d DC:%d BL:%d)",
             LCD_PIN_MOSI, LCD_PIN_SCLK, LCD_PIN_CS, LCD_PIN_DC, LCD_PIN_BL);
}

#else

// Stubs for hardware boards without an onboard ST7789 LCD (such as YD-ESP32-S3)
void lcd_display_init(void) {
}

void lcd_display_update(void) {
}

#endif // HAS_ONBOARD_LCD