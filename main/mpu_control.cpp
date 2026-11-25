#include "mpu_control.h"
#include "node_config.h"
#include "imu.h"
#include "servo_controller.h"
#include "espnow_sync.h"
#include "esp_log.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "MPU_CTRL";

#define CTRL_TICK_MS     25
#define DEADBAND_DEG     1.0f
#define SMOOTH_ALPHA     0.35f
#define GAIN_MIN         0.5f
#define GAIN_MAX         4.0f
#define GAIN_DEFAULT     2.0f

// Pose index 0=roll, 1=pitch, 2=yaw -> servo index (0=S1 claw, 1=S2 arm, 2=S3 shoulder)
static const int AXIS_SERVO[3] = { 1, 2, 0 };

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_enabled = false;
static float s_gain = GAIN_DEFAULT;
static bool  s_axes[3] = { true, true, false };
static bool  s_inv[3]  = { false, false, false };

void mpu_control_save_settings(void) {
    int32_t gain_x100;
    int32_t axes_bits = 0;
    int32_t inv_bits = 0;
    portENTER_CRITICAL(&s_lock);
    gain_x100 = (int32_t)lroundf(s_gain * 100.0f);
    for (int i = 0; i < 3; i++) {
        if (s_axes[i]) axes_bits |= (1 << i);
        if (s_inv[i]) inv_bits |= (1 << i);
    }
    portEXIT_CRITICAL(&s_lock);

    nvs_handle_t h;
    if (nvs_open("mpu_cfg", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "ctl_gain", gain_x100);
        nvs_set_i32(h, "ctl_axes", axes_bits);
        nvs_set_i32(h, "ctl_inv", inv_bits);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void load_settings(void) {
    nvs_handle_t h;
    if (nvs_open("mpu_cfg", NVS_READONLY, &h) != ESP_OK) return;
    int32_t v = 0;
    if (nvs_get_i32(h, "ctl_gain", &v) == ESP_OK) {
        float g = (float)v / 100.0f;
        if (g < GAIN_MIN) g = GAIN_MIN;
        if (g > GAIN_MAX) g = GAIN_MAX;
        s_gain = g;
    }
    if (nvs_get_i32(h, "ctl_axes", &v) == ESP_OK) {
        for (int i = 0; i < 3; i++) s_axes[i] = ((v >> i) & 1) != 0;
    }
    if (nvs_get_i32(h, "ctl_inv", &v) == ESP_OK) {
        for (int i = 0; i < 3; i++) s_inv[i] = ((v >> i) & 1) != 0;
    }
    nvs_close(h);
}

static void control_task(void *pvParameters) {
    (void)pvParameters;

    bool was_enabled = false;
    float filt[3] = { 90.0f, 90.0f, 90.0f };
    int last_sent[3] = { -1000, -1000, -1000 };

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_TICK_MS));

        if (!s_enabled) {
            was_enabled = false;
            continue;
        }

        if (!was_enabled) {
            for (int i = 0; i < 3; i++) {
                filt[i] = 90.0f;
                last_sent[i] = -1000;
            }
            was_enabled = true;
        }

        imu_pose_t pose;
        if (!imu_get(&pose)) continue;

        float rel[3] = { pose.roll, pose.pitch, pose.yaw };
        float gain;
        bool axes[3];
        bool inv[3];
        portENTER_CRITICAL(&s_lock);
        gain = s_gain;
        for (int i = 0; i < 3; i++) {
            axes[i] = s_axes[i];
            inv[i] = s_inv[i];
        }
        portEXIT_CRITICAL(&s_lock);

        int target_angles[3] = { 90, 90, 90 };

        for (int i = 0; i < 3; i++) {
            if (!axes[i]) continue;

            float v = rel[i];
            if (v > DEADBAND_DEG) v -= DEADBAND_DEG;
            else if (v < -DEADBAND_DEG) v += DEADBAND_DEG;
            else v = 0.0f;

            float target = 90.0f + (inv[i] ? -1.0f : 1.0f) * gain * v;
            if (target < (float)SERVO_MIN_ANGLE) target = (float)SERVO_MIN_ANGLE;
            if (target > (float)SERVO_MAX_ANGLE) target = (float)SERVO_MAX_ANGLE;

            filt[i] += SMOOTH_ALPHA * (target - filt[i]);
            int angle = (int)lroundf(filt[i]);
            target_angles[AXIS_SERVO[i]] = angle;
        }

        bool changed = (target_angles[0] != last_sent[0] ||
                        target_angles[1] != last_sent[1] ||
                        target_angles[2] != last_sent[2]);

        if (changed) {
            last_sent[0] = target_angles[0];
            last_sent[1] = target_angles[1];
            last_sent[2] = target_angles[2];

#if NODE_IS_SENDER
            int s1, s2, s3, s4, o1, o2, o3, o4, smin, smax;
            bool e1, e2, e3, e4, idle, run, loop;
            int slot;
            uint8_t current_s4 = 90;
            if (espnow_sync_get_cached_arm_status(&s1, &s2, &s3, &s4, &o1, &o2, &o3, &o4,
                                                 &smin, &smax, &e1, &e2, &e3, &e4,
                                                 &idle, &run, &loop, &slot)) {
                current_s4 = (uint8_t)s4;
            }
            espnow_sync_send_servos(target_angles[0], target_angles[1], target_angles[2], current_s4, 0, true);
#else
            servo_set_angle(0, target_angles[0]);
            servo_set_angle(1, target_angles[1]);
            servo_set_angle(2, target_angles[2]);
#endif
        }
    }
}

void mpu_control_init(void) {
    load_settings();
    xTaskCreatePinnedToCore(control_task, "mpu_ctrl_task", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "MPU control ready (OFF): gain=%.2f, axes R=%d P=%d Y=%d",
             s_gain, s_axes[0] ? 1 : 0, s_axes[1] ? 1 : 0, s_axes[2] ? 1 : 0);
}

void mpu_control_set_enabled(bool enabled) {
    s_enabled = enabled;
    ESP_LOGI(TAG, "MPU control %s", enabled ? "ENABLED" : "DISABLED");
}

bool mpu_control_is_enabled(void) {
    return s_enabled;
}

void mpu_control_set_gain(float gain) {
    if (gain < GAIN_MIN) gain = GAIN_MIN;
    if (gain > GAIN_MAX) gain = GAIN_MAX;
    portENTER_CRITICAL(&s_lock);
    s_gain = gain;
    portEXIT_CRITICAL(&s_lock);
}

float mpu_control_get_gain(void) {
    portENTER_CRITICAL(&s_lock);
    float g = s_gain;
    portEXIT_CRITICAL(&s_lock);
    return g;
}

void mpu_control_set_axes(bool roll, bool pitch, bool yaw) {
    portENTER_CRITICAL(&s_lock);
    s_axes[0] = roll;
    s_axes[1] = pitch;
    s_axes[2] = yaw;
    portEXIT_CRITICAL(&s_lock);
    mpu_control_save_settings();
}

void mpu_control_get_axes(bool *roll, bool *pitch, bool *yaw) {
    portENTER_CRITICAL(&s_lock);
    if (roll) *roll = s_axes[0];
    if (pitch) *pitch = s_axes[1];
    if (yaw) *yaw = s_axes[2];
    portEXIT_CRITICAL(&s_lock);
}

void mpu_control_set_invert(bool roll, bool pitch, bool yaw) {
    portENTER_CRITICAL(&s_lock);
    s_inv[0] = roll;
    s_inv[1] = pitch;
    s_inv[2] = yaw;
    portEXIT_CRITICAL(&s_lock);
    mpu_control_save_settings();
}

void mpu_control_get_invert(bool *roll, bool *pitch, bool *yaw) {
    portENTER_CRITICAL(&s_lock);
    if (roll) *roll = s_inv[0];
    if (pitch) *pitch = s_inv[1];
    if (yaw) *yaw = s_inv[2];
    portEXIT_CRITICAL(&s_lock);
}

void mpu_control_report(mpu_control_print_fn_t print_fn) {
    if (print_fn == NULL) return;
    bool ar, ap, ay, ir, ip, iy;
    mpu_control_get_axes(&ar, &ap, &ay);
    mpu_control_get_invert(&ir, &ip, &iy);
    char buf[96];
    snprintf(buf, sizeof(buf), "MPUCTRL:on=%d,gain=%.2f,er=%d,ep=%d,ey=%d,ir=%d,ip=%d,iy=%d\n",
             s_enabled ? 1 : 0, mpu_control_get_gain(),
             ar ? 1 : 0, ap ? 1 : 0, ay ? 1 : 0,
             ir ? 1 : 0, ip ? 1 : 0, iy ? 1 : 0);
    print_fn(buf);
}