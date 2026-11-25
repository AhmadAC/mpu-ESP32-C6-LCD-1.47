#include "imu.h"
#include "mpu6050.h"
#include "ble_server.h"
#include "espnow_sync.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "IMU";

#define LOOP_MS               5        // 200 Hz sampling & fusion
#define TELEMETRY_DIV         10       // 20 Hz telemetry
#define CAL_SAMPLES           400      // 2 s sampling window
#define STALE_US              150000
#define RAD2DEG               57.29577951f
#define DEG2RAD               0.01745329252f

#define STILL_GYRO_DPS        0.65f
#define STILL_ACCEL_TOL_G     0.07f
#define STILL_CONFIRM_TICKS   40

#define CAL_GYRO_RANGE_LSB    600
#define CAL_ACCEL_RANGE_LSB   1200
#define MAX_READ_FAILS        50

#define CAL_MAGIC             0x4D505546u

typedef struct {
    uint32_t magic;
    float    gyro_bias[3];      // deg/s
    float    accel_bias[3];     // g
    float    neutral_angles[3]; // [roll, pitch, yaw] neutral reference
    uint8_t  gyro_ok;
    uint8_t  accel_ok;
    uint8_t  neutral_ok;
    uint8_t  pad;
} imu_cal_t;

enum {
    REQ_NONE = 0,
    REQ_GYRO,
    REQ_ACCEL,
    REQ_NEUTRAL,
    REQ_YAW,
    REQ_CLEAR
};

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static imu_pose_t   s_pose;
static int64_t      s_pose_us   = 0;
static imu_cal_t    s_cal_shared;
static volatile bool s_sensor_ok  = false;
static volatile bool s_stream     = false;
static volatile int  s_cal_req    = REQ_NONE;
static volatile bool s_cal_busy   = false;
static volatile bool s_is_still   = true;

static float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static void cal_defaults(imu_cal_t *c) {
    memset(c, 0, sizeof(*c));
    c->magic = CAL_MAGIC;
    c->neutral_ok = 1;
}

static void cal_load(imu_cal_t *c) {
    cal_defaults(c);
    nvs_handle_t h;
    if (nvs_open("mpu_cfg", NVS_READONLY, &h) == ESP_OK) {
        imu_cal_t tmp;
        size_t len = sizeof(tmp);
        if (nvs_get_blob(h, "cal", &tmp, &len) == ESP_OK && len == sizeof(tmp) && tmp.magic == CAL_MAGIC) {
            *c = tmp;
        }
        nvs_close(h);
    }
}

static void cal_save(const imu_cal_t *c) {
    nvs_handle_t h;
    if (nvs_open("mpu_cfg", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "cal", c, sizeof(*c));
        nvs_commit(h);
        nvs_close(h);
    }
}

static void cal_clear_nvs(void) {
    nvs_handle_t h;
    if (nvs_open("mpu_cfg", NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, "cal");
        nvs_commit(h);
        nvs_close(h);
    }
}

static void cal_share(const imu_cal_t *c) {
    portENTER_CRITICAL(&s_lock);
    s_cal_shared = *c;
    portEXIT_CRITICAL(&s_lock);
}

static void format_cal_line(char *buf, size_t len, const imu_cal_t *c) {
    snprintf(buf, len,
             "MPUCAL:gyro=%d,accel=%d,nr=%.1f,np=%.1f,ny=%.1f,gbx=%.2f,gby=%.2f,gbz=%.2f,abx=%.3f,aby=%.3f,abz=%.3f\n",
             c->gyro_ok ? 1 : 0, c->accel_ok ? 1 : 0,
             c->neutral_angles[0], c->neutral_angles[1], c->neutral_angles[2],
             c->gyro_bias[0], c->gyro_bias[1], c->gyro_bias[2],
             c->accel_bias[0], c->accel_bias[1], c->accel_bias[2]);
}

static void broadcast_notify(const char *buf) {
    ble_server_send_notify(buf);
    if (espnow_sync_is_paired()) {
        espnow_sync_send_reply(buf);
    }
}

static void notify_cal_state(const imu_cal_t *c) {
    char buf[240];
    format_cal_line(buf, sizeof(buf), c);
    broadcast_notify(buf);
}

static void notify_hw(bool ok) {
    char buf[48];
    snprintf(buf, sizeof(buf), "MPUHW:ok=%d,addr=%d\n", ok ? 1 : 0, (int)mpu6050_address());
    broadcast_notify(buf);
}

static void notify_result(const char *kind, bool ok, const char *msg) {
    char buf[160];
    snprintf(buf, sizeof(buf), "MPUCALRES:kind=%s,ok=%d,msg=%s\n", kind, ok ? 1 : 0, msg);
    broadcast_notify(buf);
    ESP_LOGI(TAG, "Calibration %s: %s (%s)", kind, ok ? "OK" : "FAILED", msg);
}

static void notify_progress(const char *kind, int pct) {
    char buf[64];
    snprintf(buf, sizeof(buf), "MPUCALPROG:kind=%s,pct=%d\n", kind, pct);
    broadcast_notify(buf);
}

static void publish_pose(const imu_pose_t *p, int64_t now_us) {
    portENTER_CRITICAL(&s_lock);
    s_pose = *p;
    s_pose_us = now_us;
    portEXIT_CRITICAL(&s_lock);
}

static bool pose_snapshot(imu_pose_t *out) {
    int64_t age;
    portENTER_CRITICAL(&s_lock);
    *out = s_pose;
    age = esp_timer_get_time() - s_pose_us;
    portEXIT_CRITICAL(&s_lock);
    return s_sensor_ok && s_pose_us != 0 && age < STALE_US;
}

bool imu_format_line(char *buf, size_t len) {
    imu_pose_t p;
    if (!pose_snapshot(&p)) return false;
    snprintf(buf, len, "IMU:%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,still=%d\n",
             p.roll, p.pitch, p.yaw, p.ax, p.ay, p.az, p.gx, p.gy, p.gz, p.temp, s_is_still ? 1 : 0);
    return true;
}

static void imu_task(void *pvParameters) {
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(150));

    s_sensor_ok = mpu6050_init();
    if (s_sensor_ok) {
        ESP_LOGI(TAG, "MPU-6050 ready (SDA=GPIO%d, SCL=GPIO%d)", (int)MPU6050_SDA_PIN, (int)MPU6050_SCL_PIN);
        notify_hw(true);
    } else {
        ESP_LOGW(TAG, "MPU-6050 not detected (SDA=GPIO%d, SCL=GPIO%d). Will keep retrying.",
                 (int)MPU6050_SDA_PIN, (int)MPU6050_SCL_PIN);
    }

    imu_cal_t cal;
    cal_load(&cal);
    cal_share(&cal);

    bool initialized = false;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

    int64_t last_us = esp_timer_get_time();
    int64_t last_probe_us = last_us;
    int tele_div = 0;
    int read_fails = 0;
    int still_counter = 0;

    int collecting = REQ_NONE;
    int n = 0;
    int next_pct = 25;
    double sum[6] = { 0, 0, 0, 0, 0, 0 };
    int16_t mn[6] = { 0, 0, 0, 0, 0, 0 };
    int16_t mx[6] = { 0, 0, 0, 0, 0, 0 };

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_MS));
        int64_t now_us = esp_timer_get_time();

        if (!s_sensor_ok) {
            if (now_us - last_probe_us > 1000000) {
                last_probe_us = now_us;
                if (mpu6050_init()) {
                    s_sensor_ok = true;
                    read_fails = 0;
                    initialized = false;
                    last_us = esp_timer_get_time();
                    ESP_LOGI(TAG, "MPU-6050 online");
                    notify_hw(true);
                }
                last_wake = xTaskGetTickCount();
            }
            continue;
        }

        int req = s_cal_req;
        if (req != REQ_NONE && collecting == REQ_NONE) {
            s_cal_req = REQ_NONE;
            if (req == REQ_GYRO || req == REQ_ACCEL) {
                collecting = req;
                s_cal_busy = true;
                n = 0;
                next_pct = 25;
                for (int i = 0; i < 6; i++) { sum[i] = 0.0; mn[i] = 32767; mx[i] = -32768; }
                notify_progress(req == REQ_GYRO ? "gyro" : "accel", 0);
            } else if (req == REQ_NEUTRAL) {
                if (initialized) {
                    cal.neutral_angles[0] = roll;
                    cal.neutral_angles[1] = pitch;
                    cal.neutral_angles[2] = yaw;
                    cal.neutral_ok = 1;
                    cal_save(&cal);
                    cal_share(&cal);
                    notify_result("neutral", true, "Neutral upright pose saved");
                } else {
                    notify_result("neutral", false, "No sensor data yet");
                }
                notify_cal_state(&cal);
            } else if (req == REQ_YAW) {
                cal.neutral_angles[2] = yaw;
                cal_share(&cal);
                notify_result("yaw", true, "Yaw zeroed at current heading");
                notify_cal_state(&cal);
            } else if (req == REQ_CLEAR) {
                cal_defaults(&cal);
                cal_clear_nvs();
                cal_share(&cal);
                initialized = false;
                roll = 0.0f; pitch = 0.0f; yaw = 0.0f;
                notify_result("clear", true, "All MPU calibration cleared");
                notify_cal_state(&cal);
            }
        }

        mpu6050_raw_t raw;
        if (!mpu6050_read_raw(&raw)) {
            if (++read_fails >= MAX_READ_FAILS) {
                ESP_LOGW(TAG, "MPU-6050 connection lost");
                s_sensor_ok = false;
                collecting = REQ_NONE;
                s_cal_busy = false;
                notify_hw(false);
            }
            continue;
        }
        read_fails = 0;

        if (collecting != REQ_NONE) {
            const int16_t v[6] = { raw.ax, raw.ay, raw.az, raw.gx, raw.gy, raw.gz };
            for (int i = 0; i < 6; i++) {
                sum[i] += (double)v[i];
                if (v[i] < mn[i]) mn[i] = v[i];
                if (v[i] > mx[i]) mx[i] = v[i];
            }
            n++;

            const char *kind = (collecting == REQ_GYRO) ? "gyro" : "accel";
            int pct = (n * 100) / CAL_SAMPLES;
            if (pct >= next_pct && pct < 100) {
                notify_progress(kind, pct);
                next_pct += 25;
            }

            if (n >= CAL_SAMPLES) {
                bool gyro_still = true;
                for (int i = 3; i < 6; i++) {
                    if ((mx[i] - mn[i]) > CAL_GYRO_RANGE_LSB) gyro_still = false;
                }

                if (collecting == REQ_GYRO) {
                    if (!gyro_still) {
                        notify_result(kind, false, "Sensor moved. Keep it still and retry");
                    } else {
                        for (int i = 0; i < 3; i++) {
                            cal.gyro_bias[i] = (float)(sum[3 + i] / n) / MPU6050_GYRO_LSB_PER_DPS;
                        }
                        cal.gyro_ok = 1;
                        cal_save(&cal);
                        cal_share(&cal);
                        initialized = false;
                        notify_result(kind, true, "Gyro bias saved");
                    }
                } else {
                    bool accel_still = true;
                    for (int i = 0; i < 3; i++) {
                        if ((mx[i] - mn[i]) > CAL_ACCEL_RANGE_LSB) accel_still = false;
                    }
                    float mean_ax = (float)(sum[0] / n) / MPU6050_ACCEL_LSB_PER_G;
                    float mean_ay = (float)(sum[1] / n) / MPU6050_ACCEL_LSB_PER_G;
                    float mean_az = (float)(sum[2] / n) / MPU6050_ACCEL_LSB_PER_G;
                    float total_mag = sqrtf(mean_ax * mean_ax + mean_ay * mean_ay + mean_az * mean_az);

                    if (!gyro_still || !accel_still) {
                        notify_result(kind, false, "Sensor moved. Keep it still and retry");
                    } else if (total_mag < 0.70f || total_mag > 1.30f) {
                        notify_result(kind, false, "Invalid acceleration. Keep sensor resting still");
                    } else {
                        float max_comp = fabsf(mean_ax);
                        int axis = 0;
                        if (fabsf(mean_ay) > max_comp) { max_comp = fabsf(mean_ay); axis = 1; }
                        if (fabsf(mean_az) > max_comp) { max_comp = fabsf(mean_az); axis = 2; }

                        cal.accel_bias[0] = mean_ax;
                        cal.accel_bias[1] = mean_ay;
                        cal.accel_bias[2] = mean_az;
                        if (axis == 0) cal.accel_bias[0] -= copysignf(1.0f, mean_ax);
                        else if (axis == 1) cal.accel_bias[1] -= copysignf(1.0f, mean_ay);
                        else cal.accel_bias[2] -= copysignf(1.0f, mean_az);

                        cal.accel_ok = 1;
                        cal_save(&cal);
                        cal_share(&cal);
                        initialized = false;
                        notify_result(kind, true, "Accel bias saved (upright calibration OK)");
                    }
                }
                notify_cal_state(&cal);
                collecting = REQ_NONE;
                s_cal_busy = false;
                last_us = now_us;
            }
            continue;
        }

        float ax = (float)raw.ax / MPU6050_ACCEL_LSB_PER_G - cal.accel_bias[0];
        float ay = (float)raw.ay / MPU6050_ACCEL_LSB_PER_G - cal.accel_bias[1];
        float az = (float)raw.az / MPU6050_ACCEL_LSB_PER_G - cal.accel_bias[2];
        float gx = (float)raw.gx / MPU6050_GYRO_LSB_PER_DPS - cal.gyro_bias[0];
        float gy = (float)raw.gy / MPU6050_GYRO_LSB_PER_DPS - cal.gyro_bias[1];
        float gz = (float)raw.gz / MPU6050_GYRO_LSB_PER_DPS - cal.gyro_bias[2];
        float temp_c = (float)raw.temp / 340.0f + 36.53f;

        float dt = (float)(now_us - last_us) / 1000000.0f;
        last_us = now_us;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.05f) dt = 0.05f;

        float a_mag = sqrtf(ax * ax + ay * ay + az * az);
        float g_mag = sqrtf(gx * gx + gy * gy + gz * gz);
        bool instant_still = (g_mag < STILL_GYRO_DPS) && (fabsf(a_mag - 1.0f) < STILL_ACCEL_TOL_G);

        if (instant_still) {
            if (still_counter < STILL_CONFIRM_TICKS) still_counter++;
            else s_is_still = true;
        } else {
            still_counter = 0;
            s_is_still = false;
        }

        if (s_is_still) {
            gx = 0.0f;
            gy = 0.0f;
            gz = 0.0f;
        }

        float acc_roll  = atan2f(ay, ax) * RAD2DEG;
        float acc_pitch = atan2f(-az, sqrtf(ax * ax + ay * ay)) * RAD2DEG;

        if (!initialized) {
            roll = acc_roll;
            pitch = acc_pitch;
            yaw = 0.0f;
            initialized = true;
        } else {
            if (s_is_still) {
                roll  += 0.02f * wrap180(acc_roll - roll);
                pitch += 0.02f * wrap180(acc_pitch - pitch);
            } else {
                roll  += gz * dt;
                pitch += gy * dt;
                yaw   += gx * dt;

                if (a_mag > 0.85f && a_mag < 1.15f) {
                    roll  += 0.04f * wrap180(acc_roll - roll);
                    pitch += 0.04f * wrap180(acc_pitch - pitch);
                }
            }
            roll = wrap180(roll);
            pitch = wrap180(pitch);
            yaw = wrap180(yaw);
        }

        float out_r = wrap180(roll - cal.neutral_angles[0]);
        float out_p = wrap180(pitch - cal.neutral_angles[1]);
        float out_y = wrap180(yaw - cal.neutral_angles[2]);

        if (s_is_still) {
            if (fabsf(out_r) < 0.25f) out_r = 0.0f;
            if (fabsf(out_p) < 0.25f) out_p = 0.0f;
            if (fabsf(out_y) < 0.25f) out_y = 0.0f;
        }

        imu_pose_t out;
        out.roll  = out_r;
        out.pitch = out_p;
        out.yaw   = out_y;
        out.ax = ax; out.ay = ay; out.az = az;
        out.gx = gx; out.gy = gy; out.gz = gz;
        out.temp = temp_c;
        publish_pose(&out, now_us);

        if (++tele_div >= TELEMETRY_DIV) {
            tele_div = 0;
            if (s_stream) {
                char line[128];
                if (imu_format_line(line, sizeof(line))) {
                    if (ble_server_is_connected()) {
                        ble_server_send_notify(line);
                    }
                    if (espnow_sync_is_paired()) {
                        espnow_sync_send_imu_line(line);
                    }
                }
            }
        }
    }
}

void imu_init(void) {
    memset(&s_pose, 0, sizeof(s_pose));
    cal_defaults(&s_cal_shared);
    xTaskCreatePinnedToCore(imu_task, "imu_task", 6144, NULL, 6, NULL, 0);
}

bool imu_get(imu_pose_t *out) {
    if (out == NULL) return false;
    return pose_snapshot(out);
}

bool imu_has_sensor(void) {
    return s_sensor_ok;
}

void imu_set_stream(bool enabled) {
    s_stream = enabled;
}

bool imu_get_stream(void) {
    return s_stream;
}

bool imu_request_cal(const char *kind) {
    if (kind == NULL) return false;
    int req = REQ_NONE;
    if (strcmp(kind, "gyro") == 0) req = REQ_GYRO;
    else if (strcmp(kind, "accel") == 0) req = REQ_ACCEL;
    else if (strcmp(kind, "neutral") == 0) req = REQ_NEUTRAL;
    else if (strcmp(kind, "yaw") == 0) req = REQ_YAW;
    else if (strcmp(kind, "clear") == 0) req = REQ_CLEAR;
    if (req == REQ_NONE) return false;
    if (s_cal_busy || s_cal_req != REQ_NONE) return false;
    if (!s_sensor_ok) return false;
    s_cal_req = req;
    return true;
}

void imu_report_state(imu_print_fn_t print_fn) {
    if (print_fn == NULL) return;

    char hw[48];
    snprintf(hw, sizeof(hw), "MPUHW:ok=%d,addr=%d\n", s_sensor_ok ? 1 : 0, (int)mpu6050_address());
    print_fn(hw);

    imu_cal_t c;
    portENTER_CRITICAL(&s_lock);
    c = s_cal_shared;
    portEXIT_CRITICAL(&s_lock);

    char line[240];
    format_cal_line(line, sizeof(line), &c);
    print_fn(line);
}