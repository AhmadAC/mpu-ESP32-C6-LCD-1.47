#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float ax, ay, az;
    float gx, gy, gz;
    float temp;
} imu_pose_t;

typedef void (*imu_print_fn_t)(const char *str);

void imu_init(void);
bool imu_get(imu_pose_t *out);
bool imu_has_sensor(void);
void imu_set_stream(bool enabled);
bool imu_get_stream(void);
bool imu_request_cal(const char *kind);
void imu_report_state(imu_print_fn_t print_fn);
bool imu_format_line(char *buf, size_t len);

#ifdef __cplusplus
}
#endif