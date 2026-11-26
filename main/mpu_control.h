#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*mpu_control_print_fn_t)(const char *str);

void  mpu_control_init(void);
void  mpu_control_set_enabled(bool enabled);
bool  mpu_control_is_enabled(void);
void  mpu_control_set_gain(float gain);
float mpu_control_get_gain(void);
void  mpu_control_set_axes(bool roll, bool pitch, bool yaw);
void  mpu_control_get_axes(bool *roll, bool *pitch, bool *yaw);
void  mpu_control_set_invert(bool roll, bool pitch, bool yaw);
void  mpu_control_get_invert(bool *roll, bool *pitch, bool *yaw);
void  mpu_control_save_settings(void);
void  mpu_control_report(mpu_control_print_fn_t print_fn);

#ifdef __cplusplus
}
#endif