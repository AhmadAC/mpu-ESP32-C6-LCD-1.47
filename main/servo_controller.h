#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// 4-Axis Robotic Arm & Scissor Lift Servo Mapping for Waveshare ESP32-C6-LCD-1.47:
// S1: Claw (Gripper) - Default GPIO 0 (Unshared header pin)
// S2: Arm            - Default GPIO 1 (Unshared header pin)
// S3: Shoulder       - Default GPIO 2 (Unshared header pin)
// S4: Scissor Lift   - Default GPIO 3 (Unshared header pin)
#define SERVO_COUNT         4
#define SERVO_1_PIN_DEFAULT GPIO_NUM_0
#define SERVO_2_PIN_DEFAULT GPIO_NUM_1
#define SERVO_3_PIN_DEFAULT GPIO_NUM_2
#define SERVO_4_PIN_DEFAULT GPIO_NUM_3

#define SERVO_1_PIN         SERVO_1_PIN_DEFAULT
#define SERVO_2_PIN         SERVO_2_PIN_DEFAULT
#define SERVO_3_PIN         SERVO_3_PIN_DEFAULT
#define SERVO_4_PIN         SERVO_4_PIN_DEFAULT

#define SERVO_MIN_PULSE_US  500
#define SERVO_MAX_PULSE_US  2500
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2500
#define SERVO_MIN_ANGLE     0
#define SERVO_MAX_ANGLE     180
#define SERVO_DEFAULT_ANGLE 90

#define SERVO_OFFSET_MIN    -45
#define SERVO_OFFSET_MAX    45

#define ACTION_NUM_SLOTS    8
#define ACTION_MAX_LEN      384
#define ACTION_MAX_STEPS    24

typedef struct {
    int s1;
    int s2;
    int s3;
    int s4;
    int move_ms;
    int wait_ms;
} servo_step_t;

void servo_controller_init(void);

// Angle commands
void servo_set_angle(int idx, int angle);
int  servo_get_angle(int idx);
void servo_set_all(int s1, int s2, int s3, int s4);
void servo_move_smooth(int s1, int s2, int s3, int s4, int duration_ms);
void servo_get_all_angles(int *s1, int *s2, int *s3, int *s4);

// Dynamic Hardware Pin Mapping (Saved to NVS)
void servo_set_pins(int p1, int p2, int p3, int p4);
void servo_set_single_pin(int idx, int gpio);
void servo_get_pins(int *p1, int *p2, int *p3, int *p4);

// Scissor Lift specific range & proportional controls (Servo 4)
void servo_set_scissor_percent(int pct);
int  servo_get_scissor_percent(void);
void servo_set_scissor_range(int min_angle, int max_angle);
void servo_get_scissor_range(int *min_angle, int *max_angle);

// Power / Attach / Detach controls
void servo_detach(int idx);
void servo_detach_all(void);
void servo_attach(int idx);
void servo_attach_all(void);
void servo_get_all_enabled(bool *e1, bool *e2, bool *e3, bool *e4);
void servo_set_auto_idle(bool enabled);
bool servo_get_auto_idle(void);

// Calibration Trim Offsets
void servo_set_offset(int idx, int offset);
void servo_save_offsets(int o1, int o2, int o3, int o4);
void servo_get_all_offsets(int *o1, int *o2, int *o3, int *o4);

// Programmable Sequence Routines
bool servo_action_set(int slot, const char *def);
const char *servo_action_get(int slot);
void servo_action_reset_defaults(void);
bool servo_action_run(int slot, bool loop);
bool servo_run_sequence_str(const char *seq_str, bool loop);
void servo_stop_motion(void);
bool servo_is_sequence_running(void);
bool servo_is_sequence_looping(void);
int  servo_get_active_slot(void);

#ifdef __cplusplus
}
#endif