#include "command_processor.h"
#include "node_config.h"
#include "servo_controller.h"
#include "imu.h"
#include "mpu_control.h"
#include "espnow_sync.h"
#include "ble_server.h"

#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <ctype.h>

static const char *TAG = "CMD_PROC";

#define CLAW_CLOSED_ANGLE  0
#define CLAW_OPEN_ANGLE    130

#if NODE_IS_RECEIVER
static int parse_servo_index(const char *token) {
    if (!token) return -1;
    if (strcmp(token, "1") == 0 || strcasecmp(token, "s1") == 0 || strcasecmp(token, "claw") == 0) return 0;
    if (strcmp(token, "2") == 0 || strcasecmp(token, "s2") == 0 || strcasecmp(token, "arm") == 0) return 1;
    if (strcmp(token, "3") == 0 || strcasecmp(token, "s3") == 0 || strcasecmp(token, "shoulder") == 0) return 2;
    if (strcmp(token, "4") == 0 || strcasecmp(token, "s4") == 0 || strcasecmp(token, "scissor") == 0 || strcasecmp(token, "lift") == 0) return 3;
    return -1;
}
#endif

static bool parse_bool_arg(const char *s) {
    if (!s) return false;
    return (strcasecmp(s, "on") == 0 || strcmp(s, "1") == 0 || strcasecmp(s, "true") == 0);
}

#if NODE_IS_SENDER
static bool is_arm_motion_override(const char *cmd) {
    if (!cmd) return false;
    if ((cmd[0] == 's' || cmd[0] == 'S') && (cmd[1] >= '1' && cmd[1] <= '3')) return true;
    if (strncmp(cmd, "move", 4) == 0) return true;
    if (strncmp(cmd, "set", 3) == 0) return true;
    if (strncmp(cmd, "claw", 4) == 0) return true;
    if (strncmp(cmd, "arm", 3) == 0 && (cmd[3] == ' ' || cmd[3] == '\0')) return true;
    if (strncmp(cmd, "shoulder", 8) == 0) return true;
    if (strcmp(cmd, "home") == 0 || strcmp(cmd, "center") == 0) return true;
    if (strcmp(cmd, "stop") == 0) return true;
    if (strncmp(cmd, "action_run", 10) == 0 || strncmp(cmd, "run ", 4) == 0) return true;
    if (strncmp(cmd, "action_loop", 11) == 0 || strncmp(cmd, "loop ", 5) == 0) return true;
    if (strncmp(cmd, "seq", 3) == 0) return true;
    return false;
}

static void release_mpu_control(command_print_fn_t print_fn) {
    if (mpu_control_is_enabled()) {
        mpu_control_set_enabled(false);
        mpu_control_report(print_fn);
    }
}
#endif

void command_processor_init(void) {
    ESP_LOGI(TAG, "Command Processor Initialized for [%s] (4-Axis Arm & Scissor Lift)", NODE_ROLE_NAME);
}

size_t command_build_status(char *buf, size_t len) {
    int s1 = 90, s2 = 90, s3 = 90, s4 = 90;
    int o1 = 0, o2 = 0, o3 = 0, o4 = 0;
    int smin = 0, smax = 180;
    bool e1 = false, e2 = false, e3 = false, e4 = false;
    bool auto_idle = false;
    bool is_running = false;
    bool is_looping = false;
    int  active_slot = 0;

#if NODE_IS_RECEIVER
    servo_get_all_angles(&s1, &s2, &s3, &s4);
    servo_get_all_offsets(&o1, &o2, &o3, &o4);
    servo_get_all_enabled(&e1, &e2, &e3, &e4);
    servo_get_scissor_range(&smin, &smax);
    auto_idle = servo_get_auto_idle();
    is_running = servo_is_sequence_running();
    is_looping = servo_is_sequence_looping();
    active_slot = servo_get_active_slot();
#elif NODE_IS_SENDER
    espnow_sync_get_cached_arm_status(&s1, &s2, &s3, &s4, &o1, &o2, &o3, &o4,
                                      &smin, &smax, &e1, &e2, &e3, &e4,
                                      &auto_idle, &is_running, &is_looping, &active_slot);
#endif

    int brownout = (esp_reset_reason() == ESP_RST_BROWNOUT) ? 1 : 0;

    int n = snprintf(buf, len,
                     "STATUS:s1=%d,s2=%d,s3=%d,s4=%d,o1=%d,o2=%d,o3=%d,o4=%d,min=%d,max=%d,en1=%d,en2=%d,en3=%d,en4=%d,idle=%d,run=%d,act=%d,loop=%d,bo=%d,mpuc=%d,mpuok=%d\n",
                     s1, s2, s3, s4, o1, o2, o3, o4, smin, smax,
                     e1 ? 1 : 0, e2 ? 1 : 0, e3 ? 1 : 0, e4 ? 1 : 0,
                     auto_idle ? 1 : 0,
                     is_running ? 1 : 0,
                     active_slot,
                     is_looping ? 1 : 0,
                     brownout,
                     mpu_control_is_enabled() ? 1 : 0,
                     imu_has_sensor() ? 1 : 0);
    if (n < 0) return 0;
    return (size_t)n;
}

void command_notify_status(void) {
    if (!ble_server_is_connected()) return;
    char status_str[256];
    command_build_status(status_str, sizeof(status_str));
    ble_server_send_notify(status_str);

    char espnow_str[128];
    if (espnow_sync_format_line(espnow_str, sizeof(espnow_str))) {
        ble_server_send_notify(espnow_str);
    }
}

void command_process(const char *cmd_line, command_print_fn_t print_fn) {
    if (!cmd_line || !print_fn) return;

    const char *p = cmd_line;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0') return;

    if (strcmp(p, "espnow_get") == 0 || strcmp(p, "espnow") == 0) {
        char line[128];
        if (espnow_sync_format_line(line, sizeof(line))) {
            print_fn(line);
        }
        return;
    }

    if (strcmp(p, "status") == 0) {
        char msg[256];
        command_build_status(msg, sizeof(msg));
        print_fn(msg);

        char espnow_str[128];
        if (espnow_sync_format_line(espnow_str, sizeof(espnow_str))) {
            print_fn(espnow_str);
        }
        return;
    }

    if (strcmp(p, "help") == 0 || strcmp(p, "?") == 0) {
        print_fn("Robotic Arm & Scissor Lift Controller:\n"
                 "  espnow                               : Print wireless link status\n"
                 "  mpu <on|off>                         : Toggle 20 Hz IMU telemetry stream\n"
                 "  mpu_read / mpu_get                   : Telemetry / state readout\n"
                 "  mpu_cal <gyro|accel|neutral|yaw|clear>: Calibrate MPU\n"
                 "  mpu_ctrl <on|off>                    : Wireless MPU-to-Arm tracking\n"
                 "  claw <0-180|open|close>              : Claw gripper (S1)\n"
                 "  arm <0-180> / shoulder <0-180>       : Joint angles (S2, S3)\n"
                 "  scissor <0-180> / s4 <0-180>         : Scissor lift angle (S4)\n"
                 "  lift <0-100>                         : Proportional lift height percentage\n"
                 "  pins <p1> <p2> <p3> <p4>             : Remap all 4 hardware GPIO pins\n"
                 "  pin <s1|s2|s3|s4> <gpio>             : Remap single servo GPIO pin\n"
                 "  range_set <min> <max> / range_get    : Scissor lift end-stop trim\n"
                 "  home / min / max / mid / stop        : Presets and motion stop\n"
                 "  detach / attach                      : Motor torque management\n"
                 "  status                               : Full state dump\n");
        return;
    }

    if (strcmp(p, "reboot") == 0 || strcmp(p, "restart") == 0) {
        print_fn("Rebooting ESP32...\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        return;
    }

    bool is_mpu_cmd = (strncmp(p, "mpu", 3) == 0);
    if (is_mpu_cmd) {
#if NODE_IS_RECEIVER
        if (!imu_has_sensor() && espnow_sync_is_paired()) {
            espnow_sync_send_cmd(p);
            return;
        }
#endif
        if (strncmp(p, "mpu ", 4) == 0) {
            const char *arg = p + 4;
            while (*arg == ' ') arg++;
            if (strcasecmp(arg, "on") == 0 || strcasecmp(arg, "off") == 0) {
                imu_set_stream(parse_bool_arg(arg));
                print_fn(imu_get_stream() ? ">> MPU telemetry stream ON\n" : ">> MPU telemetry stream OFF\n");
            } else {
                print_fn(">> Usage: mpu <on|off>\n");
            }
            return;
        }

        if (strcmp(p, "mpu_read") == 0) {
            char line[128];
            if (imu_format_line(line, sizeof(line))) {
                print_fn(line);
            } else {
                print_fn(">> MPU-6050: no data (sensor missing or not responding)\n");
            }
            return;
        }

        if (strcmp(p, "mpu_get") == 0) {
            imu_report_state(print_fn);
            mpu_control_report(print_fn);
            return;
        }

        if (strncmp(p, "mpu_cal ", 8) == 0) {
            char kind[16] = {0};
            if (sscanf(p + 8, "%15s", kind) == 1 && imu_request_cal(kind)) {
                return;
            }
            if (!imu_has_sensor()) {
                print_fn("MPUCALRES:kind=none,ok=0,msg=MPU-6050 not detected. Check wiring\n");
            } else {
                print_fn("MPUCALRES:kind=none,ok=0,msg=Busy or invalid. Use gyro/accel/neutral/yaw/clear\n");
            }
            return;
        }

        if (strncmp(p, "mpu_ctrl ", 9) == 0) {
            bool en = parse_bool_arg(p + 9);
            if (!imu_has_sensor()) {
                print_fn(">> Error: MPU-6050 not detected. Check wiring.\n");
                mpu_control_set_enabled(false);
            } else {
                mpu_control_set_enabled(en);
            }
            mpu_control_report(print_fn);
            return;
        }

        if (strncmp(p, "mpu_gain ", 9) == 0) {
            char val[16] = {0};
            char flag[8] = {0};
            int n = sscanf(p + 9, "%15s %7s", val, flag);
            if (n >= 1) {
                mpu_control_set_gain(strtof(val, NULL));
                if (n == 2 && strcasecmp(flag, "save") == 0) {
                    mpu_control_save_settings();
                    mpu_control_report(print_fn);
                }
                return;
            }
            print_fn(">> Usage: mpu_gain <0.5-4.0> [save]\n");
            return;
        }

        if (strncmp(p, "mpu_axes ", 9) == 0 || strncmp(p, "mpu_inv ", 8) == 0) {
            bool is_axes = (strncmp(p, "mpu_axes ", 9) == 0);
            int r = 0, pi = 0, y = 0;
            if (sscanf(p + (is_axes ? 9 : 8), "%d %d %d", &r, &pi, &y) == 3) {
                if (is_axes) mpu_control_set_axes(r != 0, pi != 0, y != 0);
                else mpu_control_set_invert(r != 0, pi != 0, y != 0);
                mpu_control_report(print_fn);
            } else {
                print_fn(is_axes ? ">> Usage: mpu_axes <roll 0|1> <pitch 0|1> <yaw 0|1>\n"
                                 : ">> Usage: mpu_inv <roll 0|1> <pitch 0|1> <yaw 0|1>\n");
            }
            return;
        }
    }

#if NODE_IS_SENDER
    if (is_arm_motion_override(p)) {
        release_mpu_control(print_fn);
    }

    if (espnow_sync_is_paired()) {
        espnow_sync_send_cmd(p);
        return;
    } else {
        print_fn(">> [ESP-NOW Warning] Arm Receiver not paired yet. Scanning...\n");
        return;
    }
#else
    if (strncmp(p, "pins ", 5) == 0) {
        int p1 = -1, p2 = -1, p3 = -1, p4 = -1;
        if (sscanf(p + 5, "%d %d %d %d", &p1, &p2, &p3, &p4) == 4) {
            servo_set_pins(p1, p2, p3, p4);
            char resp[64];
            snprintf(resp, sizeof(resp), "PINS:%d,%d,%d,%d\n", p1, p2, p3, p4);
            print_fn(resp);
            command_notify_status();
            return;
        }
    }

    if (strncmp(p, "pin ", 4) == 0) {
        char tok[16] = {0};
        int gpio = -1;
        if (sscanf(p + 4, "%15s %d", tok, &gpio) == 2) {
            int idx = parse_servo_index(tok);
            if (idx >= 0 && gpio >= 0 && gpio < GPIO_NUM_MAX) {
                servo_set_single_pin(idx, gpio);
                int p1 = 0, p2 = 1, p3 = 2, p4 = 3;
                servo_get_pins(&p1, &p2, &p3, &p4);
                char resp[64];
                snprintf(resp, sizeof(resp), "PINS:%d,%d,%d,%d\n", p1, p2, p3, p4);
                print_fn(resp);
                command_notify_status();
                return;
            }
        }
        print_fn(">> Usage: pin <s1|s2|s3|s4|1|2|3|4> <gpio>\n");
        return;
    }

    if (strcmp(p, "pins") == 0) {
        int p1 = 0, p2 = 1, p3 = 2, p4 = 3;
        servo_get_pins(&p1, &p2, &p3, &p4);
        char resp[64];
        snprintf(resp, sizeof(resp), "PINS:%d,%d,%d,%d\n", p1, p2, p3, p4);
        print_fn(resp);
        return;
    }

    if (strncmp(p, "claw ", 5) == 0) {
        const char *arg = p + 5;
        int angle = -1;
        if (strcasecmp(arg, "close") == 0) angle = CLAW_CLOSED_ANGLE;
        else if (strcasecmp(arg, "open") == 0) angle = CLAW_OPEN_ANGLE;
        else angle = atoi(arg);

        if (angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE) {
            servo_set_angle(0, angle);
            char msg[64];
            snprintf(msg, sizeof(msg), "ANG:%d,%d,%d,%d\n", angle, servo_get_angle(1), servo_get_angle(2), servo_get_angle(3));
            print_fn(msg);
        } else {
            print_fn(">> Usage: claw <0-180|open|close>\n");
        }
        return;
    }

    if (strncmp(p, "arm ", 4) == 0) {
        int angle = atoi(p + 4);
        if (angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE) {
            servo_set_angle(1, angle);
            char msg[64];
            snprintf(msg, sizeof(msg), "ANG:%d,%d,%d,%d\n", servo_get_angle(0), angle, servo_get_angle(2), servo_get_angle(3));
            print_fn(msg);
            return;
        }
    }

    if (strncmp(p, "shoulder ", 9) == 0) {
        int angle = atoi(p + 9);
        if (angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE) {
            servo_set_angle(2, angle);
            char msg[64];
            snprintf(msg, sizeof(msg), "ANG:%d,%d,%d,%d\n", servo_get_angle(0), servo_get_angle(1), angle, servo_get_angle(3));
            print_fn(msg);
            return;
        }
    }

    if (strncmp(p, "scissor ", 8) == 0) {
        int angle = atoi(p + 8);
        if (angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE) {
            servo_set_angle(3, angle);
            char msg[64];
            snprintf(msg, sizeof(msg), "ANG:%d,%d,%d,%d\n", servo_get_angle(0), servo_get_angle(1), servo_get_angle(2), angle);
            print_fn(msg);
            return;
        }
    }

    if (strncmp(p, "lift ", 5) == 0) {
        int pct = atoi(p + 5);
        servo_set_scissor_percent(pct);
        char msg[64];
        snprintf(msg, sizeof(msg), "ANG:%d,%d,%d,%d\n", servo_get_angle(0), servo_get_angle(1), servo_get_angle(2), servo_get_angle(3));
        print_fn(msg);
        return;
    }

    if (strncmp(p, "range_set ", 10) == 0) {
        int mn = 0, mx = 180;
        if (sscanf(p + 10, "%d %d", &mn, &mx) == 2) {
            servo_set_scissor_range(mn, mx);
            char msg[48];
            snprintf(msg, sizeof(msg), "RANGE:%d,%d\n", mn, mx);
            print_fn(msg);
            command_notify_status();
            return;
        }
    }

    if (strcmp(p, "range_get") == 0) {
        int mn = 0, mx = 180;
        servo_get_scissor_range(&mn, &mx);
        char msg[48];
        snprintf(msg, sizeof(msg), "RANGE:%d,%d\n", mn, mx);
        print_fn(msg);
        return;
    }

    if ((p[0] == 's' || p[0] == 'S') && (p[1] >= '1' && p[1] <= '4') && (p[2] == ' ' || p[2] == ':')) {
        int idx = p[1] - '1';
        int angle = atoi(p + 3);
        if (angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE) {
            servo_set_angle(idx, angle);
            char msg[64];
            snprintf(msg, sizeof(msg), "ANG:%d,%d,%d,%d\n", servo_get_angle(0), servo_get_angle(1), servo_get_angle(2), servo_get_angle(3));
            print_fn(msg);
            return;
        } else {
            char msg[80];
            snprintf(msg, sizeof(msg), ">> Error: Angle must be %d-%d deg.\n", SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
            print_fn(msg);
            return;
        }
    }

    if (strncmp(p, "set ", 4) == 0) {
        int s1 = 90, s2 = 90, s3 = 90, s4 = servo_get_angle(3);
        int matched = sscanf(p + 4, "%d %d %d %d", &s1, &s2, &s3, &s4);
        if (matched >= 3) {
            servo_set_all(s1, s2, s3, s4);
            char msg[64];
            snprintf(msg, sizeof(msg), "ANG:%d,%d,%d,%d\n", s1, s2, s3, s4);
            print_fn(msg);
        } else {
            print_fn(">> Usage: set <s1> <s2> <s3> [s4]\n");
        }
        return;
    }

    if (strncmp(p, "move ", 5) == 0) {
        int s1 = 90, s2 = 90, s3 = 90, s4 = servo_get_angle(3), dur = 400;
        int n = sscanf(p + 5, "%d %d %d %d %d", &s1, &s2, &s3, &s4, &dur);
        if (n >= 5) {
            servo_move_smooth(s1, s2, s3, s4, dur);
            char msg[96];
            snprintf(msg, sizeof(msg), ">> Moving to (%d, %d, %d, %d) over %dms\n", s1, s2, s3, s4, dur);
            print_fn(msg);
            return;
        } else if (n == 4) {
            if (s4 > 180) {
                dur = s4;
                s4 = servo_get_angle(3);
            } else {
                dur = 400;
            }
            servo_move_smooth(s1, s2, s3, s4, dur);
            char msg[96];
            snprintf(msg, sizeof(msg), ">> Moving to (%d, %d, %d, %d) over %dms\n", s1, s2, s3, s4, dur);
            print_fn(msg);
            return;
        } else if (n == 3) {
            servo_move_smooth(s1, s2, s3, servo_get_angle(3), 400);
            char msg[96];
            snprintf(msg, sizeof(msg), ">> Moving to (%d, %d, %d, %d) over 400ms\n", s1, s2, s3, servo_get_angle(3));
            print_fn(msg);
            return;
        } else {
            print_fn(">> Usage: move <s1> <s2> <s3> [s4] [duration_ms]\n");
            return;
        }
    }

    if (strncmp(p, "nudge ", 6) == 0) {
        char tok[8] = {0};
        int delta = 0;
        if (sscanf(p + 6, "%7s %d", tok, &delta) == 2) {
            int idx = parse_servo_index(tok);
            if (idx >= 0) {
                int target = servo_get_angle(idx) + delta;
                if (target < SERVO_MIN_ANGLE) target = SERVO_MIN_ANGLE;
                if (target > SERVO_MAX_ANGLE) target = SERVO_MAX_ANGLE;
                servo_set_angle(idx, target);
                char msg[64];
                snprintf(msg, sizeof(msg), "ANG:%d,%d,%d,%d\n", servo_get_angle(0), servo_get_angle(1), servo_get_angle(2), servo_get_angle(3));
                print_fn(msg);
                return;
            }
        }
        print_fn(">> Usage: nudge <s1|s2|s3|s4|scissor> <+/-deg>\n");
        return;
    }

    if (strcmp(p, "min") == 0) {
        int mn = 0, mx = 180;
        servo_get_scissor_range(&mn, &mx);
        servo_set_angle(3, mn);
        char msg[64];
        snprintf(msg, sizeof(msg), ">> Scissor to Min (%d deg)\n", mn);
        print_fn(msg);
        return;
    }
    if (strcmp(p, "max") == 0) {
        int mn = 0, mx = 180;
        servo_get_scissor_range(&mn, &mx);
        servo_set_angle(3, mx);
        char msg[64];
        snprintf(msg, sizeof(msg), ">> Scissor to Max (%d deg)\n", mx);
        print_fn(msg);
        return;
    }
    if (strcmp(p, "mid") == 0) {
        int mn = 0, mx = 180;
        servo_get_scissor_range(&mn, &mx);
        int mid = (mn + mx) / 2;
        servo_set_angle(3, mid);
        char msg[64];
        snprintf(msg, sizeof(msg), ">> Scissor to Mid (%d deg)\n", mid);
        print_fn(msg);
        return;
    }

    if (strcmp(p, "home") == 0 || strcmp(p, "center") == 0) {
        servo_move_smooth(90, 90, 90, 90, 550);
        print_fn(">> Homing all servos to 90 deg...\n");
        return;
    }

    if (strcmp(p, "stop") == 0) {
        servo_stop_motion();
        print_fn(">> Motion stopped.\n");
        return;
    }

    if (strcmp(p, "detach") == 0 || strcmp(p, "idle") == 0 || strcmp(p, "disable") == 0) {
        servo_detach_all();
        print_fn(">> Servos detached (idle, PWM=0).\n");
        return;
    }

    if (strncmp(p, "detach ", 7) == 0) {
        int idx = parse_servo_index(p + 7);
        if (idx >= 0) {
            servo_detach(idx);
            char msg[48];
            snprintf(msg, sizeof(msg), ">> Servo %d detached.\n", idx + 1);
            print_fn(msg);
        } else {
            print_fn(">> Usage: detach [s1|s2|s3|s4|scissor]\n");
        }
        return;
    }

    if (strcmp(p, "attach") == 0 || strcmp(p, "enable") == 0) {
        servo_attach_all();
        print_fn(">> Servos attached (PWM enabled).\n");
        return;
    }

    if (strncmp(p, "autoidle ", 9) == 0) {
        bool en = parse_bool_arg(p + 9);
        servo_set_auto_idle(en);
        char msg[48];
        snprintf(msg, sizeof(msg), "AUTO_IDLE:%d\n", en ? 1 : 0);
        print_fn(msg);
        return;
    }

    if (strncmp(p, "offset ", 7) == 0) {
        char tok[8] = {0};
        int off = 0;
        if (sscanf(p + 7, "%7s %d", tok, &off) == 2) {
            int idx = parse_servo_index(tok);
            if (idx >= 0) {
                servo_set_offset(idx, off);
                return;
            }
        }
        print_fn(">> Usage: offset <s1|s2|s3|s4> <-45..45>\n");
        return;
    }

    if (strncmp(p, "cal_set ", 8) == 0) {
        int o1 = 0, o2 = 0, o3 = 0, o4 = 0;
        int n = sscanf(p + 8, "%d %d %d %d", &o1, &o2, &o3, &o4);
        if (n >= 3) {
            servo_save_offsets(o1, o2, o3, o4);
            servo_get_all_offsets(&o1, &o2, &o3, &o4);
            char msg[64];
            snprintf(msg, sizeof(msg), "CAL:%d,%d,%d,%d\n", o1, o2, o3, o4);
            print_fn(msg);
            command_notify_status();
        } else {
            print_fn(">> Usage: cal_set <o1> <o2> <o3> [o4]\n");
        }
        return;
    }

    if (strcmp(p, "cal_get") == 0) {
        int o1 = 0, o2 = 0, o3 = 0, o4 = 0;
        servo_get_all_offsets(&o1, &o2, &o3, &o4);
        char msg[64];
        snprintf(msg, sizeof(msg), "CAL:%d,%d,%d,%d\n", o1, o2, o3, o4);
        print_fn(msg);
        int mn = 0, mx = 180;
        servo_get_scissor_range(&mn, &mx);
        char rmsg[48];
        snprintf(rmsg, sizeof(rmsg), "RANGE:%d,%d\n", mn, mx);
        print_fn(rmsg);
        return;
    }

    if (strncmp(p, "action_run ", 11) == 0 || strncmp(p, "run ", 4) == 0) {
        int slot = atoi(strncmp(p, "run ", 4) == 0 ? (p + 4) : (p + 11));
        if (servo_action_run(slot, false)) {
            char msg[64];
            snprintf(msg, sizeof(msg), ">> Running action %d\n", slot);
            print_fn(msg);
        } else {
            print_fn(">> Error: Invalid action slot (1-8).\n");
        }
        return;
    }

    if (strncmp(p, "action_loop ", 12) == 0 || strncmp(p, "loop ", 5) == 0) {
        int slot = atoi(strncmp(p, "loop ", 5) == 0 ? (p + 5) : (p + 12));
        if (servo_action_run(slot, true)) {
            char msg[64];
            snprintf(msg, sizeof(msg), ">> Looping action %d\n", slot);
            print_fn(msg);
        } else {
            print_fn(">> Error: Invalid action slot (1-8).\n");
        }
        return;
    }

    if (strncmp(p, "action_set ", 11) == 0) {
        const char *arg = p + 11;
        while (*arg == ' ') arg++;
        char *endptr = NULL;
        long slot = strtol(arg, &endptr, 10);
        if (slot >= 1 && slot <= ACTION_NUM_SLOTS && endptr != NULL) {
            while (*endptr == ' ' || *endptr == '\t') endptr++;
            if (servo_action_set((int)slot, endptr)) {
                char resp[ACTION_MAX_LEN + 16];
                snprintf(resp, sizeof(resp), "ACT:%ld=%s\n", slot, servo_action_get((int)slot));
                print_fn(resp);
                return;
            }
        }
        print_fn(">> Usage: action_set <1-8> <name>|<s1,s2,s3,s4,move_ms,wait_ms;...>\n");
        return;
    }

    if (strcmp(p, "action_get") == 0) {
        for (int i = 1; i <= ACTION_NUM_SLOTS; i++) {
            char line[ACTION_MAX_LEN + 16];
            snprintf(line, sizeof(line), "ACT:%d=%s\n", i, servo_action_get(i));
            print_fn(line);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        return;
    }

    if (strcmp(p, "action_reset") == 0) {
        servo_action_reset_defaults();
        for (int i = 1; i <= ACTION_NUM_SLOTS; i++) {
            char line[ACTION_MAX_LEN + 16];
            snprintf(line, sizeof(line), "ACT:%d=%s\n", i, servo_action_get(i));
            print_fn(line);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        print_fn(">> Actions restored to defaults.\n");
        return;
    }

    if (strncmp(p, "seq_loop ", 9) == 0) {
        if (servo_run_sequence_str(p + 9, true)) {
            print_fn(">> Looping custom sequence...\n");
        } else {
            print_fn(">> Error: Invalid sequence format.\n");
        }
        return;
    }
    if (strncmp(p, "seq ", 4) == 0) {
        if (servo_run_sequence_str(p + 4, false)) {
            print_fn(">> Running custom sequence...\n");
        } else {
            print_fn(">> Error: Invalid sequence format.\n");
        }
        return;
    }
#endif

    char err_msg[96];
    snprintf(err_msg, sizeof(err_msg), ">> Unknown command: '%s'. Enter 'help'.\n", p);
    print_fn(err_msg);
}

void command_on_ble_received(const char *cmd, void (*reply_fn)(const char *reply)) {
    ESP_LOGI(TAG, "BLE Command: '%s'", cmd);
    command_process(cmd, reply_fn);
}