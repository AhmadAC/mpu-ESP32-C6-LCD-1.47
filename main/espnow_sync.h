#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESPNOW_CHANNEL        1
#define ESPNOW_MAGIC          0x59444152
#define ESPNOW_PAYLOAD_CMD_SZ 192

typedef enum {
    ESPNOW_ROLE_RECEIVER = 0,
    ESPNOW_ROLE_SENDER   = 1
} espnow_role_t;

typedef enum {
    ESPNOW_MSG_DISCOVERY = 1,
    ESPNOW_MSG_PAIR_REQ,
    ESPNOW_MSG_PAIR_ACK,
    ESPNOW_MSG_HEARTBEAT,
    ESPNOW_MSG_SERVO_SET,
    ESPNOW_MSG_CMD_STR,
    ESPNOW_MSG_CMD_REPLY,
    ESPNOW_MSG_IMU_LINE,
    ESPNOW_MSG_STATUS
} espnow_msg_type_t;

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint8_t  sender_role;
    uint8_t  sender_mac[6];
    uint16_t seq;
    uint8_t  msg_type;
    union {
        struct {
            uint8_t proto_version;
        } discovery;
        struct {
            uint8_t accepted;
        } pair_resp;
        struct {
            uint32_t uptime_ms;
        } heartbeat;
        struct {
            uint8_t  s1;
            uint8_t  s2;
            uint8_t  s3;
            uint8_t  s4;
            uint16_t move_ms;
            uint8_t  flags;
        } servo_set;
        struct {
            char cmd[ESPNOW_PAYLOAD_CMD_SZ];
        } cmd_str;
        struct {
            char reply[ESPNOW_PAYLOAD_CMD_SZ];
        } cmd_reply;
        struct {
            char line[ESPNOW_PAYLOAD_CMD_SZ];
        } imu_line;
        struct {
            uint8_t s1, s2, s3, s4;
            int8_t  o1, o2, o3, o4;
            uint8_t smin, smax;
            uint8_t en_mask;
            uint8_t flags;
            uint8_t active_slot;
        } status;
    } payload;
} espnow_packet_t;
#pragma pack(pop)

typedef void (*espnow_cmd_handler_t)(const char *cmd_str);

void espnow_sync_init(espnow_role_t role);
bool espnow_sync_is_paired(void);
void espnow_sync_get_peer_mac(uint8_t mac[6]);
int8_t espnow_sync_get_last_rssi(void);
espnow_role_t espnow_sync_get_role(void);

bool espnow_sync_send_servos(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, uint16_t move_ms, bool mpu_active);
bool espnow_sync_send_cmd(const char *cmd_str);
bool espnow_sync_send_reply(const char *reply_str);
bool espnow_sync_send_status(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4,
                             int8_t o1, int8_t o2, int8_t o3, int8_t o4,
                             uint8_t smin, uint8_t smax,
                             uint8_t en_mask, uint8_t flags, uint8_t active_slot);
bool espnow_sync_send_imu_line(const char *imu_str);

bool espnow_sync_get_cached_arm_status(int *s1, int *s2, int *s3, int *s4,
                                       int *o1, int *o2, int *o3, int *o4,
                                       int *smin, int *smax,
                                       bool *e1, bool *e2, bool *e3, bool *e4,
                                       bool *idle, bool *run, bool *loop, int *slot);

void espnow_sync_register_cmd_handler(espnow_cmd_handler_t handler);
bool espnow_sync_format_line(char *buf, size_t max_len);

#ifdef __cplusplus
}
#endif