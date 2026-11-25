#include "espnow_sync.h"
#include "ble_server.h"
#include "servo_controller.h"
#include "command_processor.h"
#include "console_serial.h"
#include "mpu_control.h"

#include <string.h>
#include <stdio.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "ESPNOW_SYNC";

static const uint8_t BCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

typedef struct {
    uint8_t src_mac[6];
    int8_t rssi;
    espnow_packet_t pkt;
} espnow_rx_item_t;

static espnow_role_t s_local_role = ESPNOW_ROLE_RECEIVER;
static uint8_t s_local_mac[6] = {0};
static uint8_t s_peer_mac[6]  = {0};
static volatile bool s_is_paired = false;
static int64_t s_last_recv_us = 0;
static int8_t  s_last_rssi = 0;
static uint16_t s_tx_seq = 0;

static espnow_cmd_handler_t s_cmd_handler = NULL;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t s_espnow_rx_queue = NULL;

static int  s_cached_s1 = 90, s_cached_s2 = 90, s_cached_s3 = 90, s_cached_s4 = 90;
static int  s_cached_o1 = 0,  s_cached_o2 = 0,  s_cached_o3 = 0,  s_cached_o4 = 0;
static int  s_cached_smin = 0, s_cached_smax = 180;
static bool s_cached_e1 = false, s_cached_e2 = false, s_cached_e3 = false, s_cached_e4 = false;
static bool s_cached_idle = false, s_cached_run = false, s_cached_loop = false;
static int  s_cached_slot = 0;
static bool s_has_cached_status = false;

static void notify_link_state(void);

static bool add_or_update_peer(const uint8_t *mac) {
    if (esp_now_is_peer_exist(mac)) {
        return true;
    }
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_add_peer failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

static bool send_packet(const uint8_t *dest_mac, espnow_packet_t *pkt) {
    pkt->magic = ESPNOW_MAGIC;
    pkt->sender_role = (uint8_t)s_local_role;
    pkt->seq = s_tx_seq++;
    memcpy(pkt->sender_mac, s_local_mac, 6);

    esp_err_t err = esp_now_send(dest_mac, (const uint8_t *)pkt, sizeof(*pkt));
    return (err == ESP_OK);
}

static void process_incoming_packet(const espnow_rx_item_t *item) {
    const espnow_packet_t *pkt = &item->pkt;

    if (memcmp(pkt->sender_mac, s_local_mac, 6) == 0) return;
    if (pkt->sender_role == (uint8_t)s_local_role) return;

    s_last_recv_us = esp_timer_get_time();
    s_last_rssi = item->rssi;

    add_or_update_peer(pkt->sender_mac);

    bool just_paired = false;
    portENTER_CRITICAL(&s_mux);
    if (!s_is_paired || memcmp(s_peer_mac, pkt->sender_mac, 6) != 0) {
        memcpy(s_peer_mac, pkt->sender_mac, 6);
        s_is_paired = true;
        just_paired = true;
    }
    portEXIT_CRITICAL(&s_mux);

    if (just_paired) {
        ESP_LOGI(TAG, "ESP-NOW Auto-paired with peer %02X:%02X:%02X:%02X:%02X:%02X (role=%d)",
                 pkt->sender_mac[0], pkt->sender_mac[1], pkt->sender_mac[2],
                 pkt->sender_mac[3], pkt->sender_mac[4], pkt->sender_mac[5], pkt->sender_role);
        notify_link_state();

        espnow_packet_t ack = {};
        ack.msg_type = ESPNOW_MSG_PAIR_ACK;
        ack.payload.pair_resp.accepted = 1;
        send_packet(pkt->sender_mac, &ack);
    }

    switch (pkt->msg_type) {
    case ESPNOW_MSG_DISCOVERY: {
        espnow_packet_t ack = {};
        ack.msg_type = ESPNOW_MSG_PAIR_ACK;
        ack.payload.pair_resp.accepted = 1;
        send_packet(pkt->sender_mac, &ack);
        break;
    }

    case ESPNOW_MSG_PAIR_REQ:
    case ESPNOW_MSG_PAIR_ACK:
    case ESPNOW_MSG_HEARTBEAT:
        break;

    case ESPNOW_MSG_SERVO_SET: {
        if (s_local_role == ESPNOW_ROLE_RECEIVER) {
            uint8_t s1 = pkt->payload.servo_set.s1;
            uint8_t s2 = pkt->payload.servo_set.s2;
            uint8_t s3 = pkt->payload.servo_set.s3;
            uint8_t s4 = pkt->payload.servo_set.s4;
            uint16_t mv = pkt->payload.servo_set.move_ms;
            bool mpu_active = (pkt->payload.servo_set.flags & 1) != 0;

            if (mpu_active) {
                servo_set_angle(0, s1);
                servo_set_angle(1, s2);
                servo_set_angle(2, s3);
            } else {
                if (mv > 20) {
                    servo_move_smooth(s1, s2, s3, s4, mv);
                } else {
                    servo_set_angle(0, s1);
                    servo_set_angle(1, s2);
                    servo_set_angle(2, s3);
                    servo_set_angle(3, s4);
                }
            }
        }
        break;
    }

    case ESPNOW_MSG_CMD_STR: {
        char cmd_copy[ESPNOW_PAYLOAD_CMD_SZ + 1];
        strncpy(cmd_copy, pkt->payload.cmd_str.cmd, ESPNOW_PAYLOAD_CMD_SZ);
        cmd_copy[ESPNOW_PAYLOAD_CMD_SZ] = '\0';

        if (s_cmd_handler != NULL) {
            s_cmd_handler(cmd_copy);
        } else {
            command_process(cmd_copy, [](const char *reply) {
                if (reply && strlen(reply) > 0) {
                    espnow_sync_send_reply(reply);
                    ble_server_send_notify(reply);
                }
            });
        }
        break;
    }

    case ESPNOW_MSG_CMD_REPLY: {
        char reply_copy[ESPNOW_PAYLOAD_CMD_SZ + 1];
        strncpy(reply_copy, pkt->payload.cmd_reply.reply, ESPNOW_PAYLOAD_CMD_SZ);
        reply_copy[ESPNOW_PAYLOAD_CMD_SZ] = '\0';

        ble_server_send_notify(reply_copy);
        console_print(reply_copy);
        break;
    }

    case ESPNOW_MSG_IMU_LINE: {
        char imu_copy[ESPNOW_PAYLOAD_CMD_SZ + 1];
        strncpy(imu_copy, pkt->payload.imu_line.line, ESPNOW_PAYLOAD_CMD_SZ);
        imu_copy[ESPNOW_PAYLOAD_CMD_SZ] = '\0';

        ble_server_send_notify(imu_copy);
        break;
    }

    case ESPNOW_MSG_STATUS: {
        if (s_local_role == ESPNOW_ROLE_SENDER) {
            portENTER_CRITICAL(&s_mux);
            s_cached_s1 = pkt->payload.status.s1;
            s_cached_s2 = pkt->payload.status.s2;
            s_cached_s3 = pkt->payload.status.s3;
            s_cached_s4 = pkt->payload.status.s4;
            s_cached_o1 = pkt->payload.status.o1;
            s_cached_o2 = pkt->payload.status.o2;
            s_cached_o3 = pkt->payload.status.o3;
            s_cached_o4 = pkt->payload.status.o4;
            s_cached_smin = pkt->payload.status.smin;
            s_cached_smax = pkt->payload.status.smax;
            s_cached_e1 = (pkt->payload.status.en_mask & 1) != 0;
            s_cached_e2 = (pkt->payload.status.en_mask & 2) != 0;
            s_cached_e3 = (pkt->payload.status.en_mask & 4) != 0;
            s_cached_e4 = (pkt->payload.status.en_mask & 8) != 0;
            s_cached_idle = (pkt->payload.status.flags & 1) != 0;
            s_cached_run  = (pkt->payload.status.flags & 2) != 0;
            s_cached_loop = (pkt->payload.status.flags & 4) != 0;
            s_cached_slot = pkt->payload.status.active_slot;
            s_has_cached_status = true;
            portEXIT_CRITICAL(&s_mux);

            char status_line[216];
            snprintf(status_line, sizeof(status_line),
                     "STATUS:s1=%d,s2=%d,s3=%d,s4=%d,o1=%d,o2=%d,o3=%d,o4=%d,min=%d,max=%d,en1=%d,en2=%d,en3=%d,en4=%d,idle=%d,run=%d,act=%d,loop=%d,bo=0,mpuc=%d,mpuok=1\n",
                     s_cached_s1, s_cached_s2, s_cached_s3, s_cached_s4,
                     s_cached_o1, s_cached_o2, s_cached_o3, s_cached_o4,
                     s_cached_smin, s_cached_smax,
                     s_cached_e1 ? 1 : 0, s_cached_e2 ? 1 : 0,
                     s_cached_e3 ? 1 : 0, s_cached_e4 ? 1 : 0,
                     s_cached_idle ? 1 : 0, s_cached_run ? 1 : 0,
                     s_cached_slot, s_cached_loop ? 1 : 0,
                     mpu_control_is_enabled() ? 1 : 0);
            ble_server_send_notify(status_line);
        }
        break;
    }

    default:
        break;
    }
}

static void espnow_rx_task(void *pvParameters) {
    (void)pvParameters;
    espnow_rx_item_t item;

    while (1) {
        if (xQueueReceive(s_espnow_rx_queue, &item, portMAX_DELAY) == pdTRUE) {
            process_incoming_packet(&item);
        }
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!info || !data || len < (int)sizeof(espnow_packet_t) || s_espnow_rx_queue == NULL) return;
    espnow_rx_item_t item;
    memcpy(item.src_mac, info->src_addr, 6);
    item.rssi = (info->rx_ctrl != NULL) ? info->rx_ctrl->rssi : 0;
    memcpy(&item.pkt, data, sizeof(espnow_packet_t));
    if (item.pkt.magic == ESPNOW_MAGIC) {
        xQueueSend(s_espnow_rx_queue, &item, 0);
    }
}
#else
static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (!mac_addr || !data || len < (int)sizeof(espnow_packet_t) || s_espnow_rx_queue == NULL) return;
    espnow_rx_item_t item;
    memcpy(item.src_mac, mac_addr, 6);
    item.rssi = 0;
    memcpy(&item.pkt, data, sizeof(espnow_packet_t));
    if (item.pkt.magic == ESPNOW_MAGIC) {
        xQueueSend(s_espnow_rx_queue, &item, 0);
    }
}
#endif

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    (void)mac_addr;
    (void)status;
}

static void notify_link_state(void) {
    char line[128];
    if (espnow_sync_format_line(line, sizeof(line))) {
        ble_server_send_notify(line);
        console_print(line);
    }
}

static void espnow_sync_task(void *pvParameters) {
    (void)pvParameters;
    uint32_t loop_counter = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        loop_counter++;

        int64_t now_us = esp_timer_get_time();

        if (s_is_paired) {
            if (now_us - s_last_recv_us > 3500000) {
                ESP_LOGW(TAG, "ESP-NOW link lost (peer silent). Re-entering auto-discovery.");
                portENTER_CRITICAL(&s_mux);
                s_is_paired = false;
                memset(s_peer_mac, 0, 6);
                portEXIT_CRITICAL(&s_mux);
                notify_link_state();
            } else {
                if (s_local_role == ESPNOW_ROLE_RECEIVER) {
                    int s1 = 90, s2 = 90, s3 = 90, s4 = 90;
                    int o1 = 0,  o2 = 0,  o3 = 0,  o4 = 0;
                    int smin = 0, smax = 180;
                    bool e1 = false, e2 = false, e3 = false, e4 = false;

                    servo_get_all_angles(&s1, &s2, &s3, &s4);
                    servo_get_all_offsets(&o1, &o2, &o3, &o4);
                    servo_get_all_enabled(&e1, &e2, &e3, &e4);
                    servo_get_scissor_range(&smin, &smax);

                    uint8_t en_mask = (e1 ? 1 : 0) | (e2 ? 2 : 0) | (e3 ? 4 : 0) | (e4 ? 8 : 0);
                    uint8_t flags = (servo_get_auto_idle() ? 1 : 0) |
                                    (servo_is_sequence_running() ? 2 : 0) |
                                    (servo_is_sequence_looping() ? 4 : 0);
                    uint8_t slot = (uint8_t)servo_get_active_slot();

                    espnow_sync_send_status((uint8_t)s1, (uint8_t)s2, (uint8_t)s3, (uint8_t)s4,
                                            (int8_t)o1, (int8_t)o2, (int8_t)o3, (int8_t)o4,
                                            (uint8_t)smin, (uint8_t)smax,
                                            en_mask, flags, slot);
                } else {
                    espnow_packet_t hb = {};
                    hb.msg_type = ESPNOW_MSG_HEARTBEAT;
                    hb.payload.heartbeat.uptime_ms = (uint32_t)(now_us / 1000);
                    send_packet(s_peer_mac, &hb);
                }

                if (loop_counter % 4 == 0) {
                    notify_link_state();
                }
            }
        } else {
            espnow_packet_t disc = {};
            disc.msg_type = ESPNOW_MSG_DISCOVERY;
            disc.payload.discovery.proto_version = 1;
            send_packet(BCAST_MAC, &disc);

            if (loop_counter % 6 == 0) {
                notify_link_state();
            }
        }
    }
}

void espnow_sync_init(espnow_role_t role) {
    s_local_role = role;
    s_is_paired = false;
    memset(s_peer_mac, 0, 6);

    s_espnow_rx_queue = xQueueCreate(16, sizeof(espnow_rx_item_t));

    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(ret);
    }
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(ret);
    }

    if (!esp_netif_get_handle_from_ifkey("WIFI_STA_DEF")) {
        esp_netif_create_default_wifi_sta();
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, s_local_mac));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    add_or_update_peer(BCAST_MAC);

    xTaskCreatePinnedToCore(espnow_rx_task, "espnow_rx_task", 6144, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(espnow_sync_task, "espnow_task", 4096, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "ESP-NOW Auto-Sync initialized on ESP32-C6 (Role=%s, MAC=%02X:%02X:%02X:%02X:%02X:%02X, CH=%d)",
             role == ESPNOW_ROLE_SENDER ? "MPU-Sender" : "Arm-Receiver",
             s_local_mac[0], s_local_mac[1], s_local_mac[2], s_local_mac[3], s_local_mac[4], s_local_mac[5],
             ESPNOW_CHANNEL);
}

bool espnow_sync_is_paired(void) {
    return s_is_paired;
}

void espnow_sync_get_peer_mac(uint8_t mac[6]) {
    portENTER_CRITICAL(&s_mux);
    memcpy(mac, s_peer_mac, 6);
    portEXIT_CRITICAL(&s_mux);
}

int8_t espnow_sync_get_last_rssi(void) {
    return s_last_rssi;
}

espnow_role_t espnow_sync_get_role(void) {
    return s_local_role;
}

bool espnow_sync_send_servos(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, uint16_t move_ms, bool mpu_active) {
    if (!s_is_paired) return false;

    espnow_packet_t pkt = {};
    pkt.msg_type = ESPNOW_MSG_SERVO_SET;
    pkt.payload.servo_set.s1 = s1;
    pkt.payload.servo_set.s2 = s2;
    pkt.payload.servo_set.s3 = s3;
    pkt.payload.servo_set.s4 = s4;
    pkt.payload.servo_set.move_ms = move_ms;
    pkt.payload.servo_set.flags = mpu_active ? 1 : 0;

    return send_packet(s_peer_mac, &pkt);
}

bool espnow_sync_send_cmd(const char *cmd_str) {
    if (!s_is_paired || !cmd_str) return false;

    espnow_packet_t pkt = {};
    pkt.msg_type = ESPNOW_MSG_CMD_STR;
    strncpy(pkt.payload.cmd_str.cmd, cmd_str, ESPNOW_PAYLOAD_CMD_SZ - 1);
    pkt.payload.cmd_str.cmd[ESPNOW_PAYLOAD_CMD_SZ - 1] = '\0';

    return send_packet(s_peer_mac, &pkt);
}

bool espnow_sync_send_reply(const char *reply_str) {
    if (!s_is_paired || !reply_str) return false;

    espnow_packet_t pkt = {};
    pkt.msg_type = ESPNOW_MSG_CMD_REPLY;
    strncpy(pkt.payload.cmd_reply.reply, reply_str, ESPNOW_PAYLOAD_CMD_SZ - 1);
    pkt.payload.cmd_reply.reply[ESPNOW_PAYLOAD_CMD_SZ - 1] = '\0';

    return send_packet(s_peer_mac, &pkt);
}

bool espnow_sync_send_status(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4,
                             int8_t o1, int8_t o2, int8_t o3, int8_t o4,
                             uint8_t smin, uint8_t smax,
                             uint8_t en_mask, uint8_t flags, uint8_t active_slot) {
    if (!s_is_paired) return false;

    espnow_packet_t pkt = {};
    pkt.msg_type = ESPNOW_MSG_STATUS;
    pkt.payload.status.s1 = s1;
    pkt.payload.status.s2 = s2;
    pkt.payload.status.s3 = s3;
    pkt.payload.status.s4 = s4;
    pkt.payload.status.o1 = o1;
    pkt.payload.status.o2 = o2;
    pkt.payload.status.o3 = o3;
    pkt.payload.status.o4 = o4;
    pkt.payload.status.smin = smin;
    pkt.payload.status.smax = smax;
    pkt.payload.status.en_mask = en_mask;
    pkt.payload.status.flags = flags;
    pkt.payload.status.active_slot = active_slot;

    return send_packet(s_peer_mac, &pkt);
}

bool espnow_sync_send_imu_line(const char *imu_str) {
    if (!s_is_paired || !imu_str) return false;

    espnow_packet_t pkt = {};
    pkt.msg_type = ESPNOW_MSG_IMU_LINE;
    strncpy(pkt.payload.imu_line.line, imu_str, ESPNOW_PAYLOAD_CMD_SZ - 1);
    pkt.payload.imu_line.line[ESPNOW_PAYLOAD_CMD_SZ - 1] = '\0';

    return send_packet(s_peer_mac, &pkt);
}

bool espnow_sync_get_cached_arm_status(int *s1, int *s2, int *s3, int *s4,
                                       int *o1, int *o2, int *o3, int *o4,
                                       int *smin, int *smax,
                                       bool *e1, bool *e2, bool *e3, bool *e4,
                                       bool *idle, bool *run, bool *loop, int *slot) {
    if (!s_has_cached_status) return false;

    portENTER_CRITICAL(&s_mux);
    if (s1) *s1 = s_cached_s1;
    if (s2) *s2 = s_cached_s2;
    if (s3) *s3 = s_cached_s3;
    if (s4) *s4 = s_cached_s4;
    if (o1) *o1 = s_cached_o1;
    if (o2) *o2 = s_cached_o2;
    if (o3) *o3 = s_cached_o3;
    if (o4) *o4 = s_cached_o4;
    if (smin) *smin = s_cached_smin;
    if (smax) *smax = s_cached_smax;
    if (e1) *e1 = s_cached_e1;
    if (e2) *e2 = s_cached_e2;
    if (e3) *e3 = s_cached_e3;
    if (e4) *e4 = s_cached_e4;
    if (idle) *idle = s_cached_idle;
    if (run)  *run  = s_cached_run;
    if (loop) *loop = s_cached_loop;
    if (slot) *slot = s_cached_slot;
    portEXIT_CRITICAL(&s_mux);

    return true;
}

void espnow_sync_register_cmd_handler(espnow_cmd_handler_t handler) {
    s_cmd_handler = handler;
}

bool espnow_sync_format_line(char *buf, size_t max_len) {
    if (!buf || max_len < 64) return false;

    char mac_str[18] = "00:00:00:00:00:00";
    if (s_is_paired) {
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 s_peer_mac[0], s_peer_mac[1], s_peer_mac[2], s_peer_mac[3], s_peer_mac[4], s_peer_mac[5]);
    }

    snprintf(buf, max_len, "ESPNOW:paired=%d,peer=%s,rssi=%d,role=%s\n",
             s_is_paired ? 1 : 0, mac_str, (int)s_last_rssi,
             s_local_role == ESPNOW_ROLE_SENDER ? "SENDER" : "RECEIVER");
    return true;
}