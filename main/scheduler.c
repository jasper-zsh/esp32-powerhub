#include "scheduler.h"

#include "esp_log.h"

#include "freertos/queue.h"
#include "freertos/task.h"

#include "pwm_control.h"
#include "storage.h"
#include "command_codec.h"
#include "nvs.h"
#include "ble_server.h"

static const char *TAG = "sched";

// 通道运行态快照
static control_cmd_t s_channel_snapshots[PWM_CHANNEL_COUNT];

#define SCHED_QUEUE_LENGTH 16
typedef enum {
    SCHED_EVENT_CMD = 0,
} scheduler_event_type_t;

typedef struct {
    scheduler_event_type_t type;
    union {
        control_cmd_t cmd;
    } data;
} scheduler_event_t;

static QueueHandle_t s_evt_queue = NULL;
static TaskHandle_t s_sched_task = NULL;

static esp_err_t persist_states(void) {
    uint8_t states[PWM_CHANNEL_COUNT] = {0};
    pwm_control_get_states(states);
    esp_err_t err = storage_write_states(states);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Persist states failed err=%d", err);
    }
    return err;
}

static esp_err_t handle_command_internal(const control_cmd_t *cmd) {
    if (!cmd) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t validate_rc = control_cmd_validate(cmd);
    if (validate_rc != ESP_OK) {
        return validate_rc;
    }
    switch (cmd->mode) {
        case 0x00: { // set value
            if (cmd->length != 1) return ESP_ERR_INVALID_SIZE;
            uint8_t value = cmd->payload[0];
            esp_err_t err = pwm_control_apply(cmd->channel, value, 0);
            if (err == ESP_OK) {
                persist_states();
                // 保存快照
                s_channel_snapshots[cmd->channel] = *cmd;
            }
            return err;
        }
        case 0x01: { // fade
            if (cmd->length != 3) return ESP_ERR_INVALID_SIZE;
            uint8_t value = cmd->payload[0];
            uint16_t duration = ((uint16_t)cmd->payload[1] << 8) | cmd->payload[2];
            esp_err_t err = pwm_control_apply(cmd->channel, value, duration);
            if (err == ESP_OK) {
                persist_states();
                // 保存快照
                s_channel_snapshots[cmd->channel] = *cmd;
            }
            return err;
        }
        case 0x02: { // blink
            if (cmd->length != 2) return ESP_ERR_INVALID_SIZE;
            uint16_t period = ((uint16_t)cmd->payload[0] << 8) | cmd->payload[1];
            esp_err_t err = pwm_control_start_blink(cmd->channel, period);
            if (err == ESP_OK) {
                persist_states();
                // 保存快照
                s_channel_snapshots[cmd->channel] = *cmd;
            }
            return err;
        }
        case 0x03: { // strobe
            if (cmd->length != 5) return ESP_ERR_INVALID_SIZE;
            uint8_t count = cmd->payload[0];
            uint16_t total = ((uint16_t)cmd->payload[1] << 8) | cmd->payload[2];
            uint16_t pause = ((uint16_t)cmd->payload[3] << 8) | cmd->payload[4];
            esp_err_t err = pwm_control_start_strobe(cmd->channel, count, total, pause);
            if (err == ESP_OK) {
                persist_states();
                // 保存快照
                s_channel_snapshots[cmd->channel] = *cmd;
            }
            return err;
        }
        default:
            ESP_LOGW(TAG, "Unknown mode=0x%02X", cmd->mode);
            return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t stop_all_channels(void) {
    esp_err_t first_err = ESP_OK;
    for (uint8_t ch = 0; ch < PWM_CHANNEL_COUNT; ++ch) {
        esp_err_t err = pwm_control_apply(ch, 0, 0);
        if (first_err == ESP_OK && err != ESP_OK) {
            first_err = err;
        }
    }
    persist_states();

    // Check and send BLE notifications if channel states changed
    ble_server_check_state_notifications();

    return first_err;
}


static void scheduler_task(void *param) {
    scheduler_event_t evt;
    while (true) {
        if (xQueueReceive(s_evt_queue, &evt, portMAX_DELAY) == pdTRUE) {
            switch (evt.type) {
                case SCHED_EVENT_CMD: {
                    if (evt.data.cmd.channel >= PWM_CHANNEL_COUNT) {
                        ESP_LOGW(TAG, "Invalid channel %u for mode 0x%02X", evt.data.cmd.channel, evt.data.cmd.mode);
                        break;
                    }
                    esp_err_t err = handle_command_internal(&evt.data.cmd);
                    if (err == ESP_OK) {
                        // Check and send BLE notifications if channel states changed
                        ble_server_check_state_notifications();
                    } else {
                        ESP_LOGW(TAG, "Command mode=0x%02X failed err=%d", evt.data.cmd.mode, err);
                    }
                    break;
                }
                default:
                    ESP_LOGW(TAG, "Unknown scheduler event type=%d", evt.type);
                    break;
            }
        }
    }
}

esp_err_t scheduler_init(void) {
    if (s_evt_queue) {
        return ESP_OK;
    }
    s_evt_queue = xQueueCreate(SCHED_QUEUE_LENGTH, sizeof(scheduler_event_t));
    if (!s_evt_queue) {
        return ESP_ERR_NO_MEM;
    }
    BaseType_t rc = xTaskCreatePinnedToCore(scheduler_task, "sched", 4096, NULL, 5, &s_sched_task, tskNO_AFFINITY);
    if (rc != pdPASS) {
        vQueueDelete(s_evt_queue);
        s_evt_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t scheduler_submit_command(const control_cmd_t *cmd, TickType_t ticks_to_wait) {
    if (!cmd || !s_evt_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    scheduler_event_t evt = {
        .type = SCHED_EVENT_CMD,
        .data.cmd = *cmd,
    };
    BaseType_t rc = xQueueSend(s_evt_queue, &evt, ticks_to_wait);
    if (rc != pdPASS) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}


esp_err_t scheduler_get_channel_snapshot(uint8_t ch, control_cmd_t *out_cmd) {
    if (ch >= PWM_CHANNEL_COUNT || out_cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *out_cmd = s_channel_snapshots[ch];
    return ESP_OK;
}

esp_err_t scheduler_restore_channel_snapshot(uint8_t ch, const control_cmd_t *cmd) {
    if (ch >= PWM_CHANNEL_COUNT || cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 直接执行命令恢复状态
    esp_err_t err = handle_command_internal(cmd);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Restored channel %u to mode 0x%02x", ch, cmd->mode);
    } else {
        ESP_LOGW(TAG, "Failed to restore channel %u: %s", ch, esp_err_to_name(err));
    }
    return err;
}
