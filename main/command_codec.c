#include "command_codec.h"

#include <string.h>

#include "hardware_defs.h"

#define CONTROL_MAX_CHANNEL PWM_CHANNEL_COUNT

static esp_err_t expected_payload_len(uint8_t mode, size_t *out_len) {
    if (!out_len) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t len = 0;
    switch (mode) {
        case 0x00: len = 1; break; // set
        case 0x01: len = 3; break; // fade (value + duration)
        case 0x02: len = 2; break; // blink period
        case 0x03: len = 5; break; // strobe (count + total + pause)
        default:
            return ESP_ERR_INVALID_ARG;
    }
    *out_len = len;
    return ESP_OK;
}

esp_err_t control_cmd_parse(const uint8_t *buf, size_t len, size_t *consumed, control_cmd_t *out_cmd) {
    if (!buf || !consumed || !out_cmd) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len < 2) {
        return ESP_ERR_INVALID_SIZE;
    }
    out_cmd->mode = buf[0];
    out_cmd->channel = buf[1];
    size_t payload_len = 0;
    esp_err_t err = expected_payload_len(out_cmd->mode, &payload_len);
    if (err != ESP_OK) {
        return err;
    }
    if ((2 + payload_len) > len || payload_len > CONTROL_CMD_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (payload_len > 0) {
        memcpy(out_cmd->payload, buf + 2, payload_len);
    }
    out_cmd->length = (uint8_t)payload_len;
    *consumed = 2 + payload_len;
    return ESP_OK;
}

esp_err_t control_cmd_validate(const control_cmd_t *cmd) {
    if (!cmd) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t payload_len = 0;
    esp_err_t err = expected_payload_len(cmd->mode, &payload_len);
    if (err != ESP_OK) {
        return err;
    }
    if (cmd->length != payload_len) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (cmd->channel >= CONTROL_MAX_CHANNEL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cmd->length > CONTROL_CMD_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

size_t control_cmd_serialized_size(const control_cmd_t *cmd) {
    if (!cmd) {
        return 0;
    }
    return 2 + cmd->length;
}
