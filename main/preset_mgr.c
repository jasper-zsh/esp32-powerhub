#include "preset_mgr.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"

#include "storage.h"
#include "command_codec.h"

static const char *TAG = "preset";

esp_err_t preset_mgr_init(void) {
    return ESP_OK;
}

typedef struct {
    preset_entry_cb_t cb;
    void *ctx;
} preset_entry_adapter_t;

static esp_err_t preset_entry_adapter(uint8_t preset_id, const uint8_t *blob, size_t len, void *ctx) {
    (void)preset_id;
    preset_entry_adapter_t *adapter = (preset_entry_adapter_t *)ctx;
    return adapter->cb(blob, len, adapter->ctx);
}

esp_err_t preset_mgr_for_each_entry(preset_entry_cb_t cb, void *ctx) {
    if (!cb) {
        return ESP_ERR_INVALID_ARG;
    }
    preset_entry_adapter_t adapter = {
        .cb = cb,
        .ctx = ctx,
    };
    return storage_for_each_preset(preset_entry_adapter, &adapter);
}

esp_err_t preset_mgr_store(const uint8_t *preset_blob, size_t len) {
    if (!preset_blob || len < 2) {
        return ESP_ERR_INVALID_SIZE;
    }
    uint8_t preset_id = preset_blob[0];
    uint8_t cmd_count = preset_blob[1];

    if (preset_id == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t offset = 2;
    for (uint8_t i = 0; i < cmd_count; ++i) {
        control_cmd_t cmd = {0};
        size_t consumed = 0;
        esp_err_t err = control_cmd_parse(preset_blob + offset, len - offset, &consumed, &cmd);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Preset %u command %u invalid err=%d", preset_id, i, err);
            return err;
        }
        err = control_cmd_validate(&cmd);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Preset %u command %u validation err=%d", preset_id, i, err);
            return err;
        }
        offset += consumed;
    }

    if (cmd_count == 0) {
        // No commands means delete
        if (len != 2) {
            return ESP_ERR_INVALID_SIZE;
        }
        return storage_delete_preset(preset_id);
    }

    if (offset != len) {
        // Extra bytes trailing
        return ESP_ERR_INVALID_SIZE;
    }

    return storage_save_preset(preset_id, preset_blob, len);
}

static esp_err_t iterate_commands_from_blob(const uint8_t *blob, size_t len, preset_cmd_cb_t cb, void *ctx) {
    if (!blob || len < 2) {
        return ESP_ERR_INVALID_SIZE;
    }
    uint8_t preset_id = blob[0];
    uint8_t cmd_count = blob[1];
    size_t offset = 2;
    for (uint8_t i = 0; i < cmd_count; ++i) {
        control_cmd_t cmd = {0};
        size_t consumed = 0;
        esp_err_t err = control_cmd_parse(blob + offset, len - offset, &consumed, &cmd);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Preset %u command parse failed idx=%u err=%d", preset_id, i, err);
            return err;
        }
        err = control_cmd_validate(&cmd);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Preset %u command validate failed idx=%u err=%d", preset_id, i, err);
            return err;
        }
        err = cb(&cmd, ctx);
        if (err != ESP_OK) {
            return err;
        }
        offset += consumed;
    }
    if (offset != len) {
        ESP_LOGW(TAG, "Preset %u len mismatch (offset=%zu len=%zu)", preset_id, offset, len);
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

esp_err_t preset_mgr_iterate_commands(uint8_t preset_id, preset_cmd_cb_t cb, void *ctx) {
    if (!cb) {
        return ESP_ERR_INVALID_ARG;
    }
    if (preset_id == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t len = 0;
    esp_err_t err = storage_load_preset(preset_id, NULL, &len);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t *buf = malloc(len);
    if (!buf) {
        return ESP_ERR_NO_MEM;
    }
    err = storage_load_preset(preset_id, buf, &len);
    if (err == ESP_OK) {
        err = iterate_commands_from_blob(buf, len, cb, ctx);
    }
    free(buf);
    return err;
}
