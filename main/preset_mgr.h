#pragma once

#include <stddef.h>
#include "esp_err.h"

#include "control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t preset_mgr_init(void);

typedef esp_err_t (*preset_entry_cb_t)(const uint8_t *entry, size_t len, void *ctx);
esp_err_t preset_mgr_for_each_entry(preset_entry_cb_t cb, void *ctx);

esp_err_t preset_mgr_store(const uint8_t *preset_blob, size_t len);

typedef esp_err_t (*preset_cmd_cb_t)(const control_cmd_t *cmd, void *ctx);
esp_err_t preset_mgr_iterate_commands(uint8_t preset_id, preset_cmd_cb_t cb, void *ctx);

#ifdef __cplusplus
}
#endif

