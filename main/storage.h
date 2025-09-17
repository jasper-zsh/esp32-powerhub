#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STORAGE_NAMESPACE "powerhub"
#define STORAGE_KEY_STATES "ch_state"
#define STORAGE_KEY_PRESET_PREFIX "preset_"

// Read 4-byte states from NVS. If not found, fills zeros.
esp_err_t storage_read_states(uint8_t out_states[4]);

// Write 4-byte states to NVS.
esp_err_t storage_write_states(const uint8_t states[4]);

// Save a preset blob under preset_<id> key.
esp_err_t storage_save_preset(uint8_t preset_id, const uint8_t *data, size_t len);

// Load preset blob for preset_<id> into caller-provided buffer.
esp_err_t storage_load_preset(uint8_t preset_id, uint8_t *data, size_t *len);

// Delete preset_<id> entry (ignored if not present).
esp_err_t storage_delete_preset(uint8_t preset_id);

typedef esp_err_t (*storage_preset_iter_cb)(uint8_t preset_id, const uint8_t *blob, size_t len, void *ctx);

// Iterate over all stored presets; callback controls error handling/early exit.
esp_err_t storage_for_each_preset(storage_preset_iter_cb cb, void *ctx);

#ifdef __cplusplus
}
#endif
