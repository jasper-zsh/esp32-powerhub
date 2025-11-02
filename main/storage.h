#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STORAGE_NAMESPACE "powerhub"
#define STORAGE_KEY_STATES "ch_state"
// Read 6-byte states from NVS. If not found, fills zeros.
esp_err_t storage_read_states(uint8_t out_states[6]);

// Write 6-byte states to NVS.
esp_err_t storage_write_states(const uint8_t states[6]);

#ifdef __cplusplus
}
#endif
