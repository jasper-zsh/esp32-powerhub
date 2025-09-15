#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STORAGE_NAMESPACE "pwmhub"
#define STORAGE_KEY_STATES "states"

// Read 4-byte states from NVS. If not found, fills zeros.
esp_err_t storage_read_states(uint8_t out_states[4]);

// Write 4-byte states to NVS.
esp_err_t storage_write_states(const uint8_t states[4]);

#ifdef __cplusplus
}
#endif

