#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "hardware_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STORAGE_NAMESPACE "powerhub"
#define STORAGE_KEY_STATES "ch_state"
// Read channel states from NVS (length = PWM_CHANNEL_COUNT). If not found, fills zeros.
esp_err_t storage_read_states(uint8_t out_states[PWM_CHANNEL_COUNT]);

// Write channel states to NVS (length = PWM_CHANNEL_COUNT).
esp_err_t storage_write_states(const uint8_t states[PWM_CHANNEL_COUNT]);

#ifdef __cplusplus
}
#endif
