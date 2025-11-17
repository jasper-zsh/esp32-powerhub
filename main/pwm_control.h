// PWM and GPIO control for 6 channels on ESP32-C6
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#include "hardware_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

// Fixed mapping per latest hardware design: CH1..CH6 -> GPIO8/9/10/11/12/13
typedef enum {
    CH1 = 0,
    CH2 = 1,
    CH3 = 2,
    CH4 = 3,
    CH5 = 4,
    CH6 = 5,
} pwm_channel_t;

// Initialize LEDC timer + channels and configure GPIOs.
esp_err_t pwm_control_init(void);

// Apply a new target value (0..255) to channel with optional fade time in ms.
// Semantics per design:
//   0   => High-impedance (GPIO input, LEDC detached)
//   255 => Logic High (constant high level)
//   1..254 => PWM with duty = value/255
esp_err_t pwm_control_apply(uint8_t channel, uint8_t value, uint16_t duration_ms);

// Get current target states (the values we expose via BLE and persist).
void pwm_control_get_states(uint8_t out[PWM_CHANNEL_COUNT]);

// Load initial states and drive outputs accordingly; if states is NULL, leaves outputs unchanged.
esp_err_t pwm_control_load_and_apply(const uint8_t *states);

// Stop any running pattern (blink/strobe) on a channel.
esp_err_t pwm_control_stop_pattern(uint8_t channel);

// Start equal-speed blink: toggles between 0 and last non-zero value.
// period_ms is the total on+off cycle period.
esp_err_t pwm_control_start_blink(uint8_t channel, uint16_t period_ms);

// Start strobe burst: flashes 'count' times within total duration dur_ms (50% duty per pulse).
// After a burst, if pause_ms > 0, stays off for pause_ms then repeats; if pause_ms == 0, stops and turns off.
esp_err_t pwm_control_start_strobe(uint8_t channel, uint8_t count, uint16_t dur_ms, uint16_t pause_ms);

#ifdef __cplusplus
}
#endif
