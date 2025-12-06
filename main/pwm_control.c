#include "pwm_control.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "storage.h"

static const char *TAG = "pwm";

// LEDC channels mapping helper
static inline ledc_channel_t ledc_channel_for_idx(int idx) {
    return (ledc_channel_t)(LEDC_CHANNEL_0 + idx);
}

// ESP32-S3 exposes PWM via low speed group only; keep 5 kHz timer per design
static const ledc_mode_t s_mode = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t s_timer = LEDC_TIMER_0;
static const ledc_timer_bit_t s_res = LEDC_TIMER_13_BIT; // higher resolution for smoother fades
static uint32_t s_duty_max = 0; // (1<<res) - 1

// Keep current target states
static uint8_t s_states[PWM_CHANNEL_COUNT] = {0};

// Timers for deferred GPIO mode changes after fade
static TimerHandle_t s_defer_timers[PWM_CHANNEL_COUNT] = {0};

// Pattern control (blink/strobe)
typedef enum {
    PATTERN_NONE = 0,
    PATTERN_BLINK,
    PATTERN_STROBE,
} pattern_type_t;

typedef struct {
    pattern_type_t type;
    TimerHandle_t timer;       // periodic toggle or scheduler
    uint16_t period_ms;        // for blink
    uint16_t pulse_dur_ms;     // for strobe: ON and OFF duration per pulse
    uint16_t pause_ms;         // for strobe
    uint8_t on_value;          // last non-zero value used when ON
    uint8_t remaining_on_pulses; // for strobe: remaining ON pulses in current burst
    uint8_t initial_pulses;    // for strobe: pulses per burst
    bool phase_on;             // current phase: true=ON phase, false=OFF phase
    bool in_pause;             // strobe: currently in pause window between bursts
} pattern_state_t;

static pattern_state_t s_pattern[PWM_CHANNEL_COUNT] = {0};

// Forward declarations for helpers used in timer callback
static void set_gpio_low(int idx);
static void set_gpio_high(int idx);
// Forward declaration: internal apply that can skip canceling pattern
static esp_err_t apply_value_internal(uint8_t channel, uint8_t value, uint16_t duration_ms, bool cancel_pattern, bool silent);

static esp_err_t restart_defer_timer(uint8_t channel, uint32_t delay_ms);

static void pwm_defer_timer_cb(TimerHandle_t xTimer) {
    int ch = (int)(intptr_t) pvTimerGetTimerID(xTimer);
    if (ch < 0 || ch >= PWM_CHANNEL_COUNT) return;
    uint8_t v = s_states[ch];
    if (v == 0) {
        set_gpio_low(ch);
    } else if (v == 255) {
        set_gpio_high(ch);
    }
}

static void pattern_timer_cb(TimerHandle_t xTimer) {
    int ch = (int)(intptr_t) pvTimerGetTimerID(xTimer);
    if (ch < 0 || ch >= PWM_CHANNEL_COUNT) return;
    pattern_state_t *ps = &s_pattern[ch];
    if (ps->type == PATTERN_BLINK) {
        // Toggle between OFF and ON
        ps->phase_on = !ps->phase_on;
        uint8_t target = ps->phase_on ? (ps->on_value ? ps->on_value : 255) : 0;
        // Immediate change, no fade
        apply_value_internal((uint8_t)ch, target, 0, false, true);
        // Keep timer running with half-period; create as periodic already
    } else if (ps->type == PATTERN_STROBE) {
        if (ps->in_pause) {
            // Pause complete, start next burst
            ps->in_pause = false;
            // Reset pulses for new burst
            ps->remaining_on_pulses = ps->initial_pulses;
            ps->phase_on = false; // force transition to ON on next toggle
            xTimerChangePeriod(ps->timer, pdMS_TO_TICKS(ps->pulse_dur_ms ? ps->pulse_dur_ms : 1), 0);
            return;
        }

        // Toggle state
        ps->phase_on = !ps->phase_on;
        if (ps->phase_on) {
            // Enter ON phase: apply ON and decrement pulses
            uint8_t target = ps->on_value ? ps->on_value : 255;
            apply_value_internal((uint8_t)ch, target, 0, false, true);
            if (ps->remaining_on_pulses > 0) {
                ps->remaining_on_pulses--;
            }
        } else {
            // Enter OFF phase
            apply_value_internal((uint8_t)ch, 0, 0, false, true);
            // If completed all pulses in this burst, handle pause or stop
            if (ps->remaining_on_pulses == 0) {
                if (ps->pause_ms == 0) {
                    // Stop pattern and leave OFF per spec when pause_ms==0
                    pwm_control_stop_pattern((uint8_t)ch);
                    return;
                } else {
                    // Enter pause window; stay OFF and schedule next burst after pause_ms
                    ps->in_pause = true;
                    // Use one-shot for pause duration
                    xTimerChangePeriod(ps->timer, pdMS_TO_TICKS(ps->pause_ms), 0);
                    return;
                }
            }
        }
        // Continue toggling with pulse duration for both ON and OFF
        xTimerChangePeriod(ps->timer, pdMS_TO_TICKS(ps->pulse_dur_ms ? ps->pulse_dur_ms : 1), 0);
    }
}

static esp_err_t restart_defer_timer(uint8_t channel, uint32_t delay_ms) {
    if (channel >= PWM_CHANNEL_COUNT || !s_defer_timers[channel]) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xTimerStop(s_defer_timers[channel], 0) != pdPASS) {
        return ESP_FAIL;
    }
    if (xTimerChangePeriod(s_defer_timers[channel], pdMS_TO_TICKS(delay_ms), 0) != pdPASS) {
        return ESP_FAIL;
    }
    if (xTimerStart(s_defer_timers[channel], 0) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

// Helper: attach LEDC to a channel's GPIO if not already
// Helper: drive channel using LEDC duty only
static void set_gpio_low(int idx) {
    ledc_channel_t ch = ledc_channel_for_idx(idx);
    (void) ledc_set_duty(s_mode, ch, 0);
    (void) ledc_update_duty(s_mode, ch);
}

static void set_gpio_high(int idx) {
    ledc_channel_t ch = ledc_channel_for_idx(idx);
    (void) ledc_set_duty(s_mode, ch, s_duty_max);
    (void) ledc_update_duty(s_mode, ch);
}

esp_err_t pwm_control_init(void) {
    // LEDC timer: 13-bit resolution, 5 kHz per design baseline
    ledc_timer_config_t t = {
        .speed_mode = s_mode,
        .duty_resolution = s_res,
        .timer_num = s_timer,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&t);
    if (err != ESP_OK) {
        return err;
    }

    s_duty_max = (1U << s_res) - 1U;
    ESP_LOGI(TAG, "LEDC init: freq=%uHz res_bits=%d duty_max=%u", (unsigned)t.freq_hz, (int)s_res, (unsigned)s_duty_max);

    err = ledc_fade_func_install(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    // Initialize all channels with default values first
    for (int i = 0; i < PWM_CHANNEL_COUNT; ++i) {
        s_states[i] = 0;
        ledc_channel_t ledc_ch = ledc_channel_for_idx(i);
        ledc_channel_config_t ch = {
            .gpio_num = PWM_CHANNEL_GPIOS[i],
            .speed_mode = s_mode,
            .channel = ledc_ch,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = s_timer,
            .duty = 0,
            .hpoint = 0,
            .flags = { .output_invert = 0 },
        };
        err = ledc_channel_config(&ch);
        if (err != ESP_OK) {
            return err;
        }
        set_gpio_low(i);
        s_defer_timers[i] = xTimerCreate("pwmDefer", pdMS_TO_TICKS(10), pdFALSE, (void *)(intptr_t)i, pwm_defer_timer_cb);
        if (!s_defer_timers[i]) {
            return ESP_ERR_NO_MEM;
        }
        s_pattern[i].type = PATTERN_NONE;
        s_pattern[i].timer = xTimerCreate("pwmPat", pdMS_TO_TICKS(100), pdTRUE, (void *)(intptr_t)i, pattern_timer_cb);
        if (!s_pattern[i].timer) {
            return ESP_ERR_NO_MEM;
        }
        s_pattern[i].on_value = 255;
        s_pattern[i].in_pause = false;
        s_pattern[i].phase_on = false;
    }

    // Load and apply saved channel states from storage
    uint8_t saved_states[PWM_CHANNEL_COUNT];
    esp_err_t storage_err = storage_read_states(saved_states);
    if (storage_err == ESP_OK) {
        bool has_non_zero_states = false;
        for (int i = 0; i < PWM_CHANNEL_COUNT; ++i) {
            if (saved_states[i] != 0) {
                has_non_zero_states = true;
                break;
            }
        }

        if (has_non_zero_states) {
            ESP_LOGI(TAG, "Restoring %d saved channel states", PWM_CHANNEL_COUNT);
            err = pwm_control_load_and_apply(saved_states);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to apply saved channel states: %s", esp_err_to_name(err));
                // Don't return error - continue with default states
            }
        } else {
            ESP_LOGI(TAG, "No saved channel states found, using defaults");
        }
    } else {
        ESP_LOGW(TAG, "Failed to read saved channel states: %s", esp_err_to_name(storage_err));
    }

    return ESP_OK;
}

static esp_err_t apply_value_internal(uint8_t channel, uint8_t value, uint16_t duration_ms, bool cancel_pattern, bool silent) {
    if (channel >= PWM_CHANNEL_COUNT) return ESP_ERR_INVALID_ARG;
    if (cancel_pattern) {
        pwm_control_stop_pattern(channel);
    }
    s_states[channel] = value;
    ledc_channel_t ledc_ch = ledc_channel_for_idx(channel);
    if (!silent) {
        ESP_LOGI(TAG, "apply ch=%u val=%u dur_ms=%u (cancel=%d)", channel, value, (unsigned)duration_ms, (int)cancel_pattern);
    }

    // Map 0..255 to high-resolution duty 0..s_duty_max
    uint32_t duty_target = (uint32_t)value * s_duty_max / 255U;

    // 0 -> drive low (0% duty)
    if (value == 0) {
        if (duration_ms > 0) {
            esp_err_t err = ledc_set_fade_with_time(s_mode, ledc_ch, 0, duration_ms);
            if (err != ESP_OK) return err;
            err = ledc_fade_start(s_mode, ledc_ch, LEDC_FADE_NO_WAIT);
            if (err != ESP_OK) return err;
            uint32_t delay_ms = duration_ms + 5U;
            err = restart_defer_timer(channel, delay_ms);
            if (err != ESP_OK) return err;
            if (!silent) ESP_LOGI(TAG, "ch=%u fade->0 for %u ms, then drive low", channel, (unsigned)duration_ms);
        } else {
            set_gpio_low(channel);
            if (!silent) ESP_LOGI(TAG, "ch=%u set low immediately", channel);
        }
        return ESP_OK;
    }

    // 255 -> constant high
    if (value == 255) {
        if (duration_ms > 0) {
            esp_err_t err = ledc_set_fade_with_time(s_mode, ledc_ch, s_duty_max, duration_ms);
            if (err != ESP_OK) return err;
            err = ledc_fade_start(s_mode, ledc_ch, LEDC_FADE_NO_WAIT);
            if (err != ESP_OK) return err;
            uint32_t delay_ms = duration_ms + 5U;
            err = restart_defer_timer(channel, delay_ms);
            if (err != ESP_OK) return err;
            if (!silent) ESP_LOGI(TAG, "ch=%u fade->255 for %u ms", channel, (unsigned)duration_ms);
        } else {
            set_gpio_high(channel);
            if (!silent) ESP_LOGI(TAG, "ch=%u set 100%% duty immediately", channel);
        }
        return ESP_OK;
    }

    // 1..254 -> PWM
    esp_err_t err;
    if (duration_ms > 0) {
        err = ledc_set_fade_with_time(s_mode, ledc_ch, duty_target, duration_ms);
        if (err != ESP_OK) return err;
        err = ledc_fade_start(s_mode, ledc_ch, LEDC_FADE_NO_WAIT);
        if (err != ESP_OK) return err;
        if (!silent) ESP_LOGI(TAG, "ch=%u PWM fade to val=%u duty=%u for %u ms", channel, value, (unsigned)duty_target, (unsigned)duration_ms);
    } else {
        err = ledc_set_duty(s_mode, ledc_ch, duty_target);
        if (err != ESP_OK) return err;
        err = ledc_update_duty(s_mode, ledc_ch);
        if (err != ESP_OK) return err;
        if (!silent) ESP_LOGI(TAG, "ch=%u PWM set val=%u duty=%u immediately", channel, value, (unsigned)duty_target);
    }
    return ESP_OK;
}

static inline esp_err_t apply_value(uint8_t channel, uint8_t value, uint16_t duration_ms, bool cancel_pattern) {
    return apply_value_internal(channel, value, duration_ms, cancel_pattern, false);
}

esp_err_t pwm_control_apply(uint8_t channel, uint8_t value, uint16_t duration_ms) {
    return apply_value_internal(channel, value, duration_ms, true, false);
}

void pwm_control_get_states(uint8_t out[PWM_CHANNEL_COUNT]) {
    for (int i = 0; i < PWM_CHANNEL_COUNT; ++i) out[i] = s_states[i];
}

esp_err_t pwm_control_load_and_apply(const uint8_t *states) {
    if (!states) return ESP_OK;
    for (int i = 0; i < PWM_CHANNEL_COUNT; ++i) {
        (void) pwm_control_apply(i, states[i], 0);
    }
    return ESP_OK;
}

esp_err_t pwm_control_stop_pattern(uint8_t channel) {
    if (channel >= PWM_CHANNEL_COUNT) return ESP_ERR_INVALID_ARG;
    pattern_state_t *ps = &s_pattern[channel];
    if (ps->timer) {
        xTimerStop(ps->timer, 0);
    }
    ps->type = PATTERN_NONE;
    ps->in_pause = false;
    ps->phase_on = false;
    return ESP_OK;
}

esp_err_t pwm_control_start_blink(uint8_t channel, uint16_t period_ms) {
    if (channel >= PWM_CHANNEL_COUNT || period_ms == 0) return ESP_ERR_INVALID_ARG;
    pattern_state_t *ps = &s_pattern[channel];
    pwm_control_stop_pattern(channel);
    // Remember last non-zero as ON value; if current is zero, default to 255
    uint8_t cur = s_states[channel];
    if (cur != 0) ps->on_value = cur;
    ps->type = PATTERN_BLINK;
    ps->period_ms = period_ms;
    ps->phase_on = false; // start from OFF -> next tick goes ON
    // Immediate off to start
    apply_value_internal(channel, 0, 0, false, true);
    // Start periodic toggle every half-period
    TickType_t half = pdMS_TO_TICKS(period_ms / 2 ? period_ms / 2 : 1);
    xTimerChangePeriod(ps->timer, half, 0);
    xTimerStart(ps->timer, 0);
    return ESP_OK;
}

esp_err_t pwm_control_start_strobe(uint8_t channel, uint8_t count, uint16_t total_ms, uint16_t pause_ms) {
    if (channel >= PWM_CHANNEL_COUNT || total_ms == 0 || count == 0) return ESP_ERR_INVALID_ARG;
    pattern_state_t *ps = &s_pattern[channel];
    pwm_control_stop_pattern(channel);
    uint8_t cur = s_states[channel];
    if (cur != 0) ps->on_value = cur;
    ps->type = PATTERN_STROBE;
    ps->pause_ms = pause_ms;
    ps->remaining_on_pulses = count;
    ps->initial_pulses = count;
    ps->in_pause = false;
    ps->phase_on = false; // start from OFF
    // Compute period = total_ms / count, ON and OFF duration = period/2 each
    uint32_t period = total_ms / count;
    if (period == 0) period = 1; // ensure at least 1ms
    uint32_t half = period / 2;
    if (half == 0) half = 1;
    ps->pulse_dur_ms = (uint16_t)half;
    // Set OFF initially
    apply_value_internal(channel, 0, 0, false, true);
    ESP_LOGI(TAG, "strobe start ch=%u count=%u total=%u pause=%u", channel, (unsigned)count, (unsigned)total_ms, (unsigned)pause_ms);
    // Start toggling with pulse duration
    TickType_t tick = pdMS_TO_TICKS(ps->pulse_dur_ms ? ps->pulse_dur_ms : 1);
    xTimerChangePeriod(ps->timer, tick, 0);
    xTimerStart(ps->timer, 0);
    return ESP_OK;
}

esp_err_t pwm_control_shutdown_all(void) {
    esp_err_t first_err = ESP_OK;

    ESP_LOGI(TAG, "Shutting down all PWM channels for deep sleep");

    // Stop all patterns and set all channels to low state
    for (int i = 0; i < PWM_CHANNEL_COUNT; ++i) {
        // Stop any running pattern on this channel
        esp_err_t pattern_err = pwm_control_stop_pattern(i);
        if (pattern_err != ESP_OK && first_err == ESP_OK) {
            first_err = pattern_err;
        }

        // Set channel to low state (0% duty cycle) immediately
        esp_err_t apply_err = apply_value_internal(i, 0, 0, false, true);
        if (apply_err != ESP_OK && first_err == ESP_OK) {
            first_err = apply_err;
        }

        // Ensure LEDC output is set to 0 duty
        ledc_channel_t ledc_ch = ledc_channel_for_idx(i);
        esp_err_t duty_err = ledc_set_duty(s_mode, ledc_ch, 0);
        if (duty_err != ESP_OK && first_err == ESP_OK) {
            first_err = duty_err;
        }
        duty_err = ledc_update_duty(s_mode, ledc_ch);
        if (duty_err != ESP_OK && first_err == ESP_OK) {
            first_err = duty_err;
        }
    }

    if (first_err != ESP_OK) {
        ESP_LOGW(TAG, "PWM shutdown completed with errors: %s", esp_err_to_name(first_err));
    } else {
        ESP_LOGI(TAG, "All PWM channels shut down successfully");
    }

    // Configure RTC GPIOs for deep sleep
    esp_err_t rtc_err = pwm_control_configure_rtc_gpio_deep_sleep();
    if (rtc_err != ESP_OK && first_err == ESP_OK) {
        first_err = rtc_err;
    }

    return first_err;
}

esp_err_t pwm_control_configure_rtc_gpio_deep_sleep(void) {
    esp_err_t first_err = ESP_OK;

    ESP_LOGI(TAG, "Configuring RTC GPIOs for deep sleep (GPIOs 8-13)");

    // Configure GPIOs 8-13 as RTC GPIOs (PWM channels CH1-CH6)
    // All GPIOs 8-13 are RTC-capable on ESP32-S3

    const int rtc_gpios[] = {8, 9, 10, 11, 12, 13};
    const int rtc_gpio_count = sizeof(rtc_gpios) / sizeof(rtc_gpios[0]);

    for (int i = 0; i < rtc_gpio_count; i++) {
        int gpio_num = rtc_gpios[i];

        ESP_LOGI(TAG, "Configuring GPIO %d as RTC GPIO for deep sleep", gpio_num);

        // Configure GPIO as RTC GPIO, output low
        esp_err_t rtc_gpio_err = rtc_gpio_init(gpio_num);
        if (rtc_gpio_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to init RTC GPIO %d: %s", gpio_num, esp_err_to_name(rtc_gpio_err));
            if (first_err == ESP_OK) first_err = rtc_gpio_err;
            continue;
        }

        rtc_gpio_err = rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_OUTPUT_ONLY);
        if (rtc_gpio_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set direction for RTC GPIO %d: %s", gpio_num, esp_err_to_name(rtc_gpio_err));
            if (first_err == ESP_OK) first_err = rtc_gpio_err;
            continue;
        }

        rtc_gpio_err = rtc_gpio_set_level(gpio_num, 0); // Set low
        if (rtc_gpio_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set level for RTC GPIO %d: %s", gpio_num, esp_err_to_name(rtc_gpio_err));
            if (first_err == ESP_OK) first_err = rtc_gpio_err;
            continue;
        }

        // Enable hold to maintain state during deep sleep
        rtc_gpio_err = rtc_gpio_hold_en(gpio_num);
        if (rtc_gpio_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable hold for RTC GPIO %d: %s", gpio_num, esp_err_to_name(rtc_gpio_err));
            if (first_err == ESP_OK) first_err = rtc_gpio_err;
            continue;
        }

        ESP_LOGI(TAG, "RTC GPIO %d configured: output low, hold enabled", gpio_num);
    }

    // Enable force hold for all RTC IOs to ensure state is maintained during deep sleep
    esp_err_t force_hold_err = rtc_gpio_force_hold_en_all();
    if (force_hold_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable force hold for all RTC IOs: %s", esp_err_to_name(force_hold_err));
        if (first_err == ESP_OK) first_err = force_hold_err;
    } else {
        ESP_LOGI(TAG, "Force hold enabled for all RTC IOs");
    }

    if (first_err == ESP_OK) {
        ESP_LOGI(TAG, "RTC GPIO deep sleep configuration completed successfully");
    } else {
        ESP_LOGW(TAG, "RTC GPIO configuration completed with errors");
    }

    return first_err;
}

esp_err_t pwm_control_restore_rtc_gpio_wake(void) {
    esp_err_t first_err = ESP_OK;

    ESP_LOGI(TAG, "Restoring RTC GPIOs from deep sleep");

    // Disable force hold for all RTC IOs first
    esp_err_t force_hold_err = rtc_gpio_force_hold_dis_all();
    if (force_hold_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable force hold: %s", esp_err_to_name(force_hold_err));
        if (first_err == ESP_OK) first_err = force_hold_err;
    } else {
        ESP_LOGI(TAG, "Force hold disabled for all RTC IOs");
    }

    // Restore GPIOs 8-13 from RTC mode to normal operation
    const int rtc_gpios[] = {8, 9, 10, 11, 12, 13};
    const int rtc_gpio_count = sizeof(rtc_gpios) / sizeof(rtc_gpios[0]);

    for (int i = 0; i < rtc_gpio_count; i++) {
        int gpio_num = rtc_gpios[i];

        ESP_LOGI(TAG, "Restoring GPIO %d from RTC mode", gpio_num);

        // Disable hold first
        esp_err_t hold_err = rtc_gpio_hold_dis(gpio_num);
        if (hold_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to disable hold for RTC GPIO %d: %s", gpio_num, esp_err_to_name(hold_err));
        }

        // Initialize GPIO back to normal mode (this will disconnect from RTC)
        esp_err_t init_err = rtc_gpio_deinit(gpio_num);
        if (init_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to deinit RTC GPIO %d: %s", gpio_num, esp_err_to_name(init_err));
            if (first_err == ESP_OK) first_err = init_err;
            continue;
        }

        ESP_LOGI(TAG, "RTC GPIO %d restored to normal mode", gpio_num);
    }

    if (first_err == ESP_OK) {
        ESP_LOGI(TAG, "RTC GPIO restoration completed successfully");
    } else {
        ESP_LOGW(TAG, "RTC GPIO restoration completed with errors");
    }

    return first_err;
}
