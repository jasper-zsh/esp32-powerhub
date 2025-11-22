#include "storage.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "nvs_flash.h"
#include "nvs.h"

esp_err_t storage_read_states(uint8_t out_states[PWM_CHANNEL_COUNT]) {
    if (!out_states) return ESP_ERR_INVALID_ARG;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        // If not found, default to zeros
        for (int i = 0; i < PWM_CHANNEL_COUNT; ++i) out_states[i] = 0;
        return ESP_OK;
    }
    size_t stored_len = 0;
    err = nvs_get_blob(handle, STORAGE_KEY_STATES, NULL, &stored_len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        for (int i = 0; i < PWM_CHANNEL_COUNT; ++i) out_states[i] = 0;
        err = ESP_OK;
    } else if (err == ESP_OK) {
        size_t read_len = stored_len;
        uint8_t temp[6] = {0}; // historical max size
        if (stored_len > sizeof(temp)) {
            read_len = sizeof(temp);
        }
        err = nvs_get_blob(handle, STORAGE_KEY_STATES, temp, &read_len);
        if (err == ESP_OK) {
            size_t copy_len = (read_len < PWM_CHANNEL_COUNT) ? read_len : PWM_CHANNEL_COUNT;
            memcpy(out_states, temp, copy_len);
            if (copy_len < PWM_CHANNEL_COUNT) {
                memset(out_states + copy_len, 0, PWM_CHANNEL_COUNT - copy_len);
            }
        }
    }
    nvs_close(handle);
    return err;
}

esp_err_t storage_write_states(const uint8_t states[PWM_CHANNEL_COUNT]) {
    if (!states) return ESP_ERR_INVALID_ARG;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(handle, STORAGE_KEY_STATES, states, PWM_CHANNEL_COUNT);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}
