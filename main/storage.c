#include "storage.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "nvs_flash.h"
#include "nvs.h"

esp_err_t storage_read_states(uint8_t out_states[6]) {
    if (!out_states) return ESP_ERR_INVALID_ARG;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        // If not found, default to zeros
        for (int i = 0; i < 6; ++i) out_states[i] = 0;
        return ESP_OK;
    }
    size_t len = 6;
    err = nvs_get_blob(handle, STORAGE_KEY_STATES, out_states, &len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        for (int i = 0; i < 6; ++i) out_states[i] = 0;
        err = ESP_OK;
    }
    nvs_close(handle);
    return err;
}

esp_err_t storage_write_states(const uint8_t states[6]) {
    if (!states) return ESP_ERR_INVALID_ARG;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(handle, STORAGE_KEY_STATES, states, 6);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}