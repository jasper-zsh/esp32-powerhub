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

static void build_preset_key(uint8_t preset_id, char out_key[16]) {
    snprintf(out_key, 16, STORAGE_KEY_PRESET_PREFIX "%02X", preset_id);
}

esp_err_t storage_save_preset(uint8_t preset_id, const uint8_t *data, size_t len) {
    if (!data || len == 0 || preset_id == 0) return ESP_ERR_INVALID_ARG;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    char key[16];
    build_preset_key(preset_id, key);
    err = nvs_set_blob(handle, key, data, len);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t storage_load_preset(uint8_t preset_id, uint8_t *data, size_t *len) {
    if (!len || preset_id == 0) return ESP_ERR_INVALID_ARG;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return err;
    char key[16];
    build_preset_key(preset_id, key);
    err = nvs_get_blob(handle, key, data, len);
    nvs_close(handle);
    return err;
}

esp_err_t storage_delete_preset(uint8_t preset_id) {
    if (preset_id == 0) return ESP_ERR_INVALID_ARG;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    char key[16];
    build_preset_key(preset_id, key);
    err = nvs_erase_key(handle, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t storage_for_each_preset(storage_preset_iter_cb cb, void *ctx) {
    if (!cb) return ESP_ERR_INVALID_ARG;

    nvs_iterator_t it = NULL;
    esp_err_t err = nvs_entry_find(NVS_DEFAULT_PART_NAME, STORAGE_NAMESPACE, NVS_TYPE_BLOB, &it);
    while (err == ESP_OK) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);
        if (strncmp(info.key, STORAGE_KEY_PRESET_PREFIX, strlen(STORAGE_KEY_PRESET_PREFIX)) == 0) {
            nvs_handle_t handle;
            esp_err_t open_err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
            if (open_err != ESP_OK) {
                nvs_release_iterator(it);
                return open_err;
            }
            size_t len = 0;
            esp_err_t get_err = nvs_get_blob(handle, info.key, NULL, &len);
            if (get_err == ESP_OK && len > 0) {
                uint8_t *buf = malloc(len);
                if (!buf) {
                    nvs_close(handle);
                    nvs_release_iterator(it);
                    return ESP_ERR_NO_MEM;
                }
                get_err = nvs_get_blob(handle, info.key, buf, &len);
                nvs_close(handle);
                if (get_err == ESP_OK) {
                    uint8_t preset_id = (uint8_t)strtol(info.key + strlen(STORAGE_KEY_PRESET_PREFIX), NULL, 16);
                    if (preset_id != 0) {
                        esp_err_t cb_err = cb(preset_id, buf, len, ctx);
                        free(buf);
                        if (cb_err != ESP_OK) {
                            nvs_release_iterator(it);
                            return cb_err;
                        }
                    } else {
                        free(buf);
                    }
                } else {
                    free(buf);
                    nvs_release_iterator(it);
                    return get_err;
                }
            } else {
                nvs_close(handle);
                if (get_err != ESP_ERR_NVS_NOT_FOUND) {
                    nvs_release_iterator(it);
                    return get_err;
                }
            }
        }
        err = nvs_entry_next(&it);
    }
    if (it) {
        nvs_release_iterator(it);
    }
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    return err;
}
