#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/rmt_tx.h"

#include "pwm_control.h"
#include "storage.h"
#include "ble_server.h"
#include "preset_mgr.h"
#include "scheduler.h"
#include "led_status.h"
#include "led_rgb.h"

static const char *TAG = "app";

void app_main(void) {
    ESP_LOGI(TAG, "System starting...");
    
    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // Initialize RGB LED (handled in led_status_init)
    ESP_LOGI(TAG, "RGB LED initialized");
    
    // Set initial LED status (system starting)
    // ESP_ERROR_CHECK(led_rgb_set_status(false, false));
    // ESP_LOGI(TAG, "Initial LED status set");

    // Init LED status module
    ESP_ERROR_CHECK(led_status_init());
    ESP_LOGI(TAG, "LED status module initialized");
    
    // Set initial state to indicate system is starting
    led_status_set_bluetooth_connected(false);
    led_status_set_error(false);
    ESP_LOGI(TAG, "Initial states set: connected=0, error=0");

    // Initialize PWM and load persisted states
    ESP_LOGI(TAG, "Initializing PWM...");
    ESP_ERROR_CHECK(pwm_control_init());
    uint8_t states[4];
    ESP_ERROR_CHECK(storage_read_states(states));
    ESP_ERROR_CHECK(pwm_control_load_and_apply(states));
    ESP_LOGI(TAG, "PWM initialized");

    ESP_LOGI(TAG, "Initializing preset manager...");
    ESP_ERROR_CHECK(preset_mgr_init());
    ESP_LOGI(TAG, "Preset manager initialized");

    ESP_LOGI(TAG, "Initializing scheduler...");
    ESP_ERROR_CHECK(scheduler_init());
    ESP_LOGI(TAG, "Scheduler initialized");

    // Init BLE GATT server
    ESP_LOGI(TAG, "Initializing BLE server...");
    esp_err_t ble_ret = ble_server_init();
    if (ble_ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE server init failed: %s", esp_err_to_name(ble_ret));
    } else {
        ESP_LOGI(TAG, "BLE server initialized");
    }

    ESP_LOGI(TAG, "System initialized.");
}
