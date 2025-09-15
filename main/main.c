#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "pwm_control.h"
#include "storage.h"
#include "ble_server.h"

static const char *TAG = "app";

void app_main(void) {
    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize PWM and load persisted states
    ESP_ERROR_CHECK(pwm_control_init());
    uint8_t states[4];
    ESP_ERROR_CHECK(storage_read_states(states));
    ESP_ERROR_CHECK(pwm_control_load_and_apply(states));

    // Init BLE GATT server
    ESP_ERROR_CHECK(ble_server_init());

    ESP_LOGI(TAG, "System initialized.");
}

