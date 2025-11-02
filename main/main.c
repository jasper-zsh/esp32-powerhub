#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "driver/rmt_tx.h"

#include "pwm_control.h"
#include "storage.h"
#include "ble_server.h"
#include "preset_mgr.h"
#include "scheduler.h"
#include "led_status.h"
#include "power_mgr.h"
#include "adc128s102.h"

static const char *TAG = "app";

void app_main(void) {
    // 检查重启原因
    esp_reset_reason_t reset_reason = esp_reset_reason();
    ESP_LOGI(TAG, "System starting...");
    ESP_LOGI(TAG, "Reset reason: %d (%s)", reset_reason,
             reset_reason == ESP_RST_POWERON ? "Power on" :
             reset_reason == ESP_RST_EXT ? "External pin" :
             reset_reason == ESP_RST_SW ? "Software" :
             reset_reason == ESP_RST_PANIC ? "Panic/WDT" :
             reset_reason == ESP_RST_INT_WDT ? "Interrupt WDT" :
             reset_reason == ESP_RST_TASK_WDT ? "Task WDT" :
             reset_reason == ESP_RST_WDT ? "Other WDT" :
             reset_reason == ESP_RST_DEEPSLEEP ? "Deep sleep" :
             reset_reason == ESP_RST_BROWNOUT ? "Brownout" : "Unknown");

    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // Initialize RGB LED (handled in led_status_init)
    ESP_ERROR_CHECK(power_mgr_init());

    // Init LED status module
    ESP_ERROR_CHECK(led_status_init());
    ESP_LOGI(TAG, "LED status module initialized");
    
    // Set initial state to indicate system is starting
    led_status_set_bluetooth_connected(false);
    led_status_set_error(false);
    ESP_LOGI(TAG, "Initial states set: connected=0, error=0");

    // 初始化PWM控制器
    esp_err_t pwm_err = pwm_control_init();
    if (pwm_err != ESP_OK) {
        ESP_LOGE(TAG, "PWM control init failed: %s", esp_err_to_name(pwm_err));
    } else {
        ESP_LOGI(TAG, "PWM control initialized");
    }

    // 初始化ADC128S102
    esp_err_t adc_err = adc128s102_init();
    if (adc_err != ESP_OK) {
        ESP_LOGE(TAG, "ADC128S102 init failed: %s", esp_err_to_name(adc_err));
    } else {
        ESP_LOGI(TAG, "ADC128S102 initialized");
        adc_err = adc128s102_start_continuous_sampling(10);
        if (adc_err != ESP_OK) {
            ESP_LOGE(TAG, "ADC sampling start failed: %s", esp_err_to_name(adc_err));
        } else {
            ESP_LOGI(TAG, "ADC sampling started");
        }
    }

    // 初始化预设管理器
    esp_err_t preset_err = preset_mgr_init();
    if (preset_err != ESP_OK) {
        ESP_LOGE(TAG, "Preset manager init failed: %s", esp_err_to_name(preset_err));
    } else {
        ESP_LOGI(TAG, "Preset manager initialized");
    }

    // 初始化调度器
    esp_err_t sched_err = scheduler_init();
    if (sched_err != ESP_OK) {
        ESP_LOGE(TAG, "Scheduler init failed: %s", esp_err_to_name(sched_err));
    } else {
        ESP_LOGI(TAG, "Scheduler initialized");
    }

    // BLE服务器初始化
    esp_err_t ble_ret = ble_server_init();
    if (ble_ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE server init failed: %s", esp_err_to_name(ble_ret));
    } else {
        ESP_LOGI(TAG, "BLE server initialized");
    }

    ESP_LOGI(TAG, "System initialized successfully.");

    // 主循环 - 任务已经启动，这里只是保持主任务运行
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // 10秒间隔
        ESP_LOGD(TAG, "Main task running...");
    }
}
