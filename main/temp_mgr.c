#include "temp_mgr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "onewire_bus.h"
#include "ds18b20.h"

static const char *TAG = "temp_mgr";

// DS18B20引脚定义
#define DS18B20_VDD_GPIO    GPIO_NUM_11
#define DS18B20_DQ_GPIO     GPIO_NUM_12
#define DS18B20_GND_GPIO    GPIO_NUM_13
static bool temp_mgr_initialized = false;

static onewire_bus_handle_t bus = NULL;
static ds18b20_device_handle_t ds18b20 = NULL;

// 设置GPIO为输出模式
static void set_gpio_output(gpio_num_t gpio, int level) {
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&conf);
    gpio_set_level(gpio, level);
}

// 设置GPIO为高阻态
static void set_gpio_high_z(gpio_num_t gpio) {
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&conf);
}

esp_err_t temp_mgr_init(void) {
    if (temp_mgr_initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing DS18B20 temperature sensor");
    ESP_LOGI(TAG, "VDD GPIO: %d, DQ GPIO: %d, GND GPIO: %d", 
             DS18B20_VDD_GPIO, DS18B20_DQ_GPIO, DS18B20_GND_GPIO);
    
    // 配置供电引脚
    set_gpio_output(DS18B20_GND_GPIO, 0);  // GND
    set_gpio_output(DS18B20_VDD_GPIO, 1);  // VDD
    
    // 稳定延时
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "Initializing DS18B20");
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = DS18B20_DQ_GPIO,
        .flags = {
            .en_pull_up = true,
        }
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10,
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));
    ESP_LOGI(TAG, "1-Wire bus installed on GPIO%d", DS18B20_DQ_GPIO);

    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    do {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK) {
            ds18b20_config_t ds_cfg = {};
            onewire_device_address_t address;
            if (ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &ds18b20) == ESP_OK) {
                ds18b20_get_device_address(ds18b20, &address);
                ESP_LOGI(TAG, "Found a DS18B20, address: %016llX", address);
                break;
            }
        } else {
            ESP_LOGI(TAG, "Found an unknown device, address: %016llX", next_onewire_device.address);
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    
    temp_mgr_initialized = true;
    ESP_LOGI(TAG, "DS18B20 temperature sensor initialized");
    
    return ESP_OK;
}

void temp_mgr_deinit(void) {
    if (!temp_mgr_initialized) {
        return;
    }
    
    // 将供电GPIO置为高阻态
    set_gpio_high_z(DS18B20_VDD_GPIO);
    set_gpio_high_z(DS18B20_GND_GPIO);
    
    // 将数据GPIO置为高阻态
    set_gpio_high_z(DS18B20_DQ_GPIO);
    
    temp_mgr_initialized = false;
    ds18b20 = NULL;
    
    ESP_LOGI(TAG, "DS18B20 temperature sensor deinitialized");
}

esp_err_t temp_mgr_sample_once(int16_t *out_temp) {
    if (!temp_mgr_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (out_temp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (ds18b20 == NULL) {
        ESP_LOGW(TAG, "No DS18B20 device present");
        return ESP_ERR_NOT_FOUND;
    }
    
    // 启动温度转换
    esp_err_t err = ds18b20_trigger_temperature_conversion_for_all(bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start temperature conversion: %s", esp_err_to_name(err));
        return err;
    }
    
    // 转换为摄氏度（12位分辨率，0.0625°C/位）
    float temperature;
    err = ds18b20_get_temperature(ds18b20, &temperature);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get temperature: %s", esp_err_to_name(err));
        return err;
    }
    
    // 转换为0.01°C单位
    int32_t temp_calc = (int32_t)(temperature * 100);
    
    // 范围检查
    if (temp_calc > 32767 || temp_calc < -32768) {
        ESP_LOGW(TAG, "Temperature out of range: %ld", temp_calc);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    *out_temp = (int16_t)temp_calc;
    
    // 正确显示温度值，包括负数的小数部分
    if (*out_temp >= 0) {
        ESP_LOGI(TAG, "Temperature: %d.%02d°C", *out_temp / 100, *out_temp % 100);
    } else {
        int16_t temp_abs = -*out_temp;
        ESP_LOGI(TAG, "Temperature: -%d.%02d°C", temp_abs / 100, temp_abs % 100);
    }
    
    return ESP_OK;
}
