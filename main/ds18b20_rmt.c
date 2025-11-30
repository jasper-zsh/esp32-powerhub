#include "ds18b20_rmt.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "ds18b20_rmt";

esp_err_t ds18b20_rmt_init(const ds18b20_rmt_config_t *config, ds18b20_rmt_manager_t *manager) {
    if (!config || !manager) {
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize manager structure
    memset(manager, 0, sizeof(ds18b20_rmt_manager_t));

    // Configure 1-Wire bus (exactly like the example)
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = config->bus_gpio_num,
        .flags = {
            .en_pull_up = config->enable_pull_up, // enable the internal pull-up resistor in case the external device didn't have one
        }
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = config->max_rx_bytes, // 1byte ROM command + 8byte ROM number + 1byte device command
    };

    // Create 1-Wire bus with RMT
    esp_err_t err = onewire_new_bus_rmt(&bus_config, &rmt_config, &manager->bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create 1-Wire RMT bus: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "1-Wire bus installed on GPIO%d", config->bus_gpio_num);
    manager->is_initialized = true;

    return ESP_OK;
}

esp_err_t ds18b20_rmt_discover_sensors(ds18b20_rmt_manager_t *manager) {
    if (!manager || !manager->is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Device iterator created, start searching...");

    // create 1-wire device iterator, which is used for device search (exactly like the example)
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    esp_err_t err = onewire_new_device_iter(manager->bus, &iter);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create device iterator: %s", esp_err_to_name(err));
        return err;
    }

    // Search for devices (exactly like the example)
    do {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
            ds18b20_config_t ds_cfg = {};
            onewire_device_address_t address;
            // check if the device is a DS18B20, if so, return the ds18b20 handle
            if (ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &manager->sensors[manager->sensor_count]) == ESP_OK) {
                ds18b20_get_device_address(manager->sensors[manager->sensor_count], &address);
                ESP_LOGI(TAG, "Found a DS18B20[%d], address: %016llX", manager->sensor_count, address);
                manager->sensor_count++;
                if (manager->sensor_count >= DS18B20_RMT_MAX_SENSORS) {
                    ESP_LOGI(TAG, "Max DS18B20 number reached, stop searching...");
                    break;
                }
            } else {
                ESP_LOGI(TAG, "Found an unknown device, address: %016llX", next_onewire_device.address);
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);

    onewire_del_device_iter(iter);
    ESP_LOGI(TAG, "Searching done, %d DS18B20 device(s) found", manager->sensor_count);

    return ESP_OK;
}

esp_err_t ds18b20_rmt_trigger_conversion_all(ds18b20_rmt_manager_t *manager) {
    if (!manager || !manager->is_initialized || manager->sensor_count == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    // trigger temperature conversion for all sensors on the bus (exactly like the example)
    esp_err_t err = ds18b20_trigger_temperature_conversion_for_all(manager->bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger temperature conversion: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGD(TAG, "Temperature conversion triggered for %d sensor(s)", manager->sensor_count);
    return ESP_OK;
}

esp_err_t ds18b20_rmt_get_temperature(ds18b20_rmt_manager_t *manager, uint8_t sensor_index, float *temperature) {
    if (!manager || !manager->is_initialized || !temperature) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sensor_index >= manager->sensor_count) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read temperature from the specific sensor (exactly like the example)
    esp_err_t err = ds18b20_get_temperature(manager->sensors[sensor_index], temperature);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature from sensor %d: %s",
                sensor_index, esp_err_to_name(err));
        return err;
    }

    ESP_LOGD(TAG, "temperature read from DS18B20[%d]: %.2fC", sensor_index, *temperature);
    return ESP_OK;
}

esp_err_t ds18b20_rmt_get_sensor_address(ds18b20_rmt_manager_t *manager, uint8_t sensor_index, uint64_t *address) {
    if (!manager || !manager->is_initialized || !address) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sensor_index >= manager->sensor_count) {
        return ESP_ERR_INVALID_ARG;
    }

    // Get the address of the DS18B20 device (exactly like the example)
    onewire_device_address_t ret_address;
    esp_err_t err = ds18b20_get_device_address(manager->sensors[sensor_index], &ret_address);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get device address for sensor %d: %s", sensor_index, esp_err_to_name(err));
        return err;
    }

    *address = ret_address;
    return ESP_OK;
}

uint8_t ds18b20_rmt_get_sensor_count(ds18b20_rmt_manager_t *manager) {
    if (!manager) {
        return 0;
    }
    return manager->sensor_count;
}

bool ds18b20_rmt_is_sensor_connected(ds18b20_rmt_manager_t *manager, uint8_t sensor_index) {
    if (!manager || sensor_index >= manager->sensor_count) {
        return false;
    }
    return true; // Assume sensor is connected if within range
}

uint32_t ds18b20_rmt_get_sensor_error_count(ds18b20_rmt_manager_t *manager, uint8_t sensor_index) {
    if (!manager || sensor_index >= manager->sensor_count) {
        return 0;
    }
    return 0; // Error tracking not available in simplified API
}

void ds18b20_rmt_reset_sensor_error_count(ds18b20_rmt_manager_t *manager, uint8_t sensor_index) {
    // Error tracking not available in simplified API - no-op
    (void)manager;
    (void)sensor_index;
}

esp_err_t ds18b20_rmt_deinit(ds18b20_rmt_manager_t *manager) {
    if (!manager) {
        return ESP_ERR_INVALID_ARG;
    }

    // Delete all DS18B20 devices
    for (uint8_t i = 0; i < manager->sensor_count; i++) {
        if (manager->sensors[i]) {
            ds18b20_del_device(manager->sensors[i]);
            manager->sensors[i] = NULL;
        }
    }

    // Delete 1-Wire bus
    if (manager->bus) {
        onewire_bus_del(manager->bus);
        manager->bus = NULL;
    }

    // Clear manager state
    memset(manager, 0, sizeof(ds18b20_rmt_manager_t));

    ESP_LOGI(TAG, "DS18B20 RMT manager deinitialized");
    return ESP_OK;
}