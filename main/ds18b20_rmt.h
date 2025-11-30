#ifndef DS18B20_RMT_H
#define DS18B20_RMT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "onewire_bus.h"
#include "onewire_device.h"
#include "ds18b20.h"

#ifdef __cplusplus
extern "C" {
#endif

// Maximum number of DS18B20 sensors supported
#define DS18B20_RMT_MAX_SENSORS    8

// DS18B20 RMT configuration structure
typedef struct {
    int bus_gpio_num;                        // GPIO number that used by the 1-Wire bus
    bool enable_pull_up;                     // Enable internal pull-up resistor
    uint32_t max_rx_bytes;                   // Maximum RX bytes for RMT
} ds18b20_rmt_config_t;

// DS18B20 RMT manager context
typedef struct {
    onewire_bus_handle_t bus;                           // 1-Wire bus handle
    ds18b20_device_handle_t sensors[DS18B20_RMT_MAX_SENSORS]; // DS18B20 device handles
    uint8_t sensor_count;                               // Number of discovered sensors
    bool is_initialized;                                // Initialization status
} ds18b20_rmt_manager_t;

/**
 * @brief Initialize DS18B20 manager with RMT-based 1-Wire bus
 *
 * @param config Configuration structure
 * @param[out] manager Manager context (allocated by caller)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ds18b20_rmt_init(const ds18b20_rmt_config_t *config, ds18b20_rmt_manager_t *manager);

/**
 * @brief Discover DS18B20 sensors on the bus
 *
 * @param manager Manager context
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ds18b20_rmt_discover_sensors(ds18b20_rmt_manager_t *manager);

/**
 * @brief Trigger temperature conversion for all sensors on the bus
 *
 * @param manager Manager context
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ds18b20_rmt_trigger_conversion_all(ds18b20_rmt_manager_t *manager);

/**
 * @brief Read temperature from a specific sensor
 *
 * @param manager Manager context
 * @param sensor_index Sensor index (0 to sensor_count-1)
 * @param[out] temperature Temperature in Celsius
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ds18b20_rmt_get_temperature(ds18b20_rmt_manager_t *manager, uint8_t sensor_index, float *temperature);

/**
 * @brief Get sensor device address
 *
 * @param manager Manager context
 * @param sensor_index Sensor index
 * @param[out] address Device address
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ds18b20_rmt_get_sensor_address(ds18b20_rmt_manager_t *manager, uint8_t sensor_index, uint64_t *address);

/**
 * @brief Get the number of discovered sensors
 *
 * @param manager Manager context
 * @return uint8_t Number of sensors
 */
uint8_t ds18b20_rmt_get_sensor_count(ds18b20_rmt_manager_t *manager);

/**
 * @brief Check if a sensor is currently connected
 *
 * @param manager Manager context
 * @param sensor_index Sensor index
 * @return bool True if sensor is connected
 */
bool ds18b20_rmt_is_sensor_connected(ds18b20_rmt_manager_t *manager, uint8_t sensor_index);

/**
 * @brief Get sensor error count
 *
 * @param manager Manager context
 * @param sensor_index Sensor index
 * @return uint32_t Error count
 */
uint32_t ds18b20_rmt_get_sensor_error_count(ds18b20_rmt_manager_t *manager, uint8_t sensor_index);

/**
 * @brief Reset sensor error count (called after successful read)
 *
 * @param manager Manager context
 * @param sensor_index Sensor index
 */
void ds18b20_rmt_reset_sensor_error_count(ds18b20_rmt_manager_t *manager, uint8_t sensor_index);

/**
 * @brief Deinitialize DS18B20 manager
 *
 * @param manager Manager context
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ds18b20_rmt_deinit(ds18b20_rmt_manager_t *manager);

#ifdef __cplusplus
}
#endif

#endif // DS18B20_RMT_H