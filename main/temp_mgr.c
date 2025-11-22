#include "temp_mgr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "onewire_gpio.h"
#include "ds18b20_gpio.h"

static const char *TAG = "temp_mgr";

// DS18B20引脚定义 - 根据最新硬件定义，两个DS18B20连接到同一GPIO
#define DS18B20_DQ_GPIO     ((gpio_num_t)TEMP_SENSOR_GPIO_NUM)
static bool temp_mgr_initialized = false;

#if TEMP_MGR_MAX_SENSORS == 0

esp_err_t temp_mgr_init(void) {
    ESP_LOGI(TAG, "Temperature sensors disabled (TEMP_SENSOR_COUNT=0)");
    temp_mgr_initialized = true;
    return ESP_OK;
}

esp_err_t temp_mgr_sample_once(temp_sensor_type_t sensor_type, int16_t *out_temp) {
    (void)sensor_type;
    if (out_temp) {
        *out_temp = 0;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t temp_mgr_sample_all(int16_t out_temps[TEMP_SENSOR_COUNT]) {
    (void)out_temps;
    return ESP_ERR_NOT_SUPPORTED;
}

int temp_mgr_get_sensor_count(void) {
    return 0;
}

void temp_mgr_deinit(void) {
    temp_mgr_initialized = false;
}

#else

static onewire_gpio_bus_handle_t bus = NULL;
static ds18b20_gpio_device_handle_t ds18b20_sensors[TEMP_MGR_MAX_SENSORS] = {NULL};
static uint64_t sensor_addresses[TEMP_MGR_MAX_SENSORS] = {0};
static int connected_sensor_count = 0;



esp_err_t temp_mgr_init(void) {
    if (temp_mgr_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing DS18B20 temperature sensors using GPIO bit-bang on GPIO%d", TEMP_SENSOR_GPIO_NUM);

    // 初始化1-Wire GPIO总线
    ESP_LOGI(TAG, "Creating 1-Wire GPIO bus for DS18B20");
    ESP_LOGI(TAG, "GPIO%d configuration: internal pull-up disabled, external 4.7kΩ pull-up required", TEMP_SENSOR_GPIO_NUM);

    onewire_gpio_config_t bus_config = {
        .gpio_num = DS18B20_DQ_GPIO,
        .enable_pullup = false,  // 需要外部4.7kΩ上拉电阻
    };

    esp_err_t ret = onewire_gpio_new_bus(&bus_config, &bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create 1-Wire GPIO bus: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Check GPIO%d wiring and external pull-up resistor (4.7kΩ recommended)", TEMP_SENSOR_GPIO_NUM);
        ESP_LOGE(TAG, "This GPIO implementation avoids RMT channel conflicts with LED");
        return ret;
    }
    ESP_LOGI(TAG, "1-Wire GPIO bus successfully created on GPIO%d", TEMP_SENSOR_GPIO_NUM);

    // 等待总线稳定
    ESP_LOGD(TAG, "Waiting for 1-Wire bus to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(100));

    // 搜索所有DS18B20传感器，增加重试机制
    onewire_gpio_device_iter_handle_t iter = NULL;
    onewire_gpio_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;
    int sensor_count = 0;
    int retry_count = 0;
    const int max_retries = 3;

    while (retry_count < max_retries && sensor_count == 0) {
        ESP_LOGI(TAG, "Searching for DS18B20 sensors (attempt %d/%d)", retry_count + 1, max_retries);
        ESP_LOGD(TAG, "Performing 1-Wire reset and presence detection...");

        ret = onewire_gpio_new_device_iter(bus, &iter);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create device iterator: %s", esp_err_to_name(ret));
            ESP_LOGD(TAG, "1-Wire bus may be disconnected or shorted");
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        ESP_LOGD(TAG, "Device iterator created successfully, starting ROM search...");

        do {
            search_result = onewire_gpio_device_iter_get_next(iter, &next_onewire_device);
            if (search_result == ESP_OK) {
                // 打印发现的设备地址原始数据 (假设address是64位整数)
                uint64_t device_rom = next_onewire_device.address;
                ESP_LOGI(TAG, "Found 1-Wire device, raw address: %016llX", device_rom);

                // 提取字节数组用于CRC计算
                uint8_t addr_bytes[8];
                for (int i = 0; i < 8; i++) {
                    addr_bytes[i] = (device_rom >> (i * 8)) & 0xFF;
                }

                // 计算并显示CRC校验结果
                uint8_t calculated_crc = onewire_gpio_crc8(addr_bytes, 7);
                if (calculated_crc == addr_bytes[7]) {
                    ESP_LOGI(TAG, "CRC check: PASS (calculated=0x%02X, received=0x%02X)",
                             calculated_crc, addr_bytes[7]);
                } else {
                    ESP_LOGW(TAG, "CRC check: FAIL for device %016llX", device_rom);
                    ESP_LOGW(TAG, "  Raw address: %02X %02X %02X %02X %02X %02X %02X %02X",
                             addr_bytes[0], addr_bytes[1], addr_bytes[2], addr_bytes[3],
                             addr_bytes[4], addr_bytes[5], addr_bytes[6], addr_bytes[7]);
                    ESP_LOGW(TAG, "  Calculated CRC: 0x%02X, Received CRC: 0x%02X",
                             calculated_crc, addr_bytes[7]);
                    ESP_LOGW(TAG, "  Family code: 0x%02X", addr_bytes[0]);
                }

                ds18b20_gpio_config_t ds_cfg = {
                    .resolution = DS18B20_RESOLUTION_12BIT,
                    .parasite_power = false,
                };
                ds18b20_gpio_device_handle_t sensor;
                if (ds18b20_gpio_new_device(bus, &next_onewire_device, &ds_cfg, &sensor) == ESP_OK) {
                    uint64_t address;
                    ds18b20_gpio_get_device_address(sensor, &address);
                    ESP_LOGI(TAG, "DS18B20 sensor #%d successfully initialized (address: %016llX)", sensor_count + 1, address);

                    // 保存传感器句柄和地址，最多支持TEMP_MGR_MAX_SENSORS个传感器
                    if (sensor_count < TEMP_MGR_MAX_SENSORS) {
                        ds18b20_sensors[sensor_count] = sensor;
                        sensor_addresses[sensor_count] = address;
                        ESP_LOGI(TAG, "Sensor #%d assigned to slot %d", sensor_count + 1, sensor_count);
                    } else {
                        ESP_LOGW(TAG, "Found more sensors than supported (%d), ignoring sensor with address %016llX",
                                TEMP_MGR_MAX_SENSORS, address);
                        ds18b20_gpio_del_device(sensor);
                    }
                    sensor_count++;
                } else {
                    ESP_LOGW(TAG, "Found 1-Wire device but it's not a valid DS18B20");
                }
            } else if (search_result == ESP_ERR_INVALID_CRC) {
                ESP_LOGW(TAG, "CRC error during device search - received data failed CRC validation");
                ESP_LOGW(TAG, "This typically happens when no devices are connected or there's noise on the line");
                vTaskDelay(pdMS_TO_TICKS(2000));
            } else if (search_result != ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG, "Unexpected error during device search: %s", esp_err_to_name(search_result));
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        } while (search_result != ESP_ERR_NOT_FOUND);

        onewire_gpio_del_device_iter(iter);

        if (sensor_count == 0) {
            retry_count++;
            if (retry_count < max_retries) {
                ESP_LOGW(TAG, "No DS18B20 sensors found, retrying in 500ms...");
                ESP_LOGW(TAG, "Troubleshooting tips:");
                ESP_LOGW(TAG, "  1. Check GPIO%d connection to DS18B20 data line", TEMP_SENSOR_GPIO_NUM);
                ESP_LOGW(TAG, "  2. Verify 4.7kΩ pull-up resistor between GPIO%d and VCC", TEMP_SENSOR_GPIO_NUM);
                ESP_LOGW(TAG, "  3. Ensure DS18B20 is powered (VCC connected to 3.3V)");
                ESP_LOGW(TAG, "  4. Check for loose connections or short circuits");
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        } else {
            ESP_LOGI(TAG, "Device search completed successfully on attempt %d", retry_count + 1);
        }
    }

    if (sensor_count == 0) {
        ESP_LOGW(TAG, "No DS18B20 sensors found after %d attempts", max_retries);
        ESP_LOGW(TAG, "This is normal if no sensors are connected to GPIO%d", TEMP_SENSOR_GPIO_NUM);
        // 清理资源
        if (bus) {
            onewire_gpio_del_bus(bus);
            bus = NULL;
        }
        return ESP_ERR_NOT_FOUND;
    }

    // 更新连接的传感器数量（取实际找到的数量和最大支持数量的最小值）
    connected_sensor_count = (sensor_count < TEMP_MGR_MAX_SENSORS) ? sensor_count : TEMP_MGR_MAX_SENSORS;

    temp_mgr_initialized = true;
    ESP_LOGI(TAG, "DS18B20 temperature sensors initialized (%d sensors found, %d supported)",
             sensor_count, connected_sensor_count);

    // 打印传感器分配信息
    for (int i = 0; i < connected_sensor_count; i++) {
        const char* sensor_names[] = {"POWER", "CONTROL"};
        ESP_LOGI(TAG, "Sensor %d: %s - Address: %016llX",
                i, sensor_names[i], sensor_addresses[i]);
    }

    return ESP_OK;
}

void temp_mgr_deinit(void) {
    if (!temp_mgr_initialized) {
        return;
    }

    // 清理所有DS18B20设备
    for (int i = 0; i < TEMP_MGR_MAX_SENSORS; i++) {
        if (ds18b20_sensors[i] != NULL) {
            ds18b20_gpio_del_device(ds18b20_sensors[i]);
            ds18b20_sensors[i] = NULL;
        }
    }

    // 清理1-Wire总线
    if (bus != NULL) {
        onewire_gpio_del_bus(bus);
        bus = NULL;
    }

    // 重置传感器计数
    connected_sensor_count = 0;
    temp_mgr_initialized = false;

    ESP_LOGI(TAG, "DS18B20 temperature sensors deinitialized");
}

esp_err_t temp_mgr_sample_once(temp_sensor_type_t sensor_type, int16_t *out_temp) {
    if (!temp_mgr_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (out_temp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sensor_type >= connected_sensor_count) {
        ESP_LOGW(TAG, "Sensor %d not available (connected: %d)", sensor_type, connected_sensor_count);
        return ESP_ERR_NOT_FOUND;
    }

    if (ds18b20_sensors[sensor_type] == NULL) {
        ESP_LOGW(TAG, "DS18B20 sensor %d not initialized", sensor_type);
        return ESP_ERR_NOT_FOUND;
    }

    const char* sensor_names[] = {"POWER", "CONTROL"};

    // 启动温度转换（针对指定传感器）
    esp_err_t err = ds18b20_gpio_trigger_temperature_conversion(ds18b20_sensors[sensor_type]);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start temperature conversion for %s sensor: %s",
                sensor_names[sensor_type], esp_err_to_name(err));
        return err;
    }

    // 等待转换完成（使用了高速的传感器，转换只需要150ms）
    ESP_LOGD(TAG, "Waiting for temperature conversion to complete for %s sensor...", sensor_names[sensor_type]);
    vTaskDelay(pdMS_TO_TICKS(150));

    // 读取温度
    float temperature;
    err = ds18b20_gpio_get_temperature(ds18b20_sensors[sensor_type], &temperature);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get temperature from %s sensor: %s",
                sensor_names[sensor_type], esp_err_to_name(err));
        return err;
    }

    // 转换为0.01°C单位
    int32_t temp_calc = (int32_t)(temperature * 100);

    // 范围检查
    if (temp_calc > 32767 || temp_calc < -32768) {
        ESP_LOGW(TAG, "Temperature out of range for %s sensor: %ld", sensor_names[sensor_type], temp_calc);
        return ESP_ERR_INVALID_RESPONSE;
    }

    *out_temp = (int16_t)temp_calc;

    return ESP_OK;
}

esp_err_t temp_mgr_sample_all(int16_t out_temps[TEMP_SENSOR_COUNT]) {
    if (!temp_mgr_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (out_temps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (connected_sensor_count == 0) {
        ESP_LOGW(TAG, "No sensors connected");
        return ESP_ERR_NOT_FOUND;
    }

    // 初始化输出数组
    for (int i = 0; i < TEMP_SENSOR_COUNT; i++) {
        out_temps[i] = 0; // 默认值为0°C
    }

    // 启动所有传感器的温度转换
    esp_err_t err = ds18b20_gpio_trigger_temperature_conversion_for_all(bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start temperature conversion for all sensors: %s", esp_err_to_name(err));
        return err;
    }

    // 等待转换完成（12位分辨率需要约750ms）
    ESP_LOGD(TAG, "Waiting for temperature conversion to complete for all sensors...");
    vTaskDelay(pdMS_TO_TICKS(800));

    // 逐个读取传感器温度
    const char* sensor_names[] = {"POWER", "CONTROL"};
    bool any_success = false;

    for (int i = 0; i < connected_sensor_count; i++) {
        float temperature;
        err = ds18b20_gpio_get_temperature(ds18b20_sensors[i], &temperature);
        if (err == ESP_OK) {
            // 转换为0.01°C单位
            int32_t temp_calc = (int32_t)(temperature * 100);

            // 范围检查
            if (temp_calc <= 32767 && temp_calc >= -32768) {
                out_temps[i] = (int16_t)temp_calc;

                any_success = true;
            } else {
                ESP_LOGW(TAG, "Temperature out of range for %s sensor: %ld", sensor_names[i], temp_calc);
            }
        } else {
            ESP_LOGE(TAG, "Failed to get temperature from %s sensor: %s", sensor_names[i], esp_err_to_name(err));
        }
    }

    return any_success ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

int temp_mgr_get_sensor_count(void) {
    return connected_sensor_count;
}

#endif // TEMP_MGR_MAX_SENSORS == 0
