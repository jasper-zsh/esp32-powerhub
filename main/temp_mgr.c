#include "temp_mgr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "onewire_gpio.h"
#include "ds18b20_gpio.h"

static const char *TAG = "temp_mgr";

// DS18B20引脚定义 - 根据最新硬件定义，两个DS18B20连接到同一GPIO
#define DS18B20_DQ_GPIO     ((gpio_num_t)TEMP_SENSOR_GPIO_NUM)

// 任务相关变量
static TaskHandle_t temp_sampling_task_handle = NULL;
static bool temp_task_running = false;

// 共享数据存储
static temp_sensor_data_t cached_data[TEMP_MGR_MAX_SENSORS];
static SemaphoreHandle_t data_mutex = NULL;

// FreeRTOS事件系统
static EventGroupHandle_t temp_event_group = NULL;
static QueueHandle_t temp_event_queue = NULL;
#define TEMP_EVENT_QUEUE_SIZE 10

// 配置参数
static uint32_t sampling_interval_ms = TEMP_DEFAULT_SAMPLING_INTERVAL;
static uint32_t max_retry_count = 3;
static uint32_t retry_delay_ms = 1000;

// 错误跟踪
static uint32_t consecutive_failures = 0;
static uint32_t total_samples = 0;
static uint32_t successful_samples = 0;

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

// === 内部辅助函数 ===

// 发送温度事件到FreeRTOS事件系统
static void emit_temp_event(temp_queue_event_type_t type, temp_sensor_type_t sensor_type, int16_t temperature) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    EventBits_t event_bits = 0;

    // 设置事件组位
    switch (type) {
        case TEMP_QUEUE_EVENT_NEW_DATA:
            event_bits = TEMP_EVENT_NEW_DATA_BIT;
            break;
        case TEMP_QUEUE_EVENT_THRESHOLD_HIGH:
            event_bits = TEMP_EVENT_THRESHOLD_HIGH_BIT;
            break;
        case TEMP_QUEUE_EVENT_THRESHOLD_LOW:
            event_bits = TEMP_EVENT_THRESHOLD_LOW_BIT;
            break;
        case TEMP_QUEUE_EVENT_SENSOR_ERROR:
            event_bits = TEMP_EVENT_SENSOR_ERROR_BIT;
            break;
        case TEMP_QUEUE_EVENT_SENSOR_RECOVER:
            event_bits = TEMP_EVENT_SENSOR_RECOVER_BIT;
            break;
    }

    // 发送事件组通知
    if (temp_event_group != NULL) {
        xEventGroupSetBits(temp_event_group, event_bits);
    }

    // 发送详细事件到队列
    if (temp_event_queue != NULL) {
        temp_queue_event_t queue_event = {
            .type = type,
            .sensor_type = sensor_type,
            .temperature = temperature,
            .timestamp = current_time
        };

        // 非阻塞发送到队列，如果队列满了则丢弃最旧的事件
        if (uxQueueSpacesAvailable(temp_event_queue) == 0) {
            temp_queue_event_t old_event;
            xQueueReceive(temp_event_queue, &old_event, 0);
        }

        xQueueSend(temp_event_queue, &queue_event, 0);
    }
}

// 更新缓存的温度数据
static void update_cached_data(temp_sensor_type_t sensor_type, int16_t temperature, bool success) {
    if (sensor_type >= TEMP_MGR_MAX_SENSORS) return;

    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (success) {
            cached_data[sensor_type].temperature = temperature;
            cached_data[sensor_type].timestamp = current_time;
            cached_data[sensor_type].status = TEMP_STATUS_VALID;
            cached_data[sensor_type].error_count = 0;

            // 发送新数据事件
            emit_temp_event(TEMP_QUEUE_EVENT_NEW_DATA, sensor_type, temperature);

            ESP_LOGD(TAG, "Updated cached data for sensor %d: %d.%02d°C",
                     sensor_type, temperature / 100, abs(temperature % 100));
        } else {
            cached_data[sensor_type].error_count++;
            if (cached_data[sensor_type].error_count >= max_retry_count) {
                cached_data[sensor_type].status = TEMP_STATUS_ERROR;
                emit_temp_event(TEMP_QUEUE_EVENT_SENSOR_ERROR, sensor_type, cached_data[sensor_type].temperature);
                ESP_LOGW(TAG, "Sensor %d marked as error after %d consecutive failures",
                         sensor_type, cached_data[sensor_type].error_count);
            }
        }
        xSemaphoreGive(data_mutex);
    }
}

// 执行实际的传感器采样 (同步，但被任务调用)
static bool perform_sensor_sample(temp_sensor_type_t sensor_type, int16_t *out_temp) {
    if (sensor_type >= connected_sensor_count || ds18b20_sensors[sensor_type] == NULL) {
        return false;
    }

    // 启动温度转换
    esp_err_t err = ds18b20_gpio_trigger_temperature_conversion(ds18b20_sensors[sensor_type]);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger temperature conversion for sensor %d: %s",
                 sensor_type, esp_err_to_name(err));
        return false;
    }

    // 等待转换完成
    vTaskDelay(pdMS_TO_TICKS(150));

    // 读取温度
    float temperature;
    err = ds18b20_gpio_get_temperature(ds18b20_sensors[sensor_type], &temperature);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature from sensor %d: %s",
                 sensor_type, esp_err_to_name(err));
        return false;
    }

    // 转换为0.01°C单位
    int32_t temp_calc = (int32_t)(temperature * 100);
    if (temp_calc > 32767 || temp_calc < -32768) {
        ESP_LOGW(TAG, "Temperature out of range for sensor %d: %ld", sensor_type, temp_calc);
        return false;
    }

    *out_temp = (int16_t)temp_calc;
    return true;
}

// 温度采样任务
static void temp_sampling_task(void *parameter) {
    ESP_LOGI(TAG, "Temperature sampling task started");

    consecutive_failures = 0;
    total_samples = 0;
    successful_samples = 0;

    while (temp_task_running) {
        bool any_success = false;

        // 采样所有传感器
        for (int i = 0; i < connected_sensor_count; i++) {
            int16_t temperature;
            bool success = perform_sensor_sample((temp_sensor_type_t)i, &temperature);

            if (success) {
                update_cached_data((temp_sensor_type_t)i, temperature, true);
                any_success = true;
                successful_samples++;
            } else {
                update_cached_data((temp_sensor_type_t)i, 0, false);
            }
            total_samples++;

            // 传感器间的小延迟
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // 更新连续失败计数
        if (any_success) {
            consecutive_failures = 0;
        } else {
            consecutive_failures++;
            if (consecutive_failures > 5) {
                ESP_LOGW(TAG, "Multiple consecutive sampling failures, increasing retry delay");
                vTaskDelay(pdMS_TO_TICKS(retry_delay_ms * 2));
            }
        }

        // 等待下次采样或立即采样通知
        uint32_t notification_value = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(sampling_interval_ms));
        if (notification_value > 0) {
            ESP_LOGD(TAG, "Immediate sample notification received");
        }
    }

    ESP_LOGI(TAG, "Temperature sampling task stopped");
    vTaskDelete(NULL);
}

// 初始化缓存数据
static void init_cached_data(void) {
    for (int i = 0; i < TEMP_MGR_MAX_SENSORS; i++) {
        cached_data[i].temperature = 0;
        cached_data[i].timestamp = 0;
        cached_data[i].status = TEMP_STATUS_INVALID;
        cached_data[i].error_count = 0;
    }
}



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

    // 初始化新的异步组件
    ESP_LOGI(TAG, "Initializing async temperature manager components with FreeRTOS events");

    // 创建互斥锁
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        goto cleanup;
    }

    // 创建FreeRTOS事件组
    temp_event_group = xEventGroupCreate();
    if (temp_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create temperature event group");
        goto cleanup;
    }

    // 创建FreeRTOS队列
    temp_event_queue = xQueueCreate(TEMP_EVENT_QUEUE_SIZE, sizeof(temp_queue_event_t));
    if (temp_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create temperature event queue");
        goto cleanup;
    }

    // 初始化缓存数据
    init_cached_data();

    // 创建温度采样任务
    temp_task_running = true;
    BaseType_t task_result = xTaskCreate(
        temp_sampling_task,
        "temp_sampling",
        4096,  // 栈大小
        NULL,
        5,     // 优先级 (中等优先级)
        &temp_sampling_task_handle
    );

    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create temperature sampling task");
        temp_task_running = false;
        goto cleanup;
    }

    temp_mgr_initialized = true;
    ESP_LOGI(TAG, "DS18B20 temperature sensors initialized (%d sensors found, %d supported)",
             sensor_count, connected_sensor_count);
    ESP_LOGI(TAG, "Async temperature sampling task started with %dms interval", sampling_interval_ms);

    // 打印传感器分配信息
    for (int i = 0; i < connected_sensor_count; i++) {
        const char* sensor_names[] = {"POWER", "CONTROL"};
        ESP_LOGI(TAG, "Sensor %d: %s - Address: %016llX",
                i, sensor_names[i], sensor_addresses[i]);
    }

    return ESP_OK;

cleanup:
    // 清理资源
    if (data_mutex) {
        vSemaphoreDelete(data_mutex);
        data_mutex = NULL;
    }
    if (temp_event_group) {
        vEventGroupDelete(temp_event_group);
        temp_event_group = NULL;
    }
    if (temp_event_queue) {
        vQueueDelete(temp_event_queue);
        temp_event_queue = NULL;
    }
    if (bus) {
        onewire_gpio_del_bus(bus);
        bus = NULL;
    }
    for (int i = 0; i < TEMP_MGR_MAX_SENSORS; i++) {
        if (ds18b20_sensors[i] != NULL) {
            ds18b20_gpio_del_device(ds18b20_sensors[i]);
            ds18b20_sensors[i] = NULL;
        }
    }
    return ESP_ERR_NO_MEM;
}

void temp_mgr_deinit(void) {
    if (!temp_mgr_initialized) {
        return;
    }

    ESP_LOGI(TAG, "Deinitializing temperature manager");

    // 停止采样任务
    if (temp_task_running) {
        ESP_LOGI(TAG, "Stopping temperature sampling task");
        temp_task_running = false;

        // 等待任务结束
        if (temp_sampling_task_handle != NULL) {
            // 发送通知给任务以加速停止
            xTaskNotifyGive(temp_sampling_task_handle);

            // 等待任务删除，最多等待2秒
            int wait_count = 0;
            while (eTaskGetState(temp_sampling_task_handle) != eDeleted && wait_count < 20) {
                vTaskDelay(pdMS_TO_TICKS(100));
                wait_count++;
            }

            if (eTaskGetState(temp_sampling_task_handle) != eDeleted) {
                ESP_LOGW(TAG, "Temperature sampling task did not stop cleanly, forcing deletion");
                vTaskDelete(temp_sampling_task_handle);
            }

            temp_sampling_task_handle = NULL;
        }
    }

    // 清理互斥锁
    if (data_mutex != NULL) {
        vSemaphoreDelete(data_mutex);
        data_mutex = NULL;
    }

    // 清理FreeRTOS事件系统
    if (temp_event_group != NULL) {
        vEventGroupDelete(temp_event_group);
        temp_event_group = NULL;
    }

    if (temp_event_queue != NULL) {
        vQueueDelete(temp_event_queue);
        temp_event_queue = NULL;
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

    // 重置传感器计数和状态
    connected_sensor_count = 0;
    consecutive_failures = 0;
    total_samples = 0;
    successful_samples = 0;

    temp_mgr_initialized = false;

    ESP_LOGI(TAG, "Temperature manager deinitialized successfully");
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

    // 使用缓存数据 (非阻塞)
    temp_sensor_data_t data;
    esp_err_t ret = temp_mgr_get_cached(sensor_type, &data);
    if (ret == ESP_OK) {
        *out_temp = data.temperature;
        return ESP_OK;
    }

    // 如果没有有效数据，返回错误
    return ESP_ERR_INVALID_STATE;
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

    // 使用缓存数据 (非阻塞)
    temp_sensor_data_t data[TEMP_SENSOR_COUNT];
    esp_err_t ret = temp_mgr_get_all_cached(data);
    if (ret == ESP_OK) {
        for (int i = 0; i < TEMP_SENSOR_COUNT; i++) {
            out_temps[i] = data[i].temperature;
        }
        return ESP_OK;
    }

    // 如果没有有效数据，初始化为0
    for (int i = 0; i < TEMP_SENSOR_COUNT; i++) {
        out_temps[i] = 0;
    }
    return ESP_ERR_INVALID_STATE;
}

int temp_mgr_get_sensor_count(void) {
    return connected_sensor_count;
}

// === 新的异步API实现 ===

esp_err_t temp_mgr_get_cached(temp_sensor_type_t sensor_type, temp_sensor_data_t *out_data) {
    if (!temp_mgr_initialized || data_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (sensor_type >= TEMP_MGR_MAX_SENSORS || out_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *out_data = cached_data[sensor_type];
        xSemaphoreGive(data_mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t temp_mgr_get_all_cached(temp_sensor_data_t out_data[TEMP_SENSOR_COUNT]) {
    if (!temp_mgr_initialized || data_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (out_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < TEMP_MGR_MAX_SENSORS; i++) {
            out_data[i] = cached_data[i];
        }
        xSemaphoreGive(data_mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

bool temp_mgr_is_data_fresh(temp_sensor_type_t sensor_type, uint32_t max_age_ms) {
    if (!temp_mgr_initialized || sensor_type >= TEMP_MGR_MAX_SENSORS) {
        return false;
    }

    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        bool fresh = (cached_data[sensor_type].status == TEMP_STATUS_VALID) &&
                     (current_time - cached_data[sensor_type].timestamp <= max_age_ms);
        xSemaphoreGive(data_mutex);
        return fresh;
    }

    return false;
}

// === FreeRTOS事件系统API实现 ===

EventGroupHandle_t temp_mgr_get_event_group(void) {
    return temp_event_group;
}

uint32_t temp_mgr_wait_for_events(EventBits_t bits_to_wait, TickType_t timeout) {
    if (!temp_mgr_initialized || temp_event_group == NULL) {
        return 0;
    }

    return xEventGroupWaitBits(temp_event_group, bits_to_wait, pdTRUE, pdFALSE, timeout);
}

QueueHandle_t temp_mgr_get_event_queue(void) {
    return temp_event_queue;
}

bool temp_mgr_receive_event(temp_queue_event_t *event, TickType_t timeout) {
    if (!temp_mgr_initialized || temp_event_queue == NULL || event == NULL) {
        return false;
    }

    return xQueueReceive(temp_event_queue, event, timeout) == pdPASS;
}

void temp_mgr_clear_events(EventBits_t bits_to_clear) {
    if (temp_event_group != NULL) {
        xEventGroupClearBits(temp_event_group, bits_to_clear);
    }
}

esp_err_t temp_mgr_set_sampling_interval(uint32_t interval_ms) {
    if (!temp_mgr_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (interval_ms < 100 || interval_ms > 60000) {
        return ESP_ERR_INVALID_ARG;
    }

    sampling_interval_ms = interval_ms;
    ESP_LOGI(TAG, "Temperature sampling interval set to %dms", interval_ms);
    return ESP_OK;
}

uint32_t temp_mgr_get_sampling_interval(void) {
    return sampling_interval_ms;
}

esp_err_t temp_mgr_start_continuous(void) {
    if (!temp_mgr_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (temp_task_running) {
        return ESP_OK; // 已经在运行
    }

    // 创建采样任务
    temp_task_running = true;
    BaseType_t task_result = xTaskCreate(
        temp_sampling_task,
        "temp_sampling",
        4096,
        NULL,
        5,
        &temp_sampling_task_handle
    );

    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create temperature sampling task");
        temp_task_running = false;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Temperature sampling task started");
    return ESP_OK;
}

esp_err_t temp_mgr_stop_continuous(void) {
    if (!temp_mgr_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!temp_task_running) {
        return ESP_OK; // 已经停止
    }

    ESP_LOGI(TAG, "Stopping temperature sampling task");
    temp_task_running = false;

    // 等待任务结束
    if (temp_sampling_task_handle != NULL) {
        xTaskNotifyGive(temp_sampling_task_handle);
        vTaskDelay(pdMS_TO_TICKS(100));
        temp_sampling_task_handle = NULL;
    }

    ESP_LOGI(TAG, "Temperature sampling task stopped");
    return ESP_OK;
}

bool temp_mgr_is_running(void) {
    return temp_task_running;
}

esp_err_t temp_mgr_trigger_immediate_sample(void) {
    if (!temp_mgr_initialized || !temp_task_running) {
        return ESP_ERR_INVALID_STATE;
    }

    // 通知任务立即采样
    if (temp_sampling_task_handle != NULL) {
        xTaskNotifyGive(temp_sampling_task_handle);
        return ESP_OK;
    }

    return ESP_ERR_INVALID_STATE;
}

#endif // TEMP_MGR_MAX_SENSORS == 0
