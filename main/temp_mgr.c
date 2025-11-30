#include "temp_mgr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ds18b20_rmt.h"

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

// DS18B20 RMT manager context
static ds18b20_rmt_manager_t ds18b20_manager;
static bool ds18b20_manager_initialized = false;

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

// Execute actual sensor sampling (synchronous, but called by task)
static bool perform_sensor_sample(temp_sensor_type_t sensor_type, int16_t *out_temp) {
    if (!ds18b20_manager_initialized) {
        return false;
    }

    uint8_t actual_sensor_count = ds18b20_rmt_get_sensor_count(&ds18b20_manager);
    if (sensor_type >= actual_sensor_count) {
        return false;
    }

    if (!ds18b20_rmt_is_sensor_connected(&ds18b20_manager, sensor_type)) {
        ESP_LOGW(TAG, "Sensor %d is not connected", sensor_type);
        return false;
    }

    // Read temperature from the sensor
    float temperature;
    esp_err_t err = ds18b20_rmt_get_temperature(&ds18b20_manager, sensor_type, &temperature);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature from sensor %d: %s",
                 sensor_type, esp_err_to_name(err));
        return false;
    }

    // Convert to 0.01°C units
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

        // Trigger temperature conversion for all sensors first
        esp_err_t trigger_err = ds18b20_rmt_trigger_conversion_all(&ds18b20_manager);
        if (trigger_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to trigger temperature conversion: %s", esp_err_to_name(trigger_err));
        }

        // Wait for conversion to complete (DS18B20 requires ~750ms for 12-bit resolution)
        vTaskDelay(pdMS_TO_TICKS(750));

        // Sample all sensors
        uint8_t actual_sensor_count = ds18b20_rmt_get_sensor_count(&ds18b20_manager);
        for (int i = 0; i < actual_sensor_count && i < TEMP_MGR_MAX_SENSORS; i++) {
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

    ESP_LOGI(TAG, "Initializing DS18B20 temperature sensors using RMT-based 1-Wire on GPIO%d", TEMP_SENSOR_GPIO_NUM);

    // Initialize DS18B20 RMT manager (exactly like the example)
    ds18b20_rmt_config_t config = {
        .bus_gpio_num = TEMP_SENSOR_GPIO_NUM,
        .enable_pull_up = true,  // enable the internal pull-up resistor in case the external device didn't have one
        .max_rx_bytes = 10,      // 1byte ROM command + 8byte ROM number + 1byte device command
    };

    esp_err_t ret = ds18b20_rmt_init(&config, &ds18b20_manager);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DS18B20 RMT manager: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Check GPIO%d wiring and pull-up resistor", TEMP_SENSOR_GPIO_NUM);
        return ret;
    }

    // Discover DS18B20 sensors
    ret = ds18b20_rmt_discover_sensors(&ds18b20_manager);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to discover DS18B20 sensors: %s", esp_err_to_name(ret));
        ds18b20_rmt_deinit(&ds18b20_manager);
        return ret;
    }

    uint8_t sensor_count = ds18b20_rmt_get_sensor_count(&ds18b20_manager);
    if (sensor_count == 0) {
        ESP_LOGW(TAG, "No DS18B20 sensors found on GPIO%d", TEMP_SENSOR_GPIO_NUM);
        ESP_LOGW(TAG, "This is normal if no sensors are connected");
        ds18b20_rmt_deinit(&ds18b20_manager);
        return ESP_ERR_NOT_FOUND;
    }

    ds18b20_manager_initialized = true;
    ESP_LOGI(TAG, "DS18B20 RMT manager initialized successfully (%d sensors found)", sensor_count);

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
    uint8_t actual_sensor_count = ds18b20_rmt_get_sensor_count(&ds18b20_manager);
    ESP_LOGI(TAG, "DS18B20 temperature sensors initialized (%d sensors found)", actual_sensor_count);
    ESP_LOGI(TAG, "Async temperature sampling task started with %dms interval", sampling_interval_ms);

    // Print sensor assignment information
    const char* sensor_names[] = {"POWER", "CONTROL"};
    for (int i = 0; i < actual_sensor_count && i < TEMP_MGR_MAX_SENSORS; i++) {
        uint64_t address;
        ds18b20_rmt_get_sensor_address(&ds18b20_manager, i, &address);
        ESP_LOGI(TAG, "Sensor %d: %s - Address: %016llX",
                i, (i < 2) ? sensor_names[i] : "UNKNOWN", address);
    }

    return ESP_OK;

cleanup:
    // Clean up resources
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
    if (ds18b20_manager_initialized) {
        ds18b20_rmt_deinit(&ds18b20_manager);
        ds18b20_manager_initialized = false;
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

    // Clean up DS18B20 RMT manager
    if (ds18b20_manager_initialized) {
        ds18b20_rmt_deinit(&ds18b20_manager);
        ds18b20_manager_initialized = false;
    }

    // Reset statistics
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

    uint8_t actual_sensor_count = ds18b20_rmt_get_sensor_count(&ds18b20_manager);
    if (sensor_type >= actual_sensor_count) {
        ESP_LOGW(TAG, "Sensor %d not available (connected: %d)", sensor_type, actual_sensor_count);
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

    uint8_t actual_sensor_count = ds18b20_rmt_get_sensor_count(&ds18b20_manager);
    if (actual_sensor_count == 0) {
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
    if (!ds18b20_manager_initialized) {
        return 0;
    }
    return ds18b20_rmt_get_sensor_count(&ds18b20_manager);
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
