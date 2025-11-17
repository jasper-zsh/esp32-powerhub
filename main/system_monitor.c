#include "system_monitor.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"

#include "power_mgr.h"
#include "adc128s102.h"
#include "temp_mgr.h"

static const char *TAG = "system_monitor";

// 事件队列大小
#define EVENT_QUEUE_SIZE 20
#define MONITOR_TASK_STACK_SIZE 4096
#define MONITOR_TASK_PRIORITY 5

// 默认配置
#define DEFAULT_VOLTAGE_LOW_WARNING    11000   // 11.0V
#define DEFAULT_VOLTAGE_LOW_CRITICAL   10000   // 10.0V
#define DEFAULT_VOLTAGE_RECOVERY       12000   // 12.0V

#define DEFAULT_CURRENT_OVERLOAD       20000   // 20.0A (乘以1000)
#define DEFAULT_CURRENT_OVERLOAD_DURATION 5000 // 5秒

#define DEFAULT_TEMP_HIGH_WARNING      7000    // 70.0°C (乘以100)
#define DEFAULT_TEMP_HIGH_CRITICAL     8000    // 80.0°C (乘以100)
#define DEFAULT_TEMP_RECOVERY          6000    // 60.0°C (乘以100)

#define DEFAULT_VOLTAGE_MONITOR_INTERVAL  1000  // 1秒
#define DEFAULT_CURRENT_MONITOR_INTERVAL  1000  // 1秒
#define DEFAULT_TEMP_MONITOR_INTERVAL     1000  // 1秒

// 全局变量
static TaskHandle_t s_monitor_task_handle = NULL;
static QueueHandle_t s_event_queue = NULL;
static bool s_monitor_running = false;

// 配置和状态
static monitor_config_t s_config = {
    .voltage_low_warning = DEFAULT_VOLTAGE_LOW_WARNING,
    .voltage_low_critical = DEFAULT_VOLTAGE_LOW_CRITICAL,
    .voltage_recovery = DEFAULT_VOLTAGE_RECOVERY,
    .current_overload_threshold = DEFAULT_CURRENT_OVERLOAD,
    .current_overload_duration = DEFAULT_CURRENT_OVERLOAD_DURATION,
    .temp_high_warning = DEFAULT_TEMP_HIGH_WARNING,
    .temp_high_critical = DEFAULT_TEMP_HIGH_CRITICAL,
    .temp_recovery = DEFAULT_TEMP_RECOVERY,
    .voltage_monitor_interval = DEFAULT_VOLTAGE_MONITOR_INTERVAL,
    .current_monitor_interval = DEFAULT_CURRENT_MONITOR_INTERVAL,
    .temp_monitor_interval = DEFAULT_TEMP_MONITOR_INTERVAL,
};

static monitor_state_t s_state = {0};

// 事件处理器
typedef struct {
    system_event_type_t event_type;
    system_event_handler_t handler;
    void *user_data;
} event_handler_entry_t;

#define MAX_EVENT_HANDLERS 10
static event_handler_entry_t s_event_handlers[MAX_EVENT_HANDLERS] = {0};
static uint8_t s_handler_count = 0;

// 最新事件
static system_event_t s_last_event = {0};

// 内部函数声明
static void monitor_task(void *arg);
static esp_err_t create_and_send_event(system_event_type_t type, float value, uint8_t channel, const char *description);
static esp_err_t check_voltage_thresholds(float voltage);
static esp_err_t check_current_thresholds(const float *currents, uint8_t count);
static esp_err_t check_temperature_thresholds(float power_temp, float control_temp);
static esp_err_t dispatch_event(const system_event_t *event);
static const char* get_event_description(system_event_type_t type);
static event_priority_t get_event_priority(system_event_type_t type);

// 初始化系统监控
esp_err_t system_monitor_init(void) {
    ESP_LOGI(TAG, "Initializing system monitor...");

    // 创建事件队列
    s_event_queue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(system_event_t));
    if (s_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return ESP_ERR_NO_MEM;
    }

    // 清零状态
    memset(&s_state, 0, sizeof(s_state));
    memset(&s_last_event, 0, sizeof(s_last_event));

    ESP_LOGI(TAG, "System monitor initialized successfully");
    return ESP_OK;
}

// 启动监控任务
esp_err_t system_monitor_start(void) {
    if (s_monitor_running) {
        ESP_LOGW(TAG, "Monitor already running");
        return ESP_OK;
    }

    BaseType_t ret = xTaskCreate(monitor_task, "system_monitor",
                                MONITOR_TASK_STACK_SIZE, NULL,
                                MONITOR_TASK_PRIORITY, &s_monitor_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
        return ESP_ERR_NO_MEM;
    }

    s_monitor_running = true;
    ESP_LOGI(TAG, "System monitor started");
    return ESP_OK;
}

// 停止监控任务
esp_err_t system_monitor_stop(void) {
    if (!s_monitor_running) {
        return ESP_OK;
    }

    s_monitor_running = false;

    if (s_monitor_task_handle != NULL) {
        vTaskDelete(s_monitor_task_handle);
        s_monitor_task_handle = NULL;
    }

    ESP_LOGI(TAG, "System monitor stopped");
    return ESP_OK;
}

// 注册事件处理器
esp_err_t system_monitor_register_handler(system_event_type_t event_type,
                                        system_event_handler_t handler,
                                        void *user_data) {
    if (s_handler_count >= MAX_EVENT_HANDLERS) {
        ESP_LOGE(TAG, "Too many event handlers");
        return ESP_ERR_NO_MEM;
    }

    // 检查是否已经注册过相同类型的事件处理器
    for (uint8_t i = 0; i < s_handler_count; i++) {
        if (s_event_handlers[i].event_type == event_type) {
            ESP_LOGW(TAG, "Handler for event type %d already exists, replacing", event_type);
            s_event_handlers[i].handler = handler;
            s_event_handlers[i].user_data = user_data;
            return ESP_OK;
        }
    }

    // 添加新的处理器
    s_event_handlers[s_handler_count].event_type = event_type;
    s_event_handlers[s_handler_count].handler = handler;
    s_event_handlers[s_handler_count].user_data = user_data;
    s_handler_count++;

    ESP_LOGI(TAG, "Registered handler for event type %d", event_type);
    return ESP_OK;
}

// 注销事件处理器
esp_err_t system_monitor_unregister_handler(system_event_type_t event_type) {
    for (uint8_t i = 0; i < s_handler_count; i++) {
        if (s_event_handlers[i].event_type == event_type) {
            // 移动后面的处理器
            for (uint8_t j = i; j < s_handler_count - 1; j++) {
                s_event_handlers[j] = s_event_handlers[j + 1];
            }
            s_handler_count--;
            ESP_LOGI(TAG, "Unregistered handler for event type %d", event_type);
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "Handler for event type %d not found", event_type);
    return ESP_ERR_NOT_FOUND;
}

// 获取监控配置
esp_err_t system_monitor_get_config(monitor_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *config = s_config;
    return ESP_OK;
}

// 设置监控配置
esp_err_t system_monitor_set_config(const monitor_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 验证配置的合理性
    if (config->voltage_low_critical >= config->voltage_low_warning ||
        config->voltage_low_warning >= config->voltage_recovery) {
        ESP_LOGE(TAG, "Invalid voltage thresholds");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->temp_high_critical <= config->temp_high_warning ||
        config->temp_high_warning <= config->temp_recovery) {
        ESP_LOGE(TAG, "Invalid temperature thresholds");
        return ESP_ERR_INVALID_ARG;
    }

    s_config = *config;
    ESP_LOGI(TAG, "Monitor configuration updated");
    return ESP_OK;
}

// 获取监控状态
esp_err_t system_monitor_get_state(monitor_state_t *state) {
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *state = s_state;
    return ESP_OK;
}

// 手动触发事件
esp_err_t system_monitor_trigger_event(const system_event_t *event) {
    if (event == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return dispatch_event(event);
}

// 获取最新事件
esp_err_t system_monitor_get_last_event(system_event_t *event) {
    if (event == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *event = s_last_event;
    return ESP_OK;
}

// 检查是否有未处理的高优先级事件
bool system_monitor_has_critical_events(void) {
    return (s_state.voltage_low_critical_active ||
            s_state.temp_high_critical_active ||
            s_state.current_overload_active);
}

// ===== 内部函数实现 =====

// 监控任务主函数
static void monitor_task(void *arg) {
    ESP_LOGI(TAG, "Monitor task started - System monitoring active");

    uint32_t last_voltage_check = 0;
    uint32_t last_current_check = 0;
    uint32_t last_temp_check = 0;
    uint32_t last_console_output = 0;
    uint32_t current_time = 0;
    uint32_t task_start_time = current_time;

    // 初始状态检查
    ESP_LOGI(TAG, "Performing initial system checks...");

    // 立即输出一次监控状态，确认系统正在运行
    ESP_LOGI(TAG, "=== SYSTEM MONITOR STARTED ===");
    ESP_LOGI(TAG, "Monitor task active, checking data sources...");
    ESP_LOGI(TAG, "Voltage monitor interval: %lums", s_config.voltage_monitor_interval);
    ESP_LOGI(TAG, "Current monitor interval: %lums", s_config.current_monitor_interval);
    ESP_LOGI(TAG, "Temperature monitor interval: %lums", s_config.temp_monitor_interval);

    // 进行初始数据读取测试
    ESP_LOGI(TAG, "Performing initial data read test...");

    // 测试电压读取
    uint16_t test_voltage;
    esp_err_t voltage_ret = power_mgr_get_voltage_mv(&test_voltage, false);
    if (voltage_ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Voltage test: %umV", test_voltage);
    } else {
        ESP_LOGW(TAG, "✗ Voltage test failed: %s", esp_err_to_name(voltage_ret));
    }

    // 测试电流读取 - 延迟一段时间确保ADC已初始化
    vTaskDelay(pdMS_TO_TICKS(100));  // 等待100ms确保ADC初始化完成

    current_sensor_data_t test_current;
    esp_err_t current_ret = adc128s102_get_latest_data(&test_current);
    if (current_ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Current test: Total=%.3fA", test_current.total_input_current);
    } else {
        ESP_LOGW(TAG, "✗ Current test failed: %s", esp_err_to_name(current_ret));
    }

    // 检查温度传感器状态
    int temp_sensor_count = temp_mgr_get_sensor_count();
    ESP_LOGI(TAG, "Temperature sensors found: %d", temp_sensor_count);
    if (temp_sensor_count == 0) {
        ESP_LOGW(TAG, "No DS18B20 temperature sensors found on GPIO%d", TEMP_SENSOR_GPIO_NUM);
        ESP_LOGW(TAG, "Temperature monitoring will show default values until sensors are connected");
        ESP_LOGW(TAG, "To fix: Check 4.7kΩ pull-up resistor and sensor connections");
    } else {
        ESP_LOGI(TAG, "DS18B20 temperature sensors are active and ready");
    }

    ESP_LOGI(TAG, "=== MONITOR WILL OUTPUT DATA EVERY 1 SECOND ===");

    while (s_monitor_running) {
        current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // 每30秒输出一次任务状态
        if (current_time - task_start_time >= 30000) {
            ESP_LOGI(TAG, "Monitor task running - %lu seconds elapsed", (current_time - task_start_time) / 1000);
            task_start_time = current_time;
        }

        // 电压监控
        if (current_time - last_voltage_check >= s_config.voltage_monitor_interval) {
            uint16_t voltage_mv;
            esp_err_t voltage_ret = power_mgr_get_voltage_mv(&voltage_mv, false);
            if (voltage_ret == ESP_OK) {
                float voltage_v = voltage_mv / 1000.0f;
                s_state.last_voltage = voltage_v;
                check_voltage_thresholds(voltage_v);
            } else {
                ESP_LOGD(TAG, "Failed to get voltage data: %s", esp_err_to_name(voltage_ret));
            }
            last_voltage_check = current_time;
        }

        // 电流监控
        if (current_time - last_current_check >= s_config.current_monitor_interval) {
            current_sensor_data_t current_data;
            esp_err_t current_ret = adc128s102_get_latest_data(&current_data);
            if (current_ret == ESP_OK) {
                s_state.last_total_current = current_data.total_input_current;
                memcpy(s_state.last_channel_currents, current_data.channel_currents,
                       sizeof(s_state.last_channel_currents));
                check_current_thresholds(current_data.channel_currents, 6);
            } else if (current_ret == ESP_ERR_TIMEOUT) {
                // 互斥锁超时，可能ADC正在初始化或忙碌
                ESP_LOGD(TAG, "ADC data busy - will retry later");
            } else {
                ESP_LOGD(TAG, "Failed to get current data: %s", esp_err_to_name(current_ret));
                // 设置为无数据状态
                s_state.last_total_current = -1.0f;
                for (int i = 0; i < 6; i++) {
                    s_state.last_channel_currents[i] = -1.0f;
                }
            }
            last_current_check = current_time;
        }

        // 温度监控
        if (current_time - last_temp_check >= s_config.temp_monitor_interval) {
            int16_t temps[TEMP_SENSOR_COUNT];
            esp_err_t temp_ret = temp_mgr_sample_all(temps);
            int16_t power_temp = temps[0];
            int16_t control_temp = temps[1];
            if (temp_ret == ESP_OK) {
                s_state.last_power_temp = power_temp / 100.0f;
                s_state.last_control_temp = control_temp / 100.0f;
                check_temperature_thresholds(s_state.last_power_temp, s_state.last_control_temp);
            } else if (temp_ret == ESP_ERR_INVALID_STATE) {
                // 检查是否是因为没有传感器
                int sensor_count = temp_mgr_get_sensor_count();
                if (sensor_count == 0) {
                    ESP_LOGD(TAG, "No temperature sensors connected (GPIO%d)", TEMP_SENSOR_GPIO_NUM);
                    // 设置为指示没有传感器的特殊值
                    s_state.last_power_temp = -273.0f;  // 绝对零度表示无传感器
                    s_state.last_control_temp = -273.0f;
                } else {
                    ESP_LOGD(TAG, "Temperature data not yet available (sensors initializing)");
                    // 设置为合理的默认值以便显示
                    s_state.last_power_temp = 25.0f;  // 默认室温
                    s_state.last_control_temp = 25.0f; // 默认室温
                }
            } else {
                ESP_LOGW(TAG, "Failed to get temperature data: %s", esp_err_to_name(temp_ret));
                // 设置为错误指示值
                s_state.last_power_temp = -999.0f;
                s_state.last_control_temp = -999.0f;
            }
            last_temp_check = current_time;
        }

        // 处理事件队列
        system_event_t event;
        while (xQueueReceive(s_event_queue, &event, 0) == pdTRUE) {
            dispatch_event(&event);
        }

        // 每秒输出一次完整的监控数据到控制台
        if (current_time - last_console_output >= 1000) {
            ESP_LOGI(TAG, "=== MONITOR ===");

            // 输出电压信息
            if (s_state.last_voltage > 0) {
                ESP_LOGI(TAG, "🔋 Voltage: %.2fV %s",
                        s_state.last_voltage,
                        s_state.voltage_low_warning_active ?
                        (s_state.voltage_low_critical_active ? "[CRITICAL LOW]" : "[LOW]") : "[OK]");
            } else {
                ESP_LOGI(TAG, "🔋 Voltage: [NO DATA]");
            }

            // 输出电流信息 - 总电流
            if (s_state.last_total_current >= 0) {
                ESP_LOGI(TAG, "⚡ Total Current: %.2fA", s_state.last_total_current);
            } else {
                ESP_LOGI(TAG, "⚡ Total Current: [NO DATA]");
            }

            // 输出通道电流信息 - 独立显示，不受总电流限制
            ESP_LOGI(TAG, "⚡ Channel Currents:");
            for (int i = 0; i < 6; i++) {
                if (s_state.last_channel_currents[i] > -10.0f) {  // 允许-10A到+10A的范围，排除明显的错误值
                    ESP_LOGI(TAG, "   CH%d: %.3fA %s", i + 1, s_state.last_channel_currents[i],
                            (s_state.current_overload_active && s_state.overload_channel == i) ? "[OVERLOAD]" : "");
                } else {
                    ESP_LOGI(TAG, "   CH%d: [NO DATA]", i + 1);
                }
            }

            // 输出温度信息 - 两个区域
            if (s_state.last_power_temp > -200.0f && s_state.last_control_temp > -200.0f) {
                bool power_default = (s_state.last_power_temp == 25.0f);
                bool control_default = (s_state.last_control_temp == 25.0f);
                ESP_LOGI(TAG, "🌡️  Power Area: %.1f°C %s", s_state.last_power_temp,
                        power_default ? "[DEFAULT]" : "[OK]");
                ESP_LOGI(TAG, "🌡️  Control Area: %.1f°C %s", s_state.last_control_temp,
                        control_default ? "[DEFAULT]" : "[OK]");
            } else if (s_state.last_power_temp <= -272.0f || s_state.last_control_temp <= -272.0f) {
                ESP_LOGI(TAG, "🌡️  Temperature: [NO SENSORS]");
            } else {
                ESP_LOGI(TAG, "🌡️  Temperature: [ERROR]");
            }

            // 输出状态信息
            bool has_critical = system_monitor_has_critical_events();
            bool has_warnings = s_state.voltage_low_warning_active || s_state.temp_high_warning_active;

            if (has_critical || has_warnings) {
                ESP_LOGI(TAG, "⚠️  Status: %s%s%s%s",
                        has_critical ? "[CRITICAL] " : "",
                        s_state.voltage_low_critical_active ? "[VOLT_CRIT] " : "",
                        s_state.temp_high_critical_active ? "[TEMP_CRIT] " : "",
                        s_state.current_overload_active ? "[CURRENT_OVER] " : "");
            } else {
                ESP_LOGI(TAG, "✅ Status: [OK]");
            }

            last_console_output = current_time;
        }

        // 短暂延时
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "Monitor task finished");
    vTaskDelete(NULL);
}

// 创建并发送事件
static esp_err_t create_and_send_event(system_event_type_t type, float value, uint8_t channel, const char *description) {
    system_event_t event = {
        .type = type,
        .priority = get_event_priority(type),
        .timestamp = esp_timer_get_time() / 1000,
        .value = value,
        .channel = channel,
    };

    if (description) {
        strncpy(event.description, description, sizeof(event.description) - 1);
        event.description[sizeof(event.description) - 1] = '\0';
    } else {
        strncpy(event.description, get_event_description(type), sizeof(event.description) - 1);
        event.description[sizeof(event.description) - 1] = '\0';
    }

    // 发送到事件队列
    if (xQueueSend(s_event_queue, &event, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Event queue full, dropping event type %d", type);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

// 检查电压阈值
static esp_err_t check_voltage_thresholds(float voltage) {
    uint16_t voltage_mv = (uint16_t)(voltage * 1000);

    // 检查临界低电压
    if (voltage_mv < s_config.voltage_low_critical) {
        if (!s_state.voltage_low_critical_active) {
            s_state.voltage_low_critical_active = true;
            s_state.voltage_low_warning_active = true; // 临界时警告也激活
            ESP_LOGW(TAG, "CRITICAL: Low voltage detected: %.2fV", voltage);
            return create_and_send_event(SYSTEM_EVENT_VOLTAGE_LOW_CRITICAL, voltage, 0,
                                       "Critical low voltage");
        }
    }
    // 检查警告低电压
    else if (voltage_mv < s_config.voltage_low_warning) {
        if (!s_state.voltage_low_warning_active && !s_state.voltage_low_critical_active) {
            s_state.voltage_low_warning_active = true;
            ESP_LOGW(TAG, "WARNING: Low voltage detected: %.2fV", voltage);
            return create_and_send_event(SYSTEM_EVENT_VOLTAGE_LOW_WARNING, voltage, 0,
                                       "Low voltage warning");
        }
    }
    // 检查电压恢复
    else if (voltage_mv >= s_config.voltage_recovery) {
        if (s_state.voltage_low_warning_active || s_state.voltage_low_critical_active) {
            bool was_critical = s_state.voltage_low_critical_active;
            s_state.voltage_low_warning_active = false;
            s_state.voltage_low_critical_active = false;
            ESP_LOGI(TAG, "Voltage recovered: %.2fV", voltage);
            return create_and_send_event(SYSTEM_EVENT_VOLTAGE_RECOVERY, voltage, 0,
                                       was_critical ? "Voltage recovered from critical" : "Voltage recovered from warning");
        }
    }

    // 正常电压
    if (!s_state.voltage_low_warning_active && !s_state.voltage_low_critical_active) {
        return create_and_send_event(SYSTEM_EVENT_VOLTAGE_NORMAL, voltage, 0, "Voltage normal");
    }

    return ESP_OK;
}

// 检查电流阈值
static esp_err_t check_current_thresholds(const float *currents, uint8_t count) {
    uint32_t current_time = esp_timer_get_time() / 1000;

    for (uint8_t i = 0; i < count; i++) {
        // 忽略负值和零值，这些是无效或初始化状态
        if (currents[i] <= 0.0f) {
            continue;
        }

        uint16_t current_ma = (uint16_t)(currents[i] * 1000);

        if (current_ma > s_config.current_overload_threshold) {
            if (!s_state.current_overload_active) {
                // 开始过载计时
                s_state.overload_start_time = current_time;
                s_state.overload_channel = i;
                ESP_LOGW(TAG, "Current overload started on channel %d: %.2fA", i + 1, currents[i]);
            } else if (s_state.overload_channel == i) {
                // 检查是否超过持续时间
                if (current_time - s_state.overload_start_time >= s_config.current_overload_duration) {
                    if (!s_state.current_overload_active) {
                        s_state.current_overload_active = true;
                        ESP_LOGE(TAG, "CRITICAL: Sustained current overload on channel %d: %.2fA", i + 1, currents[i]);
                        return create_and_send_event(SYSTEM_EVENT_CURRENT_OVERLOAD, currents[i], i,
                                                   "Sustained current overload");
                    }
                }
            }
        } else {
            // 电流正常，检查是否需要重置过载状态
            if (s_state.current_overload_active && s_state.overload_channel == i) {
                s_state.current_overload_active = false;
                s_state.overload_start_time = 0;
                ESP_LOGI(TAG, "Current overload cleared on channel %d: %.2fA", i + 1, currents[i]);
                return create_and_send_event(SYSTEM_EVENT_CURRENT_NORMAL, currents[i], i,
                                           "Current normal");
            }
        }
    }

    return ESP_OK;
}

// 检查温度阈值
static esp_err_t check_temperature_thresholds(float power_temp, float control_temp) {
    uint16_t power_temp_centi = (uint16_t)(power_temp * 100);
    uint16_t control_temp_centi = (uint16_t)(control_temp * 100);
    uint16_t max_temp = (power_temp > control_temp) ? power_temp_centi : control_temp_centi;

    // 检查临界高温
    if (max_temp >= s_config.temp_high_critical) {
        if (!s_state.temp_high_critical_active) {
            s_state.temp_high_critical_active = true;
            s_state.temp_high_warning_active = true; // 临界时警告也激活
            ESP_LOGE(TAG, "CRITICAL: High temperature detected: Power=%.1f°C, Control=%.1f°C",
                    power_temp, control_temp);
            return create_and_send_event(SYSTEM_EVENT_TEMP_HIGH_CRITICAL, max_temp / 100.0f, 0,
                                       "Critical high temperature");
        }
    }
    // 检查警告高温
    else if (max_temp >= s_config.temp_high_warning) {
        if (!s_state.temp_high_warning_active && !s_state.temp_high_critical_active) {
            s_state.temp_high_warning_active = true;
            ESP_LOGW(TAG, "WARNING: High temperature detected: Power=%.1f°C, Control=%.1f°C",
                    power_temp, control_temp);
            return create_and_send_event(SYSTEM_EVENT_TEMP_HIGH_WARNING, max_temp / 100.0f, 0,
                                       "High temperature warning");
        }
    }
    // 检查温度恢复
    else if (max_temp <= s_config.temp_recovery) {
        if (s_state.temp_high_warning_active || s_state.temp_high_critical_active) {
            bool was_critical = s_state.temp_high_critical_active;
            s_state.temp_high_warning_active = false;
            s_state.temp_high_critical_active = false;
            ESP_LOGI(TAG, "Temperature recovered: Power=%.1f°C, Control=%.1f°C",
                    power_temp, control_temp);
            return create_and_send_event(SYSTEM_EVENT_TEMP_RECOVERY, max_temp / 100.0f, 0,
                                       was_critical ? "Temperature recovered from critical" : "Temperature recovered from warning");
        }
    }

    return ESP_OK;
}

// 分发事件
static esp_err_t dispatch_event(const system_event_t *event) {
    // 保存最新事件
    s_last_event = *event;

    // 记录事件
    ESP_LOGI(TAG, "Event: %s (%.2f, ch:%d, pri:%d)",
            event->description, event->value, event->channel, event->priority);

    // 查找并调用注册的处理器
    for (uint8_t i = 0; i < s_handler_count; i++) {
        if (s_event_handlers[i].event_type == event->type) {
            if (s_event_handlers[i].handler != NULL) {
                esp_err_t ret = s_event_handlers[i].handler(event, s_event_handlers[i].user_data);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Event handler returned error: %s", esp_err_to_name(ret));
                }
            }
        }
    }

    return ESP_OK;
}

// 获取事件描述
static const char* get_event_description(system_event_type_t type) {
    switch (type) {
        case SYSTEM_EVENT_VOLTAGE_NORMAL: return "Voltage normal";
        case SYSTEM_EVENT_VOLTAGE_LOW_WARNING: return "Low voltage warning";
        case SYSTEM_EVENT_VOLTAGE_LOW_CRITICAL: return "Critical low voltage";
        case SYSTEM_EVENT_VOLTAGE_RECOVERY: return "Voltage recovery";
        case SYSTEM_EVENT_CURRENT_OVERLOAD: return "Current overload";
        case SYSTEM_EVENT_CURRENT_NORMAL: return "Current normal";
        case SYSTEM_EVENT_TEMP_HIGH_WARNING: return "High temperature warning";
        case SYSTEM_EVENT_TEMP_HIGH_CRITICAL: return "Critical high temperature";
        case SYSTEM_EVENT_TEMP_RECOVERY: return "Temperature recovery";
        case SYSTEM_EVENT_EXTERNAL_POWER_LOST: return "External power lost";
        case SYSTEM_EVENT_EXTERNAL_POWER_RESTORED: return "External power restored";
        default: return "Unknown event";
    }
}

// 获取事件优先级
static event_priority_t get_event_priority(system_event_type_t type) {
    switch (type) {
        case SYSTEM_EVENT_VOLTAGE_LOW_CRITICAL:
        case SYSTEM_EVENT_CURRENT_OVERLOAD:
        case SYSTEM_EVENT_TEMP_HIGH_CRITICAL:
            return EVENT_PRIORITY_CRITICAL;

        case SYSTEM_EVENT_VOLTAGE_LOW_WARNING:
        case SYSTEM_EVENT_TEMP_HIGH_WARNING:
            return EVENT_PRIORITY_HIGH;

        case SYSTEM_EVENT_VOLTAGE_RECOVERY:
        case SYSTEM_EVENT_CURRENT_NORMAL:
        case SYSTEM_EVENT_TEMP_RECOVERY:
        case SYSTEM_EVENT_EXTERNAL_POWER_RESTORED:
            return EVENT_PRIORITY_MEDIUM;

        case SYSTEM_EVENT_VOLTAGE_NORMAL:
        case SYSTEM_EVENT_EXTERNAL_POWER_LOST:
            return EVENT_PRIORITY_LOW;

        default:
            return EVENT_PRIORITY_LOW;
    }
}
