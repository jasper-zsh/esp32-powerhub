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
#include "scheduler.h"
#include "led_status.h"
#include "power_mgr.h"
#include "adc128s102.h"
#include "temp_mgr.h"
#include "led_rgb.h"
#include "system_monitor.h"

static const char *TAG = "app";

// 电源管理模式配置 (ULP为当前模式)
#define ENABLE_ULP_MODE 1  // 1:启用ULP模式, 0:禁用ULP模式

// 子系统初始化状态跟踪
typedef struct {
    bool nvs_initialized;
    bool external_power_initialized;
    bool led_rgb_initialized;
    bool led_status_initialized;
    bool power_mgr_initialized;
    bool temp_mgr_initialized;
    bool pwm_control_initialized;
    bool adc128s102_initialized;
      bool scheduler_initialized;
    bool ble_server_initialized;
    bool adc_sampling_started;
    bool system_monitor_initialized;
    bool system_monitor_started;
} subsystem_status_t;

static subsystem_status_t g_subsys_status = {0};

// 子系统初始化函数声明
static esp_err_t init_nvs_flash(void);
static esp_err_t init_external_power(void);
static esp_err_t init_led_rgb(void);
static esp_err_t init_led_status(void);
static esp_err_t init_power_manager(void);
static esp_err_t init_temperature_manager(void);
static esp_err_t init_pwm_controller(void);
static esp_err_t init_adc128s102(void);
static esp_err_t init_scheduler(void);
static esp_err_t init_ble_server(void);
static esp_err_t start_adc_sampling(void);
static esp_err_t init_system_monitor(void);
static esp_err_t start_system_monitor(void);

void app_main(void) {
    // 设置调试日志级别
    esp_log_level_set("adc128s102", ESP_LOG_DEBUG);
    esp_log_level_set("app", ESP_LOG_DEBUG);

    // 检查重启原因
    esp_reset_reason_t reset_reason = esp_reset_reason();
    ESP_LOGI(TAG, "=== ESP32 Power Hub System Starting ===");
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

    ESP_LOGI(TAG, "=== Starting Subsystem Initialization ===");

    // 按照硬件要求和依赖关系顺序初始化各个子系统
    esp_err_t ret;

    // 1. 初始化NVS存储 (其他系统可能需要读取配置)
    ret = init_nvs_flash();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS initialization failed, system cannot continue");
        return;
    }

    // 2. 初始化外设电源控制 (为其他外设提供电源)
    ret = init_external_power();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "External power initialization failed");
        // 继续执行，但记录错误
    }

    // 3. 初始化LED RGB驱动 (状态指示)
    ret = init_led_rgb();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED RGB initialization failed");
        // 继续执行，但状态指示可能不可用
    }

    // 4. 初始化LED状态管理
    ret = init_led_status();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED status initialization failed");
        // 继续执行，但状态指示可能不可用
    }

    // 5. 配置电源管理模式 (ULP vs LightSleep)
    ESP_LOGI(TAG, "Configuring power management mode...");
    ret = power_mgr_enable_ulp_mode(ENABLE_ULP_MODE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure power mode: %s", esp_err_to_name(ret));
        return;  // 电源模式配置失败
    }
    ESP_LOGI(TAG, "Power mode configured: %s", ENABLE_ULP_MODE ? "ULP" : "ADC");

    // 6. 初始化电源管理器 (包括电压监测)
    ret = init_power_manager();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Power manager initialization failed");
        return;  // 电源管理器是关键组件
    }

    // 启动电源管理任务
    BaseType_t task_ret = xTaskCreate(power_mgr_task, "power_mgr", 4096, NULL, 5, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create power manager task");
        return;
    }
    ESP_LOGI(TAG, "Power manager task started");

    // 7. 初始化温度管理器
    ret = init_temperature_manager();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Temperature manager initialization failed");
        // 继续执行，但温度保护不可用
    }

    // 8. 初始化PWM控制器
    ret = init_pwm_controller();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM controller initialization failed");
        return;  // PWM是核心功能
    }

    // 9. 初始化ADC128S102 (电流监测)
    ret = init_adc128s102();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC128S102 initialization failed");
        // 继续执行，但电流监测不可用
    }

    // 10. 初始化系统监控 (在ADC初始化之后)
    ret = init_system_monitor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "System monitor initialization failed");
        // 继续执行，但监控功能不可用
    }

  
    // 11. 初始化调度器
    ret = init_scheduler();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Scheduler initialization failed");
        return;  // 调度器是关键组件
    }

    // 12. 初始化BLE服务器
    ret = init_ble_server();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE server initialization failed");
        return;  // BLE是主要接口
    }

    // 14. 启动ADC采样
    ret = start_adc_sampling();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC sampling start failed");
        // 继续执行，但电流数据不可用
    }

    // 15. 系统监控已经在初始化时启动

    // 设置初始系统状态
    led_status_set_bluetooth_connected(false);
    led_status_set_error(false);
    ESP_LOGI(TAG, "Initial system states set");

    ESP_LOGI(TAG, "=== System Initialization Complete ===");
    ESP_LOGI(TAG, "System ready and operational");

    // 主循环 - 任务已经启动，这里只是保持主任务运行
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));  // 30秒间隔
        ESP_LOGD(TAG, "Main task running... All subsystems operational");

        // 可以在这里添加系统状态检查
        ESP_LOGD(TAG, "Subsystem Status: NVS=%d, Power=%d, LED=%d, PWM=%d, ADC=%d, BLE=%d",
                 g_subsys_status.nvs_initialized,
                 g_subsys_status.power_mgr_initialized,
                 g_subsys_status.led_status_initialized,
                 g_subsys_status.pwm_control_initialized,
                 g_subsys_status.adc128s102_initialized,
                 g_subsys_status.ble_server_initialized);
    }
}

// ===== 子系统初始化函数实现 =====

/**
 * @brief 初始化NVS存储系统
 * @note 其他系统可能需要读取配置，所以优先初始化
 */
static esp_err_t init_nvs_flash(void) {
    ESP_LOGI(TAG, "[1/13] Initializing NVS flash...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if (ret == ESP_OK) {
        g_subsys_status.nvs_initialized = true;
        ESP_LOGI(TAG, "✓ NVS flash initialized successfully");
    } else {
        ESP_LOGE(TAG, "✗ NVS flash initialization failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 初始化外设电源控制
 * @note GPIO1控制所有外设电源，需要在其他外设初始化前开启
 */
static esp_err_t init_external_power(void) {
    ESP_LOGI(TAG, "[2/13] Initializing external power control...");

    esp_err_t ret = power_mgr_external_power_init();
    if (ret == ESP_OK) {
        g_subsys_status.external_power_initialized = true;
        ESP_LOGI(TAG, "✓ External power control initialized (GPIO1)");
    } else {
        ESP_LOGE(TAG, "✗ External power control initialization failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 初始化LED RGB驱动
 * @note 用于系统状态指示
 */
static esp_err_t init_led_rgb(void) {
    ESP_LOGI(TAG, "[3/13] Initializing LED RGB driver...");

    esp_err_t ret = led_rgb_init();
    if (ret == ESP_OK) {
        g_subsys_status.led_rgb_initialized = true;
        ESP_LOGI(TAG, "✓ LED RGB driver initialized (GPIO21)");
    } else {
        ESP_LOGE(TAG, "✗ LED RGB driver initialization failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 初始化LED状态管理
 * @note 管理LED状态显示逻辑
 */
static esp_err_t init_led_status(void) {
    ESP_LOGI(TAG, "[4/13] Initializing LED status manager...");

    esp_err_t ret = led_status_init();
    if (ret == ESP_OK) {
        g_subsys_status.led_status_initialized = true;
        ESP_LOGI(TAG, "✓ LED status manager initialized");
    } else {
        ESP_LOGE(TAG, "✗ LED status manager initialization failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 初始化电源管理器
 * @note 包括电压监测、外设电源控制、温度保护等
 */
static esp_err_t init_power_manager(void) {
    ESP_LOGI(TAG, "[5/13] Initializing power manager...");

    esp_err_t ret = power_mgr_init();
    if (ret == ESP_OK) {
        g_subsys_status.power_mgr_initialized = true;
        ESP_LOGI(TAG, "✓ Power manager initialized (voltage monitoring & thermal protection)");
    } else {
        ESP_LOGE(TAG, "✗ Power manager initialization failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 初始化温度管理器
 * @note GPIO7上的DS18B20温度传感器
 */
static esp_err_t init_temperature_manager(void) {
    ESP_LOGI(TAG, "[6/13] Initializing temperature manager...");

    // 注意：这个初始化已经在power_mgr_init中调用
    // 这里只做状态检查，因为实际的temp_mgr_init()由power_mgr控制
    g_subsys_status.temp_mgr_initialized = true;
    ESP_LOGI(TAG, "✓ Temperature manager initialized (GPIO%d - %d x DS18B20)", TEMP_SENSOR_GPIO_NUM, TEMP_SENSOR_COUNT);

    return ESP_OK;
}

/**
 * @brief 初始化PWM控制器
 * @note 6通道PWM输出，GPIO8-13
 */
static esp_err_t init_pwm_controller(void) {
    ESP_LOGI(TAG, "[7/13] Initializing PWM controller...");

    esp_err_t ret = pwm_control_init();
    if (ret == ESP_OK) {
        g_subsys_status.pwm_control_initialized = true;
        ESP_LOGI(TAG, "✓ PWM controller initialized (6 channels, GPIO8-13)");
    } else {
        ESP_LOGE(TAG, "✗ PWM controller initialization failed: %s", esp_err_to_name(ret));
    }

    return ret;
}


/**
 * @brief 初始化ADC128S102
 * @note 外置ADC，GPIO3-6 (SPI)，用于电流监测
 */
static esp_err_t init_adc128s102(void) {
    ESP_LOGI(TAG, "[8/12] Initializing ADC128S102...");

    esp_err_t ret = adc128s102_init();
    if (ret == ESP_OK) {
        g_subsys_status.adc128s102_initialized = true;
        ESP_LOGI(TAG, "✓ ADC128S102 initialized (SPI: GPIO3-6)");
    } else {
        ESP_LOGE(TAG, "✗ ADC128S102 initialization failed: %s", esp_err_to_name(ret));
    }

    return ret;
}


/**
 * @brief 初始化调度器
 * @note 任务调度管理
 */
static esp_err_t init_scheduler(void) {
    ESP_LOGI(TAG, "[9/12] Initializing scheduler...");

    esp_err_t ret = scheduler_init();
    if (ret == ESP_OK) {
        g_subsys_status.scheduler_initialized = true;
        ESP_LOGI(TAG, "✓ Scheduler initialized");
    } else {
        ESP_LOGE(TAG, "✗ Scheduler initialization failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 初始化BLE服务器
 * @note 蓝牙通信服务
 */
static esp_err_t init_ble_server(void) {
    ESP_LOGI(TAG, "[10/12] Initializing BLE server...");

    esp_err_t ret = ble_server_init();
    if (ret == ESP_OK) {
        g_subsys_status.ble_server_initialized = true;
        ESP_LOGI(TAG, "✓ BLE server initialized");
    } else {
        ESP_LOGE(TAG, "✗ BLE server initialization failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 初始化系统监控
 * @note 统一监控电压、电流、温度
 */
static esp_err_t init_system_monitor(void) {
    ESP_LOGI(TAG, "[7/13] Initializing system monitor...");

    esp_err_t ret = system_monitor_init();
    if (ret == ESP_OK) {
        g_subsys_status.system_monitor_initialized = true;
        ESP_LOGI(TAG, "✓ System monitor initialized");

        // 立即启动监控任务
        ret = system_monitor_start();
        if (ret == ESP_OK) {
            g_subsys_status.system_monitor_started = true;
            ESP_LOGI(TAG, "✓ System monitor started");
        } else {
            ESP_LOGE(TAG, "✗ System monitor start failed: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "✗ System monitor initialization failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 启动系统监控
 * @note 启动监控任务
 */
static esp_err_t start_system_monitor(void) {
    ESP_LOGI(TAG, "[13/13] Starting system monitor...");

    esp_err_t ret = ESP_OK;
    if (g_subsys_status.system_monitor_initialized) {
        // 注册电源管理器为事件处理器
        ret = system_monitor_register_handler(SYSTEM_EVENT_VOLTAGE_LOW_WARNING,
                                            (system_event_handler_t)power_mgr_handle_system_event, NULL);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to register voltage warning handler");
        }

        ret = system_monitor_register_handler(SYSTEM_EVENT_VOLTAGE_LOW_CRITICAL,
                                            (system_event_handler_t)power_mgr_handle_system_event, NULL);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to register voltage critical handler");
        }

        ret = system_monitor_register_handler(SYSTEM_EVENT_VOLTAGE_RECOVERY,
                                            (system_event_handler_t)power_mgr_handle_system_event, NULL);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to register voltage recovery handler");
        }

        ret = system_monitor_register_handler(SYSTEM_EVENT_CURRENT_OVERLOAD,
                                            (system_event_handler_t)power_mgr_handle_system_event, NULL);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to register current overload handler");
        }

        ret = system_monitor_register_handler(SYSTEM_EVENT_TEMP_HIGH_CRITICAL,
                                            (system_event_handler_t)power_mgr_handle_system_event, NULL);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to register temperature critical handler");
        }

        // 启动监控任务
        ret = system_monitor_start();
        if (ret == ESP_OK) {
            g_subsys_status.system_monitor_started = true;
            ESP_LOGI(TAG, "✓ System monitor started");
        } else {
            ESP_LOGE(TAG, "✗ System monitor start failed: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "System monitor not initialized, skipping start");
        ret = ESP_ERR_NOT_FOUND;
    }

    return ret;
}

/**
 * @brief 启动ADC采样
 * @note 开始连续电流数据采集
 */
static esp_err_t start_adc_sampling(void) {
    ESP_LOGI(TAG, "[12/13] Starting ADC sampling...");

    esp_err_t ret = ESP_OK;
    if (g_subsys_status.adc128s102_initialized) {
        ret = adc128s102_start_continuous_sampling(1000); // 1秒间隔
        if (ret == ESP_OK) {
            g_subsys_status.adc_sampling_started = true;
            ESP_LOGI(TAG, "✓ ADC sampling started (1s interval)");
        } else {
            ESP_LOGE(TAG, "✗ ADC sampling start failed: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "ADC128S102 not initialized, skipping sampling start");
        ret = ESP_ERR_NOT_FOUND;
    }

    return ret;
}
