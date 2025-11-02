#include "power_mgr.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "soc/adc_channel.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "host/ble_hs.h"

#include "led_status.h"
#include "pwm_control.h"
#include "storage.h"
#include "temp_mgr.h"
#include "scheduler.h"

// ESP32C6使用LightSleep模式替代ULP进行电压监测

static const char *TAG = "power_mgr";

// 外设电源控制引脚
#define EXTERNAL_POWER_CTRL_GPIO    0  // GPIO0控制所有外设电源

// NVS配置键
#define NVS_NS "power_mgr"
#define KEY_SLEEP "sleep_mv"
#define KEY_WAKE  "wake_mv"
#define KEY_CAL   "cal_off"
#define KEY_T_HI  "t_hi"
#define KEY_T_REC "t_rec"

#define DEFAULT_SLEEP_MV 11000  // 11.0V 睡眠阈值
#define DEFAULT_WAKE_MV  12500  // 12.5V 唤醒阈值
#define DEFAULT_TEMP_HI  6000   // 60.00°C
#define DEFAULT_TEMP_REC 5500   // 55.00°C

#define SAMPLE_INTERVAL_MS 1000
#define TEMP_DEBOUNCE_COUNT 3

static uint16_t s_sleep_mv = DEFAULT_SLEEP_MV;
static uint16_t s_wake_mv  = DEFAULT_WAKE_MV;
static int16_t s_cal_off_mv = 0;

// 温度相关变量
static int16_t s_temp_hi = DEFAULT_TEMP_HI;
static int16_t s_temp_rec = DEFAULT_TEMP_REC;
static int16_t s_latest_power_temp = 0;    // 电源区域温度
static int16_t s_latest_control_temp = 0;  // 控制区域温度
static bool s_temp_valid = false;          // 温度数据是否有效（任一传感器）
static bool s_power_temp_valid = false;    // 电源区域温度是否有效
static bool s_control_temp_valid = false;  // 控制区域温度是否有效
static bool s_thermal_protection = false;
static int s_hot_debounce_count = 0;
static int s_cool_debounce_count = 0;

// BLE通知相关
static bool s_ble_notifications_enabled = false;
static uint16_t s_ble_conn_handle = 0;
static uint16_t s_ble_attr_handle = 0;

// 外设电源状态
static bool s_external_power_enabled = false;

// ADC相关变量
static uint32_t s_last_voltage_mv = 0;
static bool s_wake_flag = false;

// ADC句柄
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_adc_cali_handle = NULL;

// 前向声明
static void thermal_protection_trigger(void);
static void thermal_protection_recovery(void);

static esp_err_t nvs_load_thresholds(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) return err;
    uint16_t v;
    if (nvs_get_u16(h, KEY_SLEEP, &v) == ESP_OK) s_sleep_mv = v;
    if (nvs_get_u16(h, KEY_WAKE,  &v) == ESP_OK) s_wake_mv  = v;
    
    // 加载温度阈值
    int16_t temp_val;
    if (nvs_get_i16(h, KEY_T_HI, &temp_val) == ESP_OK) {
        s_temp_hi = temp_val;
    }
    if (nvs_get_i16(h, KEY_T_REC, &temp_val) == ESP_OK) {
        s_temp_rec = temp_val;
    }
    nvs_close(h);
    return ESP_OK;
}

static esp_err_t nvs_store_thresholds(void) {
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NS, NVS_READWRITE, &h), TAG, "nvs open");
    ESP_RETURN_ON_ERROR(nvs_set_u16(h, KEY_SLEEP, s_sleep_mv), TAG, "set sleep");
    ESP_RETURN_ON_ERROR(nvs_set_u16(h, KEY_WAKE,  s_wake_mv),  TAG, "set wake");
    ESP_RETURN_ON_ERROR(nvs_set_blob(h, KEY_CAL, &s_cal_off_mv, sizeof(s_cal_off_mv)), TAG, "set cal");
    ESP_RETURN_ON_ERROR(nvs_set_i16(h, KEY_T_HI, s_temp_hi), TAG, "set temp_hi");
    ESP_RETURN_ON_ERROR(nvs_set_i16(h, KEY_T_REC, s_temp_rec), TAG, "set temp_rec");
    ESP_RETURN_ON_ERROR(nvs_commit(h), TAG, "commit");
    nvs_close(h);
    return ESP_OK;
}

// ADC初始化函数
static esp_err_t adc_init(void) {
    // ADC初始化
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &s_adc_handle));

    // 配置ADC通道
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC1_GPIO1_CHANNEL, &config));

    // ADC校准初始化
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &s_adc_cali_handle));

    return ESP_OK;
}

// 读取电压值
static uint16_t adc_read_voltage_mv(void) {
    int raw_value;
    int voltage_mv;

    if (adc_oneshot_read(s_adc_handle, ADC1_GPIO1_CHANNEL, &raw_value) == ESP_OK) {
        if (adc_cali_raw_to_voltage(s_adc_cali_handle, raw_value, &voltage_mv) == ESP_OK) {
            // 分压比计算 (240k:910k = 1:4.7917)
            s_last_voltage_mv = (uint32_t)(voltage_mv * 4.7917f) + s_cal_off_mv;
        }
    }

    return (uint16_t)s_last_voltage_mv;
}

// LightSleep定时器唤醒任务
static void lightsleep_monitor_task(void *arg) {
    const TickType_t sleep_interval = pdMS_TO_TICKS(5000); // 15秒检查一次
    const TickType_t startup_delay = pdMS_TO_TICKS(10000);  // 启动延迟60秒
    const int low_voltage_count_threshold = 5; // 连续5次低电压才进入睡眠
    int low_voltage_count = 0;

    ESP_LOGI(TAG, "LightSleep monitor task started, waiting %d seconds for system stabilization",
             10);
    ESP_LOGI(TAG, "Configuration: sleep=%umV, wake=%umV, check_interval=%us, threshold_count=%d",
             s_sleep_mv, s_wake_mv, 5, low_voltage_count_threshold);

    // 启动延迟，给系统充分时间稳定
    vTaskDelay(startup_delay);

    ESP_LOGI(TAG, "LightSleep monitor task beginning voltage monitoring");

    while (1) {
        // 读取电压
        uint16_t current_voltage = adc_read_voltage_mv();
        ESP_LOGI(TAG, "Voltage check: %umV (sleep=%u, wake=%u, low_count=%d)",
                 current_voltage, s_sleep_mv, s_wake_mv, low_voltage_count);

        // 检查是否达到唤醒条件
        if (current_voltage >= s_wake_mv && s_wake_flag == false) {
            s_wake_flag = true;
            low_voltage_count = 0; // 重置低电压计数
            ESP_LOGI(TAG, "Voltage threshold reached, system is now active");
        } else if (current_voltage < s_wake_mv && s_wake_flag == true) {
            // 如果电压降到唤醒阈值以下，标记为非活跃状态
            s_wake_flag = false;
            ESP_LOGI(TAG, "Voltage dropped below wake threshold, monitoring for sleep");
        }

        // 只有在电压低于睡眠阈值且系统非活跃时才考虑睡眠
        if (current_voltage < s_sleep_mv && s_wake_flag == false) {
            low_voltage_count++;
            ESP_LOGW(TAG, "Low voltage detected (%d/%d)", low_voltage_count, low_voltage_count_threshold);

            // 连续检测到低电压才进入睡眠
            if (low_voltage_count >= low_voltage_count_threshold) {
                ESP_LOGW(TAG, "Entering LightSleep mode due to persistent low voltage");

                // 配置LightSleep
                esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

                // 设置定时器唤醒 (60秒后检查)
                const uint64_t sleep_duration_us = 60000000; // 60秒
                esp_sleep_enable_timer_wakeup(sleep_duration_us);

                // 关闭LED指示以节省功耗
                led_status_enable(false);

                // 进入LightSleep
                esp_err_t sleep_result = esp_light_sleep_start();

                // 从LightSleep唤醒后重新启用LED
                led_status_enable(true);
                led_status_set_low_battery(true);

                if (sleep_result == ESP_OK) {
                    ESP_LOGI(TAG, "Woke up from LightSleep, checking voltage again");
                } else {
                    ESP_LOGE(TAG, "LightSleep failed: %s", esp_err_to_name(sleep_result));
                }

                // 重置计数器
                low_voltage_count = 0;
            }
        } else {
            // 电压正常，重置低电压计数
            low_voltage_count = 0;
            led_status_set_low_battery(current_voltage < s_sleep_mv);
        }

        // 温度管理 - 同时读取两个区域的温度传感器
        int16_t temps[TEMP_SENSOR_COUNT];
        esp_err_t temp_err = temp_mgr_sample_all(temps);

        // 重置有效性标志
        s_power_temp_valid = false;
        s_control_temp_valid = false;

        if (temp_err == ESP_OK) {
            // 更新温度数据
            if (temp_mgr_get_sensor_count() >= 1) {
                s_latest_power_temp = temps[TEMP_SENSOR_POWER];
                s_power_temp_valid = true;
                ESP_LOGI(TAG, "Power area temperature: %d.%02d°C",
                        s_latest_power_temp / 100, abs(s_latest_power_temp % 100));
            }

            if (temp_mgr_get_sensor_count() >= 2) {
                s_latest_control_temp = temps[TEMP_SENSOR_CONTROL];
                s_control_temp_valid = true;
                ESP_LOGI(TAG, "Control area temperature: %d.%02d°C",
                        s_latest_control_temp / 100, abs(s_latest_control_temp % 100));
            }

            // 只要任一传感器有效，就认为温度数据有效
            s_temp_valid = s_power_temp_valid || s_control_temp_valid;

            // 热保护状态机 - 优先使用控制区域温度，如果不可用则使用电源区域温度
            int16_t monitor_temp = 0;
            bool use_control_temp = s_control_temp_valid;

            if (use_control_temp) {
                monitor_temp = s_latest_control_temp;
                ESP_LOGD(TAG, "Using control area temperature for thermal protection");
            } else if (s_power_temp_valid) {
                monitor_temp = s_latest_power_temp;
                ESP_LOGD(TAG, "Using power area temperature for thermal protection (control sensor unavailable)");
            } else {
                ESP_LOGW(TAG, "No temperature sensors available for thermal protection");
                s_temp_valid = false;
            }

            if (s_temp_valid) {
                if (!s_thermal_protection) {
                    // 检查是否超温
                    if (monitor_temp >= s_temp_hi) {
                        s_hot_debounce_count++;
                        s_cool_debounce_count = 0;
                        ESP_LOGD(TAG, "Temperature %d.%02d°C >= high threshold %d.%02d°C (count: %d/%d)",
                                monitor_temp / 100, abs(monitor_temp % 100),
                                s_temp_hi / 100, abs(s_temp_hi % 100),
                                s_hot_debounce_count, TEMP_DEBOUNCE_COUNT);
                        if (s_hot_debounce_count >= TEMP_DEBOUNCE_COUNT) {
                            thermal_protection_trigger();
                        }
                    } else {
                        s_hot_debounce_count = 0;
                    }
                } else {
                    // 检查是否可以恢复
                    if (monitor_temp <= s_temp_rec) {
                        s_cool_debounce_count++;
                        s_hot_debounce_count = 0;
                        ESP_LOGD(TAG, "Temperature %d.%02d°C <= recovery threshold %d.%02d°C (count: %d/%d)",
                                monitor_temp / 100, abs(monitor_temp % 100),
                                s_temp_rec / 100, abs(s_temp_rec % 100),
                                s_cool_debounce_count, TEMP_DEBOUNCE_COUNT);
                        if (s_cool_debounce_count >= TEMP_DEBOUNCE_COUNT) {
                            thermal_protection_recovery();
                        }
                    } else {
                        s_cool_debounce_count = 0;
                    }
                }
            }
        } else {
            if (temp_err != ESP_OK) {
                ESP_LOGD(TAG, "Temperature sampling failed: %s", esp_err_to_name(temp_err));
            }
        }

        // BLE通知
        if (s_ble_notifications_enabled && s_ble_conn_handle != 0) {
            uint8_t notify_data[14];  // 扩展以容纳两个温度
            // 电压（mV）
            notify_data[0] = (current_voltage >> 8) & 0xFF;
            notify_data[1] = current_voltage & 0xFF;
            // 电源区域温度（0.01°C）
            notify_data[2] = (s_latest_power_temp >> 8) & 0xFF;
            notify_data[3] = s_latest_power_temp & 0xFF;
            // 控制区域温度（0.01°C）
            notify_data[4] = (s_latest_control_temp >> 8) & 0xFF;
            notify_data[5] = s_latest_control_temp & 0xFF;
            // 高温阈值
            notify_data[6] = (s_temp_hi >> 8) & 0xFF;
            notify_data[7] = s_temp_hi & 0xFF;
            // 恢复阈值
            notify_data[8] = (s_temp_rec >> 8) & 0xFF;
            notify_data[9] = s_temp_rec & 0xFF;
            // 睡眠电压阈值
            notify_data[10] = (s_sleep_mv >> 8) & 0xFF;
            notify_data[11] = s_sleep_mv & 0xFF;
            // 唤醒电压阈值
            notify_data[12] = (s_wake_mv >> 8) & 0xFF;
            notify_data[13] = s_wake_mv & 0xFF;

            struct os_mbuf *om = ble_hs_mbuf_from_flat(notify_data, sizeof(notify_data));
            if (om != NULL) {
                ble_gattc_notify_custom(s_ble_conn_handle, s_ble_attr_handle, om);
            }
        }

        // 等待下一次检查
        vTaskDelay(sleep_interval);
    }
}

esp_err_t power_mgr_init(void) {
    nvs_load_thresholds();

    // 初始化ADC
    ESP_ERROR_CHECK(adc_init());

    // 进行初始电压读取
    s_last_voltage_mv = adc_read_voltage_mv();

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "Wake cause: %d, initial voltage: %u mV", cause, s_last_voltage_mv);

    // 根据初始电压设置系统状态
    if (s_last_voltage_mv >= s_wake_mv) {
        s_wake_flag = true;
        ESP_LOGI(TAG, "Initial voltage sufficient, system starts in active mode");
    } else {
        s_wake_flag = false;
        ESP_LOGW(TAG, "Initial voltage low, system starts in monitoring mode");
    }

    led_status_enable(true);
    led_status_set_low_battery(false);

    // 初始化外设电源控制
    esp_err_t power_err = power_mgr_external_power_init();
    if (power_err != ESP_OK) {
        ESP_LOGE(TAG, "External power init failed: %s", esp_err_to_name(power_err));
        return power_err;
    }
    
    // 初始化温度管理器
    esp_err_t temp_err = temp_mgr_init();
    if (temp_err != ESP_OK) {
        ESP_LOGW(TAG, "Temperature manager init failed: %s", esp_err_to_name(temp_err));
    }

    // 启动LightSleep监控任务
    BaseType_t ret = xTaskCreate(lightsleep_monitor_task, "lightsleep_monitor",
                                 3072, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create lightsleep monitor task");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "LightSleep monitor task started");

    return ESP_OK;
}

static void thermal_protection_trigger(void) {
    if (s_thermal_protection) {
        return; // 已经在热保护状态
    }
    
    ESP_LOGW(TAG, "Thermal protection triggered - Power: %d.%02d°C, Control: %d.%02d°C",
             s_latest_power_temp / 100, abs(s_latest_power_temp % 100),
             s_latest_control_temp / 100, abs(s_latest_control_temp % 100));
    
    s_thermal_protection = true;
    
    // 保存所有通道的快照并关闭
    for (uint8_t ch = 0; ch < PWM_CHANNEL_COUNT; ch++) {
        control_cmd_t snapshot;
        esp_err_t err = scheduler_get_channel_snapshot(ch, &snapshot);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to get snapshot for channel %u", ch);
        }
        
        // 关闭通道
        pwm_control_stop_pattern(ch);
        pwm_control_apply(ch, 0, 0);
    }
    
    ESP_LOGI(TAG, "All channels shut down for thermal protection");
}

static void thermal_protection_recovery(void) {
    if (!s_thermal_protection) {
        return; // 不在热保护状态
    }
    
    ESP_LOGI(TAG, "Thermal protection recovery - Power: %d.%02d°C, Control: %d.%02d°C",
             s_latest_power_temp / 100, abs(s_latest_power_temp % 100),
             s_latest_control_temp / 100, abs(s_latest_control_temp % 100));
    
    s_thermal_protection = false;
    
    // 恢复所有通道的快照
    for (uint8_t ch = 0; ch < PWM_CHANNEL_COUNT; ch++) {
        control_cmd_t snapshot;
        esp_err_t err = scheduler_get_channel_snapshot(ch, &snapshot);
        if (err == ESP_OK) {
            err = scheduler_restore_channel_snapshot(ch, &snapshot);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to restore channel %u: %s", ch, esp_err_to_name(err));
            }
        }
    }
    
    ESP_LOGI(TAG, "All channels restored from thermal protection");
}

// 旧的power_mgr_task已被弃用，功能已迁移到lightsleep_monitor_task
// void power_mgr_task(void *arg) { ... }

esp_err_t power_mgr_get_voltage_mv(uint16_t *out_mv, bool force_refresh) {
    if (!out_mv) return ESP_ERR_INVALID_ARG;
    uint16_t vin_mv = adc_read_voltage_mv();
    *out_mv = vin_mv;
    return ESP_OK;
}

esp_err_t power_mgr_set_thresholds(uint16_t sleep_mv, uint16_t wake_mv) {
    if (sleep_mv == 0 || wake_mv == 0 || wake_mv <= sleep_mv) return ESP_ERR_INVALID_ARG;
    s_sleep_mv = sleep_mv;
    s_wake_mv = wake_mv;
    return nvs_store_thresholds();
}

esp_err_t power_mgr_set_cal_offset_mv(int16_t off_mv) {
    if (off_mv < -2000 || off_mv > 2000) return ESP_ERR_INVALID_ARG;
    s_cal_off_mv = off_mv;
    return nvs_store_thresholds();
}

esp_err_t power_mgr_force_sleep(void) {
    ESP_LOGI(TAG, "Entering LightSleep mode");
    uint8_t states[6];
    storage_read_states(states);
    storage_write_states(states);
    (void)led_status_set_bluetooth_advertising(false);
    (void)led_status_set_bluetooth_connected(false);
    (void)led_status_set_low_battery(false);
    (void)led_status_enable(false);

    // 关闭温度传感器
    temp_mgr_deinit();

    // 清理热保护状态
    s_thermal_protection = false;
    s_hot_debounce_count = 0;
    s_cool_debounce_count = 0;

    // 重置唤醒标志
    s_wake_flag = false;

    // 配置LightSleep
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    // 设置定时器唤醒 (30秒后检查)
    const uint64_t sleep_duration_us = 30000000; // 30秒
    esp_sleep_enable_timer_wakeup(sleep_duration_us);

    ESP_LOGI(TAG, "Entering LightSleep for %llu microseconds", sleep_duration_us);

    // 进入LightSleep
    esp_light_sleep_start();

    ESP_LOGI(TAG, "Woke up from forced LightSleep");

    // 重新初始化LED状态
    led_status_enable(true);
    led_status_set_low_battery(s_last_voltage_mv < s_sleep_mv);

    return ESP_OK;
}

int power_mgr_ble_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt) {
    // 保存连接信息用于通知
    s_ble_conn_handle = conn;
    s_ble_attr_handle = attr;
    
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // v005: 返回16字节的扩展状态
        uint8_t response[16];
        
        // uint16 电压（mV）
        uint16_t vin_mv;
        power_mgr_get_voltage_mv(&vin_mv, false);
        response[0] = (vin_mv >> 8) & 0xFF;
        response[1] = vin_mv & 0xFF;
        
        // int16 电源区域温度（0.01°C）
        response[2] = (s_latest_power_temp >> 8) & 0xFF;
        response[3] = s_latest_power_temp & 0xFF;

        // int16 控制区域温度（0.01°C）
        response[4] = (s_latest_control_temp >> 8) & 0xFF;
        response[5] = s_latest_control_temp & 0xFF;
        
        // int16 高温阈值（0.01°C）
        response[6] = (s_temp_hi >> 8) & 0xFF;
        response[7] = s_temp_hi & 0xFF;

        // int16 恢复阈值（0.01°C）
        response[8] = (s_temp_rec >> 8) & 0xFF;
        response[9] = s_temp_rec & 0xFF;
        
        // uint16 睡眠电压阈值（mV）
        response[10] = (s_sleep_mv >> 8) & 0xFF;
        response[11] = s_sleep_mv & 0xFF;

        // uint16 唤醒电压阈值（mV）
        response[12] = (s_wake_mv >> 8) & 0xFF;
        response[13] = s_wake_mv & 0xFF;

        // uint8 状态标志
        uint8_t status_flags = 0;
        if (s_thermal_protection) status_flags |= 0x01;     // bit0: 热保护中
        if (s_power_temp_valid) status_flags |= 0x02;      // bit1: 电源区域温度有效
        if (s_control_temp_valid) status_flags |= 0x04;    // bit2: 控制区域温度有效
        // 其余位保留
        response[14] = status_flags;
        response[15] = 0;
        
        return os_mbuf_append(ctxt->om, response, sizeof(response)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t data[3] = {0};
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len != 3) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        uint16_t copied = 0;
        if (ble_hs_mbuf_to_flat(ctxt->om, data, sizeof(data), &copied) != 0 || copied != 3) return BLE_ATT_ERR_UNLIKELY;
        
        uint8_t cmd = data[0];
        int16_t param = (int16_t)((data[1] << 8) | data[2]);
        esp_err_t err = ESP_OK;
        
        switch (cmd) {
            // 原有命令（兼容）
            case 0x01: err = power_mgr_set_thresholds((uint16_t)param, s_wake_mv); break;
            case 0x02: err = power_mgr_set_thresholds(s_sleep_mv, (uint16_t)param); break;
            case 0x03: err = power_mgr_force_sleep(); break;
            case 0x04: err = ESP_OK; break; // 强制唤醒（无操作）
            
            // v004新增命令
            case 0x11: // 设置高温阈值
                s_temp_hi = param;
                err = nvs_store_thresholds();
                break;
                
            case 0x12: // 设置恢复阈值
                s_temp_rec = param;
                err = nvs_store_thresholds();
                break;
                
            default: 
                return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
        }
        
        return err == ESP_OK ? 0 : BLE_ATT_ERR_UNLIKELY;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC) {
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

void power_mgr_ble_subscribe(uint16_t conn_handle, uint16_t attr_handle, bool enabled) {
    if (enabled) {
        s_ble_notifications_enabled = true;
        s_ble_conn_handle = conn_handle;
        s_ble_attr_handle = attr_handle;
        ESP_LOGI(TAG, "BLE notifications enabled (conn=%u attr=%u)", conn_handle, attr_handle);
        return;
    }

    s_ble_notifications_enabled = false;

    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || s_ble_conn_handle == conn_handle) {
        s_ble_conn_handle = 0;
    }

    if (attr_handle != 0) {
        s_ble_attr_handle = attr_handle;
    }

    ESP_LOGI(TAG, "BLE notifications disabled");
}

esp_err_t power_mgr_get_temperature(int16_t *out_temp) {
    if (out_temp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 兼容性接口：优先返回控制区域温度，如果不可用则返回电源区域温度
    if (s_control_temp_valid) {
        *out_temp = s_latest_control_temp;
        return ESP_OK;
    } else if (s_power_temp_valid) {
        *out_temp = s_latest_power_temp;
        return ESP_OK;
    } else {
        return ESP_ERR_INVALID_STATE;
    }
}

esp_err_t power_mgr_get_temperatures(int16_t *power_temp, int16_t *control_temp) {
    if (power_temp == NULL || control_temp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *power_temp = s_latest_power_temp;
    *control_temp = s_latest_control_temp;

    // 返回任一传感器的状态
    return (s_power_temp_valid || s_control_temp_valid) ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t power_mgr_set_temp_thresholds(int16_t high_temp, int16_t recover_temp) {
    if (high_temp <= recover_temp) {
        return ESP_ERR_INVALID_ARG;
    }
    
    s_temp_hi = high_temp;
    s_temp_rec = recover_temp;
    
    // 重置去抖计数器
    s_hot_debounce_count = 0;
    s_cool_debounce_count = 0;
    
    return nvs_store_thresholds();
}

esp_err_t power_mgr_get_temp_thresholds(int16_t *high_temp, int16_t *recover_temp) {
    if (high_temp == NULL || recover_temp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *high_temp = s_temp_hi;
    *recover_temp = s_temp_rec;
    return ESP_OK;
}

bool power_mgr_is_thermal_protection_active(void) {
    return s_thermal_protection;
}

esp_err_t power_mgr_get_thresholds(uint16_t *sleep_mv, uint16_t *wake_mv) {
    if (sleep_mv == NULL || wake_mv == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *sleep_mv = s_sleep_mv;
    *wake_mv = s_wake_mv;
    return ESP_OK;
}

// 外设电源管理实现
esp_err_t power_mgr_external_power_init(void) {
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Initializing external power control on GPIO %d", EXTERNAL_POWER_CTRL_GPIO);

    // 配置GPIO为输出模式
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << EXTERNAL_POWER_CTRL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure external power GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    // 默认开启外设电源
    ret = power_mgr_external_power_on();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to turn on external power: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "External power control initialized successfully");
    return ESP_OK;
}

esp_err_t power_mgr_external_power_on(void) {
    ESP_LOGI(TAG, "Turning on external power");

    esp_err_t ret = gpio_set_level(EXTERNAL_POWER_CTRL_GPIO, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set external power GPIO high: %s", esp_err_to_name(ret));
        return ret;
    }

    s_external_power_enabled = true;

    // 等待外设电源稳定
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "External power is now ON");
    return ESP_OK;
}

esp_err_t power_mgr_external_power_off(void) {
    ESP_LOGI(TAG, "Turning off external power");

    esp_err_t ret = gpio_set_level(EXTERNAL_POWER_CTRL_GPIO, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set external power GPIO low: %s", esp_err_to_name(ret));
        return ret;
    }

    s_external_power_enabled = false;

    ESP_LOGI(TAG, "External power is now OFF");
    return ESP_OK;
}

bool power_mgr_external_power_is_on(void) {
    return s_external_power_enabled;
}
