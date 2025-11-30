#include "power_mgr.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "host/ble_hs.h"
#include "driver/gpio.h"

#include "led_status.h"
#include "pwm_control.h"
#include "storage.h"
#include "temp_mgr.h"
#include "scheduler.h"

#include "ulp_power.h"
#include "ulp_riscv.h"
#include "ulp_common.h"
#include "ulp_riscv_adc.h"

extern const uint8_t ulp_power_bin_start[] asm("_binary_ulp_power_bin_start");
extern const uint8_t ulp_power_bin_end[]   asm("_binary_ulp_power_bin_end");

static const char *TAG = "power_mgr";

#define NVS_NS "power_mgr"
#define KEY_SLEEP "sleep_mv"
#define KEY_WAKE  "wake_mv"
#define KEY_CAL   "cal_off"
#define KEY_T_HI  "t_hi"
#define KEY_T_REC "t_rec"

#define DEFAULT_SLEEP_MV 12500
#define DEFAULT_WAKE_MV  13200
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
static int16_t s_latest_temp = 0;
static bool s_temp_valid = false;
static bool s_thermal_protection = false;
static int s_hot_debounce_count = 0;
static int s_cool_debounce_count = 0;

// BLE通知相关
static bool s_ble_notifications_enabled = false;
static uint16_t s_ble_conn_handle = 0;
static uint16_t s_ble_attr_handle = 0;

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

// static uint16_t ulp_get_voltage_mv(void) { return ulp_last_mv & 0xFFFF; }

static uint16_t ulp_get_voltage_mv(void) { return 13500; }

static void ulp_share_params(void) {
    ulp_wake_mv = s_wake_mv;
    ulp_cal_off_mv = s_cal_off_mv;
}

static esp_err_t ulp_start(void) {
    ulp_riscv_adc_cfg_t cfg = {
        .adc_n = ADC_UNIT_1,
        .channel = ADC_CHANNEL_0,
        .atten = ADC_ATTEN_DB_12,
        .width = ADC_BITWIDTH_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_RISCV,
    };
    ESP_RETURN_ON_ERROR(ulp_riscv_adc_init(&cfg), TAG, "ulp adc init");

    ulp_riscv_load_binary(ulp_power_bin_start, (ulp_power_bin_end - ulp_power_bin_start));
    ulp_share_params();
    ulp_set_wakeup_period(0, 1000000);
    ulp_riscv_run();
    esp_sleep_enable_ulp_wakeup();

    return ESP_OK;
}

esp_err_t power_mgr_init(void) {
    nvs_load_thresholds();
    ulp_share_params();

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    if (cause == ESP_SLEEP_WAKEUP_ULP) {
        ESP_LOGI(TAG, "Deep sleep wakeup");
        uint16_t voltage = ulp_get_voltage_mv();
        ESP_LOGI(TAG, "Wake from ULP (vin=%u mV)", voltage);
    } else {
        ESP_LOGI(TAG, "Cold boot or other wake cause=%d", cause);
        ESP_ERROR_CHECK(ulp_start());
    }

    led_status_enable(true);
    led_status_set_low_battery(false);

    // 初始化温度管理器
    esp_err_t temp_err = temp_mgr_init();
    if (temp_err != ESP_OK) {
        ESP_LOGW(TAG, "Temperature manager init failed: %s", esp_err_to_name(temp_err));
    }

    return ESP_OK;
}

static void thermal_protection_trigger(void) {
    if (s_thermal_protection) {
        return; // 已经在热保护状态
    }

    ESP_LOGW(TAG, "Thermal protection triggered, temperature: %d.%02d°C",
             s_latest_temp / 100, abs(s_latest_temp % 100));

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

    ESP_LOGI(TAG, "Thermal protection recovery, temperature: %d.%02d°C",
             s_latest_temp / 100, abs(s_latest_temp % 100));

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

void power_mgr_task(void *arg) {
    TickType_t last = xTaskGetTickCount();
    static int low_cnt = 0;

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));

        // 电压管理
        #ifndef BUILTIN_ADC
        uint16_t vin_mv = ulp_get_voltage_mv();
        led_status_set_low_battery(vin_mv < s_sleep_mv);
        if (vin_mv < s_sleep_mv) {
            if (++low_cnt >= 12) {
                ESP_LOGI(TAG, "Low for >=12s, enter deep sleep");
                power_mgr_force_sleep();
            }
        } else {
            low_cnt = 0;
        }
        #endif

        // 温度管理
        int16_t temp;
        esp_err_t temp_err = temp_mgr_sample_once(TEMP_SENSOR_POWER, &temp);
        if (temp_err == ESP_OK) {
            s_latest_temp = temp;
            s_temp_valid = true;

            // 热保护状态机
            if (!s_thermal_protection) {
                // 检查是否超温
                if (temp >= s_temp_hi) {
                    s_hot_debounce_count++;
                    s_cool_debounce_count = 0;
                    if (s_hot_debounce_count >= TEMP_DEBOUNCE_COUNT) {
                        thermal_protection_trigger();
                    }
                } else {
                    s_hot_debounce_count = 0;
                }
            } else {
                // 检查是否可以恢复
                if (temp <= s_temp_rec) {
                    s_cool_debounce_count++;
                    s_hot_debounce_count = 0;
                    if (s_cool_debounce_count >= TEMP_DEBOUNCE_COUNT) {
                        thermal_protection_recovery();
                    }
                } else {
                    s_cool_debounce_count = 0;
                }
            }
        } else {
            if (temp_err != ESP_OK) {
                ESP_LOGD(TAG, "Temperature sampling failed: %s", esp_err_to_name(temp_err));
            }
        }

        // 实时监控通知已移至CH_MONITORING特征，此处不再发送通知
    }
}

esp_err_t power_mgr_get_voltage_mv(uint16_t *out_mv, bool force_refresh) {
    if (!out_mv) return ESP_ERR_INVALID_ARG;
    uint16_t vin_mv = ulp_get_voltage_mv();
    *out_mv = vin_mv;
    return ESP_OK;
}

esp_err_t power_mgr_set_thresholds(uint16_t sleep_mv, uint16_t wake_mv) {
    if (sleep_mv == 0 || wake_mv == 0 || wake_mv <= sleep_mv) return ESP_ERR_INVALID_ARG;
    s_sleep_mv = sleep_mv;
    s_wake_mv = wake_mv;
    ulp_share_params();
    return nvs_store_thresholds();
}

esp_err_t power_mgr_set_cal_offset_mv(int16_t off_mv) {
    if (off_mv < -2000 || off_mv > 2000) return ESP_ERR_INVALID_ARG;
    s_cal_off_mv = off_mv;
    ulp_share_params();
    return nvs_store_thresholds();
}

esp_err_t power_mgr_force_sleep(void) {
    ESP_LOGI(TAG, "Preparing deep sleep");
    uint8_t states[PWM_CHANNEL_COUNT > 0 ? PWM_CHANNEL_COUNT : 1];
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

    ulp_wake_flag = 0;
    ESP_LOGI(TAG, "Enter deep sleep");
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_sleep_enable_ulp_wakeup();
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_deep_sleep_start();
    return ESP_OK;
}

int power_mgr_ble_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt) {
    // 保存连接信息用于通知
    s_ble_conn_handle = conn;
    s_ble_attr_handle = attr;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // 电源管理特征现在只返回配置数据（8字节），状态位已移至监控特征
        uint8_t response[8];
        int offset = 0;

        // int16 高温阈值（0.01°C）
        response[offset++] = (s_temp_hi >> 8) & 0xFF;
        response[offset++] = s_temp_hi & 0xFF;

        // int16 恢复阈值（0.01°C）
        response[offset++] = (s_temp_rec >> 8) & 0xFF;
        response[offset++] = s_temp_rec & 0xFF;

        // uint16 睡眠电压阈值（mV）
        response[offset++] = (s_sleep_mv >> 8) & 0xFF;
        response[offset++] = s_sleep_mv & 0xFF;

        // uint16 唤醒电压阈值（mV）
        response[offset++] = (s_wake_mv >> 8) & 0xFF;
        response[offset++] = s_wake_mv & 0xFF;

        // 确保返回了8字节
        if (offset != 8) {
            ESP_LOGE(TAG, "Power management config size mismatch: expected 8, got %d", offset);
            return BLE_ATT_ERR_UNLIKELY;
        }

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

    if (!s_temp_valid) {
        return ESP_ERR_INVALID_STATE;
    }

    *out_temp = s_latest_temp;
    return ESP_OK;
}

esp_err_t power_mgr_get_latest_temp(int16_t *out_temp) {
    if (!out_temp) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_temp = s_latest_temp;
    return s_temp_valid ? ESP_OK : ESP_ERR_INVALID_STATE;
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

esp_err_t power_mgr_enable_ulp_mode(bool enable) {
    ESP_LOGI(TAG, "ULP mode configuration: %s", enable ? "enabled" : "disabled");
    // ULP模式始终启用，参数被忽略
    return ESP_OK;
}

bool power_mgr_is_ulp_mode_enabled(void) {
    return true;
}

// 外设电源管理接口
esp_err_t power_mgr_external_power_init(void) {
    // GPIO6控制所有外设的电源
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PWR_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return ret;
    }

    // 默认开启外设电源
    gpio_set_level(PWR_GPIO, 1);
    return ESP_OK;
}

esp_err_t power_mgr_external_power_on(void) {
    gpio_set_level(PWR_GPIO, 1);
    return ESP_OK;
}

esp_err_t power_mgr_external_power_off(void) {
    gpio_set_level(PWR_GPIO, 0);
    return ESP_OK;
}

bool power_mgr_external_power_is_on(void) {
    return gpio_get_level(PWR_GPIO) == 1;
}

// 事件处理接口
esp_err_t power_mgr_handle_system_event(const void *event, void *user_data) {
    // 简化的事件处理，目前只是返回成功
    return ESP_OK;
}

esp_err_t power_mgr_register_sleep_callback(void (*callback)(bool entering_sleep)) {
    // 简化的回调注册，目前只是返回成功
    return ESP_OK;
}
