#include "power_mgr.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "host/ble_hs.h"

#include "led_status.h"
#include "pwm_control.h"
#include "storage.h"

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

#define DEFAULT_SLEEP_MV 12500
#define DEFAULT_WAKE_MV  13200

#define SAMPLE_INTERVAL_MS 1000

static uint16_t s_sleep_mv = DEFAULT_SLEEP_MV;
static uint16_t s_wake_mv  = DEFAULT_WAKE_MV;
static int16_t s_cal_off_mv = 0;

static esp_err_t nvs_load_thresholds(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) return err;
    uint16_t v;
    if (nvs_get_u16(h, KEY_SLEEP, &v) == ESP_OK) s_sleep_mv = v;
    if (nvs_get_u16(h, KEY_WAKE,  &v) == ESP_OK) s_wake_mv  = v;
    nvs_close(h);
    return ESP_OK;
}

static esp_err_t nvs_store_thresholds(void) {
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NS, NVS_READWRITE, &h), TAG, "nvs open");
    ESP_RETURN_ON_ERROR(nvs_set_u16(h, KEY_SLEEP, s_sleep_mv), TAG, "set sleep");
    ESP_RETURN_ON_ERROR(nvs_set_u16(h, KEY_WAKE,  s_wake_mv),  TAG, "set wake");
    ESP_RETURN_ON_ERROR(nvs_set_blob(h, KEY_CAL, &s_cal_off_mv, sizeof(s_cal_off_mv)), TAG, "set cal");
    ESP_RETURN_ON_ERROR(nvs_commit(h), TAG, "commit");
    nvs_close(h);
    return ESP_OK;
}

#define SAMPLE_INTERVAL_MS 1000

static uint16_t ulp_get_voltage_mv(void) { return ulp_last_mv & 0xFFFF; }

static void ulp_share_params(void) {
    ulp_wake_mv = s_wake_mv;
    ulp_cal_off_mv = s_cal_off_mv;
    ulp_wake_hyst_mv = 200;
}

static void ulp_start(void) {
    ulp_riscv_load_binary(ulp_power_bin_start, (ulp_power_bin_end - ulp_power_bin_start));
    ulp_ok_cnt = 0;
    ulp_min_sleep_ticks = 0;
    ulp_share_params();
    ulp_riscv_timer_stop();
    ulp_set_wakeup_period(0, 1000000);
    ulp_riscv_timer_resume();
    ulp_riscv_run();
    esp_sleep_enable_ulp_wakeup();
}

esp_err_t power_mgr_init(void) {
    nvs_load_thresholds();

    ulp_share_params();

    ulp_riscv_adc_cfg_t cfg = {
        .adc_n = ADC_UNIT_1,
        .channel = ADC_CHANNEL_1,
        .atten = ADC_ATTEN_DB_12,
        .width = ADC_BITWIDTH_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_RISCV,
    };
    ESP_RETURN_ON_ERROR(ulp_riscv_adc_init(&cfg), TAG, "ulp adc init");

    ulp_start();
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_ULP) {
        if (ulp_wake_flag == 0) {
            ulp_ok_cnt = 0;
            ulp_min_sleep_ticks = 15;
            esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
            esp_sleep_enable_ulp_wakeup();
            esp_deep_sleep_start();
        }
        ESP_LOGI(TAG, "Wake from ULP");
    } else {
        ESP_LOGI(TAG, "Cold boot or other wake cause=%d", cause);
    }
    led_status_enable(true);
    led_status_set_low_battery(false);
    return ESP_OK;
}

void power_mgr_task(void *arg) {
    TickType_t last = xTaskGetTickCount();
    static int low_cnt = 0;
    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
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
    uint8_t states[4];
    storage_read_states(states);
    storage_write_states(states);
    (void)led_status_set_bluetooth_advertising(false);
    (void)led_status_set_bluetooth_connected(false);
    (void)led_status_set_low_battery(false);
    (void)led_status_enable(false);
    ulp_wake_flag = 0;
    ulp_ok_cnt = 0;
    ulp_min_sleep_ticks = 15;
    ESP_LOGI(TAG, "Enter deep sleep");
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_sleep_enable_ulp_wakeup();
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_deep_sleep_start();
    return ESP_OK;
}

int power_mgr_ble_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint16_t vin_mv;
        power_mgr_get_voltage_mv(&vin_mv, false);
        power_mgr_get_voltage_mv(&vin_mv, false);
        uint16_t buf[3] = { __builtin_bswap16(vin_mv), __builtin_bswap16(s_sleep_mv), __builtin_bswap16(s_wake_mv) };
        return os_mbuf_append(ctxt->om, buf, sizeof(buf)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t data[3] = {0};
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len != 3) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        uint16_t copied = 0;
        if (ble_hs_mbuf_to_flat(ctxt->om, data, sizeof(data), &copied) != 0 || copied != 3) return BLE_ATT_ERR_UNLIKELY;
        uint8_t cmd = data[0];
        uint16_t param = (data[1] << 8) | data[2];
        esp_err_t err = ESP_OK;
        switch (cmd) {
            case 0x01: err = power_mgr_set_thresholds(param, s_wake_mv); break;
            case 0x02: err = power_mgr_set_thresholds(s_sleep_mv, param); break;
            case 0x03: err = power_mgr_force_sleep(); break;
            case 0x04: err = ESP_OK; break;
            case 0x05: err = power_mgr_set_cal_offset_mv((int16_t)param); break;
            case 0x06: {
                uint16_t measured;
                power_mgr_get_voltage_mv(&measured, false);
                int32_t off = (int32_t)param - (int32_t)measured;
                if (off < -2000) off = -2000;
                if (off >  2000) off =  2000;
                err = power_mgr_set_cal_offset_mv((int16_t)off);
                break;
            }
            default: return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
        }
        return err == ESP_OK ? 0 : BLE_ATT_ERR_UNLIKELY;
    }
    return BLE_ATT_ERR_UNLIKELY;
}
