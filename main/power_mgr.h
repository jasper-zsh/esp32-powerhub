#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

struct ble_gatt_access_ctxt;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t power_mgr_init(void);
void power_mgr_task(void *);
esp_err_t power_mgr_get_voltage_mv(uint16_t *out_mv, bool force_refresh);
esp_err_t power_mgr_set_thresholds(uint16_t sleep_mv, uint16_t wake_mv);
esp_err_t power_mgr_get_thresholds(uint16_t *sleep_mv, uint16_t *wake_mv);
esp_err_t power_mgr_force_sleep(void);
esp_err_t power_mgr_set_cal_offset_mv(int16_t off_mv);

// 温度相关接口
esp_err_t power_mgr_get_temperature(int16_t *out_temp);
esp_err_t power_mgr_set_temp_thresholds(int16_t high_temp, int16_t recover_temp);
esp_err_t power_mgr_get_temp_thresholds(int16_t *high_temp, int16_t *recover_temp);
bool power_mgr_is_thermal_protection_active(void);

int power_mgr_ble_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt);
void power_mgr_ble_subscribe(uint16_t conn_handle, uint16_t attr_handle, bool enabled);

#ifdef __cplusplus
}
#endif
