/**
 * @file led_status.h
 * @brief LED状态指示模块头文件
 */

#ifndef _LED_STATUS_H_
#define _LED_STATUS_H_

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化LED状态指示模块
 * @return ESP_OK on success, otherwise error code
 */
esp_err_t led_status_init(void);

/**
 * @brief 设置蓝牙连接状态
 * @param connected true表示已连接，false表示未连接
 * @return ESP_OK on success, otherwise error code
 */
esp_err_t led_status_set_bluetooth_connected(bool connected);

/**
 * @brief 设置蓝牙广播状态
 * @param advertising true表示正在广播，false表示未广播
 * @return ESP_OK on success, otherwise error code
 */
esp_err_t led_status_set_bluetooth_advertising(bool advertising);

/**
 * @brief 设置错误状态
 * @param error true表示有错误，false表示无错误
 * @return ESP_OK on success, otherwise error code
 */
esp_err_t led_status_set_error(bool error);

/**
 * @brief 设置低电压状态
 * @param low true表示电压低，false表示电压正常
 * @return ESP_OK on success, otherwise error code
 */
esp_err_t led_status_set_low_battery(bool low);

/**
 * @brief 设置LED亮度
 * @param brightness 亮度值（0-255）
 * @return ESP_OK on success, otherwise error code
 */
esp_err_t led_status_set_brightness(uint8_t brightness);

/**
 * @brief 启用/禁用LED状态显示
 * @param enable true表示启用，false表示禁用
 * @return ESP_OK on success, otherwise error code
 */
esp_err_t led_status_enable(bool enable);

/**
 * @brief 更新LED显示状态
 * @return ESP_OK on success, otherwise error code
 */
esp_err_t led_status_update(void);

#ifdef __cplusplus
}
#endif

#endif /* _LED_STATUS_H_ */