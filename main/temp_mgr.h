#ifndef TEMP_MGR_H
#define TEMP_MGR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "hardware_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

// 最大支持的DS18B20传感器数量
#define TEMP_MGR_MAX_SENSORS    TEMP_SENSOR_COUNT

// 温度传感器类型
typedef enum {
    TEMP_SENSOR_POWER = 0,    // 电源区域温度传感器
    TEMP_SENSOR_CONTROL = 1,  // 控制区域温度传感器
} temp_sensor_type_t;

// 温度数据状态
typedef enum {
    TEMP_STATUS_INVALID = 0,     // 无效数据
    TEMP_STATUS_VALID,           // 有效数据
    TEMP_STATUS_STALE,           // 数据过期
    TEMP_STATUS_ERROR,           // 传感器错误
} temp_data_status_t;

// 单个传感器数据
typedef struct {
    int16_t temperature;         // 温度值 (0.01°C单位)
    uint32_t timestamp;          // 时间戳 (ms)
    temp_data_status_t status;   // 数据状态
    uint32_t error_count;        // 连续错误计数
} temp_sensor_data_t;

// === 温度传感器地址映射 ===
// DS18B20传感器地址映射条目
typedef struct {
    uint64_t address;            // DS18B20 64位地址
    bool configured;             // 是否已配置映射
} temp_sensor_mapping_t;

// DS18B20地址映射配置
typedef struct {
    temp_sensor_mapping_t sensors[TEMP_MGR_MAX_SENSORS];
} temp_sensor_config_t;

// FreeRTOS事件组位定义
#define TEMP_EVENT_NEW_DATA_BIT          (1 << 0)  // 新温度数据可用
#define TEMP_EVENT_SENSOR_ERROR_BIT      (1 << 1)  // 传感器错误
#define TEMP_EVENT_THRESHOLD_HIGH_BIT    (1 << 2)  // 高温阈值触发
#define TEMP_EVENT_THRESHOLD_LOW_BIT     (1 << 3)  // 低温阈值触发
#define TEMP_EVENT_SENSOR_RECOVER_BIT    (1 << 4)  // 传感器恢复
#define TEMP_EVENT_ALL_BITS             0x1F       // 所有事件位

// 温度事件类型（用于队列）
typedef enum {
    TEMP_QUEUE_EVENT_NEW_DATA = 0,     // 新温度数据
    TEMP_QUEUE_EVENT_THRESHOLD_HIGH,   // 高温阈值触发
    TEMP_QUEUE_EVENT_THRESHOLD_LOW,    // 低温阈值触发
    TEMP_QUEUE_EVENT_SENSOR_ERROR,     // 传感器错误
    TEMP_QUEUE_EVENT_SENSOR_RECOVER,   // 传感器恢复
} temp_queue_event_type_t;

// 温度队列事件数据
typedef struct {
    temp_queue_event_type_t type;       // 事件类型
    temp_sensor_type_t sensor_type;     // 传感器类型
    int16_t temperature;                // 温度值
    uint32_t timestamp;                 // 时间戳
} temp_queue_event_t;

// 默认采样间隔 (ms)
#define TEMP_DEFAULT_SAMPLING_INTERVAL    1000
#define TEMP_STALE_TIMEOUT_MS            30000  // 30秒后数据被认为过期

// === 兼容性API (保持向后兼容) ===
// 温度管理器初始化
esp_err_t temp_mgr_init(void);

// 执行一次温度采样（指定传感器）- 现在返回缓存数据
esp_err_t temp_mgr_sample_once(temp_sensor_type_t sensor_type, int16_t *out_temp);

// 执行一次温度采样（所有传感器）- 现在返回缓存数据
esp_err_t temp_mgr_sample_all(int16_t out_temps[TEMP_SENSOR_COUNT]);

// 获取已连接的传感器数量
int temp_mgr_get_sensor_count(void);

// 关闭温度传感器（用于深度睡眠前）
void temp_mgr_deinit(void);

// === 新的异步API ===
// 获取缓存的温度数据 (非阻塞)
esp_err_t temp_mgr_get_cached(temp_sensor_type_t sensor_type, temp_sensor_data_t *out_data);

// 获取所有缓存的温度数据
esp_err_t temp_mgr_get_all_cached(temp_sensor_data_t out_data[TEMP_SENSOR_COUNT]);

// 检查数据是否有效
bool temp_mgr_is_data_fresh(temp_sensor_type_t sensor_type, uint32_t max_age_ms);

// === FreeRTOS事件系统API ===
// 获取事件组句柄（用于外部任务等待温度事件）
EventGroupHandle_t temp_mgr_get_event_group(void);

// 等待温度事件（阻塞等待指定事件）
uint32_t temp_mgr_wait_for_events(EventBits_t bits_to_wait, TickType_t timeout);

// 获取事件队列句柄（用于接收详细的温度事件）
QueueHandle_t temp_mgr_get_event_queue(void);

// 从事件队列接收温度事件（阻塞接收）
bool temp_mgr_receive_event(temp_queue_event_t *event, TickType_t timeout);

// 清除指定的事件位
void temp_mgr_clear_events(EventBits_t bits_to_clear);

// 设置采样间隔
esp_err_t temp_mgr_set_sampling_interval(uint32_t interval_ms);

// 获取当前采样间隔
uint32_t temp_mgr_get_sampling_interval(void);

// 启动/停止连续采样
esp_err_t temp_mgr_start_continuous(void);
esp_err_t temp_mgr_stop_continuous(void);

// 获取任务状态
bool temp_mgr_is_running(void);

// 强制立即采样 (异步)
esp_err_t temp_mgr_trigger_immediate_sample(void);

// === 温度传感器地址映射API ===
//
// 这些API允许配置温度传感器的DS18B20地址到监控区域的映射，
// 提供确定性的传感器分配，不受传感器发现顺序影响。
//
// 默认映射配置（在初始化时自动设置）：
// - 控制区域 (TEMP_SENSOR_CONTROL): 0x3D0225127D25CF28
// - 电源区域 (TEMP_SENSOR_POWER):    0xDB02253BBFE25F28
//
// 使用示例：
// 1. 自动检测传感器：
//    esp_err_t ret = temp_mgr_auto_detect_and_map();
//    // 日志将显示发现的传感器地址和当前分配
//
// 2. 配置自定义映射（覆盖默认配置）：
//    temp_mgr_set_sensor_mapping(TEMP_SENSOR_POWER, 0x28FF641D32150428ULL);
//    temp_mgr_set_sensor_mapping(TEMP_SENSOR_CONTROL, 0x28FF1A3C32150471ULL);
//
// 3. 检查映射配置：
//    uint64_t addr;
//    bool configured;
//    temp_mgr_get_sensor_mapping(TEMP_SENSOR_POWER, &addr, &configured);
//    if (configured) {
//        char addr_str[17];
//        temp_mgr_address_to_string(addr, addr_str, sizeof(addr_str));
//        printf("Power sensor mapped to: %s\n", addr_str);
//    }
//
// 4. 清除映射并回退到发现顺序：
//    temp_mgr_clear_sensor_mapping(TEMP_SENSOR_POWER);
//    temp_mgr_clear_sensor_mapping(TEMP_SENSOR_CONTROL);

// 设置温度传感器地址映射
// @param sensor_type 传感器类型 (TEMP_SENSOR_POWER 或 TEMP_SENSOR_CONTROL)
// @param address DS18B20 64位地址
// @return ESP_OK 成功，ESP_ERR_INVALID_ARG 无效参数或地址，ESP_ERR_INVALID_STATE 传感器未初始化
esp_err_t temp_mgr_set_sensor_mapping(temp_sensor_type_t sensor_type, uint64_t address);

// 获取温度传感器地址映射
// @param sensor_type 传感器类型
// @param address 输出参数，返回配置的地址
// @param configured 输出参数，返回是否已配置映射
// @return ESP_OK 成功，ESP_ERR_INVALID_ARG 无效参数
esp_err_t temp_mgr_get_sensor_mapping(temp_sensor_type_t sensor_type, uint64_t *address, bool *configured);

// 清除温度传感器地址映射
// @param sensor_type 要清除映射的传感器类型
// @return ESP_OK 成功，ESP_ERR_INVALID_ARG 无效参数
esp_err_t temp_mgr_clear_sensor_mapping(temp_sensor_type_t sensor_type);

// 自动检测并建议地址映射
// 扫描连接的DS18B20传感器并显示地址信息，便于配置映射
// @return ESP_OK 成功，ESP_ERR_NOT_FOUND 未发现传感器，ESP_ERR_INVALID_STATE 传感器未初始化
esp_err_t temp_mgr_auto_detect_and_map(void);

// 验证DS18B20地址格式
// @param address 要验证的64位地址
// @return true 地址格式有效（DS18B20家族码0x28），false 无效
bool temp_mgr_is_valid_ds18b20_address(uint64_t address);

// 将64位地址转换为字符串（用于显示和日志）
// @param address DS18B20 64位地址
// @param str 输出字符串缓冲区（至少17字节）
// @param len 缓冲区长度
void temp_mgr_address_to_string(uint64_t address, char *str, size_t len);

#ifdef __cplusplus
}
#endif

#endif // TEMP_MGR_H
