// 系统监控模块 - 统一监控电压、电流、温度
#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 事件类型定义
typedef enum {
    SYSTEM_EVENT_NONE = 0,

    // 电压事件
    SYSTEM_EVENT_VOLTAGE_NORMAL = 1,      // 电压正常
    SYSTEM_EVENT_VOLTAGE_LOW_WARNING,     // 电压低警告
    SYSTEM_EVENT_VOLTAGE_LOW_CRITICAL,    // 电压低临界
    SYSTEM_EVENT_VOLTAGE_RECOVERY,        // 电压恢复

    // 电流事件
    SYSTEM_EVENT_CURRENT_OVERLOAD,        // 电流过载
    SYSTEM_EVENT_CURRENT_NORMAL,          // 电流正常

    // 温度事件
    SYSTEM_EVENT_TEMP_HIGH_WARNING,       // 温度高警告
    SYSTEM_EVENT_TEMP_HIGH_CRITICAL,      // 温度高临界
    SYSTEM_EVENT_TEMP_RECOVERY,           // 温度恢复

    // 系统事件
    SYSTEM_EVENT_EXTERNAL_POWER_LOST,     // 外设电源丢失
    SYSTEM_EVENT_EXTERNAL_POWER_RESTORED,  // 外设电源恢复
} system_event_type_t;

// 事件优先级
typedef enum {
    EVENT_PRIORITY_LOW = 0,
    EVENT_PRIORITY_MEDIUM = 1,
    EVENT_PRIORITY_HIGH = 2,
    EVENT_PRIORITY_CRITICAL = 3,
} event_priority_t;

// 事件数据结构
typedef struct {
    system_event_type_t type;        // 事件类型
    event_priority_t priority;       // 事件优先级
    uint32_t timestamp;              // 事件时间戳 (ms)
    float value;                     // 相关数值 (电压V/电流A/温度°C)
    uint8_t channel;                 // 相关通道 (用于电流事件)
    char description[32];            // 事件描述
} system_event_t;

// 监控配置
typedef struct {
    // 电压阈值 (mV)
    uint16_t voltage_low_warning;     // 低电压警告阈值
    uint16_t voltage_low_critical;    // 低电压临界阈值
    uint16_t voltage_recovery;        // 电压恢复阈值

    // 电流阈值 (A, 乘以1000存储为整数)
    uint16_t current_overload_threshold;  // 电流过载阈值
    uint16_t current_overload_duration;   // 过载持续时间 (ms)

    // 温度阈值 (°C, 乘以100存储为整数)
    uint16_t temp_high_warning;       // 高温警告阈值
    uint16_t temp_high_critical;      // 高温临界阈值
    uint16_t temp_recovery;           // 温度恢复阈值

    // 监控间隔 (ms)
    uint32_t voltage_monitor_interval;    // 电压监控间隔
    uint32_t current_monitor_interval;    // 电流监控间隔
    uint32_t temp_monitor_interval;       // 温度监控间隔
} monitor_config_t;

// 监控状态
typedef struct {
    // 当前状态
    float last_voltage;                // 最后电压值 (V)
    float last_total_current;          // 最后总电流值 (A)
    float last_channel_currents[6];    // 最后各通道电流值 (A)
    float last_power_temp;             // 最后电源区域温度 (°C)
    float last_control_temp;           // 最后控制区域温度 (°C)

    // 状态标志
    bool voltage_low_warning_active;   // 低电压警告激活
    bool voltage_low_critical_active;  // 低电压临界激活
    bool current_overload_active;      // 电流过载激活
    bool temp_high_warning_active;     // 高温警告激活
    bool temp_high_critical_active;    // 高温临界激活

    // 过载计时
    uint32_t overload_start_time;      // 过载开始时间
    uint8_t overload_channel;          // 过载通道
} monitor_state_t;

// 事件处理回调函数类型
typedef esp_err_t (*system_event_handler_t)(const system_event_t *event, void *user_data);

// 初始化系统监控
esp_err_t system_monitor_init(void);

// 启动监控任务
esp_err_t system_monitor_start(void);

// 停止监控任务
esp_err_t system_monitor_stop(void);

// 注册事件处理器
esp_err_t system_monitor_register_handler(system_event_type_t event_type,
                                        system_event_handler_t handler,
                                        void *user_data);

// 注销事件处理器
esp_err_t system_monitor_unregister_handler(system_event_type_t event_type);

// 获取监控配置
esp_err_t system_monitor_get_config(monitor_config_t *config);

// 设置监控配置
esp_err_t system_monitor_set_config(const monitor_config_t *config);

// 获取监控状态
esp_err_t system_monitor_get_state(monitor_state_t *state);

// 手动触发事件
esp_err_t system_monitor_trigger_event(const system_event_t *event);

// 获取最新事件
esp_err_t system_monitor_get_last_event(system_event_t *event);

// 检查是否有未处理的高优先级事件
bool system_monitor_has_critical_events(void);

#ifdef __cplusplus
}
#endif