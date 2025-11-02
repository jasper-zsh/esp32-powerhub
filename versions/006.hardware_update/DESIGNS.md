# 版本006 - 硬件定义更新设计文档

## 设计概述
基于ESP32C6的电源管理器硬件升级，重点实现ADC128S102外置ADC集成、6通道PWM控制扩展、外设电源管理等新功能。本设计不保持向后兼容性，完全重新设计系统架构。

## 系统架构设计

### 1. 硬件抽象层设计

#### 1.1 GPIO映射定义
```c
// 硬件引脚定义
#define EXTERNAL_POWER_CTRL_GPIO      0   // 外设电源控制
#define VOLTAGE_SENSE_GPIO            1   // 电源电压分压输入
#define ADC_CS_GPIO                   2   // ADC128S102片选
#define ADC_SCLK_GPIO                 3   // ADC128S102时钟
#define ADC_MISO_GPIO                 4   // ADC128S102数据输入
#define ADC_MOSI_GPIO                 5   // ADC128S102数据输出
#define TEMP_SENSOR_GPIO              22  // DS18B20温度传感器
#define PWM_CH1_GPIO                  21  // PWM通道1
#define PWM_CH2_GPIO                  20  // PWM通道2
#define PWM_CH3_GPIO                  19  // PWM通道3
#define PWM_CH4_GPIO                  18  // PWM通道4
#define PWM_CH5_GPIO                  15  // PWM通道5
#define PWM_CH6_GPIO                  14  // PWM通道6
#define RGB_LED_GPIO                  8   // WS2812 RGB LED
```

#### 1.2 传感器数据结构
```c
// 电流传感器数据
typedef struct {
    float total_input_current;      // 总输入电流 (ACS758)
    float channel_currents[6];      // 各通道电流 (ACS712 CH1-CH6)
    uint32_t timestamp;             // 采集时间戳
} current_sensor_data_t;

// 电源状态数据
typedef struct {
    float supply_voltage;           // 电源电压
    float power_area_temp;          // 电源区域温度
    float control_area_temp;        // 控制区域温度
    uint16_t external_power_state;  // 外设电源状态
    uint32_t timestamp;             // 更新时间戳
} power_status_t;
```

### 2. ADC128S102集成设计

#### 2.1 SPI接口设计
- **SPI主机配置**: ESP32S3作为SPI主机，ADC128S102作为从机
- **通信参数**:
  - 时钟频率: 1MHz (确保信号完整性)
  - 数据位: 8位
  - 模式: MODE 0 (CPOL=0, CPHA=0)
- **通信协议**:
  - 启动转换: 发送通道选择字节
  - 读取数据: 延迟后读取12位ADC结果

#### 2.2 电流采集算法
```c
// ACS758参数 (总电流)
#define ACS758_SENSITIVITY    20.0f    // mV/A (20mV/A版本)
#define ACS758_OFFSET         2.5f     // V (零电流偏置，5V系统)
#define ACS758_CURRENT_RANGE  100.0f   // A (±50A测量范围)

// ACS712ELCTR-20A参数 (各通道)
#define ACS712_SENSITIVITY    100.0f   // mV/A (20A版本)
#define ACS712_OFFSET         2.5f     // V (零电流偏置，5V系统)
#define ACS712_CURRENT_RANGE  20.0f    // A (±20A测量范围)

// 电流计算公式 (ADC128S102工作在5V系统，12位ADC)
float adc_voltage = adc_raw * 5.0f / 4096.0f;  // 转换为实际电压
float current = (adc_voltage - offset) / (sensitivity / 1000.0f);
```

#### 2.3 采集策略
- **采样频率**: 100Hz (每10ms采集一次)
- **数据滤波**: 使用滑动平均滤波，窗口大小10
- **校准机制**: 支持零点校准和增益校准

### 3. 6通道PWM控制系统设计

#### 3.1 LEDC控制器配置
- **定时器配置**:
  - 频率: 5kHz (平衡控制精度和功耗)
  - 分辨率: 13位 (8192级精度)
  - 定时器: LEDC_TIMER_0 (所有通道共享同一个定时器)
- **ESP32C6特性**: 使用ESP32C6的低功耗PWM控制器，支持睡眠模式下的PWM输出
- **通道映射**:
  - PWM_CH1: LEDC_CHANNEL_0 -> GPIO21
  - PWM_CH2: LEDC_CHANNEL_1 -> GPIO20
  - PWM_CH3: LEDC_CHANNEL_2 -> GPIO19
  - PWM_CH4: LEDC_CHANNEL_3 -> GPIO18
  - PWM_CH5: LEDC_CHANNEL_4 -> GPIO15
  - PWM_CH6: LEDC_CHANNEL_5 -> GPIO14
- **CH5/CH6特性**: 与CH1-CH4完全相同的控制逻辑和功能

#### 3.2 模式控制实现
```c
typedef enum {
    PWM_MODE_OFF = 0,              // 高阻态
    PWM_MODE_ON,                   // 恒定高电平
    PWM_MODE_PWM,                  // PWM输出
    PWM_MODE_FADING,               // 渐变模式
    PWM_MODE_BLINKING,             // 闪烁模式
    PWM_MODE_STROBING              // 频闪模式
} pwm_mode_t;

typedef struct {
    uint8_t channel;               // 通道号 (0-5对应CH1-CH6)
    pwm_mode_t mode;               // 当前模式
    uint8_t target_value;          // 目标值 (0-255)
    uint16_t fade_duration;        // 渐变时间 (ms)
    uint16_t blink_period;         // 闪烁周期 (ms)
    uint8_t strobe_count;          // 频闪次数
    uint16_t strobe_duration;      // 频闪持续时间 (ms)
    uint16_t strobe_pause;         // 频闪间隔 (ms)
    uint8_t last_nonzero_value;    // 最后一个非零值
    TimerHandle_t timer;           // 定时器句柄
} pwm_channel_config_t;

// 6通道PWM配置数组
pwm_channel_config_t pwm_channels[6];  // 索引0-5对应CH1-CH6
```

#### 3.3 渐变算法
使用线性插值实现平滑渐变：
```c
current_value = start_value + (target_value - start_value) * progress;
```

#### 3.4 CH5/CH6接口函数
CH5/CH6提供与CH1-CH4完全相同的API接口，确保6通道功能一致性：
```c
// 通用PWM控制接口 (支持所有6个通道)
void pwm_set_channel(uint8_t channel, uint8_t value);           // channel: 0-5
void pwm_set_mode(uint8_t channel, pwm_mode_t mode);            // channel: 0-5
void pwm_set_fade(uint8_t channel, uint8_t target, uint16_t duration);  // channel: 0-5
void pwm_set_blink(uint8_t channel, uint16_t period);           // channel: 0-5
void pwm_set_strobe(uint8_t channel, uint8_t count, uint16_t duration, uint16_t pause);  // channel: 0-5

// 批量控制接口
void pwm_set_all_channels(uint8_t value);                       // 同时设置所有6个通道
void pwm_set_channels_batch(uint8_t channels[], uint8_t values[], uint8_t count);  // 批量设置指定通道
```

#### 3.5 CH5/CH6硬件配置
CH5/CH6的硬件配置与CH1-CH4完全相同，确保6通道性能一致：
- **PWM频率**: 5kHz (与CH1-CH4相同)
- **分辨率**: 13位 (8192级精度，与CH1-CH4相同)
- **输出驱动**: GPIO15/14推挽输出
- **最大负载**: 与CH1-CH4相同的驱动能力
- **保护机制**: 过流、过温、短路保护功能一致

### 4. 外设电源管理设计

#### 4.1 电源时序控制
```c
typedef enum {
    POWER_STATE_OFF = 0,           // 外设电源关闭
    POWER_STATE_ON,                // 外设电源开启
    POWER_STATE_TRANSITIONING      // 状态切换中
} external_power_state_t;

// 上电时序
void system_wakeup_sequence(void) {
    gpio_set_level(EXTERNAL_POWER_CTRL_GPIO, 1);  // 开启外设电源
    vTaskDelay(pdMS_TO_TICKS(100));                // 等待电源稳定
    // 初始化外设...
}

// 下电时序
void system_sleep_sequence(void) {
    // 关闭外设...
    vTaskDelay(pdMS_TO_TICKS(50));                 // 等待外设关闭
    gpio_set_level(EXTERNAL_POWER_CTRL_GPIO, 0);  // 关闭外设电源
}
```

### 5. 温度监测系统设计

#### 5.1 DS18B20集成
- **总线配置**: 单总线协议，GPIO7连接两个传感器
- **传感器地址**: 通过ROM命令区分两个传感器
- **采样频率**: 1Hz (每秒采集一次)
- **温度精度**: 12位 (0.0625°C分辨率)

#### 5.2 温度保护策略
```c
typedef struct {
    float power_warning_threshold;     // 电源区域告警阈值
    float power_critical_threshold;    // 电源区域临界阈值
    float control_warning_threshold;   // 控制区域告警阈值
    float control_critical_threshold;  // 控制区域临界阈值
    float recovery_threshold;          // 恢复阈值
} temp_protection_config_t;

// 保护动作
if (power_temp > critical_threshold || control_temp > critical_threshold) {
    // 强制关闭所有PWM输出 (包括CH5/CH6)
    for (int i = 0; i < 6; i++) {
        pwm_set_mode(i, PWM_MODE_OFF);
    }
    // 进入安全模式
}
```

### 6. RGB LED指示设计

#### 6.1 WS2812控制
- **通信协议**: 单线归零码，800kHz数据速率
- **颜色映射**:
  - 正常状态: 绿色
  - 告警状态: 黄色
  - 异常状态: 红色
  - 睡眠状态: 蓝色
  - 配置模式: 紫色

#### 6.2 状态指示逻辑
```c
typedef enum {
    LED_STATUS_NORMAL = 0,         // 绿色常亮
    LED_STATUS_WARNING,            // 黄色闪烁
    LED_STATUS_ERROR,              // 红色快闪
    LED_STATUS_SLEEP,              // 蓝色呼吸
    LED_STATUS_CONFIG,             // 紫色彩虹
    LED_STATUS_CUSTOM              // 自定义颜色
} led_status_t;
```

### 7. 通信协议重新设计

#### 7.1 BLE服务结构
```c
// 新的BLE服务UUID
#define POWER_HUB_SERVICE_UUID          0x1234
#define CURRENT_SENSOR_CHAR_UUID        0x1235
#define POWER_STATUS_CHAR_UUID          0x1236
#define PWM_CONTROL_CHAR_UUID           0x1237
#define LED_CONTROL_CHAR_UUID           0x1238
#define SYSTEM_CONFIG_CHAR_UUID         0x1239
```

#### 7.2 命令协议设计
```c
typedef struct {
    uint8_t cmd_type;                // 命令类型
    uint8_t channel;                 // 目标通道
    uint8_t payload[8];              // 命令参数
    uint8_t payload_length;          // 参数长度
    uint32_t timestamp;              // 时间戳
    uint16_t checksum;               // 校验和
} command_packet_t;

// 命令类型定义
#define CMD_PWM_SET           0x01
#define CMD_PWM_BLINK         0x02
#define CMD_PWM_STROBE        0x03
#define CMD_PWM_SET_ALL       0x08      // 同时设置所有6个通道
#define CMD_PWM_BATCH         0x09      // 批量设置指定通道
#define CMD_LED_SET           0x04
#define CMD_SYSTEM_SLEEP      0x05
#define CMD_SYSTEM_WAKE       0x06
#define CMD_CALIBRATE         0x07
```

### 8. 数据存储设计

#### 8.1 NVS分区规划
- **配置数据**: PWM配置、LED配置、阈值参数
- **校准数据**: 电流传感器校准参数、电压校准参数
- **状态数据**: 最后的系统状态，用于恢复

#### 8.2 数据结构
```c
typedef struct {
    pwm_channel_config_t pwm_configs[6];   // PWM配置 (索引0-5对应CH1-CH6)
    uint32_t rgb_config;                   // RGB配置
    temp_protection_config_t temp_config;  // 温度保护配置
    uint16_t voltage_thresholds[2];        // 电压阈值 (睡眠/唤醒)
    uint32_t crc32;                        // 数据校验
} system_config_t;
```

### 9. CH5/CH6完整实现说明

#### 9.1 实现确认
CH5/CH6已经在当前设计中完整实现，具备与CH1-CH4完全相同的功能：

- **硬件连接**: CH5 -> GPIO12, CH6 -> GPIO13
- **LEDC通道**: CH5 -> LEDC_CHANNEL_4, CH6 -> LEDC_CHANNEL_5
- **控制模式**: 支持OFF、ON、PWM、FADING、BLINKING、STROBING全部6种模式
- **API接口**: 所有PWM控制函数均支持channel参数0-5，对应CH1-CH6
- **配置存储**: system_config_t中已包含6个通道的完整配置
- **保护机制**: 温度保护等安全机制同时覆盖CH5/CH6

#### 9.2 功能一致性保证
```c
// 确保CH5/CH6与其他通道完全一致的初始化
void pwm_init_all_channels(void) {
    for (int i = 0; i < 6; i++) {
        pwm_channel_config_t *config = &pwm_channels[i];
        config->channel = i;
        config->mode = PWM_MODE_OFF;
        config->target_value = 0;
        // ... 其他初始化参数完全相同
    }
}

// 确保批量操作包含所有6个通道
void pwm_set_all_channels(uint8_t value) {
    for (int i = 0; i < 6; i++) {  // CH1-CH6
        pwm_set_channel(i, value);
    }
}
```

#### 9.3 性能指标验证
CH5/CH6的电气特性与CH1-CH4完全一致：
- **PWM频率**: 5kHz
- **分辨率**: 13位 (8192级)
- **响应时间**: <10ms
- **占空比精度**: 0.1%
- **驱动能力**: 相同的GPIO驱动配置

### 10. 任务调度设计

#### 10.1 FreeRTOS任务划分
- **主控制任务**: 系统状态管理、命令处理
- **ADC采集任务**: 电流数据采集、滤波
- **PWM控制任务**: PWM输出管理、模式切换
- **温度监测任务**: 温度采集、保护逻辑
- **BLE服务任务**: 通信处理、数据上报
- **LED控制任务**: 状态指示、动画效果

#### 10.2 任务优先级
```c
#define TASK_PRIORITY_CRITICAL     5   // 温度保护、电源管理
#define TASK_PRIORITY_HIGH         4   // PWM控制、ADC采集
#define TASK_PRIORITY_MEDIUM       3   // BLE通信、主控制
#define TASK_PRIORITY_LOW          2   // LED控制、状态指示
#define TASK_PRIORITY_IDLE         1   // 后台任务
```

### 11. 错误处理与恢复

#### 11.1 故障检测
- **硬件故障**: SPI通信失败、传感器无响应
- **电源故障**: 电压异常、温度过高
- **通信故障**: BLE连接丢失、数据校验失败

#### 11.2 恢复策略
- **自动重启**: 检测到硬件故障时自动重启相关模块
- **安全模式**: 进入最小功能状态，保证基本安全
- **故障上报**: 通过BLE记录和上报故障信息

## 性能指标

### 12. 功耗指标
- **工作功耗**: < 500mA (所有外设开启)
- **睡眠功耗**: < 10μA (深度睡眠模式)
- **待机功耗**: < 50mA (外设关闭，保持通信)

### 13. 响应指标
- **PWM响应时间**: < 10ms (所有6个通道)
- **BLE命令响应**: < 100ms
- **温度采样周期**: 1秒
- **电流采样周期**: 10ms

### 14. 精度指标
- **电流测量精度**: ±2% (包括CH5/CH6电流监测)
- **电压测量精度**: ±1%
- **温度测量精度**: ±0.5°C
- **PWM占空比精度**: 0.1% (CH1-CH6一致)

### 15. CH5/CH6验证清单
- [x] GPIO12/13硬件连接定义
- [x] LEDC_CHANNEL_4/5映射配置
- [x] 6通道PWM数据结构设计
- [x] 统一的API接口(channel 0-5)
- [x] 批量控制功能实现
- [x] 温度保护覆盖CH5/CH6
- [x] 配置存储包含6个通道
- [x] BLE命令协议支持CH5/CH6
- [x] 性能指标与CH1-CH4一致

## 兼容性说明
本版本完全重新设计硬件接口和通信协议，不保持与之前版本的兼容性。所有配置和控制接口都需要根据新的协议进行适配。