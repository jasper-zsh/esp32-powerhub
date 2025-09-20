# 设计文档 - LED 状态指示（v002）

## 目标
实现 LED 状态指示，用于显示蓝牙、错误、低电压等状态，并尽可能降低功耗。

## 架构
- 硬件：WS2812 RGB（GPIO21），通过 RMT 控制
- 组件：
  - 状态管理：维护状态位与亮度
  - 显示控制：根据状态与优先级输出颜色/闪烁
  - 驱动适配：基于 led_rgb 模块（RMT 发码）
  - API：对外状态设置接口

## 数据结构
```c
typedef enum {
    LED_STATUS_BLUETOOTH_CONNECTED,
    LED_STATUS_ERROR,
    LED_STATUS_LOW_BATTERY,
    LED_STATUS_MAX
} led_status_type_t;

typedef struct {
    bool status[LED_STATUS_MAX];
    bool enabled;
    uint8_t brightness;          // 全局基准亮度 0-255
    uint32_t last_update_time;   // 供将来扩展
    bool bluetooth_advertising;  // 蓝牙广播中
} led_status_context_t;
```

## 颜色与模式
- 颜色宏（RGB）：
```c
#define LED_COLOR_GREEN  0x00FF00
#define LED_COLOR_BLUE   0x0000FF
#define LED_COLOR_RED    0xFF0000
#define LED_COLOR_YELLOW 0xFFFF00
#define LED_COLOR_OFF    0x000000
```
- 状态优先级：错误 > 低电压 > 已连接 > 广播中
- 映射：
  - 错误：红色，快闪 5Hz（100ms 周期一半亮一半灭）
  - 低电压：黄色，快闪 5Hz
  - 蓝牙已连接：绿色，慢闪 1Hz（500ms/500ms）
  - 蓝牙广播/等待连接：蓝色，慢闪 1Hz
- 亮度策略：
  - 已连接：按基准亮度的 12.5% 输出（brightness * 32 / 255）
  - 广播/未连接：按基准亮度的 100% 输出

## API 接口
```c
esp_err_t led_status_init(void);
esp_err_t led_status_update(void);
esp_err_t led_status_set_bluetooth_connected(bool connected);
esp_err_t led_status_set_bluetooth_advertising(bool advertising);
esp_err_t led_status_set_error(bool error);
esp_err_t led_status_set_low_battery(bool low);
esp_err_t led_status_set_brightness(uint8_t brightness);
esp_err_t led_status_enable(bool enable);
```

## 逻辑与时序
- 颜色决策（伪码）：
```c
if (error) return RED;
if (low_batt) return YELLOW;
if (bt_connected) return GREEN;
if (bt_advertising) return BLUE;
return OFF; // 正常流程下不会出现“空闲”设计，不作为对外状态
```
- 闪烁决策（伪码）：
```c
RED/YELLOW -> 100ms 切换（5Hz）
GREEN/BLUE -> 500ms 切换（1Hz）
```
- 亮度计算（伪码）：
```c
uint8_t eff = brightness;
if (!error && !low_batt && bt_connected) eff = brightness * 32 / 255;
```

## 实现要点
- 使用 FreeRTOS 软件定时器每 100ms 调度一次 led_status_update
- led_rgb 通过 RMT 发码（顺序与硬件验证一致）；避免重复初始化与阻塞等待导致的超时
- 不实现呼吸效果与自动降亮度

## 测试
- 验证四种状态颜色/频率与优先级覆盖关系
- 与 BLE 集成：连接/断开/广播切换显示
- 长时间运行稳定性与功耗观察
