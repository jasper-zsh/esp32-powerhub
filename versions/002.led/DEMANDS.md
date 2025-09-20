# 需求文档 - LED 状态指示（颜色-状态定义）

## 目标
实现一个 LED 状态指示系统，用于显示蓝牙、错误与电源低电等状态，并尽可能降低功耗。

## 颜色-状态定义

- 术语约定：
  - 显示模式：常亮、常灭、慢闪（1Hz，亮500ms/灭500ms）、快闪（5Hz，亮100ms/灭100ms）

- 映射表：

| 优先级 | 类别 | 具体状态 | 颜色 | 显示模式 | 备注 |
|---|---|---|---|---|---|
| 1 | 系统 | 错误 | 红色 | 快闪（5Hz） | 最高优先级，覆盖其他状态 |
| 2 | 电源 | 低电压 | 黄色 | 快闪（5Hz） | 明显提醒 |
| 3 | 蓝牙 | 已连接 | 绿色 | 慢闪（1Hz） | 正常连接状态 |
| 4 | 蓝牙 | 广播/等待连接 | 蓝色 | 慢闪（1Hz） | 处于可被连接的状态 |

- 状态优先级：错误 > 低电压 > 已连接 > 广播中
- 颜色定义（RGB）：
  - 绿色 0x00FF00
  - 蓝色 0x0000FF
  - 红色 0xFF0000
  - 黄色 0xFFFF00

## 功能要求
- 使用 WS2812 RGB LED（GPIO21）
- 提供 API：
  - led_status_init()
  - led_status_set_bluetooth_connected(bool connected)
  - led_status_set_bluetooth_advertising(bool advertising)
  - led_status_set_error(bool error)
  - led_status_set_low_battery(bool low)
  - led_status_set_brightness(uint8_t brightness)
  - led_status_enable(bool enable)
  - led_status_update()

## 非功能要求
- 响应时间 < 100ms；不影响主流程
- 无“呼吸效果”与自动降亮度设计