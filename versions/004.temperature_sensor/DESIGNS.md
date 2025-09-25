# 设计文档 - 温度传感器与热保护（v004）

## 总体架构
- 新增模块：temp_mgr（DS18B20 管理与采样）
- 扩展模块：
  - power_mgr：整合温度状态、扩展 0xFFF5 BLE 接口、统一周期通知
  - scheduler：提供通道运行态快照/恢复接口
  - ble_server：保持现有 GATT 结构，0xFFF5 回调仍由 power_mgr 提供

## 引脚与电源时序
- 正常运行：持续上电
  - GPIO13=LOW（GND）、GPIO11=HIGH（VDD）、GPIO12（DQ）由 One-Wire 驱动
- 深度睡眠/下电：GPIO11/12/13 置为输入（高阻）
- 上电/唤醒流程：置位供电引脚 → 稳定延时 ≥ 10ms → 安装 RMT 1-Wire 总线并枚举设备 → 首次采样

## temp_mgr 模块
- 接口
  - esp_err_t temp_mgr_init(void)
  - esp_err_t temp_mgr_sample_once(int16_t *out_temp);
- 实现
  - 使用 managed components：onewire_bus（RMT 实现）+ ds18b20，不自行 bit-bang
  - 初始化：GPIO13=LOW、GPIO11=HIGH，上电稳定≥10ms；安装 1-Wire RMT 总线；枚举设备并记录 present
  - 采样：调用 ds18b20_trigger_temperature_conversion_for_all(bus) 后直接 ds18b20_get_temperature(handle)，由驱动内部等待转换完成；将摄氏度转换为 0.01℃ 的 int16，越界丢弃
  - CRC：ds18b20_get_temperature 进行 CRC 校验，返回 ESP_ERR_INVALID_CRC 表示失败
  - 无设备：初始化时未发现设备则保持 ds18b20 句柄为 NULL；采样阶段返回 ESP_ERR_NOT_FOUND
  - 反初始化：GPIO11/12/13 置为高阻

## 热保护状态机（power_mgr 驱动）
- 配置项（NVS）：
  - KEY_T_HI=int16 高温阈值（默认 6000）
  - KEY_T_REC=int16 恢复阈值（默认 5500）
- 运行态：
  - NORMAL：温度 < T_HI 或已从 HOT 恢复
  - HOT：温度 ≥ T_HI 连续 3 次 → 触发
  - RECOVER：温度 ≤ T_REC 连续 3 次 → 从 HOT 退出至 NORMAL
- 触发动作：
  - HOT：保存通道运行态快照 → 关断 CH1~CH4 → 标记热保护中
  - RECOVER：按快照恢复模式与参数（不恢复进度）→ 清除热保护标记

## 通道运行态快照/恢复（scheduler 扩展）
- 新增接口：
  - esp_err_t scheduler_get_channel_snapshot(uint8_t ch, control_cmd_t *out_cmd);
  - esp_err_t scheduler_restore_channel_snapshot(uint8_t ch, const control_cmd_t *cmd);
- 约定：
  - 使用 control_cmd_t 表示通道"当前模式与参数"。示例：
    - mode=0x00 payload[0]=value
    - mode=0x01 payload[0]=value, payload[1..2]=duration_ms
    - mode=0x02 payload[0..1]=period_ms
    - mode=0x03 payload[0]=count, payload[1..2]=dur_ms, payload[3..4]=pause_ms
  - 关断实现：调用 pwm_control_stop_pattern + pwm_control_apply(ch, 0, 0)

## BLE 0xFFF5 扩展
- READ（12 字节）：
  - uint16 电压mV
  - int16 温度0.01℃
  - int16 高温阈值0.01℃
  - int16 恢复阈值0.01℃
  - uint8 状态标志（bit0:热保护中 bit1:温度有效 bit2:保留 bit3:深睡策略中）
  - uint8 保留
- WRITE：
  - 0x01 设置睡眠阈值mV（保留）
  - 0x02 设置唤醒阈值mV（保留）
  - 0x03 强制睡眠（保留）
  - 0x04 强制唤醒（保留）
  - 0x11 设置高温阈值（int16 0.01℃）
  - 0x12 设置恢复阈值（int16 0.01℃）
- NOTIFY：
  - 订阅后由 power_mgr 每 1000ms 推送一帧，负载同 READ 前 4 字段

## power_mgr 时序与任务
- 复用 power_mgr_task 的 1s 心跳：
  - 读电压（ULP）
  - 调用 temp_mgr_sample_once（轮询式）并直接使用返回值
  - 执行热保护状态机（去抖 3 次）
  - 若有订阅：按固定 1s 调用 ble_gatts_notify 更新 0xFFF5
- 进入深睡前：
  - 热保护标志清理，温度供电脚置高阻
- 唤醒后：
  - 恢复供电 → temp_mgr_init → 首次采样后进入正常轮询

## NVS 设计
- 命名空间："power_mgr"
- KEY_T_HI="t_hi"，KEY_T_REC="t_rec"（int16）
- 读失败时采用默认并写回

## 错误与退避
- temp CRC/无设备：标志位清零，不更新温度值；记录错误计数
- 若连续错误 ≥ 3 次：每次仍按 1s 周期尝试，无额外 backoff（简化）

## 兼容性
- 保留既有 0xFFF5 行为与命令（0x01..0x04），扩展读/写/通知；不引入新的 Characteristic
- 不影响现有 LED、预设与 PWM 控制

## 测试要点
- 正常采样与 CRC 校验
- 超温触发：关断、快照内容、恢复后模式一致
- 并发：在爆闪/渐变等模式下的恢复正确性（不要求进度连续）
- BLE：每秒通知频率与读写阈值兼容性
- 深睡/唤醒：供电与初始化流程正确
