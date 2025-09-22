# 设计文档 - 电源管理功能（v003，简化版，ULP）

## 目标
- 用于汽车启动/熄火检测：电压恢复到发电机充电电压即唤醒；电压长时间回到静置电压即入睡。
- 保持实现最小化：固定采样周期、固定去抖计数、固定阈值（可通过 BLE 改）。

## 硬件与测量
- ADC：ESP32‑S3 ADC1_CH1 (GPIO2)，12bit，11dB 衰减（≈0–3.9V）。
- 分压：910kΩ / 240kΩ，分压比 k = 0.2087；VIN = (adc_raw/4095*Vref_mv)/k。
- Vref_mv 默认 3514（经现场校准初值），可在主核和 ULP 内统一使用。

## 阈值与去抖
- 默认阈值：sleep = 12500mV；wake = 13200mV（可通过 BLE 设置）。
- 去抖：
  - 唤醒：连续 2 次（2s）测得 VIN ≥ wake 判为“启动”，立即唤醒主核。
  - 入睡：连续 10 次（10s）测得 VIN < sleep 判为“熄火”，进入深度睡眠。
- 迟滞：位于 [sleep, wake) 区间保持当前状态。
- 唤醒宽限：唤醒后 10s 内忽略入睡判断（避免短时波动）。

## 运行与休眠分工
- 运行态（主核）：
  - 1Hz 读取电压用于上报和 LED 提示（不参与入睡决策）。
  - 当检测到低电压（<sleep）时仅点亮“低电”指示，不立即睡眠。
  - 接到“强制睡眠”命令时，准备并入睡。
- 深睡态（ULP）：
  - 固定 1s 周期唤醒，采样 8 次求平均，换算 VIN_mV。
  - 维护 wake/sleep 去抖计数并判定；满足唤醒条件时置标志并唤醒主核；满足入睡条件时保持深睡（无动作）。

## 流程
- 进入深睡：
  1) 保存阈值/计数器/最近电压到 RTC SLOW MEM；备份 PWM/标志；关闭外设。
  2) 写入 ULP 共用内存（阈值、计数清零、周期=1s），使能 esp_sleep_enable_ulp_wakeup()。
  3) esp_deep_sleep_start()。
- 唤醒（ESP_SLEEP_WAKEUP_ULP）：
  1) 读取 ULP 标志与最近电压；恢复 PWM/LED/BLE；启动 10s 宽限计时。
  2) 继续正常运行；入睡决策交由 ULP 处理。

## 共享数据（RTC SLOW MEM，整数）
- sleep_mv(uint16_t), wake_mv(uint16_t)
- last_mv(uint16_t)
- wake_cnt(uint8_t), sleep_cnt(uint8_t)
- grace_s(uint8_t)（唤醒后宽限剩余秒，主核维护）
- wake_flag(uint8_t)
- cal_off_mv(int16_t，电压零点/比例综合偏移，单位 mV，BLE 可设置，应用于主核与 ULP）

## BLE 接口（0xFFF5）
- READ：返回 [vin_mv(2B) | sleep_mv(2B) | wake_mv(2B)]（BE）。
- WRITE：cmd(1B)+param(2B)
  - 0x01 设置 sleep 阈值（mV）
  - 0x02 设置 wake 阈值（mV）
  - 0x03 强制进入睡眠
  - 0x04 取消待睡/强制唤醒（清计数与标志）
  - 0x05 设置校准偏移 cal_off_mv（mV，带符号，范围 -2000..+2000）
  - 0x06 提供实际电压（mV），设备自动计算并保存校准偏移（off = actual - measured）

## 与现有模块
- led_status：<sleep 显示低电；入睡前关闭；唤醒后恢复。
- pwm_control/storage/scheduler/ble：入睡前停止并备份；唤醒后恢复。

## 集成与构建
- 新增模块：main/power_mgr.[ch]、ulp/ulp_power.S 或 C（RISC‑V）。
- CMake：在 main/CMakeLists.txt 引入 ULP 构建与符号导出；固化 1s 周期常量。

## 测试
- 台架：12.0/12.5/13.2/14.0V 点测；验证 2s 唤醒与 10s 入睡；验证校准指令 0x05/0x06。
- 车载：启停与短时负载波动下，唤醒稳定且无误睡；唤醒后 10s 内忽略下跌验证。
