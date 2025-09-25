# 设计文档 - 电源管理功能（v003，简化版，ULP）

## 目标
- 用于汽车启动/熄火检测：电压恢复到发电机充电电压即唤醒；电压长时间回到静置电压即入睡。
- 保持实现最小化：固定采样周期、固定去抖计数、固定阈值（可通过 BLE 改）。

## 硬件与测量
- ADC：ESP32‑S3 ADC1_CH1 (GPIO2)，12bit，11dB 衰减（≈0–3.9V）。
- 分压：910kΩ / 240kΩ，分压比 k = 0.2087；VIN = (adc_raw/4095*Vref_mv)/k。
- Vref_mv 默认 3514（经现场校准初值），可在主核和 ULP 内统一使用。

## 阈值与判定
- 默认阈值：sleep = 12500mV；wake = 13200mV（可通过 BLE 设置）。
- 唤醒：ULP 每秒采样一次原始电压，检测到 VIN ≥ wake 即写入 `last_mv`、置 `wake_flag` 并唤醒主核，不做额外去抖；`wake_flag` 由主核在再次入睡前清零。
- 入睡：主核任务 1Hz 读取最近一次 ULP 测得的电压，连续 12 次（约 12s）低于 sleep 判定为“熄火”，随后触发深度睡眠。

## 运行与休眠分工
- 运行态（主核）：
  - 1Hz 读取电压用于上报和 LED 提示（不参与入睡决策）。
  - 当检测到低电压（<sleep）时点亮“低电”指示，并统计连续低压次数；达到 12 次时触发深度睡眠流程。
  - 接到“强制睡眠”命令时，准备并入睡。
- 深睡态（ULP）：
  - 固定 1s 周期唤醒，单次采样电压并计算 VIN_mV。
  - 满足 `VIN ≥ wake` 即写入 `last_mv`、置 `wake_flag` 并调用 `ulp_riscv_wakeup_main_processor()`。

## 流程
- 进入深睡：
  1) 停止 LED 指示与 PWM 输出，备份当前 PWM 状态；
  2) 关闭温度传感器，清除热保护状态与去抖计数；
  3) 清零 `wake_flag`，保持 ULP 运行并启用 `esp_sleep_enable_ulp_wakeup()`，随后调用 `esp_deep_sleep_start()`。
- 唤醒（ESP_SLEEP_WAKEUP_ULP）：
  1) 读取 `ulp_last_mv` 并记录日志；ULP 继续按照 1s 周期运行，无需重新加载程序；
  2) 主核恢复业务逻辑，继续 1Hz 轮询电压与温度。

## 共享数据（RTC SLOW MEM，整数）
- wake_mv(uint16_t)
- last_mv(uint16_t)
- wake_flag(uint8_t)
- cal_off_mv(int16_t，电压零点/比例综合偏移，单位 mV，BLE 可设置，主核与 ULP 共用）

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
- 台架：12.0/12.5/13.2/14.0V 点测；验证阈值跨越时 ULP 立即唤醒主核、低电压保持约 12s 后进入深睡；验证校准指令 0x05/0x06。
- 车载：启停与短时负载波动下，确认不会误唤醒或提前入睡，电压回落后重新计时约 12s 才入睡。
