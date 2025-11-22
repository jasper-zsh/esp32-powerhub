#include <stdint.h>
#include "ulp_riscv_utils.h"
#include "ulp_riscv_adc_ulp_core.h"

// ULP变量 - 主处理器和ULP共享
uint16_t sleep_mv;
uint16_t wake_mv;
uint16_t last_mv;
int16_t cal_off_mv;
uint8_t wake_flag;
uint16_t min_sleep_ticks;
uint8_t ok_cnt;
uint16_t wake_hyst_mv;

// ADC读数转换为输入电压(mV)
// 适配ESP32S3的分压比和ADC特性
static uint16_t raw_to_vin_mv(int32_t raw) {
    if (raw < 0) raw = 0;

    // ADC原始值转换为GPIO电压(mV)
    // ADC参考电压3.3V，12位ADC (0-4095)
    int32_t gpio_mv = (raw * 3300) / 4095;

    // 根据最新的硬件定义：分压电阻为240k:910k
    // 实际分压比 = (240k+910k)/910k ≈ 1.2637（GPIO电压 * 115 / 91）
    // 这里使用整数运算：(gpio_mv * 115000) / 91000 约等于 *1.2637
    int32_t vin_mv = (gpio_mv * 115000) / 91000;

    // 应用校准偏移
    vin_mv += (int32_t)cal_off_mv;

    // 限制范围
    if (vin_mv < 0) vin_mv = 0;
    if (vin_mv > 65535) vin_mv = 65535;

    return (uint16_t)vin_mv;
}

// ULP主函数 - 电压监控
int main(void) {
    int32_t l_result = -32768;
    int32_t result = 0;

    // 稳定ADC读数 - 确保读数稳定
    for (;;) {
        result = ulp_riscv_adc_read_channel(ADC_UNIT_1, 1);  // ADC1_GPIO1
        int32_t d = result - l_result;
        if (d > -10 && d < 10) {
            break;  // 读数稳定
        }
        l_result = result;
    }

    // 转换为实际电压
    uint16_t vin = raw_to_vin_mv(result);
    last_mv = vin;

    // 强制最小睡眠时间，避免频繁唤醒
    if (min_sleep_ticks > 0) {
        min_sleep_ticks--;
        return 0;
    }

    // 计算唤醒阈值(带回差)
    uint32_t thr = (uint32_t)wake_mv + (uint32_t)wake_hyst_mv;
    if (thr > 65535) thr = 65535;

    // 连续检测阈值 - 需要连续3次满足条件才唤醒
    if (vin >= thr) {
        if (ok_cnt < 3) ok_cnt++;
    } else {
        ok_cnt = 0;
    }

    // 满足唤醒条件
    if (ok_cnt >= 3) {
        wake_flag = 1;
        ulp_riscv_wakeup_main_processor();
    }

    return 0;
}
