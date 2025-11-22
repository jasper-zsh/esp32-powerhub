#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 全局PWM硬件定义
#define PWM_CHANNEL_COUNT 1

// 温度传感器硬件定义
#define TEMP_SENSOR_COUNT 0
#define TEMP_SENSOR_GPIO_NUM 7

extern const int PWM_CHANNEL_GPIOS[PWM_CHANNEL_COUNT];

// #define EXTERNAL_ADC
// #define EXTERNAL_ADC_CS_PIN 3
// #define EXTERNAL_ADC_SCLK_PIN 4
// #define EXTERNAL_ADC_MISO_PIN 5
// #define EXTERNAL_ADC_MOSI_PIN 6
#define BUILTIN_ADC

// 电流传感器通道，-1为没有该传感器
#define TOTAL_CURRENT_CH -1
#define TOTAL_CURRENT_OFFSET 2.5f
#define TOTAL_CURRENT_SENSITIVITY 0.020f

// 各PWM通道对应的ADC电流传感器通道，-1为没有该传感器
extern const int CHANNEL_CURRENT_ADC_CHS[PWM_CHANNEL_COUNT];
#define CHANNEL_CURRENT_OFFSET 1.65f
#define CHANNEL_CURRENT_SENSITIVITY 0.066f

// 自适应配置辅助宏
#define HAS_TOTAL_CURRENT_SENSOR() (TOTAL_CURRENT_CH >= 0)
#define HAS_CHANNEL_CURRENT_SENSOR(ch) (CHANNEL_CURRENT_ADC_CHS[ch] >= 0)

// 计算有效的电流传感器数量
#define CALC_CHANNEL_CURRENT_SENSOR_COUNT() \
    ({ \
        int count = 0; \
        for (int i = 0; i < PWM_CHANNEL_COUNT; i++) { \
            if (CHANNEL_CURRENT_ADC_CHS[i] >= 0) count++; \
        } \
        count; \
    })

#ifdef __cplusplus
}
#endif
