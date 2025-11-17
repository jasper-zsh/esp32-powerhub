#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 全局PWM硬件定义
#define PWM_CHANNEL_COUNT 6

// 温度传感器硬件定义
#define TEMP_SENSOR_COUNT 2
#define TEMP_SENSOR_GPIO_NUM 7

extern const int PWM_CHANNEL_GPIOS[PWM_CHANNEL_COUNT];

#ifdef __cplusplus
}
#endif
