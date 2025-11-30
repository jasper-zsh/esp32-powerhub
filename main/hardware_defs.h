// ESP32 Power Hub Hardware Definitions
//
// This file defines the hardware configuration for the ESP32 power hub system.
// The sensor architecture supports both external ADC (ADC128S102) and built-in ESP32 ADC
// through a unified SensorManager interface.
//
// Sensor Manager Architecture:
// - VoltageSensorDriver: Base class for hardware-specific voltage measurement only
// - SensorManager: Parent class handling voltage-to-current conversion and calibration
// - ADC drivers inherit from VoltageSensorDriver and only return voltage values
// - Configuration-based dynamic channel mapping (1-6 channels, no total current)
// - C API compatibility maintained through current_sensor.h wrapper functions
//
// Power Voltage Monitoring:
// - External ADC mode: Power voltage handled by ULP coprocessor, not returned by driver
// - Built-in ADC mode: Power voltage read directly from GPIO1 (ADC_CHANNEL_1)
// - Voltage divider scaling: V_power = V_adc * (240k + 910k) / 240k
//
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PWR_GPIO 2
#define ABAT_GPIO 1

// 全局PWM硬件定义
#define PWM_CHANNEL_COUNT 6

// 温度传感器硬件定义
#define TEMP_SENSOR_COUNT 1
#define TEMP_SENSOR_GPIO_NUM 7

extern const int PWM_CHANNEL_GPIOS[PWM_CHANNEL_COUNT];

// ADC Hardware Configuration Selection
//
// The system supports two ADC hardware options through the same SensorManager API:
// - EXTERNAL_ADC: ADC128S102 connected via SPI (external current sensors)
// - BUILTIN_ADC: ESP32 internal ADC (direct GPIO connections)
//
// To switch between ADC types, comment/uncomment the appropriate line below:
#define EXTERNAL_ADC
// #define BUILTIN_ADC

#ifdef EXTERNAL_ADC
// ADC128S102 SPI Interface Configuration
// External ADC provides 8 channels (0-7) for individual channel current measurements
// Channels 0-5 are mapped to PWM channels CH6-CH1 respectively
#define EXTERNAL_ADC_CS_PIN 6      // SPI Chip Select GPIO
#define EXTERNAL_ADC_SCLK_PIN 5    // SPI Serial Clock GPIO
#define EXTERNAL_ADC_MISO_PIN 4    // SPI MISO GPIO
#define EXTERNAL_ADC_MOSI_PIN 3    // SPI MOSI GPIO
// Note: Power voltage is handled by ULP coprocessor, not external ADC
#endif

#ifdef BUILTIN_ADC
// ESP32 Internal ADC Configuration
// Current sensors connect directly to GPIO pins configured as ADC inputs
// Power voltage is read from GPIO1 (ADC_CHANNEL_1) with voltage divider scaling
// ADC unit is automatically determined by GPIO pin routing
#endif

// Current Sensor Configuration for Channel-Only Architecture
//
// The sensor configuration uses a flexible mapping system where:
// - adc_channel >= 0: Sensor is enabled and mapped to that ADC channel
// - adc_channel = -1: Sensor is disabled
// - Dynamic allocation: Only enabled sensors consume memory
// - Per-sensor calibration: Each sensor has independent offset and sensitivity
//
// Channel Current Sensors (ACS712):
// Each PWM channel has an independent current sensor mapped to ADC channels
extern const int CHANNEL_CURRENT_ADC_CHS[PWM_CHANNEL_COUNT];  // [-1 = disabled, 0-7 = ADC channel]
#define CHANNEL_CURRENT_OFFSET 1.65f        // Zero-point voltage reference (V)
#define CHANNEL_CURRENT_SENSITIVITY 0.066f  // Voltage-to-current conversion factor (V/A)

// Configuration Helper Macros:
// These macros support runtime validation and dynamic channel allocation
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
