#include "hardware_defs.h"

// CH1~CH6 -> GPIO8~GPIO13
const int PWM_CHANNEL_GPIOS[PWM_CHANNEL_COUNT] = {8, 9, 10, 11, 12, 13};
const int CHANNEL_CURRENT_ADC_CHS[PWM_CHANNEL_COUNT] = {5, 4, 3, 2, 1, 0};

// const int PWM_CHANNEL_GPIOS[PWM_CHANNEL_COUNT] = {7};
// ADC channel mapping uses ADC1 channel index; GPIO2 corresponds to ADC1 channel 1 on ESP32-S3.
// const int CHANNEL_CURRENT_ADC_CHS[PWM_CHANNEL_COUNT] = {2};
