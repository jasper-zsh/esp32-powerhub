#include <stdint.h>
#include "ulp_riscv_utils.h"
#include "ulp_riscv_adc_ulp_core.h"

uint16_t sleep_mv;
uint16_t wake_mv;
uint16_t last_mv;
int16_t cal_off_mv;
uint8_t wake_flag;

static uint16_t raw_to_vin_mv(int32_t raw) {
    if (raw < 0) raw = 0;
    int32_t gpio_mv = (raw * 3514) / 4095;
    int32_t vin_mv = (gpio_mv * 479 + 50) / 100;
    vin_mv += (int32_t)cal_off_mv;
    if (vin_mv < 0) vin_mv = 0;
    if (vin_mv > 65535) vin_mv = 65535;
    return (uint16_t)vin_mv;
}

int main(void) {
    int32_t l_result = -32768;
    int32_t result = 0;
    for (;;) {
        result = ulp_riscv_adc_read_channel(ADC_UNIT_1, 1);
        int32_t d = result - l_result;
        if (d > -10 && d < 10) {
            break;
        }
        l_result = result;
    }

    uint16_t vin = raw_to_vin_mv(result);
    last_mv = vin;

    if (vin >= wake_mv && wake_flag == 0) {
        wake_flag = 1;
        ulp_riscv_wakeup_main_processor();
    }

    return 0;
}
