# ULP配置
set(ulp_app_name ulp_power)
set(ulp_riscv_sources "ulp/ulp_power.c")
set(ulp_exp_dep_srcs "power_mgr.c")

idf_component_register(SRCS "adc128s102.c" "temp_mgr.c" "power_mgr.c" "led_rgb.c" "led_status.c" "command_codec.c" "scheduler.c" "main.c" "pwm_control.c" "storage.c" "ble_server.c" "onewire_gpio.c" "ds18b20_gpio.c" "system_monitor.c"
                    INCLUDE_DIRS "."
                    REQUIRES soc esp_driver_spi esp_driver_gpio esp_driver_ledc esp_driver_rmt esp_adc nvs_flash bt esp_timer ulp)

# 嵌入ULP二进制
ulp_embed_binary(${ulp_app_name} "${ulp_riscv_sources}" "${ulp_exp_dep_srcs}")

