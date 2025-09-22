set(ulp_app_name ulp_power)
set(ulp_riscv_sources "ulp/ulp_power.c")
set(ulp_exp_dep_srcs "power_mgr.c")

idf_component_register(SRCS "power_mgr.c" "led_rgb.c" "led_status.c" "command_codec.c" "scheduler.c" "preset_mgr.c" "main.c" "pwm_control.c" "storage.c" "ble_server.c"
                    INCLUDE_DIRS "."
                    REQUIRES ulp soc esp_adc nvs_flash bt)

ulp_embed_binary(${ulp_app_name} "${ulp_riscv_sources}" "${ulp_exp_dep_srcs}")

