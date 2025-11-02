idf_component_register(SRCS "adc128s102.c" "temp_mgr.c" "power_mgr.c" "led_rgb.c" "led_status.c" "command_codec.c" "scheduler.c" "preset_mgr.c" "main.c" "pwm_control.c" "storage.c" "ble_server.c" "onewire_gpio.c" "ds18b20_gpio.c"
                    INCLUDE_DIRS "."
                    REQUIRES soc esp_driver_spi esp_driver_gpio esp_driver_ledc esp_driver_rmt esp_adc nvs_flash bt esp_timer)

