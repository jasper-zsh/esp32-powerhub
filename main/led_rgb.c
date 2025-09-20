#include "led_rgb.h"
#include "driver/rmt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "led_rgb";

#define WS2812_LED_GPIO 21
#define RMT_CHANNEL RMT_CHANNEL_0
#define RMT_MEM_BLOCK_NUM (1)

// WS2812 timing constants
#define T0H_NS (350)
#define T0L_NS (1000)
#define T1H_NS (1000)
#define T1L_NS (350)
#define RESET_US (280)

// Current LED color state
static uint8_t s_red = 0;
static uint8_t s_green = 0;
static uint8_t s_blue = 0;

static void ws2812_rmt_adapter(uint8_t red, uint8_t green, uint8_t blue, rmt_item32_t *items) {
    int i;
    uint32_t value = (green << 16) | (red << 8) | blue;
    
    for (i = 0; i < 24; i++) {
        if (value & (1 << (23 - i))) {
            items[i].level0 = 1;
            items[i].duration0 = T1H_NS / 100; // 100ns unit
            items[i].level1 = 0;
            items[i].duration1 = T1L_NS / 100;
        } else {
            items[i].level0 = 1;
            items[i].duration0 = T0H_NS / 100;
            items[i].level1 = 0;
            items[i].duration1 = T0L_NS / 100;
        }
    }
}

esp_err_t led_rgb_init(void) {
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(WS2812_LED_GPIO, RMT_CHANNEL);
    config.clk_div = 2;
    config.mem_block_num = RMT_MEM_BLOCK_NUM;
    
    esp_err_t err = rmt_config(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure RMT: %s", esp_err_to_name(err));
        return err;
    }
    
    err = rmt_driver_install(config.channel, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install RMT driver: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "WS2812 LED initialized on GPIO %d", WS2812_LED_GPIO);
    return ESP_OK;
}

esp_err_t led_rgb_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    rmt_item32_t items[24];
    ws2812_rmt_adapter(red, green, blue, items);
    
    esp_err_t err = rmt_write_items(RMT_CHANNEL, items, 24, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write RMT items: %s", esp_err_to_name(err));
        return err;
    }
    
    // Store current color
    s_red = red;
    s_green = green;
    s_blue = blue;
    
    return ESP_OK;
}

esp_err_t led_rgb_off(void) {
    return led_rgb_set_color(0, 0, 0);
}

esp_err_t led_rgb_set_status(bool is_connected, bool has_error) {
    if (has_error) {
        // Red color for error
        return led_rgb_set_color(255, 0, 0);
    } else if (is_connected) {
        // Blue color for connected
        return led_rgb_set_color(0, 0, 255);
    } else {
        // Green color for idle/ready
        return led_rgb_set_color(0, 255, 0);
    }
}
