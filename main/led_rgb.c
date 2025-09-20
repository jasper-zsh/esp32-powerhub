#include "led_rgb.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "led_rgb";

#define WS2812_LED_GPIO 21
#define RMT_CHANNEL_CLK_HZ 10000000 // 10MHz resolution, 100ns tick

// WS2812 timing constants
#define T0H_NS (350)
#define T0L_NS (1000)
#define T1H_NS (1000)
#define T1L_NS (350)

// Current LED color state
static uint8_t s_red = 0;
static uint8_t s_green = 0;
static uint8_t s_blue = 0;
static rmt_channel_handle_t s_led_channel = NULL;
static rmt_encoder_handle_t s_copy_encoder = NULL;

typedef struct {
    rmt_symbol_word_t level0;
    rmt_symbol_word_t level1;
} ws2812_symbol_pair_t;

static void ws2812_rmt_adapter(uint8_t red, uint8_t green, uint8_t blue, rmt_symbol_word_t *symbols) {
    uint32_t value = (red << 16) | (green << 8) | blue;
    
    for (int i = 0; i < 24; i++) {
        if (value & (1 << (23 - i))) {
            // Logic 1: high for 1000ns, low for 350ns
            symbols[i].duration0 = (T1H_NS + 50) / 100; // Convert to 100ns ticks, round up
            symbols[i].level0 = 1;
            symbols[i].duration1 = (T1L_NS + 50) / 100;
            symbols[i].level1 = 0;
        } else {
            // Logic 0: high for 350ns, low for 1000ns
            symbols[i].duration0 = (T0H_NS + 50) / 100;
            symbols[i].level0 = 1;
            symbols[i].duration1 = (T0L_NS + 50) / 100;
            symbols[i].level1 = 0;
        }
    }
}

esp_err_t led_rgb_init(void) {
    ESP_LOGI(TAG, "Initializing WS2812 LED on GPIO %d", WS2812_LED_GPIO);
    
    // Initialize RMT TX channel
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = WS2812_LED_GPIO,
        .mem_block_symbols = 48, // 24 bits * 2 (since each bit is represented by 2 RMT symbols)
        .resolution_hz = RMT_CHANNEL_CLK_HZ,
        .trans_queue_depth = 4,
        .flags.with_dma = false,
    };
    
    esp_err_t err = rmt_new_tx_channel(&tx_chan_config, &s_led_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel: %s", esp_err_to_name(err));
        return err;
    }
    
    // Create copy encoder
    rmt_copy_encoder_config_t copy_encoder_config = {};
    err = rmt_new_copy_encoder(&copy_encoder_config, &s_copy_encoder);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT copy encoder: %s", esp_err_to_name(err));
        return err;
    }
    
    // Enable the channel
    err = rmt_enable(s_led_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT TX channel: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "WS2812 LED initialized on GPIO %d", WS2812_LED_GPIO);
    return ESP_OK;
}

esp_err_t led_rgb_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    if (s_led_channel == NULL || s_copy_encoder == NULL) {
        ESP_LOGE(TAG, "RMT not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_red == red && s_green == green && s_blue == blue) {
        return ESP_OK;
    }
    
    rmt_symbol_word_t symbols[24];
    ws2812_rmt_adapter(red, green, blue, symbols);
    
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };
    
    esp_err_t err = rmt_transmit(s_led_channel, s_copy_encoder, symbols, sizeof(symbols), &tx_config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "RMT transmit failed: %s", esp_err_to_name(err));
        return err;
    }
    
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
