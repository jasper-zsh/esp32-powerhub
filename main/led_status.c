/**
 * @file led_status.c
 * @brief LED状态指示模块源文件
 */

#include "led_status.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_rgb.h"
#include "esp_err.h"

// 颜色定义
#define LED_COLOR_GREEN    0x00FF00
#define LED_COLOR_BLUE     0x0000FF
#define LED_COLOR_RED      0xFF0000
#define LED_COLOR_YELLOW   0xFFFF00
#define LED_COLOR_WHITE    0xFFFFFF
#define LED_COLOR_OFF      0x000000

// 状态类型枚举
typedef enum {
    LED_STATUS_BLUETOOTH_CONNECTED,
    LED_STATUS_ERROR,
    LED_STATUS_LOW_BATTERY,
    LED_STATUS_MAX
} led_status_type_t;

// LED状态上下文
typedef struct {
    bool status[LED_STATUS_MAX];
    bool enabled;
    uint8_t brightness;
    uint32_t last_update_time;
    TimerHandle_t timer;
    bool bluetooth_advertising; // 新增：蓝牙广播状态
} led_status_context_t;

// 全局上下文
static led_status_context_t g_led_ctx = {0};
static const char *TAG = "led_status";

// 私有函数声明
static void led_status_timer_callback(TimerHandle_t xTimer);
static uint32_t get_led_color(led_status_context_t *ctx);
static void set_led_color(led_status_context_t *ctx, uint32_t color);

// 获取LED颜色
static uint32_t get_led_color(led_status_context_t *ctx) {
    // 错误状态优先级最高
    if (ctx->status[LED_STATUS_ERROR]) {
        return LED_COLOR_RED;
    }
    
    // 低电压状态次之
    if (ctx->status[LED_STATUS_LOW_BATTERY]) {
        return LED_COLOR_YELLOW;
    }
    
    // 蓝牙连接状态
    if (ctx->status[LED_STATUS_BLUETOOTH_CONNECTED]) {
        return LED_COLOR_GREEN;
    } else if (ctx->bluetooth_advertising) {
        // 蓝牙广播状态（蓝灯）
        return LED_COLOR_BLUE;
    } else {
        // 蓝牙未连接状态（关）
        return LED_COLOR_OFF;
    }
}

// 设置LED颜色
static void set_led_color(led_status_context_t *ctx, uint32_t color) {
    uint8_t eff = ctx->brightness;
    if (!ctx->status[LED_STATUS_ERROR] && !ctx->status[LED_STATUS_LOW_BATTERY]) {
        if (ctx->status[LED_STATUS_BLUETOOTH_CONNECTED]) {
            eff = (uint16_t)ctx->brightness * 32 / 255;
        }
    }
    uint8_t r = ((color >> 16) & 0xFF) * eff / 255;
    uint8_t g = ((color >> 8) & 0xFF) * eff / 255;
    uint8_t b = (color & 0xFF) * eff / 255;
    led_rgb_set_color(r, g, b);
}

// 定时器回调函数
static void led_status_timer_callback(TimerHandle_t xTimer) {
    led_status_update();
}

// 初始化LED状态指示模块
esp_err_t led_status_init(void) {
    // 注意：LED RGB驱动现在由main.c单独初始化
    // 这里不再调用 led_rgb_init()，避免重复初始化

    ESP_LOGI(TAG, "Initializing LED status manager...");
    
    // 明确初始化所有状态值
    for (int i = 0; i < LED_STATUS_MAX; i++) {
        g_led_ctx.status[i] = false;
    }
    g_led_ctx.bluetooth_advertising = false;
    
    // 初始化默认值
    g_led_ctx.enabled = true;
    g_led_ctx.brightness = 255;
    
    // 创建定时器
    g_led_ctx.timer = xTimerCreate("led_timer", pdMS_TO_TICKS(100), pdTRUE, NULL, led_status_timer_callback);
    if (g_led_ctx.timer == NULL) {
        ESP_LOGE(TAG, "Failed to create LED timer");
        return ESP_FAIL;
    }
    
    // 启动定时器
    if (xTimerStart(g_led_ctx.timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start LED timer");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "LED status module initialized: connected=%d, error=%d, advertising=%d", 
             g_led_ctx.status[LED_STATUS_BLUETOOTH_CONNECTED], 
             g_led_ctx.status[LED_STATUS_ERROR],
             g_led_ctx.bluetooth_advertising);
    return ESP_OK;
}

// 设置蓝牙连接状态
esp_err_t led_status_set_bluetooth_connected(bool connected) {
    g_led_ctx.status[LED_STATUS_BLUETOOTH_CONNECTED] = connected;
    return ESP_OK;
}

// 设置蓝牙广播状态
esp_err_t led_status_set_bluetooth_advertising(bool advertising) {
    g_led_ctx.bluetooth_advertising = advertising;
    return ESP_OK;
}

// 设置错误状态
esp_err_t led_status_set_error(bool error) {
    g_led_ctx.status[LED_STATUS_ERROR] = error;
    return ESP_OK;
}

// 设置低电压状态
esp_err_t led_status_set_low_battery(bool low) {
    g_led_ctx.status[LED_STATUS_LOW_BATTERY] = low;
    return ESP_OK;
}

// 设置LED亮度
esp_err_t led_status_set_brightness(uint8_t brightness) {
    g_led_ctx.brightness = brightness;
    return ESP_OK;
}

// 启用/禁用LED状态显示
esp_err_t led_status_enable(bool enable) {
    g_led_ctx.enabled = enable;
    if (!enable) {
        // 关闭LED
        led_rgb_off();
    }
    return ESP_OK;
}

// 更新LED显示状态
esp_err_t led_status_update(void) {
    // 如果未启用，则不更新
    if (!g_led_ctx.enabled) {
        return ESP_OK;
    }
    
    // 获取当前应该显示的颜色
    uint32_t color = get_led_color(&g_led_ctx);
    
    // 简单的闪烁实现（实际项目中可以优化）
    static bool blink_state = false;
    static uint32_t last_blink_time = 0;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // 根据颜色和状态确定闪烁频率
    uint32_t blink_interval = 0;
    if (color == LED_COLOR_RED) {
        blink_interval = 100;
    } else if (color == LED_COLOR_YELLOW && g_led_ctx.status[LED_STATUS_LOW_BATTERY]) {
        blink_interval = 100;
    } else if (color == LED_COLOR_BLUE && g_led_ctx.bluetooth_advertising) {
        blink_interval = 500;
    } else if (color == LED_COLOR_GREEN && g_led_ctx.status[LED_STATUS_BLUETOOTH_CONNECTED]) {
        blink_interval = 500;
    } else {
        blink_interval = 0;
    }
    
    // 更新闪烁状态
    if (blink_interval > 0 && current_time - last_blink_time > blink_interval) {
        blink_state = !blink_state;
        last_blink_time = current_time;
    } else if (blink_interval == 0) {
        // 常亮状态
        blink_state = true;
    }
    
    // 如果是闪烁状态且应该熄灭，则显示黑色
    if (!blink_state) {
        set_led_color(&g_led_ctx, LED_COLOR_OFF);
    } else {
        set_led_color(&g_led_ctx, color);
    }
    
    return ESP_OK;
}