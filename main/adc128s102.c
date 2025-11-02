#include "adc128s102.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include <string.h>
#include <math.h>
#include "host/ble_hs.h"
#include "power_mgr.h"
#include "temp_mgr.h"

struct ble_gatt_access_ctxt;

static const char *TAG = "adc128s102";

// SPI配置 - 根据最新硬件定义更新
#define ADC_CS_PIN               3   // GPIO3 (CS)
#define ADC_SCLK_PIN             4   // GPIO4 (SCLK)
#define ADC_MISO_PIN             5   // GPIO5 (MISO)
#define ADC_MOSI_PIN             6   // GPIO6 (MOSI)

// SPI主机配置
#define ADC_SPI_HOST             SPI2_HOST
#define ADC_SPI_CLOCK_SPEED      1000000  // 1MHz

// ADC参数
#define ADC_RESOLUTION           4096.0f  // 12位ADC
#define ADC_VREF                 5.0f     // 5V参考电压

// ACS758参数 (100A版本)
#define ACS758_DEFAULT_OFFSET    2.5f     // V (零电流偏置)
#define ACS758_DEFAULT_SENSITIVITY 0.020f // V/A (20mV/A)

// ACS712参数 (20A版本)
#define ACS712_DEFAULT_OFFSET    2.5f     // V (零电流偏置)
#define ACS712_DEFAULT_SENSITIVITY 0.100f // V/A (100mV/A)

// 滤波参数
#define FILTER_WINDOW_SIZE       10

// 全局变量
static spi_device_handle_t s_spi_handle = NULL;
static TaskHandle_t s_sampling_task_handle = NULL;
static current_sensor_data_t s_latest_data = {0};
static SemaphoreHandle_t s_data_mutex = NULL;
static adc_calibration_t s_calibration = {
    .acs758_offset = ACS758_DEFAULT_OFFSET,
    .acs758_sensitivity = ACS758_DEFAULT_SENSITIVITY,
    .acs712_offset = ACS712_DEFAULT_OFFSET,
    .acs712_sensitivity = ACS712_DEFAULT_SENSITIVITY,
    .enable_filter = true
};

// 滤波缓冲区
static struct {
    uint16_t buffer[FILTER_WINDOW_SIZE];
    uint8_t index;
    bool filled;
} s_filter_buffers[ADC128S102_CHANNEL_COUNT];

// BLE通知相关
static uint16_t s_ble_conn_handle = 0;
static uint16_t s_ble_attr_handle = 0;
static bool s_ble_notifications_enabled = false;

// 内部函数声明
static esp_err_t adc128s102_spi_init(void);
static esp_err_t adc128s102_read_channel_raw(uint8_t channel, uint16_t *raw_value);
static float raw_to_current(uint8_t channel, uint16_t raw_value);
static void filter_push(uint8_t channel, uint16_t value);
static uint16_t filter_get_average(uint8_t channel);
static void sampling_task(void *arg);

// 初始化ADC128S102
esp_err_t adc128s102_init(void) {
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Initializing ADC128S102...");

    // 创建数据互斥锁
    s_data_mutex = xSemaphoreCreateMutex();
    if (!s_data_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // 初始化SPI接口
    ret = adc128s102_spi_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI");
        return ret;
    }

    // 初始化滤波缓冲区
    memset(s_filter_buffers, 0, sizeof(s_filter_buffers));

    ESP_LOGI(TAG, "ADC128S102 initialized successfully");
    return ESP_OK;
}

// SPI初始化
static esp_err_t adc128s102_spi_init(void) {
    esp_err_t ret = ESP_OK;

    // 配置SPI总线
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = ADC_MOSI_PIN,
        .miso_io_num = ADC_MISO_PIN,
        .sclk_io_num = ADC_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    ret = spi_bus_initialize(ADC_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // 配置SPI设备
    spi_device_interface_config_t dev_cfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,  // MODE 0
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = ADC_SPI_CLOCK_SPEED,
        .input_delay_ns = 0,
        .spics_io_num = ADC_CS_PIN,
        .flags = 0,  // 全双工模式
        .queue_size = 1
    };

    ret = spi_bus_add_device(ADC_SPI_HOST, &dev_cfg, &s_spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPI initialized successfully");
    return ESP_OK;
}

// 读取单个通道ADC原始值
esp_err_t adc128s102_read_raw(uint8_t channel, uint16_t *raw_value) {
    if (channel >= ADC128S102_CHANNEL_COUNT || !raw_value) {
        return ESP_ERR_INVALID_ARG;
    }

    return adc128s102_read_channel_raw(channel, raw_value);
}

// 读取单个通道原始值 (内部实现)
static esp_err_t adc128s102_read_channel_raw(uint8_t channel, uint16_t *raw_value) {
    if (!s_spi_handle) {
        return ESP_ERR_INVALID_STATE;
    }

    // ADC128S102通信协议：16位时钟周期，包含通道选择和数据读取
    uint8_t tx_data[2] = {0};
    uint8_t rx_data[2] = {0};

    // 构造命令：前4位为通道选择，后12位为 dummy 数据
    // 命令格式：0000xxxx xxxxxxxx，其中xxxx是通道号(0-7)
    tx_data[0] = (channel & 0x07) << 4;  // 通道选择移到高4位

    spi_transaction_t trans = {
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
        .length = 16,      // 16位时钟周期
        .rxlength = 16     // 同时接收16位数据
    };

    esp_err_t ret = spi_device_transmit(s_spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 提取12位ADC值 (MSB first)
    uint16_t adc_value = ((rx_data[0] & 0x0F) << 8) | rx_data[1];
    *raw_value = adc_value & 0x0FFF;

    return ESP_OK;
}

// 读取单个通道电流值
esp_err_t adc128s102_read_current(uint8_t channel, float *current) {
    if (channel >= ADC128S102_CHANNEL_COUNT || !current) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw_value;
    esp_err_t ret = adc128s102_read_raw(channel, &raw_value);
    if (ret != ESP_OK) {
        return ret;
    }

    *current = raw_to_current(channel, raw_value);
    return ESP_OK;
}

// 批量读取所有电流数据
esp_err_t adc128s102_read_all_currents(current_sensor_data_t *data) {
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    // 读取总输入电流 (IN0)
    ret = adc128s102_read_current(ADC_CHANNEL_TOTAL_CURRENT, &data->total_input_current);
    if (ret != ESP_OK) {
        return ret;
    }

    // 读取各通道电流 (IN1-IN6)
    for (int i = 0; i < 6; i++) {
        ret = adc128s102_read_current(ADC_CHANNEL_CH1_CURRENT + i, &data->channel_currents[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    data->timestamp = esp_timer_get_time() / 1000;  // 转换为毫秒
    return ESP_OK;
}

// 将ADC原始值转换为电流
static float raw_to_current(uint8_t channel, uint16_t raw_value) {
    // 将ADC值转换为电压
    float voltage = (float)raw_value * ADC_VREF / ADC_RESOLUTION;

    float current = 0.0f;

    if (channel == ADC_CHANNEL_TOTAL_CURRENT) {
        // ACS758计算
        current = (voltage - s_calibration.acs758_offset) / s_calibration.acs758_sensitivity;
    } else if (channel >= ADC_CHANNEL_CH1_CURRENT && channel <= ADC_CHANNEL_CH6_CURRENT) {
        // ACS712计算
        current = (voltage - s_calibration.acs712_offset) / s_calibration.acs712_sensitivity;
    }

    return current;
}

// 滤波：推入新值
static void filter_push(uint8_t channel, uint16_t value) {
    if (channel >= ADC128S102_CHANNEL_COUNT) {
        return;
    }

    s_filter_buffers[channel].buffer[s_filter_buffers[channel].index] = value;
    s_filter_buffers[channel].index = (s_filter_buffers[channel].index + 1) % FILTER_WINDOW_SIZE;

    if (!s_filter_buffers[channel].filled && s_filter_buffers[channel].index == 0) {
        s_filter_buffers[channel].filled = true;
    }
}

// 滤波：获取平均值
static uint16_t filter_get_average(uint8_t channel) {
    if (channel >= ADC128S102_CHANNEL_COUNT) {
        return 0;
    }

    uint32_t sum = 0;
    uint8_t count = s_filter_buffers[channel].filled ? FILTER_WINDOW_SIZE : s_filter_buffers[channel].index;

    if (count == 0) {
        return 0;
    }

    for (uint8_t i = 0; i < count; i++) {
        sum += s_filter_buffers[channel].buffer[i];
    }

    return (uint16_t)(sum / count);
}

// 连续采集任务
static void sampling_task(void *arg) {
    uint32_t interval_ms = (uint32_t)(uintptr_t)arg;

    ESP_LOGI(TAG, "Starting continuous sampling with %lu ms interval", interval_ms);

    while (1) {
        current_sensor_data_t data;
        esp_err_t ret = adc128s102_read_all_currents(&data);

        if (ret == ESP_OK) {
            // 更新全局数据
            if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                s_latest_data = data;
                xSemaphoreGive(s_data_mutex);
            }

            // 发送BLE通知
            if (s_ble_notifications_enabled && s_ble_conn_handle != 0) {
                // 构建36字节的监控数据通知
                uint8_t notify_data[36];
                int offset = 0;

                // 1. 输入电压 (uint16, mV)
                uint16_t vin_mv;
                if (power_mgr_get_voltage_mv(&vin_mv, false) == ESP_OK) {
                    notify_data[offset++] = (vin_mv >> 8) & 0xFF;
                    notify_data[offset++] = vin_mv & 0xFF;
                } else {
                    notify_data[offset++] = 0xFF;
                    notify_data[offset++] = 0xFF;
                }

                // 2. 电源区域温度 (int16, 0.01°C)
                int16_t power_temp;
                if (temp_mgr_sample_once(TEMP_SENSOR_POWER, &power_temp) == ESP_OK) {
                    notify_data[offset++] = (power_temp >> 8) & 0xFF;
                    notify_data[offset++] = power_temp & 0xFF;
                } else {
                    notify_data[offset++] = 0x80;  // 无效温度标记
                    notify_data[offset++] = 0x00;
                }

                // 3. 控制区域温度 (int16, 0.01°C)
                int16_t control_temp;
                if (temp_mgr_sample_once(TEMP_SENSOR_CONTROL, &control_temp) == ESP_OK) {
                    notify_data[offset++] = (control_temp >> 8) & 0xFF;
                    notify_data[offset++] = control_temp & 0xFF;
                } else {
                    notify_data[offset++] = 0x80;  // 无效温度标记
                    notify_data[offset++] = 0x00;
                }

                // 4. 电流数据
                // 4.1 总输入电流 (float, 4字节)
                union { float f; uint32_t u; } total_current_union;
                total_current_union.f = data.total_input_current;
                notify_data[offset++] = (total_current_union.u >> 24) & 0xFF;
                notify_data[offset++] = (total_current_union.u >> 16) & 0xFF;
                notify_data[offset++] = (total_current_union.u >> 8) & 0xFF;
                notify_data[offset++] = total_current_union.u & 0xFF;

                // 4.2 各通道电流 (6 x float = 24字节)
                for (int i = 0; i < 6; i++) {
                    union { float f; uint32_t u; } channel_current_union;
                    channel_current_union.f = data.channel_currents[i];
                    notify_data[offset++] = (channel_current_union.u >> 24) & 0xFF;
                    notify_data[offset++] = (channel_current_union.u >> 16) & 0xFF;
                    notify_data[offset++] = (channel_current_union.u >> 8) & 0xFF;
                    notify_data[offset++] = channel_current_union.u & 0xFF;
                }

                // 5. 系统状态标志
                uint8_t status_flags = 0;

                // 热保护状态
                if (power_mgr_is_thermal_protection_active()) {
                    status_flags |= 0x01;  // bit0: 热保护中
                }

                // 温度数据有效性
                int16_t power_temp_check, control_temp_check;
                if (temp_mgr_sample_once(TEMP_SENSOR_POWER, &power_temp_check) == ESP_OK &&
                    temp_mgr_sample_once(TEMP_SENSOR_CONTROL, &control_temp_check) == ESP_OK) {
                    status_flags |= 0x02;  // bit1: 温度数据有效
                }

                // 电流数据有效性 (在当前上下文中已经是有效的)
                status_flags |= 0x04;  // bit2: 电流数据有效

                // 校准状态
                bool calibration_valid = true;  // TODO: 实现真正的校准状态检查
                if (calibration_valid) {
                    status_flags |= 0x08;  // bit3: 校准状态
                }

                // 外设电源状态
                if (power_mgr_external_power_is_on()) {
                    status_flags |= 0x10;  // bit4: 外设电源开启
                }

                notify_data[offset++] = status_flags;

                // 6. 保留字节
                notify_data[offset++] = 0x00;

                // 发送通知
                struct os_mbuf *om = ble_hs_mbuf_att_pkt();
                if (om) {
                    int append_rc = os_mbuf_append(om, notify_data, sizeof(notify_data));
                    if (append_rc == 0) {
                        int notify_rc = ble_gatts_notify_custom(s_ble_conn_handle, s_ble_attr_handle, om);
                        if (notify_rc == 0) {
                            ESP_LOGD(TAG, "Sent monitoring notification (36 bytes)");
                        } else {
                            ESP_LOGW(TAG, "Failed to send monitoring notification: %d", notify_rc);
                            os_mbuf_free_chain(om);
                        }
                    } else {
                        ESP_LOGW(TAG, "Failed to append monitoring data: %d", append_rc);
                        os_mbuf_free_chain(om);
                    }
                } else {
                    ESP_LOGW(TAG, "Failed to allocate mbuf for monitoring notification");
                }
            }
        } else {
            ESP_LOGW(TAG, "Failed to read current data: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }
}

// 启动连续采集
esp_err_t adc128s102_start_continuous_sampling(uint32_t interval_ms) {
    if (s_sampling_task_handle) {
        ESP_LOGW(TAG, "Sampling task already running");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ret = xTaskCreate(sampling_task, "adc_sampling", 4096, (void*)(uintptr_t)interval_ms, 5, &s_sampling_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sampling task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Started continuous sampling with %lu ms interval", interval_ms);
    return ESP_OK;
}

// 停止连续采集
esp_err_t adc128s102_stop_continuous_sampling(void) {
    if (!s_sampling_task_handle) {
        ESP_LOGW(TAG, "No sampling task running");
        return ESP_ERR_INVALID_STATE;
    }

    vTaskDelete(s_sampling_task_handle);
    s_sampling_task_handle = NULL;

    ESP_LOGI(TAG, "Stopped continuous sampling");
    return ESP_OK;
}

// 获取最新数据
esp_err_t adc128s102_get_latest_data(current_sensor_data_t *data) {
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *data = s_latest_data;
        xSemaphoreGive(s_data_mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

// 设置校准参数
esp_err_t adc128s102_set_calibration(const adc_calibration_t *cal) {
    if (!cal) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_calibration = *cal;
        xSemaphoreGive(s_data_mutex);

        ESP_LOGI(TAG, "Calibration updated: ACS758 offset=%.3fV, sens=%.3fV/A, ACS712 offset=%.3fV, sens=%.3fV/A",
                 s_calibration.acs758_offset, s_calibration.acs758_sensitivity,
                 s_calibration.acs712_offset, s_calibration.acs712_sensitivity);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

// 获取校准参数
esp_err_t adc128s102_get_calibration(adc_calibration_t *cal) {
    if (!cal) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *cal = s_calibration;
        xSemaphoreGive(s_data_mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

// 零点校准
esp_err_t adc128s102_calibrate_zero_point(void) {
    ESP_LOGI(TAG, "Starting zero point calibration...");

    const uint8_t sample_count = 50;
    float total_voltage_sum = 0.0f;
    float channel_voltage_sums[6] = {0.0f};

    // 采集多次数据求平均值
    for (uint8_t i = 0; i < sample_count; i++) {
        uint16_t raw_value;

        // 读取总电流通道
        if (adc128s102_read_raw(ADC_CHANNEL_TOTAL_CURRENT, &raw_value) == ESP_OK) {
            float voltage = (float)raw_value * ADC_VREF / ADC_RESOLUTION;
            total_voltage_sum += voltage;
        }

        // 读取各通道电流
        for (uint8_t ch = 0; ch < 6; ch++) {
            if (adc128s102_read_raw(ADC_CHANNEL_CH1_CURRENT + ch, &raw_value) == ESP_OK) {
                float voltage = (float)raw_value * ADC_VREF / ADC_RESOLUTION;
                channel_voltage_sums[ch] += voltage;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // 20ms间隔
    }

    // 更新校准参数
    adc_calibration_t new_cal = s_calibration;
    new_cal.acs758_offset = total_voltage_sum / sample_count;

    for (uint8_t ch = 0; ch < 6; ch++) {
        new_cal.acs712_offset = channel_voltage_sums[ch] / sample_count;
    }

    esp_err_t ret = adc128s102_set_calibration(&new_cal);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Zero point calibration completed: ACS758 offset=%.3fV, ACS712 offset=%.3fV",
                 new_cal.acs758_offset, new_cal.acs712_offset);
    }

    return ret;
}

// BLE接口实现
int adc128s102_ble_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // 监控特征返回36字节的完整监控数据
        // 格式: 电压(2) + 电源温度(2) + 控制温度(2) + 总电流(4) + 6通道电流(24) + 状态(1) + 保留(1) = 36字节
        uint8_t response[36];
        int offset = 0;

        // 1. 输入电压 (uint16, mV)
        uint16_t vin_mv;
        if (power_mgr_get_voltage_mv(&vin_mv, false) == ESP_OK) {
            response[offset++] = (vin_mv >> 8) & 0xFF;
            response[offset++] = vin_mv & 0xFF;
        } else {
            response[offset++] = 0xFF;
            response[offset++] = 0xFF;
        }

        // 2. 电源区域温度 (int16, 0.01°C)
        int16_t power_temp;
        if (temp_mgr_sample_once(TEMP_SENSOR_POWER, &power_temp) == ESP_OK) {
            response[offset++] = (power_temp >> 8) & 0xFF;
            response[offset++] = power_temp & 0xFF;
        } else {
            response[offset++] = 0x80;  // 无效温度标记
            response[offset++] = 0x00;
        }

        // 3. 控制区域温度 (int16, 0.01°C)
        int16_t control_temp;
        if (temp_mgr_sample_once(TEMP_SENSOR_CONTROL, &control_temp) == ESP_OK) {
            response[offset++] = (control_temp >> 8) & 0xFF;
            response[offset++] = control_temp & 0xFF;
        } else {
            response[offset++] = 0x80;  // 无效温度标记
            response[offset++] = 0x00;
        }

        // 4. 电流数据
        current_sensor_data_t current_data;
        if (adc128s102_get_latest_data(&current_data) == ESP_OK) {
            // 4.1 总输入电流 (float, 4字节, IEEE 754, A)
            union { float f; uint32_t u; } total_current_union;
            total_current_union.f = current_data.total_input_current;
            response[offset++] = (total_current_union.u >> 24) & 0xFF;
            response[offset++] = (total_current_union.u >> 16) & 0xFF;
            response[offset++] = (total_current_union.u >> 8) & 0xFF;
            response[offset++] = total_current_union.u & 0xFF;

            // 4.2 各通道电流 (6 x float = 24字节)
            for (int i = 0; i < 6; i++) {
                union { float f; uint32_t u; } channel_current_union;
                channel_current_union.f = current_data.channel_currents[i];
                response[offset++] = (channel_current_union.u >> 24) & 0xFF;
                response[offset++] = (channel_current_union.u >> 16) & 0xFF;
                response[offset++] = (channel_current_union.u >> 8) & 0xFF;
                response[offset++] = channel_current_union.u & 0xFF;
            }
        } else {
            // 填充无效电流数据 (28字节)
            for (int i = 0; i < 28; i++) {
                response[offset++] = 0xFF;
            }
        }

        // 5. 系统状态标志
        uint8_t status_flags = 0;

        // 热保护状态
        if (power_mgr_is_thermal_protection_active()) {
            status_flags |= 0x01;  // bit0: 热保护中
        }

        // 温度数据有效性
        int16_t power_temp_check, control_temp_check;
        bool temp_valid = false;
        if (temp_mgr_sample_once(TEMP_SENSOR_POWER, &power_temp_check) == ESP_OK &&
            temp_mgr_sample_once(TEMP_SENSOR_CONTROL, &control_temp_check) == ESP_OK) {
            status_flags |= 0x02;  // bit1: 温度数据有效
            temp_valid = true;
        }

        // 电流数据有效性
        if (adc128s102_get_latest_data(&current_data) == ESP_OK) {
            status_flags |= 0x04;  // bit2: 电流数据有效
        }

        // 校准状态 (需要检查ADC是否已校准)
        bool calibration_valid = true;  // TODO: 实现真正的校准状态检查
        if (calibration_valid) {
            status_flags |= 0x08;  // bit3: 校准状态
        }

        // 外设电源状态
        if (power_mgr_external_power_is_on()) {
            status_flags |= 0x10;  // bit4: 外设电源开启
        }

        response[offset++] = status_flags;

        // 6. 保留字节
        response[offset++] = 0x00;

        // 确保我们返回了36字节
        if (offset != 36) {
            ESP_LOGE(TAG, "Monitoring data size mismatch: expected 36, got %d", offset);
            return BLE_ATT_ERR_UNLIKELY;
        }

        int rc = os_mbuf_append(ctxt->om, response, sizeof(response));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

void adc128s102_ble_subscribe(uint16_t conn_handle, uint16_t attr_handle, bool enabled) {
    s_ble_conn_handle = conn_handle;
    s_ble_attr_handle = attr_handle;
    s_ble_notifications_enabled = enabled;

    ESP_LOGI(TAG, "BLE subscription %s for connection %u", enabled ? "enabled" : "disabled", conn_handle);
}