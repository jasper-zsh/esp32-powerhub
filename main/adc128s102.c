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
#include "hardware_defs.h"

struct ble_gatt_access_ctxt;

static const char *TAG = "adc128s102";

// SPI配置 - 使用hardware_defs.h中的定义
#define ADC_CS_PIN               EXTERNAL_ADC_CS_PIN
#define ADC_SCLK_PIN             EXTERNAL_ADC_SCLK_PIN
#define ADC_MISO_PIN             EXTERNAL_ADC_MISO_PIN
#define ADC_MOSI_PIN             EXTERNAL_ADC_MOSI_PIN

// SPI主机配置
#define ADC_SPI_HOST             SPI2_HOST
#define ADC_SPI_CLOCK_SPEED      500000   // 500KHz (符合数据手册要求，低功耗且稳定)

// ADC参数
#define ADC_RESOLUTION           4096.0f  // 12位ADC
#define ADC_VREF                 5.0f     // 5V参考电压

// ACS758参数 - 使用hardware_defs.h中的定义
// ACS712参数 - 使用hardware_defs.h中的定义

// 全局变量
static spi_device_handle_t s_spi_handle = NULL;
static TaskHandle_t s_sampling_task_handle = NULL;
static current_sensor_data_t s_latest_data = {0};
static SemaphoreHandle_t s_data_mutex = NULL;
static adc_calibration_t s_calibration = {
    .acs758_offset = TOTAL_CURRENT_OFFSET,
    .acs758_sensitivity = TOTAL_CURRENT_SENSITIVITY,
    .acs712_offset = CHANNEL_CURRENT_OFFSET,
    .acs712_sensitivity = CHANNEL_CURRENT_SENSITIVITY
};


// BLE通知相关
static uint16_t s_ble_conn_handle = 0;
static uint16_t s_ble_attr_handle = 0;
static bool s_ble_notifications_enabled = false;

// 内部函数声明
static esp_err_t adc128s102_spi_init(void);
static esp_err_t adc128s102_read_channel_raw(uint8_t channel, uint16_t *raw_value);
static esp_err_t adc128s102_read_all_channels_continuous(uint16_t *channel_values);
static float raw_to_current(uint8_t channel, uint16_t raw_value);
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
        .mode = 0,  // MODE 0 (CPOL=0, CPHA=0) - 符合数据手册时序要求
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


// 连续读取所有通道 - 使用单个SPI事务读取多个frame
static esp_err_t adc128s102_read_all_channels_continuous(uint16_t *channel_values) {
    if (!s_spi_handle || !channel_values) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "ADC CONTINUOUS: Starting continuous read of all 8 channels");

    // ADC128S102连续读取协议：
    // 每个frame发送2字节，接收2字节，总共8个frame = 16字节
    // 1. Frame 0: 发送[00,00]，接收上一次转换结果
    // 2. Frame 1: 发送[08,00]，接收通道0转换结果
    // 3. Frame 2: 发送[10,00]，接收通道1转换结果
    // 4. 以此类推...

    // 准备16字节的TX和RX数据（8个frame，每个frame 2字节）
    uint8_t tx_data[16] = {0};
    uint8_t rx_data[16] = {0};

    // 构造TX命令：每个frame 2字节，通道命令在第一个字节
    for (int i = 0; i < 8; i++) {
        tx_data[i * 2] = i << 3;     // 通道命令字节：ADD2-ADD0 << 3
        tx_data[i * 2 + 1] = 0x00;   // 第二个字节为0
    }

    ESP_LOGI(TAG, "ADC CONTINUOUS: TX commands (16 bytes):");
    for (int i = 0; i < 8; i++) {
        ESP_LOGI(TAG, "  Frame %d: [0x%02X, 0x%02X] (Channel %d)",
                 i, tx_data[i * 2], tx_data[i * 2 + 1], i);
    }

    // 配置SPI事务：16字节传输（8个frame，每个frame 2字节TX + 2字节RX）
    spi_transaction_t trans = {
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
        .length = 128,     // 128位 = 16字节
        .rxlength = 128    // 接收16字节
    };

    esp_err_t ret = spi_device_transmit(s_spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC CONTINUOUS: SPI transaction failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ADC CONTINUOUS: RX data received (16 bytes):");

    // 解析接收到的数据
    for (int i = 0; i < 8; i++) {
        // 每个frame占2字节
        uint8_t rx_high = rx_data[i * 2];
        uint8_t rx_low = rx_data[i * 2 + 1];
        uint16_t received_data = ((uint16_t)rx_high << 8) | rx_low;

        // 提取12位ADC值（低12位有效）
        channel_values[i] = received_data & 0x0FFF;

        ESP_LOGI(TAG, "ADC CONTINUOUS: Frame %d - RX[0]=0x%02X, RX[1]=0x%02X, combined=0x%04X, raw=%d",
                 i, rx_high, rx_low, received_data, channel_values[i]);
    }

    // 注意：由于流水线特性，接收到的数据是错位的：
    // frame 0 接收的是上一次的转换结果
    // frame 1 接收的是通道0的结果
    // frame 2 接收的是通道1的结果
    // ...
    // frame 7 接收的是通道6的结果

    // 需要重新排列数据
    uint16_t temp_values[ADC128S102_CHANNEL_COUNT];
    temp_values[0] = channel_values[1];  // 通道0的结果在frame 1
    temp_values[1] = channel_values[2];  // 通道1的结果在frame 2
    temp_values[2] = channel_values[3];  // 通道2的结果在frame 3
    temp_values[3] = channel_values[4];  // 通道3的结果在frame 4
    temp_values[4] = channel_values[5];  // 通道4的结果在frame 5
    temp_values[5] = channel_values[6];  // 通道5的结果在frame 6
    temp_values[6] = channel_values[7];  // 通道6的结果在frame 7
    temp_values[7] = 0;                  // 通道7的结果需要下一次读取

    ESP_LOGI(TAG, "ADC CONTINUOUS: Channel mapping after pipeline correction:");
    for (int i = 0; i < ADC128S102_CHANNEL_COUNT - 1; i++) {
        ESP_LOGI(TAG, "  Channel %d: raw=%d", i, temp_values[i]);
    }

    // 将校正后的数据复制回输出数组
    memcpy(channel_values, temp_values, sizeof(uint16_t) * 8);

    return ESP_OK;
}

// 读取单个通道ADC原始值
esp_err_t adc128s102_read_raw(uint8_t channel, uint16_t *raw_value) {
    if (channel >= ADC128S102_CHANNEL_COUNT || !raw_value) {
        return ESP_ERR_INVALID_ARG;
    }

    return adc128s102_read_channel_raw(channel, raw_value);
}

// 读取单个通道原始值 (内部实现) - 按照数据手册时序要求
static esp_err_t adc128s102_read_channel_raw(uint8_t channel, uint16_t *raw_value) {
    if (!s_spi_handle) {
        return ESP_ERR_INVALID_STATE;
    }

    // ADC128S102通信协议：根据数据手册时序要求
    // 1. CS下降沿启动转换
    // 2. 前8个SCLK上升沿发送控制寄存器数据：DONTC DONTC ADD2 ADD1 ADD0 DONTC DONTC DONTC
    // 3. DOUT在第5-16个SCLK下降沿输出当前转换结果
    // 4. 控制寄存器设置下一次转换的通道

    uint8_t tx_data[2] = {0};
    uint8_t rx_data[2] = {0};

    // 构造控制寄存器数据（设置下一次转换的通道）
    // 格式：Bit7-6: DONTC, Bit5-3: ADD2-ADD0, Bit2-0: DONTC
    tx_data[0] = channel << 3;

    ESP_LOGI(TAG, "ADC RAW: Reading channel %d, TX cmd=0x%02X", channel, tx_data[0]);

    // 执行16位SPI传输
    spi_transaction_t trans = {
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
        .length = 16,      // 16位时钟周期
        .rxlength = 16     // 同时接收16位数据
    };

    esp_err_t ret = spi_device_transmit(s_spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // DOUT在第5-16个SCLK下降沿输出数据，前4个下降沿是leading zeros
    // 接收到的16位数据格式：0000 + 12位ADC数据（MSB first）
    uint16_t received_data = ((rx_data[0] & 0xFF) << 8) | rx_data[1];
    *raw_value = received_data & 0x0FFF;  // 提取低12位ADC值

    ESP_LOGI(TAG, "ADC RAW: Channel %d -> RX[0]=0x%02X, RX[1]=0x%02X, combined=0x%04X, raw=%d",
             channel, rx_data[0], rx_data[1], received_data, *raw_value);

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

    ESP_LOGI(TAG, "ADC CURRENT: Channel %d (enum=%d) -> raw=%d, voltage=%.3fV, current=%.3fA",
             channel, channel, raw_value, (float)raw_value * ADC_VREF / ADC_RESOLUTION, *current);

    return ESP_OK;
}

// 批量读取所有电流数据 - 自适应传感器配置
esp_err_t adc128s102_read_all_currents(current_sensor_data_t *data) {
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "ADC BATCH: Starting adaptive batch read of current sensors");

    // 初始化输出数据
    memset(data, 0, sizeof(current_sensor_data_t));

    // 使用连续读取方法一次性获取所有通道的原始数据
    uint16_t raw_values[ADC128S102_CHANNEL_COUNT];
    esp_err_t ret = adc128s102_read_all_channels_continuous(raw_values);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC BATCH: Failed to read all channels continuously");
        return ret;
    }

    ESP_LOGI(TAG, "ADC BATCH: Converting raw values to current (adaptive mode):");

    // 处理总电流传感器（如果存在）
    if (HAS_TOTAL_CURRENT_SENSOR()) {
        uint16_t raw_val = raw_values[TOTAL_CURRENT_CH];
        float voltage = (float)raw_val * ADC_VREF / ADC_RESOLUTION;

        // ACS758计算 - 注意：值越小电流越大（反向关系）
        data->total_input_current = (s_calibration.acs758_offset - voltage) / s_calibration.acs758_sensitivity;

        ESP_LOGI(TAG, "ADC BATCH: Total current (IN%d, ACS758) raw=%d -> voltage=%.3fV -> current=%.3fA",
                 TOTAL_CURRENT_CH, raw_val, voltage, data->total_input_current);
    } else {
        data->total_input_current = 0.0f;
        ESP_LOGI(TAG, "ADC BATCH: No total current sensor configured");
    }

    // 处理各PWM通道的电流传感器
    int valid_channel_count = 0;
    for (int ch = 0; ch < PWM_CHANNEL_COUNT; ch++) {
        if (HAS_CHANNEL_CURRENT_SENSOR(ch)) {
            uint8_t adc_ch = CHANNEL_CURRENT_ADC_CHS[ch];
            if (adc_ch < ADC128S102_CHANNEL_COUNT) {
                uint16_t raw_val = raw_values[adc_ch];
                float voltage = (float)raw_val * ADC_VREF / ADC_RESOLUTION;

                // ACS712计算 - 正常关系
                data->channel_currents[ch] = (voltage - s_calibration.acs712_offset) / s_calibration.acs712_sensitivity;

                ESP_LOGI(TAG, "ADC BATCH: CH%d (IN%d, ACS712) raw=%d -> voltage=%.3fV -> current=%.3fA",
                         ch + 1, adc_ch, raw_val, voltage, data->channel_currents[ch]);

                valid_channel_count++;
            } else {
                ESP_LOGW(TAG, "ADC BATCH: CH%d configured with invalid ADC channel %d", ch + 1, adc_ch);
                data->channel_currents[ch] = 0.0f;
            }
        } else {
            data->channel_currents[ch] = 0.0f;
            ESP_LOGD(TAG, "ADC BATCH: CH%d has no current sensor configured", ch + 1);
        }
    }

    // 输出最终结果摘要
    ESP_LOGI(TAG, "ADC BATCH: Adaptive processing complete:");
    if (HAS_TOTAL_CURRENT_SENSOR()) {
        ESP_LOGI(TAG, "  Total current: %.3fA", data->total_input_current);
    } else {
        ESP_LOGI(TAG, "  Total current: N/A (no sensor)");
    }

    for (int i = 0; i < PWM_CHANNEL_COUNT; i++) {
        if (HAS_CHANNEL_CURRENT_SENSOR(i)) {
            ESP_LOGI(TAG, "  CH%d current: %.3fA", i + 1, data->channel_currents[i]);
        } else {
            ESP_LOGI(TAG, "  CH%d current: N/A (no sensor)", i + 1);
        }
    }

    data->timestamp = esp_timer_get_time() / 1000;  // 转换为毫秒
    return ESP_OK;
}

// 将ADC原始值转换为电流 - 自适应传感器类型
static float raw_to_current(uint8_t channel, uint16_t raw_value) {
    // 将ADC值转换为电压
    float voltage = (float)raw_value * ADC_VREF / ADC_RESOLUTION;

    float current = 0.0f;
    const char *sensor_type = "UNKNOWN";

    // 检查是否为总电流传感器通道
    if (HAS_TOTAL_CURRENT_SENSOR() && channel == TOTAL_CURRENT_CH) {
        // ACS758计算 - 注意：值越小电流越大（反向关系）
        sensor_type = "ACS758";
        current = (s_calibration.acs758_offset - voltage) / s_calibration.acs758_sensitivity;

        ESP_LOGD(TAG, "ADC CONV: Channel %d (%s, INVERSE) raw=%d -> voltage=%.3fV -> current=%.3fA",
                 channel, sensor_type, raw_value, voltage, current);
    } else {
        // 检查是否为某个通道的电流传感器
        bool is_channel_sensor = false;
        for (int ch = 0; ch < PWM_CHANNEL_COUNT; ch++) {
            if (HAS_CHANNEL_CURRENT_SENSOR(ch) && CHANNEL_CURRENT_ADC_CHS[ch] == channel) {
                // ACS712计算 - 正常关系
                sensor_type = "ACS712";
                current = (voltage - s_calibration.acs712_offset) / s_calibration.acs712_sensitivity;
                is_channel_sensor = true;

                ESP_LOGD(TAG, "ADC CONV: Channel %d (CH%d %s, NORMAL) raw=%d -> voltage=%.3fV -> current=%.3fA",
                         channel, ch + 1, sensor_type, raw_value, voltage, current);
                break;
            }
        }

        if (!is_channel_sensor) {
            ESP_LOGW(TAG, "ADC CONV: Channel %d has no current sensor configured, returning 0A", channel);
        }
    }

    return current;
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

                // 4.2 各通道电流 (PWM_CHANNEL_COUNT x float = 24字节)
                for (int i = 0; i < PWM_CHANNEL_COUNT; i++) {
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

// 零点校准 - 自适应传感器配置
esp_err_t adc128s102_calibrate_zero_point(void) {
    ESP_LOGI(TAG, "Starting adaptive zero point calibration...");

    const uint8_t sample_count = 50;
    float total_voltage_sum = 0.0f;
    float *channel_voltage_sums = calloc(PWM_CHANNEL_COUNT, sizeof(float));
    uint32_t total_samples = 0;
    uint32_t *channel_samples = calloc(PWM_CHANNEL_COUNT, sizeof(uint32_t));

    if (!channel_voltage_sums || !channel_samples) {
        ESP_LOGE(TAG, "ADC CAL: Failed to allocate memory for calibration");
        free(channel_voltage_sums);
        free(channel_samples);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "ADC CAL: Collecting %d samples for adaptive zero point calibration", sample_count);

    // 采集多次数据求平均值
    for (uint8_t i = 0; i < sample_count; i++) {
        uint16_t raw_value;

        // 读取总电流传感器（如果存在）
        if (HAS_TOTAL_CURRENT_SENSOR()) {
            if (adc128s102_read_raw(TOTAL_CURRENT_CH, &raw_value) == ESP_OK) {
                float voltage = (float)raw_value * ADC_VREF / ADC_RESOLUTION;
                total_voltage_sum += voltage;
                total_samples++;

                if (i == 0 || i == sample_count - 1) {
                    ESP_LOGI(TAG, "ADC CAL: Sample %d - Total current (IN%d): raw=%d, voltage=%.3fV",
                             i, TOTAL_CURRENT_CH, raw_value, voltage);
                }
            }
        }

        // 读取各通道电流传感器
        for (uint8_t ch = 0; ch < PWM_CHANNEL_COUNT; ch++) {
            if (HAS_CHANNEL_CURRENT_SENSOR(ch)) {
                uint8_t adc_ch = CHANNEL_CURRENT_ADC_CHS[ch];
                if (adc128s102_read_raw(adc_ch, &raw_value) == ESP_OK) {
                    float voltage = (float)raw_value * ADC_VREF / ADC_RESOLUTION;
                    channel_voltage_sums[ch] += voltage;
                    channel_samples[ch]++;

                    if (i == 0 || i == sample_count - 1) {
                        ESP_LOGI(TAG, "ADC CAL: Sample %d - CH%d (IN%d): raw=%d, voltage=%.3fV",
                                 i, ch + 1, adc_ch, raw_value, voltage);
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // 20ms间隔
    }

    ESP_LOGI(TAG, "ADC CAL: Adaptive sample collection complete");

    // 更新校准参数
    adc_calibration_t new_cal = s_calibration;

    // 校准总电流传感器（ACS758）
    if (HAS_TOTAL_CURRENT_SENSOR() && total_samples > 0) {
        new_cal.acs758_offset = total_voltage_sum / total_samples;
        ESP_LOGI(TAG, "ADC CAL: ACS758 offset calibrated from %lu samples: %.3fV",
                 total_samples, new_cal.acs758_offset);
    } else {
        ESP_LOGI(TAG, "ADC CAL: No total current sensor to calibrate");
    }

    // 校准通道电流传感器（ACS712）
    float acs712_total_offset = 0.0f;
    uint32_t acs712_valid_channels = 0;

    for (uint8_t ch = 0; ch < PWM_CHANNEL_COUNT; ch++) {
        if (HAS_CHANNEL_CURRENT_SENSOR(ch) && channel_samples[ch] > 0) {
            float ch_offset = channel_voltage_sums[ch] / channel_samples[ch];
            ESP_LOGI(TAG, "ADC CAL: CH%d (IN%d) offset calibrated from %lu samples: %.3fV",
                     ch + 1, CHANNEL_CURRENT_ADC_CHS[ch], channel_samples[ch], ch_offset);
            acs712_total_offset += ch_offset;
            acs712_valid_channels++;
        }
    }

    if (acs712_valid_channels > 0) {
        new_cal.acs712_offset = acs712_total_offset / acs712_valid_channels;
        ESP_LOGI(TAG, "ADC CAL: Average ACS712 offset from %d channels: %.3fV",
                 acs712_valid_channels, new_cal.acs712_offset);
    } else {
        ESP_LOGI(TAG, "ADC CAL: No channel current sensors to calibrate");
    }

    // 清理内存
    free(channel_voltage_sums);
    free(channel_samples);

    esp_err_t ret = adc128s102_set_calibration(&new_cal);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC CAL: Adaptive zero point calibration completed");
        if (HAS_TOTAL_CURRENT_SENSOR()) {
            ESP_LOGI(TAG, "ADC CAL: ACS758 offset: %.3fV", new_cal.acs758_offset);
        }
        if (acs712_valid_channels > 0) {
            ESP_LOGI(TAG, "ADC CAL: ACS712 offset: %.3fV", new_cal.acs712_offset);
        }
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

            // 4.2 各通道电流 (PWM_CHANNEL_COUNT x float = 24字节)
            for (int i = 0; i < PWM_CHANNEL_COUNT; i++) {
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
        if (temp_mgr_sample_once(TEMP_SENSOR_POWER, &power_temp_check) == ESP_OK &&
            temp_mgr_sample_once(TEMP_SENSOR_CONTROL, &control_temp_check) == ESP_OK) {
            status_flags |= 0x02;  // bit1: 温度数据有效
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