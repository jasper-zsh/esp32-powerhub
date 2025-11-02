#include "ds18b20_gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ds18b20_gpio";

// DS18B20设备结构
struct ds18b20_gpio_device {
    onewire_gpio_bus_handle_t bus;
    uint64_t address;
    ds18b20_gpio_config_t config;
};

// DS18B20族码
#define DS18B20_FAMILY_CODE 0x28

// 匹配ROM命令
static esp_err_t ds18b20_gpio_match_rom(ds18b20_gpio_device_handle_t device) {
    esp_err_t ret = onewire_gpio_write_byte(device->bus, 0x55); // MATCH ROM命令
    if (ret != ESP_OK) {
        return ret;
    }

    // 写入64位ROM地址
    for (int i = 0; i < 8; i++) {
        uint8_t addr_byte = (device->address >> (i * 8)) & 0xFF;
        ret = onewire_gpio_write_byte(device->bus, addr_byte);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}


// 从1-Wire设备创建DS18B20设备
esp_err_t ds18b20_gpio_new_device(onewire_gpio_bus_handle_t bus,
                                 const onewire_gpio_device_t *onewire_device,
                                 const ds18b20_gpio_config_t *config,
                                 ds18b20_gpio_device_handle_t *ret_device) {
    if (bus == NULL || onewire_device == NULL || ret_device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 检查族码是否为DS18B20
    uint8_t family_code = onewire_device->address & 0xFF;
    if (family_code != DS18B20_FAMILY_CODE) {
        ESP_LOGE(TAG, "Device family code 0x%02X is not DS18B20 (expected 0x%02X)",
                 family_code, DS18B20_FAMILY_CODE);
        return ESP_ERR_INVALID_ARG;
    }

    ds18b20_gpio_device_handle_t device = calloc(1, sizeof(struct ds18b20_gpio_device));
    if (device == NULL) {
        return ESP_ERR_NO_MEM;
    }

    device->bus = bus;
    device->address = onewire_device->address;

    // 使用默认配置或提供的配置
    if (config != NULL) {
        device->config = *config;
    } else {
        device->config.resolution = DS18B20_RESOLUTION_12BIT;
        device->config.parasite_power = false;
    }

    ESP_LOGI(TAG, "DS18B20 device created with address %016llX", device->address);
    *ret_device = device;
    return ESP_OK;
}

// 删除DS18B20设备
esp_err_t ds18b20_gpio_del_device(ds18b20_gpio_device_handle_t device) {
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    free(device);
    ESP_LOGI(TAG, "DS18B20 device deleted");
    return ESP_OK;
}

// 获取DS18B20设备地址
esp_err_t ds18b20_gpio_get_device_address(ds18b20_gpio_device_handle_t device, uint64_t *address) {
    if (device == NULL || address == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *address = device->address;
    return ESP_OK;
}

// 启动温度转换
esp_err_t ds18b20_gpio_trigger_temperature_conversion(ds18b20_gpio_device_handle_t device) {
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 复位总线
    esp_err_t ret = onewire_gpio_reset(device->bus);
    if (ret != ESP_OK) {
        return ret;
    }

    // 匹配ROM
    ret = ds18b20_gpio_match_rom(device);
    if (ret != ESP_OK) {
        return ret;
    }

    // 发送温度转换命令
    ret = onewire_gpio_write_byte(device->bus, DS18B20_CMD_CONVERT_T);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGV(TAG, "Temperature conversion triggered for device %016llX", device->address);
    return ESP_OK;
}

// 为总线上所有设备启动温度转换
esp_err_t ds18b20_gpio_trigger_temperature_conversion_for_all(onewire_gpio_bus_handle_t bus) {
    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 复位总线
    esp_err_t ret = onewire_gpio_reset(bus);
    if (ret != ESP_OK) {
        return ret;
    }

    // 跳过ROM
    ret = onewire_gpio_write_byte(bus, 0xCC);
    if (ret != ESP_OK) {
        return ret;
    }

    // 发送温度转换命令
    ret = onewire_gpio_write_byte(bus, DS18B20_CMD_CONVERT_T);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGV(TAG, "Temperature conversion triggered for all devices on bus");
    return ESP_OK;
}

// 读取温度
esp_err_t ds18b20_gpio_get_temperature(ds18b20_gpio_device_handle_t device, float *temperature) {
    if (device == NULL || temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 复位总线
    esp_err_t ret = onewire_gpio_reset(device->bus);
    if (ret != ESP_OK) {
        return ret;
    }

    // 匹配ROM
    ret = ds18b20_gpio_match_rom(device);
    if (ret != ESP_OK) {
        return ret;
    }

    // 发送读暂存器命令
    ret = onewire_gpio_write_byte(device->bus, DS18B20_CMD_READ_SCRATCHPAD);
    if (ret != ESP_OK) {
        return ret;
    }

    // 读取暂存器
    uint8_t scratchpad[DS18B20_SCRATCHPAD_SIZE];
    for (int i = 0; i < DS18B20_SCRATCHPAD_SIZE; i++) {
        ret = onewire_gpio_read_byte(device->bus, &scratchpad[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    // 验证CRC
    uint8_t crc = onewire_gpio_crc8(scratchpad, DS18B20_SCRATCHPAD_SIZE - 1);
    if (crc != scratchpad[8]) {
        ESP_LOGW(TAG, "Scratchpad CRC error for device %016llX:", device->address);
        ESP_LOGW(TAG, "  Raw scratchpad data: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                 scratchpad[0], scratchpad[1], scratchpad[2], scratchpad[3],
                 scratchpad[4], scratchpad[5], scratchpad[6], scratchpad[7],
                 scratchpad[8]);
        ESP_LOGW(TAG, "  Calculated CRC: 0x%02X, Received CRC: 0x%02X",
                 crc, scratchpad[8]);
        ESP_LOGW(TAG, "  Temperature raw: 0x%02X%02X", scratchpad[1], scratchpad[0]);
        // return ESP_ERR_INVALID_CRC;
    }

    // 提取温度值（前两个字节）
    int16_t temp_raw = (scratchpad[1] << 8) | scratchpad[0];

    // 转换为摄氏度
    *temperature = (float)temp_raw / 16.0f;

    ESP_LOGV(TAG, "Temperature read: %.4f°C for device %016llX", *temperature, device->address);
    return ESP_OK;
}

// 读取暂存器
esp_err_t ds18b20_gpio_read_scratchpad(ds18b20_gpio_device_handle_t device,
                                      uint8_t *scratchpad, size_t length) {
    if (device == NULL || scratchpad == NULL || length < DS18B20_SCRATCHPAD_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    // 复位总线
    esp_err_t ret = onewire_gpio_reset(device->bus);
    if (ret != ESP_OK) {
        return ret;
    }

    // 匹配ROM
    ret = ds18b20_gpio_match_rom(device);
    if (ret != ESP_OK) {
        return ret;
    }

    // 发送读暂存器命令
    ret = onewire_gpio_write_byte(device->bus, DS18B20_CMD_READ_SCRATCHPAD);
    if (ret != ESP_OK) {
        return ret;
    }

    // 读取暂存器
    for (int i = 0; i < DS18B20_SCRATCHPAD_SIZE; i++) {
        ret = onewire_gpio_read_byte(device->bus, &scratchpad[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

// 写暂存器
esp_err_t ds18b20_gpio_write_scratchpad(ds18b20_gpio_device_handle_t device,
                                       const uint8_t *scratchpad, size_t length) {
    if (device == NULL || scratchpad == NULL || length < 3) {
        return ESP_ERR_INVALID_ARG;
    }

    // 复位总线
    esp_err_t ret = onewire_gpio_reset(device->bus);
    if (ret != ESP_OK) {
        return ret;
    }

    // 匹配ROM
    ret = ds18b20_gpio_match_rom(device);
    if (ret != ESP_OK) {
        return ret;
    }

    // 发送写暂存器命令
    ret = onewire_gpio_write_byte(device->bus, DS18B20_CMD_WRITE_SCRATCHPAD);
    if (ret != ESP_OK) {
        return ret;
    }

    // 写入前3个字节（TH、TL、配置寄存器）
    for (int i = 0; i < 3; i++) {
        ret = onewire_gpio_write_byte(device->bus, scratchpad[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

// 设置分辨率
esp_err_t ds18b20_gpio_set_resolution(ds18b20_gpio_device_handle_t device, uint8_t resolution) {
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 验证分辨率值
    if (resolution != DS18B20_RESOLUTION_9BIT &&
        resolution != DS18B20_RESOLUTION_10BIT &&
        resolution != DS18B20_RESOLUTION_11BIT &&
        resolution != DS18B20_RESOLUTION_12BIT) {
        return ESP_ERR_INVALID_ARG;
    }

    // 读取当前暂存器
    uint8_t scratchpad[DS18B20_SCRATCHPAD_SIZE];
    esp_err_t ret = ds18b20_gpio_read_scratchpad(device, scratchpad, sizeof(scratchpad));
    if (ret != ESP_OK) {
        return ret;
    }

    // 更新配置寄存器（字节4）
    scratchpad[4] = (scratchpad[4] & 0x60) | resolution;

    // 写回暂存器
    ret = ds18b20_gpio_write_scratchpad(device, scratchpad, 3);
    if (ret != ESP_OK) {
        return ret;
    }

    // 保存到EEPROM
    ret = onewire_gpio_reset(device->bus);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = ds18b20_gpio_match_rom(device);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = onewire_gpio_write_byte(device->bus, DS18B20_CMD_COPY_SCRATCHPAD);
    if (ret != ESP_OK) {
        return ret;
    }

    // 等待EEPROM写入完成（最多10ms）
    vTaskDelay(pdMS_TO_TICKS(10));

    device->config.resolution = resolution;
    ESP_LOGI(TAG, "Resolution set to %d bits for device %016llX",
             9 + ((resolution >> 5) & 0x03), device->address);

    return ESP_OK;
}

// 检查温度转换是否完成
esp_err_t ds18b20_gpio_is_conversion_complete(ds18b20_gpio_device_handle_t device, bool *complete) {
    if (device == NULL || complete == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // DS18B20转换期间，读时隙会返回0，转换完成后返回1
    // 简单的实现：发送读时隙然后检查返回值
    uint8_t bit;
    esp_err_t ret = onewire_gpio_read_bit(device->bus, &bit);
    if (ret != ESP_OK) {
        return ret;
    }

    // 如果读到0，转换正在进行；如果读到1，转换完成
    *complete = (bit == 1);

    ESP_LOGV(TAG, "Conversion status: %s", *complete ? "complete" : "in progress");
    return ESP_OK;
}