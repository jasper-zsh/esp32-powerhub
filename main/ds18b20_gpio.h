#ifndef DS18B20_GPIO_H
#define DS18B20_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "onewire_gpio.h"

// DS18B20配置
typedef struct {
    uint8_t resolution;      // 分辨率：9-12位
    bool parasite_power;     // 是否使用寄生电源
} ds18b20_gpio_config_t;

// DS18B20设备句柄
typedef struct ds18b20_gpio_device* ds18b20_gpio_device_handle_t;

// DS18B20命令
#define DS18B20_CMD_CONVERT_T        0x44  // 启动温度转换
#define DS18B20_CMD_WRITE_SCRATCHPAD 0x4E  // 写暂存器
#define DS18B20_CMD_READ_SCRATCHPAD  0xBE  // 读暂存器
#define DS18B20_CMD_COPY_SCRATCHPAD  0x48  // 复制暂存器到EEPROM
#define DS18B20_CMD_RECALL_E2        0xB8  // 从EEPROM恢复数据
#define DS18B20_CMD_READ_POWER       0xB4  // 读电源状态

// DS18B20分辨率设置
#define DS18B20_RESOLUTION_9BIT     0x1F  // 9位：0.5°C
#define DS18B20_RESOLUTION_10BIT    0x3F  // 10位：0.25°C
#define DS18B20_RESOLUTION_11BIT    0x5F  // 11位：0.125°C
#define DS18B20_RESOLUTION_12BIT    0x7F  // 12位：0.0625°C

// DS18B20暂存器大小
#define DS18B20_SCRATCHPAD_SIZE     9

// 从1-Wire设备创建DS18B20设备
esp_err_t ds18b20_gpio_new_device(onewire_gpio_bus_handle_t bus,
                                 const onewire_gpio_device_t *onewire_device,
                                 const ds18b20_gpio_config_t *config,
                                 ds18b20_gpio_device_handle_t *ret_device);

// 删除DS18B20设备
esp_err_t ds18b20_gpio_del_device(ds18b20_gpio_device_handle_t device);

// 获取DS18B20设备地址
esp_err_t ds18b20_gpio_get_device_address(ds18b20_gpio_device_handle_t device,
                                         uint64_t *address);

// 启动温度转换
esp_err_t ds18b20_gpio_trigger_temperature_conversion(ds18b20_gpio_device_handle_t device);

// 为总线上所有设备启动温度转换
esp_err_t ds18b20_gpio_trigger_temperature_conversion_for_all(onewire_gpio_bus_handle_t bus);

// 读取温度（返回摄氏度）
esp_err_t ds18b20_gpio_get_temperature(ds18b20_gpio_device_handle_t device, float *temperature);

// 读取暂存器
esp_err_t ds18b20_gpio_read_scratchpad(ds18b20_gpio_device_handle_t device,
                                      uint8_t *scratchpad, size_t length);

// 写暂存器
esp_err_t ds18b20_gpio_write_scratchpad(ds18b20_gpio_device_handle_t device,
                                       const uint8_t *scratchpad, size_t length);

// 设置分辨率
esp_err_t ds18b20_gpio_set_resolution(ds18b20_gpio_device_handle_t device, uint8_t resolution);

// 检查温度转换是否完成
esp_err_t ds18b20_gpio_is_conversion_complete(ds18b20_gpio_device_handle_t device, bool *complete);

#endif // DS18B20_GPIO_H