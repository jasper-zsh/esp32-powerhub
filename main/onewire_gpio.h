#ifndef ONEWIRE_GPIO_H
#define ONEWIRE_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

// 1-Wire GPIO驱动配置
typedef struct {
    gpio_num_t gpio_num;        // GPIO引脚号
    bool enable_pullup;         // 是否启用内部上拉（一般需要外部上拉）
} onewire_gpio_config_t;

// 1-Wire总线句柄
typedef struct onewire_gpio_bus* onewire_gpio_bus_handle_t;

// 1-Wire设备结构
typedef struct {
    uint64_t address;           // 64位ROM地址
} onewire_gpio_device_t;

// 1-Wire设备迭代器
typedef struct onewire_gpio_iter* onewire_gpio_device_iter_handle_t;

// 创建1-Wire总线
esp_err_t onewire_gpio_new_bus(const onewire_gpio_config_t *config, onewire_gpio_bus_handle_t *ret_bus);

// 删除1-Wire总线
esp_err_t onewire_gpio_del_bus(onewire_gpio_bus_handle_t bus);

// 创建设备迭代器
esp_err_t onewire_gpio_new_device_iter(onewire_gpio_bus_handle_t bus, onewire_gpio_device_iter_handle_t *ret_iter);

// 获取下一个设备
esp_err_t onewire_gpio_device_iter_get_next(onewire_gpio_device_iter_handle_t iter, onewire_gpio_device_t *device);

// 删除设备迭代器
esp_err_t onewire_gpio_del_device_iter(onewire_gpio_device_iter_handle_t iter);

// 1-Wire基本操作
esp_err_t onewire_gpio_reset(onewire_gpio_bus_handle_t bus);
esp_err_t onewire_gpio_read_bit(onewire_gpio_bus_handle_t bus, uint8_t *bit);
esp_err_t onewire_gpio_write_bit(onewire_gpio_bus_handle_t bus, uint8_t bit);
esp_err_t onewire_gpio_read_byte(onewire_gpio_bus_handle_t bus, uint8_t *byte);
esp_err_t onewire_gpio_write_byte(onewire_gpio_bus_handle_t bus, uint8_t byte);
esp_err_t onewire_gpio_read_bytes(onewire_gpio_bus_handle_t bus, uint8_t *data, size_t len);
esp_err_t onewire_gpio_write_bytes(onewire_gpio_bus_handle_t bus, const uint8_t *data, size_t len);

// CRC8校验
uint8_t onewire_gpio_crc8(const uint8_t *data, size_t length);

#endif // ONEWIRE_GPIO_H