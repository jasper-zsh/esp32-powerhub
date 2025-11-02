#include "onewire_gpio.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_idf_version.h"
#include "esp_rom_sys.h"

// 为了兼容性，添加缺失的宏定义
#ifndef BIT
#define BIT(n) (1ULL << (n))
#endif

static const char *TAG = "onewire_gpio";

// 1-Wire时序定义（单位：微秒）- 参考正常工作的代码
#define ONEWIRE_RESET_PULSE         480
#define ONEWIRE_PRESENCE_WAIT_MIN   70
#define ONEWIRE_PRESENCE_WAIT_MAX   410

#define ONEWIRE_WRITE_1_LOW         5
#define ONEWIRE_WRITE_0_LOW         60
#define ONEWIRE_WRITE_RECOVERY      0

#define ONEWIRE_READ_SLOT_LOW       2
#define ONEWIRE_READ_SAMPLE_TIME    10
#define ONEWIRE_READ_RECOVERY       50

// 1-Wire总线结构
struct onewire_gpio_bus {
    gpio_num_t gpio_num;
    bool enable_pullup;
    bool is_output;  // 跟踪GPIO当前状态
};

// 1-Wire设备迭代器结构
struct onewire_gpio_iter {
    onewire_gpio_bus_handle_t bus;
    uint64_t last_discrepancy;
    uint64_t last_family_discrepancy;
    uint64_t last_device;
    bool done;
};

// 微秒级延时 - 使用ESP ROM提供的精确延时函数
static inline void delay_us(uint32_t us) {
    if (us == 0) return;

    // 使用ESP ROM的延时函数，提供更精确的微秒级延时
    esp_rom_delay_us(us);
}

// 设置GPIO为开漏输出模式（1-Wire协议要求）
static inline void set_gpio_output_od(onewire_gpio_bus_handle_t bus) {
    gpio_set_direction(bus->gpio_num, GPIO_MODE_OUTPUT_OD);
    bus->is_output = true;
}

// 设置GPIO为输入模式
static inline void set_gpio_input(onewire_gpio_bus_handle_t bus) {
    gpio_set_direction(bus->gpio_num, GPIO_MODE_INPUT);
    if (bus->enable_pullup) {
        gpio_set_pull_mode(bus->gpio_num, GPIO_PULLUP_ONLY);
    } else {
        gpio_set_pull_mode(bus->gpio_num, GPIO_FLOATING);
    }
    bus->is_output = false;
}

// 写GPIO电平
static inline void write_gpio(onewire_gpio_bus_handle_t bus, bool level) {
    gpio_set_level(bus->gpio_num, level ? 1 : 0);
}

// 读GPIO电平
static inline bool read_gpio(onewire_gpio_bus_handle_t bus) {
    return gpio_get_level(bus->gpio_num) != 0;
}

// 1-Wire复位时序 - 参考正常工作的代码
esp_err_t onewire_gpio_reset(onewire_gpio_bus_handle_t bus) {
    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t presence = 0;

    // 发送复位脉冲
    set_gpio_output_od(bus);
    write_gpio(bus, 0);
    delay_us(ONEWIRE_RESET_PULSE);

    // 释放总线，等待存在脉冲
    set_gpio_input(bus);
    delay_us(ONEWIRE_PRESENCE_WAIT_MIN);

    // 检查存在脉冲
    if (!read_gpio(bus)) {
        presence = 1;
    }

    delay_us(ONEWIRE_PRESENCE_WAIT_MAX - ONEWIRE_PRESENCE_WAIT_MIN);

    if (presence) {
        ESP_LOGV(TAG, "Presence pulse detected");
        return ESP_OK;
    } else {
        ESP_LOGV(TAG, "No presence pulse detected");
        return ESP_ERR_NOT_FOUND;
    }
}

// 写1位 - 参考正常工作的代码
esp_err_t onewire_gpio_write_bit(onewire_gpio_bus_handle_t bus, uint8_t bit) {
    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    set_gpio_output_od(bus);
    write_gpio(bus, 0);
    delay_us(ONEWIRE_WRITE_1_LOW);

    if (bit) {
        write_gpio(bus, 1);
    }
    delay_us(ONEWIRE_WRITE_0_LOW);
    write_gpio(bus, 1);

    return ESP_OK;
}

// 读1位 - 参考正常工作的代码
esp_err_t onewire_gpio_read_bit(onewire_gpio_bus_handle_t bus, uint8_t *bit) {
    if (bus == NULL || bit == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    set_gpio_output_od(bus);
    write_gpio(bus, 0);
    delay_us(ONEWIRE_READ_SLOT_LOW);

    set_gpio_input(bus);
    delay_us(ONEWIRE_READ_SAMPLE_TIME);

    if (read_gpio(bus)) {
        *bit = 1;
    } else {
        *bit = 0;
    }

    delay_us(ONEWIRE_READ_RECOVERY);
    return ESP_OK;
}

// 写1字节
esp_err_t onewire_gpio_write_byte(onewire_gpio_bus_handle_t bus, uint8_t byte) {
    esp_err_t ret;

    for (int i = 0; i < 8; i++) {
        ret = onewire_gpio_write_bit(bus, byte & 0x01);
        if (ret != ESP_OK) {
            return ret;
        }
        byte >>= 1;
    }

    return ESP_OK;
}

// 读1字节
esp_err_t onewire_gpio_read_byte(onewire_gpio_bus_handle_t bus, uint8_t *byte) {
    if (byte == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t value = 0;

    for (int i = 0; i < 8; i++) {
        uint8_t bit;
        ret = onewire_gpio_read_bit(bus, &bit);
        if (ret != ESP_OK) {
            return ret;
        }
        value |= (bit << i);
    }

    *byte = value;
    return ESP_OK;
}

// 写多字节
esp_err_t onewire_gpio_write_bytes(onewire_gpio_bus_handle_t bus, const uint8_t *data, size_t len) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    for (size_t i = 0; i < len; i++) {
        ret = onewire_gpio_write_byte(bus, data[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

// 读多字节
esp_err_t onewire_gpio_read_bytes(onewire_gpio_bus_handle_t bus, uint8_t *data, size_t len) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    for (size_t i = 0; i < len; i++) {
        ret = onewire_gpio_read_byte(bus, &data[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

// CRC8计算
uint8_t onewire_gpio_crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0;

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0x8C; // 1-Wire CRC8多项式
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

// 1-Wire ROM搜索命令
#define ONEWIRE_CMD_SEARCH_ROM       0xF0
#define ONEWIRE_CMD_READ_ROM         0x33
#define ONEWIRE_CMD_MATCH_ROM        0x55
#define ONEWIRE_CMD_SKIP_ROM         0xCC
#define ONEWIRE_CMD_ALARM_SEARCH     0xEC

// ROM搜索算法 - 参考正常工作的代码
static esp_err_t onewire_gpio_search_rom(onewire_gpio_bus_handle_t bus,
                                        onewire_gpio_device_iter_handle_t iter) {
    uint8_t id_bit, cmp_id_bit;
    uint8_t search_direction;
    uint8_t last_zero = 0;
    uint8_t rom_byte_number = 0;
    uint8_t rom_byte_mask = 1;
    static uint8_t last_discrepancy = 0;
    static uint8_t search_complete = 0;
      uint8_t rom[8] = {0};

    // 如果搜索已完成，重置状态
    if (search_complete) {
        search_complete = 0;
        last_discrepancy = 0;
        return ESP_ERR_NOT_FOUND;
    }

    // 发送复位脉冲
    esp_err_t ret = onewire_gpio_reset(bus);
    if (ret != ESP_OK) {
        return ret;
    }

    // 发送搜索ROM命令
    ret = onewire_gpio_write_byte(bus, ONEWIRE_CMD_SEARCH_ROM);
    if (ret != ESP_OK) {
        return ret;
    }

    // 搜索64位ROM
    for (uint8_t i = 0; i < 64; i++) {
        // 读取两个位（设备位和它的补码）
        ret = onewire_gpio_read_bit(bus, &id_bit);
        if (ret != ESP_OK) return ret;

        ret = onewire_gpio_read_bit(bus, &cmp_id_bit);
        if (ret != ESP_OK) return ret;

        if ((id_bit == 1) && (cmp_id_bit == 1)) {
            // 没有设备响应或总线错误
            return ESP_ERR_INVALID_RESPONSE;
        } else if (id_bit != cmp_id_bit) {
            // 两个位不同，使用真实值
            search_direction = id_bit;
        } else {
            // 两个位都是0，发现冲突
            if (i == last_discrepancy) {
                search_direction = 1;
            } else if (i > last_discrepancy) {
                search_direction = 0;
                last_zero = i;
            } else {
                search_direction = ((rom[rom_byte_number] & rom_byte_mask) > 0);
                if (search_direction == 0) {
                    last_zero = i;
                }
            }
        }

        // 在ROM中设置搜索方向
        if (search_direction == 1) {
            rom[rom_byte_number] |= rom_byte_mask;
        } else {
            rom[rom_byte_number] &= ~rom_byte_mask;
        }

        // 写入搜索方向位
        ret = onewire_gpio_write_bit(bus, search_direction);
        if (ret != ESP_OK) return ret;

        rom_byte_mask <<= 1;
        if (rom_byte_mask == 0) {
            rom_byte_number++;
            rom_byte_mask = 1;
        }
    }

    last_discrepancy = last_zero;

    if (last_discrepancy == 0) {
        search_complete = 1;
    }

    // 检查是否找到有效设备
    if (rom[0] == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    // 验证CRC
    // uint8_t calculated_crc = onewire_gpio_crc8(rom, 7);
    // if (calculated_crc != rom[7]) {
    //     ESP_LOGE(TAG, "CRC error for discovered device:");
    //     ESP_LOGE(TAG, "  Raw ROM data: %02X %02X %02X %02X %02X %02X %02X %02X",
    //              rom[0], rom[1], rom[2], rom[3], rom[4], rom[5], rom[6], rom[7]);
    //     ESP_LOGE(TAG, "  Calculated CRC: 0x%02X, Received CRC: 0x%02X",
    //              calculated_crc, rom[7]);
    //     ESP_LOGE(TAG, "  Family code: 0x%02X", rom[0]);
    //     return ESP_ERR_INVALID_CRC;
    // }

    // 将ROM地址转换为64位整数
    uint64_t rom_number = 0;
    for (int i = 0; i < 8; i++) {
        rom_number |= ((uint64_t)rom[i] << (i * 8));
    }

    iter->last_device = rom_number;
    iter->last_discrepancy = last_zero;

    // 如果没有冲突，搜索完成
    if (last_zero == 0) {
        iter->done = true;
    }

    return ESP_OK;
}

// 创建1-Wire总线
esp_err_t onewire_gpio_new_bus(const onewire_gpio_config_t *config, onewire_gpio_bus_handle_t *ret_bus) {
    if (config == NULL || ret_bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    onewire_gpio_bus_handle_t bus = calloc(1, sizeof(struct onewire_gpio_bus));
    if (bus == NULL) {
        return ESP_ERR_NO_MEM;
    }

    bus->gpio_num = config->gpio_num;
    bus->enable_pullup = config->enable_pullup;

    // 配置GPIO为开漏输出模式（1-Wire协议要求）
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->gpio_num),
        .mode = GPIO_MODE_OUTPUT_OD,  // 开漏输出
        .pull_up_en = config->enable_pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        free(bus);
        return ret;
    }

    // 设置初始状态为高电平（释放总线）
    gpio_set_level(config->gpio_num, 1);

    ESP_LOGI(TAG, "1-Wire GPIO bus created on GPIO %d", config->gpio_num);
    *ret_bus = bus;
    return ESP_OK;
}

// 删除1-Wire总线
esp_err_t onewire_gpio_del_bus(onewire_gpio_bus_handle_t bus) {
    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    free(bus);
    ESP_LOGI(TAG, "1-Wire GPIO bus deleted");
    return ESP_OK;
}

// 创建设备迭代器
esp_err_t onewire_gpio_new_device_iter(onewire_gpio_bus_handle_t bus, onewire_gpio_device_iter_handle_t *ret_iter) {
    if (bus == NULL || ret_iter == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    onewire_gpio_device_iter_handle_t iter = calloc(1, sizeof(struct onewire_gpio_iter));
    if (iter == NULL) {
        return ESP_ERR_NO_MEM;
    }

    iter->bus = bus;
    iter->last_discrepancy = 0;
    iter->last_family_discrepancy = 0;
    iter->last_device = 0;
    iter->done = false;

    *ret_iter = iter;
    return ESP_OK;
}

// 获取下一个设备
esp_err_t onewire_gpio_device_iter_get_next(onewire_gpio_device_iter_handle_t iter, onewire_gpio_device_t *device) {
    if (iter == NULL || device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (iter->done) {
        return ESP_ERR_NOT_FOUND;
    }

    // 执行复位
    esp_err_t ret = onewire_gpio_reset(iter->bus);
    if (ret != ESP_OK) {
        iter->done = true;
        return ret;
    }

    // 执行ROM搜索
    ret = onewire_gpio_search_rom(iter->bus, iter);
    if (ret != ESP_OK) {
        iter->done = true;
        return ret;
    }

    device->address = iter->last_device;
    return ESP_OK;
}

// 删除设备迭代器
esp_err_t onewire_gpio_del_device_iter(onewire_gpio_device_iter_handle_t iter) {
    if (iter == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    free(iter);
    return ESP_OK;
}