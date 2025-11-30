#include "current_sensor.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <memory>

extern "C" {
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
}

#include "hardware_defs.h"
#include "power_mgr.h"
#include "temp_mgr.h"

static const char *TAG = "current_sensor";

namespace {

constexpr float kExternalAdcVref = 3.3f;
constexpr float kExternalAdcResolution = 4096.0f;
constexpr float kInternalAdcVref = 3.3f;
constexpr float kInternalAdcResolution = 4096.0f;
constexpr uint8_t kAdcChannelCount = 10;  // ADC1 supports channels 0-9 on ESP32-S3
constexpr float kPowerVoltageDivider = (240000.0f + 910000.0f) / 240000.0f;

uint16_t s_ble_conn_handle = 0;
uint16_t s_ble_attr_handle = 0;
bool s_ble_notifications_enabled = false;

struct current_sensor_params_t {
    float offset;       // Zero-point voltage (V)
    float sensitivity;  // V/A sensitivity
    int8_t adc_channel; // -1 disabled, otherwise ADC channel index or GPIO (before resolve)
    int8_t pwm_index;   // PWM channel this sensor maps to
};

struct sensor_config_t {
    current_sensor_params_t *channel_current;
    uint8_t channel_count;
    uint32_t sample_interval_ms;
};

struct current_measurement_t {
    float voltage;
    float current;
    uint32_t timestamp_ms;
    bool is_valid;
};

struct voltage_measurement_t {
    float voltage;        // ADC side voltage
    float actual_voltage; // After divider scaling
    uint32_t timestamp_ms;
    bool is_valid;
};

struct sensor_readings_t {
    current_measurement_t *channel_currents;
    uint8_t channel_count;
    voltage_measurement_t power_voltage;
    bool *channel_current_valid;
};

struct RawVoltageSample {
    float channel_voltage[PWM_CHANNEL_COUNT];
    bool channel_valid[PWM_CHANNEL_COUNT];
    float power_voltage;
    bool power_voltage_valid;
    uint32_t timestamp_ms;
};

struct adc128s102_config_t {
    uint8_t cs_pin;
    uint8_t sclk_pin;
    uint8_t miso_pin;
    uint8_t mosi_pin;
    uint32_t clock_speed_hz;
    bool use_dma;
};

struct esp32_adc_config_t {
    adc_atten_t attenuation;
    adc_bitwidth_t bitwidth;
};

// Validation/utility helpers
bool validate_sensor_config(const sensor_config_t &config);
bool validate_internal_adc_config(const sensor_config_t &config);
bool validate_external_adc_config(const adc128s102_config_t &cfg);
bool clone_sensor_config(const sensor_config_t &src, sensor_config_t *dst);
bool copy_config_from_api(const current_sensor_config_t &api, sensor_config_t *dst);
void rebuild_channel_slot_map(const sensor_config_t &config, std::array<int8_t, PWM_CHANNEL_COUNT> *channel_slot_map);
void log_config_summary(const sensor_config_t &config, const std::array<int8_t, PWM_CHANNEL_COUNT> &slot_map);

class VoltageSensorDriver {
public:
    virtual ~VoltageSensorDriver() = default;
    virtual esp_err_t init(const sensor_config_t &config) = 0;
    virtual esp_err_t read_raw(RawVoltageSample *sample) = 0;
};

class Adc128s102Driver final : public VoltageSensorDriver {
public:
    explicit Adc128s102Driver(const adc128s102_config_t &cfg) : hw_config_(cfg) {}

    esp_err_t init(const sensor_config_t &config) override {
        config_ = config;
        if (!validate_external_adc_config(hw_config_)) {
            return ESP_ERR_INVALID_ARG;
        }
        if (!validate_sensor_config(config_)) {
            return ESP_ERR_INVALID_ARG;
        }
        spi_bus_config_t bus_cfg = {};
        bus_cfg.mosi_io_num = hw_config_.mosi_pin;
        bus_cfg.miso_io_num = hw_config_.miso_pin;
        bus_cfg.sclk_io_num = hw_config_.sclk_pin;
        bus_cfg.quadwp_io_num = -1;
        bus_cfg.quadhd_io_num = -1;
        bus_cfg.max_transfer_sz = 32;

        esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus_cfg, hw_config_.use_dma ? SPI_DMA_CH_AUTO : SPI_DMA_DISABLED);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Failed to init SPI bus: %s", esp_err_to_name(err));
            return err;
        }

        spi_device_interface_config_t dev_cfg = {};
        dev_cfg.mode = 0;
        dev_cfg.duty_cycle_pos = 128;
        dev_cfg.clock_speed_hz = static_cast<int>(hw_config_.clock_speed_hz);
        dev_cfg.spics_io_num = hw_config_.cs_pin;
        dev_cfg.queue_size = 1;

        err = spi_bus_add_device(SPI2_HOST, &dev_cfg, &device_);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(err));
            return err;
        }
        return ESP_OK;
    }

    esp_err_t read_raw(RawVoltageSample *sample) override {
        if (!device_ || !sample) {
            return ESP_ERR_INVALID_STATE;
        }

        reset_sample(sample);

        uint8_t tx_data[kAdcChannelCount * 2] = {0};
        uint8_t rx_data[kAdcChannelCount * 2] = {0};

        for (int i = 0; i < kAdcChannelCount; ++i) {
            tx_data[i * 2] = i << 3;
            tx_data[i * 2 + 1] = 0;
        }

        spi_transaction_t trans = {};
        trans.tx_buffer = tx_data;
        trans.rx_buffer = rx_data;
        trans.length = sizeof(tx_data) * 8;
        trans.rxlength = sizeof(rx_data) * 8;

        esp_err_t err = spi_device_transmit(device_, &trans);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(err));
            return err;
        }

        uint16_t raw_values[kAdcChannelCount] = {0};
        for (int i = 0; i < kAdcChannelCount; ++i) {
            uint16_t combined = (static_cast<uint16_t>(rx_data[i * 2]) << 8) | rx_data[i * 2 + 1];
            raw_values[i] = combined & 0x0FFF;
        }

        uint16_t corrected[kAdcChannelCount] = {0};
        corrected[0] = raw_values[1];
        corrected[1] = raw_values[2];
        corrected[2] = raw_values[3];
        corrected[3] = raw_values[4];
        corrected[4] = raw_values[5];
        corrected[5] = raw_values[6];
        corrected[6] = raw_values[7];
        corrected[7] = raw_values[0];

        for (int i = 0; i < config_.channel_count; ++i) {
            int adc_idx = config_.channel_current[i].adc_channel;
            int pwm_idx = config_.channel_current[i].pwm_index;
            if (adc_idx >= 0 && adc_idx < kAdcChannelCount && pwm_idx >= 0 && pwm_idx < PWM_CHANNEL_COUNT) {
                sample->channel_valid[pwm_idx] = true;
                sample->channel_voltage[pwm_idx] = corrected[adc_idx] * kExternalAdcVref / kExternalAdcResolution;
            }
        }

        sample->power_voltage_valid = false;
        sample->timestamp_ms = esp_timer_get_time() / 1000;
        return ESP_OK;
    }

private:
    void reset_sample(RawVoltageSample *sample) {
        sample->power_voltage = 0.0f;
        sample->power_voltage_valid = false;
        sample->timestamp_ms = 0;
        for (int i = 0; i < PWM_CHANNEL_COUNT; ++i) {
            sample->channel_valid[i] = false;
            sample->channel_voltage[i] = 0.0f;
        }
    }

    sensor_config_t config_ = {};
    adc128s102_config_t hw_config_;
    spi_device_handle_t device_ = nullptr;
};

class Esp32InternalAdcDriver final : public VoltageSensorDriver {
public:
    explicit Esp32InternalAdcDriver(const esp32_adc_config_t &cfg) : hw_config_(cfg) {}

    esp_err_t init(const sensor_config_t &config) override {
        config_ = config;
        if (!validate_sensor_config(config_)) {
            return ESP_ERR_INVALID_ARG;
        }
        if (!validate_internal_adc_config(config_)) {
            return ESP_ERR_INVALID_ARG;
        }
        adc_unit_t unit;
        adc_channel_t chan;
        ESP_ERROR_CHECK(adc_oneshot_io_to_channel(GPIO_NUM_1, &unit, &chan));
        resolved_unit_ = unit;

        adc_oneshot_unit_init_cfg_t unit_cfg = {};
        unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
        unit_cfg.unit_id = resolved_unit_;
        esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &unit_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create ADC unit: %s", esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "ESP32 ADC driver using unit %d (GPIO-derived)", unit_cfg.unit_id);

        adc_oneshot_chan_cfg_t chan_cfg = {};
        chan_cfg.atten = hw_config_.attenuation;
        chan_cfg.bitwidth = hw_config_.bitwidth;

        // Configure power voltage channel on GPIO1
        resolved_power_channel_ = chan;
        ESP_ERROR_CHECK(adc_oneshot_config_channel(unit_, resolved_power_channel_, &chan_cfg));
        ESP_LOGI(TAG, "Initialized power voltage on unit %d channel %d", unit, chan);

        // Configure all requested channels (deduplicate)
        std::array<bool, kAdcChannelCount> configured{};
        for (int i = 0; i < config_.channel_count; ++i) {
            int adc_idx = config_.channel_current[i].adc_channel;
            if (adc_idx < 0) {
                continue;
            }
            adc_channel_t adc_ch = static_cast<adc_channel_t>(adc_idx);
            config_.channel_current[i].adc_channel = static_cast<int8_t>(adc_ch);
            if (adc_ch < kAdcChannelCount && !configured[adc_ch]) {
                ESP_ERROR_CHECK(adc_oneshot_config_channel(unit_, adc_ch, &chan_cfg));
                configured[adc_ch] = true;
                ESP_LOGI(TAG, "Initialized channel current on unit %d channel %d (PWM ch%d)", resolved_unit_, adc_ch,
                         config_.channel_current[i].pwm_index + 1);
            }
        }
        return ESP_OK;
    }

    esp_err_t read_raw(RawVoltageSample *sample) override {
        if (!sample || !unit_) {
            return ESP_ERR_INVALID_STATE;
        }

        reset_sample(sample);

        int raw = 0;
        if (adc_oneshot_read(unit_, resolved_power_channel_, &raw) == ESP_OK) {
            const float measured = static_cast<float>(raw) * kInternalAdcVref / kInternalAdcResolution;
            sample->power_voltage = measured * kPowerVoltageDivider;
            sample->power_voltage_valid = true;
        }

        
        for (int i = 0; i < config_.channel_count; ++i) {
            const auto &sensor = config_.channel_current[i];
            adc_channel_t adc_ch =
                sensor.adc_channel >= 0 ? static_cast<adc_channel_t>(sensor.adc_channel) : kInvalidChannel;
            int pwm_idx = sensor.pwm_index;
            if (adc_ch == kInvalidChannel || pwm_idx < 0 || pwm_idx >= PWM_CHANNEL_COUNT) {
                continue;
            }
            esp_err_t r = adc_oneshot_read(unit_, adc_ch, &raw);
            if (r == ESP_OK) {
                sample->channel_valid[pwm_idx] = true;
                sample->channel_voltage[pwm_idx] =
                    static_cast<float>(raw) * kInternalAdcVref / kInternalAdcResolution;
                ESP_LOGI(TAG, "PWM ch%d (ADC ch%d) raw=%d voltage=%.4f",
                         pwm_idx + 1, adc_ch, raw, sample->channel_voltage[pwm_idx]);
            } else {
                ESP_LOGW(TAG, "ADC read failed for PWM ch%d (ADC ch%d): %s",
                         pwm_idx + 1, adc_ch, esp_err_to_name(r));
            }
        }

        sample->timestamp_ms = esp_timer_get_time() / 1000;
        return ESP_OK;
    }

private:
    void reset_sample(RawVoltageSample *sample) {
        sample->power_voltage = 0.0f;
        sample->power_voltage_valid = false;
        sample->timestamp_ms = 0;
        for (int i = 0; i < PWM_CHANNEL_COUNT; ++i) {
            sample->channel_valid[i] = false;
            sample->channel_voltage[i] = 0.0f;
        }
    }

    static constexpr adc_channel_t kInvalidChannel = static_cast<adc_channel_t>(-1);
    adc_unit_t resolved_unit_ = ADC_UNIT_1;
    adc_channel_t resolved_power_channel_ = kInvalidChannel;

    sensor_config_t config_ = {};
    esp32_adc_config_t hw_config_;
    adc_oneshot_unit_handle_t unit_ = nullptr;
};

sensor_config_t build_sensor_config_from_hardware_defs(std::array<int8_t, PWM_CHANNEL_COUNT> *channel_slot_map) {
    sensor_config_t config = {};
    if (channel_slot_map) {
        channel_slot_map->fill(-1);
    }

    uint8_t channel_count = 0;
    for (int i = 0; i < PWM_CHANNEL_COUNT; i++) {
        if (HAS_CHANNEL_CURRENT_SENSOR(i)) {
            channel_count++;
        }
    }

    config.channel_current = nullptr;
    if (channel_count > 0) {
        config.channel_current = static_cast<current_sensor_params_t*>(
            calloc(channel_count, sizeof(current_sensor_params_t)));
    }
    config.channel_count = channel_count;

    int idx = 0;
    for (int i = 0; i < PWM_CHANNEL_COUNT; i++) {
        if (!HAS_CHANNEL_CURRENT_SENSOR(i)) {
            continue;
        }
        config.channel_current[idx].adc_channel = CHANNEL_CURRENT_ADC_CHS[i];
        config.channel_current[idx].offset = CHANNEL_CURRENT_OFFSET;
        config.channel_current[idx].sensitivity = CHANNEL_CURRENT_SENSITIVITY;
        config.channel_current[idx].pwm_index = i;
        if (channel_slot_map) {
            (*channel_slot_map)[i] = idx;
        }
        idx++;
    }

    config.sample_interval_ms = 100;
    return config;
}

void free_sensor_config(sensor_config_t *config) {
    if (!config) {
        return;
    }
    if (config->channel_current) {
        free(config->channel_current);
    }
    config->channel_current = nullptr;
    config->channel_count = 0;
}

#if defined(EXTERNAL_ADC)
adc128s102_config_t build_external_adc_config() {
    adc128s102_config_t cfg = {};
    cfg.cs_pin = EXTERNAL_ADC_CS_PIN;
    cfg.sclk_pin = EXTERNAL_ADC_SCLK_PIN;
    cfg.miso_pin = EXTERNAL_ADC_MISO_PIN;
    cfg.mosi_pin = EXTERNAL_ADC_MOSI_PIN;
    cfg.clock_speed_hz = 500000;
    cfg.use_dma = true;
    return cfg;
}
#endif

esp32_adc_config_t build_internal_adc_config() {
    esp32_adc_config_t cfg = {};
    cfg.attenuation = ADC_ATTEN_DB_12;
    cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    return cfg;
}

void rebuild_channel_slot_map(const sensor_config_t &config, std::array<int8_t, PWM_CHANNEL_COUNT> *channel_slot_map) {
    if (!channel_slot_map) {
        return;
    }
    channel_slot_map->fill(-1);
    int idx = 0;
    for (int pwm = 0; pwm < PWM_CHANNEL_COUNT; ++pwm) {
        if (!HAS_CHANNEL_CURRENT_SENSOR(pwm)) {
            continue;
        }
        if (idx < config.channel_count) {
            (*channel_slot_map)[pwm] = idx;
            idx++;
        }
    }
}

bool validate_external_adc_config(const adc128s102_config_t &cfg) {
    const bool pins_valid = cfg.cs_pin <= 40 && cfg.sclk_pin <= 40 && cfg.miso_pin <= 40 && cfg.mosi_pin <= 40;
    if (!pins_valid) {
        ESP_LOGE(TAG, "External ADC pin config invalid (cs=%d, sclk=%d, miso=%d, mosi=%d)", cfg.cs_pin, cfg.sclk_pin,
                 cfg.miso_pin, cfg.mosi_pin);
    }
    if (cfg.clock_speed_hz == 0) {
        ESP_LOGE(TAG, "External ADC clock speed must be > 0");
    }
    return pins_valid && cfg.clock_speed_hz > 0;
}

bool validate_internal_adc_config(const sensor_config_t &config) {
    bool ok = true;
    for (int i = 0; i < config.channel_count; ++i) {
        int ch = config.channel_current[i].adc_channel;
        if (ch >= kAdcChannelCount) {
            ESP_LOGE(TAG, "Channel %d ADC index %d out of range", i, ch);
            ok = false;
        }
    }
    return ok;
}

bool validate_sensor_config(const sensor_config_t &config) {
    if (config.channel_count > PWM_CHANNEL_COUNT) {
        ESP_LOGE(TAG, "Channel count %u exceeds PWM channel count %d", config.channel_count, PWM_CHANNEL_COUNT);
        return false;
    }
    std::array<bool, kAdcChannelCount> used{};
    auto validate_adc_field = [&](int adc_idx, const char *label) {
        if (adc_idx < -1 || adc_idx >= kAdcChannelCount) {
            ESP_LOGE(TAG, "%s ADC channel %d out of range", label, adc_idx);
            return false;
        }
        if (adc_idx >= 0 && used[adc_idx]) {
            ESP_LOGW(TAG, "Duplicate ADC channel %d used", adc_idx);
        }
        if (adc_idx >= 0) {
            used[adc_idx] = true;
        }
        return true;
    };

    bool ok = true;
    for (int i = 0; i < config.channel_count; ++i) {
        if (!validate_adc_field(config.channel_current[i].adc_channel, "Channel current")) {
            ok = false;
        }
    }
    return ok;
}

bool clone_sensor_config(const sensor_config_t &src, sensor_config_t *dst) {
    if (!dst) {
        return false;
    }
    free_sensor_config(dst);
    dst->channel_count = src.channel_count;
    dst->sample_interval_ms = src.sample_interval_ms;
    if (src.channel_count == 0) {
        dst->channel_current = nullptr;
        return true;
    }
    dst->channel_current =
        static_cast<current_sensor_params_t *>(calloc(src.channel_count, sizeof(current_sensor_params_t)));
    if (!dst->channel_current) {
        dst->channel_count = 0;
        return false;
    }
    for (int i = 0; i < src.channel_count; ++i) {
        dst->channel_current[i] = src.channel_current[i];
    }
    return true;
}

bool copy_config_from_api(const current_sensor_config_t &api, sensor_config_t *dst) {
    if (!dst) {
        return false;
    }
    if (api.channel_count > PWM_CHANNEL_COUNT) {
        ESP_LOGE(TAG, "API channel count %u exceeds maximum %d", api.channel_count, PWM_CHANNEL_COUNT);
        return false;
    }
    free_sensor_config(dst);
    dst->sample_interval_ms = api.sample_interval_ms == 0 ? 100 : api.sample_interval_ms;
    dst->channel_count = api.channel_count;
    if (api.channel_count == 0) {
        dst->channel_current = nullptr;
        return true;
    }
    dst->channel_current =
        static_cast<current_sensor_params_t *>(calloc(api.channel_count, sizeof(current_sensor_params_t)));
    if (!dst->channel_current) {
        dst->channel_count = 0;
        return false;
    }
    for (int i = 0; i < api.channel_count; ++i) {
        dst->channel_current[i].adc_channel = api.channel_current[i].adc_channel;
        dst->channel_current[i].offset = api.channel_current[i].offset;
        dst->channel_current[i].sensitivity = api.channel_current[i].sensitivity;
        dst->channel_current[i].pwm_index = (i < PWM_CHANNEL_COUNT) ? i : -1;
    }
    return true;
}

void log_config_summary(const sensor_config_t &config, const std::array<int8_t, PWM_CHANNEL_COUNT> &slot_map) {
    ESP_LOGI(TAG, "Sensor config: channels=%u, sample_interval=%ums", config.channel_count, config.sample_interval_ms);
    for (int pwm = 0; pwm < PWM_CHANNEL_COUNT; ++pwm) {
        int slot = slot_map[pwm];
        if (slot < 0 || slot >= config.channel_count) {
            ESP_LOGI(TAG, "  CH%d: disabled", pwm + 1);
            continue;
        }
        const auto &sensor = config.channel_current[slot];
        ESP_LOGI(TAG, "  CH%d: adc=%d offset=%.3f sens=%.3f", pwm + 1, sensor.adc_channel, sensor.offset,
                 sensor.sensitivity);
    }
}

#if defined(BUILTIN_ADC)
static bool resolve_gpio_to_adc(sensor_config_t *config) {
    if (!config) {
        return false;
    }
    adc_unit_t unit = ADC_UNIT_1;
    adc_channel_t chan = static_cast<adc_channel_t>(-1);

    for (int i = 0; i < config->channel_count; ++i) {
        int gpio_idx = config->channel_current[i].adc_channel;
        if (gpio_idx < 0) {
            continue;
        }
        if (adc_oneshot_io_to_channel(static_cast<gpio_num_t>(gpio_idx), &unit, &chan) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to map PWM ch%d GPIO%d to ADC channel", config->channel_current[i].pwm_index + 1,
                     gpio_idx);
            return false;
        }
        config->channel_current[i].adc_channel = static_cast<int8_t>(chan);
        ESP_LOGI(TAG, "Mapped PWM ch%d GPIO%d -> ADC unit %d channel %d", config->channel_current[i].pwm_index + 1,
                 gpio_idx, unit, chan);
    }
    return true;
}
#endif

class SensorManager {
public:
    ~SensorManager() {
        free_sensor_config(&config_);
        free_sensor_config(&override_config_);
        free_readings();
        if (mutex_) {
            vSemaphoreDelete(mutex_);
        }
    }

    esp_err_t init() {
        if (!mutex_) {
            mutex_ = xSemaphoreCreateMutex();
            if (!mutex_) {
                return ESP_ERR_NO_MEM;
            }
        }

        free_sensor_config(&config_);
        free_readings();

        sensor_config_t built = build_sensor_config_from_hardware_defs(nullptr);
        if (has_override_config_) {
            if (!clone_sensor_config(override_config_, &config_)) {
                free_sensor_config(&built);
                return ESP_ERR_NO_MEM;
            }
        } else {
            if (!clone_sensor_config(built, &config_)) {
                free_sensor_config(&built);
                return ESP_ERR_NO_MEM;
            }
        }
        rebuild_channel_slot_map(config_, &channel_slot_map_);
        free_sensor_config(&built);

        calibration_.acs712_offset = CHANNEL_CURRENT_OFFSET;
        calibration_.acs712_sensitivity = CHANNEL_CURRENT_SENSITIVITY;
        apply_calibration_to_config();

        // Map GPIO indexes to ADC channels for built-in ADC
#if defined(BUILTIN_ADC)
        if (!resolve_gpio_to_adc(&config_)) {
            free_sensor_config(&config_);
            return ESP_ERR_INVALID_ARG;
        }
#endif

        if (!initialize_readings()) {
            free_sensor_config(&config_);
            return ESP_ERR_NO_MEM;
        }

        driver_ = create_driver();
        if (!driver_) {
            return ESP_ERR_NOT_SUPPORTED;
        }
        if (!validate_sensor_config(config_)) {
            return ESP_ERR_INVALID_ARG;
        }
        log_config_summary(config_, channel_slot_map_);
        return driver_->init(config_);
    }

    esp_err_t start(uint32_t interval_ms) {
        if (task_handle_) {
            return ESP_ERR_INVALID_STATE;
        }
        interval_ms_ = std::max<uint32_t>(interval_ms, std::max<uint32_t>(config_.sample_interval_ms, 50));
        BaseType_t rc = xTaskCreatePinnedToCore(&SensorManager::task_entry, "current_sensor", 4096, this, 5,
                                                &task_handle_, 0);
        if (rc != pdPASS) {
            task_handle_ = nullptr;
            return ESP_ERR_NO_MEM;
        }
        return ESP_OK;
    }

    esp_err_t stop() {
        if (!task_handle_) {
            return ESP_ERR_INVALID_STATE;
        }
        TaskHandle_t handle = task_handle_;
        task_handle_ = nullptr;
        vTaskDelete(handle);
        return ESP_OK;
    }

    esp_err_t get_latest(current_sensor_data_t *out) {
        if (!out) {
            return ESP_ERR_INVALID_ARG;
        }
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        *out = compatibility_cache_;
        xSemaphoreGive(mutex_);
        return ESP_OK;
    }

    esp_err_t clone_config_api(current_sensor_config_t *out) {
        if (!out) {
            return ESP_ERR_INVALID_ARG;
        }
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        memset(out, 0, sizeof(*out));
        out->channel_count = config_.channel_count;
        out->sample_interval_ms = config_.sample_interval_ms;
        if (config_.channel_count > 0 && !config_.channel_current) {
            xSemaphoreGive(mutex_);
            return ESP_ERR_INVALID_STATE;
        }
        if (config_.channel_count > 0) {
            out->channel_current = static_cast<current_sensor_params_config_t *>(
                calloc(config_.channel_count, sizeof(current_sensor_params_config_t)));
            if (!out->channel_current) {
                xSemaphoreGive(mutex_);
                out->channel_count = 0;
                return ESP_ERR_NO_MEM;
            }
            for (int i = 0; i < config_.channel_count; ++i) {
                out->channel_current[i].adc_channel = config_.channel_current[i].adc_channel;
                out->channel_current[i].offset = config_.channel_current[i].offset;
                out->channel_current[i].sensitivity = config_.channel_current[i].sensitivity;
            }
        }
        xSemaphoreGive(mutex_);
        return ESP_OK;
    }

    esp_err_t apply_api_config(const current_sensor_config_t *config) {
        if (!config) {
            return ESP_ERR_INVALID_ARG;
        }
        if (task_handle_) {
            return ESP_ERR_INVALID_STATE;
        }
        sensor_config_t converted = {};
        if (!copy_config_from_api(*config, &converted)) {
            return ESP_ERR_INVALID_ARG;
        }
        if (!validate_sensor_config(converted)) {
            free_sensor_config(&converted);
            return ESP_ERR_INVALID_ARG;
        }
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
            free_sensor_config(&converted);
            return ESP_ERR_TIMEOUT;
        }
        free_sensor_config(&override_config_);
        if (!clone_sensor_config(converted, &override_config_)) {
            xSemaphoreGive(mutex_);
            free_sensor_config(&converted);
            return ESP_ERR_NO_MEM;
        }
        has_override_config_ = true;
        xSemaphoreGive(mutex_);
        free_sensor_config(&converted);
        return ESP_OK;
    }

    esp_err_t set_calibration(const current_sensor_calibration_t *cal) {
        (void)cal;
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t get_calibration(current_sensor_calibration_t *cal) {
        if (!cal) {
            return ESP_ERR_INVALID_ARG;
        }
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        *cal = calibration_;
        xSemaphoreGive(mutex_);
        return ESP_OK;
    }

    esp_err_t calibrate_zero(uint16_t sample_count) {
        (void)sample_count;
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t get_power_voltage_mv(uint16_t *out_mv) {
        if (!out_mv) {
            return ESP_ERR_INVALID_ARG;
        }
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        if (!readings_.power_voltage.is_valid) {
            xSemaphoreGive(mutex_);
            return ESP_ERR_NOT_SUPPORTED;
        }
        float mv = readings_.power_voltage.actual_voltage * 1000.0f;
        if (mv < 0.0f) {
            mv = 0.0f;
        }
        if (mv > 65535.0f) {
            mv = 65535.0f;
        }
        *out_mv = static_cast<uint16_t>(mv);
        xSemaphoreGive(mutex_);
        return ESP_OK;
    }

    esp_err_t ble_read(uint8_t *buffer, size_t size) {
        const size_t payload_size = 8 + 4 * PWM_CHANNEL_COUNT;
        if (size < payload_size) {
            return ESP_ERR_INVALID_ARG;
        }
        int offset = 0;

        uint16_t vin_mv;
        if (power_mgr_get_voltage_mv(&vin_mv, false) == ESP_OK) {
            buffer[offset++] = (vin_mv >> 8) & 0xFF;
            buffer[offset++] = vin_mv & 0xFF;
        } else {
            buffer[offset++] = 0xFF;
            buffer[offset++] = 0xFF;
        }

        // Temperature data - use status indicators instead of re-sampling
        // This prevents double sampling that interferes with DS18B20 readings
        buffer[offset++] = 0x80;  // Power temp status placeholder
        buffer[offset++] = 0x00;
        buffer[offset++] = 0x80;  // Control temp status placeholder
        buffer[offset++] = 0x00;

        current_sensor_data_t data;
        if (get_latest(&data) == ESP_OK) {
            union { float f; uint32_t u; } conv;
            for (int i = 0; i < PWM_CHANNEL_COUNT; ++i) {
                conv.f = data.channel_currents[i];
                buffer[offset++] = (conv.u >> 24) & 0xFF;
                buffer[offset++] = (conv.u >> 16) & 0xFF;
                buffer[offset++] = (conv.u >> 8) & 0xFF;
                buffer[offset++] = conv.u & 0xFF;
            }
        } else {
            size_t remaining = payload_size - offset - 2; // leave space for status bytes
            for (size_t i = 0; i < remaining; ++i) {
                buffer[offset++] = 0xFF;
            }
        }

        uint8_t status = 0;
        if (power_mgr_is_thermal_protection_active()) {
            status |= 0x01;
        }
        // Use cached temperature value from power_mgr to avoid re-sampling
        int16_t cached_temp;
        if (power_mgr_get_latest_temp(&cached_temp) == ESP_OK) {
            status |= 0x02;  // Temperature data available
        }
        if (get_latest(&data) == ESP_OK) {
            status |= 0x04;
        }
        status |= 0x08;  // assume calibrated
        if (power_mgr_external_power_is_on()) {
            status |= 0x10;
        }
        buffer[offset++] = status;
        buffer[offset++] = 0x00;
        return (offset == payload_size) ? ESP_OK : ESP_FAIL;
    }

private:
    static void task_entry(void *arg) {
        auto *self = reinterpret_cast<SensorManager *>(arg);
        self->task_loop();
    }

    void task_loop() {
        RawVoltageSample sample;
        while (true) {
            if (driver_ && driver_->read_raw(&sample) == ESP_OK) {
                update_latest(sample);
            }
            vTaskDelay(pdMS_TO_TICKS(interval_ms_));
        }
    }

    bool initialize_readings() {
        free_readings();
        if (config_.channel_count == 0) {
            readings_.channel_currents = nullptr;
            readings_.channel_current_valid = nullptr;
            readings_.channel_count = 0;
            readings_.power_voltage.actual_voltage = -1.0f;
            readings_.power_voltage.voltage = 0.0f;
            readings_.power_voltage.timestamp_ms = 0;
            readings_.power_voltage.is_valid = false;
            reset_compatibility_cache();
            return true;
        }

        readings_.channel_currents = static_cast<current_measurement_t *>(
            calloc(config_.channel_count, sizeof(current_measurement_t)));
        readings_.channel_current_valid = static_cast<bool *>(
            calloc(config_.channel_count, sizeof(bool)));
        readings_.channel_count = config_.channel_count;
        if (!readings_.channel_currents || !readings_.channel_current_valid) {
            return false;
        }

        for (int i = 0; i < config_.channel_count; ++i) {
            readings_.channel_currents[i].voltage = 0.0f;
            readings_.channel_currents[i].current = -1.0f;
            readings_.channel_currents[i].timestamp_ms = 0;
            readings_.channel_currents[i].is_valid = false;
            readings_.channel_current_valid[i] = false;
        }
        readings_.power_voltage.actual_voltage = -1.0f;
        readings_.power_voltage.voltage = 0.0f;
        readings_.power_voltage.timestamp_ms = 0;
        readings_.power_voltage.is_valid = false;
        reset_compatibility_cache();
        return true;
    }

    void free_readings() {
        if (readings_.channel_currents) {
            free(readings_.channel_currents);
        }
        if (readings_.channel_current_valid) {
            free(readings_.channel_current_valid);
        }
        readings_.channel_currents = nullptr;
        readings_.channel_current_valid = nullptr;
        readings_.channel_count = 0;
    }

    void apply_calibration_to_config() {
        for (int i = 0; i < config_.channel_count; ++i) {
            config_.channel_current[i].offset = calibration_.acs712_offset;
            config_.channel_current[i].sensitivity = calibration_.acs712_sensitivity;
        }
    }

    void update_latest(const RawVoltageSample &sample) {
        if (mutex_ && xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
            return;
        }
        const uint32_t timestamp = sample.timestamp_ms;

        for (int i = 0; i < config_.channel_count; ++i) {
            const auto &sensor = config_.channel_current[i];
            readings_.channel_currents[i].is_valid = false;
            readings_.channel_currents[i].timestamp_ms = timestamp;
            readings_.channel_currents[i].current = -1.0f;
            readings_.channel_currents[i].voltage = 0.0f;

            int pwm_idx = sensor.pwm_index;
            if (pwm_idx >= 0 && pwm_idx < PWM_CHANNEL_COUNT &&
                sample.channel_valid[pwm_idx] && sensor.sensitivity > 0.0f) {
                readings_.channel_currents[i].voltage = sample.channel_voltage[pwm_idx];
                readings_.channel_currents[i].current =
                    (readings_.channel_currents[i].voltage - sensor.offset) / sensor.sensitivity;
                readings_.channel_currents[i].is_valid = true;
            }
            readings_.channel_current_valid[i] = readings_.channel_currents[i].is_valid;
        }

        readings_.power_voltage.is_valid = sample.power_voltage_valid;
        readings_.power_voltage.timestamp_ms = timestamp;
        if (sample.power_voltage_valid) {
            readings_.power_voltage.actual_voltage = sample.power_voltage;
            readings_.power_voltage.voltage = sample.power_voltage / kPowerVoltageDivider;
        } else {
            readings_.power_voltage.actual_voltage = -1.0f;
            readings_.power_voltage.voltage = 0.0f;
        }

        update_compatibility_cache(timestamp);
        if (mutex_) {
            xSemaphoreGive(mutex_);
        }
        notify_ble();
    }

    void reset_compatibility_cache() {
        compatibility_cache_.valid_mask = 0;
        compatibility_cache_.timestamp_ms = 0;
        for (float &value : compatibility_cache_.channel_currents) {
            value = -1.0f;
        }
    }

    void update_compatibility_cache(uint32_t timestamp_ms) {
        reset_compatibility_cache();
        compatibility_cache_.timestamp_ms = timestamp_ms;

        for (int pwm = 0; pwm < PWM_CHANNEL_COUNT; ++pwm) {
            const int sensor_idx = channel_slot_map_[pwm];
            if (sensor_idx < 0 || sensor_idx >= readings_.channel_count) {
                compatibility_cache_.channel_currents[pwm] = -1.0f;
                continue;
            }
            if (readings_.channel_currents[sensor_idx].is_valid) {
                compatibility_cache_.channel_currents[pwm] = readings_.channel_currents[sensor_idx].current;
                compatibility_cache_.valid_mask |= (1u << (pwm + 1));
            } else {
                compatibility_cache_.channel_currents[pwm] = -1.0f;
            }
        }
    }

    std::unique_ptr<VoltageSensorDriver> create_driver() {
#if defined(EXTERNAL_ADC)
        return std::unique_ptr<VoltageSensorDriver>(new Adc128s102Driver(build_external_adc_config()));
#elif defined(BUILTIN_ADC)
        return std::unique_ptr<VoltageSensorDriver>(new Esp32InternalAdcDriver(build_internal_adc_config()));
#else
        return nullptr;
#endif
    }

    void notify_ble() {
        if (!s_ble_notifications_enabled || s_ble_conn_handle == 0) {
            return;
        }
        uint8_t payload[36];
        if (ble_read(payload, sizeof(payload)) != ESP_OK) {
            return;
        }

        struct os_mbuf *om = ble_hs_mbuf_att_pkt();
        if (!om) {
            ESP_LOGW(TAG, "Failed to allocate mbuf for monitoring notification");
            return;
        }
        if (os_mbuf_append(om, payload, sizeof(payload)) != 0) {
            ESP_LOGW(TAG, "Failed to append monitoring payload");
            os_mbuf_free_chain(om);
            return;
        }
        int rc = ble_gatts_notify_custom(s_ble_conn_handle, s_ble_attr_handle, om);
        if (rc != 0) {
            ESP_LOGW(TAG, "Failed to send monitoring notification: %d", rc);
            os_mbuf_free_chain(om);
        } else {
            ESP_LOGD(TAG, "Sent monitoring notification");
        }
    }

    sensor_config_t config_ = {};
    sensor_config_t override_config_ = {};
    bool has_override_config_ = false;
    sensor_readings_t readings_ = {};
    current_sensor_calibration_t calibration_ = {};
    current_sensor_data_t compatibility_cache_ = {};
    std::unique_ptr<VoltageSensorDriver> driver_;
    std::array<int8_t, PWM_CHANNEL_COUNT> channel_slot_map_{};
    SemaphoreHandle_t mutex_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
    uint32_t interval_ms_ = 100;
};

SensorManager &get_manager() {
    static SensorManager manager;
    return manager;
}

}  // namespace

extern "C" {

esp_err_t current_sensor_init(void) { return get_manager().init(); }
esp_err_t current_sensor_start(uint32_t interval_ms) { return get_manager().start(interval_ms); }
esp_err_t current_sensor_stop(void) { return get_manager().stop(); }
esp_err_t current_sensor_get_latest(current_sensor_data_t *data) { return get_manager().get_latest(data); }
esp_err_t current_sensor_set_calibration(const current_sensor_calibration_t *cal) { return get_manager().set_calibration(cal); }
esp_err_t current_sensor_get_calibration(current_sensor_calibration_t *cal) { return get_manager().get_calibration(cal); }
esp_err_t current_sensor_calibrate_zero(uint16_t sample_count) { return get_manager().calibrate_zero(sample_count); }
esp_err_t current_sensor_config_clone(current_sensor_config_t *config_out) { return get_manager().clone_config_api(config_out); }
void current_sensor_config_free(current_sensor_config_t *config) {
    if (!config) {
        return;
    }
    if (config->channel_current) {
        free(config->channel_current);
        config->channel_current = nullptr;
    }
    config->channel_count = 0;
}
esp_err_t current_sensor_apply_config(const current_sensor_config_t *config) { return get_manager().apply_api_config(config); }
esp_err_t current_sensor_get_power_voltage_mv(uint16_t *out_mv) { return get_manager().get_power_voltage_mv(out_mv); }

int current_sensor_ble_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    const size_t payload_size = 8 + 4 * PWM_CHANNEL_COUNT;
    uint8_t payload[payload_size];
    if (get_manager().ble_read(payload, sizeof(payload)) != ESP_OK) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    return os_mbuf_append(ctxt->om, payload, sizeof(payload)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

void current_sensor_ble_subscribe(uint16_t conn_handle, uint16_t attr_handle, bool enabled) {
    s_ble_conn_handle = conn_handle;
    s_ble_attr_handle = attr_handle;
    s_ble_notifications_enabled = enabled;
    ESP_LOGI(TAG, "BLE subscription %s for connection %u", enabled ? "enabled" : "disabled", conn_handle);
}

}  // extern "C"
