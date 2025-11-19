#include "current_sensor.h"

#include <memory>
#include <algorithm>

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

constexpr float kExternalAdcVref = 5.0f;
constexpr float kExternalAdcResolution = 4096.0f;
constexpr float kInternalAdcVref = 3.3f;
constexpr float kInternalAdcResolution = 4096.0f;
constexpr uint8_t kExternalAdcChannelCount = 8;

uint16_t s_ble_conn_handle = 0;
uint16_t s_ble_attr_handle = 0;
bool s_ble_notifications_enabled = false;

struct CurrentSensorConfig {
    bool total_present = false;
    int total_channel = -1;
    int channel_map[PWM_CHANNEL_COUNT] = {0};
    uint8_t channel_count = 0;
};

struct RawVoltageSample {
    bool total_valid = false;
    float total_voltage = 0.0f;
    bool channel_valid[PWM_CHANNEL_COUNT] = {false};
    float channel_voltage[PWM_CHANNEL_COUNT] = {0.0f};
};

class ICurrentSensorDriver {
public:
    virtual ~ICurrentSensorDriver() = default;
    virtual esp_err_t init(const CurrentSensorConfig &config) = 0;
    virtual esp_err_t read_raw(RawVoltageSample *sample) = 0;
};

class Adc128s102Driver final : public ICurrentSensorDriver {
public:
    esp_err_t init(const CurrentSensorConfig &config) override {
        config_ = config;

        spi_bus_config_t bus_cfg = {};
        bus_cfg.mosi_io_num = EXTERNAL_ADC_MOSI_PIN;
        bus_cfg.miso_io_num = EXTERNAL_ADC_MISO_PIN;
        bus_cfg.sclk_io_num = EXTERNAL_ADC_SCLK_PIN;
        bus_cfg.quadwp_io_num = -1;
        bus_cfg.quadhd_io_num = -1;
        bus_cfg.max_transfer_sz = 32;

        esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Failed to init SPI bus: %s", esp_err_to_name(err));
            return err;
        }

        spi_device_interface_config_t dev_cfg = {};
        dev_cfg.mode = 0;
        dev_cfg.duty_cycle_pos = 128;
        dev_cfg.clock_speed_hz = 500000;
        dev_cfg.spics_io_num = EXTERNAL_ADC_CS_PIN;
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

        sample->total_valid = false;
        for (int i = 0; i < PWM_CHANNEL_COUNT; ++i) {
            sample->channel_valid[i] = false;
            sample->channel_voltage[i] = 0.0f;
        }

        uint8_t tx_data[kExternalAdcChannelCount * 2] = {0};
        uint8_t rx_data[kExternalAdcChannelCount * 2] = {0};

        for (int i = 0; i < kExternalAdcChannelCount; ++i) {
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

        uint16_t raw_values[kExternalAdcChannelCount] = {0};
        for (int i = 0; i < kExternalAdcChannelCount; ++i) {
            uint16_t combined = (static_cast<uint16_t>(rx_data[i * 2]) << 8) | rx_data[i * 2 + 1];
            raw_values[i] = combined & 0x0FFF;
        }

        uint16_t corrected[kExternalAdcChannelCount] = {0};
        corrected[0] = raw_values[1];
        corrected[1] = raw_values[2];
        corrected[2] = raw_values[3];
        corrected[3] = raw_values[4];
        corrected[4] = raw_values[5];
        corrected[5] = raw_values[6];
        corrected[6] = raw_values[7];
        corrected[7] = raw_values[0];

        if (config_.total_present && config_.total_channel >= 0 && config_.total_channel < kExternalAdcChannelCount) {
            sample->total_valid = true;
            sample->total_voltage = corrected[config_.total_channel] * kExternalAdcVref / kExternalAdcResolution;
        }

        for (int ch = 0; ch < PWM_CHANNEL_COUNT; ++ch) {
            int adc_idx = config_.channel_map[ch];
            if (adc_idx >= 0 && adc_idx < kExternalAdcChannelCount) {
                sample->channel_valid[ch] = true;
                sample->channel_voltage[ch] = corrected[adc_idx] * kExternalAdcVref / kExternalAdcResolution;
            }
        }

        return ESP_OK;
    }

private:
    CurrentSensorConfig config_;
    spi_device_handle_t device_ = nullptr;
};

class Esp32InternalAdcDriver final : public ICurrentSensorDriver {
public:
    esp_err_t init(const CurrentSensorConfig &config) override {
        config_ = config;
        adc_oneshot_unit_init_cfg_t unit_cfg = {};
        unit_cfg.unit_id = ADC_UNIT_1;
        unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
        esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &unit_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create ADC unit: %s", esp_err_to_name(err));
            return err;
        }

        adc_oneshot_chan_cfg_t chan_cfg = {};
        chan_cfg.atten = ADC_ATTEN_DB_12;
        chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;

        if (config_.total_present && config_.total_channel >= 0) {
            adc_oneshot_config_channel(unit_, static_cast<adc_channel_t>(config_.total_channel), &chan_cfg);
        }

        for (int ch = 0; ch < PWM_CHANNEL_COUNT; ++ch) {
            if (config_.channel_map[ch] >= 0) {
                adc_oneshot_config_channel(unit_, static_cast<adc_channel_t>(config_.channel_map[ch]), &chan_cfg);
            }
        }
        return ESP_OK;
    }

    esp_err_t read_raw(RawVoltageSample *sample) override {
        if (!sample || !unit_) {
            return ESP_ERR_INVALID_STATE;
        }

        sample->total_valid = false;
        for (int i = 0; i < PWM_CHANNEL_COUNT; ++i) {
            sample->channel_valid[i] = false;
            sample->channel_voltage[i] = 0.0f;
        }

        int raw = 0;
        if (config_.total_present && config_.total_channel >= 0) {
            if (adc_oneshot_read(unit_, static_cast<adc_channel_t>(config_.total_channel), &raw) == ESP_OK) {
                sample->total_valid = true;
                sample->total_voltage = static_cast<float>(raw) * kInternalAdcVref / kInternalAdcResolution;
            }
        }

        for (int ch = 0; ch < PWM_CHANNEL_COUNT; ++ch) {
            if (config_.channel_map[ch] >= 0) {
                if (adc_oneshot_read(unit_, static_cast<adc_channel_t>(config_.channel_map[ch]), &raw) == ESP_OK) {
                    sample->channel_valid[ch] = true;
                    sample->channel_voltage[ch] = static_cast<float>(raw) * kInternalAdcVref / kInternalAdcResolution;
                }
            }
        }
        return ESP_OK;
    }

private:
    CurrentSensorConfig config_;
    adc_oneshot_unit_handle_t unit_ = nullptr;
};

class CurrentSensorManager {
public:
    esp_err_t init() {
        if (!mutex_) {
            mutex_ = xSemaphoreCreateMutex();
            if (!mutex_) {
                return ESP_ERR_NO_MEM;
            }
        }

        config_.total_present = HAS_TOTAL_CURRENT_SENSOR();
        config_.total_channel = config_.total_present ? TOTAL_CURRENT_CH : -1;
        config_.channel_count = 0;
        for (int ch = 0; ch < PWM_CHANNEL_COUNT; ++ch) {
            config_.channel_map[ch] = CHANNEL_CURRENT_ADC_CHS[ch];
            if (config_.channel_map[ch] >= 0) {
                config_.channel_count++;
            }
        }

        calibration_.acs758_offset = TOTAL_CURRENT_OFFSET;
        calibration_.acs758_sensitivity = TOTAL_CURRENT_SENSITIVITY;
        calibration_.acs712_offset = CHANNEL_CURRENT_OFFSET;
        calibration_.acs712_sensitivity = CHANNEL_CURRENT_SENSITIVITY;

        reset_latest();
        driver_ = create_driver();
        if (!driver_) {
            return ESP_ERR_NOT_SUPPORTED;
        }
        return driver_->init(config_);
    }

    esp_err_t start(uint32_t interval_ms) {
        if (task_handle_) {
            return ESP_ERR_INVALID_STATE;
        }
        interval_ms_ = std::max<uint32_t>(interval_ms, 50);
        BaseType_t rc = xTaskCreatePinnedToCore(&CurrentSensorManager::task_entry, "current_sensor", 4096, this, 5, &task_handle_, 0);
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
        *out = latest_;
        xSemaphoreGive(mutex_);
        return ESP_OK;
    }

    esp_err_t set_calibration(const current_sensor_calibration_t *cal) {
        if (!cal) {
            return ESP_ERR_INVALID_ARG;
        }
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        calibration_ = *cal;
        xSemaphoreGive(mutex_);
        return ESP_OK;
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
        if (!driver_) {
            return ESP_ERR_INVALID_STATE;
        }
        const uint16_t samples = std::max<uint16_t>(sample_count, 10);
        float total_sum = 0.0f;
        uint32_t total_samples = 0;
        float channel_sum[PWM_CHANNEL_COUNT] = {0};
        uint32_t channel_samples[PWM_CHANNEL_COUNT] = {0};

        RawVoltageSample sample;
        for (uint16_t i = 0; i < samples; ++i) {
            if (driver_->read_raw(&sample) != ESP_OK) {
                continue;
            }
            if (sample.total_valid) {
                total_sum += sample.total_voltage;
                ++total_samples;
            }
            for (int ch = 0; ch < PWM_CHANNEL_COUNT; ++ch) {
                if (sample.channel_valid[ch]) {
                    channel_sum[ch] += sample.channel_voltage[ch];
                    channel_samples[ch]++;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        current_sensor_calibration_t new_cal = calibration_;
        if (total_samples > 0) {
            new_cal.acs758_offset = total_sum / total_samples;
        }
        uint32_t valid_channels = 0;
        float avg = 0.0f;
        for (int ch = 0; ch < PWM_CHANNEL_COUNT; ++ch) {
            if (channel_samples[ch] > 0) {
                avg += channel_sum[ch] / channel_samples[ch];
                valid_channels++;
            }
        }
        if (valid_channels > 0) {
            new_cal.acs712_offset = avg / valid_channels;
        }
        return set_calibration(&new_cal);
    }

    esp_err_t ble_read(uint8_t *buffer, size_t size) {
        if (size < 36) {
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

        auto append_temp = [&buffer, &offset](temp_sensor_type_t sensor) {
            int16_t temp;
            if (temp_mgr_sample_once(sensor, &temp) == ESP_OK) {
                buffer[offset++] = (temp >> 8) & 0xFF;
                buffer[offset++] = temp & 0xFF;
            } else {
                buffer[offset++] = 0x80;
                buffer[offset++] = 0x00;
            }
        };
        append_temp(TEMP_SENSOR_POWER);
        append_temp(TEMP_SENSOR_CONTROL);

        current_sensor_data_t data;
        if (get_latest(&data) == ESP_OK) {
            union { float f; uint32_t u; } conv;
            conv.f = data.total_input_current;
            buffer[offset++] = (conv.u >> 24) & 0xFF;
            buffer[offset++] = (conv.u >> 16) & 0xFF;
            buffer[offset++] = (conv.u >> 8) & 0xFF;
            buffer[offset++] = conv.u & 0xFF;
            for (int i = 0; i < PWM_CHANNEL_COUNT; ++i) {
                conv.f = data.channel_currents[i];
                buffer[offset++] = (conv.u >> 24) & 0xFF;
                buffer[offset++] = (conv.u >> 16) & 0xFF;
                buffer[offset++] = (conv.u >> 8) & 0xFF;
                buffer[offset++] = conv.u & 0xFF;
            }
        } else {
            for (int i = 0; i < 28; ++i) {
                buffer[offset++] = 0xFF;
            }
        }

        uint8_t status = 0;
        if (power_mgr_is_thermal_protection_active()) {
            status |= 0x01;
        }
        int16_t t1, t2;
        if (temp_mgr_sample_once(TEMP_SENSOR_POWER, &t1) == ESP_OK &&
            temp_mgr_sample_once(TEMP_SENSOR_CONTROL, &t2) == ESP_OK) {
            status |= 0x02;
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
        return (offset == 36) ? ESP_OK : ESP_FAIL;
    }

private:
    static void task_entry(void *arg) {
        auto *self = reinterpret_cast<CurrentSensorManager*>(arg);
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

    void update_latest(const RawVoltageSample &sample) {
        current_sensor_data_t data;
        data.total_input_current = -1.0f;
        data.valid_mask = 0;
        data.timestamp_ms = esp_timer_get_time() / 1000;
        for (float &value : data.channel_currents) {
            value = -1.0f;
        }

        if (sample.total_valid && calibration_.acs758_sensitivity > 0.0f) {
            data.total_input_current = (sample.total_voltage - calibration_.acs758_offset) /
                                       calibration_.acs758_sensitivity;
            data.valid_mask |= 0x01;
        }

        for (int ch = 0; ch < PWM_CHANNEL_COUNT; ++ch) {
            if (sample.channel_valid[ch] && calibration_.acs712_sensitivity > 0.0f) {
                data.channel_currents[ch] = (sample.channel_voltage[ch] - calibration_.acs712_offset) /
                                             calibration_.acs712_sensitivity;
                data.valid_mask |= (1u << (ch + 1));
            } else {
                data.channel_currents[ch] = -1.0f;
            }
        }

        if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
            latest_ = data;
            xSemaphoreGive(mutex_);
        }
        notify_ble();
    }

    std::unique_ptr<ICurrentSensorDriver> create_driver() {
#if defined(EXTERNAL_ADC)
        return std::unique_ptr<ICurrentSensorDriver>(new Adc128s102Driver());
#elif defined(BUILTIN_ADC)
        return std::unique_ptr<ICurrentSensorDriver>(new Esp32InternalAdcDriver());
#else
        return nullptr;
#endif
    }

    void reset_latest() {
        latest_.total_input_current = -1.0f;
        for (float &value : latest_.channel_currents) {
            value = -1.0f;
        }
        latest_.valid_mask = 0;
        latest_.timestamp_ms = 0;
    }

    std::unique_ptr<ICurrentSensorDriver> driver_;
    current_sensor_calibration_t calibration_ = {};
    current_sensor_data_t latest_ = {};
    CurrentSensorConfig config_;
    SemaphoreHandle_t mutex_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
    uint32_t interval_ms_ = 1000;

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
};

CurrentSensorManager &get_manager() {
    static CurrentSensorManager manager;
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

int current_sensor_ble_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    uint8_t payload[36];
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
