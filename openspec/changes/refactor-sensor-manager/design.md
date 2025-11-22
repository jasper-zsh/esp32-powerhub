# Design: Sensor Manager Refactor

## Context
The ESP32 power hub currently uses a monolithic current sensor implementation that mixes hardware-specific ADC code with business logic for voltage-to-current conversion. This creates tight coupling and makes it difficult to support different ADC hardware or add new sensor types.

Current architecture:
- ADC128S102 and ESP32 internal ADC drivers directly implement current sensor logic
- Voltage-to-current conversion formulas embedded in driver classes
- Fixed 6-channel configuration regardless of actual hardware
- No clear separation between voltage measurement and value calculation
- Hardware definitions scattered between compile-time macros and runtime code

## Hardware Definition Strategy

### Current Hardware Configuration
```cpp
// From hardware_defs.h
#define PWM_CHANNEL_COUNT 6
#define EXTERNAL_ADC_CS_PIN 3
#define EXTERNAL_ADC_SCLK_PIN 4
#define EXTERNAL_ADC_MISO_PIN 5
#define EXTERNAL_ADC_MOSI_PIN 6

// Current sensor channel mapping
#define TOTAL_CURRENT_CH 0
extern const int CHANNEL_CURRENT_ADC_CHS[PWM_CHANNEL_COUNT]; // [-1, 0-7]
#define HAS_TOTAL_CURRENT_SENSOR() (TOTAL_CURRENT_CH >= 0)
#define HAS_CHANNEL_CURRENT_SENSOR(ch) (CHANNEL_CURRENT_ADC_CHS[ch] >= 0)
```

### Proposed Hardware Definition Passing

#### 1. Current Sensor Parameters Structure
```cpp
typedef struct {
    float offset;        // Zero-point voltage (V)
    float sensitivity;   // V/A sensitivity
    int8_t adc_channel;  // ADC channel (-1 = disabled, 0-7 = ADC channel)
} current_sensor_params_t;
```

#### 2. Dynamic Sensor Configuration Structure
```cpp
typedef struct {
    // Total current sensor (optional)
    current_sensor_params_t total_current;

    // Channel current sensors (dynamic count)
    current_sensor_params_t* channel_current;  // Array of channel sensors
    uint8_t channel_count;                     // Number of configured channels (1-6)

    // General sensor configuration
    uint32_t sample_interval_ms;  // Sampling interval
} sensor_config_t;
```

#### 3. ADC Driver-Specific Configuration
```cpp
// ADC128S102 specific configuration
typedef struct {
    uint8_t cs_pin;        // GPIO pin for chip select
    uint8_t sclk_pin;      // GPIO pin for serial clock
    uint8_t miso_pin;      // GPIO pin for MISO
    uint8_t mosi_pin;      // GPIO pin for MOSI
    uint32_t clock_speed;  // SPI clock frequency
    bool use_dma;          // Whether to use DMA
} adc128s102_config_t;

// ESP32 Internal ADC specific configuration
typedef struct {
    adc_atten_t attenuation;   // ADC attenuation
    adc_bitwidth_t bitwidth;   // ADC resolution
} esp32_adc_config_t;
```

#### 6. ADC Driver Internal Power Voltage Implementation

##### ADC128S102 Driver (EXTERNAL_ADC)
```cpp
class Adc128s102Driver final : public VoltageSensorDriver {
public:
    esp_err_t read_raw(RawVoltageSample *sample) override {
        // Read current sensor channels from external ADC
        // Power voltage is NOT read from external ADC
        read_external_adc_channels(sample);

        // Power voltage is handled internally by ULP, not returned here
        sample->power_voltage_valid = false;

        return ESP_OK;
    }

private:
    void read_external_adc_channels(RawVoltageSample *sample) {
        // Read channels 0-7 from ADC128S102 via SPI
        // Only current sensors are connected to external ADC
    }
};
```

##### ESP32 Internal ADC Driver (BUILTIN_ADC)
```cpp
class Esp32InternalAdcDriver final : public VoltageSensorDriver {
public:
    esp_err_t init(const esp32_adc_config_t& config) override {
        config_ = config;

        // Create ADC unit handle (unit determined by GPIO pins automatically)
        adc_oneshot_unit_init_cfg_t unit_cfg = {};
        unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
        esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &unit_);
        if (err != ESP_OK) {
            return err;
        }

        // Configure common ADC parameters
        adc_oneshot_chan_cfg_t chan_cfg = {};
        chan_cfg.bitwidth = config_.bitwidth;
        chan_cfg.atten = config_.attenuation;

        // Configure all required ADC channels (current sensors + power voltage)
        adc_oneshot_config_channel(unit_, ADC_CHANNEL_1, &chan_cfg);  // Power voltage on GPIO1
        for (const auto& sensor : sensor_configs_) {
            if (sensor.adc_channel >= 0 && sensor.adc_channel < ADC_CHANNEL_MAX) {
                adc_oneshot_config_channel(unit_, static_cast<adc_channel_t>(sensor.adc_channel), &chan_cfg);
            }
        }

        return ESP_OK;
    }

    esp_err_t read_raw(RawVoltageSample *sample) override {
        // Read current sensor channels
        read_current_sensor_channels(sample);

        // Read power voltage from GPIO1 (channel 1) - driver internal implementation
        read_power_voltage(sample);

        return ESP_OK;
    }

private:
    void read_power_voltage(RawVoltageSample *sample) {
        // Read power voltage from GPIO1 (ADC_CHANNEL_1)
        int raw = 0;
        if (adc_oneshot_read(unit_, ADC_CHANNEL_1, &raw) == ESP_OK) {
            sample->power_voltage = raw * kInternalAdcVref / kInternalAdcResolution;
            sample->power_voltage_valid = true;
        } else {
            sample->power_voltage_valid = false;
        }
    }

    void read_current_sensor_channels(RawVoltageSample *sample) {
        // Read current sensors from configured ADC channels
        // ADC unit is automatically selected based on GPIO pin routing
        for (size_t i = 0; i < sensor_configs_.size(); ++i) {
            const auto& sensor = sensor_configs_[i];
            if (sensor.adc_channel >= 0 && sensor.adc_channel < ADC_CHANNEL_MAX) {
                int raw = 0;
                if (adc_oneshot_read(unit_, static_cast<adc_channel_t>(sensor.adc_channel), &raw) == ESP_OK) {
                    sample->channel_voltage[i] = raw * kInternalAdcVref / kInternalAdcResolution;
                    sample->channel_valid[i] = true;
                } else {
                    sample->channel_valid[i] = false;
                }
            }
        }
    }

    esp32_adc_config_t config_;
    adc_oneshot_unit_handle_t unit_;
    std::vector<current_sensor_params_t> sensor_configs_;
};
```

#### 4. Hardware Definition Resolution
```cpp
// Runtime configuration builder
sensor_config_t build_sensor_config_from_hardware_defs() {
    sensor_config_t config = {};

    // Count actual configured channels
    uint8_t actual_channel_count = 0;
    for (int i = 0; i < PWM_CHANNEL_COUNT; i++) {
        if (HAS_CHANNEL_CURRENT_SENSOR(i)) {
            actual_channel_count++;
        }
    }

    // Allocate dynamic array for channel sensors
    config.channel_current = (current_sensor_params_t*)malloc(actual_channel_count * sizeof(current_sensor_params_t));
    config.channel_count = actual_channel_count;

    // Power voltage configuration is handled internally by ADC drivers
    // No power voltage configuration needed at this level

    // Configure total current sensor (optional)
    config.total_current.adc_channel = HAS_TOTAL_CURRENT_SENSOR() ? TOTAL_CURRENT_CH : -1;
    config.total_current.offset = TOTAL_CURRENT_OFFSET;
    config.total_current.sensitivity = TOTAL_CURRENT_SENSITIVITY;

    // Configure channel current sensors (dynamic)
    int channel_index = 0;
    for (int i = 0; i < PWM_CHANNEL_COUNT; i++) {
        if (HAS_CHANNEL_CURRENT_SENSOR(i)) {
            config.channel_current[channel_index].adc_channel = CHANNEL_CURRENT_ADC_CHS[i];
            config.channel_current[channel_index].offset = CHANNEL_CURRENT_OFFSET;
            config.channel_current[channel_index].sensitivity = CHANNEL_CURRENT_SENSITIVITY;
            channel_index++;
        }
    }

    // Sample interval
    config.sample_interval_ms = 100;

    return config;
}

void free_sensor_config(sensor_config_t* config) {
    if (config && config->channel_current) {
        free(config->channel_current);
        config->channel_current = NULL;
        config->channel_count = 0;
    }
}
```

### ADC Channel to Sensor Value Mapping Strategy

#### 1. Dynamic Channel Mapping Table
```cpp
typedef struct {
    float voltage;         // Measured voltage (V)
    float current;         // Converted current (A)
    uint32_t timestamp;    // Measurement timestamp (ms)
    bool is_valid;         // Measurement validity flag
} current_measurement_t;

typedef struct {
    float voltage;         // Measured voltage (V)
    float actual_voltage;  // Voltage after divider scaling (V)
    uint32_t timestamp;    // Measurement timestamp (ms)
    bool is_valid;         // Measurement validity flag
} voltage_measurement_t;

typedef struct {
    // Total current (optional)
    current_measurement_t total_current;

    // Channel currents (dynamic count)
    current_measurement_t* channel_currents;  // Dynamic array
    uint8_t channel_count;                    // Number of channels

    // Power voltage (separate ADC)
    voltage_measurement_t power_voltage;

    // Validity flags
    bool total_current_valid;
    bool power_voltage_valid;
    bool* channel_current_valid;  // Dynamic array
} sensor_readings_t;
```

#### 2. Dynamic Channel Resolution Process
```cpp
class SensorManager {
private:
    sensor_config_t config_;
    sensor_readings_t latest_readings_;

    void initialize_readings() {
        // Allocate dynamic arrays
        latest_readings_.channel_currents = (current_measurement_t*)malloc(config_.channel_count * sizeof(current_measurement_t));
        latest_readings_.channel_current_valid = (bool*)malloc(config_.channel_count * sizeof(bool));

        // Initialize arrays
        for (int i = 0; i < config_.channel_count; i++) {
            latest_readings_.channel_currents[i].is_valid = false;
            latest_readings_.channel_currents[i].current = -1.0f;
            latest_readings_.channel_current_valid[i] = false;
        }

        latest_readings_.total_current_valid = false;
        latest_readings_.power_voltage_valid = false;
    }

    void resolve_adc_channels_to_sensors(const RawVoltageSample& adc_sample) {
        uint32_t timestamp = esp_timer_get_time() / 1000;

        // Map total current
        if (config_.total_current.adc_channel >= 0) {
            int8_t adc_ch = config_.total_current.adc_channel;
            if (adc_ch < 8 && adc_sample.channel_valid[adc_ch]) {
                latest_readings_.total_current.voltage = adc_sample.channel_voltage[adc_ch];
                latest_readings_.total_current.current = convert_voltage_to_current(
                    latest_readings_.total_current.voltage,
                    config_.total_current.offset,
                    config_.total_current.sensitivity
                );
                latest_readings_.total_current.timestamp = timestamp;
                latest_readings_.total_current_valid = true;
            } else {
                latest_readings_.total_current_valid = false;
            }
        }

        // Map channel currents (dynamic)
        for (int i = 0; i < config_.channel_count; i++) {
            const current_sensor_params_t* sensor = &config_.channel_current[i];
            if (sensor->adc_channel >= 0 && sensor->adc_channel < 8) {
                if (adc_sample.channel_valid[sensor->adc_channel]) {
                    latest_readings_.channel_currents[i].voltage = adc_sample.channel_voltage[sensor->adc_channel];
                    latest_readings_.channel_currents[i].current = convert_voltage_to_current(
                        latest_readings_.channel_currents[i].voltage,
                        sensor->offset,
                        sensor->sensitivity
                    );
                    latest_readings_.channel_currents[i].timestamp = timestamp;
                    latest_readings_.channel_current_valid[i] = true;
                } else {
                    latest_readings_.channel_current_valid[i] = false;
                }
            } else {
                latest_readings_.channel_current_valid[i] = false;
            }
        }

        // Map power voltage (separate handling)
        // This would use the internal ADC on GPIO1 with voltage divider
        latest_readings_.power_voltage_valid = false;  // To be implemented
    }

    float convert_voltage_to_current(float voltage, float offset, float sensitivity) {
        if (sensitivity <= 0.0f) return -1.0f;
        return (voltage - offset) / sensitivity;
    }
};
```

#### 3. Dynamic Channel Discovery
```cpp
// Runtime channel validation
void validate_sensor_configuration(const sensor_config_t& config) {
    ESP_LOGI(TAG, "=== Sensor Configuration ===");

    // Validate power voltage
    ESP_LOGI(TAG, "Power voltage: %s ADC, divider_ratio=%.3f, reference=%.1fV",
            config.power_voltage.use_internal_adc ? "Internal" : "External",
            config.power_voltage.divider_ratio,
            config.power_voltage.reference_voltage);

    // Validate total current
    if (config.total_current.is_valid) {
        ESP_LOGI(TAG, "Total current: ADC channel %d (offset=%.3fV, sensitivity=%.3fV/A)",
                config.total_current.adc_channel,
                config.total_current.offset,
                config.total_current.sensitivity);
    } else {
        ESP_LOGI(TAG, "Total current: DISABLED");
    }

    // Validate channel currents (dynamic)
    ESP_LOGI(TAG, "Channel currents: %d sensors configured", config.channel_count);
    for (int i = 0; i < config.channel_count; i++) {
        const current_sensor_params_t* sensor = &config.channel_current[i];
        if (sensor->is_valid) {
            ESP_LOGI(TAG, "  CH%d: ADC channel %d (offset=%.3fV, sensitivity=%.3fV/A)",
                    i + 1, sensor->adc_channel, sensor->offset, sensor->sensitivity);
        }
    }
}
```

## Goals / Non-Goals
- Goals: Clean separation of concerns, configurable channel count, optional total current
- Goals: Unified voltage-to-value conversion in parent class
- Goals: Preserve existing API compatibility
- Non-Goals: Support for other sensor types (temperature, etc.)
- Non-Goals: Dynamic runtime sensor configuration changes
- Non-Goals: Real-time performance optimization beyond current capabilities

## Decisions
- Decision: Create base `VoltageSensorDriver` class that only returns voltage values
- Decision: Create `SensorManager` parent class with voltage-to-current conversion logic
- Decision: ADC drivers inherit from `VoltageSensorDriver` and only handle hardware communication
- Decision: Configuration-based channel mapping (1-6 channels, optional total current)
- Decision: Preserve existing `current_sensor.h` C API for compatibility

Alternatives considered:
- Single unified sensor class for all measurement types → Too complex, mixes concerns
- Template-based approach → Over-engineering for limited use cases
- Runtime sensor discovery → Not needed for this embedded system

## Risks / Trade-offs
- Risk: API compatibility breakage → Mitigation: Keep existing C wrapper functions
- Risk: Performance overhead from abstraction → Mitigation: Minimal inheritance depth
- Trade-off: Flexibility vs simplicity → Leaning toward simplicity for embedded context

## Migration Plan
1. Create new base classes while preserving existing implementation
2. Migrate ADC drivers to new base class structure
3. Create unified SensorManager with parent class logic
4. Update C wrapper functions to use new implementation
5. Remove old code after validation
6. Update system monitor integration (if needed)

## Open Questions
- Should voltage-to-current calibration be per-channel or global? → Current: per sensor type (total vs channel)
- How to handle missing channels in the data structure? → Current: -1.0f sentinel values