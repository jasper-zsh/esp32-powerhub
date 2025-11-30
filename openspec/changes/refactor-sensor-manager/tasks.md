## 1. Create Base Architecture
- [x] 1.1 Create `VoltageSensorDriver` base class with pure voltage measurement interface
- [x] 1.2 Create `SensorManager` parent class with voltage-to-current conversion logic
- [x] 1.3 Define simplified current_sensor_params_t structure (remove is_valid field)
- [x] 1.4 Define dynamic sensor_config_t structure with channel_count and dynamic arrays
- [x] 1.5 Create current_measurement_t and voltage_measurement_t structures
- [x] 1.6 Create dynamic sensor_readings_t structure with allocated arrays
- [x] 1.7 Add ADC driver-specific configuration structures (adc128s102_config_t, esp32_adc_config_t without unit field)
- [x] 1.8 Add hardware definition resolution functions with dynamic allocation
- [x] 1.9 Add calibration management methods to SensorManager with per-sensor parameters
- [x] 1.10 Update sensor validity logic to use adc_channel >= 0 instead of is_valid field
- [x] 1.11 Update ESP32 ADC driver to use GPIO-based ADC unit determination

## 2. Refactor ADC Drivers
- [x] 2.1 Update `Adc128s102Driver` to inherit from `VoltageSensorDriver`
- [x] 2.2 Update `Esp32InternalAdcDriver` to inherit from `VoltageSensorDriver`
- [x] 2.3 Remove voltage-to-current conversion logic from ADC drivers
- [x] 2.4 Add hardware configuration injection via ADC-specific config structures
- [x] 2.5 Add voltage-only measurement methods to ADC drivers
- [x] 2.6 Maintain hardware-specific SPI and ADC initialization code
- [x] 2.7 Add hardware validation functions for ADC-specific parameters
- [x] 2.8 Remove common ADC hardware parameters from shared configuration
- [x] 2.9 Validate ESP32 ADC unit determination from GPIO pins in driver initialization

## 3. Implement Unified SensorManager
- [x] 3.1 Create new `SensorManager` using parent class architecture
- [x] 3.2 Implement dynamic channel allocation based on channel_count
- [x] 3.3 Add optional total current sensor support with simplified current_sensor_params_t
- [x] 3.4 Implement ADC channel to sensor mapping using dynamic arrays
- [x] 3.5 Implement voltage-to-current conversion with per-sensor calibration
- [x] 3.6 Add calibration management using current_sensor_params_t structures
- [x] 3.7 Add memory management for dynamic array allocation and deallocation
- [x] 3.8 Implement hardware configuration validation at startup
- [x] 3.9 Implement power voltage reading inside Adc128s102Driver (ULP integration)
- [x] 3.10 Implement power voltage reading inside Esp32InternalAdcDriver (GPIO1 direct read)
- [x] 3.11 Remove power voltage configuration from common sensor_config_t structure
- [x] 3.12 Ensure power voltage interface is consistent across both ADC drivers

## 4. Update API Compatibility Layer
- [x] 4.1 Preserve existing `current_sensor.h` C API functions
- [x] 4.2 Update C wrapper functions to use new SensorManager implementation
- [x] 4.3 Ensure `current_sensor_data_t` structure maintains compatibility
- [x] 4.4 Add dynamic configuration passing functions for C API
- [x] 4.5 Add memory management wrapper functions for C API compatibility
- [x] 4.6 Validate BLE interface continues to work with new implementation

## 5. Update System Integration
- [x] 5.1 Update `system_monitor.c` to work with new SensorManager (if needed)
- [x] 5.2 Validate power voltage monitoring separation from current sensing
- [x] 5.3 Test BLE notification system with new SensorManager
- [x] 5.4 Ensure calibration zero-point procedure works with new implementation

## 6. Testing and Validation
- [x] 6.1 Test ADC128S102 driver returns pure voltage measurements
- [x] 6.2 Test variable channel configurations (1, 3, 6 channels)
- [x] 6.3 Test optional total current sensor enable/disable
- [x] 6.4 Validate dynamic configuration building from hardware_defs.h
- [x] 6.5 Test ADC channel to sensor mapping with dynamic arrays
- [x] 6.6 Test variable channel count allocation (1, 3, 6 channels)
- [x] 6.7 Test memory management for dynamic allocation/deallocation
- [x] 6.8 Test simplified current_sensor_params_t per-sensor calibration
- [x] 6.9 Test sensor validity logic using adc_channel >= 0 instead of is_valid field
- [x] 6.10 Test power voltage interface consistency across both ADC drivers
- [x] 6.11 Validate Adc128s102Driver does not read power voltage from external ADC
- [x] 6.12 Validate Esp32InternalAdcDriver reads power voltage from GPIO1 directly
- [x] 6.13 Test ULP integration for power voltage when using external ADC driver
- [x] 6.14 Validate ESP32 ADC unit determination from GPIO routing
- [x] 6.15 Validate voltage-to-current conversion accuracy
- [x] 6.16 Test calibration procedures for both sensor types
- [x] 6.17 Verify existing C API compatibility
- [x] 6.18 Test BLE interface with sensor data
- [x] 6.19 Validate system monitor integration
- [x] 6.20 Test memory leak prevention with repeated allocation/deallocation

## 7. Documentation and Cleanup
- [x] 7.1 Update hardware_defs.h documentation for new SensorManager architecture
- [ ] 7.2 Remove old current sensor implementation code
- [ ] 7.3 Update inline code documentation for dynamic memory management
- [ ] 7.4 Add memory management safety documentation
- [x] 7.5 Validate build system compatibility with new structure
