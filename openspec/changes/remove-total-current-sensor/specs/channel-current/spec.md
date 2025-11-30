## ADDED Requirements

### Requirement: Channel-Only Current Sensing Architecture
The system SHALL provide current sensing exclusively through individual PWM channel sensors without total current aggregation.

#### Scenario: SensorManager initialization
- **WHEN** the SensorManager initializes
- **THEN** it configures only channel current sensors (CH1-CH6)
- **AND** allocates memory exclusively for channel sensor data
- **AND** processes ADC channels 0-5 for corresponding PWM channels CH6-CH1
- **AND** does not allocate or process total current sensor data

#### Scenario: C API data structure access
- **WHEN** reading current sensor data through the C API
- **THEN** the `current_sensor_data_t` structure contains only channel current values
- **AND** `total_input_current` field is removed from the structure
- **AND** `valid_mask` uses only bits 1-6 for CH1-CH6 channel validity
- **AND** bit0 is never set for total current validity

## MODIFIED Requirements

### Requirement: Simplified Current Sensor Data Structure
The system SHALL use a simplified current sensor data structure that excludes total current measurements.

#### Scenario: Application data access
- **WHEN** applications request current sensor data
- **THEN** the returned structure contains only channel current measurements
- **AND** channel current values are indexed [0-5] corresponding to CH1-CH6
- **AND** invalid channel measurements return -1.0f sentinel values
- **AND** timestamp reflects the time of channel current collection

#### Scenario: System monitor data processing
- **WHEN** system monitor processes current sensor data
- **THEN** it processes only channel current values
- **AND** updates `last_channel_currents` array with individual channel values
- **AND** performs threshold checking per channel independently
- **AND** does not reference or process total current values

## REMOVED Requirements

### Requirement: Total Current Sensing Functionality
The system SHALL NOT provide total current measurement functionality.

#### Scenario: Hardware configuration processing
- **WHEN** the hardware configuration is processed
- **THEN** the system does not scan for or configure total current sensors
- **AND** ADC channel allocation excludes total current sensor assignments
- **AND** `HAS_TOTAL_CURRENT_SENSOR()` macro and related definitions are removed
- **AND** no total current sensor configuration is stored or processed

#### Scenario: SensorManager ADC processing
- **WHEN** the SensorManager processes ADC samples
- **THEN** it converts only channel ADC readings to current values
- **AND** applies ACS712 calibration parameters for channel sensors
- **AND** does not apply ACS758 calibration or total current conversion
- **AND** returns only channel current measurements in sensor data structures