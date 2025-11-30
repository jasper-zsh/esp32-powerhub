## REMOVED Requirements

### Requirement: Total Current Sensor Hardware Definitions
The system SHALL NOT include hardware definitions or configuration parameters for total current sensors.

#### Scenario: Hardware definition processing
- **WHEN** processing hardware configuration definitions
- **THEN** `TOTAL_CURRENT_CH`, `TOTAL_CURRENT_OFFSET`, and `TOTAL_CURRENT_SENSITIVITY` definitions are removed
- **AND** `HAS_TOTAL_CURRENT_SENSOR()` macro and related helper functions are removed
- **AND** ADC channel mapping excludes any total current sensor assignments
- **AND** hardware validation does not check for total current sensor configuration

#### Scenario: Dynamic sensor configuration building
- **WHEN** building dynamic sensor configuration from hardware definitions
- **THEN** the configuration includes only channel current sensors
- **AND** no memory or processing is allocated for total current sensor parameters
- **AND** sensor validation does not reference total current sensor requirements
- **AND** hardware compatibility checks exclude total current sensor validation

## MODIFIED Requirements

### Requirement: Exclusive Channel Current Sensor Mapping
The system SHALL map external ADC channels exclusively to individual PWM channel current sensors.

#### Scenario: External ADC channel configuration
- **WHEN** the external ADC configuration is processed
- **THEN** ADC channels 0-5 are mapped to PWM channels CH6-CH1 respectively
- **AND** no ADC channel is reserved for total current measurement
- **AND** sensor configuration allocates exactly 6 channel current sensors
- **AND** each channel sensor uses ACS712 calibration parameters

#### Scenario: Runtime dynamic configuration
- **WHEN** dynamic sensor configuration is built at runtime
- **THEN** the configuration structure contains only channel current sensor parameters
- **AND** `channel_count` field is set to 6 for all enabled channels
- **AND** `total_current` field and related structures are not allocated
- **AND** memory usage matches exactly the number of configured channel sensors