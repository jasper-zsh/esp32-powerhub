## ADDED Requirements

### Requirement: ADC Channel to Sensor Mapping
The system SHALL map physical ADC channels to logical sensor values through dynamic channel mapping structures.

#### Scenario: Dynamic channel configuration
- **WHEN** the SensorManager initializes
- **THEN** it allocates dynamic arrays based on actual configured sensor count
- **AND** maps each current sensor to its ADC channel using current_sensor_params_t structures
- **AND** uses adc_channel >= 0 to indicate enabled sensors, adc_channel = -1 to indicate disabled sensors

#### Scenario: Variable channel count support
- **WHEN** hardware configuration changes from 3 to 6 current sensors
- **THEN** the system reallocates dynamic arrays for new sensor count
- **AND** updates channel_current array size accordingly
- **AND** maintains measurement accuracy during reallocation

#### Scenario: Flexible ADC channel assignment
- **WHEN** hardware uses non-standard ADC channel assignments
- **THEN** the system supports any ADC channel 0-7 for any current sensor
- **AND** validates ADC channel ranges (0-7, -1 for disabled)
- **AND** prevents duplicate ADC channel assignments unless hardware requires

### Requirement: Configurable Channel Current Monitoring
The system SHALL support 1-6 configurable current channels with optional total current measurement using SensorManager voltage-to-current conversion logic.

#### Scenario: Variable channel configuration
- **WHEN** the system initializes with 3 current sensors
- **THEN** channel_count field equals 3 and only 3 measurement entries are allocated
- **AND** measurement arrays are sized exactly for the configured sensors
- **AND** the system adapts to the actual sensor count automatically
- **AND** dynamic memory allocation matches actual sensor count

#### Scenario: Optional total current
- **WHEN** hardware configuration has no total current sensor
- **THEN** total_current.adc_channel is set to -1
- **AND** total_current_valid flag reflects disabled state
- **AND** no ADC sampling is performed for total current channel

#### Scenario: Voltage-to-current conversion in SensorManager
- **WHEN** raw voltage measurements are available from ADC drivers
- **THEN** the SensorManager applies calibration from current_sensor_params_t structures
- **AND** converts voltage to current using per-sensor offset and sensitivity values
- **AND** provides current measurements in Amperes with 0.001A precision
- **AND** skips conversion for sensors with adc_channel = -1

#### Scenario: Per-channel sensor validation
- **WHEN** reading current measurements
- **THEN** each sensor's adc_channel >= 0 determines conversion attempts
- **AND** disabled sensors (adc_channel = -1) return -1.0f sentinel values
- **AND** valid sensors but failed ADC readings are marked invalid appropriately
- **AND** channel_current_valid array accurately tracks measurement validity

### Requirement: Runtime Channel Reconfiguration
The system SHALL support runtime changes to channel mapping and sensor configuration.

#### Scenario: Hot-swapping sensor configuration
- **WHEN** system configuration changes during operation
- **THEN** the SensorManager validates new channel mapping
- **AND** updates internal channel resolution tables
- **AND** provides immediate feedback on configuration changes
- **AND** maintains sensor data continuity during transition

#### Scenario: Sensor type switching
- **WHEN** hardware supports different sensor types
- **THEN** the system applies appropriate calibration parameters
- **AND** switches between ACS712 and ACS758 conversion formulas
- **AND** maintains measurement accuracy across sensor type changes

### Requirement: Current Sensor Calibration
The system SHALL provide unified calibration management through the SensorManager with support for different sensor types.

#### Scenario: ACS758 total current calibration
- **WHEN** calibrating total current sensor (ACS758)
- **THEN** the system measures zero-current voltage offset
- **AND** applies 2.5V nominal offset with 0.020V/A sensitivity
- **AND** stores calibration for subsequent measurements

#### Scenario: ACS712 channel current calibration
- **WHEN** calibrating channel current sensors (ACS712)
- **THEN** the system measures zero-current voltage offset for all channels
- **AND** applies 2.5V nominal offset with 0.100V/A sensitivity
- **AND** uses average offset across all valid channels

#### Scenario: Runtime calibration updates
- **WHEN** calibration parameters are updated
- **THEN** the SensorManager immediately applies new values
- **AND** subsequent measurements use updated calibration
- **AND** existing measurements remain unchanged