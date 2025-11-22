## ADDED Requirements

### Requirement: ADC Driver-Internal Power Voltage Implementation
Power voltage reading SHALL be implemented as an internal feature of each ADC driver, not as a configurable option.

#### Scenario: ADC128S102 Driver Implementation
- **WHEN** using external ADC for current sensors
- **THEN** the ADC128S102 driver only reads current sensor channels (0-7)
- **AND** power voltage reading is handled by ULP coprocessor separately
- **AND** the driver returns power_voltage_valid = false for all samples
- **AND** power voltage is read from built-in ADC by ULP, not by external ADC

#### Scenario: ESP32 Built-in ADC Driver Implementation
- **WHEN** using built-in ADC for current sensors
- **THEN** the ESP32 ADC driver reads both current sensors and power voltage
- **AND** power voltage is read directly from GPIO1 (ADC channel 1)
- **AND** the driver returns power_voltage with proper divider scaling applied
- **AND** power voltage reading is transparent to upper layers

#### Scenario: Transparent Power Voltage Interface
- **WHEN** SensorManager receives data from any ADC driver
- **THEN** power voltage availability depends on driver implementation
- **AND** power_voltage field is populated when available
- **AND** upper layers use consistent interface regardless of ADC variant
- **AND** power voltage source selection is not configurable by application

#### Scenario: Hardware Validation at Startup
- **WHEN** the system starts
- **THEN** it validates ADC driver-specific hardware requirements
- **AND** checks GPIO pin assignments for SPI or ADC configurations
- **AND** reports hardware configuration errors before sensor operations begin

### Requirement: Power Voltage Monitoring
The system SHALL provide independent power voltage monitoring using ADC voltage measurements with fixed scaling for voltage divider circuits.

#### Scenario: Voltage divider scaling
- **WHEN** ADC hardware provides raw voltage measurements from GPIO1
- **THEN** the system calculates actual power voltage using fixed divider formula
- **AND** applies V_power = V_adc * (240k + 910k) / 910k scaling
- **AND** provides calibrated voltage readings for power management with 0.01V precision

#### Scenario: Consistent Interface Across ADC Variants
- **WHEN** SensorManager receives power voltage data from any ADC driver
- **THEN** the power_voltage field contains actual voltage after divider scaling
- **AND** power_voltage_valid indicates whether power voltage is available
- **AND** the interface is identical regardless of which ADC driver is used
- **AND** upper layers do not need to know the power voltage source

## MODIFIED Requirements

### Requirement: ADC Driver Interface
ADC drivers SHALL only handle voltage measurement operations without business logic for unit conversion.

#### Scenario: Pure voltage measurement
- **WHEN** an ADC driver reads a channel
- **THEN** it returns voltage in volts relative to ADC reference
- **AND** does not apply current sensor calibration or scaling
- **AND** provides timestamp and validity information for the measurement

#### Scenario: Hardware abstraction
- **WHEN** switching between external ADC128S102 and internal ESP32 ADC
- **THEN** both drivers return consistent voltage format
- **AND** hardware-specific details are encapsulated in driver implementation
- **AND** upper layers use identical interface