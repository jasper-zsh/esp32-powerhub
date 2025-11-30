## ADDED Requirements

### Requirement: DS18B20 Temperature Sensor Integration
The system SHALL provide standardized temperature sensing using the official Espressif DS18B20 library with 1-Wire RMT communication.

#### Scenario: Initialize DS18B20 bus and discover sensors
- **WHEN** the system starts up
- **THEN** it SHALL initialize a 1-Wire bus on GPIO7 with internal pull-up enabled
- **AND** it SHALL automatically discover all connected DS18B20 sensors
- **AND** it SHALL log the addresses of discovered sensors
- **AND** it SHALL support up to 2 sensors (power area and control area)

#### Scenario: Read temperature from multiple sensors
- **WHEN** temperature readings are requested
- **THEN** the system SHALL trigger temperature conversion for all sensors simultaneously
- **AND** it SHALL read temperatures from each discovered sensor individually
- **AND** it SHALL return temperature values in Celsius with 2 decimal precision
- **AND** it SHALL handle CRC validation errors gracefully

#### Scenario: Handle sensor communication errors
- **WHEN** a DS18B20 sensor communication fails
- **THEN** the system SHALL log appropriate error messages
- **AND** it SHALL continue reading from other functional sensors
- **AND** it SHALL implement retry logic for failed readings
- **AND** it SHALL report sensor status to the monitoring system

### Requirement: Temperature Data Access API
The system SHALL provide a non-blocking API for accessing cached temperature data from DS18B20 sensors.

#### Scenario: Get latest temperature readings
- **WHEN** the application requests temperature data
- **THEN** it SHALL return the most recent cached temperature values
- **AND** it SHALL include sensor status (healthy/error) for each sensor
- **AND** it SHALL provide timestamp of last successful reading
- **AND** it SHALL return ESP_OK if data is available, error code otherwise

#### Scenario: Background temperature sampling
- **WHEN** the system is running
- **THEN** it SHALL continuously sample temperatures from all sensors in the background
- **AND** it SHALL update cached data at configurable intervals (default 5000ms)
- **AND** it SHALL trigger immediate sampling when requested
- **AND** it SHALL use FreeRTOS tasks for non-blocking operation

### Requirement: Temperature-Based Protection
The system SHALL use DS18B20 temperature readings for thermal protection of the power hub.

#### Scenario: Temperature threshold protection
- **WHEN** sensor temperatures exceed safe operating limits
- **THEN** the system SHALL trigger protective shutdown procedures
- **AND** it SHALL log temperature threshold violations
- **AND** it SHALL provide hysteresis to prevent rapid cycling
- **AND** it SHALL report thermal events to the monitoring system

#### Scenario: Temperature monitoring integration
- **WHEN** temperature events occur
- **THEN** the system SHALL notify the system monitoring task
- **AND** it SHALL include temperature values and sensor identification
- **AND** it SHALL integrate with the existing FreeRTOS event framework
- **AND** it SHALL maintain compatibility with current monitoring dashboards