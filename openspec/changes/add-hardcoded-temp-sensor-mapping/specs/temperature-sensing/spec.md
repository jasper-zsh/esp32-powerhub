## MODIFIED Requirements

### Requirement: Temperature Sensor Area Assignment
The system SHALL provide deterministic assignment of DS18B20 temperature sensors to monitoring areas (power area, control area) with configurable address-based mapping and fallback to discovery order.

#### Scenario: Configured mapping with valid addresses
- **WHEN** temperature sensors are initialized with configured DS18B20 addresses
- **AND** sensors with matching addresses are connected
- **THEN** each monitoring area is assigned the sensor with the configured address
- **AND** temperature readings are consistently associated with the correct area across reboots

#### Scenario: Configured mapping with mismatched addresses
- **WHEN** temperature sensors are initialized with configured DS18B20 addresses
- **AND** no connected sensors match the configured addresses
- **THEN** the system falls back to discovery order assignment
- **AND** logs warnings about the mismatch

#### Scenario: No configured mapping
- **WHEN** temperature sensors are initialized with no configured address mappings
- **THEN** the system assigns sensors to areas based on discovery order
- **AND** the first discovered sensor is assigned to the power area
- **AND** the second discovered sensor is assigned to the control area
- **AND** maintains backward compatibility with existing behavior

#### Scenario: Partial mapping configuration
- **WHEN** only some monitoring areas have configured address mappings
- **THEN** configured areas use the specified address-based assignment
- **AND** unconfigured areas use discovery order assignment with remaining sensors

## ADDED Requirements

### Requirement: Temperature Sensor Mapping Configuration
The system SHALL provide APIs for configuring, retrieving, and managing DS18B20 address to monitoring area mappings.

#### Scenario: Configure sensor mapping
- **WHEN** user calls the sensor mapping configuration API
- **AND** provides a valid DS18B20 address for a monitoring area
- **THEN** the mapping is stored in non-volatile configuration
- **AND** the system validates the address format
- **AND** returns success confirmation

#### Scenario: Retrieve sensor mapping
- **WHEN** user calls the sensor mapping retrieval API
- **THEN** the system returns the currently configured address for the specified monitoring area
- **AND** indicates whether the mapping is configured or default

#### Scenario: Clear sensor mapping
- **WHEN** user calls the sensor mapping clear API for a monitoring area
- **THEN** the configured mapping is removed
- **AND** the area will use discovery order assignment on next initialization

#### Scenario: Auto-detect and suggest mappings
- **WHEN** temperature sensors are connected and the system is initialized
- **THEN** the system logs the discovered DS18B20 addresses and their current assignments
- **AND** provides suggestions for configuring deterministic mappings

### Requirement: DS18B20 Address Validation
The system SHALL validate DS18B20 addresses used in sensor mappings to ensure proper format and prevent conflicts.

#### Scenario: Validate address format
- **WHEN** user attempts to configure a sensor mapping
- **THEN** the system validates the address is a valid 64-bit DS18B20 format
- **AND** rejects addresses with invalid family codes or checksums

#### Scenario: Detect duplicate addresses
- **WHEN** user attempts to configure multiple monitoring areas with the same DS18B20 address
- **THEN** the system detects the duplicate
- **AND** rejects the configuration with an appropriate error message

#### Scenario: Address presence validation
- **WHEN** system is initialized with configured mappings
- **THEN** the system checks if configured addresses correspond to connected sensors
- **AND** logs warnings for configured addresses not found on the bus