## MODIFIED Requirements

### Requirement: Reduced BLE Current Monitoring Payload
The system SHALL provide current monitoring data through BLE with a reduced payload format excluding total current measurements.

#### Scenario: BLE characteristic read access
- **WHEN** a BLE client reads the monitoring characteristic
- **THEN** the payload contains only channel current values (CH1-CH6)
- **AND** the payload size is reduced by 4 bytes (no total current field)
- **AND** channel current data starts immediately after voltage and temperature fields
- **AND** each channel current is transmitted as a 4-byte IEEE-754 float value

#### Scenario: BLE notification updates
- **WHEN** the current sensor generates BLE notification updates
- **THEN** notifications contain only channel current measurements
- **AND** total current field is omitted from all notifications
- **AND** notification timing remains unchanged for channel current updates
- **AND** missing or invalid channel currents transmit as -1.0f sentinel values

## REMOVED Requirements

### Requirement: BLE Total Current Transmission
The system SHALL NOT transmit total current measurements through BLE interfaces.

#### Scenario: BLE payload construction
- **WHEN** BLE payload is constructed for current sensor data
- **THEN** no bytes are allocated for total current measurement
- **AND** payload construction excludes total current field processing
- **AND** BLE characteristic access does not process or return total current data
- **AND** notification callbacks never include total current values