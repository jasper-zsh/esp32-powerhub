# Design: Remove Total Current Sensor

## Context
Based on the current hardware definition in CLAUDE.md, the total current sensor (ACS758) is no longer part of the hardware design. The external ADC128S102 now dedicates all channels to individual PWM channel current sensors:

- IN0: ACS712 for CH6 current
- IN1: ACS712 for CH5 current
- IN2: ACS712 for CH4 current
- IN3: ACS712 for CH3 current
- IN4: ACS712 for CH2 current
- IN5: ACS712 for CH1 current

However, the codebase still contains remnants of total current sensor logic that creates unnecessary complexity and potential confusion.

## Current State Analysis

### Hardware Definition Remnants
```cpp
// Total Current Sensor (ACS758, typically):
#define TOTAL_CURRENT_CH -1  // Set to -1 to disable, or ADC channel 0-7 to enable
#define TOTAL_CURRENT_OFFSET 2.5f        // Zero-point voltage reference (V)
#define TOTAL_CURRENT_SENSITIVITY 0.020f  // Voltage-to-current conversion factor (V/A)
#define HAS_TOTAL_CURRENT_SENSOR() (TOTAL_CURRENT_CH >= 0)
```

### Data Structure Remnants
```cpp
typedef struct {
    float total_input_current;                     // Total input current (A), -1 if no data
    float channel_currents[CURRENT_SENSOR_MAX_CHANNELS]; // Channel currents (A), -1 if no data
    uint32_t timestamp_ms;                         // Collection timestamp
    uint32_t valid_mask;                           // bit0=total, bit1-6=CH1-CH6
} current_sensor_data_t;

typedef struct {
    float acs758_offset;       // Total current sensor zero point (V)
    float acs758_sensitivity;  // Total current sensor sensitivity (V/A)
    float acs712_offset;       // Channel sensor zero point (V)
    float acs712_sensitivity;  // Channel sensor sensitivity (V/A)
} current_sensor_calibration_t;
```

### BLE Payload Remnants
```cpp
// Current BLE payload includes 4 bytes for total current
conv.f = data.total_input_current;  // 4 bytes
for (int i = 0; i < PWM_CHANNEL_COUNT; ++i) {
    conv.f = data.channel_currents[i];  // 4 bytes per channel
}
```

## Design Decisions

### 1. Complete Removal vs Optional Disable
**Decision**: Complete removal from codebase.

**Rationale**: The total current sensor is no longer in hardware design. Keeping optional code paths adds unnecessary complexity and maintenance burden. Future designs would require adding new code if needed.

### 2. API Compatibility
**Decision**: Breaking changes to C API structures.

**Rationale**: The current API exposes total current as a fundamental concept. Partial compatibility would be confusing and incomplete. A clean break ensures API reflects actual hardware capabilities.

### 3. BLE Payload Format
**Decision**: Remove total current field, reduce payload size by 4 bytes.

**Rationale**: Maintains backward compatibility is not possible without keeping dead data. Reduced payload size is beneficial for BLE efficiency. Client applications will need to be updated.

### 4. SensorManager Architecture
**Decision**: Simplify to channel-only architecture.

**Rationale**: Total current sensor logic adds branching and conditional processing throughout the SensorManager. Removal will simplify code paths and improve maintainability.

## Migration Strategy

### Data Structure Changes
```cpp
// Before:
typedef struct {
    float total_input_current;                     // To be removed
    float channel_currents[CURRENT_SENSOR_MAX_CHANNELS];
    uint32_t timestamp_ms;
    uint32_t valid_mask;                           // bit0 to be removed
} current_sensor_data_t;

// After:
typedef struct {
    float channel_currents[CURRENT_SENSOR_MAX_CHANNELS];
    uint32_t timestamp_ms;
    uint32_t valid_mask;                           // bit1-6 for CH1-CH6 only
} current_sensor_data_t;
```

### Calibration Structure Changes
```cpp
// Before:
typedef struct {
    float acs758_offset;       // To be removed
    float acs758_sensitivity;  // To be removed
    float acs712_offset;
    float acs712_sensitivity;
} current_sensor_calibration_t;

// After:
typedef struct {
    float acs712_offset;
    float acs712_sensitivity;
} current_sensor_calibration_t;
```

### SensorManager Internal Changes
- Remove `total_current` field from `sensor_config_t`
- Remove `total_current` from `sensor_readings_t`
- Remove total current processing from `resolve_adc_channels_to_sensors()`
- Remove total current from BLE payload generation
- Remove total current from compatibility cache

### System Monitor Changes
- Remove `last_total_current` from `monitor_state_t`
- Remove total current logging and threshold checking
- Update diagnostic messages to reference channel currents only

## Benefits
1. **Simplified Code**: Removes conditional logic and dead code paths
2. **Reduced Memory**: Smaller data structures and less RAM usage
3. **Clear Architecture**: Channel-only sensor model is easier to understand
4. **Better Performance**: Fewer conditional checks in sensor processing
5. **Accurate Documentation**: Code matches actual hardware implementation

## Risks and Mitigations
- **Risk**: Breaking existing client applications expecting total current in BLE data
- **Mitigation**: Document breaking change clearly and provide migration guidance

- **Risk**: System monitor may have referenced total current in event handling
- **Mitigation**: Remove total current from all system monitor logic and event conditions

- **Risk**: Calibration functions may have side effects on total current state
- **Mitigation**: Remove all total current state management from calibration code