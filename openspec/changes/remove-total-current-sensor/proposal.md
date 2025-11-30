# Change: Remove Total Current Sensor from Hardware Design

## Why
The total current sensor (ACS758) has been removed from the current hardware design as documented in CLAUDE.md. The external ADC (ADC128S102) channels are now fully allocated to individual channel current sensors (CH1-CH6) using ACS712 sensors. However, the codebase still contains remnants of total current sensor logic that should be removed to:

- Eliminate confusion and dead code
- Simplify the sensor management architecture
- Reduce memory footprint by removing unused data structures
- Ensure hardware definitions match actual hardware implementation
- Prevent future maintenance issues with obsolete code paths

## What Changes
- Remove `TOTAL_CURRENT_*` hardware definitions and macros from `hardware_defs.h`
- Remove total current sensor fields from data structures in `current_sensor.h`
- Remove ACS758 calibration parameters and related functions
- Remove total current sensor logic from SensorManager implementation
- Update BLE payload format to exclude total current (save 4 bytes)
- Update system monitor to remove total current monitoring and logging
- Remove total current sensor validation and configuration code
- Update documentation to reflect simplified channel-only architecture

## Impact
- **BREAKING**: Changes C API data structures (`current_sensor_data_t`, `current_sensor_calibration_t`)
- **BREAKING**: Changes BLE notification payload format (reduced size)
- **Internal**: Simplifies SensorManager code paths
- **Internal**: Reduces memory usage and processing overhead
- **Documentation**: Updated hardware definitions and sensor architecture docs

## Scope
This change only affects the total current sensor removal. Individual channel current sensors (CH1-CH6) and power voltage monitoring remain unaffected.