# Change: Add Hard-coded Temperature Sensor Area to Address Mapping

## Why
Currently, the temperature sensor assignment relies on the order in which DS18B20 sensors are discovered during initialization, which can lead to inconsistent sensor mapping across reboots or when sensors are replaced. This change adds hard-coded area-to-address mapping to provide stable and predictable temperature sensor identification while maintaining fallback behavior when no configuration exists.

## What Changes
- Add hard-coded temperature sensor area-to-address mapping configuration
- Implement fallback logic that uses response order when no mapping is configured
- Add configuration interface for setting and retrieving sensor address mappings
- Maintain backward compatibility with existing temperature sensing APIs
- **BREAKING**: Changes sensor assignment behavior when custom mappings are provided

## Impact
- Affected specs: temperature-sensing
- Affected code: main/temp_mgr.c, main/temp_mgr.h, possibly hardware_defs.h
- Provides deterministic sensor identification for power area and control area sensors
- Enables flexible configuration while maintaining default behavior