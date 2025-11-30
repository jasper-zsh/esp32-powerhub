## Context

The ESP32 power hub project uses DS18B20 temperature sensors connected to GPIO7, with two sensors expected:
- One for the power area temperature monitoring
- One for the control area temperature monitoring

Currently, sensor assignment is based on discovery order, which can be inconsistent across reboots or when sensors are replaced. This design introduces hard-coded area-to-address mapping while maintaining backward compatibility.

## Goals / Non-Goals

### Goals
- Provide deterministic sensor assignment across reboots
- Enable configuration of sensor address mappings
- Maintain backward compatibility with existing APIs
- Support fallback to discovery order when no mapping exists
- Allow runtime configuration and retrieval of mappings

### Non-Goals
- Change the physical sensor connection layout
- Modify the underlying DS18B20 RMT driver
- Change the temperature data format or API structure
- Support more than 2 temperature sensors (hardware limitation)

## Decisions

### Decision: Configuration-based mapping
**What**: Store sensor mappings as configurable 64-bit addresses associated with sensor types.
**Why**: Allows users to set specific sensor addresses for each area while providing flexibility for different hardware configurations.
**Implementation**: Add mapping table in temperature manager with configuration APIs.

### Decision: Fallback to discovery order
**What**: When no mapping is configured, use the order sensors are discovered during DS18B20 enumeration.
**Why**: Maintains backward compatibility and provides default behavior for unconfigured systems.
**Implementation**: Check mapping table first, fall back to index-based assignment when empty.

### Decision: 64-bit DS18B20 address format
**What**: Use full 64-bit DS18B20 addresses for mapping (including family code, serial number, CRC).
**Why**: DS18B20 sensors have unique 64-bit addresses, ensuring unambiguous identification.
**Implementation**: Store addresses as uint64_t with helper functions for display and validation.

## Architecture

### Data Structures
```c
typedef struct {
    uint64_t address;                    // DS18B20 64-bit address
    bool configured;                     // Whether mapping is set
} temp_sensor_mapping_t;

typedef struct {
    temp_sensor_mapping_t sensors[TEMP_MGR_MAX_SENSORS];
} temp_sensor_config_t;
```

### API Design
- `temp_mgr_set_sensor_mapping(sensor_type, address)` - Configure mapping
- `temp_mgr_get_sensor_mapping(sensor_type, address)` - Retrieve mapping
- `temp_mgr_clear_sensor_mapping(sensor_type)` - Clear mapping
- `temp_mgr_auto_detect_and_map()` - Auto-detect and map available sensors

### Assignment Logic
1. During initialization, check for configured mappings
2. If mapping exists, find corresponding DS18B20 address
3. If no mapping, fall back to discovery order assignment
4. Log mapping assignments for debugging

## Risks / Trade-offs

### Risk: Address conflicts
**Risk**: Multiple sensors configured with same address
**Mitigation**: Validate addresses during configuration, reject duplicates

### Risk: Invalid addresses
**Risk**: Configured address doesn't match any connected sensor
**Mitigation**: Check address existence during assignment, fall back to discovery if not found

### Trade-off: Configuration complexity vs predictability
**Trade-off**: Adding configuration increases complexity but provides deterministic behavior
**Balance**: Keep API simple, provide sensible defaults, clear error messages

### Trade-off: Memory usage
**Trade-off**: Storing mapping tables uses additional RAM (minimal impact)
**Balance**: Only store 2 mappings (8 bytes each), negligible impact on ESP32

## Migration Plan

### Phase 1: Add mapping infrastructure
- Add data structures to temp_mgr.h
- Implement mapping APIs in temp_mgr.c
- No change to existing behavior

### Phase 2: Update assignment logic
- Modify sensor assignment in temp_mgr_init()
- Add fallback logic for unconfigured mappings
- Add logging for debugging

### Phase 3: Testing and validation
- Test with configured mappings
- Test fallback behavior
- Validate error handling

### Rollback
- Remove mapping infrastructure changes
- Restore original discovery order logic
- All existing APIs remain unchanged

## Open Questions

- Should mapping configuration persist across reboots? (Current design: yes, stored in static config)
- Should we provide CLI commands for mapping configuration? (Current design: API only)
- How to handle partial configuration (only one sensor mapped)? (Current design: fallback for unmapped sensors)
- Should we provide auto-discovery and mapping suggestions? (Current design: yes, via logging)