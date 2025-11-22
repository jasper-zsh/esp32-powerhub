# Change: Refactor Sensor Manager Architecture

## Why
The current sensor implementation has tightly coupled ADC drivers with business logic, mixing voltage-to-current conversion with hardware-specific code. This creates maintenance overhead and limits extensibility. We need a clean separation between ADC drivers (voltage-only) and sensor logic (voltage-to-value conversion), while supporting variable channel counts and optional total current measurement.

## What Changes
- Create unified SensorManager with parent class handling voltage-to-value conversion
- Separate ADC driver layer to only handle voltage measurements
- Support configurable number of current channels (1-6)
- Make total current sensor optional
- Maintain existing API compatibility for legacy current_sensor.h
- Keep power voltage monitoring separate from current sensing
- Preserve BLE interface and system monitor integration

## Impact
- Affected specs: power-voltage, channel-current (new capabilities)
- Affected code: main/adc128s102.cpp, main/current_sensor.h, main/system_monitor.c
- **BREAKING**: Internal ADC driver abstraction changes but external API preserved