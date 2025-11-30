# Change: Refactor Temperature Sensor Logic with Task-Based Design

## Why
The current temperature sensor implementation uses synchronous blocking calls that delay system operation and tightly couple sensor reading with business logic. Each temperature reading requires 150-800ms delays, blocking the calling thread. We need an asynchronous task-based approach that continuously samples sensors in the background and provides immediate access to cached data through getter methods, with event notifications when new data is available.

## What Changes
- Create a dedicated FreeRTOS task for continuous temperature sensor sampling
- Store latest temperature readings in shared memory with proper synchronization
- Provide getter methods for immediate access to cached temperature data
- Implement event system to notify other modules when new temperature data is available
- Remove blocking delays from main application logic
- Maintain existing API compatibility where possible
- Support configurable sampling intervals
- Add error handling for sensor failures and disconnections

## Impact
- Affected specs: temperature-sensing (new capability)
- Affected code: main/temp_mgr.c, main/temp_mgr.h, main/power_mgr.c, main/system_monitor.c
- **BREAKING**: Changes internal synchronization model but preserves most external APIs