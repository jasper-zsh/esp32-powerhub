# Change: Ensure PWM Channels Are Off During Deep Sleep

## Why
The system currently saves PWM channel states before entering deep sleep but doesn't explicitly ensure all PWM channels are fully powered off. This can lead to:

1. **Unintended Power Consumption**: PWM channels may remain in an active state during deep sleep, consuming unnecessary power
2. **Hardware Stress**: PWM outputs might keep external devices powered when the system should be fully shut down
3. **Inconsistent Wake States**: Channels might not start from a clean, known state after deep sleep wake
4. **LEDC Peripheral Limitation**: LEDC peripheral doesn't maintain GPIO state during deep sleep, requiring RTC GPIO configuration

## What Changes
- **Updated Approach**: Use RTC GPIO configuration to ensure PWM channels stay off during deep sleep
- **Complete RTC Coverage**: Configure ALL GPIOs 8-13 as RTC GPIOs with hold functionality (all 6 channels are RTC-capable)
- **Hardware Configuration**: Set RTC GPIOs to output low and enable hold before deep sleep
- **State Preservation**: Maintain existing state persistence and restoration functionality
- **Clean Wake Integration**: Add RTC GPIO restoration on system wake-up

## Impact
- **Affected specs**: None (implementation change only)
- **Affected code**:
  - `main/pwm_control.c` - Add RTC GPIO shutdown and restoration functions
  - `main/power_mgr.c` - Integrate RTC GPIO shutdown in sleep preparation
  - `main/main.c` - Add RTC GPIO restoration on wake-up
- **Hardware Coverage**: ALL GPIOs 8-13 will be held low during deep sleep using RTC GPIO hold
- **Power behavior**: Significantly reduced deep sleep power consumption, guaranteed off state for all 6 channels