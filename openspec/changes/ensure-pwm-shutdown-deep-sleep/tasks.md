## 1. Implementation - RTC GPIO Solution
- [x] 1.1 Basic PWM shutdown function (already implemented - needs refinement)
- [x] 1.2 Add RTC GPIO configuration for GPIOs 8-12 in deep sleep shutdown
- [x] 1.3 Implement RTC GPIO hold functionality to maintain low state during sleep
- [x] 1.4 Add RTC GPIO restoration function for wake-up
- [x] 1.5 Update power_mgr_force_sleep() to use RTC GPIO configuration
- [x] 1.6 Add RTC GPIO restoration call to main.c wake-up sequence

## 2. Validation
- [x] 2.1 Build and test with `idf.py build`
- [x] 2.2 Verify ALL RTC GPIOs 8-13 maintain low state during deep sleep
- [x] 2.3 Test complete RTC GPIO coverage for all 6 channels
- [x] 2.4 Verify deep sleep entry and wake cycles work properly
- [x] 2.5 Confirm RTC GPIO restoration works correctly on wake
- [x] 2.6 Test that saved channel states are restored after wake