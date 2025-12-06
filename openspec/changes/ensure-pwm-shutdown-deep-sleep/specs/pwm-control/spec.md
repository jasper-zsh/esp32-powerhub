## ADDED Requirements

### Requirement: RTC GPIO Configuration for PWM Deep Sleep
The system SHALL use RTC GPIO configuration to ensure PWM channels remain powered off during deep sleep.

#### Scenario: Deep sleep RTC GPIO preparation
- **WHEN** the system prepares to enter deep sleep
- **THEN** ALL PWM channels on GPIOs 8-13 SHALL be configured as RTC GPIOs
- **AND** these RTC GPIOs SHALL be set to output low state
- **AND** hold functionality SHALL be enabled to maintain low state during sleep
- **AND** force hold SHALL be enabled for all RTC IOs

#### Scenario: RTC GPIO restoration on wake
- **WHEN** the system wakes from deep sleep
- **THEN** force hold functionality SHALL be disabled for all RTC IOs
- **AND** individual RTC GPIO hold SHALL be disabled
- **AND** RTC GPIOs SHALL be restored to normal LEDC control
- **AND** saved PWM channel states SHALL be restored normally

#### Scenario: PWM state preservation compatibility
- **WHEN** PWM channels are configured for deep sleep
- **THEN** the current channel states SHALL be saved before RTC GPIO configuration
- **AND** the saved states SHALL be restored after wake and RTC restoration
- **AND** thermal protection integration SHALL be maintained

#### Scenario: Complete RTC GPIO coverage
- **WHEN** configuring PWM channels for deep sleep
- **THEN** ALL GPIOs 8-13 SHALL use RTC GPIO hold functionality
- **AND** all 6 PWM channels SHALL be guaranteed to remain off during deep sleep
- **AND** the system SHALL maintain consistent behavior across all channels