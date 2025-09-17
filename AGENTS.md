# Hardware Definition
This project use ESP32S3 micro controller with following parts:

## PWM Channel
4 PWM Channels CH1/2/3/4 on GPIO7/8/9/10

Each channel can work in switch mode and PWM mode.
In switch mode, high level means ON, hi-res means OFF.

# Documents
Original demands are in `DEMAND.md`.
Designs are in `DESIGNS.md`

# References
Always use context7 when I need code generation, setup or configuration steps, or
library/API documentation. This means you should automatically use the Context7 MCP
tools to resolve library id and get library docs without me having to explicitly ask.

For ESP32 or ESP-IDF related knowledge, use library /espressif/esp-idf for API and docs.

# Build
Always use idf.py to build this project, DO NOT use cmake directly.