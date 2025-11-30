<!-- OPENSPEC:START -->
# OpenSpec Instructions

These instructions are for AI assistants working in this project.

Always open `@/openspec/AGENTS.md` when the request:
- Mentions planning or proposals (words like proposal, spec, change, plan)
- Introduces new capabilities, breaking changes, architecture shifts, or big performance/security work
- Sounds ambiguous and you need the authoritative spec before coding

Use `@/openspec/AGENTS.md` to learn:
- How to create and apply change proposals
- Spec format and conventions
- Project structure and guidelines

Keep this managed block so 'openspec update' can refresh the instructions.

<!-- OPENSPEC:END -->

# Hardware Definition
This project use ESP32S3 micro controller with following parts:

## 外设电源
GPIO1控制所有外设的电源，唤醒时先将GPIO1设置为高电平输出以使所有外设上电；睡眠前将GPIO1设置为低电平以节省功耗。

## ADC128S102和电流传感器
通过SPI连接了一个外置ADC以读取电流数据。
MCU上的连接情况如下：
- GPIO3为CS
- GPIO4为SCLK
- GPIO5为MISO
- GPIO6为MOSI

ADC128S102各个通道连接情况如下：
- IN0: 一个ACS712用于读取CH6的电流
- IN1: 一个ACS712用于读取CH5的电流
- IN2: 一个ACS712用于读取CH4的电流
- IN3: 一个ACS712用于读取CH3的电流
- IN4: 一个ACS712用于读取CH2的电流
- IN5: 一个ACS712用于读取CH1的电流


## PWM Channel
6 PWM Channels CH1/2/3/4/5/6 on GPIO8/9/10/11/12/13

Each channel can work in switch mode and PWM mode.
In switch mode, high level means ON, hi-res means OFF.

## WS2812 RGB LED
Connected on GPIO21.

## 电源电压探针
GPIO1上连接了一个分压电阻，使用ADC读取可以获得电源电压，分压电阻为240k:910k

## Temperature sensor DS18B20
Two DS18B20 are connected to GPIO7, one for power area, one for control area.

# References
Always use context7 when I need code generation, setup or configuration steps, or
library/API documentation. This means you should automatically use the Context7 MCP
tools to resolve library id and get library docs without me having to explicitly ask.

For ESP32 or ESP-IDF related knowledge, use library /espressif/esp-idf for API and docs.

# Build
Always use idf.py to build this project, DO NOT use cmake directly.