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
- IN0: 一个ACS758用于读取总输入电流
- IN1~IN7: 各连接了一个ACS712用于读取CH1~CH6的电流


## PWM Channel
6 PWM Channels CH1/2/3/4/5/6 on GPIO8/9/10/11/12/13

Each channel can work in switch mode and PWM mode.
In switch mode, high level means ON, hi-res means OFF.

## WS2812 RGB LED
Connected on GPIO21.

## 电源电压探针
GPIO2上连接了一个分压电阻，使用ADC读取可以获得电源电压

## Temperature sensor DS18B20
Two DS18B20 are connected to GPIO7, one for power area, one for control area.

# Documents
Documents are divided into multiple folders in `versions` folder, like `versions/001.init`.
In every document folder, there're two main file:
- Original demands are in `DEMANDS.md`.
- Designs are in `DESIGNS.md`

Remember, write documents in Chinese.

# Workflow for creating new version
You MUST work in following flow when creating a new version:
1. Chat with the user and write demands, then ask the user to check the document.
2. Repeat step 1 until the user CONFIRM that you can go next.
3. Chat with the user and write designs base on the demands you wrote before, then ask the user to check the document.
4. Repeat step 3 until the user CONFIRM that you can go next.
5. Generate code according to documents you wrote before carefully, make sure you won't broke exist logic and designs, until the new design tell you to do so.
6. Make sure the project can build successfully.
7. Ask the user to test the new version, resolve all problems for the user. Remember to check and update documents if nessesary when you making any changes.

You can begin at any step if the user ask you to do so.

# References
Always use context7 when I need code generation, setup or configuration steps, or
library/API documentation. This means you should automatically use the Context7 MCP
tools to resolve library id and get library docs without me having to explicitly ask.

For ESP32 or ESP-IDF related knowledge, use library /espressif/esp-idf for API and docs.

# Build
Always use idf.py to build this project, DO NOT use cmake directly.