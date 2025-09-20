# Hardware Definition
This project use ESP32S3 micro controller with following parts:

## PWM Channel
4 PWM Channels CH1/2/3/4 on GPIO7/8/9/10

Each channel can work in switch mode and PWM mode.
In switch mode, high level means ON, hi-res means OFF.

## WS2812 RGB LED
Connected on GPIO21.

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