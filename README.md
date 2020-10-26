# STM32 Bootloader Interface Library for Arduino

This library is designed to provide an easy-to-use abstraction for talking to STM32 bootloaders, with the primary intention of making it easy to preform F-OTA to a 'slave' STM32 MCU from another 'master' MCU.

The library was made out of a personal need, and I thought it would be useful for others. The base-code is based off of [this example from mengguang](http://https://github.com/mengguang/esp8266_stm32_isp "this example from mengguang"), which demonstrates talking to an STM32 bootloader from an ESP8266.

From Mengguang's example code, I was able to extend it further, and neatly package it up in an Arduino library that providers a simple API to read and write bytes(primary firmware) to the target device.

------------

### Example Code(s):
> Most/all of the examples are written for ESP32 Arduino core, however they could be adapted with ease to the Arduino MCU you are using. Just choose appropriate pins, and Serial/SoftSerial at 1200-115200 baud,  **8E1 (8 Bits, Even Parity, 1 Stop Bit)**.

Look into the examples folder to see the examples I have written for the library.
They demonstrate the primary functions and method calls you would need/want to use
for uploading firmware to an STM32 BluePill (STM32F103C8T6) via:
1. **File/Bytes Stream:** A firmware.bin file stored in SPIFFS read as stream
2. **Arbitrary-length Buffer:** A firmware upload/update from a websocket client chunked over multiple, separate, binary buffer transfers.

All examples also show you how to:
1. Easily preform validation of written data to STM32 via hashing written data and read-back.
2. Reboot target into User-Code mode via RST pins, or via 'Go' command in bootloader.

------------

### Using outside of Arduino:
While the code has be written as an Arduino library, there is very little preventing it from being usable from another platform. You would really just need to replace the serial read/write function to match API.

------------

### How to contribute:
*The code is certainly able to be cleaned & extended further. I welcome PRs :)*