---
title: ESP32-C3-Supermini
author: dhivan
date: 2024-09-04 20:00:00 +/-0000
categories: [Components, Microcontrollers]
tags: [esp32-c3-supermini]
description: Basic setup guide for the ESP32-C3-Supermini.
---
# ESP32-C3-Supermini
The ESP32 C3 Supermini is a compact and low-power development board designed for IoT applications. Featuring a powerful 32-bit RISC-V processor, Wi-Fi and Bluetooth connectivity, and rich I/O options, it's ideal for battery-powered devices and small-scale projects. Its miniature size, combined with its security features, makes it a versatile and efficient choice for building Internet of Things solutions.

## Setup
1. Connect ESP32 C3 Supermini to computer with USB-C Cable
2. Launch Arduino IDE
* Go to File > Preferences

![image](https://github.com/user-attachments/assets/295c1666-6fe0-4a78-80bb-0ca4104d813e)
* Select Additional board manager URLs

![image](https://github.com/user-attachments/assets/64d672a9-ac54-4ee0-bf76-c51ba978aac1)
* Paste in: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

![image](https://github.com/user-attachments/assets/a0b13667-6dfa-46aa-851d-a1ea85a58ab1)

* Select Ok
3. Board Manager
* Go to Tools > Board > Boards Manager

![image](https://github.com/user-attachments/assets/24d06e0e-9ad1-43d2-b8bb-546253d7b190)
* Search for ESP32

![image](https://github.com/user-attachments/assets/e7d829aa-073f-4dfc-8869-c422d8e760c3)
* Install ESP32 by Espressif Systems
4.  Flashing Blink
* Go to File > Examples > 01.Basics > Blink

![image](https://github.com/user-attachments/assets/5c8b84b7-557d-423a-83ad-4576d0098f39)
* Add "#define LED_BUILTIN 8" before void setup

![image](https://github.com/user-attachments/assets/0f0514b9-740b-4a25-8816-36a881b44aa2)
* Go to Tools > Board > esp32 > ESP32C3 Dev Module

![image](https://github.com/user-attachments/assets/8beda531-575f-435f-bfc0-a99296f2f3ab)
* Go to Tools > Port and choose the correct port

![image](https://github.com/user-attachments/assets/b762bc15-6920-4979-82c1-00eb8b9c88f4)
* Select the arrow uplaod icon

![image](https://github.com/user-attachments/assets/bef2c4bd-26aa-4941-8062-41216c850690)
6. Upload your code with the upload button

The built-in LED on your ESP32 C3 Supermini should start flashing
