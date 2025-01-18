---
title: Setting Up an ESP32 with a BMP280 Sensor
author: Dhivan Shah
date: 2025-01-18 12:00:00 +0000
categories: [IoT, Tutorial]
tags: [ESP32, BMP280, Sensors, IoT]
render_with_liquid: false
---

This tutorial will guide you through the process of setting up an ESP32 development board with a BMP280 sensor to measure temperature and pressure.

## Requirements

Before proceeding, ensure you have the following components:

- ESP32 development board
- BMP280 sensor module
- Jumper wires
- Breadboard (optional)
- Computer with Arduino IDE installed

## Wiring the BMP280 to the ESP32

Connect the BMP280 sensor to the ESP32 using the I2C interface:

| BMP280 Pin | ESP32 Pin |
|-----------|----------|
| VCC       | 3.3V     |
| GND       | GND      |
| SCL       | GPIO 22  |
| SDA       | GPIO 21  |

> Ensure that your BMP280 sensor operates at 3.3V. Some modules may require a logic level shifter.
{: .prompt-warning }

## Installing the Required Libraries

To interface with the BMP280, install the following libraries in the Arduino IDE:

1. Open the **Arduino IDE** and go to **Sketch** > **Include Library** > **Manage Libraries**.
2. Search for `Adafruit BMP280` and install the library.
3. Install the `Wire` library if it's not already installed.

## Writing the Code

Create a new Arduino sketch and copy the following code:

```cpp
#include <Wire.h>
#include <Adafruit_BMP280.h>

#define SDA_PIN 21  // ESP32 I2C SDA
#define SCL_PIN 22  // ESP32 I2C SCL

Adafruit_BMP280 bmp; // I2C instance

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("BMP280 Sensor Test");

  // Start I2C with custom pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // Try to initialize BMP280 sensor
  if (!bmp.begin(0x76)) { // Use 0x77 if needed
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  // Set sensor to normal mode
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,  // Temperature oversampling
                  Adafruit_BMP280::SAMPLING_X16, // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,   // Filtering
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time
}

void loop() {
  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure() / 100.0F); // Convert Pa to hPa
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(1013.25)); // Adjust for sea level pressure
  Serial.println(" m");

  Serial.println();
  delay(2000);
}

