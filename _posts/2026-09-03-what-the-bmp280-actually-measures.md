---
layout: post
title: "What the BMP280 Actually Measures (and What It Doesn't)"
date: 2026-09-03
categories: [guides]
products:
  - B0DJYT1J4H
  - B0DJYTSDW5
---

If you've searched for this sensor on Amazon, you've almost certainly seen it described as a "temperature, humidity, and pressure" module — DST's own listing said exactly that until this guide was written. It's wrong. The BMP280 doesn't measure humidity at all.

## What Bosch's own datasheet says

The [official BMP280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf) titles the part outright: **"BMP280 — Digital Pressure Sensor."** Not environmental sensor, not humidity sensor — pressure sensor. Its key parameters are pressure range, accuracy, temperature range, and interface speed. Its listed applications are GPS navigation refinement, indoor floor/elevation detection, weather forecasting, and watches — every one of them a pressure or temperature use case, with humidity absent from the entire document.

What it actually measures, verified against that datasheet:

| | BMP280 |
|---|---|
| Pressure range | 300–1100 hPa |
| Pressure accuracy | ±1 hPa (typ., 950–1050 hPa, 0–40°C) |
| Temperature range | −40°C to +85°C |
| Interface | I²C (up to 3.4 MHz) / SPI (up to 10 MHz) |

## Where the confusion comes from

Bosch makes a genuinely different chip, the **BME280**, that adds a humidity sensor on top of the same pressure/temperature core — one letter apart in the part number, "P" for pressure vs "E" for environmental. The two chips ship on nearly identical-looking breakout boards, so sellers routinely reuse one listing template for both, and the humidity claim just gets copy-pasted onto BMP280 boards that physically cannot measure it. It's an extremely common mix-up, not a DST-specific mistake — but that doesn't make it accurate, which is why it's fixed here.

## The practical answer

**If you only need temperature and pressure** (altitude estimation, a weather station's barometric trend, general environmental logging) — the [BMP280](/shop/bmp280-environmental-sensor-temperature-humidity-atmospheric-pressure/) is the right, and cheaper, part for the job.

**If you need humidity as well** — you need a board built around the BME280 chip specifically, not this one. DST doesn't currently stock a BME280 board; check the listing you're looking at names the chip explicitly before buying if humidity matters for your build.
