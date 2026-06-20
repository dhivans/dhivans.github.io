---
title: "ESP32 Mood Ring — Knob-Controlled WS2812B RGB Light"
date: 2026-06-20
description: >-
  A desk mood light built from one knob and one ring: an ESP32 driving an
  8-LED WS2812B ring, with a rotary encoder for colour, brightness, and a
  chase mode.
tags: [esp32, ws2812b, rotary-encoder, build-log]
products:
  - asin: B0DJPZHZ1X
    qty: 1
  - asin: B0DNNPTVDR
    qty: 1
  - asin: B0DJPYB5N8
    qty: 1
  - asin: B0DLYTCS3J
    qty: 1
  - asin: B0DH8D9N9F
    qty: 1
---

A knob and a ring of light, wired straight to an ESP32: twist for colour, twist for brightness, click to switch modes. No soldering — just a breadboard, and an evening.

{% include bom.html %}

## Why this build

The ESP32 board has Wi-Fi and Bluetooth sitting unused in this version — deliberately. Getting an addressable LED ring and a rotary encoder talking to each other cleanly is plenty for one evening, and it's the foundation for the obvious next step (network control) without having to relearn the LED/encoder wiring later.

## What you'll also need

Everything above ships from the BOM, but a breadboard build also needs **male-to-male jumper wires** (roughly 10–12) to connect everything together. Any 22–24AWG hookup wire or jumper pack works fine — it doesn't need to be anything specific.

## Wiring

| From (ESP32) | To | Why |
|---|---|---|
| 5V / VIN | Breadboard + rail, LED ring VCC | WS2812B ring is rated DC 5V — don't run it off 3V3, it'll be dim and unreliable |
| GND | Breadboard − rail, LED ring GND, encoder GND | Common ground for everything |
| GPIO5 | LED ring DIN (through a 330Ω resistor) | Data line. The resistor isn't optional-optional — see below |
| 3V3 | Encoder + | **Not 5V** — see below |
| GPIO18 | Encoder CLK | |
| GPIO19 | Encoder DT | |
| GPIO21 | Encoder SW (push button) | |

Two wiring details that actually matter:

**Power the encoder from 3V3, not 5V.** The KY-040 is sold as a 5V module and its silkscreen usually labels the supply pin "+5V," but it's really just mechanical contacts with pull-up resistors on the breakout — it works fine at 3.3V. Wire it to 5V instead and CLK/DT/SW will swing up to 5V on release, which is well past the ESP32 GPIO's 3.6V absolute maximum. That's a real way to damage the pin, not a theoretical one.

**Put a resistor (~330Ω) in series on the LED data line**, and if you've got a 470µF+ capacitor spare, put it across the ring's 5V/GND right at the ring. Neither is strictly required for 8 LEDs on a short wire, but both protect the first LED's input from voltage spikes when the board powers up — cheap insurance.

## How it works

The encoder's rotation does different things depending on mode; its push button cycles modes. Three modes, one knob:

1. **Hue** — rotate to spin through the colour wheel at a fixed brightness.
2. **Brightness** — rotate to dim/brighten the current colour.
3. **Chase** — rotate to change chase speed; the ring runs a moving rainbow regardless of position.

Click the knob to step between the three.

## The code

Arduino IDE, ESP32 board package installed, `Adafruit NeoPixel` library installed. Board: whichever "ESP32 Dev Module" entry matches your board's USB-to-serial chip.

```cpp
#include <Adafruit_NeoPixel.h>

#define LED_PIN    5
#define LED_COUNT  8
#define ENC_CLK    18
#define ENC_DT     19
#define ENC_SW     21

Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

volatile int32_t encoderDelta = 0;
volatile uint8_t lastCLK = HIGH;

void IRAM_ATTR onEncoder() {
  uint8_t clk = digitalRead(ENC_CLK);
  if (clk != lastCLK) {
    encoderDelta += (digitalRead(ENC_DT) != clk) ? 1 : -1;
  }
  lastCLK = clk;
}

uint8_t mode = 0;        // 0 = hue, 1 = brightness, 2 = chase
uint8_t hue = 0;
uint8_t brightness = 80;
uint8_t chaseSpeed = 4;
bool lastButton = HIGH;

uint32_t wheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85)  return ring.Color(255 - pos * 3, 0, pos * 3);
  if (pos < 170) { pos -= 85; return ring.Color(0, pos * 3, 255 - pos * 3); }
  pos -= 170;
  return ring.Color(pos * 3, 255 - pos * 3, 0);
}

void setup() {
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), onEncoder, CHANGE);

  ring.begin();
  ring.setBrightness(brightness);
  ring.show();
}

void loop() {
  bool button = digitalRead(ENC_SW);
  if (button == LOW && lastButton == HIGH) {
    mode = (mode + 1) % 3;
    delay(150); // crude debounce — fine for a hand-press
  }
  lastButton = button;

  int32_t delta = encoderDelta;
  encoderDelta = 0;

  if (mode == 0) {
    hue += delta * 4;
    ring.setBrightness(brightness);
    for (int i = 0; i < LED_COUNT; i++) ring.setPixelColor(i, wheel(hue));
  } else if (mode == 1) {
    brightness = constrain((int)brightness + delta * 4, 0, 255);
    ring.setBrightness(brightness);
    for (int i = 0; i < LED_COUNT; i++) ring.setPixelColor(i, wheel(hue));
  } else {
    chaseSpeed = constrain((int)chaseSpeed + delta, 1, 12);
    static uint8_t chase = 0;
    chase += chaseSpeed;
    ring.setBrightness(brightness);
    for (int i = 0; i < LED_COUNT; i++) {
      ring.setPixelColor(i, wheel(chase + i * (256 / LED_COUNT)));
    }
  }

  ring.show();
  delay(15);
}
```

## What actually mattered

**Brightness ceiling on USB power.** Eight WS2812B LEDs at full white draw close to 480mA on top of what the ESP32 itself needs. A laptop USB port or a cheap charger can struggle with that combined load. The code defaults to `brightness = 80` (out of 255) for exactly this reason — comfortably bright on a desk, comfortably within what a USB port supplies. Push it to 255 and watch for brownout resets if your power source is marginal.

**Cheap KY-040 modules are noisy.** The `delay(150)` debounce on the button is doing real work — these modules bounce more than a "proper" tactile switch. If you see double-triggers on the mode button, that delay is the first thing to extend.

**GRB vs RGB.** WS2812B is wired internally as GRB despite the name — `NEO_GRB` in the constructor isn't a typo. Get this wrong and your "red" comes out green.

## What's next

The Wi-Fi radio sitting idle in this build is the natural next step — a small web page or MQTT topic to set colour/mode remotely, instead of only the knob. That's a follow-up post, not a same-evening one.
