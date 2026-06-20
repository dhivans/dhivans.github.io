---
title: "3D Printer Endstop Swap — Wiring and Testing a D2F-01 Microswitch"
date: 2026-06-20
description: >-
  Replacing a worn 3D printer endstop with a D2F-01 microswitch: identifying
  the COM/NO/NC pins with a multimeter before wiring it in, then routing the
  new cable cleanly along the gantry.
tags: [3d-printing, microswitch, multimeter, build-log]
products:
  - asin: B0DFKGHT16
    qty: 1
  - asin: B0DJYPHZ27
    qty: 1
  - asin: B0DNNMN9BV
    qty: 1
  - asin: B07WYQHCY5
    qty: 1
---

The stock endstop on most budget printers is a cheap, unbranded microswitch riveted into a 3D-printed bracket. When one starts chattering or stops triggering consistently, it's a five-minute swap — as long as you check which pin does what before you cut into the harness.

{% include bom.html %}

## The problem

Mechanical endstops fail in two boring, predictable ways: the leaf spring fatigues and the lever stops returning cleanly, or the contacts pit and the switch starts reading "triggered" intermittently when it isn't. Either way the fix is the same — pull it, drop in a fresh D2F-01, and don't introduce a new fault while you're in there. Wrongly identifying the switch's three terminals is the easiest way to do that, since a homed axis that slams past its limit because the endstop is wired backwards is how you find out the hard way.

## Identify the pins first

A D2F-01 is a single-pole double-throw (SPDT) snap-action switch: three terminals — **COM**, **NO** (normally open), and **NC** (normally closed). Don't trust the silkscreen alone on a budget part; confirm it with the multimeter before it goes anywhere near the printer:

1. Multimeter in continuity/resistance mode.
2. Probe COM and the terminal you think is NO. Lever released: no continuity. Lever pressed: continuity.
3. Probe COM and the remaining terminal (NC). You should get the opposite — continuity when released, open when pressed.

If neither terminal behaves like that, you've got COM wrong — try the third combination. Two minutes with a meter here is cheaper than a re-wire after it's zip-tied to the gantry.

## Wiring

A bare mechanical endstop only needs two of the connector's three pins — signal and ground. The VCC pin matters on breakout boards with an onboard LED or optical sensor; a plain microswitch doesn't use it.

| Switch terminal | Endstop connector | Notes |
|---|---|---|
| COM | Signal | |
| NC or NO | GND | Pick based on your firmware's invert setting — see below |
| *(unused)* | VCC | Not needed for a bare mechanical switch |

**NC vs NO — check your firmware before you decide.** Most printer firmware (Marlin and its forks included) has a per-axis "endstop inverting" setting that has to match whichever contact you wired. Wiring NC is the safer default where you have the choice: if the wire ever chafes through or a connector works loose, an open NC circuit reads as "triggered," and the printer stops instead of homing straight through the bed. Whatever you choose, confirm it in the firmware config — guessing here is how axes crash on the next power-up.

## What actually mattered

**Budget microswitches have inconsistent throw.** Bag of three, no two felt quite the same under the lever — slightly different actuation force, slightly different point of engagement. Worth testing the one you're about to install through a full press-and-release cycle on the bench, not just confirming continuity once.

**Strain-relieve the new cable before you trust it.** The switch terminals are small enough that the wire itself becomes the weak point. A loop of slack at the switch, secured with a cable tie to the nearest frame member, keeps gantry movement from working the joint loose over a few thousand print-head passes.

**Tape the splice, not just the bare leads.** If you're extending the existing harness rather than re-terminating it, insulate each joint individually before bundling — wrapping the whole loom in one pass leaves pinholes where two leads sit side by side.

## What's next

Swapping the bare switch is the cheap fix. The next step up is a proper Klicky-style probe mount — a 3D-printed magnetic dock that lets the same kind of microswitch double as a removable bed probe instead of a fixed limit switch. That's a printed-parts project in its own right, not a same-evening one.
