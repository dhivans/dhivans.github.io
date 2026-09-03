---
layout: post
title: "D2F-01 vs D2F-5L: Which Microswitch for Your 3D Printer Endstop?"
date: 2026-09-03
categories: [guides]
products:
  - B0DFJ7DT9D
  - B0DBGPD14M
---

Two switches, near-identical listings, both titled for the same job: "3D Printer Microswitch for Klicky Probe, Bed Levelling, Limit Switches." Same lever design, same M2 mounting holes, same 6.5mm hole spacing. If you're picking one for an endstop or a Klicky probe, there's nothing in the photos or the titles that tells you which to buy.

There's a real answer, and it's not in either listing: **use the D2F-01, not the D2F-5L.**

## The difference that actually matters

Both are genuine Omron D2F switches — same family, same [datasheet](https://omronfs.omron.com/en_US/ecb/products/pdf/en-d2f.pdf). Where they diverge is the contact material, which decides how little current the switch can reliably handle:

| | [D2F-01](/shop/d2f-01-3d-printer-microswitch-for-klicky-probe-bed-levelling-limit-switches-fila/) | [D2F-5L](/shop/omron-japan-d2f-5l-3d-printer-microswitch-for-klicky-probe-bed-levelling-limit-s/) |
|---|---|---|
| Rated voltage/current | 30V DC, 0.1A | 250V AC, 5A |
| Contact material | Gold alloy | Silver alloy |
| Minimum applicable load | **1 mA** at 5V DC | **100 mA** at 5V DC |

That minimum-load figure is the whole story. Omron's own datasheet spells out why it exists, under "Using Micro Loads":

> Using a model for ordinary loads to open or close the contact of a micro load circuit may result in faulty contact.

A 3D printer mainboard's endstop or probe input is a logic-level signal — it's not switching mains power or driving a motor, it's just telling a GPIO pin "triggered" or "not triggered." Depending on the board's pull-up resistor, that's often a fraction of a milliamp to a few milliamps flowing through the switch. That's below the D2F-5L's 100 mA minimum, and comfortably inside the D2F-01's 1 mA minimum.

Below the rated minimum, a silver contact doesn't necessarily fail outright — it's a reliability problem, not a hard cutoff. Too little current through the contact means it doesn't self-clean the way it's meant to, so oxide/film buildup can accumulate over repeated switching, showing up eventually as an intermittent or missed trigger rather than an immediate failure. Gold contacts don't have that problem at low currents, which is exactly why the low-current "micro-load" variant of this switch exists in Omron's lineup at all.

## So why does DST (and everyone else) sell both under the same title?

Because the D2F-5L isn't a *bad* switch — it's the wrong tool for this one specific job. Its 5A/250V rating makes sense if you're using a microswitch to directly cut real power to something (not through a microcontroller), which is a legitimate use for a limit switch in general. It's just not what a Klicky probe or endstop input actually needs, and the generic "3D printer microswitch" listing title doesn't draw that distinction for you.

## The practical takeaway

- **Klicky probe, bed-levelling endstop, filament runout sensor, any switch wired into a printer mainboard's logic input** → [D2F-01](/shop/d2f-01-3d-printer-microswitch-for-klicky-probe-bed-levelling-limit-switches-fila/).
- **Switching real current directly (not through a microcontroller)** → the [D2F-5L](/shop/omron-japan-d2f-5l-3d-printer-microswitch-for-klicky-probe-bed-levelling-limit-s/) is the correct part for that, just not for the printer-input use case above.

Both are stocked in multiple pack sizes if you need more than one — a full Klicky probe setup typically wants a few.
