---
title: "Bench Power Supply Build — LM317 Adjustable Regulator"
date: 2026-03-30
description: >-
  Building a simple adjustable bench power supply from scratch using an LM317
  regulator. Covers schematic, PCB layout considerations, and measured output performance.
tags: [power-supply, LM317, build-log]
---

<!--
  ================================================================
  HOW TO ADD FUTURE PROJECTS
  ================================================================
  1. Duplicate this file inside _projects/
  2. Rename it: YYYY-MM-DD-your-project-slug.md
     (date prefix controls sort order on the listing page)
  3. Update the front matter:
       title       — display title
       date        — YYYY-MM-DD
       description — one or two sentence summary shown on the listing
       tags        — list of relevant tags (optional)
  4. Write your content below the front matter in Markdown
  5. Commit and push — GitHub Pages builds it automatically
  ================================================================
-->

A bench power supply is the first thing worth building properly. Variable voltage, current limiting, clean regulation — a good one saves every project after it.

## The spec

Target output: 1.25V to 25V adjustable, up to 1.5A continuous. Source: a salvaged 24V transformer from a dead printer. Regulation via LM317T in TO-220 package mounted to a small aluminium heatsink.

## Schematic overview

The LM317 is configured in the standard adjustable output topology:

- Input from rectified/filtered transformer secondary (~28V DC under load)
- Output voltage set by resistor divider: R1 = 240Ω fixed, R2 = 5kΩ pot
- 0.1µF ceramic on input, 1µF electrolytic on output for stability
- 1N4002 protection diodes across input and output per the datasheet recommendation

Nothing exotic. The datasheet application circuit is good enough — no need to reinvent it.

## What actually mattered during the build

**Heatsink sizing.** At 25V input and 1.25V output with 1.5A load, the LM317 dissipates about 35W. A TO-220 without a heatsink handles maybe 2W before thermal shutdown. The aluminium plate salvaged from the same printer (approximately 80 × 50 × 3mm) brought junction temperature down to a safe range. Thermal paste between package and heatsink matters — don't skip it.

**Capacitor placement.** The output cap needs to be close to the adj and output pins, not at the load terminals. Learnt this the hard way — oscillation on the output rail at around 150kHz when the cap was at the far end of the wiring.

**Current limiting.** Not implemented in this first version. A proper bench supply needs current limiting (fold-back or constant-current). This one relies on the LM317's internal thermal protection. Adequate for light use, not for debugging shorted circuits.

## Measured results

| Pot position | Measured output | Ripple (no load) |
|---|---|---|
| Minimum | 1.27V | < 5mV |
| Mid | 12.4V | < 8mV |
| Maximum | 24.8V | < 12mV |

Regulation under 1A load: < 50mV drop across the range. Acceptable for most bench work.

## What comes next

Current limiting circuit — either a sense resistor + op-amp comparator approach, or a dedicated IC like the LM723. Will cover that in a follow-up build log.
