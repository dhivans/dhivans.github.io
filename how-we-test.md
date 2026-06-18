---
layout: page
title: How We Test
permalink: /how-we-test/
---

DST treats marketplace listings as claims, not proof. Product pages separate live catalogue data from verified measurements so buyers can see what has been checked and what still needs bench validation.

## What gets checked

- ASIN, product URL, price, and stock state are checked by the Amazon sync.
- Category and catalogue placement are normalised into the site data index.
- Bench measurements are added to product front matter as `verified_specs` when a product has been physically checked.
- Datasheets are linked when a manufacturer or credible component-level source is available.

## How to read a product page

The "Tested by DST" panel appears on every product. If measured specs are available, the panel shows the listed claim alongside the checked value. If a product has not yet had bench measurements added, the panel states that only catalogue-level data has been checked so far.

Amazon handles checkout, fulfilment, and returns. DST's job is to make the product information clearer before you click through.
