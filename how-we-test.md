---
layout: page
title: How We Test
permalink: /how-we-test/
---

DST treats marketplace listings as claims, not proof. Product pages separate live catalogue data from what's actually been checked — and by whom — so buyers can see exactly what that means for any given item.

## The three badges, and what each one actually means

- **Visual QC by DST** — every item in the catalogue passes through DST and is physically handled and visually inspected before it's listed: correct part, condition, packaging. This applies to everything, on every product page.
- **Bench-Verified by DST** — a step further, for specific products: DST has physically measured particular specs on a real unit, not just taken the manufacturer's or listing's word for it. Shown only when that's actually been done, with the measured values in a table alongside the original claim.
- **Manufacturer Tested** — a note that the manufacturer reports testing something (e.g. a factory QC or functional test). This is *their* claim, not an independent DST check, and is labelled that way deliberately — it's shown separately so it never gets confused with DST's own verification.

## What gets checked

- ASIN, product URL, price, and stock state are checked by the Amazon sync.
- Category and catalogue placement are normalised into the site data index.
- Every item gets a DST visual QC pass before listing (see above) — this is the one universal claim on every product page.
- Bench measurements are added to product front matter as `verified_specs` when a product has actually been physically measured by DST.
- Datasheets are linked when a manufacturer or credible component-level source is available.

## How to read a product page

Every product shows the Visual QC badge. If a product has also been bench-verified, a second badge and a measured-values table appear alongside it. If it hasn't been bench-verified yet, the panel says so plainly, rather than implying a check that hasn't happened.

## Where the detail comes from

Manufacturers are asked directly for real specs and datasheets first. Where that isn't available, some products have an open request on **[Help Us Document the Catalogue](/help-document/)** — a UK-only programme where a product is sent out for free in exchange for the specific missing detail, evidenced with a photo, and credited on the product page as community-provided (never presented as a DST claim).

Amazon handles checkout, fulfilment, and returns. DST's job is to make the product information clearer before you click through.
