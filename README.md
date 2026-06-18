# DhivanSTech Website

Jekyll site for the DhivanSTech catalogue, projects, and posts.

## Product Data

Product markdown files live in `_products/`. Build-time data is generated from that front matter:

```powershell
python scripts/build_catalog.py
python scripts/validate_products.py
```

The Amazon sync also runs those checks after updating products. It writes:

- `_data/catalog.json` - one normalized record per ASIN for templates and client-side widgets.
- `_data/history/<asin>.json` - append-only price and stock snapshots.
- `_data/price_changes.json` - latest detected price changes for badges and `/deals/`.

Category rules and stock thresholds live in `_data/categories.yml` and `_data/site.yml`.

## Trust and Commerce Surfaces

- Product pages always render a "Tested by DST" panel. Add measured checks with `verified_specs` in product front matter; optional `test_notes` and `datasheet_url` are also supported.
- The test methodology page lives at `/how-we-test/`.
- `/deals/` renders price drops and 90-day lows from generated data.
- Use `{% include bom.html %}` in posts/projects to render a live-priced bill of materials from the page `products:` front matter, or pass comma-separated ASINs with `{% include bom.html asins="B0...,B0..." %}`.
- The nightly sync generates a low-stock report and opens or updates a single GitHub issue when any item is at or below `_data/site.yml`'s `low_stock_threshold`.

## Local Build

```powershell
pip install -r requirements.txt
python scripts/build_catalog.py
python scripts/validate_products.py
python scripts/check_stock_alerts.py
bundle exec jekyll build
```
