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

Category rules and stock thresholds live in `_data/categories.yml` and `_data/site.yml`.

## Local Build

```powershell
pip install -r requirements.txt
python scripts/build_catalog.py
python scripts/validate_products.py
bundle exec jekyll build
```
