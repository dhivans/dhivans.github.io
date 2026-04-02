#!/usr/bin/env python3
"""
sync_products.py — Pulls live data from Amazon SP-API and keeps
Jekyll product markdown files in _products/ in sync.

What it does:
  1. Fetches all ACTIVE listings (title + price) using the Listings Items API.
     Price comes from the 'offers' includedData — no separate pricing call needed.
  2. Indexes every existing _products/*.md file by ASIN.
  3. For each listing that already has a matching .md file:
       - Updates the price from the listings data.
       - Skips writing the file if nothing changed.
  4. For each listing with NO matching .md file:
       - Generates a slug and assigns a category from the listing title.
       - Creates a new _products/{slug}.md from the standard template.
       - Never overwrites an existing file.
  5. Prints a summary: checked / updated / created / skipped.

NOTE: The Catalog Items API (/catalog/2022-04-01/items/{asin}) requires a
separate "Catalog Items" role in your SP-API app. If that role isn't granted,
image and description fields will be left as TODO placeholders in new files.
You can add that role in Seller Central → Apps & Services → Develop Apps,
and re-run the script to backfill images automatically.

Required environment variables (never hardcoded):
  AMAZON_CLIENT_ID       — SP-API app client ID (LWA)
  AMAZON_CLIENT_SECRET   — SP-API app client secret (LWA)
  AMAZON_REFRESH_TOKEN   — LWA refresh token for your seller account
  AMAZON_MARKETPLACE_ID  — e.g. A1F83G8C2ARO7P  (UK marketplace)
  AMAZON_SELLER_ID       — your Seller Central merchant ID

Run locally:
  export AMAZON_CLIENT_ID=...  (set all five)
  python scripts/sync_products.py
"""

import os
import re
import sys
import logging
import unicodedata
from pathlib import Path

import requests

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Config — read from environment, fail fast if anything is missing
# ---------------------------------------------------------------------------
def _require_env(name: str) -> str:
    value = os.environ.get(name, "").strip()
    if not value:
        log.error("Missing required environment variable: %s", name)
        sys.exit(1)
    return value

CLIENT_ID      = _require_env("AMAZON_CLIENT_ID")
CLIENT_SECRET  = _require_env("AMAZON_CLIENT_SECRET")
REFRESH_TOKEN  = _require_env("AMAZON_REFRESH_TOKEN")
MARKETPLACE_ID = _require_env("AMAZON_MARKETPLACE_ID")
SELLER_ID      = _require_env("AMAZON_SELLER_ID")

# SP-API base URL — UK / EU endpoint (eu-west-1 region)
SPAPI_BASE = "https://sellingpartnerapi-eu.amazon.com"

# _products/ directory (two levels up from this script)
PRODUCTS_DIR = Path(__file__).resolve().parent.parent / "_products"

# ---------------------------------------------------------------------------
# Category keyword mapping
# Checked in order — first match wins; falls back to "diy-components".
# ---------------------------------------------------------------------------
CATEGORY_RULES: list[tuple[str, list[str]]] = [
    ("test-equipment", [
        "multimeter", "oscilloscope", "power supply", "signal",
        "probe", "meter", "tester",
    ]),
    ("kits", [
        "kit", "starter", "bundle", "set", "pack",
    ]),
]
DEFAULT_CATEGORY = "diy-components"


# ===========================================================================
# Authentication
# ===========================================================================

def get_access_token() -> str:
    """
    Exchange the long-lived LWA refresh token for a short-lived access token.
    POST https://api.amazon.co.uk/auth/o2/token
    """
    log.info("Fetching LWA access token …")
    resp = requests.post(
        "https://api.amazon.co.uk/auth/o2/token",
        data={
            "grant_type":    "refresh_token",
            "refresh_token": REFRESH_TOKEN,
            "client_id":     CLIENT_ID,
            "client_secret": CLIENT_SECRET,
        },
        timeout=15,
    )
    resp.raise_for_status()
    token = resp.json()["access_token"]
    log.info("Access token obtained.")
    return token


# ===========================================================================
# SP-API helper
# ===========================================================================

def spapi_get(token: str, path: str, params: dict = None) -> dict:
    """
    Authenticated GET against the SP-API.
    Raises requests.HTTPError on non-2xx responses.
    """
    resp = requests.get(
        SPAPI_BASE + path,
        headers={
            "x-amz-access-token": token,
            "Content-Type": "application/json",
        },
        params=params or {},
        timeout=20,
    )
    resp.raise_for_status()
    return resp.json()


# ===========================================================================
# Step 1 — Fetch all active listings (title + price in one call)
# ===========================================================================

def fetch_active_listings(token: str) -> list[dict]:
    """
    Pages through all ACTIVE listings for this seller.

    We request 'summaries' (asin, title) and 'offers' (our listing price).

    Valid includedData values for /listings/2021-08-01/items/{sellerId}:
      summaries | attributes | offers | issues | fulfillmentAvailability | procurement
    Note: 'prices' is NOT valid here — it causes a 400 error.

    GET /listings/2021-08-01/items/{sellerId}
      ?marketplaceIds=...&includedData=summaries,offers&status=ACTIVE

    Returns a list of dicts:
      [{"asin": ..., "sku": ..., "title": ..., "price": "£9.99"}, ...]
    """
    log.info("Fetching active listings (summaries + offers) …")
    listings: list[dict] = []
    page_token = None

    while True:
        params: dict = {
            "marketplaceIds": MARKETPLACE_ID,
            "includedData":   "summaries,offers",
            "status":         "ACTIVE",
            "pageSize":       20,
        }
        if page_token:
            params["pageToken"] = page_token

        try:
            data = spapi_get(token, f"/listings/2021-08-01/items/{SELLER_ID}", params)
        except requests.HTTPError as exc:
            log.error("Failed to fetch listings page: %s", exc)
            break

        for item in data.get("items", []):
            sku = item.get("sku", "")

            # summaries — asin + title
            summaries = item.get("summaries", [{}])
            summary   = summaries[0] if summaries else {}
            asin  = summary.get("asin", "")
            title = summary.get("itemName", "")
            if not asin:
                continue

            # offers — our own listing price (no separate pricing API call needed)
            price_str = _extract_price(item.get("offers", []))

            listings.append({
                "asin":  asin,
                "sku":   sku,
                "title": title,
                "price": price_str,   # may be None if listing has no active offer
            })

        page_token = data.get("pagination", {}).get("nextToken")
        if not page_token:
            break

    log.info("Found %d active listings.", len(listings))
    return listings


def _extract_price(offers_list: list) -> str | None:
    """
    Parses the 'offers' array from a Listings Items 2021-08-01 response.
    Returns a formatted price string like "£9.99", or None if unavailable.

    Offer structure:
      [{"marketplaceId": "...", "offerType": "B2C",
        "price": {"listingPrice": {"amount": 9.99, "currencyCode": "GBP"}}}]

    Prefers the B2C offer type (retail); falls back to the first offer found.
    Some listings return an empty offers list (no active price set) — returns None.
    """
    b2c: tuple | None = None
    fallback: tuple | None = None

    for offer in offers_list:
        lp = offer.get("price", {}).get("listingPrice", {})
        amount   = lp.get("amount")
        currency = lp.get("currencyCode", "GBP")
        if amount is None:
            continue
        entry = (float(amount), currency)
        if fallback is None:
            fallback = entry
        if offer.get("offerType") == "B2C":
            b2c = entry
            break

    chosen = b2c or fallback
    if chosen is None:
        return None
    amount, currency = chosen
    symbol = "£" if currency == "GBP" else f"{currency} "
    return f"{symbol}{amount:.2f}"


# ===========================================================================
# Step 2 — Slug and category helpers
# ===========================================================================

def make_slug(title: str) -> str:
    """
    Converts a product title into a URL-safe file slug.
    e.g. "ESP32 WROOM-32 Dev Board (Wi-Fi)" → "esp32-wroom-32-dev-board-wi-fi"
    """
    title = unicodedata.normalize("NFKD", title)
    title = title.encode("ascii", "ignore").decode("ascii")
    title = title.lower()
    title = re.sub(r"[^a-z0-9]+", "-", title)
    title = re.sub(r"-{2,}", "-", title).strip("-")
    if len(title) > 80:
        title = title[:80].rsplit("-", 1)[0]
    return title


def assign_category(title: str) -> str:
    """
    Returns a category string based on keyword matches in the product title.
    Falls back to DEFAULT_CATEGORY if no keywords match.
    """
    lower = title.lower()
    for category, keywords in CATEGORY_RULES:
        if any(kw in lower for kw in keywords):
            return category
    return DEFAULT_CATEGORY


# ===========================================================================
# Step 3 — Parse and index existing .md files
# ===========================================================================

def parse_front_matter(text: str) -> tuple[str, str, str]:
    """
    Splits a Jekyll markdown file into (full_match, front_matter_raw, body).
    Returns ("", "", text) if no front matter delimiters are found.
    We keep the raw FM string to avoid a full YAML round-trip that would strip
    comments and reorder keys.
    """
    m = re.match(r"^---\s*\n(.*?)\n---\s*\n(.*)$", text, re.DOTALL)
    if not m:
        return "", "", text
    return m.group(0), m.group(1), m.group(2)


def load_product_files() -> dict[str, dict]:
    """
    Reads every .md in _products/ and builds an ASIN → file lookup:
      {
        "B0DJPZHZ1X": {
            "path":          Path("_products/esp32-wroom-32.md"),
            "type":          "single" | "variant",
            "variant_index": None | int,
        },
        ...
      }
    Files with multiple variants each register an entry per ASIN.
    """
    index: dict[str, dict] = {}

    for md_path in sorted(PRODUCTS_DIR.glob("*.md")):
        text = md_path.read_text(encoding="utf-8")
        _, fm_raw, _ = parse_front_matter(text)
        if not fm_raw:
            continue

        # Single top-level  asin: B0XXXXXXXX
        single = re.search(r"^asin:\s*[\"']?([A-Z0-9]{10})[\"']?", fm_raw, re.MULTILINE)
        if single:
            index[single.group(1)] = {"path": md_path, "type": "single", "variant_index": None}
            continue

        # Variant list items:  - asin: B0XXXXXXXX  (indented)
        variant_asins = re.findall(
            r"^\s+-\s+.*?asin:\s*[\"']?([A-Z0-9]{10})[\"']?",
            fm_raw,
            re.MULTILINE,
        )
        for i, asin in enumerate(variant_asins):
            index[asin] = {"path": md_path, "type": "variant", "variant_index": i}

    log.info(
        "Indexed %d ASINs across %d product files.",
        len(index),
        len({v["path"] for v in index.values()}),
    )
    return index


# ===========================================================================
# Step 4 — In-place front matter surgery (update existing files)
# ===========================================================================

def update_price_in_fm(fm_raw: str, new_price: str, variant_index: int | None) -> str:
    """Updates the price field in the raw front matter string."""
    if variant_index is None:
        return re.sub(
            r'^(price:\s*)["\']?[^"\'#\n]*["\']?',
            f'price: "{new_price}"',
            fm_raw,
            count=1,
            flags=re.MULTILINE,
        )
    else:
        # Price inside the Nth variant block (indented 4 spaces)
        variant_blocks = list(re.finditer(
            r"(  - (?:label|asin):.*?)(?=\n  - (?:label|asin):|\Z)",
            fm_raw,
            re.DOTALL,
        ))
        if variant_index >= len(variant_blocks):
            return fm_raw
        bm = variant_blocks[variant_index]
        new_block = re.sub(
            r'(    price:\s*)["\']?[^"\'#\n]*["\']?',
            f'    price: "{new_price}"',
            bm.group(0),
            count=1,
            flags=re.MULTILINE,
        )
        return fm_raw[: bm.start()] + new_block + fm_raw[bm.end():]


def write_updated_file(md_path: Path, fm_raw: str, body: str) -> None:
    """Writes the reconstructed markdown file back to disk."""
    md_path.write_text(f"---\n{fm_raw}\n---\n{body}", encoding="utf-8")


# ===========================================================================
# Step 5 — Create a new product file for an unmatched ASIN
# ===========================================================================

def _yaml_str(value: str) -> str:
    """Wraps a value in double quotes, escaping any embedded double quotes."""
    return '"' + value.replace('"', '\\"') + '"'


def create_product_file(asin: str, title: str, price: str | None) -> Path | None:
    """
    Generates a new _products/{slug}.md from the standard template.
    Title and price come directly from the listings API response.
    Image and description are left as TODO placeholders — they can be filled
    in manually or by a future run once the Catalog Items API role is granted.

    Returns the Path of the created file, or None if the slug is empty or
    a file with that name already exists (safety guard).
    """
    if not title:
        title = f"Product {asin}"

    slug = make_slug(title)
    if not slug:
        log.warning("Could not generate slug for ASIN %s (title: %r)", asin, title)
        return None

    md_path = PRODUCTS_DIR / f"{slug}.md"
    if md_path.exists():
        log.warning("File already exists, skipping creation: %s", md_path.name)
        return None

    category   = assign_category(title)
    price_str  = price or ""
    amazon_url = f"https://www.amazon.co.uk/dp/{asin}"

    content = f"""\
---
layout: product
title: {_yaml_str(title)}
description: "" # TODO: add description
category: "{category}"
price: "{price_str}"
amazon_url: "{amazon_url}"
asin: "{asin}"
image: "" # TODO: add image
featured: false
hot: false
badge: ""
tags: []
---

"""
    md_path.write_text(content, encoding="utf-8")
    return md_path


# ===========================================================================
# Main
# ===========================================================================

def main():
    token      = get_access_token()
    listings   = fetch_active_listings(token)
    asin_index = load_product_files()

    # De-duplicate: if the same ASIN appears more than once in listings (can
    # happen when the same product has multiple SKUs), process it only once.
    seen: set[str] = set()

    checked  = 0
    updated  = 0
    created  = 0
    no_price = 0
    skipped  = 0

    for listing in listings:
        asin  = listing["asin"]
        price = listing["price"]
        title = listing["title"]

        if asin in seen:
            continue
        seen.add(asin)
        checked += 1

        # ----------------------------------------------------------------
        # Case A — existing .md file found: update price only
        # ----------------------------------------------------------------
        if asin in asin_index:
            if price is None:
                log.debug("No price in listings data for %s — skipping update.", asin)
                no_price += 1
                continue

            entry   = asin_index[asin]
            md_path = entry["path"]
            v_idx   = entry["variant_index"]

            original = md_path.read_text(encoding="utf-8")
            _, fm_raw, body = parse_front_matter(original)
            if not fm_raw:
                log.warning("Could not parse front matter in %s", md_path.name)
                skipped += 1
                continue

            modified_fm = update_price_in_fm(fm_raw, price, v_idx)

            if modified_fm == fm_raw:
                log.info("No change: %s  (ASIN %s)", md_path.name, asin)
                continue

            write_updated_file(md_path, modified_fm, body)
            log.info("Updated  %s  (ASIN %s)  price=%s", md_path.name, asin, price)
            updated += 1

        # ----------------------------------------------------------------
        # Case B — no matching .md file: create it from listing data
        # ----------------------------------------------------------------
        else:
            new_file = create_product_file(asin, title, price)
            if new_file:
                log.info("Created  %s  (ASIN %s)  price=%s", new_file.name, asin, price or "—")
                created += 1
            else:
                skipped += 1

    # ----------------------------------------------------------------
    # Summary
    # ----------------------------------------------------------------
    print("\n" + "=" * 62)
    print(f"  Unique ASINs checked      : {checked}")
    print(f"  Existing files updated    : {updated}")
    print(f"  New files created         : {created}")
    print(f"  Skipped (no price data)   : {no_price}")
    print(f"  Skipped (other)           : {skipped}")
    if no_price:
        print("\n  Tip: ASINs with no price have no active B2C offer in the")
        print("  Listings API. Check these in Seller Central.")
    print("=" * 62)


if __name__ == "__main__":
    main()
