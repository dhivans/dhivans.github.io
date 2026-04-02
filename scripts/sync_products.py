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
import time
import logging
import unicodedata
from datetime import datetime, timezone, timedelta
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
        "multimeter", "oscilloscope", "power supply", "signal generator",
        "meter", "tester", "caliper", "gauge",
    ]),
    ("tools", [
        "crimping tool", "crimper", "soldering iron", "solder sucker",
        "desoldering", "tweezers", "wrench", "allen key", "hex key",
        "screwdriver", "pliers", "wire stripper", "ruler", "brush",
    ]),
    ("mechanical-components", [
        "bearing", "belt", "pulley", "shaft", "bushing", "coupling",
        "spring", "screw", "bolt", "nut", "washer", "bracket",
        "cable tie", "zip tie", "hinge", "rail", "rod",
    ]),
]
DEFAULT_CATEGORY = "electrical-components"


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

def fetch_fba_inventory(token: str) -> dict[str, int]:
    """
    Pages through all FBA inventory summaries and returns a dict of
    ASIN -> fulfillable quantity.

    Uses fulfillableQuantity (inventory actually available to ship) rather
    than totalQuantity (which includes inbound/researching stock).

    Falls back to an empty dict if the API is unavailable, in which case
    main() will use stock=1 for all active listings as a safe fallback.
    """
    log.info("Fetching FBA inventory …")
    inventory: dict[str, int] = {}
    next_token = None

    while True:
        params: dict = {
            "granularityType": "Marketplace",
            "granularityId":   MARKETPLACE_ID,
            "marketplaceIds":  MARKETPLACE_ID,
            "details":         "true",
        }
        if next_token:
            params["nextToken"] = next_token

        try:
            resp = requests.get(
                SPAPI_BASE + "/fba/inventory/v1/summaries",
                headers={
                    "x-amz-access-token": token,
                    "Content-Type": "application/json",
                },
                params=params,
                timeout=20,
            )
            resp.raise_for_status()
        except requests.HTTPError as exc:
            log.warning("FBA Inventory API unavailable (%s) — stock will default to 1 for active listings.", exc)
            return {}

        payload = resp.json().get("payload", {})
        for s in payload.get("inventorySummaries", []):
            asin = s.get("asin", "")
            if not asin:
                continue
            qty = (s.get("inventoryDetails") or {}).get("fulfillableQuantity") or 0
            # Sum in case the same ASIN appears across multiple SKUs
            inventory[asin] = inventory.get(asin, 0) + int(qty)

        next_token = (resp.json().get("pagination") or {}).get("nextToken")
        if not next_token:
            break

    log.info("FBA inventory fetched: %d ASINs with stock data.", len(inventory))
    return inventory


def fetch_order_sales(token: str, days: int = 30) -> dict[str, int]:
    """
    Fetches all orders in the last `days` days and returns units sold per ASIN.

    Adaptive rate limiting: starts at 1.0 s/request (SP-API burst=30, restore=0.5 req/s).
    On every 429 the delay grows by 0.1 s so it self-corrects without manual tuning.
    Falls back to an empty dict on any non-recoverable error so the rest of the
    sync continues normally.
    """
    created_after = (datetime.now(timezone.utc) - timedelta(days=days)).strftime(
        "%Y-%m-%dT%H:%M:%SZ"
    )
    log.info("Fetching orders since %s for sales data …", created_after)

    orders = []
    next_token = None

    while True:
        params: dict = {
            "MarketplaceIds": MARKETPLACE_ID,
            "CreatedAfter":   created_after,
            "OrderStatuses":  "Shipped,Unshipped,PartiallyShipped,Pending",
        }
        if next_token:
            params["NextToken"] = next_token
        try:
            data = spapi_get(token, "/orders/v0/orders", params)
        except requests.HTTPError as exc:
            log.warning("Orders API unavailable (%s) — sales_30d will not be updated.", exc)
            return {}
        payload    = data.get("payload", {})
        orders.extend(payload.get("Orders", []))
        next_token = payload.get("NextToken")
        if not next_token:
            break

    log.info("Fetched %d orders — counting units per ASIN …", len(orders))

    asin_units: dict[str, int] = {}
    delay = 1.0  # seconds between requests; grows by 0.1 s on every 429

    for i, order in enumerate(orders):
        order_id = order.get("AmazonOrderId", "")
        if not order_id:
            continue

        for _attempt in range(10):
            try:
                data  = spapi_get(token, f"/orders/v0/orders/{order_id}/orderItems")
                time.sleep(delay)
                for item in data.get("payload", {}).get("OrderItems", []):
                    asin = item.get("ASIN", "")
                    qty  = int(item.get("QuantityOrdered", 0))
                    if asin:
                        asin_units[asin] = asin_units.get(asin, 0) + qty
                break
            except requests.HTTPError as exc:
                if exc.response is not None and exc.response.status_code == 429:
                    delay = round(delay + 0.1, 1)
                    log.debug("Rate limited — delay now %.1fs, retrying %s …", delay, order_id)
                    time.sleep(delay)
                else:
                    log.warning("Failed to fetch items for order %s: %s", order_id, exc)
                    break

        if (i + 1) % 20 == 0:
            log.info("  Sales data: %d/%d orders processed (delay=%.1fs) …",
                     i + 1, len(orders), delay)

    log.info("Sales data complete: %d ASINs with units sold in last %d days.", len(asin_units), days)
    return asin_units


def fetch_active_listings(token: str) -> tuple[list[dict], set[str]]:
    """
    Pages through all ACTIVE listings for this seller.

    Returns (listings, parent_asins) where:
      listings     — purchasable child ASINs with price/image data
      parent_asins — ASINs with no offers (variation-family parents).
                     Any .md file found for these should be deleted.
    """
    log.info("Fetching active listings (summaries + offers) …")
    listings: list[dict] = []
    parent_asins: set[str] = set()
    page_token = None

    while True:
        params: dict = {
            "marketplaceIds": MARKETPLACE_ID,
            "includedData":   "summaries,offers,fulfillmentAvailability,attributes",
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

            offers = item.get("offers", [])
            if not offers:
                # Variation-family parent: not purchasable, no SKU we own.
                # Track it so main() can delete any stale .md file for it.
                parent_asins.add(asin)
                continue

            # offers — our own listing price (no separate pricing API call needed)
            price_str = _extract_price(offers)

            # mainImage — Amazon CDN URL for the product image
            image_url = summary.get("mainImage", {}).get("link", "")

            # attributes — extra images and product description
            attrs = item.get("attributes", {})

            # Collect up to 4 additional images (other_product_image_locator_1..4)
            extra_images: list[str] = []
            for i in range(1, 5):
                locators = attrs.get(f"other_product_image_locator_{i}", [])
                if locators:
                    url = locators[0].get("media_location", "")
                    if url:
                        extra_images.append(url)

            # Long-form product description from attributes
            desc_entries = attrs.get("product_description", [])
            description = desc_entries[0].get("value", "") if desc_entries else ""

            # Stock is resolved in main() from the FBA inventory dict.
            # Placeholder 0 here; main() overwrites it.
            stock = 0

            listings.append({
                "asin":        asin,
                "sku":         sku,
                "title":       title,
                "price":       price_str,    # may be None if listing has no active offer
                "image":       image_url,    # may be "" if no image in API response
                "images":      extra_images, # additional gallery images
                "description": description,  # long-form product description
                "stock":       stock,        # total units available (0 = out of stock)
            })

        page_token = data.get("pagination", {}).get("nextToken")
        if not page_token:
            break

    log.info("Found %d purchasable listings, %d parent ASINs skipped.",
             len(listings), len(parent_asins))
    return listings, parent_asins


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
        lp = offer.get("price", {})
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

    Two-pass approach:
      Pass 1 — index all consolidated variant files (variants: block with
               indented  asin:  entries).
      Pass 2 — index single-asin files, but delete any whose ASIN is already
               covered by a consolidated variant file (they are duplicates
               created by a previous script run).
    """
    index: dict[str, dict] = {}
    single_files: list[tuple[Path, str]] = []

    for md_path in sorted(PRODUCTS_DIR.glob("*.md")):
        text = md_path.read_text(encoding="utf-8")
        _, fm_raw, _ = parse_front_matter(text)
        if not fm_raw:
            continue

        # Single top-level  asin: B0XXXXXXXX  (no leading spaces)
        single = re.search(r"^asin:\s*[\"']?([A-Z0-9]{10})[\"']?", fm_raw, re.MULTILINE)
        if single:
            single_files.append((md_path, single.group(1)))
            continue

        # Variant files: asin: appears indented under a variants: block.
        # Supports both  "- label: ...\n    asin: ..."  and  "- asin: ..."  formats.
        variant_asins = re.findall(
            r"^\s{2,}asin:\s*[\"']?([A-Z0-9]{10})[\"']?",
            fm_raw,
            re.MULTILINE,
        )
        for i, asin in enumerate(variant_asins):
            index[asin] = {"path": md_path, "type": "variant", "variant_index": i}

    # Pass 2: index single files; delete duplicates already covered by a variant file.
    deleted = 0
    for md_path, asin in single_files:
        if asin in index:
            md_path.unlink()
            log.info(
                "Deleted duplicate file %s (ASIN %s is a variant in %s)",
                md_path.name, asin, index[asin]["path"].name,
            )
            deleted += 1
        else:
            index[asin] = {"path": md_path, "type": "single", "variant_index": None}

    if deleted:
        log.info("Cleaned up %d duplicate single-product files.", deleted)

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


def update_stock_in_fm(fm_raw: str, stock: int, variant_index: int | None) -> str:
    """Updates (or inserts) the stock field in the raw front matter string."""
    if variant_index is None:
        if re.search(r'^stock:', fm_raw, re.MULTILINE):
            return re.sub(
                r'^(stock:\s*)\d*',
                f'stock: {stock}',
                fm_raw, count=1, flags=re.MULTILINE,
            )
        # Insert after price: line
        return re.sub(
            r'^(price:.*)',
            rf'\1\nstock: {stock}',
            fm_raw, count=1, flags=re.MULTILINE,
        )
    else:
        variant_blocks = list(re.finditer(
            r"(  - (?:label|asin):.*?)(?=\n  - (?:label|asin):|\Z)",
            fm_raw, re.DOTALL,
        ))
        if variant_index >= len(variant_blocks):
            return fm_raw
        bm = variant_blocks[variant_index]
        block = bm.group(0)
        if re.search(r'^\s{4}stock:', block, re.MULTILINE):
            new_block = re.sub(
                r'(    stock:\s*)\d*',
                f'    stock: {stock}',
                block, count=1, flags=re.MULTILINE,
            )
        else:
            new_block = block.rstrip() + f'\n    stock: {stock}'
        return fm_raw[: bm.start()] + new_block + fm_raw[bm.end():]


def update_sales_in_fm(fm_raw: str, sales: int) -> str:
    """Updates (or inserts) the top-level sales_30d field."""
    if re.search(r'^sales_30d:', fm_raw, re.MULTILINE):
        return re.sub(
            r'^(sales_30d:\s*)\d*',
            f'sales_30d: {sales}',
            fm_raw, count=1, flags=re.MULTILINE,
        )
    # Insert after hot: if present, else after featured:
    for anchor in (r'^(hot:.*)', r'^(featured:.*)'):
        if re.search(anchor, fm_raw, re.MULTILINE):
            return re.sub(anchor, rf'\1\nsales_30d: {sales}',
                          fm_raw, count=1, flags=re.MULTILINE)
    return fm_raw + f'\nsales_30d: {sales}'


def update_url_in_fm(fm_raw: str, asin: str, variant_index: int | None) -> str:
    """Ensures the Amazon product URL uses the clean https://www.amazon.co.uk/dp/{ASIN} format."""
    url = f"https://www.amazon.co.uk/dp/{asin}"
    if variant_index is None:
        return re.sub(
            r'^(amazon_url:\s*)["\']?[^\'"#\n]*["\']?',
            f'amazon_url: "{url}"',
            fm_raw,
            count=1,
            flags=re.MULTILINE,
        )
    else:
        variant_blocks = list(re.finditer(
            r"(  - (?:label|asin):.*?)(?=\n  - (?:label|asin):|\Z)",
            fm_raw,
            re.DOTALL,
        ))
        if variant_index >= len(variant_blocks):
            return fm_raw
        bm = variant_blocks[variant_index]
        new_block = re.sub(
            r'(    url:\s*)["\']?[^\'"#\n]*["\']?',
            f'    url: "{url}"',
            bm.group(0),
            count=1,
            flags=re.MULTILINE,
        )
        return fm_raw[: bm.start()] + new_block + fm_raw[bm.end():]


def fix_main_image_if_local(fm_raw: str, image_url: str) -> str:
    """
    For variant product files: if the top-level image: field is a local path
    (not an http URL), replace it with image_url pulled from a variant.
    Safe to call on every variant pass — once the field is an http URL it won't
    be touched again.
    """
    m = re.search(r'^image:\s*["\']?(.*?)["\']?\s*(?:#[^\n]*)?$', fm_raw, re.MULTILINE)
    if m:
        current = m.group(1).strip()
        if current.startswith("http"):
            return fm_raw  # already a live URL — leave it alone
    return re.sub(
        r'^(image:\s*)["\']?[^"\'#\n]*["\']?(\s*#[^\n]*)?',
        f'image: "{image_url}"',
        fm_raw,
        count=1,
        flags=re.MULTILINE,
    )


def update_image_in_fm(fm_raw: str, image_url: str, variant_index: int | None) -> str:
    """Updates (or inserts) the image field in the raw front matter string."""
    if variant_index is None:
        # Single product: replace top-level image: value, preserving any trailing comment
        return re.sub(
            r'^(image:\s*)["\']?[^"\'#\n]*["\']?(\s*#[^\n]*)?',
            f'image: "{image_url}"',
            fm_raw,
            count=1,
            flags=re.MULTILINE,
        )
    else:
        # Variant: add or update image: in the Nth variant block
        variant_blocks = list(re.finditer(
            r"(  - (?:label|asin):.*?)(?=\n  - (?:label|asin):|\Z)",
            fm_raw,
            re.DOTALL,
        ))
        if variant_index >= len(variant_blocks):
            return fm_raw
        bm = variant_blocks[variant_index]
        block = bm.group(0)
        if re.search(r'^\s{4}image:', block, re.MULTILINE):
            new_block = re.sub(
                r'(    image:\s*)["\']?[^"\'#\n]*["\']?(\s*#[^\n]*)?',
                f'    image: "{image_url}"',
                block,
                count=1,
                flags=re.MULTILINE,
            )
        else:
            new_block = block.rstrip() + f'\n    image: "{image_url}"'
        return fm_raw[: bm.start()] + new_block + fm_raw[bm.end():]


def update_images_list_in_fm(fm_raw: str, image_urls: list[str]) -> str:
    """
    Updates (or inserts) the top-level images: list in front matter.
    This is a page-level field used by the product image carousel —
    it is not per-variant.
    """
    if not image_urls:
        return fm_raw
    yaml_lines = "images:\n" + "".join(f'  - "{u}"\n' for u in image_urls)
    yaml_block = yaml_lines.rstrip("\n")

    # Replace existing multi-line images: block
    existing = re.search(r"^images:(?:\n  - [^\n]+)+", fm_raw, re.MULTILINE)
    if existing:
        return fm_raw[: existing.start()] + yaml_block + fm_raw[existing.end():]

    # Replace simple  images: []  or  images: ""  on one line
    existing_simple = re.search(r"^images:.*$", fm_raw, re.MULTILINE)
    if existing_simple:
        return fm_raw[: existing_simple.start()] + yaml_block + fm_raw[existing_simple.end():]

    # Insert after the top-level image: line
    return re.sub(
        r"^(image:.*)",
        rf"\1\n{yaml_block}",
        fm_raw, count=1, flags=re.MULTILINE,
    )


def write_updated_file(md_path: Path, fm_raw: str, body: str) -> None:
    """Writes the reconstructed markdown file back to disk."""
    md_path.write_text(f"---\n{fm_raw}\n---\n{body}", encoding="utf-8")


# ===========================================================================
# Step 5 — Create a new product file for an unmatched ASIN
# ===========================================================================

def _yaml_str(value: str) -> str:
    """Wraps a value in double quotes, escaping any embedded double quotes."""
    return '"' + value.replace('"', '\\"') + '"'


def _extract_variant_label(title: str) -> str:
    """
    Extracts a short variant label from an Amazon title.
    Amazon typically appends the variant info in trailing parentheses,
    e.g. "(2M)", "(3)", "(0.5mm Diameter, 30g)".
    Falls back to the first 40 characters of the title.
    """
    m = re.search(r'\(([^)]+)\)\s*$', title)
    if m:
        return m.group(1)
    return title[:40].strip()


def _find_likely_parent(new_slug: str, asin_index: dict) -> Path | None:
    """
    Returns the path of an existing variant (or single) file whose slug shares
    at least 30 characters of common prefix with new_slug.
    Checks variant files first, then single-product files.
    """
    MIN_COMMON = 30
    variant_paths = {e["path"] for e in asin_index.values() if e["type"] == "variant"}
    single_paths  = {e["path"] for e in asin_index.values() if e["type"] == "single"}

    for md_path in sorted(variant_paths | single_paths):
        common = len(os.path.commonprefix([new_slug, md_path.stem]))
        if common >= MIN_COMMON:
            return md_path
    return None


def auto_add_variant(md_path: Path, asin: str, title: str,
                     price: str | None, image_url: str, stock: int) -> bool:
    """
    Appends a new variant to an existing product file.

    - If the file already has a variants: block, the new entry is appended.
    - If it is a single-product file, it is converted to variant format:
        the existing top-level asin/amazon_url/stock fields are moved into
        a variants: block as the first entry, and the new ASIN becomes the
        second entry.

    Returns True on success, False if the file could not be parsed.
    """
    text = md_path.read_text(encoding="utf-8")
    _, fm_raw, body = parse_front_matter(text)
    if not fm_raw:
        return False

    url       = f"https://www.amazon.co.uk/dp/{asin}"
    label     = _extract_variant_label(title)
    price_val = price or ""

    new_entry = (
        f'  - label: "{label}"\n'
        f'    asin: "{asin}"\n'
        f'    price: "{price_val}"\n'
        f'    url: "{url}"\n'
        f'    image: "{image_url}"\n'
        f'    stock: {stock}'
    )

    # ── Case 1: variants: block already exists — just append ──────────────
    if re.search(r'^variants:', fm_raw, re.MULTILINE):
        new_fm = fm_raw + '\n' + new_entry
        write_updated_file(md_path, new_fm, body)
        return True

    # ── Case 2: single-product file — convert to variant format ───────────
    def _get(pattern):
        m = re.search(pattern, fm_raw, re.MULTILINE)
        return m.group(1).strip() if m else ""

    existing_title  = _get(r'^title:\s*["\']?(.*?)["\']?\s*(?:#[^\n]*)?$')
    existing_asin   = _get(r'^asin:\s*["\']?([A-Z0-9]{10})["\']?')
    existing_price  = _get(r'^price:\s*["\']?([^"\'#\n]*)["\']?')
    existing_image  = _get(r'^image:\s*["\']?([^"\'#\n]*)["\']?')
    existing_url    = _get(r'^amazon_url:\s*["\']?([^"\'#\n]*)["\']?') \
                      or f"https://www.amazon.co.uk/dp/{existing_asin}"
    existing_stock_m = re.search(r'^stock:\s*(\d+)', fm_raw, re.MULTILINE)
    existing_stock  = int(existing_stock_m.group(1)) if existing_stock_m else 0
    existing_label  = _extract_variant_label(existing_title)

    first_entry = (
        f'  - label: "{existing_label}"\n'
        f'    asin: "{existing_asin}"\n'
        f'    price: "{existing_price}"\n'
        f'    url: "{existing_url}"\n'
        f'    image: "{existing_image}"\n'
        f'    stock: {existing_stock}'
    )

    # Remove single-product-only top-level fields
    new_fm = re.sub(r'^asin:.*\n',       '', fm_raw,  flags=re.MULTILINE)
    new_fm = re.sub(r'^amazon_url:.*\n', '', new_fm,  flags=re.MULTILINE)
    new_fm = re.sub(r'^stock:.*\n',      '', new_fm,  flags=re.MULTILINE)

    # Update top-level price to "From £X.XX"
    try:
        ep = float(re.sub(r'[^\d.]', '', existing_price))
        np = float(re.sub(r'[^\d.]', '', price_val))
        sym = '£' if '£' in existing_price else ''
        from_price = f'From {sym}{min(ep, np):.2f}'
    except (ValueError, TypeError):
        from_price = existing_price
    new_fm = re.sub(
        r'^(price:\s*)["\']?[^"\'#\n]*["\']?',
        f'price: "{from_price}"',
        new_fm, count=1, flags=re.MULTILINE,
    )

    new_fm = new_fm.rstrip() + f'\nvariants:\n{first_entry}\n{new_entry}'
    write_updated_file(md_path, new_fm, body)
    return True


def create_product_file(asin: str, title: str, price: str | None, image_url: str = "", extra_images: list | None = None, description: str = "", stock: int = 0) -> Path | None:
    """
    Generates a new _products/{slug}.md from the standard template.
    Title, price, and image URL come directly from the listings API response.

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
        log.warning(
            "File %s already exists and was not matched as a variant parent — "
            "ASIN %s (%r) was not added. Check manually.",
            md_path.name, asin, title,
        )
        return None

    category   = assign_category(title)
    price_str  = price or ""
    amazon_url = f"https://www.amazon.co.uk/dp/{asin}"

    images_yaml = ""
    if extra_images:
        lines = "\n".join(f'  - "{u}"' for u in extra_images)
        images_yaml = f"\nimages:\n{lines}"

    body = description.strip() + "\n" if description else "\n"

    content = f"""\
---
layout: product
title: {_yaml_str(title)}
description: "" # TODO: add description
category: "{category}"
price: "{price_str}"
stock: {stock}
amazon_url: "{amazon_url}"
asin: "{asin}"
image: "{image_url}"{images_yaml}
featured: false
sales_30d: 0
badge: ""
tags: []
---
{body}"""
    md_path.write_text(content, encoding="utf-8")
    return md_path


# ===========================================================================
# Main
# ===========================================================================

def main():
    token                  = get_access_token()
    listings, parent_asins = fetch_active_listings(token)
    fba_inventory          = fetch_fba_inventory(token)
    asin_sales             = fetch_order_sales(token)
    asin_index             = load_product_files()

    # Auto-delete any .md files that exist for parent ASINs.
    # Parents are variation-family roots: not purchasable, no offers.
    # Their children (the real variants) handle the product pages.
    for asin in parent_asins:
        if asin in asin_index and asin_index[asin]["type"] == "single":
            path = asin_index[asin]["path"]
            path.unlink()
            log.info("Deleted parent-ASIN file %s  (ASIN %s)", path.name, asin)
            del asin_index[asin]

    # De-duplicate: if the same ASIN appears more than once in listings (can
    # happen when the same product has multiple SKUs), process it only once.
    seen: set[str] = set()

    checked  = 0
    updated  = 0
    created  = 0
    no_price = 0
    skipped  = 0

    for listing in listings:
        asin        = listing["asin"]
        price       = listing["price"]
        title       = listing["title"]
        image_url   = listing["image"]
        extra_imgs  = listing["images"]
        description = listing["description"]
        # Use real FBA fulfillable quantity; fall back to 1 if the inventory
        # API returned nothing (e.g. role not yet propagated) so active
        # listings still appear on the shop.
        stock = fba_inventory.get(asin, 1) if fba_inventory else 1

        if asin in seen:
            continue
        seen.add(asin)
        checked += 1

        # ----------------------------------------------------------------
        # Case A — existing .md file found: update price and image
        # ----------------------------------------------------------------
        if asin in asin_index:
            if price is None and not image_url:
                log.debug("No price or image for %s — skipping update.", asin)
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

            modified_fm   = fm_raw
            modified_body = body
            if price:
                modified_fm = update_price_in_fm(modified_fm, price, v_idx)
            if image_url:
                modified_fm = update_image_in_fm(modified_fm, image_url, v_idx)
                if v_idx is not None:
                    modified_fm = fix_main_image_if_local(modified_fm, image_url)
            modified_fm = update_url_in_fm(modified_fm, asin, v_idx)
            modified_fm = update_stock_in_fm(modified_fm, stock, v_idx)

            # Images list and body description are page-level (not per-variant).
            # For variant files only update on the first variant (index 0) to
            # avoid overwriting with a different variant's content.
            if v_idx is None or v_idx == 0:
                if extra_imgs:
                    modified_fm = update_images_list_in_fm(modified_fm, extra_imgs)
                if description:
                    modified_body = description.strip() + "\n"

            if modified_fm == fm_raw and modified_body == body:
                log.info("No change: %s  (ASIN %s)", md_path.name, asin)
                continue

            write_updated_file(md_path, modified_fm, modified_body)
            log.info("Updated  %s  (ASIN %s)  price=%s", md_path.name, asin, price)
            updated += 1

        # ----------------------------------------------------------------
        # Case B — no matching .md file: auto-add variant or create new
        # ----------------------------------------------------------------
        else:
            new_slug     = make_slug(title)
            exact_path   = PRODUCTS_DIR / f"{new_slug}.md"
            parent_path  = exact_path if exact_path.exists() else _find_likely_parent(new_slug, asin_index)

            if parent_path:
                label = _extract_variant_label(title)
                if auto_add_variant(parent_path, asin, title, price or "", image_url, stock):
                    log.info(
                        "Auto-added variant %r to %s  (ASIN %s  price=%s)",
                        label, parent_path.name, asin, price or "—",
                    )
                    updated += 1
                else:
                    log.warning(
                        "Could not auto-add ASIN %s to %s — front matter unparseable",
                        asin, parent_path.name,
                    )
                    skipped += 1
                continue

            new_file = create_product_file(asin, title, price, image_url, extra_imgs, description, stock)
            if new_file:
                log.info("Created  %s  (ASIN %s)  price=%s", new_file.name, asin, price or "—")
                created += 1
            else:
                log.warning(
                    "Skipped ASIN %s (%r) — slug %r already exists but could not be matched",
                    asin, title, new_slug,
                )
                skipped += 1

    # ----------------------------------------------------------------
    # Update sales_30d on every product file (totalled across variants)
    # ----------------------------------------------------------------
    if asin_sales:
        # Sum sales for all ASINs that map to the same file
        file_sales: dict[Path, int] = {}
        for asin, entry in asin_index.items():
            file_sales[entry["path"]] = (
                file_sales.get(entry["path"], 0) + asin_sales.get(asin, 0)
            )
        sales_updated = 0
        for md_path, total_sales in sorted(file_sales.items()):
            text = md_path.read_text(encoding="utf-8")
            _, fm_raw, body = parse_front_matter(text)
            if not fm_raw:
                continue
            new_fm = update_sales_in_fm(fm_raw, total_sales)
            if new_fm != fm_raw:
                write_updated_file(md_path, new_fm, body)
                sales_updated += 1
        log.info("Updated sales_30d on %d product files.", sales_updated)

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
