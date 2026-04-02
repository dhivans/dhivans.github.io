#!/usr/bin/env python3
"""
sync_products.py — Pulls live data from Amazon SP-API and keeps
Jekyll product markdown files in _products/ in sync.

What it does:
  1. Fetches all ACTIVE listings for your seller account (includes price).
  2. Indexes every existing _products/*.md file by ASIN.
  3. For each listing that already has a matching .md file:
       - Updates the price from the listings data (no separate pricing call).
       - Updates the main image URL from the Catalog API.
       - Skips writing the file if nothing changed.
  4. For each listing with NO matching .md file:
       - Calls the Catalog API to fetch title, description, and image.
       - Generates a slug and assigns a category from title keywords.
       - Creates a new _products/{slug}.md from the standard template.
       - Never overwrites an existing file.
  5. Prints a summary: checked / updated / created / unmatched.

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
# Step 1 — Fetch all active listings (includes price, no separate pricing call)
# ===========================================================================

def fetch_active_listings(token: str) -> list[dict]:
    """
    Pages through all ACTIVE listings for this seller using the Listings Items
    endpoint.  We request both 'summaries' (asin, title) and 'offers'
    (listingPrice) so we never need to call the competitive pricing endpoint.

    Valid includedData values for 2021-08-01:
      summaries, attributes, offers, issues, fulfillmentAvailability, procurement
    Note: 'prices' is NOT a valid value here — it causes a 400 error.

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

            # offers — our own listing price (avoids pricing API permission issues)
            price_str = _extract_price(item.get("offers", []))

            listings.append({
                "asin":  asin,
                "sku":   sku,
                "title": title,
                "price": price_str,   # may be None if not returned
            })

        page_token = data.get("pagination", {}).get("nextToken")
        if not page_token:
            break

    log.info("Found %d active listings.", len(listings))
    return listings


def _extract_price(offers_list: list) -> str | None:
    """
    Parses the 'offers' array from a Listings Items 2021-08-01 response and
    returns a formatted price string like "£9.99", or None if unavailable.

    The offers array structure:
      [{"marketplaceId": "...", "offerType": "B2C",
        "price": {"listingPrice": {"amount": 9.99, "currencyCode": "GBP"}}}]

    We prefer the B2C offer; fall back to the first offer if no B2C entry.
    """
    b2c_offer = None
    first_offer = None

    for offer in offers_list:
        price_block = offer.get("price", {})
        lp = price_block.get("listingPrice", {})
        amount   = lp.get("amount")
        currency = lp.get("currencyCode", "GBP")
        if amount is None:
            continue
        entry = (float(amount), currency)
        if first_offer is None:
            first_offer = entry
        if offer.get("offerType") == "B2C":
            b2c_offer = entry
            break  # found preferred offer

    chosen = b2c_offer or first_offer
    if chosen is None:
        return None
    amount, currency = chosen
    symbol = "£" if currency == "GBP" else f"{currency} "
    return f"{symbol}{amount:.2f}"


# ===========================================================================
# Step 2 — Catalog Items API (image + description + title for new files)
# ===========================================================================

def fetch_catalog_data(token: str, asin: str) -> dict:
    """
    Calls the Catalog Items API to get the main image, product description,
    and title for a given ASIN.

    GET /catalog/2022-04-01/items/{asin}
      ?marketplaceIds=...&includedData=images,attributes,summaries

    Returns a dict:
      {
        "image":       "https://m.media-amazon.com/..." | None,
        "title":       "Product Title"                  | None,
        "description": "First 2-3 sentences."           | "",
      }
    """
    result = {"image": None, "title": None, "description": ""}
    try:
        data = spapi_get(
            token,
            f"/catalog/2022-04-01/items/{asin}",
            {
                "marketplaceIds": MARKETPLACE_ID,
                "includedData":   "images,attributes,summaries",
            },
        )
    except requests.HTTPError as exc:
        log.warning("Catalog fetch failed for ASIN %s: %s", asin, exc)
        return result

    # -- Image --
    for image_set in data.get("images", []):
        for img in image_set.get("images", []):
            if img.get("variant") == "MAIN":
                result["image"] = img.get("link")
                break
        if result["image"]:
            break

    # -- Title (from summaries) --
    summaries = data.get("summaries", [{}])
    if summaries:
        result["title"] = summaries[0].get("itemName") or summaries[0].get("itemClassificationName")

    # -- Description (from attributes) --
    attrs = data.get("attributes", {})

    # Try product_description first, then bullet_points as fallback
    desc_list   = attrs.get("product_description", [])
    bullet_list = attrs.get("bullet_point", [])

    raw_desc = ""
    if desc_list:
        raw_desc = desc_list[0].get("value", "") if isinstance(desc_list[0], dict) else str(desc_list[0])
    elif bullet_list:
        # Join up to 3 bullet points into a short description
        bullets = []
        for bp in bullet_list[:3]:
            val = bp.get("value", "") if isinstance(bp, dict) else str(bp)
            if val:
                bullets.append(val.rstrip(".") + ".")
        raw_desc = " ".join(bullets)

    result["description"] = _trim_to_sentences(raw_desc, max_sentences=3)
    return result


def _trim_to_sentences(text: str, max_sentences: int = 3) -> str:
    """Trims a long description string to at most max_sentences sentences."""
    if not text:
        return ""
    # Split on sentence-ending punctuation followed by whitespace or end-of-string
    sentences = re.split(r"(?<=[.!?])\s+", text.strip())
    trimmed = " ".join(sentences[:max_sentences])
    return trimmed


# ===========================================================================
# Step 3 — Slug and category helpers
# ===========================================================================

def make_slug(title: str) -> str:
    """
    Converts a product title into a URL-safe file slug.
    e.g. "ESP32 WROOM-32 Dev Board (Wi-Fi)" → "esp32-wroom-32-dev-board-wi-fi"
    """
    # Normalise unicode characters to ASCII equivalents
    title = unicodedata.normalize("NFKD", title)
    title = title.encode("ascii", "ignore").decode("ascii")
    title = title.lower()
    # Replace any non-alphanumeric character (except hyphens) with a hyphen
    title = re.sub(r"[^a-z0-9]+", "-", title)
    # Collapse multiple hyphens and strip leading/trailing hyphens
    title = re.sub(r"-{2,}", "-", title).strip("-")
    # Truncate to 80 characters at a word boundary
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
# Step 4 — Parse and index existing .md files
# ===========================================================================

def parse_front_matter(text: str) -> tuple[str, str, str]:
    """
    Splits a Jekyll markdown file into (full_match, front_matter_raw, body).
    Returns ("", "", text) if no front matter delimiters are found.
    Keeping the raw FM string avoids a full YAML round-trip that would strip
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
# Step 5 — In-place front matter surgery (update existing files)
# ===========================================================================

def update_price_in_fm(fm_raw: str, new_price: str, variant_index: int | None) -> str:
    """Updates the price field in the raw front matter string."""
    if variant_index is None:
        # Top-level price: "..."
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


def update_image_in_fm(fm_raw: str, new_image: str) -> str:
    """
    Updates the top-level image: field.
    Preserves any trailing comment (e.g. # Add image file here).
    """
    def replacer(m):
        comment = m.group(1) or ""
        return f'image: "{new_image}"{comment}'

    return re.sub(
        r'^image:\s*["\']?[^"\'#\n]*["\']?(\s*#.*)?$',
        replacer,
        fm_raw,
        count=1,
        flags=re.MULTILINE,
    )


def write_updated_file(md_path: Path, fm_raw: str, body: str) -> None:
    """Writes the reconstructed markdown file back to disk."""
    md_path.write_text(f"---\n{fm_raw}\n---\n{body}", encoding="utf-8")


# ===========================================================================
# Step 6 — Create a new product file for an unmatched ASIN
# ===========================================================================

def _yaml_str(value: str) -> str:
    """Wraps a value in double quotes, escaping any embedded double quotes."""
    return '"' + value.replace('"', '\\"') + '"'


def create_product_file(
    asin: str,
    price: str | None,
    catalog: dict,
) -> Path | None:
    """
    Generates a new _products/{slug}.md from the standard template.
    Returns the Path of the created file, or None if the file already exists
    (safety guard) or the slug could not be determined.
    """
    title = catalog.get("title") or ""
    if not title:
        # Fall back to the ASIN as a placeholder title
        title = f"Product {asin}"

    slug = make_slug(title)
    if not slug:
        log.warning("Could not generate slug for ASIN %s (title: %r)", asin, title)
        return None

    md_path = PRODUCTS_DIR / f"{slug}.md"
    if md_path.exists():
        # Never overwrite — this ASIN just wasn't indexed (e.g. missing asin: field)
        log.warning("File already exists, skipping creation: %s", md_path.name)
        return None

    description = catalog.get("description") or ""
    image       = catalog.get("image")   or ""
    category    = assign_category(title)
    price_str   = price or ""
    amazon_url  = f"https://www.amazon.co.uk/dp/{asin}"

    # Build the description line — add a TODO comment if empty
    if description:
        desc_line = f"description: {_yaml_str(description)}"
    else:
        desc_line = 'description: "" # TODO: add description'

    content = f"""\
---
layout: product
title: {_yaml_str(title)}
{desc_line}
category: "{category}"
price: "{price_str}"
amazon_url: "{amazon_url}"
asin: "{asin}"
image: "{image}"
featured: false
hot: false
badge: ""
tags: []
---

{description}
"""

    md_path.write_text(content, encoding="utf-8")
    return md_path


# ===========================================================================
# Main
# ===========================================================================

def main():
    token    = get_access_token()
    listings = fetch_active_listings(token)
    asin_index = load_product_files()

    checked   = 0
    updated   = 0
    created   = 0
    no_price  = 0
    unmatched_no_catalog: list[str] = []

    # Cache: md_path → image URL (one Catalog API call per file, not per ASIN)
    image_cache: dict[Path, str | None] = {}

    for listing in listings:
        asin  = listing["asin"]
        price = listing["price"]   # extracted from listings data — no pricing API call
        checked += 1

        # ----------------------------------------------------------------
        # Case A — existing .md file found: update price + image
        # ----------------------------------------------------------------
        if asin in asin_index:
            entry   = asin_index[asin]
            md_path = entry["path"]
            v_idx   = entry["variant_index"]

            if price is None:
                log.warning("No price in listings data for %s — skipping update.", asin)
                no_price += 1
                continue

            # Fetch image via Catalog API (cached per file)
            if md_path not in image_cache:
                cat = fetch_catalog_data(token, asin)
                image_cache[md_path] = cat["image"]
            new_image = image_cache[md_path]

            original = md_path.read_text(encoding="utf-8")
            _, fm_raw, body = parse_front_matter(original)
            if not fm_raw:
                log.warning("Could not parse front matter in %s", md_path.name)
                continue

            modified_fm = update_price_in_fm(fm_raw, price, v_idx)
            if new_image and new_image.startswith("https://"):
                modified_fm = update_image_in_fm(modified_fm, new_image)

            if modified_fm == fm_raw:
                log.info("No change: %s  (ASIN %s)", md_path.name, asin)
                continue

            write_updated_file(md_path, modified_fm, body)
            log.info("Updated  %s  (ASIN %s)  price=%s", md_path.name, asin, price)
            updated += 1

        # ----------------------------------------------------------------
        # Case B — no matching .md file: create it
        # ----------------------------------------------------------------
        else:
            log.info("Creating new file for ASIN %s …", asin)
            catalog = fetch_catalog_data(token, asin)

            if not catalog["title"] and not catalog["image"]:
                log.warning("Catalog returned no data for ASIN %s — skipping.", asin)
                unmatched_no_catalog.append(asin)
                continue

            new_file = create_product_file(asin, price, catalog)
            if new_file:
                log.info("Created  %s  (ASIN %s)", new_file.name, asin)
                created += 1
            else:
                unmatched_no_catalog.append(asin)

    # ----------------------------------------------------------------
    # Summary
    # ----------------------------------------------------------------
    print("\n" + "=" * 62)
    print(f"  Products checked          : {checked}")
    print(f"  Existing files updated    : {updated}")
    print(f"  New files created         : {created}")
    print(f"  Skipped (no price data)   : {no_price}")
    print(f"  Skipped (no catalog data) : {len(unmatched_no_catalog)}")
    if unmatched_no_catalog:
        print("\n  ASINs with no catalog data (may need manual files):")
        for a in unmatched_no_catalog:
            print(f"    {a}")
    print("=" * 62)


if __name__ == "__main__":
    main()
