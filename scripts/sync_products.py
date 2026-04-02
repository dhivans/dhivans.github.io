#!/usr/bin/env python3
"""
sync_products.py — Pulls live price + image data from Amazon SP-API
and updates Jekyll product markdown files in _products/.

Requires environment variables (never hardcoded):
  AMAZON_CLIENT_ID       — SP-API app client ID (LWA)
  AMAZON_CLIENT_SECRET   — SP-API app client secret (LWA)
  AMAZON_REFRESH_TOKEN   — LWA refresh token for your seller account
  AMAZON_MARKETPLACE_ID  — e.g. A1F83G8C2ARO7P  (UK marketplace)
  AMAZON_SELLER_ID       — your Seller Central merchant ID

Run locally:
  export AMAZON_CLIENT_ID=...
  python scripts/sync_products.py
"""

import os
import re
import sys
import json
import logging
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

# SP-API base URL — UK endpoint (eu-west-1 region)
SPAPI_BASE = "https://sellingpartnerapi-eu.amazon.com"

# _products/ directory (relative to this script's repo root)
PRODUCTS_DIR = Path(__file__).resolve().parent.parent / "_products"


# ---------------------------------------------------------------------------
# Step 1 — Get a fresh LWA access token
# ---------------------------------------------------------------------------
def get_access_token() -> str:
    """
    Exchange the long-lived refresh token for a short-lived access token.
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


# ---------------------------------------------------------------------------
# Helpers — authenticated SP-API request
# ---------------------------------------------------------------------------
def spapi_get(token: str, path: str, params: dict = None) -> dict:
    """
    GET request to the SP-API with the required auth headers.
    Raises requests.HTTPError on non-2xx responses.
    """
    headers = {
        "x-amz-access-token": token,
        "Content-Type": "application/json",
    }
    resp = requests.get(
        SPAPI_BASE + path,
        headers=headers,
        params=params or {},
        timeout=20,
    )
    resp.raise_for_status()
    return resp.json()


# ---------------------------------------------------------------------------
# Step 2 — Fetch all active listings (ASINs + SKUs)
# ---------------------------------------------------------------------------
def fetch_active_listings(token: str) -> list[dict]:
    """
    Uses the Listings Items endpoint to page through all ACTIVE listings
    for this seller and marketplace.

    Returns a list of dicts: [{"asin": ..., "sku": ..., "title": ...}, ...]

    SP-API docs:
      GET /listings/2021-08-01/items/{sellerId}
    """
    log.info("Fetching active listings …")
    listings = []
    page_token = None

    while True:
        params = {
            "marketplaceIds": MARKETPLACE_ID,
            "includedData":   "summaries",
            "status":         "ACTIVE",
            "pageSize":       20,
        }
        if page_token:
            params["pageToken"] = page_token

        try:
            data = spapi_get(
                token,
                f"/listings/2021-08-01/items/{SELLER_ID}",
                params,
            )
        except requests.HTTPError as exc:
            log.error("Failed to fetch listings page: %s", exc)
            break

        for item in data.get("items", []):
            sku = item.get("sku", "")
            summaries = item.get("summaries", [{}])
            summary = summaries[0] if summaries else {}
            asin = summary.get("asin", "")
            title = summary.get("itemName", "")
            if asin:
                listings.append({"asin": asin, "sku": sku, "title": title})

        page_token = data.get("pagination", {}).get("nextToken")
        if not page_token:
            break

    log.info("Found %d active listings.", len(listings))
    return listings


# ---------------------------------------------------------------------------
# Step 3 — Fetch live price for a single ASIN
# ---------------------------------------------------------------------------
def fetch_price(token: str, asin: str) -> str | None:
    """
    Uses the Product Pricing endpoint to get the lowest listed price for
    a given ASIN.

    GET /products/pricing/2022-05-01/items/{asin}/offers
    Returns a formatted price string like "£9.99", or None on failure.
    """
    try:
        data = spapi_get(
            token,
            f"/products/pricing/2022-05-01/items/{asin}/offers",
            {
                "marketplaceId": MARKETPLACE_ID,
                "itemCondition": "New",
            },
        )
        offers = (
            data.get("payload", {})
                .get("Offers", [])
        )
        if not offers:
            return None

        # Pick the lowest listing price across all offers
        prices = []
        for offer in offers:
            lp = offer.get("ListingPrice", {})
            amount = lp.get("Amount")
            currency = lp.get("CurrencyCode", "GBP")
            if amount is not None:
                prices.append((float(amount), currency))

        if not prices:
            return None

        amount, currency = min(prices, key=lambda x: x[0])
        symbol = "£" if currency == "GBP" else currency + " "
        return f"{symbol}{amount:.2f}"

    except requests.HTTPError as exc:
        log.warning("Price fetch failed for ASIN %s: %s", asin, exc)
        return None


# ---------------------------------------------------------------------------
# Step 4 — Fetch main product image for a single ASIN
# ---------------------------------------------------------------------------
def fetch_image(token: str, asin: str) -> str | None:
    """
    Uses the Catalog Items API (2022-04-01) to retrieve the primary
    product image URL.

    GET /catalog/2022-04-01/items/{asin}
    Returns an https://m.media-amazon.com/... URL, or None on failure.
    """
    try:
        data = spapi_get(
            token,
            f"/catalog/2022-04-01/items/{asin}",
            {
                "marketplaceIds": MARKETPLACE_ID,
                "includedData":   "images",
            },
        )
        images = data.get("images", [])
        for image_set in images:
            for img in image_set.get("images", []):
                if img.get("variant") == "MAIN":
                    return img.get("link")
        return None

    except requests.HTTPError as exc:
        log.warning("Image fetch failed for ASIN %s: %s", asin, exc)
        return None


# ---------------------------------------------------------------------------
# Step 5 — Parse and index existing markdown files
# ---------------------------------------------------------------------------
def parse_front_matter(text: str) -> tuple[dict, str, str]:
    """
    Splits a markdown file into (front_matter_text, body).
    Returns (raw_fm_str, body_str) — keeps the raw front matter as a string
    so we can do targeted in-place edits without a full YAML round-trip
    (which would destroy comments and formatting).
    """
    match = re.match(r"^---\s*\n(.*?)\n---\s*\n(.*)$", text, re.DOTALL)
    if not match:
        return "", "", text
    return match.group(0), match.group(1), match.group(2)


def load_product_files() -> dict[str, dict]:
    """
    Reads every .md in _products/ and builds a lookup:
      {
        asin_string: {
            "path":     Path object,
            "type":     "single" | "variant",
            "variant_index": int | None,   # for variant files
        },
        ...
      }
    Multiple ASINs from the same file each get their own entry.
    """
    index: dict[str, dict] = {}

    for md_path in sorted(PRODUCTS_DIR.glob("*.md")):
        text = md_path.read_text(encoding="utf-8")
        _, fm_raw, _ = parse_front_matter(text)
        if not fm_raw:
            continue

        # Single top-level asin:
        single = re.search(r"^asin:\s*[\"']?([A-Z0-9]{10})[\"']?", fm_raw, re.MULTILINE)
        if single:
            index[single.group(1)] = {"path": md_path, "type": "single", "variant_index": None}
            continue

        # Variant ASINs (list items with `asin:` indented)
        variant_asins = re.findall(r"^\s+-\s+.*?asin:\s*[\"']?([A-Z0-9]{10})[\"']?", fm_raw, re.MULTILINE)
        for i, asin in enumerate(variant_asins):
            index[asin] = {"path": md_path, "type": "variant", "variant_index": i}

    log.info("Indexed %d ASINs across %d product files.", len(index), len({v["path"] for v in index.values()}))
    return index


# ---------------------------------------------------------------------------
# Step 6 — Update a markdown file's front matter in-place (string surgery)
# ---------------------------------------------------------------------------
def update_price_in_fm(fm_raw: str, new_price: str, variant_index: int | None) -> str:
    """
    Replaces the price value in the raw front matter string.
    - For single-ASIN files: updates the top-level `price:` field.
    - For variant files: updates the price within the Nth variant block.
    """
    if variant_index is None:
        # Replace top-level price: "..." line
        return re.sub(
            r'^(price:\s*)["\']?.*?["\']?\s*$',
            lambda m: f'price: "{new_price}"',
            fm_raw,
            count=1,
            flags=re.MULTILINE,
        )
    else:
        # Find and replace the price inside the Nth variant
        # Variants start with "  - label:" or "  - asin:"
        variant_blocks = list(re.finditer(
            r"(  - (?:label|asin):.*?)(?=\n  - (?:label|asin):|\Z)",
            fm_raw,
            re.DOTALL,
        ))
        if variant_index >= len(variant_blocks):
            return fm_raw
        block_match = variant_blocks[variant_index]
        old_block = block_match.group(0)
        new_block = re.sub(
            r'(    price:\s*)["\']?.*?["\']?(\s*)$',
            lambda m: f'    price: "{new_price}"',
            old_block,
            count=1,
            flags=re.MULTILINE,
        )
        return fm_raw[: block_match.start()] + new_block + fm_raw[block_match.end():]


def update_image_in_fm(fm_raw: str, new_image: str) -> str:
    """
    Replaces the top-level `image:` field (always at top level, not per-variant).
    If the line has a trailing comment (e.g. # Add image file here), it is preserved.
    """
    def replacer(m):
        comment = m.group(2) or ""
        return f'image: "{new_image}"{comment}'

    return re.sub(
        r'^(image:\s*)["\']?.*?["\']?(\s*#.*?)?\s*$',
        replacer,
        fm_raw,
        count=1,
        flags=re.MULTILINE,
    )


def write_updated_file(md_path: Path, fm_raw: str, body: str, full_text: str) -> None:
    """Reconstructs and writes the markdown file."""
    new_text = f"---\n{fm_raw}\n---\n{body}"
    md_path.write_text(new_text, encoding="utf-8")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    token   = get_access_token()
    listings = fetch_active_listings(token)
    asin_index = load_product_files()

    checked  = 0
    updated  = 0
    unmatched: list[str] = []

    # Track which files we've already fetched an image for (one fetch per file,
    # using the first variant's ASIN as the representative)
    image_cache: dict[Path, str | None] = {}

    for listing in listings:
        asin = listing["asin"]
        checked += 1

        if asin not in asin_index:
            log.debug("No markdown file for ASIN %s (%s)", asin, listing.get("title", ""))
            unmatched.append(asin)
            continue

        entry   = asin_index[asin]
        md_path = entry["path"]
        v_idx   = entry["variant_index"]

        # -- Fetch live price --
        new_price = fetch_price(token, asin)
        if new_price is None:
            log.warning("Could not get price for %s — skipping.", asin)
            continue

        # -- Fetch image (cached per file) --
        if md_path not in image_cache:
            image_cache[md_path] = fetch_image(token, asin)
        new_image = image_cache[md_path]

        # -- Read current file --
        original = md_path.read_text(encoding="utf-8")
        _, fm_raw, body = parse_front_matter(original)
        if not fm_raw:
            log.warning("Could not parse front matter in %s", md_path.name)
            continue

        modified_fm = fm_raw

        # -- Update price --
        modified_fm = update_price_in_fm(modified_fm, new_price, v_idx)

        # -- Update image (only if we got one and it's an Amazon URL) --
        if new_image and new_image.startswith("https://"):
            modified_fm = update_image_in_fm(modified_fm, new_image)

        # -- Only write if something actually changed --
        if modified_fm == fm_raw:
            log.info("No change for %s (ASIN %s)", md_path.name, asin)
            continue

        write_updated_file(md_path, modified_fm, body, original)
        log.info("Updated  %s  (ASIN %s)  price=%s", md_path.name, asin, new_price)
        updated += 1

    # ---------------------------------------------------------------------------
    # Summary
    # ---------------------------------------------------------------------------
    print("\n" + "=" * 60)
    print(f"  Products checked : {checked}")
    print(f"  Files updated    : {updated}")
    print(f"  Unmatched ASINs  : {len(unmatched)}")
    if unmatched:
        print("\n  ASINs with no matching markdown file:")
        for a in unmatched:
            print(f"    {a}")
    print("=" * 60)


if __name__ == "__main__":
    main()
