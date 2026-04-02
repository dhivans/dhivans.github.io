#!/usr/bin/env python3
"""
debug_stock.py — Lists all FBA inventory with ASIN, product name, and stock level.

Run:
  . ./scripts/set_env.ps1
  python scripts/debug_stock.py
"""

import os, sys, requests

def _req(name):
    v = os.environ.get(name, "").strip()
    if not v:
        print(f"ERROR: missing {name}"); sys.exit(1)
    return v

CLIENT_ID      = _req("AMAZON_CLIENT_ID")
CLIENT_SECRET  = _req("AMAZON_CLIENT_SECRET")
REFRESH_TOKEN  = _req("AMAZON_REFRESH_TOKEN")
MARKETPLACE_ID = _req("AMAZON_MARKETPLACE_ID")
SELLER_ID      = _req("AMAZON_SELLER_ID")
SPAPI_BASE     = "https://sellingpartnerapi-eu.amazon.com"

# ── Get access token ─────────────────────────────────────────────────────────
r = requests.post(
    "https://api.amazon.co.uk/auth/o2/token",
    data={"grant_type": "refresh_token", "refresh_token": REFRESH_TOKEN,
          "client_id": CLIENT_ID, "client_secret": CLIENT_SECRET},
    timeout=15,
)
r.raise_for_status()
token = r.json()["access_token"]
headers = {"x-amz-access-token": token, "Content-Type": "application/json"}

# ── Page through all FBA inventory summaries ─────────────────────────────────
inventory = {}   # asin -> {name, fulfillable, total}
next_token = None

while True:
    params = {
        "granularityType": "Marketplace",
        "granularityId":   MARKETPLACE_ID,
        "marketplaceIds":  MARKETPLACE_ID,
        "details":         "true",
    }
    if next_token:
        params["nextToken"] = next_token

    resp = requests.get(
        f"{SPAPI_BASE}/fba/inventory/v1/summaries",
        headers=headers, params=params, timeout=20,
    )
    if resp.status_code != 200:
        print(f"FBA Inventory API error {resp.status_code}: {resp.text[:300]}")
        sys.exit(1)

    payload = resp.json().get("payload", {})
    for s in payload.get("inventorySummaries", []):
        asin = s.get("asin", "")
        if not asin:
            continue
        fulfillable = (s.get("inventoryDetails") or {}).get("fulfillableQuantity", 0) or 0
        total       = s.get("totalQuantity", 0) or 0
        name        = s.get("productName") or s.get("fnSku") or s.get("sellerSku") or "—"
        # Accumulate in case same ASIN appears under multiple SKUs
        if asin in inventory:
            inventory[asin]["fulfillable"] += fulfillable
            inventory[asin]["total"]       += total
        else:
            inventory[asin] = {"name": name, "fulfillable": fulfillable, "total": total}

    next_token = (resp.json().get("pagination") or {}).get("nextToken")
    if not next_token:
        break

# ── Also pull product names from the Listings API ────────────────────────────
# FBA summaries don't always include a readable product name, so cross-reference
# with the listings summaries which have itemName.
page_token = None
names = {}  # asin -> itemName
while True:
    lp = requests.get(
        f"{SPAPI_BASE}/listings/2021-08-01/items/{SELLER_ID}",
        headers=headers,
        params={"marketplaceIds": MARKETPLACE_ID, "includedData": "summaries",
                "status": "ACTIVE", "pageSize": 20,
                **({"pageToken": page_token} if page_token else {})},
        timeout=20,
    )
    if lp.status_code != 200:
        break
    for item in lp.json().get("items", []):
        summaries = item.get("summaries", [{}])
        asin  = (summaries[0] if summaries else {}).get("asin", "")
        iname = (summaries[0] if summaries else {}).get("itemName", "")
        if asin and iname:
            names[asin] = iname
    page_token = lp.json().get("pagination", {}).get("nextToken")
    if not page_token:
        break

# ── Print results ─────────────────────────────────────────────────────────────
# Sort: in-stock first, then by name
rows = sorted(
    inventory.items(),
    key=lambda kv: (kv[1]["fulfillable"] == 0, names.get(kv[0], kv[1]["name"])[:40])
)

col_asin  = 12
col_stock = 8
col_total = 8
col_name  = 60

print(f"\n{'ASIN':<{col_asin}}  {'AVAIL':>{col_stock}}  {'TOTAL':>{col_total}}  {'PRODUCT NAME'}")
print("-" * (col_asin + col_stock + col_total + col_name + 6))

for asin, data in rows:
    name        = names.get(asin, data["name"])
    fulfillable = data["fulfillable"]
    total       = data["total"]
    name_trunc  = name[:col_name] if len(name) > col_name else name
    stock_flag  = "  *** OUT OF STOCK" if fulfillable == 0 else ""
    print(f"{asin:<{col_asin}}  {fulfillable:>{col_stock}}  {total:>{col_total}}  {name_trunc}{stock_flag}")

print()
print(f"Total ASINs: {len(inventory)}  |  In stock: {sum(1 for v in inventory.values() if v['fulfillable'] > 0)}  |  Out of stock: {sum(1 for v in inventory.values() if v['fulfillable'] == 0)}")
