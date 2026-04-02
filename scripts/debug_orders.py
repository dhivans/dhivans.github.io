#!/usr/bin/env python3
"""
debug_orders.py — Test the Orders API and count units sold per ASIN
in the last 30 days.

Run:
  . ./scripts/set_env.ps1  (or source it)
  python scripts/debug_orders.py
"""

import os
import sys
import json
import time
import requests
from datetime import datetime, timezone, timedelta

def _require_env(name):
    value = os.environ.get(name, "").strip()
    if not value:
        print(f"ERROR: Missing environment variable: {name}")
        sys.exit(1)
    return value

CLIENT_ID      = _require_env("AMAZON_CLIENT_ID")
CLIENT_SECRET  = _require_env("AMAZON_CLIENT_SECRET")
REFRESH_TOKEN  = _require_env("AMAZON_REFRESH_TOKEN")
MARKETPLACE_ID = _require_env("AMAZON_MARKETPLACE_ID")
SPAPI_BASE     = "https://sellingpartnerapi-eu.amazon.com"


def get_access_token():
    print("Fetching access token...")
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
    print("Token obtained.\n")
    return resp.json()["access_token"]


def spapi_get(token, path, params=None):
    resp = requests.get(
        SPAPI_BASE + path,
        headers={
            "x-amz-access-token": token,
            "Content-Type": "application/json",
        },
        params=params or {},
        timeout=20,
    )
    if not resp.ok:
        print(f"  HTTP {resp.status_code}: {resp.text[:300]}")
    resp.raise_for_status()
    return resp.json()


def fetch_orders(token, days=30):
    created_after = (datetime.now(timezone.utc) - timedelta(days=days)).strftime(
        "%Y-%m-%dT%H:%M:%SZ"
    )
    print(f"Fetching orders since {created_after}...")

    orders = []
    next_token = None

    while True:
        params = {
            "MarketplaceIds": MARKETPLACE_ID,
            "CreatedAfter":   created_after,
            "OrderStatuses":  "Shipped,Unshipped,PartiallyShipped,Pending",
        }
        if next_token:
            params["NextToken"] = next_token

        try:
            data = spapi_get(token, "/orders/v0/orders", params)
        except requests.HTTPError as e:
            print(f"\nOrders API failed: {e}")
            print("This likely means the Orders role isn't active yet.")
            sys.exit(1)

        payload = data.get("payload", {})
        batch = payload.get("Orders", [])
        orders.extend(batch)
        print(f"  Page fetched: {len(batch)} orders (total so far: {len(orders)})")

        next_token = payload.get("NextToken")
        if not next_token:
            break

    print(f"\nTotal orders found: {len(orders)}")
    return orders


def fetch_order_items(token, order_id):
    """Fetch order items with retry on 429 (rate limit).
    SP-API Order Items: burst=30, restore=0.5 req/s → must pace at 2s/req.
    """
    for attempt in range(8):
        try:
            data = spapi_get(token, f"/orders/v0/orders/{order_id}/orderItems")
            time.sleep(2.0)   # 0.5 req/s restore rate
            return data.get("payload", {}).get("OrderItems", [])
        except requests.HTTPError as e:
            if e.response is not None and e.response.status_code == 429:
                wait = 2 ** attempt   # 1, 2, 4, 8, 16 …
                print(f"    Rate limited — waiting {wait}s...")
                time.sleep(wait)
            else:
                return []
    print(f"    Gave up on {order_id} after retries")
    return []


def main():
    token  = get_access_token()
    orders = fetch_orders(token, days=30)

    if not orders:
        print("No orders in the last 30 days.")
        return

    print("\nCounting units per ASIN from order items...")
    asin_units = {}
    for i, order in enumerate(orders):
        order_id = order.get("AmazonOrderId", "")
        items = fetch_order_items(token, order_id)
        for item in items:
            asin = item.get("ASIN", "")
            qty  = int(item.get("QuantityOrdered", 0))
            if asin:
                asin_units[asin] = asin_units.get(asin, 0) + qty

        if (i + 1) % 10 == 0:
            print(f"  Processed {i+1}/{len(orders)} orders...")

    print("\n" + "=" * 50)
    print("UNITS SOLD PER ASIN (last 30 days):")
    print("=" * 50)
    for asin, units in sorted(asin_units.items(), key=lambda x: -x[1]):
        print(f"  {asin}  →  {units} units")

    print("\nTop 3:")
    top3 = sorted(asin_units.items(), key=lambda x: -x[1])[:3]
    for asin, units in top3:
        print(f"  {asin}  →  {units} units")


if __name__ == "__main__":
    main()
