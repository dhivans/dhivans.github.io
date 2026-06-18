#!/usr/bin/env python3
"""Generate _data/catalog.json from _products front matter."""

from __future__ import annotations

import json
from datetime import datetime, timezone

from product_data import DATA_DIR, load_product_documents, parse_price_amount, price_currency, product_asin_records


def load_history_summary(asin: str) -> dict:
    path = DATA_DIR / "history" / f"{asin}.json"
    if not path.exists():
        return {"sparkline_points": "", "lowest_90d": None}

    history = json.loads(path.read_text(encoding="utf-8"))
    prices = [point.get("price") for point in history[-90:] if point.get("price") is not None]
    if not prices:
        return {"sparkline_points": "", "lowest_90d": None}

    low = min(prices)
    high = max(prices)
    if len(prices) == 1:
        return {"sparkline_points": "0,20 100,20", "lowest_90d": low}

    spread = high - low
    points: list[str] = []
    for index, price in enumerate(prices):
        x = 100 * index / (len(prices) - 1)
        y = 20 if spread == 0 else 40 - ((price - low) / spread * 40)
        points.append(f"{x:.2f},{y:.2f}")
    return {"sparkline_points": " ".join(points), "lowest_90d": low}


def build_catalog() -> list[dict]:
    updated_at = datetime.now(timezone.utc).replace(microsecond=0).isoformat()
    catalog: list[dict] = []

    for document in load_product_documents():
        for record in product_asin_records(document):
            price_amount = parse_price_amount(record["price"])
            stock_qty = int(record["stock_qty"] or 0)
            history = load_history_summary(record["asin"])
            catalog.append(
                {
                    "asin": record["asin"],
                    "slug": record["slug"],
                    "title": record["title"],
                    "product_title": record["product_title"],
                    "variant_label": record["variant_label"],
                    "category": record["category"],
                    "subcategory": record["subcategory"],
                    "price": record["price"],
                    "price_amount": price_amount,
                    "currency": price_currency(record["price"]),
                    "in_stock": stock_qty > 0,
                    "stock_qty": stock_qty,
                    "sales_30d": record["sales_30d"],
                    "bsr": record["bsr"],
                    "url": f"/shop/{record['slug']}/",
                    "amazon_url": record["amazon_url"],
                    "image": record["image"],
                    "description": record["description"],
                    "specs": record["specs"],
                    "tags": record["tags"],
                    "sparkline_points": history["sparkline_points"],
                    "lowest_90d": history["lowest_90d"],
                    "updated_at": updated_at,
                }
            )

    return sorted(catalog, key=lambda item: (str(item["category"]), str(item["title"])))


def main() -> None:
    DATA_DIR.mkdir(exist_ok=True)
    path = DATA_DIR / "catalog.json"
    path.write_text(json.dumps(build_catalog(), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Wrote {path}")


if __name__ == "__main__":
    main()
