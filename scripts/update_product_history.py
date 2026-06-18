#!/usr/bin/env python3
"""Append one price/stock point per ASIN to _data/history."""

from __future__ import annotations

import json
from datetime import datetime, timezone

from product_data import DATA_DIR, load_product_documents, parse_price_amount, product_asin_records


def main() -> None:
    history_dir = DATA_DIR / "history"
    history_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).replace(microsecond=0).isoformat()
    written = 0

    for document in load_product_documents():
        for record in product_asin_records(document):
            point = {
                "date": timestamp,
                "price": parse_price_amount(record["price"]),
                "stock_qty": int(record["stock_qty"] or 0),
            }
            path = history_dir / f"{record['asin']}.json"
            if path.exists():
                data = json.loads(path.read_text(encoding="utf-8"))
                if not isinstance(data, list):
                    data = []
            else:
                data = []
            data.append(point)
            path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            written += 1

    print(f"Appended history for {written} ASINs")


if __name__ == "__main__":
    main()
