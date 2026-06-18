#!/usr/bin/env python3
from __future__ import annotations

import argparse

from product_data import CATALOG_PATH, append_history, enrich_from_history, load_catalog_records, write_json


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate _data/catalog.json from _products front matter.")
    parser.add_argument(
        "--append-history",
        action="store_true",
        help="Append one price/stock point per ASIN before writing the catalog.",
    )
    args = parser.parse_args()

    records = load_catalog_records()
    if args.append_history:
        append_history(records)

    write_json(CATALOG_PATH, enrich_from_history(records))
    print(f"Wrote {CATALOG_PATH} with {len(records)} ASIN records.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
