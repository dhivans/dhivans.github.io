#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from jsonschema import Draft202012Validator

from product_data import PRODUCTS_DIR, ROOT, load_catalog_records, parse_front_matter


SCHEMA_PATH = ROOT / "schemas" / "product.schema.json"


def load_schema() -> dict:
    return json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate _products front matter and generated catalog shape.")
    parser.parse_args()

    schema = load_schema()
    validator = Draft202012Validator(schema)
    failures: list[str] = []

    for path in sorted(PRODUCTS_DIR.glob("*.md")):
        try:
            front_matter, _body = parse_front_matter(path)
        except ValueError as exc:
            failures.append(str(exc))
            continue

        for error in sorted(validator.iter_errors(front_matter), key=lambda e: list(e.path)):
            location = ".".join(str(part) for part in error.path) or "<root>"
            failures.append(f"{path}: {location}: {error.message}")

    try:
        catalog = load_catalog_records()
    except Exception as exc:
        failures.append(f"catalog generation failed: {exc}")
    else:
        seen: set[str] = set()
        for record in catalog:
            asin = record["asin"]
            if asin in seen:
                failures.append(f"duplicate ASIN in catalog: {asin}")
            seen.add(asin)

    if failures:
        for failure in failures:
            print(failure)
        return 1

    print("Product front matter validated.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
