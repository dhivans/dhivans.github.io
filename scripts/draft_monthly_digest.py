#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def main() -> int:
    parser = argparse.ArgumentParser(description="Draft a monthly catalogue digest post.")
    parser.add_argument("--month", default=date.today().strftime("%Y-%m"), help="Month label, e.g. 2026-06.")
    args = parser.parse_args()

    catalog = json.loads((ROOT / "_data" / "catalog.json").read_text(encoding="utf-8"))
    changes_path = ROOT / "_data" / "price_changes.json"
    changes = json.loads(changes_path.read_text(encoding="utf-8")) if changes_path.exists() else []
    out_dir = ROOT / "_drafts"
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / f"{args.month}-catalogue-digest.md"

    restocked = [item for item in catalog if item.get("in_stock")]
    low_stock = [item for item in catalog if int(item.get("stock_qty") or 0) <= 3]
    drops = [change for change in changes if change.get("direction") == "drop"]

    lines = [
        "---",
        f'title: "DST Catalogue Digest - {args.month}"',
        "layout: post",
        "categories: [digest]",
        "---",
        "",
        "## Price drops",
        "",
    ]
    if drops:
        for change in drops:
            lines.append(f"- `{change['asin']}` {change['title']}: {change['currency']} {change['old_price']} -> {change['currency']} {change['new_price']}")
    else:
        lines.append("- No price drops detected in the latest sync.")

    lines += ["", "## Low stock", ""]
    for item in low_stock[:25]:
        lines.append(f"- `{item['asin']}` {item['title']}: {item.get('stock_qty', 0)} available")

    lines += ["", "## Active catalogue count", "", f"- {len(restocked)} in-stock ASIN records in the latest catalogue."]
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
