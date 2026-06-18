#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parent.parent
CATALOG_PATH = ROOT / "_data" / "catalog.json"
SITE_DATA_PATH = ROOT / "_data" / "site.yml"


def load_threshold() -> int:
    if not SITE_DATA_PATH.exists():
        return 3
    data = yaml.safe_load(SITE_DATA_PATH.read_text(encoding="utf-8")) or {}
    return int((data.get("shop") or {}).get("low_stock_threshold") or 3)


def load_catalog() -> list[dict[str, Any]]:
    return json.loads(CATALOG_PATH.read_text(encoding="utf-8"))


def stock_rows(threshold: int) -> list[dict[str, Any]]:
    rows = []
    for item in load_catalog():
        qty = int(item.get("stock_qty") or 0)
        if qty <= threshold:
            rows.append(
                {
                    "asin": item.get("asin", ""),
                    "title": item.get("title", ""),
                    "variant": item.get("variant") or item.get("variant_label") or "",
                    "stock_qty": qty,
                    "page_url": item.get("page_url") or item.get("url") or "",
                    "amazon_url": item.get("amazon_url") or item.get("url") or "",
                    "status": "Out of stock" if qty == 0 else "Low stock",
                }
            )
    return sorted(rows, key=lambda row: (row["stock_qty"], row["title"], row["variant"]))


def render_report(rows: list[dict[str, Any]], threshold: int) -> str:
    if not rows:
        return f"No products are at or below the low-stock threshold ({threshold}).\n"

    lines = [
        f"{len(rows)} catalogue item(s) are at or below the low-stock threshold ({threshold}).",
        "",
        "| Status | Qty | ASIN | Product | Variant | Links |",
        "|---|---:|---|---|---|---|",
    ]
    for row in rows:
        page_link = f"[site]({row['page_url']})" if row["page_url"] else "site"
        amazon_link = f"[Amazon]({row['amazon_url']})" if row["amazon_url"] else "Amazon"
        lines.append(
            f"| {row['status']} | {row['stock_qty']} | `{row['asin']}` | {row['title']} | {row['variant']} | {page_link} / {amazon_link} |"
        )
    lines.append("")
    lines.append("This issue is updated by the nightly product sync. Close it once stock is replenished above the threshold.")
    return "\n".join(lines) + "\n"


def write_github_output(count: int, report_path: Path) -> None:
    output_path = os.environ.get("GITHUB_OUTPUT")
    if not output_path:
        return
    with Path(output_path).open("a", encoding="utf-8") as handle:
        handle.write(f"count={count}\n")
        handle.write(f"report_path={report_path.as_posix()}\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate a low-stock report from _data/catalog.json.")
    parser.add_argument("--output", default="low-stock-report.md", help="Markdown report path.")
    args = parser.parse_args()

    threshold = load_threshold()
    rows = stock_rows(threshold)
    report_path = Path(args.output)
    report_path.write_text(render_report(rows, threshold), encoding="utf-8")
    write_github_output(len(rows), report_path)
    print(f"Found {len(rows)} item(s) at or below stock threshold {threshold}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
