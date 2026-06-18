from __future__ import annotations

import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parent.parent
PRODUCTS_DIR = ROOT / "_products"
DATA_DIR = ROOT / "_data"
HISTORY_DIR = DATA_DIR / "history"
CATALOG_PATH = DATA_DIR / "catalog.json"
PRICE_CHANGES_PATH = DATA_DIR / "price_changes.json"


def parse_front_matter(path: Path) -> tuple[dict[str, Any], str]:
    text = path.read_text(encoding="utf-8")
    match = re.match(r"^---\s*\n(.*?)\n---\s*\n?(.*)$", text, re.DOTALL)
    if not match:
        raise ValueError(f"{path} is missing YAML front matter")
    data = yaml.safe_load(match.group(1)) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} front matter must be a mapping")
    return data, match.group(2)


def product_slug(path: Path) -> str:
    return path.stem


def parse_price(value: Any) -> float | None:
    if value is None:
        return None
    text = str(value)
    numbers = re.findall(r"\d+(?:\.\d+)?", text.replace(",", ""))
    if not numbers:
        return None
    return float(numbers[0])


def currency_from_price(value: Any) -> str:
    text = str(value or "")
    if "£" in text or "\u00a3" in text:
        return "GBP"
    if "$" in text:
        return "USD"
    if "€" in text:
        return "EUR"
    return "GBP"


def amazon_url(asin: str, explicit: Any = None) -> str:
    if explicit:
        return str(explicit)
    return f"https://www.amazon.co.uk/dp/{asin}"


def bool_stock(stock_qty: int | None) -> bool:
    return bool(stock_qty and stock_qty > 0)


def normalize_specs(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def load_catalog_records() -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    now = datetime.now(timezone.utc).replace(microsecond=0).isoformat()

    for path in sorted(PRODUCTS_DIR.glob("*.md")):
        fm, body = parse_front_matter(path)
        page_slug = product_slug(path)
        page_url = f"/shop/{page_slug}/"
        base = {
            "slug": page_slug,
            "page_url": page_url,
            "title": fm.get("title", page_slug.replace("-", " ").title()),
            "product_title": fm.get("title", page_slug.replace("-", " ").title()),
            "description": fm.get("description", ""),
            "category": fm.get("category", ""),
            "subcategory": fm.get("subcategory", ""),
            "image": fm.get("image", ""),
            "sales_30d": int(fm.get("sales_30d") or 0),
            "bsr": fm.get("bsr"),
            "tags": fm.get("tags") if isinstance(fm.get("tags"), list) else [],
            "specs": normalize_specs(fm.get("specs")),
            "updated_at": now,
        }

        variants = fm.get("variants")
        if isinstance(variants, list) and variants:
            for variant in variants:
                if not isinstance(variant, dict) or not variant.get("asin"):
                    continue
                asin = str(variant["asin"])
                price = variant.get("price", fm.get("price"))
                stock_qty = int(variant.get("stock") or 0)
                product_url = amazon_url(asin, variant.get("amazon_url") or variant.get("url"))
                price_value = parse_price(price)
                records.append(
                    {
                        **base,
                        "asin": asin,
                        "variant": variant.get("label") or variant.get("name") or "",
                        "variant_label": variant.get("label") or variant.get("name") or "",
                        "price": price or "",
                        "price_value": price_value,
                        "price_amount": price_value,
                        "currency": currency_from_price(price),
                        "in_stock": bool_stock(stock_qty),
                        "stock_qty": stock_qty,
                        "url": product_url,
                        "amazon_url": product_url,
                        "image": variant.get("image") or base["image"],
                    }
                )
        elif fm.get("asin"):
            asin = str(fm["asin"])
            price = fm.get("price")
            stock_qty = int(fm.get("stock") or 0)
            product_url = amazon_url(asin, fm.get("amazon_url"))
            price_value = parse_price(price)
            records.append(
                {
                    **base,
                    "asin": asin,
                    "variant": "",
                    "variant_label": "",
                    "price": price or "",
                    "price_value": price_value,
                    "price_amount": price_value,
                    "currency": currency_from_price(price),
                    "in_stock": bool_stock(stock_qty),
                    "stock_qty": stock_qty,
                    "url": product_url,
                    "amazon_url": product_url,
                }
            )
        else:
            raise ValueError(f"{path} must define either asin or variants")

    return records


def load_product_documents() -> list[dict[str, Any]]:
    documents: list[dict[str, Any]] = []
    for path in sorted(PRODUCTS_DIR.glob("*.md")):
        front_matter, body = parse_front_matter(path)
        documents.append(
            {
                "path": path,
                "slug": product_slug(path),
                "front_matter": front_matter,
                "body": body,
            }
        )
    return documents


def product_asin_records(document: dict[str, Any]) -> list[dict[str, Any]]:
    fm = document["front_matter"]
    slug = document["slug"]
    base = {
        "slug": slug,
        "title": fm.get("title", slug.replace("-", " ").title()),
        "product_title": fm.get("title", slug.replace("-", " ").title()),
        "description": fm.get("description", ""),
        "category": fm.get("category", ""),
        "subcategory": fm.get("subcategory", ""),
        "sales_30d": int(fm.get("sales_30d") or 0),
        "bsr": fm.get("bsr"),
        "tags": fm.get("tags") if isinstance(fm.get("tags"), list) else [],
        "specs": normalize_specs(fm.get("specs")),
        "image": fm.get("image", ""),
    }

    records: list[dict[str, Any]] = []
    variants = fm.get("variants")
    if isinstance(variants, list) and variants:
        for variant in variants:
            if not isinstance(variant, dict) or not variant.get("asin"):
                continue
            asin = str(variant["asin"])
            records.append(
                {
                    **base,
                    "asin": asin,
                    "variant_label": variant.get("label") or variant.get("name") or "",
                    "price": variant.get("price", fm.get("price")) or "",
                    "stock_qty": int(variant.get("stock") or 0),
                    "amazon_url": amazon_url(asin, variant.get("amazon_url") or variant.get("url")),
                    "image": variant.get("image") or base["image"],
                }
            )
    elif fm.get("asin"):
        asin = str(fm["asin"])
        records.append(
            {
                **base,
                "asin": asin,
                "variant_label": "",
                "price": fm.get("price") or "",
                "stock_qty": int(fm.get("stock") or 0),
                "amazon_url": amazon_url(asin, fm.get("amazon_url")),
            }
        )
    return records


def parse_price_amount(value: Any) -> float | None:
    return parse_price(value)


def price_currency(value: Any) -> str:
    return currency_from_price(value)


def read_json(path: Path, default: Any) -> Any:
    if not path.exists():
        return default
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def history_path(asin: str) -> Path:
    return HISTORY_DIR / f"{asin}.json"


def load_history(asin: str) -> list[dict[str, Any]]:
    history = read_json(history_path(asin), [])
    return history if isinstance(history, list) else []


def append_history(records: list[dict[str, Any]], date: str | None = None) -> list[dict[str, Any]]:
    date = date or datetime.now(timezone.utc).date().isoformat()
    changes: list[dict[str, Any]] = []
    HISTORY_DIR.mkdir(parents=True, exist_ok=True)

    for record in records:
        asin = record["asin"]
        price_value = record.get("price_value")
        stock_qty = record.get("stock_qty")
        point = {"date": date, "price": price_value, "stock_qty": stock_qty}
        history = load_history(asin)
        previous = history[-1] if history else None

        if previous and previous.get("date") == date:
            history[-1] = point
        else:
            history.append(point)

        if previous:
            old_price = previous.get("price")
            if isinstance(old_price, (int, float)) and isinstance(price_value, (int, float)) and old_price != price_value:
                direction = "drop" if price_value < old_price else "rise"
                changes.append(
                    {
                        "date": date,
                        "asin": asin,
                        "slug": record["slug"],
                        "title": record["title"],
                        "old_price": old_price,
                        "new_price": price_value,
                        "currency": record.get("currency", "GBP"),
                        "direction": direction,
                    }
                )

        write_json(history_path(asin), history)

    write_json(PRICE_CHANGES_PATH, changes)
    return changes


def sparkline_points(values: list[float], width: int = 100, height: int = 40) -> str:
    if not values:
        return ""
    if len(values) == 1:
        return f"0,{height / 2:.2f} {width},{height / 2:.2f}"
    low = min(values)
    high = max(values)
    span = high - low or 1
    points = []
    for index, value in enumerate(values):
        x = (index / (len(values) - 1)) * width
        y = height - ((value - low) / span) * height
        points.append(f"{x:.2f},{y:.2f}")
    return " ".join(points)


def enrich_from_history(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    for record in records:
        history = load_history(record["asin"])[-90:]
        prices = [p.get("price") for p in history if isinstance(p.get("price"), (int, float))]
        record["lowest_90d"] = min(prices) if prices else None
        record["sparkline_points"] = sparkline_points(prices)
        if len(prices) >= 2 and prices[-1] < prices[-2]:
            record["price_dropped"] = True
        else:
            record["price_dropped"] = False
    return records
