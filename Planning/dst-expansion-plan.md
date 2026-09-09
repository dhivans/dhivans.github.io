# DhivanSTech (DST) — Site Expansion Plan & Coding-Prompt Brief

A roadmap **and** a ready-to-paste brief for Claude Code. Every item is scoped as a mini-ticket with an effort tag (**S/M/L**), why it's worth it for a one-person Amazon-only brand, what to build, and an acceptance criterion.

**How to use this:** work one *phase* (or even one ticket) per Claude Code session. Paste the **Repo context** block below as standing context, then paste the ticket(s) you want, then paste the **Per-ticket expansion template** at the end if you want a single ticket fleshed out into a detailed spec. Do **Phase 0 (Foundation)** first — most other tickets depend on it.

---

## Repo context (paste this at the top of any session)

> **Project:** DhivanSTech (DST) — an electronics brand selling DIY maker components and test equipment **entirely through Amazon** (Amazon handles fulfilment/returns; there is no own checkout). The site is the marketing/discovery layer; the brand pitch is "tested, known-spec parts" in a market of low-information listings. It also publishes engineering content (build logs, teardowns, experiments) in a direct, no-padding tone.
>
> **Stack:** Jekyll static site (minima theme, customized), deployed via GitHub Pages at `dhivans.github.io`.
>
> **Structure:**
> - `_products/` — one markdown file per Amazon listing (front matter: price, stock, category, sales_30d, etc.)
> - `_projects/` and `_posts/` — build logs / blog content
> - `_layouts/`, `_includes/` — custom templates (product page, shop page, sidebar "hot/trending" widgets)
> - `shop.md` — storefront with category filtering
> - `scripts/sync_products.py` — pulls live listings (title, price, stock) from the Amazon **Selling Partner API (SP-API)** and creates/updates `_products/*.md`
> - `scripts/debug_orders.py` — local order-data debugging helper
> - `.github/workflows/sync-products.yml` — nightly cron that runs the sync and auto-commits changed product files
>
> **Constraints / conventions:**
> - Keep it **static-first**: no server, no database. Everything is build-time or client-side.
> - **Do not break the nightly sync** or its auto-commit contract.
> - **Never commit secrets or buyer PII.** SP-API credentials live in GitHub Actions secrets only.
> - Prefer a **generated `_data` JSON index** over client-side parsing of many markdown files.
> - Match existing template/style conventions; keep pages fast.
> - Small, reviewable commits; update README/docs alongside code.
> - Respect Amazon ToS: don't rehost prohibited data; prefer own product photos over hotlinking Amazon images.
>
> **Non-goals:** a separate storefront/checkout (Amazon is the funnel endpoint); a heavy JS framework/SPA; any core dependency on unofficial review-scraping APIs.

---

## Phase 0 — Foundation (do this first; it unlocks almost everything)

**F1 — Generated catalogue index `_data/catalog.json`** · **M**
Why: search, comparison, faceting and every JS widget need one machine-readable list; parsing 50 markdown files client-side is the wrong move.
Build: extend the sync (or add a small build step / Jekyll generator) to emit a normalized record per ASIN — `asin, slug, title, category, subcategory, price, currency, in_stock, stock_qty, sales_30d, bsr, url, image, specs{}, updated_at`. Make it the single source of truth; product pages can read from `site.data.catalog` too.
Done when: `_data/catalog.json` regenerates on every sync, validates against the schema (F3), and at least one template reads from it.

**F2 — Price & stock history** · **S–M**
Why: enables sparklines, price-drop detection, a Deals page, and the trust signal "we show the price trend."
Build: nightly, append `{date, price, stock_qty}` per ASIN to `_data/history/<asin>.json` (append-only — never rewrite past points). Optionally roll up points older than N months to cap size.
Done when: each history file grows by one point per ASIN per run; a product page renders an inline SVG sparkline from it (no chart library).

**F3 — Front-matter schema + validator** · **S–M**
Why: the sync writes data unattended — one malformed field shouldn't silently break the build or a widget.
Build: define a JSON Schema for `_products` front matter (required: `asin, title, price, category, in_stock`; typed). Add `scripts/validate_products.py` (jsonschema), run it in CI and at the end of the sync; fail loudly / open a GitHub Issue on violation.
Done when: CI fails a PR that introduces an invalid product file, and the sync aborts its commit if it would write invalid data.

**F4 — Build + checks in CI (on push/PR)** · **S–M**
Why: today CI only runs the nightly sync; nothing catches a broken build or dead link before it ships to the live brand site.
Build: a GitHub Actions workflow on push/PR running `bundle exec jekyll build` + html-proofer (internal links, images) + the validators (F3) + Python tests (T3). Cache gems and pip.
Done when: every PR shows a green/red build, and broken internal links/images fail the check.

**T4 — Config to `_data`** · **S** *(listed here because Phase 0 should establish it)*
Why: stop hard-coding categories/thresholds; one place to tune.
Build: move categories, low-stock threshold, brand metadata and facet definitions into `_data/*.yml`; templates and scripts read from there.
Done when: changing a threshold or adding a category is a one-file edit.

---

## 1. Shop / catalogue UX

**S1 — Site search (Pagefind)** · **S**
Why: an Amazon-only brand lives or dies on "can the visitor find the right part." Pagefind indexes at build time and ships a tiny runtime — zero infra.
Build: add Pagefind as a post-build step in CI; drop its UI on `shop.md` plus a global search box; index product pages + posts; boost product titles/specs.
Done when: typing "10k resistor" or "logic analyzer" instantly surfaces the right product and related posts, no server.

**S4 — Live stock + price-change badges** · **S**
Why: nothing wastes a click like clicking through to an out-of-stock listing.
Build: from `catalog.json`, render In stock / Low stock (qty < threshold) / Out of stock badges, plus a "↓ price dropped" badge when the latest history point < previous (F2).
Done when: badges reflect the last sync; OOS items are de-emphasized and optionally sorted last.

**S2 — Faceted filtering** · **M**
Why: makers filter by spec, not just category (voltage, range, channels, interface, count) — beats Amazon's noisy filters.
Build: client-side filtering over `catalog.json`; facets defined per category in `_data/facets.yml`; render checkboxes/sliders; reflect state in the URL query for shareable filtered views.
Done when: selecting `category=Test Equipment` + `channels≥2` narrows visibly, and the URL is shareable and restores state.

**S3 — Product comparison view** · **M**
Why: "known-spec parts" is the brand promise — let buyers compare 2–3 items side by side on the specs you've verified.
Build: an "Add to compare" control on cards (max 3); a `/compare` page reading `catalog.json`; a table of shared spec keys with differences highlighted; deep-link via `?asins=...`; one Buy-on-Amazon CTA per column.
Done when: comparing two multimeters shows a spec table with diffs highlighted and per-column CTAs.

**S5 — Sparkline + "best price seen"** · **S–M**
Why: transparency builds trust vs low-info competitors and surfaces real deals.
Build: inline SVG sparkline (F2) on product pages + a "lowest in last 90 days" marker.
Done when: each product page shows a 90-day price trend with no heavy chart lib.

**S6 — Curated kits / bundles** · **S** (content) / **M** (templated)
Why: "Starter breadboarding kit," "Beginner bench setup" — brand merchandising and content that funnels to multiple ASINs (Amazon still fulfils each separately).
Build: a `_bundles/` collection; each bundle lists ASIN refs; the template pulls live price/stock from `catalog.json`, shows a running total and a buy link per item.
Done when: a bundle page lists components with live prices and one CTA block per item.

**S7 — Related / "pairs well with"** · **S–M**
Why: cross-sell within the catalogue (a sensor → its breakout, a board → its PSU).
Build: manual `related: [asin,...]` front matter first; later auto-suggest from co-purchase data once Orders are pulled (A4). Render 2–4 on the product page.
Done when: product pages show related items from the same catalog.

---

## 2. Content / blog

**C1 — Bidirectional product ↔ post linking** · **S–M** · ⭐ *highest-leverage content feature*
Why: this is the single biggest lever — it turns the blog from decoration into a funnel and makes products feel *used*, not just listed.
Build: add `products: [asin,...]` to post/project front matter. On posts, render a "Parts used" box (live price/stock from `catalog.json`). On product pages, auto-list "Featured in" posts via a reverse lookup over `site.posts`/`site.projects`.
Done when: a build log shows its parts with live links, and the products it uses list that post under "Featured in."

**C2 — Shoppable Bill-of-Materials include** · **M**
Why: every build log should be a one-click parts list — the most natural commerce moment you have.
Build: a `{% bom asins="..." %}` include rendering a table (qty, part, live price, buy link) + a "buy all" list; quantities via `products: [{asin, qty}]`.
Done when: dropping the include into a post renders a correct, live-priced BOM.

**C5 — Buying guides keyed to categories** · **S** (tech) / content
Why: organic search top-of-funnel ("how to pick a multimeter") that funnels to your products — your main non-Amazon discovery channel.
Build: a guide layout that can embed the comparison table (S3) and a filtered product shortlist (F1) for its category.
Done when: a guide renders an embedded, live product shortlist.

**C7 — Newsletter capture (owned audience)** · **S**
Why: Amazon owns your customers; an email list is the one audience *you* own — critical for a marketplace-only brand.
Build: a static-friendly embed (Buttondown/MailerLite) in the footer and at the end of posts; no backend.
Done when: a working signup form ships site-wide.

**C3 — Post series / collections** · **S**
Why: multi-part build logs ("Bench PSU pt.1–4") keep readers and improve dwell time / SEO.
Build: a `series:` front-matter key + an include rendering "Part N of M" prev/next navigation within the series.
Done when: series posts show ordered navigation.

**C4 — Content-type templates (teardown / build log / experiment / buying guide)** · **S**
Why: consistency reads as a brand and lowers your friction to publish.
Build: layouts or include partials per type — e.g. teardown gets a "verdict + specs-vs-claimed" block; a buying guide ends in a comparison/CTA.
Done when: each content type has a distinct, reusable skeleton.

**C6 — Tags / topics taxonomy + pages** · **S**
Why: navigability, internal linking, SEO.
Build: generated tag pages for posts/projects; tag chips link through.
Done when: clicking a tag lists all matching content.

---

## 3. Automation (Amazon sync)

**A0 — SP-API capability reality (read before scoping the rest)**
- **Available via SP-API:** orders (Orders / Reports APIs), pricing & Buy Box (Product Pricing API), FBA stock (FBA Inventory API), catalog data incl. **sales rank / BSR** (Catalog Items 2022-04-01), **fee estimates** (Product Fees API), and **review/return *insights*** — most-positive/negative review topics at ASIN/browse-node level — via the **Customer Feedback API** (refreshed weekly, English-only, limited marketplaces).
- **Not available via SP-API:** raw customer **review text or star-rating numbers** per product. The Solicitations API only lets you *request* reviews (review-request emails). Third-party scrapers exist but are ToS-fraught/legacy — do not build core infrastructure on them.
- **Implication:** "show star rating on cards" is not cleanly doable from SP-API. "Show review *themes*" (Customer Feedback API) and "rank-driven trending" (Catalog Items BSR) are.

**A1 — Price-change & price-drop detection** · **S**
Why: powers the Deals page (B4), price-drop badges (S4) and notifications.
Build: in the sync, diff the new price against the last history point (F2), tag drops, and emit a dated `_data/price_changes.json`.
Done when: a price change produces a dated record consumable by both UI and alerts.

**A2 — Low-stock / OOS alerts** · **S**
Why: operational safety — never silently sell out or sit out-of-stock.
Build: when stock < threshold or hits 0, have the workflow open/update a GitHub Issue and/or hit a Discord/Slack/email webhook (de-duped).
Done when: crossing the threshold pings you within one nightly cycle, without spamming.

**A3 — Pull BSR / sales rank + fees** · **M**
Why: real rank data makes the "trending/hot" widgets truthful; fee data feeds margin views.
Build: extend the sync to call Catalog Items (`salesRanks`) and Product Fees; store per ASIN in `catalog.json` / history.
Done when: product data includes current category rank, and the trending widget can sort by it.

**A4 — Real sales analytics from Orders** · **M**
Why: data-driven merchandising + a truthful bestsellers widget — and you already have `debug_orders.py` to build on.
Build: a scheduled Orders/Reports pull → aggregate units & revenue by ASIN over 7/30/90 days → write an internal `_data/sales.json` + a public "bestsellers/hot" feed derived from deltas. **Strip PII — never commit buyer data.**
Done when: trending is driven by real sales_30d deltas (not static values), with no PII in the repo.

**A5 — Review-insight surfacing (Customer Feedback API)** · **M**
Why: "what buyers praise / flag," per product, in your honest voice — a brand-fitting alternative to scraping star ratings.
Build: a weekly pull of ASIN-level positive/negative review topics; store and render a small "Buyers mention" block on product pages.
Done when: products with data show top praise/complaint themes, refreshed weekly.

**A6 — Auto-drafted monthly digest post** · **M**
Why: zero-effort fresh content + SEO + newsletter fodder, generated from diffs you already compute.
Build: a monthly workflow assembles "New / restocked / price changes this month" from `price_changes` + stock deltas into a **draft** `_posts` markdown (opens a PR for review — not auto-publish).
Done when: a reviewable monthly digest PR appears automatically.

**A7 — Sync hardening** · **S–M**
Why: a cron job that silently fails or commits noise rots; SP-API throttles.
Build: exponential backoff on 429/5xx; commit only real diffs; flag ASINs that 404/go inactive (open an Issue); structured run log.
Done when: throttling no longer fails the run, empty diffs make no commit, and dead listings get flagged.

---

## 4. Technical / architecture

**T1 — JSON-LD structured data + SEO baseline** · **S–M** · ⭐ *high value for an Amazon-only brand*
Why: with no own checkout, organic search is your top-of-funnel. Product/Article schema + clean meta wins "best X for Y" queries and rich results.
Build: `Product` JSON-LD on product pages (name, brand=DST, sku=asin, `offers`→Amazon URL/price/availability), `Article` JSON-LD on posts, OpenGraph/Twitter cards, canonical URLs, `jekyll-sitemap`, `robots.txt`.
Done when: product pages validate as Product rich results, every page has OG tags + canonical, and the sitemap is submitted.

**T3 — Tests for the sync (pytest + mocked SP-API)** · **M**
Why: the sync is the backbone; refactors must not break the pipeline.
Build: pytest with recorded/mocked SP-API responses (e.g. VCR.py or fixtures) covering parsing, schema output, history append, diffing and error paths.
Done when: CI runs the sync tests on every push, and a parsing regression fails the build.

**T2 — Image optimization & performance** · **M**
Why: a maker audience on mobile; perf = SEO + UX, and stock Amazon hotlinks are slow/fragile.
Build: responsive images + WebP + lazy-load (a Jekyll image pipeline or prebuilt assets); minify; fingerprint assets; add a Lighthouse-CI budget in CI.
Done when: Lighthouse perf clears the target in CI and images are served responsively, not hotlinked.

**T5 — Secrets hardening + Dependabot** · **S**
Why: SP-API refresh tokens in Actions must be handled cleanly, and deps drift.
Build: confirm credentials live in Actions secrets (never printed in logs), least-privilege; enable Dependabot for pip + bundler; pin versions.
Done when: no secret is ever logged, and weekly dependency PRs arrive.

**T6 — PR preview deploys** · **S–M**
Why: see changes before they hit the live brand site.
Build: per-PR preview builds (Pages environments, or Netlify/Cloudflare Pages previews).
Done when: each PR yields a preview URL.

---

## 5. Brand layer (make it feel like DST, not a generated catalogue)

**B2 — "Tested / known-spec" as a first-class feature** · **M** · ⭐ *this IS the pitch*
Why: your differentiator is verified specs — make it visible and consistent on every product, not buried in prose.
Build: a standard **"Verified specs"** panel on product pages (your measured/confirmed values vs the listing's claims), datasheet links, a "what we checked" note; a "Tested by DST" badge; a `/how-we-test` methodology page.
Done when: every product shows a consistent verified-spec panel linking to the methodology.

**B1 — Visual identity beyond minima** · **M–L** · ⭐ *biggest "feels like a brand" lever*
Why: the #1 thing separating "brand" from "generated catalogue" is a deliberate look — type, color, logo, components.
Build: a custom SCSS theme (palette, type scale, logo, buttons, cards, code-block styling suited to an electronics blog), consistent header/footer, favicon, a styled 404, and an OG-image template. Keep it fast and static.
Done when: the site reads as DST at a glance, not default minima.

**B4 — Deals / price-drops page** · **S–M** *(needs F2 + A1)*
Why: a page that updates itself signals an active brand, not a static dump.
Build: `/deals` rendering current drops (A1) and "lowest in 90 days" items from history.
Done when: price drops appear on `/deals` automatically.

**B3 — Brand story / about / the bench** · **S**
Why: a face and a workshop build trust a marketplace listing can't.
Build: an About page (who, why, the lab), author bio, real photos of the bench/gear.
Done when: an About page with real photos and voice ships.

**B6 — Consistent microcopy + trust signals** · **S**
Why: small touches remove friction and read as professional.
Build: footer/contact, a clear "Amazon fulfils orders & handles returns" note, a spec-accuracy statement in the product template.
Done when: trust signals appear consistently site-wide.

**B5 — "Built with DST" project gallery** · **S–M**
Why: showcases your own builds (later, customers') using the parts — social proof + content.
Build: a gallery layout pulling from `_projects` tagged `showcase`, each linking its BOM (C2).
Done when: a gallery grids your builds with shoppable parts.

---

## Suggested build order (phasing)

- **Phase 0 — Foundation:** F1, F2, F3, F4, T4. *Unlocks almost everything; do first.*
- **Phase 1 — Quick, high-leverage wins:** C1, S1, T1, S4, A1, A2. *Funnel + findability + SEO + operational safety.*
- **Phase 2 — Differentiators:** S3, S2, C2, C5, B2, B4. *The "known-spec" + comparison + shoppable-BOM story.*
- **Phase 3 — Brand layer:** B1, B3, B6, C7, S6. *Make it feel like DST; capture an owned audience.*
- **Phase 4 — Deeper automation, analytics & robustness:** A3, A4, A5, A6, A7, T2, T3, T5, T6, C3, C4, C6, S5, S7, B5. *Compounding value once the foundations are solid.*

If you only do five things: **F1 → C1 → T1 → B2 → A2.** That's the spine — a data layer, the blog-to-product funnel, organic discoverability, the verified-spec brand promise made visible, and never going silently out of stock.

---

## Per-ticket expansion template (paste to Claude Code for any single ticket)

```
[Paste the Repo context block above.]

Implement ticket: <ID> — <title>

Before writing code, inspect: _products/ front matter, _layouts/, _includes/,
scripts/sync_products.py, scripts/debug_orders.py, .github/workflows/.
Then propose a short implementation plan and wait for my go-ahead.

Constraints: static-first (no server/DB); don't break the nightly sync or its
auto-commit contract; no secrets or buyer PII in the repo; prefer reading from
_data JSON over parsing markdown client-side; match existing template/style
conventions; keep pages fast.

Deliver: code changes in small reviewable commits, updated README/docs, and
tests where applicable.

Acceptance criteria: <paste the "Done when" line from the ticket>.
```
