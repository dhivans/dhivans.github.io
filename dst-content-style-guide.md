# DST Site & Content Style Guide

Reference for writing guides/posts and making site/CSS changes, so
visual and content conventions stay consistent without re-deriving them
from scratch each session. Not published — excluded from the Jekyll
build the same way as `dst-growth-strategy.md`.

## Visual conventions

- **Palette**: dark backgrounds `#0d1117` (page) / `#161b22` (cards) /
  `#111820` (panels), cyan accent `#00e5ff`, muted text `#8b949e`, body
  text `#e2eaf4` / `#c9d1d9`. Semantic colours: green `#7ee787`
  (in-stock/good), yellow `#f2cc60` (low-stock/warning), red `#ff7b72`
  (out-of-stock/bad).
- **Fonts**: "Rajdhani" for UI labels/buttons, "Courier New" monospace
  for headings and small-caps/eyebrow-style text, "HemiHeadRG" reserved
  for the site logo only — don't use it elsewhere.
- **Buttons — reuse, don't invent**: `.btn-buy-now` (solid cyan,
  compact — card-level "buy now"), `.btn-amazon-full` (solid orange,
  full-width — the primary per-product Amazon CTA), `.btn-add-kit`
  (outlined cyan, secondary action). A new button style should only
  happen for a genuinely new *kind* of action, not a new page.
- **Tables in post/page markdown**: plain markdown tables are styled
  automatically via `.post-content table` / `.split-main table` in
  `assets/main.scss` (added 2026-09-03, after the first guide's table
  rendered as unthemed white-on-dark). No custom HTML needed — just
  write a normal markdown table.
- **Where CSS lives**: a component used across multiple pages goes in
  `assets/main.scss`. A one-off layout specific to a single page can
  live inline in that page's own file — see `compare.md`, `shop.md`,
  `kit.md` for the precedent.

## Content conventions

- **Internal linking is free**: add a `products:` front-matter list of
  ASINs to any post or project, and it auto-backlinks from those
  products' own pages via the existing "Featured In" section
  (`_layouts/product.html`). No manual link management needed — always
  set this on guides.
- **Client-side data**: embed as
  `<script type="application/json" id="...">{{ data | jsonify }}</script>`
  and `JSON.parse` it client-side (see `compare.md`, `_layouts/bundle.html`)
  rather than stuffing large structures into `data-*` attributes.
- **Guides must be verified, not restated**: check technical claims
  against a primary source (real datasheet, direct physical test)
  before publishing — don't repeat secondhand or listing-copy claims as
  fact. The D2F-01 vs D2F-5L guide (2026-09-03) is the model: the whole
  piece exists because the real datasheet contradicted what the
  listings implied.
- **Never display a number the site can't currently back up.** Found
  2026-09-04: one product (1N4007 diode) had `rating: 4.5, reviews: 42`
  in its front matter, rendering a real-looking star rating + review
  count on the live page. Checked the actual Amazon listing directly —
  it has 0 reviews. Nothing in `sync_products.py` or anywhere else in
  the pipeline ever syncs rating/review data; this was manually typed
  into exactly one of 46 products in an April 2026 bulk-normalize
  commit and never touched again. Removed. The underlying
  `page.rating`/`page.reviews` template feature in `product.html` is
  fine to keep for if/when there's ever a real, kept-current source for
  it — the fix was removing the fabricated data, not the feature. Same
  root cause as the BMP280 humidity claim: a specific, checkable claim
  that looked legitimate but nothing was actually keeping true.
- **Categories**: `categories: [guides]` for buying guides/comparisons,
  `[general]` for site announcements/updates. Guides currently live in
  the existing `_posts` collection (reusing what's already built,
  distinguished only by category) rather than a dedicated `_guides`
  collection — revisit only if the format needs something a blog post
  genuinely can't do.

## Process conventions

- **A successful `git push` is not evidence anything is live.** This
  repo deploys via GitHub Actions (`.github/workflows/pages.yml`) on
  every push to `main`. After pushing, check
  `gh run list --workflow=pages.yml --limit 1`, and watch it through
  (`gh run watch <id> --exit-status`) rather than assuming success.
  Learned the hard way 2026-09-03: a broken `Gemfile.lock` silently
  failed 6 consecutive deploys — including the entire kit-builder
  feature — while every one of those commits still looked fully
  successful from git's own perspective. Don't report something as
  "live" or "pushed to production" without having actually checked the
  deploy, not just the commit.
- **Running `bundle exec jekyll serve` locally on Windows can corrupt
  `Gemfile.lock`** — Bundler may add Windows-platform gem entries with
  incomplete checksums, which pass locally but fail CI's frozen-lockfile
  install. After any local Ruby/Bundler command, run
  `git diff Gemfile.lock` before committing it. If broken entries show
  up, delete those specific platform lines rather than trying to
  complete the checksums by hand — CI runs on Ubuntu and doesn't need
  the Windows platform at all.
- This repo auto-commits and pushes as part of normal workflow
  (established preference, confirmed 2026-09-01) — but "committed" and
  "actually live" are different claims. Verify the latter before
  telling the user it's done.
