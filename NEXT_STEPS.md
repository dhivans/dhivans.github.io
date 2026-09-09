# Next Steps

_Regenerate this whenever something changes — it's a snapshot, not a hand-maintained log. Depth lives in [`Planning/`](Planning/)._

**Last generated:** 2026-09-09, from `Planning/dst-growth-strategy.md` (most recently edited, 2026-09-07), `Planning/dst-expansion-plan.md`, `README.md`, and `git log`.

## Current status

Domain migration, Search Console, GA4, JSON-LD, sitemap/robots, and the
Amazon Associates tag rollout (`dhivanstech-20`, added 2026-09-07 across
every outbound link) are done. Amazon Attribution has a pilot ad group live
(one product, proving the mechanism) but isn't rolled out further. Two
buying guides are published. The "Tested by DST" trust badge is fully built
but has **zero products populated** — that's the honest current state, not
a bug. The real, ticking constraint right now is the new Associates
account's requirement of **3 qualifying sales within 180 days of approval
(~2027-03-06)** or it risks the same closure as the previous account —
content/guide output is what's meant to drive that.

**⚠️ Contradiction found between the planning docs — flagged, not resolved:**
`Planning/dst-growth-strategy.md` §8 states its status table was "audited
against the actual repo on 2026-09-03," but `README.md` and git history show
a substantial list of engineering features from `Planning/dst-expansion-plan.md`
were already live **before** that date (committed 2026-09-01) and are
missing from the audit entirely: Pagefind site search (`/search/`), the
`/compare/` product comparison page, `/deals/` (price-drop tracking),
faceted shop filtering, the shoppable BOM include, series/tags navigation,
automated low-stock GitHub issue alerts, and a monthly auto-drafted
catalogue-digest PR. All of this is real and working per `README.md`'s own
"Trust and Commerce Surfaces" section and confirmed file/script presence —
it just isn't reflected in growth-strategy's backlog table, which tracks
content/business priorities (guides, trust-badge population, tutorials,
distribution) almost independently of the engineering layer. Net effect:
the technical foundation is further along than growth-strategy's own
table suggests. Worth reconciling next time that doc gets a real edit,
rather than treating its backlog table as a complete picture of what's built.

## Needs my attention now

- [ ] Populate "Tested by DST" (`verified_specs`) on a handful of real products — badge/schema fully built, currently zero products use it (`Planning/dst-growth-strategy.md` §8 #4)
- [ ] Keep writing buying guides & comparisons — two live (D2F-01 vs D2F-5L, BMP280), no fixed cadence, next topics from the existing catalogue first (§4.1)
- [ ] Real DST-taken product photography, best-sellers first — same physical-work blocker as the item above (§8 #7b)
- [ ] Keep the Associates 180-day sales clock in view (3 qualifying sales needed by ~2027-03-06) — this is a real deadline, not a soft target

## Waiting, not blocking

- [ ] Amazon Attribution — pilot ad group live (ESP32 dev board, Publisher "DhivanSTech", channel "Search"); expand the product pool by hand in the console as guides/products are added (§8 note: API automation deliberately deferred)
- [ ] Email newsletter — signup form built (`_includes/newsletter.html`) but `enabled: false`; switch on once there are a few guides worth mailing (§5.3)
- [ ] Domain email decision — Namecheap free forwarding is the front-runner, not yet actioned (§5.1)
- [ ] Reddit — ongoing low-cost habit (r/esp32, r/homeassistant etc.), no state to track here

## Backlog (see `Planning/dst-growth-strategy.md` §8 for the full ordered list and reasoning)

- [ ] Project tutorials — 3 real build logs already exist in `_projects/`; reframe/extend toward "problem → buy parts → build" once guides are proving out (§4.2, §8 #8)
- [ ] Customer proof / UGC collection — needs real traffic first (§3, §8 #10)
- [ ] QR-code packaging funnel — blocked on an open decision below, not effort (§5.5, §8 #11)
- [ ] DST Lab testing/teardown content hub — methodology page (`how-we-test.md`) exists, no individual write-ups yet (§4.4, §8 #12)
- [ ] Interactive tools — only if a static guide proves the underlying decision is genuinely complex enough to deserve one (§4.5, §8 #13)
- [ ] Video, creator/sample outreach, paid advertising — deliberately deferred until earlier phases are running and proven (§5.4, §5.6, §5.7, §8 #14–16)

## Open decisions only I can make (`Planning/dst-growth-strategy.md` §10)

- [ ] Whether to pursue a real, physical combined-SKU kit (new Amazon listing, packaging, inventory commitment) vs. the content-only bundles already live
- [ ] Which products DST actually controls packaging for, to unblock the QR-code funnel
- [ ] Creator/sample outreach budget and timing
- [ ] Whether/when to pursue an own-hardware product line (speculative, long-term — §9)
