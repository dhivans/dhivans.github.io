# DST Growth & Content Strategy

This is the strategic layer sitting above `dst-expansion-plan.md`. That
document turns individual pieces of work into engineering tickets; this one
answers the prior question — what the website is *for*, what it should
become, and in what order — so that ticket list has a reason behind it
rather than being a grab-bag of features.

It started as a long brainstorm covering roughly twenty different ideas.
Most of the underlying thinking is genuinely good — the "catch people with
a problem, help them, become the obvious buy" model is exactly how
SparkFun and Adafruit built durable, trusted electronics brands. But the
brainstorm was written at the scale of a company with a content team, a
video team, and a dev team running in parallel. This document keeps the
good ideas, cuts or defers what doesn't fit DST's actual situation, and
turns the rest into something one person can actually execute without
burning out three weeks in.

## Ground rules

These apply to everything below and aren't repeated per-section.

**Amazon is the only checkout.** The site never takes a payment, never
ships anything, never handles a return. Every single page's real job,
no matter how it's dressed up, is to get someone to a specific Amazon
listing already confident enough to buy. Anything in the original
brainstorm that assumed an on-site cart, payment methods, or on-site
returns has been rewritten or cut — that's not a page the site needs.

**Solo, part-time.** This plan is sequenced, not parallel. A strategy
that has you writing guides, filming videos, running a testing lab,
building an interactive tool, and posting on five social platforms all
in month one isn't a plan — it's a way to make progress on nothing.
Everything below is ordered so that each phase is something a part-time
solo operator can sustain indefinitely, not sprint through once. Genuinely
good ideas that are too effort-heavy to start now are marked **Later**,
not deleted — revisit them once the earlier phases are running themselves
and there's real data to justify the extra effort.

**Domain migration is live (§5.1).** `dhivanstech.co.uk` (primary,
DNS at Namecheap) is serving the real site over HTTPS as of this
writing; `dhivanstech.com` still needs its redirect set up.

**If something below depends on a decision only you can make — a real
inventory/packaging change, a platform commitment, anything with a cost —
it's flagged explicitly in [Section 10](#10-open-decisions-not-assumed)
rather than assumed.**

---

## 1. The core idea

Most small electronics sellers are just a wall of listings: "here are 150
parts, please buy one." That doesn't work for a brand nobody's heard of,
competing against listings that look identical and cost less.

The alternative — the one that actually built brands like SparkFun and
Adafruit — is to be useful before you're a seller. Someone has a problem
("I want my plant to tell me when it needs water") or a specific technical
question ("does this need I2C or SPI?"), the website answers it properly,
and DST's product is the obvious, already-trusted thing to click through
to on Amazon at the end.

Concretely, that means the site has to do five jobs at once, not just one:

- **Discovery** — rank for the specific, high-intent questions people
  actually search, not "ESP32" (unwinnable) but "ESP32-C3 SuperMini pinout"
  or "BMP280 vs BME280" (winnable, and the person searching it is close to
  buying something).
- **Trust** — answer "why not just buy the £2 AliExpress one?" honestly,
  with real differentiation, not marketing language.
- **Recommendation** — point people at exactly the right part for their
  project, including the accessories they didn't know they needed.
- **Documentation** — be the place with a pinout that's actually right,
  after they've already bought.
- **Handoff** — get them to Amazon cleanly, and let you measure whether
  that actually happened.

Everything in this document is in service of one or more of those five
jobs. If a proposed page or feature doesn't clearly serve one, it doesn't
belong in an early phase.

---

## 2. The baseline: what every product page should be

This isn't a phase — it's the standard every product page should meet
before content strategy matters at all, because a brilliant buying guide
that lands on a thin product page still loses the sale.

**Answer the real questions, not the marketing ones.** Someone landing on
a listing has specific doubts: is this really the chip it claims to be,
what voltage, what's the pinout, does it work with Arduino/ESPHome/Home
Assistant, what are the actual dimensions, what's included, why this one
over the cheaper listing. The page should answer all of it plainly —
price, stock, delivery being handled by Amazon, and a clear specs section
are the baseline; a pinout, compatibility notes, and honest "why buy this
one" reasoning are what actually move someone to click through.

**Real differentiation, not fluff.** Not "committed to quality" — actual,
checkable claims: physically inspected before listing, specific things
tested (and specific things *not* tested, stated plainly rather than
implied), a real pinout or datasheet link, whatever is genuinely true.
If nothing genuinely differentiates a given product, don't invent
something — that's a signal the product itself might not be worth pushing
content at yet.

**Cross-sell by function, not by category.** "Commonly used with" is one
of the highest-leverage ideas in the original brainstorm and one of the
cheapest to do: an ESP32 page linking to the sensor, display, and cable
someone will also need turns a single-item sale into a multi-item one, and
it's just a data relationship, not new content.

**Link everything to everything relevant, as you go.** Every guide
(Section 4) should link to the products it mentions and to related
guides; every product page should link to the guides that reference it.
This costs nothing beyond a moment's thought while writing, and it's one
of the few genuinely free SEO levers — it's how Google (and readers) learn
that the site actually knows its subject, rather than being a pile of
disconnected pages. Treat it as a standing habit from the first guide
onward, not a separate project.

**Real photography over the wholesaler's stock image.** Front, back,
scale/dimension shot, connector close-up, the part in use if practical.
This is genuinely one of the more effort-heavy items here, so treat it as
an ongoing background task applied to best-sellers first, not a blocking
requirement before anything else ships.

**"Tested by DST" — defined, not vague.** See Section 3. Only claim it
where it's true, but where it's true, say exactly what was checked.

---

## 3. Trust system: "Tested by DST"

**✅ Built and live, 🟡 not yet populated on any product.** The badge
logic, the per-field checkmark, the schema behind it (`spec_schema.yml`),
and the explainer page (`how-we-test.md`) all exist and work correctly —
this section's design work is done. What's left is purely the physical
work of actually testing specific products and filling in the field; zero
products carry it today, which is the honest, correct state for a system
that's never been used yet, not a bug.

This is the honest answer to "why not just buy from AliExpress" — and it
only works if it's specific and true, not a generic badge.

For any given product, define exactly what gets checked before it's
listed, and only claim what was actually done:

- Correct part/module physically confirmed (not just trusting the label)
- Visual inspection — solder joints, connector condition, obvious defects
- Power-on test, where applicable
- Communication test (e.g. confirmed over I2C), where applicable

This doesn't need to apply to everything in the catalogue, and it
shouldn't be forced onto products where it isn't true yet. A product with
no bench-verified claims should say so plainly rather than implying a
check that didn't happen — that honesty is itself part of what makes the
claims on products that *do* carry it credible.

**Customer proof, collected passively.** Once there's real traffic,
occasionally ask buyers who mention a build ("built something with a DST
part? send us a photo") and show a handful on the relevant product page.
This costs nothing but the asking — it's not a campaign, just a standing
habit once there's an audience to ask. Electronics projects are inherently
more interesting to look at than most products, which makes this cheaper
to pull off here than in most categories.

---

## 4. Content: one pillar at a time

The original brainstorm proposed five different content types (buying
guides, tutorials, reference pages, an interactive tool, a testing lab)
running simultaneously. For one part-time person, that's the fastest way
to produce five mediocre things instead of one good one. Do these in
sequence, and don't start the next until the current one is producing
measurable results.

### 4.1 First content pillar: buying guides and comparisons

Start here. This is the highest intent-to-purchase content that exists,
and the cheapest to produce well — no photography rig, no wiring, no
filming, just genuine technical knowledge written down clearly.

Someone searching "BMP280 vs BME280" or "which ESP32 should I buy" is
functionally asking to be sold to. A short, honest, well-structured page —
what's different, a simple comparison table, a clear "if you need X, buy
this one" recommendation, links to both products — can quietly sell
product for years with near-zero maintenance once written.

No fixed cadence (see Section 8) — one genuinely good page at a time,
not the batch of five-per-category the original brainstorm suggested
producing at once. Quality and correctness matter far more than volume
here — a wrong comparison actively damages trust, and there's no schedule
worth rushing a page to meet.

Pick topics from what's already in the catalogue first (comparisons
between products actually stocked), then expand once Search Console shows
real demand (see Section 6).

### 4.2 Second content pillar: project tutorials

**🟡 A head start already exists.** Three real build-log posts are
already live in `_projects/` (a bench PSU build, a 3D-printer endstop
swap, and an ESP32 + WS2812B + rotary encoder "mood ring") — each with
real wiring tables, working code, and an honest gotchas section. They're
written as build logs rather than this section's "problem → buy these
parts → build it" framing, and the mood-ring one is already close to it
in spirit, cross-linking to its three component products. Reframing or
extending these is a smaller lift than starting from nothing once this
pillar actually starts.

Don't start this until buying guides are producing measurable outbound
clicks to Amazon (see Section 6) — a project tutorial is far more expensive to
produce well (wiring diagrams, real code, real photos, troubleshooting)
and it's wasted effort if the cheaper content type hasn't already proven
the audience exists and converts.

Once it's time: a genuinely useful build (wiring diagram, working code,
photos, common troubleshooting) for something buildable entirely from
products already stocked. This is what captures people who weren't
searching for DST at all — "ESPHome BMP280 wiring" — and lands them on a
page that happens to sell every part they need.

### 4.3 Reference pages — fold into the product page standard, don't treat as separate

Pinouts, wiring references, and dimension data were listed as their own
content type in the original brainstorm. They shouldn't be — they belong
on the product page itself (Section 2). Splitting them into a separate
page type just creates two things to maintain instead of one, and dilutes
the product page, which is where this information is actually needed.

### 4.4 Later: "DST Lab" — testing, teardowns, comparisons

**🟡 The methodology page already exists** — `how-we-test.md` documents
how the "Tested by DST" badge works and what gets checked. That's real
groundwork, but it's a single policy/explainer page, not yet the content
pillar this section describes (individual test/teardown write-ups per
product or comparison).

Genuinely one of the strongest differentiators in the original
brainstorm — actual measurements are much harder for a competitor to
copy than written content, and "we measured it so you don't have to" is a
real, memorable brand position. But it needs real instrumentation time per
piece and is one of the more effort-heavy formats here. Revisit this once
the buying-guides pillar is running on its own and there's spare
capacity, not before.

### 4.5 Later, and only in a much smaller form: interactive tools

The original brainstorm proposed a full "component finder" quiz (checkbox
questions, branching logic, six separate calculators). That's real
software to build and maintain for uncertain payoff, and it's the kind of
thing that's easy to sink weeks into.

Get most of the value for a fraction of the effort first: plain static
pages ("I want to measure temperature → BMP280 or BME280, here's why")
achieve most of what the interactive version would, are just more content
written under the buying-guides pillar, and cost nothing to maintain. If one of
these static guides turns out to attract serious traffic and the "which
one do I need" decision is genuinely complex enough to deserve interactive
logic, build *that one* tool — not the full six-tool suite up front.

### 4.6 Cut or folded in

- **"Product passport" pages** — this is just the product page standard
  from Section 2 done well. Treating it as a separate format duplicates
  effort for no benefit.
- **Bundles/kits — ✅ already live, see Section 8.** Originally flagged
  here as needing a new Amazon listing, packaging, and inventory
  decisions. What actually got built (`_bundles/`) sidesteps all of that:
  a curated landing page over existing ASINs, each linking to its own
  already-live Amazon listing, not a new combined product. That specific
  concern no longer applies — a *physical* combined-SKU kit would still
  be the open business decision in Section 10, but the content-bundle
  version is done and live.
- **Beginner "journey" pages, problem-based navigation rebuild** — good
  instincts, but this is largely what a well-cross-linked set of buying
  guides and product pages already achieves (Section 2's "commonly used
  with," Section 4.1's guides). Building a whole separate navigation
  structure for it is effort better spent on the content itself.

---

## 5. Distribution

Ranked by effort-to-value ratio for one part-time person — not in the
order the original brainstorm presented them.

### 5.1 Custom domain migration

DST owns `dhivanstech.co.uk` (primary) and `dhivanstech.com`
(redirects to it), both registered at Namecheap — but **the two domains
turned out to use different DNS providers**, not the single Cloudflare
setup first assumed: `dhivanstech.co.uk`'s DNS is managed directly in
Namecheap, while `dhivanstech.com`'s DNS is on Cloudflare. Both are
correctly configured for their actual provider now.

**✅ Done:**
- `dhivanstech.co.uk`: DNS records added in Namecheap (four `A` records
  for the apex domain at GitHub Pages' IPs, one `CNAME` for `www` →
  `dhivans.github.io`), confirmed resolving correctly, `CNAME` file added
  to the repo, `_config.yml`'s `url:` updated to
  `https://dhivanstech.co.uk`, confirmed live serving the real site over
  HTTPS. `dhivans.github.io` now auto-redirects to the new domain, as
  GitHub Pages does automatically once a custom domain is set.
- `dhivanstech.com` → `dhivanstech.co.uk` redirect: set up via a
  Cloudflare Redirect Rule (proxied placeholder DNS records + a 301
  dynamic redirect preserving the request path). Confirmed working —
  `dhivanstech.com` returns a clean 301 to the matching `.co.uk` page.
- New Google Search Console property for `dhivanstech.co.uk` (Domain
  type, verified via a Namecheap DNS TXT record), and **Change of
  Address** run from the old `dhivans.github.io` property to it — so
  search history transfers instead of the new domain starting cold.

**⬜ Still open:**

- **Email** — a related but separate decision, not required for the
  domain migration itself but worth deciding alongside it since it feeds
  into §5.3's email plan. Namecheap offers free email forwarding for
  domains registered there (simplest option, matches how `.co.uk`'s DNS
  ended up being managed).

### 5.2 Do this next: measurement and technical setup

This is almost entirely one-time setup work, has zero ongoing content
burden, and makes everything after it measurable instead of a guess.

- **Google Search Console** ✅ (one small step left) — new Domain-type
  property for `dhivanstech.co.uk` verified via Namecheap DNS TXT record,
  and **Change of Address** run from the old `dhivans.github.io` property
  so search history transfers rather than starting cold. Still worth
  submitting `sitemap.xml` under the *new* property directly (Sitemaps
  tab, same as before) — Change of Address handles the old-URL-to-new-URL
  mapping, it doesn't submit a sitemap for you.
- **Product structured data** ✅ — done. Real JSON-LD Product schema
  (price, availability, brand, image) is live on every product page.
- **Google Merchant Center** ❌ **dead end for DST as currently
  structured — resolved 2026-09-03, don't re-propose without a real
  change in circumstances.** Confirmed against Google's own primary
  documentation, not a guess: the landing-page policy states outright
  *"Only link to the domain from your Merchant Center account. Don't
  redirect people to another website outside of the domain that you
  claimed in your Merchant Center account,"* and the checkout
  requirements separately require in-stock products to be "available to
  add to cart and finalize their purchase" on that same claimed domain.
  Merchant Center (standard self-service account, paid Shopping ads or
  free listings alike) is built for retailers who fulfil the sale on
  their own claimed site — not for a content site whose product pages
  link out to Amazon for the actual purchase, which is exactly DST's
  setup and isn't going to stop being DST's setup. The earlier framing
  below ("submit through the marketplace instead") was on the right
  track but understated it: that's not an alternate *mechanism* DST could
  configure, it's that this channel belongs to Amazon (as the
  marketplace operator's own Merchant Center presence for
  Amazon-fulfilled listings), not to DST. The only way this becomes
  viable is DST building real cart/checkout on dhivanstech.co.uk itself —
  a genuinely different, much bigger undertaking, not a quick win to
  revisit opportunistically. Not pursuing.
- **Basic analytics** ✅ (partial) — Google Analytics (GA4) is live in
  production. Specific event tracking (outbound Amazon click, guide page
  view as distinct events) hasn't been confirmed as set up — worth
  checking what's actually configured beyond pageviews before relying on
  it for the Section 6 decisions.
- **`sitemap.xml` / `robots.txt`** ✅ — done, via `jekyll-sitemap` and a
  correct manual `robots.txt`. Not originally its own line item here, but
  it's exactly the kind of Search Console prep this phase is about.

### 5.3 Ongoing, low-cost habits: Reddit and email

Neither of these is a content production line — they're standing habits
that reuse content already being written elsewhere, not a new workload.

**Reddit.** Be genuinely useful in relevant communities (r/esp32,
r/homeassistant, r/AskElectronics and similar) when a question comes up
that a real answer helps with, occasionally linking a guide where it's
actually relevant. This costs minutes, not hours, and reads as authentic
specifically because it isn't a campaign. Explicitly not: promotional
posts, "check out my store."

**Email.** 🟡 The signup form itself is already built
(`_includes/newsletter.html`), it's just switched off
(`enabled: false`, no email service connected yet in `_data/site.yml`) —
so turning this on later is a small task, not a new build. Keep it small
and low-pressure — no "10% off, sign up now" popup on arrival. Something
like "get the ESP32-C3 pinout PDF" or "one genuinely useful project a
month" gives people a real reason to hand over an email address. Don't
switch it on until there are a few guides worth sending — an empty list
with nothing useful to mail is wasted setup.

### 5.4 Deferred: video (YouTube and short-form)

The original brainstorm ranked YouTube as the single highest-priority
channel. This isn't deferred because it's a poor fit skill-wise — there's
real prior video/hands-on-testing experience to draw on, arguably a
better natural fit than the written guides this plan leads with. It's
sequenced later because each piece still takes real production time
(filming, editing, consistency) regardless of skill, and it should land
better once there's an existing base of guides and tested products for it
to point at and reinforce, rather than launching into a channel with
nothing built yet to support it (see Section 8's note on this — flagged
there as an inference, worth correcting if it's off). Revisit once the
backlog above it is running and there's evidence (Section 6) of what
actually converts. The same applies to repurposing content across
Instagram/TikTok/Pinterest — real production overhead for channels that
haven't been validated yet.

### 5.5 Later, needs real-world coordination: packaging QR-code funnel

Genuinely clever: a QR code on packaging linking to a product-specific
"thanks for buying, here's your quick-start" page turns an Amazon sale
(where DST has no direct relationship with the buyer) into a visit to
DST's own site. But it only works for products where DST controls the
physical packaging — not generic dropshipped items — and needs the
landing pages built and the domain migration (§5.1) done first, so the
QR code points somewhere permanent. Worth doing, not worth doing before
the buying-guides pillar and the basics in Section 5.2 are running.

### 5.6 Later, needs real budget: creator/sample outreach

Sending free parts to small creators (roughly 5k-50k followers, people
who actually build electronics rather than general tech influencers) with
no strings attached — "here's £30 of parts, build whatever you want, no
requirement to say anything nice" — is a genuinely sound idea: their
audience is far more targeted than a big-name influencer's, and it can
generate video, backlinks, and real product feedback along the way.

But unlike everything else in this section, this one costs real money up
front for an uncertain return, not just time — sending parts to even a
handful of creators is a direct spend, not a background task. Don't start
this until there's actual guide/tutorial content worth pointing creators
at (Section 4) — sending someone free parts with nothing for their
audience to read afterward wastes the spend. Start with a small number
(a handful, not the twenty from the original brainstorm) as a trial before
committing to more. See Section 10 for the budget decision this needs.

### 5.7 Last, and only once something is proven: paid advertising

Deliberately last, and for a good reason: paid traffic only makes sense
once there's something worth paying to promote. Running ads to sell a
single £4 BMP280 mostly hands margin straight to the ad platform. Paid
advertising becomes worth considering once there are bundles/kits with a
higher basket value (Section 4.6/Section 10), a page with proven
conversion (Section 6 tells you which), and enough headroom in the margin
to actually profit after ad spend.

When that point arrives, in rough order of sense for this kind of
business:

- **Google Search Ads** on strong buying-intent terms ("buy ESP32-C3
  UK") — the closest thing to guaranteed intent.
- ~~Google Shopping~~ — not viable while checkout lives on Amazon rather
  than dhivanstech.co.uk; see §5.2's Merchant Center note. Would need
  real on-site checkout first, which isn't planned.
- **Retargeting** — showing a kit to someone who viewed the matching
  project guide but didn't buy — is a far more targeted spend than broad
  prospecting ads.

Not Meta/social ads as a starting point — broad awareness advertising for
a low-price component is a poor fit until there's a higher-value bundle
to point it at.

---

## 6. Measurement — and its real ceiling

Because the site never takes a payment, it's important to be honest about
what can and can't actually be measured, rather than implying purchase
tracking that doesn't exist.

What's directly measurable: search impressions and clicks (Search
Console), product/guide page views, and outbound clicks to Amazon. What's
**not** directly measurable without extra setup: whether that outbound
click became an actual sale.

**Amazon Attribution** is the right tool to close that gap — it's a real
Amazon program built specifically for measuring how non-Amazon marketing
(search, content, social) drives Amazon sales, via tagged links and a
reporting dashboard. DST is already Brand Registry-enrolled, which is the
main eligibility gate for Attribution — so there's no real reason left to
wait on setting it up. Its *data* won't be meaningful until there's real
outbound traffic flowing, but setting it up early means linking
conventions are correct from the first guide onward instead of needing to
be retrofitted later.

The metric that actually matters, once available, is **revenue per 1,000
organic visitors, by landing page** — not traffic volume. Two pages can
make this obvious in a way pageviews alone never will:

| Page | Visitors/month | Revenue | Revenue per 1,000 visitors |
|---|---|---|---|
| "ESP32 pinout" reference | 10,000 | £210 | £21 |
| "Best temperature sensor for ESPHome" guide | 1,400 | £460 | £329 |

The pinout page looks like the bigger success by raw traffic. It isn't —
the comparison guide is worth roughly 15x as much per visitor. Once this
kind of data exists (via Amazon Attribution, or the outbound-click-rate
proxy in the meantime), it should directly steer what gets written next:
more pages shaped like the second row, not the first.

---

## 7. Explicitly cut

Short list, so it's clear these were considered and not just forgotten:

- On-site payment/cart/returns messaging — Amazon owns all of this.
- A six-tool calculator suite — see 4.5, start with one static guide, not
  a suite.
- Programmatic SEO pages generated in bulk (by protocol, by voltage,
  etc.) with thin auto-generated text — the original brainstorm itself
  flagged this as a risk. If revisited later, each one needs real
  explanatory content, not a template dump.
- A parallel "problem-based" navigation structure — achieved through
  cross-linking instead (Section 4.6).

---

## 8. The backlog

**This is a priority order, not a calendar.** Available time on this
varies a lot week to week rather than following a steady rhythm, so
there's deliberately no "one guide every N weeks" pacing anywhere below
— that kind of target just creates guilt against a schedule that was
never realistic. The rule instead: work strictly in the order below,
finish an item (however long that actually takes), then move to the
next one. If there's a spare hour, it goes to whatever's at the top of
the list that isn't finished, not to whichever item feels most
appealing that day.

**Status column, audited against the actual repo on 2026-09-03** — ✅
built and live, 🟡 mechanism/infrastructure exists but isn't populated or
switched on yet, ⬜ not started, ◻ off-site activity this audit can't see
(nothing in the codebase would show it either way).

Reordered from the original draft based on two things confirmed since:
DST is Amazon Brand Registry-enrolled (so Attribution is a real near-term
step, not a maybe), and testing/hands-on work is closer to an actual
strength than writing is — so populating "Tested by DST" (cheap, and
mostly the same physical work as the writing-averse founder is already
good at) is pulled forward ahead of the writing-heavy guide pillar, even
though guides still lead as the main *content* pillar once that's done
(see the note on why, below the table).

| # | What | Why | Status |
|---|---|---|---|
| 1 | Custom domain migration (§5.1) | Gates QR codes and general credibility | ✅ (email decision still open, doesn't block anything else) |
| 2 | Finish Search Console + Merchant Center (§5.2) | Closes out the foundation | ✅ (Search Console done on the new domain; Merchant Center resolved as not viable for DST, see §5.2 — nothing left to do here) |
| 3 | Set up Amazon Attribution | Confirmed eligible (Brand Registry) — no reason left to wait, even though there's not much to attribute yet. This is the mechanism for "which guides/bundles/pricing actually drive sales" — see the notes below the table on why Amazon Associates isn't a substitute, and why this is set up manually rather than via API. | 🟡 (pilot campaign in progress, see note below the table) |
| 4 | Populate "Tested by DST" on a handful of real products | Cheap (badge already built), plays to actual strength, immediate trust payoff | 🟡 |
| 5 | Buying guides & comparisons (§4.1) | Highest intent-to-purchase content, cheapest to produce even though writing isn't the favourite part | ⬜ |
| 6 | Internal linking | Free, compounds every guide written | 🟡 |
| 7 | Rest of the product page standard (§2) | Real photos, cross-sell links, on the remaining catalogue | 🟡 |
| 8 | Project tutorials (§4.2) | Converts problems into product sales | 🟡 |
| 9 | Reddit + email (§5.3) | Low-cost ongoing habits, repeat visitors | 🟡 |
| 10 | Customer proof / UGC (§3) | Trust, cheap to collect once there's traffic | ⬜ |
| 11 | QR-code packaging docs (§5.5) | Turns Amazon buyers into site visitors | ⬜ |
| 12 | DST Lab testing content hub (§4.4) | Hard-to-copy differentiator, and plays to strength — bigger lift than #4, this is full write-ups | 🟡 |
| 13 | Interactive tools (§4.5) | Backlinks, repeat visitors | ⬜ |
| 14 | Video (§5.4) | Distribution, evergreen discovery — see note below | ◻ |
| 15 | Creator/sample outreach (§5.6) | Targeted awareness + backlinks, needs budget | ◻ |
| 16 | Paid advertising (§5.7) | Scale only what's already proven | ◻ |

Already done, sitting outside this ordered list because there's nothing
left to schedule: **bundles/kits (§4.6)** — a content bundle already
exists and works well; a physical combined-SKU version remains a
separate future business decision, not a backlog item.

**On Amazon Associates as a substitute for #3 — considered and rejected,
2026-09-03:** the actual goal (confirmed directly) is tracking which DST
content drives DST sales, to inform what guides/bundles/pricing to build
next. That's exactly what Amazon Attribution is for, and Attribution is
the better tool for it: no re-enrollment risk (Associates requires 3
qualifying sales within 180 days of approval, which is what closed DST's
previous Associates account, and the site doesn't have independent
traffic to reliably clear that bar yet), and no self-referral question
(Attribution is Amazon's own designed-for-sellers tool for measuring a
seller's *own* external marketing, unlike Associates, which is built for
promoting *other* people's listings). Associates would only earn its
place back on this list for a genuinely different reason — e.g. earning
commission recommending non-DST products inside buying guides (a
soldering iron or multimeter DST doesn't stock, say) — which is a real,
non-overlapping idea but a separate one from measurement, not raised yet.

**On automating Attribution product-pool management via API — considered
and rejected for now, 2026-09-03:** the console requires every product
you want sales data for to sit in a per-campaign "product pool," added
manually, with no confirmed way to auto-include new products as they're
listed — a real, ongoing maintenance task, not a one-time setup step.
Checked whether the existing Amazon Ads automation (the separate
DataAnalysis/Amazon repo, which already holds live `ADS_CLIENT_ID` /
`ADS_REFRESH_TOKEN` credentials and manages Sponsored Products bids
automatically) could be extended to cover this. It can't, cheaply: that
integration is scoped entirely to the Sponsored Products API
(`adProduct: SPONSORED_PRODUCTS`) — confirmed directly in
`pull_advertising.py`, nothing there touches Attribution at all, so
there's no existing plumbing to extend. Separately, Amazon's own
Attribution API docs describe it in agency/tool-provider terms
("agencies and integrators... on behalf of their advertiser clients"),
not clearly self-serve for a single seller the way Sponsored Products
API access was — real uncertainty about whether DST could even get
approved for direct access, not just an extra form to fill in. Weighed
against the actual problem (adding one product to a pool takes ~30
seconds, and the catalogue doesn't grow explosively often), building
real API automation for this isn't worth it at today's scale. Decision:
manage the product pool by hand in the console, folded into the
existing "list a new product" workflow as one more step, not a separate
task. Revisit only if guide/catalogue volume grows enough that manual
upkeep becomes an actual bottleneck — not preemptively.

**Attribution pilot setup, in progress 2026-09-03:** account/console
access confirmed working. A custom Publisher named "DhivanSTech" was
created (the prebuilt list is all major ad networks — eBay, Google,
etc. — nothing for an owned website, so custom entry was needed).
Channel set to "Search" for website/guide traffic — none of the five
built-in options (Social, Video, Email, Display, Search) is a precise
match for organic content traffic, but Search is the closest fit since
that's how most guide readers will actually arrive; this is a reporting
label only; it doesn't change what gets tracked. First ad group: name
"Website — ESP32 Dev Board", click-through URL
`https://www.amazon.co.uk/dp/B0DJPZHZ1X` (the ESP32 dev board's real
Amazon page), used as the single pilot destination to prove the
mechanism before creating more ad groups per product/guide. One
important note on the account itself: the Ads console this pulls
products from is not scoped to DST alone — it also lists unrelated
items (e.g. "Camera 360 HD", "QRCode Barcode Scanner Pro") priced at
£0.00, confirmed by the user to be real but not DST's — so "Add all on
this page" must never be used when adding to the product pool; always
search and add DST products by name individually.

**On outsourcing the writing (#5):** this isn't a near-term budget line
— the plan should wait for Section 6's actual revenue-per-visit data
before spending money there. Once a guide or two shows real
outbound-click/revenue performance, *that's* the trigger to consider
paying someone to write more like it, not a fixed point on this list.

**On video (#14) staying this far down despite being a real skill:**
this is my inference, not something confirmed directly — worth
correcting if it's off. Even with genuine video ability, each piece
still takes real production time that competes with everything above
it, and video seems to land better once there's an existing base of
guides/tested products to point it at and reinforce, rather than
launching into a channel with nothing yet built to support it. If
that's not the actual reasoning, this position should move.

**What the 🟡 rows actually mean, in every case** — the underlying system
was built properly, but hasn't been used on real content yet. That's a
much smaller remaining task than building it was:

- **#2 Search Console + Merchant Center** — Google Analytics (GA4),
  `sitemap.xml`, `robots.txt`, and the schema.org structured data on every
  product page are all real and already live. Search Console verification
  is done. Merchant Center is closed out as a resolved dead end (§5.2) —
  not something left to finish, something decided against.
- **#4 "Tested by DST"** — the badge logic, per-field checkmark, and
  `spec_schema.yml` behind it are all fully built and live. Zero products
  currently have a populated `verified_specs` field, so the badge has
  never actually shown up yet — correctly, since nothing's been tested
  through it yet.
- **#6 Internal linking** — two separate mechanisms exist. "Featured In"
  (auto-links a product to any project referencing its ASIN) is built
  *and populated* — e.g. the ESP32/WS2812B/KY-040 products already link
  to the mood-ring project. "Pairs Well With" (manually curated cross-sell
  via a `related:` field) is built but not populated on any product yet.
- **#8 Project tutorials** — ahead of where the plan assumed: three real
  build-log posts already exist (bench PSU build, 3D-printer endstop
  swap, ESP32 rotary RGB mood ring), each with real wiring, code, and a
  "what actually mattered" gotchas section. They're written as build logs
  rather than this plan's "problem → buy these parts → build it" framing,
  but the mood-ring one is already close in spirit and cross-links to its
  three component products.
- **#9 Reddit + email** — email has a real, working signup form
  (`_includes/newsletter.html`), just switched off (`enabled: false`,
  no email service connected yet). Reddit is an off-site habit this audit
  can't see either way.
- **#12 DST Lab** — `how-we-test.md` documents the testing methodology
  and badge system properly, but there's no "Lab" content hub with
  individual test/teardown write-ups yet — one policy page, not yet a
  content pillar.

---

## 9. The long-term shape of this (speculative — not a roadmap item)

Worth naming, even though none of it is actionable yet: the further this
works, the more the business can shift shape entirely, roughly:

**Seller of generic components → curator of tested components → designer
of kits → designer of DST's own hardware.**

The end state of that progression is something like a "DST Air Sensor
Board" — an ESP32-C3 and a temperature/humidity sensor on one small,
purpose-built, Home-Assistant-ready board, rather than three generic parts
someone has to wire together themselves. At that point, everything built
under this plan (the guides, the trust system, the audience) stops being
just marketing for other people's components and becomes the distribution
channel for a genuinely differentiated, higher-margin product DST
actually owns.

This is named here because it's the reason the earlier investment (content,
trust, documentation) compounds rather than just being marketing spend —
not because it's something to act on now. Designing and selling actual
hardware is a real step up in commitment (certification, manufacturing,
inventory risk) and belongs in its own dedicated planning conversation
once there's a real content/trust base to launch it into, not folded into
this website plan. See Section 10.

---

## 10. Open decisions (not assumed)

These need an actual decision from you before they can move from "idea"
to "plan" — nothing here has been assumed one way or the other:

- **A real, physical combined-SKU kit** (as opposed to the content-only
  bundle already live, see Section 8) would still need a new Amazon
  listing, real packaging, and an inventory commitment. Worth pursuing
  eventually, but it's a business decision with real cost, not something
  to fold into the website roadmap without your sign-off.
- **Which products get the QR-code packaging treatment** — depends on
  which products DST actually controls the physical packaging for; needs
  your input on which those are.
- **Creator/sample outreach budget** — this is the one distribution idea
  in this plan that costs real money up front (free parts sent out) for
  an uncertain return. Needs an actual budget figure and a decision on
  timing from you before it moves beyond "later, maybe."
- **Own-hardware product line** — Section 9's long-term direction is
  named so the earlier work has a clear "why," not because it's a current
  proposal. If it's ever pursued, it needs its own dedicated planning
  conversation (certification, manufacturing, inventory risk are real
  commitments this document isn't scoped to cover) — not something to
  assume is on the table.
