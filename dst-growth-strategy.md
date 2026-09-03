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

**A domain is being handled separately.** This plan assumes a proper
domain exists or will exist wherever it references email capture, QR
codes, or "your domain" — it doesn't make that decision for you.

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
disconnected pages. Treat it as a standing habit from Phase 1 onward, not
a separate project.

**Real photography over the wholesaler's stock image.** Front, back,
scale/dimension shot, connector close-up, the part in use if practical.
This is genuinely one of the more effort-heavy items here, so treat it as
an ongoing background task applied to best-sellers first, not a blocking
requirement before anything else ships.

**"Tested by DST" — defined, not vague.** See Section 3. Only claim it
where it's true, but where it's true, say exactly what was checked.

---

## 3. Trust system: "Tested by DST"

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

### 4.1 Phase 1 pillar: buying guides and comparisons

Start here. This is the highest intent-to-purchase content that exists,
and the cheapest to produce well — no photography rig, no wiring, no
filming, just genuine technical knowledge written down clearly.

Someone searching "BMP280 vs BME280" or "which ESP32 should I buy" is
functionally asking to be sold to. A short, honest, well-structured page —
what's different, a simple comparison table, a clear "if you need X, buy
this one" recommendation, links to both products — can quietly sell
product for years with near-zero maintenance once written.

Realistic cadence for one part-time person: **one genuinely good page
every one to two weeks**, not the batch of five-per-category the original
brainstorm suggested producing at once. Quality and correctness matter far
more than volume here — a wrong comparison actively damages trust.

Pick topics from what's already in the catalogue first (comparisons
between products actually stocked), then expand once Search Console shows
real demand (see Section 6).

### 4.2 Phase 2 pillar: project tutorials

Don't start this until Phase 1 is producing measurable outbound clicks to
Amazon (see Section 6) — a project tutorial is far more expensive to
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

Genuinely one of the strongest differentiators in the original
brainstorm — actual measurements are much harder for a competitor to
copy than written content, and "we measured it so you don't have to" is a
real, memorable brand position. But it needs real instrumentation time per
piece and is one of the more effort-heavy formats here. Revisit this once
Phase 1 is running on its own and there's spare capacity, not before.

### 4.5 Later, and only in a much smaller form: interactive tools

The original brainstorm proposed a full "component finder" quiz (checkbox
questions, branching logic, six separate calculators). That's real
software to build and maintain for uncertain payoff, and it's the kind of
thing that's easy to sink weeks into.

Get most of the value for a fraction of the effort first: plain static
pages ("I want to measure temperature → BMP280 or BME280, here's why")
achieve most of what the interactive version would, are just more content
written under the Phase 1 pillar, and cost nothing to maintain. If one of
these static guides turns out to attract serious traffic and the "which
one do I need" decision is genuinely complex enough to deserve interactive
logic, build *that one* tool — not the full six-tool suite up front.

### 4.6 Cut or folded in

- **"Product passport" pages** — this is just the product page standard
  from Section 2 done well. Treating it as a separate format duplicates
  effort for no benefit.
- **Bundles/kits** — genuinely good for average-order-value, but this
  requires new Amazon listings, real packaging, and inventory decisions.
  It's a business decision, not a website task — see Section 10.
- **Beginner "journey" pages, problem-based navigation rebuild** — good
  instincts, but this is largely what a well-cross-linked set of buying
  guides and product pages already achieves (Section 2's "commonly used
  with," Section 4.1's guides). Building a whole separate navigation
  structure for it is effort better spent on the content itself.

---

## 5. Distribution

Ranked by effort-to-value ratio for one part-time person — not in the
order the original brainstorm presented them.

### 5.1 Do this first, before writing anything: measurement and technical setup

This is almost entirely one-time setup work, has zero ongoing content
burden, and makes everything after it measurable instead of a guess.

- **Google Search Console** — non-negotiable, set up before any content
  work starts. This is also where future content topics should come from
  (see Section 6) rather than guessing.
- **Product structured data** (schema.org Product markup: price, stock,
  brand, images, reviews) on every product page.
- **Google Merchant Center** — free product listings across Search,
  Shopping, Images, and related surfaces for eligible products. High
  leverage, mostly a one-time technical/data-feed task rather than
  ongoing content work, worth doing early.
- **Basic analytics** on key events — product page view, outbound Amazon
  click, guide page view — kept simple, not an elaborate dashboard.

### 5.2 Ongoing, low-cost habits: Reddit and email

Neither of these is a content production line — they're standing habits
that reuse content already being written elsewhere, not a new workload.

**Reddit.** Be genuinely useful in relevant communities (r/esp32,
r/homeassistant, r/AskElectronics and similar) when a question comes up
that a real answer helps with, occasionally linking a guide where it's
actually relevant. This costs minutes, not hours, and reads as authentic
specifically because it isn't a campaign. Explicitly not: promotional
posts, "check out my store."

**Email.** Keep it small and low-pressure — no "10% off, sign up now"
popup on arrival. Something like "get the ESP32-C3 pinout PDF" or "one
genuinely useful project a month" gives people a real reason to hand over
an email address, and gives repeat visitors a reason to come back without
needing fresh distribution effort each time. Don't start this until
there's Phase 1 content worth sending — an empty list with nothing useful
to mail is wasted setup.

### 5.3 Deferred: video (YouTube and short-form)

The original brainstorm ranked YouTube as the single highest-priority
channel. For a solo part-time operator, video is one of the most
expensive content formats to produce well — filming, editing, thumbnails,
consistency — and the same underlying knowledge is already being spent on
Phase 1's written guides. Revisit once the written content pipeline is
sustainable and there's evidence (Section 6) that it converts, not as a
launch commitment. The same applies to repurposing content across
Instagram/TikTok/Pinterest — real production overhead for channels that
haven't been validated yet.

### 5.4 Later, needs real-world coordination: packaging QR-code funnel

Genuinely clever: a QR code on packaging linking to a product-specific
"thanks for buying, here's your quick-start" page turns an Amazon sale
(where DST has no direct relationship with the buyer) into a visit to
DST's own site. But it only works for products where DST controls the
physical packaging — not generic dropshipped items — and needs the
landing pages built and a real domain in place first. Worth doing, not
worth doing before Phase 1 content and the basics in Section 5.1 are
running.

### 5.5 Later, needs real budget: creator/sample outreach

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

### 5.6 Last, and only once something is proven: paid advertising

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
- **Google Shopping**, once structured data and Merchant Center
  (Section 5.1) are already in place.
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
reporting dashboard. Worth investigating properly (there are eligibility
requirements to check) once there's enough outbound traffic for the data
to be meaningful — not needed on day one, but flag it early so linking
conventions can be set up correctly from the start rather than retrofitted
later.

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

## 8. Roadmap

Paced for one part-time person. These are phases, not sprints — each one
should be running comfortably on its own before starting the next. The
table below is everything in this document in one ranked, scannable list;
the phases underneath explain the pacing logic behind that order.

| # | What | Why | Phase |
|---|---|---|---|
| 1 | Product page standard (§2) | Converts the visitors already arriving | 0 |
| 2 | Search Console + basic analytics | Tells you what's actually working | 0 |
| 3 | Structured data + Merchant Center | Free discovery, mostly one-time setup | 0 |
| 4 | Buying guides & comparisons (§4.1) | Highest intent-to-purchase content, cheapest to produce | 1 |
| 5 | Internal linking | Free, compounds every guide written | 1, ongoing |
| 6 | Project tutorials (§4.2) | Converts problems into product sales | 2 |
| 7 | Reddit + email (§5.2) | Low-cost ongoing habits, repeat visitors | 2 |
| 8 | Customer proof / UGC (§3) | Trust, cheap to collect once there's traffic | 2 |
| 9 | QR-code packaging docs (§5.4) | Turns Amazon buyers into site visitors | 3 |
| 10 | Bundles/kits (§4.6) | Increases basket size | 3, needs your sign-off |
| 11 | DST Lab testing content (§4.4) | Hard-to-copy differentiator | 3+ |
| 12 | Interactive tools (§4.5) | Backlinks, repeat visitors | 3+ |
| 13 | Video (§5.3) | Distribution, evergreen discovery | 3+ |
| 14 | Creator/sample outreach (§5.5) | Targeted awareness + backlinks | 3+, needs budget |
| 15 | Paid advertising (§5.6) | Scale only what's already proven | Last |

**Phase 0 — Foundation**
Search Console, structured data, Google Merchant Center feed, basic
analytics events. Apply the Section 2 product-page standard to the
current best-sellers first (real photos, honest differentiation, "Tested
by DST" where true, cross-sell links).

**Phase 1 — Buying guides**
One well-researched comparison/buying guide roughly every one to two
weeks, prioritising products already in the catalogue, linking to every
product and guide it reasonably relates to. Watch Search Console for what
people are actually searching for and let that steer future topics.

**Phase 2 — Expand, informed by data**
Once Phase 1 shows real outbound-click movement: extend the product-page
standard to the rest of the catalogue, start Reddit and email as ongoing
habits, start collecting customer build photos, look into Amazon
Attribution, and begin the first project tutorials for whichever guide
topics are performing best.

**Phase 3 and beyond — revisit deferred items**
QR-code packaging funnel, bundles/kits (pending sign-off), DST Lab
testing content, interactive tooling, video, and creator outreach (pending
budget) — only once the earlier phases are running themselves and there's
spare capacity or clear evidence justifying the extra effort. Paid
advertising comes last, and only once a bundle or proven page exists to
point it at.

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

- **Bundles/kits** — these need new Amazon listings, real packaging, and
  an inventory commitment. Worth pursuing, but it's a business decision
  with real cost, not something to fold into the website roadmap without
  your sign-off.
- **Video, ever** — deferred for now on effort grounds, not ruled out
  permanently. Worth revisiting explicitly once Phase 1/2 are running, but
  only if there's real appetite for it — not assumed here either way.
- **Which products get the QR-code packaging treatment** — depends on
  which products DST actually controls the physical packaging for; needs
  your input on which those are.
- **Amazon Attribution eligibility** — needs checking against your actual
  seller account setup before it's relied on; not something to assume is
  available.
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
