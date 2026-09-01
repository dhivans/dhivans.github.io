---
layout: page
title: "Help Us Document the Catalogue"
permalink: /help-document/
---

<p class="shop-intro">We ask manufacturers for real specs first. When they can't provide them, we ask people who own the part — in exchange for a free one.</p>

## How it works

1. Pick an open request below.
2. Hit **Apply by email** — it pre-fills the product and what's needed.
3. Add your UK address and send.
4. If picked, DST posts the item free.
5. Send back what's listed, with a photo as evidence.

## Rules

- UK only.
- One product must be completed before another is sent out.
- Photo evidence required with every submission.
- Please provide as much detail as possible.

## Open requests

<div class="deals-list">
{% assign has_open = false %}
{% for req in site.data.info_requests %}
  {% if req.status == "open" %}
    {% assign has_open = true %}
    {% assign product = nil %}
    {% for item in site.data.catalog %}
      {% if item.slug == req.slug %}
        {% assign product = item %}
        {% break %}
      {% endif %}
    {% endfor %}
    {% if product %}
    {% assign body_l1 = "I'd like to apply for: " | append: product.title | url_encode %}
    {% assign wanted_lines = "" %}
    {% for w in req.wanted %}
      {% assign line = "- " | append: w | url_encode %}
      {% assign wanted_lines = wanted_lines | append: line | append: "%0A" %}
    {% endfor %}
    {% assign body_l2 = "I confirm I will provide:" | url_encode %}
    {% assign body_l3 = "My UK shipping address:" | url_encode %}
    {% assign body_l4 = "[add your address here]" | url_encode %}
    {% assign mailto_body = body_l1 | append: "%0A%0A" | append: body_l2 | append: "%0A" | append: wanted_lines | append: "%0A" | append: body_l3 | append: "%0A" | append: body_l4 %}
    {% assign mailto_subject = "Info request: " | append: product.title | url_encode %}
    <article class="deal-row help-doc-row">
      <div>
        <a href="{{ product.page_url | relative_url }}">{{ product.title }}</a>
        <ul class="help-doc-wanted">
          {% for w in req.wanted %}<li>{{ w }}</li>{% endfor %}
        </ul>
      </div>
      <div class="deal-prices">
        <span>{{ req.units_given | default: 0 }} of {{ req.max_units }} claimed</span>
      </div>
      <a href="mailto:dhivanshahtech+dstspecs@gmail.com?subject={{ mailto_subject }}&body={{ mailto_body }}" class="btn-buy-now">Apply by email</a>
    </article>
    {% endif %}
  {% endif %}
{% endfor %}
{% unless has_open %}
<p>Nothing open right now — check the <a href="/shop/">Shop</a>.</p>
{% endunless %}
</div>

<style>
.help-doc-wanted {
  margin: 0.3rem 0 0;
  padding-left: 1.1rem;
  font-size: 0.85rem;
  color: #8b949e;
}
.help-doc-wanted li {
  margin: 0.1rem 0;
}
</style>
