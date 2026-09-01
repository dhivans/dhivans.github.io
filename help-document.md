---
layout: page
title: "Help Us Document the Catalogue"
permalink: /help-document/
---

<p class="shop-intro">DST wants real, checked technical detail on every product — dimensions, capabilities, exact specs — not just whatever the Amazon listing happens to say. We ask manufacturers directly first. When they can't or won't provide it, we ask people who actually own the part.</p>

## How it works

1. **Check the list below** for products currently requesting information. Each one says exactly what's needed.
2. **Email [dhivanshahtech+dstspecs@gmail.com](mailto:dhivanshahtech+dstspecs@gmail.com)** — say which product you want, confirm you'll send back the requested information (with a photo as evidence — e.g. a caliper reading, or the part next to a ruler), and include your UK shipping address.
3. **That email is the whole application.** There's no form. DST reviews requests directly and decides who gets a product — being on this list is not a guarantee, and not every applicant will be selected.
4. **If you're picked**, DST posts the item to you at no cost.
5. **You send back what you agreed to provide.** Once a product's request is fulfilled, it's marked closed here and comes off this list until reopened.

## The ground rules

- **UK residents only.**
- Every open request below has a cap on how many units go out for it. Once that's reached, it closes — applying again for a closed product won't do anything until it reopens.
- Photo evidence is required alongside any measurement or claim — a number with no way to check it isn't useful to anyone.
- Whatever you send back gets credited as community-provided information on the product page, clearly labelled as such — it's never presented as something DST independently verified.

## Currently open requests

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
    {% assign body_l2 = "I confirm I will provide: " | append: req.wanted | url_encode %}
    {% assign body_l3 = "My UK shipping address:" | url_encode %}
    {% assign body_l4 = "[add your address here]" | url_encode %}
    {% assign mailto_body = body_l1 | append: "%0A%0A" | append: body_l2 | append: "%0A%0A" | append: body_l3 | append: "%0A" | append: body_l4 %}
    {% assign mailto_subject = "Info request: " | append: product.title | url_encode %}
    <article class="deal-row">
      <div>
        <a href="{{ product.page_url | relative_url }}">{{ product.title }}</a>
        <span>{{ req.wanted }}</span>
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
<p>Nothing open right now — check back later, or see the full catalogue in the <a href="/shop/">Shop</a>.</p>
{% endunless %}
</div>
