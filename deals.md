---
layout: page
title: Deals
permalink: /deals/
---

{% assign drops = site.data.price_changes | where: "direction", "drop" %}

<p class="shop-intro">Current price drops and 90-day lows from the latest catalogue sync. Amazon prices can change after the sync.</p>

{% if drops and drops.size > 0 %}
<h2>Price Drops</h2>
<div class="deals-list">
  {% for drop in drops %}
  {% assign product = nil %}
  {% for item in site.data.catalog %}
    {% if item.asin == drop.asin %}
      {% assign product = item %}
      {% break %}
    {% endif %}
  {% endfor %}
  {% if product %}
  <article class="deal-row">
    <div>
      <a href="{{ product.page_url | relative_url }}">{{ product.title }}</a>
      {% if product.variant %}<span>{{ product.variant }}</span>{% endif %}
    </div>
    <div class="deal-prices">
      <span>{{ drop.currency }} {{ drop.old_price }}</span>
      <strong>{{ drop.currency }} {{ drop.new_price }}</strong>
    </div>
    <a href="{{ product.amazon_url }}" class="btn-buy-now" target="_blank" rel="noopener noreferrer">Amazon</a>
  </article>
  {% endif %}
  {% endfor %}
</div>
{% endif %}

<h2>Lowest Seen</h2>
<div class="deals-list">
  {% assign low_items = site.data.catalog | sort: "lowest_90d" %}
  {% for product in low_items limit: 24 %}
    {% if product.lowest_90d %}
    <article class="deal-row">
      <div>
        <a href="{{ product.page_url | relative_url }}">{{ product.title }}</a>
        {% if product.variant %}<span>{{ product.variant }}</span>{% endif %}
      </div>
      <div class="deal-prices">
        <span>Current {{ product.price }}</span>
        <strong>90-day low {{ product.currency }} {{ product.lowest_90d }}</strong>
      </div>
      <a href="{{ product.amazon_url }}" class="btn-buy-now" target="_blank" rel="noopener noreferrer">Amazon</a>
    </article>
    {% endif %}
  {% endfor %}
</div>
