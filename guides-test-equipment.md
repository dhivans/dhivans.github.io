---
layout: page
title: "Buying Guide: Test Equipment"
permalink: /guides/test-equipment/
---

<p class="shop-intro">A live shortlist of DST test equipment. Use it as a starting point for comparing meters and bench tools.</p>

<div class="deals-list">
{% for product in site.data.catalog %}
  {% if product.category == "test-equipment" %}
  <article class="deal-row">
    <div>
      <a href="{{ product.page_url | relative_url }}">{{ product.title }}</a>
      <span>{% if product.in_stock %}In stock{% else %}Out of stock{% endif %}</span>
    </div>
    <div class="deal-prices">
      <strong>{{ product.price }}</strong>
      {% if product.lowest_90d %}<span>90-day low {{ product.currency }} {{ product.lowest_90d }}</span>{% endif %}
    </div>
    <a href="/compare/?asins={{ product.asin }}" class="btn-buy-now">Compare</a>
  </article>
  {% endif %}
{% endfor %}
</div>
