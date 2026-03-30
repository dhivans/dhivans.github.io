---
layout: page
title: Shop
permalink: /shop/
---

Tested components and bench tools — sourced and verified by DST, available on Amazon.
Prices are approximate and may vary. Amazon links are affiliate links.

<div class="product-grid">
  {% for product in site.products %}
  <div class="product-card">
    <h3>{{ product.title }}</h3>
    <p>{{ product.description }}</p>
    <div class="price">{{ product.price }}</div>
    <a href="{{ product.amazon_url }}" class="btn-amazon" target="_blank" rel="noopener noreferrer">
      View on Amazon &rarr;
    </a>
  </div>
  {% endfor %}
</div>
