---
layout: page
title: Shop
permalink: /shop/
body_class: page-shop
---

<p class="shop-intro">Tested components and bench tools — sourced and verified by DST, available on Amazon.
Prices are approximate and may vary. Amazon links are affiliate links.</p>

<input type="text" id="shop-search-bar" class="shop-search"
       placeholder="Filter products by name or description…" autocomplete="off">

<div class="product-list-h">
  {% for product in site.products %}
  <div class="product-card-h">
    <div class="product-card-image">
      <img
        src="{% if product.image %}{{ product.image }}{% else %}/assets/images/products/placeholder.svg{% endif %}"
        alt="{{ product.title }}"
        onerror="this.src='/assets/images/products/placeholder.svg'">
    </div>
    <div class="product-card-content">
      <div class="product-card-top">
        {% if product.category %}
        <span class="product-category-tag">{{ product.category | replace: '-', ' ' }}</span>
        {% endif %}
        <h3 class="product-card-title">{{ product.title }}</h3>
        <p class="product-card-desc">{{ product.description }}</p>
      </div>
      <div class="product-card-bottom">
        <span class="price">{{ product.price }}</span>
        <a href="{{ product.amazon_url }}" class="btn-amazon" target="_blank" rel="noopener noreferrer">
          View on Amazon &rarr;
        </a>
      </div>
    </div>
  </div>
  {% endfor %}
</div>
