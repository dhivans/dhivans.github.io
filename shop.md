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
  {% comment %}Show only products that have at least 1 unit in stock.
  For variant products, check if any individual variant has stock > 0.
  For single products, check the top-level stock field.{% endcomment %}
  {% assign in_stock = false %}
  {% if product.variants %}
    {% for v in product.variants %}
      {% if v.stock > 0 %}{% assign in_stock = true %}{% endif %}
    {% endfor %}
  {% elsif product.stock > 0 %}
    {% assign in_stock = true %}
  {% endif %}
  {% unless in_stock %}{% continue %}{% endunless %}
  <div class="product-card-h product-card">
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
        <div class="product-card-bottom-left">
          <span class="price">{{ product.price }}</span>
          {% if product.variants and product.variants.size > 0 %}
          <a href="{{ product.variants[0].url }}" class="btn-buy-now" target="_blank" rel="noopener noreferrer">Buy now &rarr;</a>
          {% endif %}
        </div>
        <a href="{{ product.url }}" class="product-view-details">View details &rarr;</a>
      </div>
    </div>
  </div>
  {% endfor %}
</div>
