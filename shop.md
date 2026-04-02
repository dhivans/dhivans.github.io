---
layout: page
title: Shop
permalink: /shop/
body_class: page-shop
---

<p class="shop-intro">Tested components and bench tools — sourced and verified by DST, available on Amazon.
Prices are approximate and may vary. Amazon links are affiliate links.</p>

<div class="shop-toolbar">
  <input type="text" id="shop-search-bar" class="shop-search"
         placeholder="Filter products by name or description…" autocomplete="off">
  <div class="shop-settings-wrap">
    <button class="shop-settings-btn" id="shop-settings-btn" aria-label="Shop settings" title="Settings">
      <svg xmlns="http://www.w3.org/2000/svg" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
        <circle cx="12" cy="12" r="3"/>
        <path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-4 0v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83-2.83l.06-.06A1.65 1.65 0 0 0 4.68 15a1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1 0-4h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 2.83-2.83l.06.06A1.65 1.65 0 0 0 9 4.68a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 4 0v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 2.83l-.06.06A1.65 1.65 0 0 0 19.4 9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1z"/>
      </svg>
    </button>
    <div class="shop-settings-popover" id="shop-settings-popover" hidden>
      <div class="shop-settings-row">
        <span class="shop-settings-label">Show out-of-stock</span>
        <button class="toggle-pill" id="toggle-out-of-stock" role="switch" aria-checked="false" type="button">
          <span class="toggle-pill-thumb"></span>
        </button>
      </div>
    </div>
  </div>
</div>

<div class="product-list-h">
  {% for product in site.products %}
  {% comment %}
    Stock data is set on each card via data attributes so JS can
    filter dynamically without a page reload.
  {% endcomment %}
  {% assign in_stock = false %}
  {% if product.variants %}
    {% for v in product.variants %}
      {% if v.stock > 0 %}{% assign in_stock = true %}{% endif %}
    {% endfor %}
  {% elsif product.stock > 0 %}
    {% assign in_stock = true %}
  {% endif %}
  <div class="product-card-h product-card" data-in-stock="{% if in_stock %}1{% else %}0{% endif %}" data-href="{{ product.url }}">
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
        {% if product.variants and product.variants.size > 0 %}
          {% assign buy_url = product.variants[0].url %}
        {% elsif product.amazon_url %}
          {% assign buy_url = product.amazon_url %}
        {% endif %}
        {% if buy_url %}
        <a href="{{ buy_url }}" class="btn-buy-now" target="_blank" rel="noopener noreferrer">Buy Now</a>
        {% endif %}
      </div>
    </div>
  </div>
  {% endfor %}
</div>

<style>
.product-card-h[data-href] {
  cursor: pointer;
}
.shop-toolbar {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  margin-bottom: 1.5rem;
}
.shop-toolbar .shop-search {
  flex: 1;
  margin-bottom: 0;
}
.shop-settings-wrap {
  position: relative;
  flex-shrink: 0;
}
.shop-settings-btn {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 2.4rem;
  height: 2.4rem;
  background: #161b22;
  border: 1px solid #30363d;
  border-radius: 4px;
  color: #8b949e;
  cursor: pointer;
  transition: color 0.2s, border-color 0.2s;
}
.shop-settings-btn:hover,
.shop-settings-btn.active {
  color: #00e5ff;
  border-color: rgba(0,229,255,0.4);
}
.shop-settings-popover {
  position: absolute;
  top: calc(100% + 0.4rem);
  right: 0;
  background: #161b22;
  border: 1px solid #30363d;
  border-radius: 6px;
  padding: 0.75rem 1rem;
  min-width: 220px;
  z-index: 100;
  box-shadow: 0 4px 16px rgba(0,0,0,0.4);
}
.shop-settings-row {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 1rem;
}
.shop-settings-label {
  font-size: 0.9rem;
  color: #c9d1d9;
}
/* Toggle pill */
.toggle-pill {
  position: relative;
  width: 2.6rem;
  height: 1.4rem;
  background: #30363d;
  border: none;
  border-radius: 999px;
  cursor: pointer;
  flex-shrink: 0;
  transition: background 0.2s;
  padding: 0;
}
.toggle-pill[aria-checked="true"] {
  background: #00e5ff;
}
.toggle-pill-thumb {
  position: absolute;
  top: 0.15rem;
  left: 0.15rem;
  width: 1.1rem;
  height: 1.1rem;
  background: #fff;
  border-radius: 50%;
  transition: transform 0.2s;
}
.toggle-pill[aria-checked="true"] .toggle-pill-thumb {
  transform: translateX(1.2rem);
}
</style>

<script>
(function () {
  var btn     = document.getElementById('shop-settings-btn');
  var popover = document.getElementById('shop-settings-popover');
  var toggle  = document.getElementById('toggle-out-of-stock');

  var showOutOfStock = false;

  // Clickable cards — navigate to detail page unless click is on a link
  document.querySelectorAll('.product-card-h[data-href]').forEach(function (card) {
    card.addEventListener('click', function (e) {
      if (e.target.closest('a')) return;
      window.location.href = card.dataset.href;
    });
  });

  // Override the global filterCards defined in default.html so that both
  // the sidebar search and the shop search bar respect the stock toggle.
  window.filterCards = function (query) {
    var cards = document.querySelectorAll('.product-card');
    query = (query || '').toLowerCase();
    cards.forEach(function (card) {
      var inStock     = card.dataset.inStock !== '0';  // undefined = single product, treat as in-stock
      var matchSearch = query === '' || card.textContent.toLowerCase().indexOf(query) !== -1;
      var matchStock  = inStock || showOutOfStock;
      card.style.display = (matchSearch && matchStock) ? '' : 'none';
    });
  };

  // Gear button — open/close popover
  btn.addEventListener('click', function (e) {
    e.stopPropagation();
    var isHidden = popover.hasAttribute('hidden');
    if (isHidden) {
      popover.removeAttribute('hidden');
      btn.classList.add('active');
    } else {
      popover.setAttribute('hidden', '');
      btn.classList.remove('active');
    }
  });

  // Close popover when clicking outside
  document.addEventListener('click', function () {
    popover.setAttribute('hidden', '');
    btn.classList.remove('active');
  });
  popover.addEventListener('click', function (e) { e.stopPropagation(); });

  // Toggle pill — re-run with current search value
  toggle.addEventListener('click', function () {
    showOutOfStock = !showOutOfStock;
    toggle.setAttribute('aria-checked', showOutOfStock ? 'true' : 'false');
    var q = (document.getElementById('shop-search-bar') || {}).value || '';
    window.filterCards(q);
  });

  // Initial render — hide out-of-stock by default
  window.filterCards('');
}());
</script>
