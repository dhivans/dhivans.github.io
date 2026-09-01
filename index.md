---
layout: default
title: DhivanSTech
---

<div class="home-page">
  <section class="home-hero">
    <div class="home-hero-copy">
      <a class="home-brand-mark" href="/" aria-label="DhivanSTech home">
        <span class="home-brand-mark__full">DhivanSTech</span>
      </a>
      <h1>Tested maker components. No mystery parts.</h1>
      <p>
        Curated electronics parts, tools, and test equipment with practical notes,
        verified specs, and Amazon availability for serious DIY builds.
      </p>
      <div class="home-actions">
        <a class="home-btn home-btn--primary" href="/shop/">Browse Shop</a>
        <a class="home-btn home-btn--secondary" href="/compare/">Compare Products</a>
      </div>
    </div>
    <div class="home-hero-panel" aria-label="Site focus">
      <div>
        <span>Catalogue</span>
        <strong>{{ site.data.catalog | size }}</strong>
      </div>
      <div>
        <span>Focus</span>
        <strong>Components</strong>
      </div>
      <div>
        <span>Fulfilment</span>
        <strong>Amazon</strong>
      </div>
    </div>
  </section>

  <section class="home-value-strip" aria-label="Why use DhivanSTech">
    <div>
      <strong>Visually QC'd</strong>
      <span>Every item is physically handled and checked by DST before it ships.</span>
    </div>
    <div>
      <strong>Stock-Aware Picks</strong>
      <span>Prices and availability surfaced where they matter.</span>
    </div>
    <div>
      <strong>Build-Tested Gear</strong>
      <span>Parts chosen for real projects, not filler listings.</span>
    </div>
  </section>

  <section class="home-section">
    <div class="home-section-heading">
      <h2>Featured Products</h2>
      <a href="/shop/">View all</a>
    </div>
    <div class="home-featured-grid">
      {% assign featured_count = 0 %}
      {% for product in site.data.catalog %}
        {% if product.in_stock and featured_count < 4 %}
          {% assign featured_count = featured_count | plus: 1 %}
          <article class="home-product-card">
            <a href="{{ product.page_url | relative_url }}" class="home-product-image">
              <img src="{{ product.image }}" alt="" loading="lazy" onerror="this.src='/assets/images/products/placeholder.svg'">
            </a>
            <div class="home-product-body">
              {% if product.category %}
                <a class="home-product-category" href="/shop/?category={{ product.category }}">{{ product.category | replace: '-', ' ' }}</a>
              {% endif %}
              <h3><a href="{{ product.page_url | relative_url }}">{{ product.title }}</a></h3>
              <div class="home-product-meta">
                <strong>{{ product.price | default: "Check price" }}</strong>
                <span class="home-stock home-stock--in">In stock</span>
              </div>
            </div>
          </article>
        {% endif %}
      {% endfor %}
    </div>
  </section>

  <section class="home-section">
    <div class="home-section-heading">
      <h2>Shop by Category</h2>
    </div>
    <div class="home-category-grid">
      {% for category in site.data.categories %}
        <a class="home-category-card" href="/shop/?category={{ category.id }}">
          <span>{{ category.label }}</span>
          <small>Browse {{ category.label | downcase }}</small>
        </a>
      {% endfor %}
    </div>
  </section>

  <section class="home-section">
    <div class="home-section-heading">
      <h2>Latest Builds & Notes</h2>
      <a href="/projects/">View projects</a>
    </div>
    <div class="home-post-list">
      {% for post in site.posts limit:3 %}
        <article class="home-post-card">
          <time datetime="{{ post.date | date_to_xmlschema }}">{{ post.date | date: "%b %-d, %Y" }}</time>
          <h3><a href="{{ post.url | relative_url }}">{{ post.title }}</a></h3>
          {% if post.excerpt %}
            <p>{{ post.excerpt | strip_html | truncate: 150 }}</p>
          {% endif %}
        </article>
      {% endfor %}
    </div>
  </section>
</div>
