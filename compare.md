---
layout: page
title: Compare
permalink: /compare/
---

<p class="shop-intro">Compare up to three catalogue items by verified specs, stock state, and current Amazon link.</p>

<style>
  .compare-cards--count-1 {
    max-width: 720px;
  }

  .compare-card-image {
    height: clamp(260px, 42vh, 420px);
  }

  .compare-cards--count-1 .compare-card-image {
    height: clamp(220px, 38vh, 360px);
  }

  .compare-card-image img {
    display: block;
    height: 100%;
    object-fit: contain;
    width: 100%;
  }
</style>

<div id="compare-app" class="compare-app">
  <div class="compare-picker">
    <label for="compare-search">Add a component</label>
    <div class="compare-search-wrap">
      <input type="text" id="compare-search" placeholder="Search by name…" autocomplete="off" aria-autocomplete="list" aria-controls="compare-results" aria-expanded="false">
      <div id="compare-results" class="compare-results" hidden></div>
    </div>
  </div>
  <div id="compare-cards" class="compare-cards"></div>
  <div id="compare-table"></div>
</div>

<script type="application/json" id="catalog-json">{{ site.data.catalog | jsonify }}</script>
<script>
(function () {
  var catalog = JSON.parse(document.getElementById('catalog-json').textContent);
  var selected = new URLSearchParams(window.location.search).get('asins');
  var asins = selected ? selected.split(',').filter(Boolean).slice(0, 3) : [];
  var MAX_ITEMS = 3;

  var search = document.getElementById('compare-search');
  var results = document.getElementById('compare-results');
  var cardsEl = document.getElementById('compare-cards');
  var tableEl = document.getElementById('compare-table');

  function itemFor(asin) {
    return catalog.find(function (item) { return item.asin === asin; });
  }

  function escapeHtml(value) {
    return String(value).replace(/[&<>"']/g, function (c) {
      return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c];
    });
  }

  function updateUrl() {
    var url = new URL(window.location.href);
    if (asins.length) url.searchParams.set('asins', asins.join(','));
    else url.searchParams.delete('asins');
    history.replaceState(null, '', url);
  }

  function addItem(asin) {
    if (asins.indexOf(asin) === -1 && asins.length < MAX_ITEMS) asins.push(asin);
    search.value = '';
    closeResults();
    render();
  }

  function removeItem(asin) {
    asins = asins.filter(function (a) { return a !== asin; });
    render();
  }

  function closeResults() {
    results.hidden = true;
    results.innerHTML = '';
    search.setAttribute('aria-expanded', 'false');
  }

  function renderResults() {
    var query = search.value.trim().toLowerCase();
    if (!query || asins.length >= MAX_ITEMS) {
      closeResults();
      return;
    }
    var matches = catalog
      .filter(function (item) { return asins.indexOf(item.asin) === -1; })
      .filter(function (item) { return item.title.toLowerCase().indexOf(query) !== -1; })
      .slice(0, 8);

    if (!matches.length) {
      results.innerHTML = '<div class="compare-results-empty">No matching components.</div>';
      results.hidden = false;
      search.setAttribute('aria-expanded', 'true');
      return;
    }

    results.innerHTML = matches.map(function (item) {
      return '<button type="button" class="compare-result" data-asin="' + item.asin + '">' +
        '<span class="compare-result-title">' + escapeHtml(item.title) + '</span>' +
        '<span class="compare-result-meta">' + escapeHtml(item.price || '') + (item.in_stock ? '' : ' · Out of stock') + '</span>' +
        '</button>';
    }).join('');
    results.hidden = false;
    search.setAttribute('aria-expanded', 'true');
  }

  search.addEventListener('input', renderResults);
  search.addEventListener('focus', renderResults);
  search.addEventListener('keydown', function (e) {
    if (e.key === 'Escape') closeResults();
    if (e.key === 'Enter') {
      var first = results.querySelector('.compare-result');
      if (first) addItem(first.dataset.asin);
      e.preventDefault();
    }
  });
  document.addEventListener('click', function (e) {
    if (!e.target.closest('.compare-search-wrap')) closeResults();
  });
  results.addEventListener('mousedown', function (e) {
    var button = e.target.closest('[data-asin]');
    if (button) addItem(button.dataset.asin);
  });

  function renderCards(items) {
    cardsEl.className = 'compare-cards compare-cards--count-' + items.length;
    if (!items.length) {
      cardsEl.innerHTML = '';
      return;
    }
    cardsEl.innerHTML = items.map(function (item) {
      var image = item.image || '/assets/images/products/placeholder.svg';
      return '<div class="compare-card">' +
        '<button type="button" class="compare-card-remove" data-remove="' + item.asin + '" aria-label="Remove ' + escapeHtml(item.title) + '">&times;</button>' +
        '<div class="compare-card-image"><img src="' + image + '" alt="" loading="lazy" onerror="this.src=\'/assets/images/products/placeholder.svg\'"></div>' +
        '<a href="' + item.page_url + '" class="compare-card-title">' + escapeHtml(item.title) + '</a>' +
        '<a href="' + item.amazon_url + '" class="btn-buy-now" target="_blank" rel="noopener noreferrer">Amazon</a>' +
        '</div>';
    }).join('');
    cardsEl.querySelectorAll('[data-remove]').forEach(function (button) {
      button.addEventListener('click', function () { removeItem(button.dataset.remove); });
    });
  }

  function renderTable(items) {
    if (items.length < 2) {
      tableEl.innerHTML = '<p class="empty-state">' +
        (items.length === 0 ? 'Search above and add up to three components to compare.' : 'Add at least one more component to see a side-by-side comparison.') +
        '</p>';
      return;
    }

    var specKeys = {};
    items.forEach(function (item) {
      Object.keys(item.specs || {}).forEach(function (key) { specKeys[key] = true; });
    });

    var rows = [
      ['Category'].concat(items.map(function (item) { return item.category || '—'; })),
      ['Price'].concat(items.map(function (item) { return item.price || '—'; })),
      ['Stock'].concat(items.map(function (item) { return item.in_stock ? 'In stock' : 'Out of stock'; }))
    ];
    var specRowKeys = Object.keys(specKeys).sort();
    specRowKeys.forEach(function (key) {
      rows.push([key].concat(items.map(function (item) { return (item.specs || {})[key] || '—'; })));
    });

    var html = '<div class="compare-table-scroll"><table class="compare-table"><tbody>';
    rows.forEach(function (row) {
      var values = row.slice(1);
      var diff = Array.from(new Set(values.map(String))).length > 1;
      html += '<tr' + (diff ? ' class="diff-row"' : '') + '><th>' + escapeHtml(row[0]) + '</th>';
      values.forEach(function (value) { html += '<td>' + escapeHtml(value) + '</td>'; });
      html += '</tr>';
    });
    html += '</tbody></table></div>';

    if (!specRowKeys.length) {
      html += '<p class="compare-table-note">No verified spec data recorded for these parts yet.</p>';
    }

    tableEl.innerHTML = html;
  }

  function render() {
    updateUrl();
    var items = asins.map(itemFor).filter(Boolean);
    search.disabled = items.length >= MAX_ITEMS;
    search.placeholder = items.length >= MAX_ITEMS ? 'Remove an item to add another' : 'Search by name…';
    renderCards(items);
    renderTable(items);
  }

  render();
})();
</script>
