---
layout: page
title: Compare
permalink: /compare/
---

<p class="shop-intro">Compare up to three catalogue items by verified specs, stock state, and current Amazon link.</p>

<div id="compare-app" class="compare-app">
  <div class="compare-picker">
    <label for="compare-select">Add product</label>
    <select id="compare-select"></select>
    <button id="compare-add" type="button">Add</button>
  </div>
  <div id="compare-table"></div>
</div>

<script type="application/json" id="catalog-json">{{ site.data.catalog | jsonify }}</script>
<script>
(function () {
  var catalog = JSON.parse(document.getElementById('catalog-json').textContent);
  var selected = new URLSearchParams(window.location.search).get('asins');
  var asins = selected ? selected.split(',').filter(Boolean).slice(0, 3) : [];
  var select = document.getElementById('compare-select');
  var table = document.getElementById('compare-table');

  catalog.forEach(function (item) {
    var option = document.createElement('option');
    option.value = item.asin;
    option.textContent = item.title;
    select.appendChild(option);
  });

  function itemFor(asin) {
    return catalog.find(function (item) { return item.asin === asin; });
  }

  function updateUrl() {
    var url = new URL(window.location.href);
    if (asins.length) url.searchParams.set('asins', asins.join(','));
    else url.searchParams.delete('asins');
    history.replaceState(null, '', url);
  }

  function render() {
    updateUrl();
    var items = asins.map(itemFor).filter(Boolean);
    if (!items.length) {
      table.innerHTML = '<p class="empty-state">Select products to compare.</p>';
      return;
    }
    var specKeys = {};
    items.forEach(function (item) {
      Object.keys(item.specs || {}).forEach(function (key) { specKeys[key] = true; });
    });
    var rows = [
      ['Price'].concat(items.map(function (item) { return item.price || ''; })),
      ['Stock'].concat(items.map(function (item) { return item.in_stock ? 'In stock (' + item.stock_qty + ')' : 'Out of stock'; })),
      ['Category'].concat(items.map(function (item) { return item.category || ''; }))
    ];
    Object.keys(specKeys).sort().forEach(function (key) {
      rows.push([key].concat(items.map(function (item) { return (item.specs || {})[key] || ''; })));
    });
    var html = '<table class="compare-table"><thead><tr><th>Spec</th>';
    items.forEach(function (item) {
      html += '<th><a href="' + item.page_url + '">' + item.title + '</a><button type="button" data-remove="' + item.asin + '">Remove</button></th>';
    });
    html += '</tr></thead><tbody>';
    rows.forEach(function (row) {
      var values = row.slice(1);
      var diff = Array.from(new Set(values.map(String))).length > 1;
      html += '<tr' + (diff ? ' class="diff-row"' : '') + '><th>' + row[0] + '</th>';
      values.forEach(function (value) { html += '<td>' + value + '</td>'; });
      html += '</tr>';
    });
    html += '<tr><th>Buy</th>' + items.map(function (item) {
      return '<td><a class="btn-buy-now" href="' + item.amazon_url + '" target="_blank" rel="noopener noreferrer">Amazon</a></td>';
    }).join('') + '</tr></tbody></table>';
    table.innerHTML = html;
    table.querySelectorAll('[data-remove]').forEach(function (button) {
      button.addEventListener('click', function () {
        asins = asins.filter(function (asin) { return asin !== button.dataset.remove; });
        render();
      });
    });
  }

  document.getElementById('compare-add').addEventListener('click', function () {
    if (asins.indexOf(select.value) === -1 && asins.length < 3) asins.push(select.value);
    render();
  });
  render();
})();
</script>
