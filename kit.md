---
layout: page
title: Your Kit
permalink: /kit/
body_class: page-kit
---

<p class="shop-intro">Parts you've picked from around the site, in one place, with a running total. Amazon doesn't offer a reliable way to add several items to your basket in one click without an active Associates account — click each item's Amazon link below to add it to your real basket.</p>

<div id="kit-empty" class="kit-empty" hidden>
  <p>Your kit is empty. <a href="/shop/">Browse the shop</a> or start from the <a href="/bundles/beginner-breadboarding-kit/">Beginner Breadboarding Kit</a> bundle.</p>
</div>

<div id="kit-items" class="kit-items"></div>

<aside id="kit-summary" class="kit-summary" hidden>
  <div class="kit-summary-row">
    <span>Approximate total</span>
    <strong id="kit-total">£0.00</strong>
  </div>
  <p class="product-amazon-note">Each item above links straight to its Amazon listing — add them one at a time, then check out on Amazon as normal.</p>
  <button type="button" id="kit-clear-btn" class="kit-clear-btn">Clear kit</button>
</aside>

<style>
  .kit-items {
    display: flex;
    flex-direction: column;
    gap: 0.75rem;
    margin: 1.5rem 0;
  }

  .kit-item {
    display: grid;
    grid-template-columns: 64px 1fr auto auto auto auto;
    align-items: center;
    gap: 1rem;
    background-color: #161b22;
    border: 1px solid #30363d;
    border-radius: 6px;
    padding: 0.85rem 1rem;
  }

  .kit-item img {
    width: 64px;
    height: 64px;
    object-fit: contain;
    background: #0d1117;
    border-radius: 4px;
  }

  .kit-item-info {
    display: flex;
    flex-direction: column;
    gap: 0.2rem;
    min-width: 0;
  }

  .kit-item-title {
    color: #e2eaf4;
    text-decoration: none;
    font-weight: 600;

    &:hover {
      color: #00e5ff;
      text-decoration: underline;
    }
  }

  .kit-item-unit-price {
    color: #8b949e;
    font-size: 0.8rem;
  }

  .kit-item-qty {
    display: flex;
    align-items: center;
    gap: 0.5rem;
  }

  .kit-qty-btn {
    width: 26px;
    height: 26px;
    background: #0d1117;
    border: 1px solid #30363d;
    border-radius: 4px;
    color: #e2eaf4;
    cursor: pointer;
    font-size: 1rem;
    line-height: 1;

    &:hover {
      border-color: #00e5ff;
      color: #00e5ff;
    }
  }

  .kit-qty-value {
    min-width: 1.5rem;
    text-align: center;
    font-family: "Courier New", monospace;
  }

  .kit-item-total {
    font-family: "Courier New", monospace;
    font-weight: bold;
    white-space: nowrap;
  }

  .kit-item-remove {
    background: transparent;
    border: none;
    color: #8b949e;
    cursor: pointer;
    font-size: 1.3rem;
    line-height: 1;
    padding: 0 0.25rem;

    &:hover {
      color: #ff7b72;
    }
  }

  .kit-empty {
    color: #8b949e;
    margin: 2rem 0;
  }

  .kit-summary {
    max-width: 420px;
    background-color: #161b22;
    border: 1px solid #30363d;
    border-radius: 6px;
    padding: 1.25rem;
    margin-top: 1.5rem;
  }

  .kit-summary-row {
    display: flex;
    justify-content: space-between;
    align-items: baseline;
    margin-bottom: 1rem;
    font-family: "Courier New", monospace;
  }

  .kit-summary-row strong {
    font-size: 1.3rem;
    color: #e2eaf4;
  }

  .kit-clear-btn {
    display: block;
    width: 100%;
    background: transparent;
    border: 1px solid #30363d;
    border-radius: 4px;
    color: #8b949e;
    cursor: pointer;
    padding: 0.4rem;
    font-family: "Rajdhani", sans-serif;
    font-size: 0.85rem;

    &:hover {
      border-color: #ff7b72;
      color: #ff7b72;
    }
  }

  @media (max-width: 560px) {
    .kit-item {
      grid-template-columns: 48px 1fr auto;
      grid-template-areas:
        "image info info"
        "image qty qty"
        "total total amazon"
        "remove remove remove";
    }
    .kit-item img { grid-area: image; width: 48px; height: 48px; }
    .kit-item-info { grid-area: info; }
    .kit-item-qty { grid-area: qty; }
    .kit-item-total { grid-area: total; }
    .kit-item .btn-buy-now { grid-area: amazon; justify-self: end; }
    .kit-item-remove { grid-area: remove; justify-self: start; }
  }
</style>

<script>
(function () {
  function escapeHtml(value) {
    return String(value).replace(/[&<>"']/g, function (c) {
      return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c];
    });
  }

  function formatGBP(amount) {
    return '£' + (Math.round(amount * 100) / 100).toFixed(2);
  }

  function render() {
    if (!window.DSTKit) return;
    var cart = window.DSTKit.get();
    var itemsEl = document.getElementById('kit-items');
    var emptyEl = document.getElementById('kit-empty');
    var summaryEl = document.getElementById('kit-summary');

    if (!cart.length) {
      itemsEl.innerHTML = '';
      emptyEl.hidden = false;
      summaryEl.hidden = true;
      return;
    }
    emptyEl.hidden = true;
    summaryEl.hidden = false;

    itemsEl.innerHTML = cart.map(function (item) {
      var image = item.image || '/assets/images/products/placeholder.svg';
      var lineTotal = (Number(item.priceAmount) || 0) * item.qty;
      return '<div class="kit-item" data-asin="' + escapeHtml(item.asin) + '">' +
        '<img src="' + image + '" alt="" loading="lazy" onerror="this.src=\'/assets/images/products/placeholder.svg\'">' +
        '<div class="kit-item-info">' +
          '<a href="' + item.pageUrl + '" class="kit-item-title">' + escapeHtml(item.title) + '</a>' +
          '<span class="kit-item-unit-price">' + escapeHtml(item.price || '') + ' each</span>' +
        '</div>' +
        '<div class="kit-item-qty">' +
          '<button type="button" class="kit-qty-btn" data-qty-action="decrease" aria-label="Decrease quantity">&minus;</button>' +
          '<span class="kit-qty-value">' + item.qty + '</span>' +
          '<button type="button" class="kit-qty-btn" data-qty-action="increase" aria-label="Increase quantity">+</button>' +
        '</div>' +
        '<div class="kit-item-total">' + formatGBP(lineTotal) + '</div>' +
        (item.amazonUrl ? '<a href="' + item.amazonUrl + '" class="btn-buy-now" target="_blank" rel="noopener noreferrer">Amazon</a>' : '<span></span>') +
        '<button type="button" class="kit-item-remove" aria-label="Remove ' + escapeHtml(item.title) + '">&times;</button>' +
      '</div>';
    }).join('');

    document.getElementById('kit-total').textContent = formatGBP(window.DSTKit.total());

    itemsEl.querySelectorAll('.kit-qty-btn').forEach(function (btn) {
      btn.addEventListener('click', function () {
        var row = btn.closest('.kit-item');
        var asin = row.dataset.asin;
        var current = window.DSTKit.get().filter(function (i) { return i.asin === asin; })[0];
        if (!current) return;
        var next = btn.dataset.qtyAction === 'increase' ? current.qty + 1 : current.qty - 1;
        window.DSTKit.setQty(asin, next);
        render();
      });
    });

    itemsEl.querySelectorAll('.kit-item-remove').forEach(function (btn) {
      btn.addEventListener('click', function () {
        var row = btn.closest('.kit-item');
        window.DSTKit.remove(row.dataset.asin);
        render();
      });
    });
  }

  var clearBtn = document.getElementById('kit-clear-btn');
  if (clearBtn) {
    clearBtn.addEventListener('click', function () {
      window.DSTKit.clear();
      render();
    });
  }

  render();
})();
</script>
