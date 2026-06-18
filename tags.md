---
layout: page
title: Tags
permalink: /tags/
---

<p class="shop-intro">Browse posts and projects by topic.</p>

<div id="tag-app"></div>

<script type="application/json" id="content-json">[
{% assign first = true %}
{% for post in site.posts %}{% unless first %},{% endunless %}{% assign first = false %}{"title":{{ post.title | jsonify }},"url":{{ post.url | relative_url | jsonify }},"tags":{{ post.tags | default: empty | jsonify }},"date":{{ post.date | date: "%Y-%m-%d" | jsonify }}}{% endfor %}
{% for project in site.projects %}{% unless first %},{% endunless %}{% assign first = false %}{"title":{{ project.title | jsonify }},"url":{{ project.url | relative_url | jsonify }},"tags":{{ project.tags | default: empty | jsonify }},"date":{{ project.date | date: "%Y-%m-%d" | jsonify }}}{% endfor %}
]</script>
<script>
(function () {
  var entries = JSON.parse(document.getElementById('content-json').textContent);
  var selected = new URLSearchParams(window.location.search).get('tag') || '';
  var tags = {};
  entries.forEach(function (entry) {
    (entry.tags || []).forEach(function (tag) { tags[tag] = (tags[tag] || 0) + 1; });
  });
  var html = '<div class="tag-chips">';
  Object.keys(tags).sort().forEach(function (tag) {
    var slug = tag.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '');
    html += '<a href="/tags/?tag=' + slug + '">' + tag + ' (' + tags[tag] + ')</a>';
  });
  html += '</div><div class="tag-results">';
  entries.filter(function (entry) {
    if (!selected) return true;
    return (entry.tags || []).some(function (tag) {
      return tag.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '') === selected;
    });
  }).forEach(function (entry) {
    html += '<a class="tag-result" href="' + entry.url + '"><strong>' + entry.title + '</strong><span>' + entry.date + '</span></a>';
  });
  html += '</div>';
  document.getElementById('tag-app').innerHTML = html;
})();
</script>
