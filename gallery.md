---
layout: page
title: Built with DST
permalink: /built-with-dst/
---

<p class="shop-intro">Projects and build logs that use DST catalogue parts.</p>

<div class="project-list">
{% for project in site.projects %}
  {% if project.tags contains "showcase" or project.products %}
  <a class="project-card" href="{{ project.url | relative_url }}">
    <h3>{{ project.title }}</h3>
    <div class="project-meta">{{ project.date | date: "%Y-%m-%d" }}</div>
    <p>{{ project.description }}</p>
    <span class="read-more">View build</span>
  </a>
  {% endif %}
{% endfor %}
</div>
