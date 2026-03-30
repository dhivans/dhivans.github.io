---
layout: page
title: Projects
permalink: /projects/
---

Build logs, experiments, and teardowns using DST-sourced gear.
Written in a direct engineering tone — what was built, why, what worked, what didn't.

<div class="project-list">
  {% assign sorted_projects = site.projects | sort: 'date' | reverse %}
  {% for project in sorted_projects %}
  <div class="project-card">
    <h3><a href="{{ project.url | relative_url }}">{{ project.title }}</a></h3>
    <div class="project-meta">{{ project.date | date: "%Y-%m-%d" }}{% if project.tags and project.tags.size > 0 %} &mdash; {{ project.tags | join: ", " }}{% endif %}</div>
    <p>{{ project.description }}</p>
    <a href="{{ project.url | relative_url }}" class="read-more">Read more &rarr;</a>
  </div>
  {% endfor %}
</div>
