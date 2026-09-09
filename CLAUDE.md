# CLAUDE.md

Guidance for Claude Code sessions working in this repo.

## Planning convention

- **`NEXT_STEPS.md`** (repo root) is the fast entry point for "where did I
  leave off." Keep it current: regenerate it at the start or end of a
  session whenever something material has changed (a doc edited, a feature
  shipped, a decision made). It's meant to be rebuilt from the underlying
  docs and git history each time, not hand-edited line by line.
- **`Planning/`** holds the larger topic-specific planning docs (strategy,
  execution plans, technical roadmaps) that `NEXT_STEPS.md` points into for
  depth. These are working documents tracking in-progress work, not
  finished deliverables.
- Planning docs can go stale or drift apart — if two of them disagree on
  current status, flag the contradiction rather than silently picking one
  as authoritative.
- `Planning/*.md` files are candidates for cleanup once their tracked work
  is genuinely done, but only the repo owner removes them — don't delete
  or archive a planning doc on your own judgement.
- Reference/style docs (e.g. `dst-content-style-guide.md`) are different
  from planning docs — they document standing conventions rather than
  tracking in-progress work, so they stay at the repo root, not in
  `Planning/`.
