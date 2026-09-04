# Agent Guide

This repository uses a shared lightweight project memory for Cursor, Codex, Claude Code, and other coding agents.

Canonical files (repo root, next to `README.md`):

- `PROGRESS.md` — current project state
- `MEMORY.md` — durable project knowledge

Do not create per-agent duplicate memory files, databases, or MCP memory servers for this purpose.

## Project Memory

Before substantial work:
1. Read `PROGRESS.md`.
2. Read relevant parts of `MEMORY.md` when needed.
3. Inspect current code, configuration, Git state, tests, and runtime evidence before making conclusions.
4. If documentation conflicts with verified implementation or runtime results, trust current code and verified evidence.

After substantial work:
1. Update `PROGRESS.md` if project state changed.
2. Update `MEMORY.md` only when durable project knowledge was discovered.
3. Do not record speculation as fact.
4. Keep both files concise and correct stale information when found.

Before changing a previously verified implementation or design choice,
check MEMORY.md and existing validation evidence first.

Do not replace a verified solution merely because another implementation
appears cleaner or more conventional.
