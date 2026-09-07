# Agent Guide

This repository uses a **shared lightweight project memory** for Cursor, Codex, Codex Desktop, Claude Code, and other coding agents.

Canonical files (repo root, next to `README.md`):

| File | Answers |
|------|---------|
| `AGENTS.md` | How agents should work |
| `PROGRESS.md` | Where are we now? |
| `MEMORY.md` | What must we remember later? |
| `PROCESS.md` | Why was it designed this way? |

Do **not** create per-agent duplicate memory files, databases, embeddings, chat dumps, or MCP memory servers for this purpose. Do not auto-log every action.

## Development norms

- Prefer small, task-scoped changes; match existing style and naming.
- Code and verified runtime/test evidence beat stale docs when they conflict.
- Do not modify unrelated packages, generated maps, or secrets.
- Never commit `TopAY/src/simulator/random_map_generator/env/map.pcd`.
- Prefer docker container `topay` for TopAY builds/tests when the host tree is root-owned.
- For C++/YAML/XML edits in TopAY work, mark changed blocks with language-appropriate
  `################################` comment fences as used in this repo.

## Modification principles

- Before changing a previously verified implementation or design choice, check
  `MEMORY.md`, `PROCESS.md`, and existing validation evidence first.
- Do **not** replace a verified solution merely because another approach looks
  cleaner or more conventional.
- Do not invent facts to fill templates; omit unknowns.

## Testing requirements

- Prefer existing gates (`test_topay_*`, audit scripts, headless smoke) over ad-hoc claims.
- Use low `catkin_make -j`, timeouts, and clean up stuck `test_topay_*` processes.
- GridMap unit gates: `agent/mode=planner` + `loadMap`, with `roscore` available.
- Do not claim PASS without command output or equivalent evidence.

## Documentation maintenance

- Update only the files that need it after a task (not all four every time).
- `PROGRESS.md`: state snapshot only — delete stale items; no full logs or git history dumps.
- `MEMORY.md`: durable verified knowledge only — future reuse / avoid repeated failure.
- `PROCESS.md`: important engineering history (attempts, root cause, final design, verification).
- Keep content concise; do not duplicate the same facts across files.

## Project Memory

Before substantial work:
- Read `PROGRESS.md`.
- Read relevant `MEMORY.md`.
- Read `PROCESS.md` when historical reasoning is needed.
- Trust current code, tests, and runtime evidence over outdated documents.

After substantial work:
- Update `PROGRESS.md` when state changes.
- Update `MEMORY.md` when durable knowledge is discovered.
- Update `PROCESS.md` when important engineering history should be preserved.

Before changing a previously verified implementation or design choice, check
`MEMORY.md`, `PROCESS.md`, and validation evidence first. Do not replace a
verified solution merely because a new approach appears more elegant.
