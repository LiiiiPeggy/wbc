# Repository Guidelines

## Development Rules

- Treat `/home/gzz/Codes/remani/wbc` as the repository root for this checkout.
- Prefer the repository's existing ROS1/catkin, C++14, Python, launch, and config patterns over introducing new frameworks.
- Keep edits scoped to the requested task. Do not modify vendored/upstream code, hardware launch files, or robot-control paths unless the task explicitly requires it.
- Preserve the default simulation path unless a task is specifically about changing simulation behavior.
- For real-robot work, keep safety boundaries explicit: `dry_run=true` must not publish hardware motion commands or call mutating arm services.

## Testing and Evidence

- Inspect current code, configuration, tests, Git state, and available runtime evidence before making conclusions.
- Run the narrowest meaningful verification for the files changed. For ROS/catkin changes, prefer package-selective builds and focused `rostest`/gtest targets before broad workspace builds.
- If a command cannot be run in the current environment, record that limitation instead of treating the result as verified.
- Do not record speculation as fact. Trust current code, tests, and runtime evidence over outdated documents.

## Modification Principles

- Do not replace a verified implementation only because another approach looks cleaner or more conventional.
- Keep public interfaces stable unless the relevant design, tests, and downstream consumers are updated together.
- Stage or commit only when explicitly asked. If committing later, stage every new/changed file explicitly when new files are involved.

## Documentation Rules

- For REMANI work, use only `remani_planner/PROGRESS.md`, `remani_planner/MEMORY.md`, and `remani_planner/PROCESS.md` as project memory (plus this root `AGENTS.md` for repo-wide agent rules).
- Do not create Cursor-, Codex-, Claude-, or agent-specific memory files.
- Keep `remani_planner/PROGRESS.md` as a current state snapshot, not a chronological log.
- Keep `remani_planner/MEMORY.md` for durable, reusable knowledge only.
- Keep `remani_planner/PROCESS.md` for important engineering history and reasoning. Do not copy all process detail into `MEMORY.md`.
- Do not keep a second REMANI memory set at the repository root.

## Project Memory (REMANI)

For REMANI work, read:

1. `remani_planner/PROGRESS.md` — current project state and next actions
2. `remani_planner/MEMORY.md` — stable engineering contracts
3. `remani_planner/PROCESS.md` — historical context when needed

Current-state precedence: `PROGRESS.md` > `MEMORY.md` > `PROCESS.md`.

Before substantial REMANI work:
- Read `remani_planner/PROGRESS.md`.
- Read relevant `remani_planner/MEMORY.md`.
- Read `remani_planner/PROCESS.md` when historical reasoning is needed.
- Trust current code, tests, and runtime evidence over outdated documents.

After substantial REMANI work:
- Update `remani_planner/PROGRESS.md` when state changes.
- Update `remani_planner/MEMORY.md` when durable knowledge is discovered.
- Update `remani_planner/PROCESS.md` when important engineering history should be preserved.

修改已验证方案前，先检查 `remani_planner/MEMORY.md`、`remani_planner/PROCESS.md` 和验证结果。不要仅因为新方案更“优雅”而替换已有有效方案。

Use only `remani_planner/{PROGRESS,MEMORY,PROCESS}.md` for REMANI project memory. Do not create Agent-specific copies.
