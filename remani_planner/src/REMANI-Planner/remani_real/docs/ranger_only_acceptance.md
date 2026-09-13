# Ranger-only Low-speed Acceptance

Software rostest must pass before any CAN/hardware session.

## Software gate

- `rostest remani_real ranger_only_execution.test`
- Ordinary `/cmd_vel` does not drive the remapped hardware subscriber
- Commands on `/remani/hardware/ranger/cmd_vel` stay ≤0.05 m/s
- Watchdog zeros within 0.25 s after Executor ticks stop
- Abort yields READY and zero commands
- Rogue hardware publisher yields ERROR and a commanded zero

## Physical checklist (authorized only)

1. Clear static-empty field; physical E-stop operator present
2. Lifted-wheel / stand test first when mechanically safe
3. CAN interface verified on Ubuntu 18.04 robot host
4. Remap `/cmd_vel` → `/remani/hardware/ranger/cmd_vel` confirmed
5. Path ≤0.20 m, command ≤0.05 m/s
6. Measure Pause/Abort/watchdog stop latency; attach logs
7. Any stop-gate failure blocks CR10 integration
