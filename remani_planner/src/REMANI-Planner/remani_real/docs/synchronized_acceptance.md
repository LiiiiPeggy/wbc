# Synchronized REMANI Real Acceptance Gate

Authorized physical stages only. Each failed stage blocks the next.
Record candidate ID, config hash, requested T0, Action send/accept, both start
timestamps, skew, stop latencies, max commands, tracking errors, final errors,
physical E-stop operator, and pass/fail.

## Stages

1. read-only dry_run feedback/TF/RobotModel/preview
2. Ranger isolation + watchdog
3. Ranger-only <=0.05 m/s, <=0.20 m
4. CR10 Action validation/cancel/Stop
5. CR10-only <=0.05 rad/s, <=0.10 rad/joint
6. shared-T0 synchronized dry-run
7. synchronized short real trajectory, |start_skew|<=0.10 s
8. early/mid/late Pause/Resume and over-tolerance rejection
9. terminal base/joint/EE checks and injected faults
10. gradually longer paths in cleared static-empty field

## Software gates (laptop)

- `rostest remani_real real_launch_contract.test`
- `rostest remani_real synchronized_dry_run.test`
- `rostest remani_real pause_resume_abort.test`
- `rostest remani_real final_completion.test`
- Canonical Ranger HW topic: `/remani/hardware/ranger/cmd_vel`
- `dry_run=true` must keep write counters and HW publishers at zero

## Launch

```bash
# Laptop/CI dry-run (default)
./run_remani.sh mode:=real dry_run:=true

# Robot host with AGX drivers
./run_remani.sh mode:=real dry_run:=true \
  start_hardware_drivers:=true \
  hardware_drivers_launch:=$(rospack find remani_real)/launch/remani_real_hardware.launch \
  use_fake_feedback:=false \
  start_planner:=true
```
