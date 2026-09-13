# CR10-only Low-speed Acceptance

Software rostest must pass before any CR10 hardware session.

## Software gate

- `rostest remani_real cr10_only_execution.test`
- Malformed goals are REJECTED with zero ServoJ-equivalent samples
- Pre-start hold: no non-hold ServoJ before `start_lead_time`
- Cancel returns PREEMPTED/CANCELED (never SUCCEEDED) with cancel path latency &lt; 0.20 s
- Settled q/qd yields SUCCEEDED; settle tolerance failure yields ABORTED
- Launch requires `enable_staged_test=true`; no Ranger hardware publisher

## Physical checklist (authorized only)

1. Physical E-stop operator present; Ranger stationary and powered down / isolated
2. CR10 `/remani/cr10_status`: connected, enabled, `error_status==0`
3. `remani_prestart_hold_mode=true`; per-joint move ≤0.10 rad; arm speed ≤0.05 rad/s
4. Measure Action accept latency, early/mid/late cancel, Stop, actual-velocity settle
5. Attach Action result evidence; any cancel/Stop/completion failure blocks synchronized Phase 5
