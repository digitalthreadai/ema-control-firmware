# Changelog

## v1.2.0 -- 2026-08-10

Bench-test release. First tag with the full DAL-A / partitioned-monitor
architecture in place and documented.

- Fixed-step (2 kHz) control loop with anti-windup position PID, one
  instance per lane.
- Resolver excitation/decode interface (`resolver_iface.c`) meeting the
  REQ-00043 accuracy budget.
- BIT monitor: excitation-loss, amplitude-out-of-range, and overcurrent
  fault detection with debounce-and-latch semantics, gating the control
  channel's output (REQ-00039 partitioning).
- Cross-lane position and current comparison in the BIT monitor
  (REQ-00055), with a ground-maintenance latch-clear path.
- PID integrator windup clamp, including fast bleed on commanded-direction
  reversal (surface snapping from one stop to the other).
- `docs/SW_DEVELOPMENT_PLAN.md` (DO-178C SDP stub) and
  `docs/REQUIREMENTS_TRACE_MATRIX.md` added.
