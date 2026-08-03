# Requirements Trace Matrix -- ema-control-firmware

Source of truth for requirement text is Thread (`RQM-00005` -- "EMA -- Flight
Control Actuator Requirements"). This table is the firmware-side trace from
those requirement IDs to the module(s) that implement them and how they are
verified. Requirement IDs are the Thread `item_number`; `Rec ID` is the
legacy program requirement reference carried in Thread as `rec_id`, kept
here for cross-referencing older program documents.

| Requirement ID | Rec ID | Title | DAL | Firmware Module(s) | Verification Method |
|---|---|---|---|---|---|
| REQ-00039 | ACT-REQ-004 | DO-178C DAL-A Firmware Partitioning | A (channel) / partitioned (monitor) | `control_loop.c` (gates on monitor health), `bit_monitor.c` (health-only output) | Review of the partitioning argument in `docs/SW_DEVELOPMENT_PLAN.md` §3; requirements-based test of the fault-gate path in `control_loop_step()` |
| REQ-00043 | -- | Resolver Position Feedback Accuracy | A | `resolver_iface.c` (`resolver_read`, `atan2_approx`) | Requirements-based test against a resolver simulator across the operating envelope; bench calibration record |
| REQ-00055 | -- | Dual-Channel Position Monitoring and Cross-Lane Comparison | partitioned (monitor) | `bit_monitor.c` (`bit_monitor_update` cross-lane block, `BIT_FAULT_CROSS_LANE_POS` / `BIT_FAULT_CROSS_LANE_CURRENT`) | Requirements-based test: inject a synthetic lane divergence on the bench and confirm the fault latches within the debounce window and the control channel commands zero output |

## Traceability notes

- **REQ-00039** is implemented as a *runtime* control, not just a build-time
  separation: `control_loop_step()` cannot proceed to `pid_step()` while
  `bit_monitor_is_healthy()` is false, and the reverse call direction does
  not exist anywhere in this repository (`bit_monitor.c` has no include of
  `pid.h` or `control_loop.h`, and no function in it writes to a
  `control_state_t`).
- **REQ-00043**'s accuracy budget flows into two constants in
  `resolver_iface.c`: `RESOLVER_STROKE_MM` (mechanical scale factor) and the
  `AMPLITUDE_MIN_FRACTION` / `AMPLITUDE_MAX_FRACTION` band (signal-chain
  health gate below which the accuracy budget cannot be claimed).
- **REQ-00055**'s thresholds are `BIT_XLANE_POSITION_TOLERANCE_MM` and
  `BIT_XLANE_CURRENT_TOLERANCE_A` in `bit_monitor.h` -- change them there,
  not inline, so this matrix and the header cannot silently drift apart.
- This matrix currently covers the requirements exercised by the v1.2.0
  bench-test build. `RQM-00005` carries additional requirements (mechanical,
  electrical, and other firmware items) not yet reflected here; extending
  this table is tracked alongside the CI structural-coverage follow-up in
  the SDP.
