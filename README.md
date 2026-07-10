# ema-control-firmware

Embedded flight-control firmware for the EMA (Electromechanical Actuator)
primary flight-control surface actuator.

## DAL context

This repository holds two firmware components running on the same MCU, at
two different DO-178C Design Assurance Levels:

| Component | Files | DAL | Role |
|---|---|---|---|
| Control channel | `control_loop.c`, `pid.c`, `resolver_iface.c` | **A** | Closes the position loop and drives the motor. Flight-critical. |
| Monitor / BIT | `bit_monitor.c` | B (partitioned) | Watches the control channel's inputs/outputs and can only force a *safe* state — it never issues a command of its own. |

The partitioning argument (`docs/REQUIREMENTS_TRACE_MATRIX.md`,
`RQM-00005` / `REQ-00039`) is: a defect in the lower-DAL monitor code can, at
worst, cause an unnecessary fault trip. It cannot cause the DAL-A control
channel to do something it wasn't already commanded to do, because the
monitor has no path to actuator authority except "zero the output."

## Layout

```
src/
  control_loop.c / .h    fixed-step scheduler (2 kHz), ties pid + resolver + BIT together
  pid.c / .h              anti-windup position PID, one instance per lane
  resolver_iface.c / .h   resolver excitation/decode -> position + motor current
  bit_monitor.c / .h      fault debounce/latch, cross-lane comparison
  hw/adc.h, hw/excitation.h   board-support contract (BSP implementation is out of tree)
docs/
  SW_DEVELOPMENT_PLAN.md          DO-178C SDP stub
  REQUIREMENTS_TRACE_MATRIX.md    firmware requirement -> module -> verification trace
```

## Build

This repo carries application source only; the board support package (ADC
driver, excitation timer driver, linker script) is vendored separately per
target. There is no build system committed here — `hw/adc.h` and
`hw/excitation.h` are the contract the application code links against.

## Status

Current release: **v1.2.0**. See `CHANGELOG.md`.
