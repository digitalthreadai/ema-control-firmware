# Software Development Plan (SDP) -- ema-control-firmware

**Status:** stub / working draft. This is the SDP skeleton required by
DO-178C for a certification project; sections below are scoped for this
repository's two components and will be expanded as the program matures
past prototype flight test.

## 1. Purpose and scope

Covers embedded firmware for the EMA (Electromechanical Actuator) primary
flight-control surface actuator: the DAL-A position control channel and the
partitioned Monitor/BIT function that supervises it. Excludes the flight
control computer, the motor drive power stage hardware, and ground support
equipment.

## 2. Software life cycle

| Phase | Control channel (DAL-A) | Monitor / BIT (partitioned, lower DAL) |
|---|---|---|
| Requirements | Captured in Thread as `requirement` items under `RQM-00005` | Same requirement set, distinguished by the module they trace to |
| Design | Reviewed against `docs/REQUIREMENTS_TRACE_MATRIX.md` | Reviewed for **independence** from the control channel -- no write path to actuator authority |
| Implementation | MISRA-C-oriented subset, no dynamic allocation, no recursion | Same coding standard; isolated translation units (`bit_monitor.c`) |
| Verification | Requirements-based test + structural coverage (target: MC/DC) | Requirements-based test; structural coverage target relaxed per partitioning argument |
| Configuration management | This repository, tagged releases (see `CHANGELOG.md`) | Same repository -- partitioning is architectural/logical, not physical repos |

## 3. DAL assignment rationale

The control channel (`control_loop.c`, `pid.c`, `resolver_iface.c`) is
assigned **DAL A**: an unannunciated loss of, or erroneous, actuator
position command can contribute to a catastrophic failure condition.

The Monitor/BIT function (`bit_monitor.c`) is assigned a **lower DAL**
under a partitioning argument: its only path to actuator behavior is
`bit_monitor_is_healthy()`, a boolean the control loop reads to decide
whether to zero its own output. It never writes a position, current, or
duty command. A software error in `bit_monitor.c` can therefore cause an
unnecessary fault trip (a nuisance / availability concern) but cannot
cause an incorrect actuator command to be issued -- see `REQ-00039` and the
trace matrix.

## 4. Verification methods used in this repo

- **Requirements-based test** -- one or more test cases per requirement in
  the trace matrix; CI runs the suite and publishes a JUnit report per
  build (synced into Thread's ALM integration as `test_summary`).
- **Review** -- design and code review against the requirement text and
  this SDP's partitioning argument.
- **Analysis** -- for parameters not practically testable on the bench
  (e.g. worst-case loop timing), documented analysis stands in for test.

## 5. Open items

- Structural coverage tooling (MC/DC) for the DAL-A channel is not yet
  wired into CI -- tracked as a follow-up, not blocking for the v1.2.0
  bench-test release.
- Tool qualification data (compiler, static analyzer) is out of scope for
  this stub and will be added when the certification basis is finalized.
