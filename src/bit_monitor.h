#ifndef BIT_MONITOR_H
#define BIT_MONITOR_H

#include <stdbool.h>
#include "lanes.h"
#include "resolver_iface.h"

/* Built-In Test / health monitoring. Runs at a LOWER design assurance level
 * than the control channel (REQ-00039 -- DO-178C DAL-A firmware
 * partitioning): it may only ever REPORT health, never command the motor
 * directly. The control loop reads bit_monitor_is_healthy() and gates its
 * own output; a bug here can make the actuator go safe, never make it do
 * something the control channel didn't already command. */

typedef enum {
    BIT_FAULT_NONE               = 0,
    BIT_FAULT_EXCITATION_LOST    = 1u << 0,
    BIT_FAULT_AMPLITUDE_OOR      = 1u << 1,
    BIT_FAULT_OVERCURRENT        = 1u << 2,
    BIT_FAULT_CROSS_LANE_POS     = 1u << 3,
    BIT_FAULT_CROSS_LANE_CURRENT = 1u << 4,
} bit_fault_flags_t;

#define BIT_OVERCURRENT_LIMIT_A   22.0f

/* Cross-lane comparison thresholds (REQ-00055 -- dual-channel position
 * monitoring and cross-lane comparison). */
#define BIT_XLANE_POSITION_TOLERANCE_MM   1.0f
#define BIT_XLANE_CURRENT_TOLERANCE_A     3.0f

/* Consecutive out-of-tolerance cycles required before a fault LATCHES.
 * One-cycle noise on a 2 kHz loop must not trip a flight control surface. */
#define BIT_LATCH_DEBOUNCE_CYCLES   20u

void bit_monitor_init(lane_id_t lane);

/* Called once per control cycle, after the resolver sample for `lane`.
 * `resolver_ok` is the raw resolver_read() health flag; `sample` is that
 * cycle's decoded resolver_sample_t. Each lane's monitor also cross-checks
 * against whatever the peer lane recorded this cycle (REQ-00055) -- both
 * lanes must have called this once for the cross-check to be current;
 * on the very first cycle after init the peer check is simply skipped. */
void bit_monitor_update(lane_id_t lane, bool resolver_ok, const resolver_sample_t *sample);

bool bit_monitor_is_healthy(lane_id_t lane);
bit_fault_flags_t bit_monitor_latched_faults(lane_id_t lane);

/* Ground-maintenance reset -- clears a latched fault. Refuses (returns
 * false) while the underlying debounced condition is still active this
 * cycle, so a latch can never be cleared out from under a live fault. */
bool bit_monitor_clear_latch(lane_id_t lane);

#endif /* BIT_MONITOR_H */
