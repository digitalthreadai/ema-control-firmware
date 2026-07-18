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
    BIT_FAULT_NONE            = 0,
    BIT_FAULT_EXCITATION_LOST = 1u << 0,
    BIT_FAULT_AMPLITUDE_OOR   = 1u << 1,
    BIT_FAULT_OVERCURRENT     = 1u << 2,
} bit_fault_flags_t;

#define BIT_OVERCURRENT_LIMIT_A   22.0f

/* Consecutive out-of-tolerance cycles required before a fault LATCHES.
 * One-cycle noise on a 2 kHz loop must not trip a flight control surface. */
#define BIT_LATCH_DEBOUNCE_CYCLES   20u

void bit_monitor_init(lane_id_t lane);

/* Called once per control cycle, after the resolver sample for `lane`.
 * `resolver_ok` is the raw resolver_read() health flag; `sample` is that
 * cycle's decoded resolver_sample_t. */
void bit_monitor_update(lane_id_t lane, bool resolver_ok, const resolver_sample_t *sample);

bool bit_monitor_is_healthy(lane_id_t lane);
bit_fault_flags_t bit_monitor_latched_faults(lane_id_t lane);

#endif /* BIT_MONITOR_H */
