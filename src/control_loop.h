#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <stdint.h>
#include <stdbool.h>
#include "lanes.h"

/* Fixed-step control loop scheduler for the EMA (Electromechanical
 * Actuator) flight-control channel. Runs at CONTROL_LOOP_RATE_HZ on a
 * hardware timer ISR; all work below is bounded-time and allocation-free
 * (DAL-A constraint: no heap use in the control channel). */

#define CONTROL_LOOP_RATE_HZ     2000U
#define CONTROL_LOOP_PERIOD_US   (1000000U / CONTROL_LOOP_RATE_HZ)

typedef struct {
    lane_id_t lane;
    float     commanded_position_mm;   /* from the flight control computer */
    float     measured_position_mm;    /* from resolver_iface */
    float     measured_current_a;
    float     output_duty;             /* -1.0 .. 1.0 motor drive command */
    bool      channel_healthy;
} control_state_t;

/* Called once at boot after resolver init. */
void control_loop_init(control_state_t *state, lane_id_t lane);

/* Called every CONTROL_LOOP_PERIOD_US from the timer ISR. Reads the
 * resolver, runs the PID, and writes the motor drive command. Must
 * complete well inside one period -- no blocking calls. */
void control_loop_step(control_state_t *state);

#endif /* CONTROL_LOOP_H */
