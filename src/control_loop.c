#include "control_loop.h"
#include "pid.h"
#include "resolver_iface.h"

/* One PID instance per lane; DAL-A channels do not share mutable state. */
static pid_state_t s_pid[2];

void control_loop_init(control_state_t *state, lane_id_t lane)
{
    state->lane = lane;
    state->commanded_position_mm = 0.0f;
    state->measured_position_mm = 0.0f;
    state->measured_current_a = 0.0f;
    state->output_duty = 0.0f;
    state->channel_healthy = false;

    pid_init(&s_pid[lane], /* kp */ 4.2f, /* ki */ 18.0f, /* kd */ 0.06f,
             /* out_min */ -1.0f, /* out_max */ 1.0f,
             (float)CONTROL_LOOP_PERIOD_US / 1000000.0f);

    resolver_init(lane);
}

void control_loop_step(control_state_t *state)
{
    resolver_sample_t sample;
    const bool resolver_ok = resolver_read(state->lane, &sample);

    state->measured_position_mm = sample.position_mm;
    state->measured_current_a = sample.motor_current_a;
    state->channel_healthy = resolver_ok;

    if (!resolver_ok) {
        state->output_duty = 0.0f;
        pid_reset_integrator(&s_pid[state->lane]);
        return;
    }

    state->output_duty = pid_step(&s_pid[state->lane],
                                   state->commanded_position_mm,
                                   state->measured_position_mm);
}
