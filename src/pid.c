#include "pid.h"
#include <math.h>

void pid_init(pid_state_t *pid, float kp, float ki, float kd,
              float out_min, float out_max, float dt_s)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->dt_s = dt_s;
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->has_prev_error = false;
}

void pid_reset_integrator(pid_state_t *pid)
{
    pid->integrator = 0.0f;
    pid->has_prev_error = false;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* A stale integrator sized for the OLD commanded direction fights the new
 * command for several cycles on a fast reversal (surface snapping from one
 * stop to the other). Bleed it hard when the error's sign flips and the
 * error that just ended was large enough that its integral contribution
 * actually mattered. */
#define PID_REVERSAL_ERROR_THRESHOLD   1.0f   /* mm */
#define PID_REVERSAL_BLEED_FACTOR      0.25f

float pid_step(pid_state_t *pid, float setpoint, float measured)
{
    const float error = setpoint - measured;

    if (pid->has_prev_error &&
        ((error > 0.0f) != (pid->prev_error > 0.0f)) &&
        (fabsf(pid->prev_error) > PID_REVERSAL_ERROR_THRESHOLD)) {
        pid->integrator *= PID_REVERSAL_BLEED_FACTOR;
    }

    pid->integrator += error * pid->dt_s;

    /* Clamp the integrator's OWN contribution to the output envelope, not
     * just the final sum -- prevents the term from silently growing far
     * past what output saturation will ever use (classic reset windup).
     * The clamp is applied to `integrator` itself (not just i_term) so the
     * stored state stays consistent with what the term will produce next
     * cycle too. */
    if (pid->ki != 0.0f) {
        const float i_min = pid->out_min / pid->ki;
        const float i_max = pid->out_max / pid->ki;
        pid->integrator = clampf(pid->integrator, i_min, i_max);
    }

    float derivative = 0.0f;
    if (pid->has_prev_error) {
        derivative = (error - pid->prev_error) / pid->dt_s;
    }
    pid->prev_error = error;
    pid->has_prev_error = true;

    const float output = (pid->kp * error) +
                          (pid->ki * pid->integrator) +
                          (pid->kd * derivative);

    return clampf(output, pid->out_min, pid->out_max);
}
