#include "pid.h"

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

float pid_step(pid_state_t *pid, float setpoint, float measured)
{
    const float error = setpoint - measured;

    pid->integrator += error * pid->dt_s;

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
