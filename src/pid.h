#ifndef PID_H
#define PID_H

#include <stdbool.h>

/* Position-loop PID for one EMA control lane. One instance per lane -- see
 * control_loop.c. */

typedef struct {
    float kp;
    float ki;
    float kd;

    float out_min;
    float out_max;
    float dt_s;

    float integrator;
    float prev_error;
    bool  has_prev_error;
} pid_state_t;

void  pid_init(pid_state_t *pid, float kp, float ki, float kd,
               float out_min, float out_max, float dt_s);
float pid_step(pid_state_t *pid, float setpoint, float measured);
void  pid_reset_integrator(pid_state_t *pid);

#endif /* PID_H */
