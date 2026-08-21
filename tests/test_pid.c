/* Minimal smoke test for pid_step — no framework, no BSP dependency.
 * Verifies: output stays clamped to [out_min, out_max], and a PID driven
 * toward a fixed setpoint converges the error down over a run of steps. */
#include <stdio.h>
#include <math.h>
#include "../src/pid.h"

static int failures = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } \
    else { printf("ok:   %s\n", msg); } \
} while (0)

int main(void)
{
    pid_state_t pid;
    pid_init(&pid, /*kp*/2.0f, /*ki*/0.5f, /*kd*/0.05f,
              /*out_min*/-10.0f, /*out_max*/10.0f, /*dt_s*/0.001f);

    float measured = 0.0f;
    const float setpoint = 5.0f;
    float last_error = fabsf(setpoint - measured);

    for (int i = 0; i < 2000; i++) {
        float out = pid_step(&pid, setpoint, measured);
        CHECK(out >= -10.0f - 1e-4f && out <= 10.0f + 1e-4f, "output stays within [out_min, out_max]");
        /* crude plant: measured position drifts toward commanded output */
        measured += out * pid.dt_s;
    }

    float final_error = fabsf(setpoint - measured);
    CHECK(final_error < last_error, "tracking error shrinks over the run");
    CHECK(final_error < 0.5f, "settles close to the commanded setpoint");

    pid_reset_integrator(&pid);
    CHECK(pid.integrator == 0.0f, "reset_integrator zeroes the integrator");

    if (failures > 0) {
        printf("%d check(s) FAILED\n", failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
