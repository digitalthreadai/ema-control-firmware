/* Minimal smoke test for pid_step — no framework, no BSP dependency.
 * Runs a fixed set of named checks and emits a JUnit-style XML report to
 * build/junit.xml so CI can ingest pass/fail counts per case (mirrors the
 * shape lib/alm/sync.ts's parseJUnitXml expects: testsuite attrs + nested
 * testcase/failure elements). */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../src/pid.h"

typedef struct {
    const char *name;
    int passed;
} test_case_t;

#define MAX_CASES 16
static test_case_t g_cases[MAX_CASES];
static int g_case_count = 0;

static void record(const char *name, int passed)
{
    g_cases[g_case_count].name = name;
    g_cases[g_case_count].passed = passed;
    g_case_count++;
    printf("%s: %s\n", passed ? "ok  " : "FAIL", name);
}

/* Drives pid_step toward a fixed setpoint over a run of steps against a
 * crude first-order plant model; returns the resulting state for the
 * individual checks below to inspect. */
typedef struct {
    float final_error;
    float initial_error;
    int   ever_out_of_range;
} run_result_t;

static run_result_t run_pid_toward_setpoint(void)
{
    pid_state_t pid;
    pid_init(&pid, /*kp*/2.0f, /*ki*/0.5f, /*kd*/0.05f,
              /*out_min*/-10.0f, /*out_max*/10.0f, /*dt_s*/0.001f);

    float measured = 0.0f;
    const float setpoint = 5.0f;
    run_result_t r;
    r.initial_error = fabsf(setpoint - measured);
    r.ever_out_of_range = 0;

    for (int i = 0; i < 2000; i++) {
        float out = pid_step(&pid, setpoint, measured);
        if (out < -10.0f - 1e-4f || out > 10.0f + 1e-4f) r.ever_out_of_range = 1;
        measured += out * pid.dt_s;
    }
    r.final_error = fabsf(setpoint - measured);
    return r;
}

static void test_output_stays_clamped(void)
{
    run_result_t r = run_pid_toward_setpoint();
    record("test_pid_output_stays_within_out_min_out_max", !r.ever_out_of_range);
}

static void test_error_converges(void)
{
    run_result_t r = run_pid_toward_setpoint();
    record("test_pid_tracking_error_shrinks_over_run", r.final_error < r.initial_error);
}

static void test_settles_near_setpoint(void)
{
    run_result_t r = run_pid_toward_setpoint();
    record("test_pid_settles_within_half_mm_of_setpoint", r.final_error < 0.5f);
}

static void test_reset_integrator_zeroes(void)
{
    pid_state_t pid;
    pid_init(&pid, 2.0f, 0.5f, 0.05f, -10.0f, 10.0f, 0.001f);
    pid_step(&pid, 5.0f, 0.0f); /* build up some integrator state */
    pid_reset_integrator(&pid);
    record("test_pid_reset_integrator_zeroes_state", pid.integrator == 0.0f && !pid.has_prev_error);
}

static void write_junit_xml(const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f) return;
    int failed = 0;
    for (int i = 0; i < g_case_count; i++) if (!g_cases[i].passed) failed++;

    fprintf(f, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    fprintf(f, "<testsuites>\n");
    fprintf(f, "  <testsuite name=\"pid_step\" tests=\"%d\" failures=\"%d\" errors=\"0\" skipped=\"0\">\n",
            g_case_count, failed);
    for (int i = 0; i < g_case_count; i++) {
        fprintf(f, "    <testcase name=\"%s\" classname=\"pid\"", g_cases[i].name);
        if (g_cases[i].passed) {
            fprintf(f, "/>\n");
        } else {
            fprintf(f, ">\n      <failure message=\"check failed\"/>\n    </testcase>\n");
        }
    }
    fprintf(f, "  </testsuite>\n");
    fprintf(f, "</testsuites>\n");
    fclose(f);
}

int main(void)
{
    test_output_stays_clamped();
    test_error_converges();
    test_settles_near_setpoint();
    test_reset_integrator_zeroes();

    write_junit_xml("build/junit.xml");

    int failures = 0;
    for (int i = 0; i < g_case_count; i++) if (!g_cases[i].passed) failures++;

    if (failures > 0) {
        printf("%d/%d check(s) FAILED\n", failures, g_case_count);
        return 1;
    }
    printf("all %d checks passed\n", g_case_count);
    return 0;
}
