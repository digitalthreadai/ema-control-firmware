#include "bit_monitor.h"
#include <math.h>
#include <string.h>

typedef struct {
    bit_fault_flags_t latched;
    uint32_t debounce_counts[3]; /* one per fault bit above */
} bit_state_t;

static bit_state_t s_bit[2];

void bit_monitor_init(lane_id_t lane)
{
    memset(&s_bit[lane], 0, sizeof(s_bit[lane]));
}

static void debounce_fault(bit_state_t *st, int bit_index, bit_fault_flags_t flag, bool condition_active)
{
    if (!condition_active) {
        st->debounce_counts[bit_index] = 0;
        return;
    }
    if (st->debounce_counts[bit_index] < BIT_LATCH_DEBOUNCE_CYCLES) {
        st->debounce_counts[bit_index]++;
    }
    if (st->debounce_counts[bit_index] >= BIT_LATCH_DEBOUNCE_CYCLES) {
        st->latched |= flag; /* sticky -- ground maintenance clears it, see below */
    }
}

void bit_monitor_update(lane_id_t lane, bool resolver_ok, const resolver_sample_t *sample)
{
    bit_state_t *st = &s_bit[lane];

    debounce_fault(st, 0, BIT_FAULT_EXCITATION_LOST, !sample->excitation_present);
    debounce_fault(st, 1, BIT_FAULT_AMPLITUDE_OOR, !sample->amplitude_in_range);
    debounce_fault(st, 2, BIT_FAULT_OVERCURRENT,
                   fabsf(sample->motor_current_a) > BIT_OVERCURRENT_LIMIT_A);

    (void)resolver_ok; /* folded into EXCITATION_LOST / AMPLITUDE_OOR above */
}

bool bit_monitor_is_healthy(lane_id_t lane)
{
    return s_bit[lane].latched == BIT_FAULT_NONE;
}

bit_fault_flags_t bit_monitor_latched_faults(lane_id_t lane)
{
    return s_bit[lane].latched;
}
