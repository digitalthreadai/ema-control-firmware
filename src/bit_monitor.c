#include "bit_monitor.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

typedef struct {
    bit_fault_flags_t latched;
    uint32_t debounce_counts[5]; /* one per fault bit above */
} bit_state_t;

static bit_state_t s_bit[2];

/* Peer lane's last sample, written by the OTHER lane's bit_monitor_update()
 * call this cycle -- both lanes run back-to-back on the same core in this
 * configuration, so by the time lane B's cross-check runs, lane A's sample
 * for this cycle is already current. */
static resolver_sample_t s_peer_sample[2];
static bool s_peer_sample_valid[2];

void bit_monitor_init(lane_id_t lane)
{
    memset(&s_bit[lane], 0, sizeof(s_bit[lane]));
    s_peer_sample_valid[lane] = false;
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

    /* Cross-lane comparison (REQ-00055): compare this lane's sample against
     * whatever the peer lane recorded this cycle, if it has run yet. */
    const lane_id_t peer = (lane == LANE_A) ? LANE_B : LANE_A;
    if (s_peer_sample_valid[peer]) {
        const float pos_delta = fabsf(sample->position_mm - s_peer_sample[peer].position_mm);
        const float cur_delta = fabsf(sample->motor_current_a - s_peer_sample[peer].motor_current_a);

        debounce_fault(st, 3, BIT_FAULT_CROSS_LANE_POS, pos_delta > BIT_XLANE_POSITION_TOLERANCE_MM);
        debounce_fault(st, 4, BIT_FAULT_CROSS_LANE_CURRENT, cur_delta > BIT_XLANE_CURRENT_TOLERANCE_A);
    }

    s_peer_sample[lane] = *sample;
    s_peer_sample_valid[lane] = true;

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

bool bit_monitor_clear_latch(lane_id_t lane)
{
    bit_state_t *st = &s_bit[lane];
    const size_t n = sizeof(st->debounce_counts) / sizeof(st->debounce_counts[0]);

    /* Refuse to clear while any debounce counter is still pegged at the
     * latch threshold -- that means the underlying condition is still
     * present this cycle, not just a stale latch from a past transient. */
    for (size_t i = 0; i < n; i++) {
        if (st->debounce_counts[i] >= BIT_LATCH_DEBOUNCE_CYCLES) {
            return false;
        }
    }

    st->latched = BIT_FAULT_NONE;
    return true;
}
