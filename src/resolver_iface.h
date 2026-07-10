#ifndef RESOLVER_IFACE_H
#define RESOLVER_IFACE_H

#include <stdint.h>
#include <stdbool.h>
#include "lanes.h"

/* Resolver excitation + decode (REQ-00043 -- resolver position feedback
 * accuracy). The resolver is driven with a fixed RESOLVER_REFERENCE_FREQ_HZ
 * sine excitation; SIN/COS envelope amplitudes are demodulated in hardware
 * (RDC front end) and read here as raw ADC counts, then converted to
 * engineering units. */

#define RESOLVER_REFERENCE_FREQ_HZ   10000U
#define RESOLVER_ADC_FULL_SCALE      4095U
#define RESOLVER_STROKE_MM           63.5f

typedef struct {
    float position_mm;
    float motor_current_a;
    bool  excitation_present;
    bool  amplitude_in_range;
} resolver_sample_t;

void resolver_init(lane_id_t lane);

/* Samples the RDC front end for `lane`, checks excitation health, and fills
 * `out`. Returns false if the sample should not be trusted this cycle
 * (excitation dropout or out-of-range envelope amplitude) -- the caller
 * (control_loop_step) treats a false return as a fault-equivalent input. */
bool resolver_read(lane_id_t lane, resolver_sample_t *out);

#endif /* RESOLVER_IFACE_H */
