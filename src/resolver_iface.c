#include "resolver_iface.h"
#include "hw/adc.h"
#include "hw/excitation.h"

#define SIN_CHANNEL(lane)     ((lane) == LANE_A ? ADC_CH_RDC_A_SIN : ADC_CH_RDC_B_SIN)
#define COS_CHANNEL(lane)     ((lane) == LANE_A ? ADC_CH_RDC_A_COS : ADC_CH_RDC_B_COS)
#define CURRENT_CHANNEL(lane) ((lane) == LANE_A ? ADC_CH_MOTOR_A_I : ADC_CH_MOTOR_B_I)

/* Envelope amplitude (sin^2 + cos^2) must stay within this band of full
 * scale for the resolver signal chain to be considered healthy; outside it
 * indicates a broken excitation winding, a shorted signal line, or a
 * resolver that has come unseated. */
#define AMPLITUDE_MIN_FRACTION   0.35f
#define AMPLITUDE_MAX_FRACTION   0.95f

#define CURRENT_ADC_ZERO_COUNTS   2048
#define CURRENT_AMPS_PER_COUNT    0.0122f  /* 25A / 2048 counts, unipolar op-amp front end */

static float atan2_approx(float y, float x);

void resolver_init(lane_id_t lane)
{
    adc_configure_channel(SIN_CHANNEL(lane));
    adc_configure_channel(COS_CHANNEL(lane));
    adc_configure_channel(CURRENT_CHANNEL(lane));
    excitation_enable(lane, RESOLVER_REFERENCE_FREQ_HZ);
}

bool resolver_read(lane_id_t lane, resolver_sample_t *out)
{
    const uint16_t sin_raw = adc_read_raw(SIN_CHANNEL(lane));
    const uint16_t cos_raw = adc_read_raw(COS_CHANNEL(lane));
    const uint16_t cur_raw = adc_read_raw(CURRENT_CHANNEL(lane));

    const float sin_v = ((float)sin_raw / (float)RESOLVER_ADC_FULL_SCALE) - 0.5f;
    const float cos_v = ((float)cos_raw / (float)RESOLVER_ADC_FULL_SCALE) - 0.5f;

    const float amplitude = (sin_v * sin_v) + (cos_v * cos_v);
    out->amplitude_in_range =
        (amplitude >= AMPLITUDE_MIN_FRACTION * AMPLITUDE_MIN_FRACTION) &&
        (amplitude <= AMPLITUDE_MAX_FRACTION * AMPLITUDE_MAX_FRACTION);
    out->excitation_present = excitation_is_locked(lane);

    /* Angle -> linear position across the ballscrew stroke. The resolver is
     * geared 1:1 to the ballscrew over one full stroke, so electrical angle
     * maps linearly to position with no turns-counting needed. */
    const float angle_turns = atan2_approx(sin_v, cos_v) / (2.0f * 3.14159265f);
    out->position_mm = angle_turns * RESOLVER_STROKE_MM;

    out->motor_current_a =
        ((float)cur_raw - (float)CURRENT_ADC_ZERO_COUNTS) * CURRENT_AMPS_PER_COUNT;

    return out->excitation_present && out->amplitude_in_range;
}

/* Small fixed-point-friendly atan2 approximation (rational form, max error
 * ~0.07 rad) -- a full libm atan2 is deliberately not linked into the DAL-A
 * build, to keep the traceable code surface small. */
static float atan2_approx(float y, float x)
{
    const float abs_y = (y < 0.0f ? -y : y) + 1e-9f;
    float angle;

    if (x >= 0.0f) {
        const float r = (x - abs_y) / (x + abs_y);
        angle = (0.1963f * r * r * r) - (0.9817f * r) + (3.14159265f / 4.0f);
    } else {
        const float r = (x + abs_y) / (abs_y - x);
        angle = (0.1963f * r * r * r) - (0.9817f * r) + (3.0f * 3.14159265f / 4.0f);
    }

    return (y < 0.0f) ? -angle : angle;
}
