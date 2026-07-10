#ifndef HW_EXCITATION_H
#define HW_EXCITATION_H

#include <stdint.h>
#include <stdbool.h>
#include "../lanes.h"

/* Resolver excitation driver contract -- board support package generates the
 * fixed-frequency sine excitation and reports PLL lock status per lane. */

void excitation_enable(lane_id_t lane, uint32_t reference_freq_hz);
bool excitation_is_locked(lane_id_t lane);

#endif /* HW_EXCITATION_H */
