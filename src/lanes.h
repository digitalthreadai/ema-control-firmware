#ifndef LANES_H
#define LANES_H

/* The EMA runs two independent control lanes (REQ-00055 -- dual-channel
 * position monitoring and cross-lane comparison). Shared by every module
 * that is lane-aware, so it lives in its own header rather than pulled in
 * transitively from control_loop.h. */

typedef enum {
    LANE_A = 0,
    LANE_B = 1,
} lane_id_t;

#endif /* LANES_H */
