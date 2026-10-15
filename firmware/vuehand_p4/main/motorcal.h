#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "joints_table.h"

/* Per-motor tick calibration for the ORCA Touch, ported from
 * vuehand/motor_cal.py. Autocalibration winds each motor toward both
 * mechanical stops at a low torque limit, records the tick positions, and
 * maps the joint's official ROM linearly onto that span:
 *
 *     ticks(deg) = lo_end + dir * fill(deg) * (hi - lo)
 *
 * Without this, servo.c can only do *relative* moves around wherever the
 * hand sat when torque came on, scaled by the mock VH_RATIO -- which covers
 * a fraction of the real range (see CLAUDE.md's calibration section).
 *
 * Deliberately NOT run on every boot: winding 17 tendon-driven motors into
 * hard stops at power-up would crush anything in the hand and wear the gear
 * train for no reason. Run it once via {"cmd":"calibrate"}; the result is
 * persisted to NVS and loaded at boot. */

typedef struct {
    uint16_t lo;    /* measured tick at the low-angle mechanical stop */
    uint16_t hi;    /* measured tick at the high-angle mechanical stop */
    int8_t dir;     /* tick direction in which the joint angle increases */
    uint8_t flags;  /* see VH_CAL_F_* */
} vh_motor_cal_t;

#define VH_CAL_F_DISABLED  0x01  /* span below MIN_LIVE_SPAN: jammed or dead */
#define VH_CAL_F_HIT_BOUND 0x02  /* reached tick 0/4095 before stalling */
#define VH_CAL_F_NO_LOAD   0x04  /* stopped advancing but load never saturated:
                                  * slack tendon or an undriven joint, so the
                                  * span is suspect even if it looks plausible */
#define VH_CAL_F_FLIPPED   0x08  /* dir overridden by cal_flip: mounted opposite to the
                                    default map; survives recalibration */
#define VH_CAL_F_UNVERIFIED 0x10 /* an endpoint could not be verified (still moving at the
                                    budget, or lost its reply); previous table kept if there
                                    was one, else disabled. Recalibrate this motor. */

/* Loads any stored calibration from NVS. Returns true if a complete one was
 * found. Safe to call before the bus is up -- it only touches flash. */
bool motorcal_load(void);

/* True once a complete calibration is available (loaded or just measured). */
bool motorcal_ready(void);

/* Absolute goal ticks for one joint, using the measured span. Only valid when
 * motorcal_ready(); servo.c falls back to relative goals otherwise. */
uint16_t motorcal_ticks_for_joint(int joint_index, float deg, int side_right);

/* Exact inverse of motorcal_ticks_for_joint(): measured ticks -> joint angle
 * in degrees. This is what lets the hand report the shape it is *actually*
 * in rather than the shape it was last told to make -- the two differ
 * whenever something is in the way, which is most of the interesting cases.
 * Returns false if the joint has no usable span (disabled or unmapped
 * motor), in which case *out_deg is untouched. */
bool motorcal_deg_for_joint(int joint_index, uint16_t ticks, int side_right,
                            float *out_deg);

/* Runs the full sweep, one motor at a time, and persists the result.
 * Blocking and slow (tens of seconds) -- it physically moves the hand.
 * Returns false if the bus is down or nothing could be measured. */
bool motorcal_run(int side_right);

/* True if this motor was measured as jammed or dead and must be left limp.
 * Mirrors urt2_link's _disabled()/_live_ids(): a motor that cannot move must
 * never be torqued or given a goal, or it just sits at stall current --
 * which is how servos on this hand got cooked in the first place. */
bool motorcal_motor_disabled(int motor_id);

/* Copies the current table out for reporting. */
void motorcal_copy(vh_motor_cal_t *out);

/* Flips the tick direction for one motor (or every motor if motor_id <= 0)
 * and re-saves. `dir` is a mounting convention that autocalibration cannot
 * measure -- motor_cal.py has the same limitation and the same escape hatch
 * (MotorCal.flip), since the default -sign guess has to be validated against
 * the physical hand. */
bool motorcal_flip(int motor_id);

/* Reads back the calibrated span for one motor, for diagnostics. Returns
 * false if that motor has no calibration. */
bool motorcal_span(int motor_id, uint16_t *out_lo, uint16_t *out_hi, int8_t *out_dir);

/* Clears the stored calibration (both in RAM and NVS). */
bool motorcal_clear(void);
