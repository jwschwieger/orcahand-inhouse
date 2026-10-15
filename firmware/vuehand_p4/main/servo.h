#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "joints_table.h"

/* REACH torque, per-mille: the ceiling used while a pose is actually in
 * flight. Shared with motorcal so endpoints are verified at exactly the
 * torque that will later be asked to reach them -- a creep endpoint found at
 * 150 permille is not reachable as a goal at this value, which is the bug
 * that cooked motor 10. Do not make this profile-dependent: if reach torque
 * varied by grasp profile, a soft grasp would be commanded to endpoints it
 * cannot reach, and stall there. Reach is a property of the hand's
 * mechanics; grasp force is the knob (see the grasp profiles below). */
#define VH_TORQUE_LIMIT_ARMED 300

/* How long after a pose is written the reach ceiling stays raised before
 * dropping to the profile's holding torque. Long enough for LIVE_SPEED to
 * cross the widest measured span (~3900 ticks at 2200 ticks/s). */
#define VH_REACH_MS 1200

/* Hard ceiling on holding torque, whatever anyone asks for. 500+ permille
 * held against a stop is how these servos get destroyed. */
#define VH_GRASP_MAX_HOLD 450

/* Grasp force is a *holding* torque limit, not a pose: the same closed
 * shape squeezes an object as hard as the ceiling lets the motor push
 * against the position error the object creates. Once a motor has actually
 * arrived at its goal there is no error, so the ceiling costs nothing --
 * it only bites when something is in the way, which is exactly when it
 * should. Each profile carries its own auto-relax leash: the harder the
 * squeeze, the shorter the hand is allowed to hold it unattended. */
typedef struct {
    const char *name;
    uint16_t hold_permille;
    uint32_t hold_ms;
} vh_grasp_profile_t;

/* Selects a named profile ("soft", "medium", "firm").
 * Returns 0 on success, -1 unknown name, -2 refused because the hand is
 * already too warm for a hard squeeze. */
int servo_set_grasp(const char *name);
/* Same, with an explicit holding torque in per-mille. Clamped into
 * [40, VH_GRASP_MAX_HOLD]; the leash comes from the nearest named profile.
 * Returns 0 or -2 as above. */
int servo_set_grasp_permille(uint16_t permille);
/* Gentle: the next closes run at the holding ceiling instead of reach torque
 * (a grasp does not need to reach an endpoint). Sticky until changed. */
void servo_set_gentle(bool on);
bool servo_gentle(void);
/* Ceiling used to close in gentle mode (default VH_GENTLE_DEFAULT); clamped to
 * [hold ceiling, reach torque] when applied. */
#define VH_GENTLE_DEFAULT 180
void servo_set_gentle_limit(uint16_t permille);
uint16_t servo_gentle_limit(void);
const char *servo_grasp_name(void);
uint16_t servo_grasp_hold(void);
uint32_t servo_grasp_hold_ms(void);
/* Highest temperature seen on the last thermal sweep (0 = nothing read). */
uint8_t servo_hottest_temp(void);

void servo_init(int side_right);
/* Which hand this firmware was initialised for -- 1 = right. */
int servo_side_right(void);
void servo_enable_torque(bool on);
void servo_set_vec(const float *deg17, int t_ms);
void servo_relax(void);
void servo_current_copy(float *out17);

/* Thermal protection (see servo.c). The monitor task polls Present
 * Temperature, ramps each finger's torque limit down as it heats, and cuts
 * torque entirely before the servos hit their own ~70 C shutdown. */
void servo_thermal_start(void);
/* Last known temperature per motor, indexed like VH_MOTOR_IDS. 0 = never
 * read successfully. `out_limits` (may be NULL) gets the torque limit
 * currently applied to each motor, in per-mille. */
void servo_temps_copy(uint8_t *out17, uint16_t *out_limits, int32_t *out_ma);
/* The P4's own die temperature in C (0 if the sensor is unavailable).
 * Reported for visibility only -- nothing throttles off it. */
float servo_board_temp(void);
/* True once a motor crossed the cut threshold and torque was dropped;
 * arming is refused until everything is back under the warn threshold. */
bool servo_thermal_cut(uint8_t *out_hottest_temp, uint8_t *out_hottest_id);
