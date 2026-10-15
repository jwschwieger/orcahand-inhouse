#include "motorcal.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "feetech.h"
#include "servo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "motorcal";

/* Autocalibration parameters, ported 1:1 from motor_cal.AUTOCAL_DEFAULTS.
 * Tendon hand: small steps, low torque, stall means the stop was found. */
#define CAL_STEP 40           /* ticks per increment (~3.5 deg) */
#define CAL_SETTLE_MS 80      /* wait between increments */
#define CAL_STALL_TICKS 5     /* less progress than this counts as a stall hit */
#define CAL_STALL_HITS 3      /* consecutive stall hits => stop found */
#define CAL_TORQUE_LIMIT 150  /* per-mille while probing -- never full torque */
#define CAL_MAX_TRAVEL 3900   /* give up after this much travel from the start */
#define CAL_SPEED 600         /* ticks/s while probing (> step/settle) */
#define CAL_ACC 20            /* x100 ticks/s^2 */
#define CAL_MIN_LIVE_SPAN 60  /* measured span below this => jammed or dead */
#define CAL_MARGIN 0.02f      /* keep goals this fraction of span off each stop */

#define ADDR_TORQUE_ENABLE 40

/* A creep endpoint is not a reachable goal. Autocal inches 40 ticks at a time
 * while antagonist tension rises, so it records a stop the motor can only
 * arrive at by creeping under one particular pose. Commanded as a one-shot
 * at pose torque, the same motor saturates hundreds of ticks short -- and
 * then sits there stalled at its cap, which is I2R with no motion to show for
 * it. That is how the hand overheated holding a fist. So every endpoint is
 * re-commanded at VH_TORQUE_LIMIT_ARMED and whatever position it actually
 * lands at becomes the stored bound. */

/* Wall-clock ceiling per motor, so a jam cannot spend the whole travel
 * budget grinding. Motor 4 (span 0) took 672ms; a healthy motor takes ~6s. */
#define CAL_MOTOR_BUDGET_MS 12000

/* Accept a stop on creep alone after this many extra hits, if load never
 * saturates. Without an escape a motor whose load never pins would burn the
 * full travel budget -- the exact stall-heat pattern being avoided. */
#define CAL_STALL_HITS_NO_LOAD 9

#define TICK_MIN 0
#define TICK_MAX 4095

/* An end that reached the encoder's edge is not a mechanical stop -- the
 * joint has more travel than one servo turn (the wrist: probe ran to tick 26,
 * verify confirmed 39). A goal near the edge is fine until an external push
 * takes the shaft past it: the position reading wraps to the far end and the
 * servo drives the wrong way at full effort. So a bound end is inset by this
 * much and the joint's ROM maps onto the rest. Multi-turn mode is the real
 * fix and is a deliberate, separate step. */
#define CAL_BOUND_INSET 300

#define NVS_NAMESPACE "vuehand"
#define NVS_KEY "motorcal"
#define CAL_BLOB_VERSION 1

typedef struct {
    uint8_t version;
    uint8_t n_motors;
    uint8_t complete;
    uint8_t reserved;
    vh_motor_cal_t motors[VH_N_MOTORS];
} cal_blob_t;

static cal_blob_t s_cal;
static bool s_ready = false;

static int motor_index_of(int motor_id)
{
    for (int i = 0; i < VH_N_MOTORS; i++) {
        if (VH_MOTOR_IDS[i] == motor_id) {
            return i;
        }
    }
    return -1;
}

/* Tick direction in which the joint angle increases. Mirrors
 * motor_cal.default_dir(): -sign from the joint->motor map, matching
 * urt2_link's TICK_SIGN convention. Stored per motor so a motor mounted the
 * other way can be corrected without touching this logic. */
static int8_t default_dir_for_joint(int joint_index, int side_right)
{
    const int *map = side_right ? VH_RIGHT_MAP : VH_LEFT_MAP;
    int signed_id = map[joint_index];
    int sign = signed_id < 0 ? -1 : 1;
    return (int8_t)(-sign);
}

static int joint_for_motor(int motor_id, int side_right)
{
    const int *map = side_right ? VH_RIGHT_MAP : VH_LEFT_MAP;
    for (int j = 0; j < VH_N_JOINTS; j++) {
        int m = map[j] < 0 ? -map[j] : map[j];
        if (m == motor_id) {
            return j;
        }
    }
    return -1;
}

/* ---------------------------------------------------------------- storage */

static bool cal_save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return false;
    }
    err = nvs_set_blob(h, NVS_KEY, &s_cal, sizeof s_cal);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "saving calibration failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "calibration saved to NVS");
    return true;
}

bool motorcal_load(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no stored calibration yet -- run {\"cmd\":\"calibrate\"}");
        return false;
    }
    size_t len = sizeof s_cal;
    esp_err_t err = nvs_get_blob(h, NVS_KEY, &s_cal, &len);
    nvs_close(h);

    if (err != ESP_OK || len != sizeof s_cal) {
        ESP_LOGI(TAG, "no usable stored calibration -- run {\"cmd\":\"calibrate\"}");
        memset(&s_cal, 0, sizeof s_cal);
        return false;
    }
    if (s_cal.version != CAL_BLOB_VERSION || s_cal.n_motors != VH_N_MOTORS) {
        ESP_LOGW(TAG, "stored calibration is v%u for %u motors, expected v%u/%u -- ignoring",
                 s_cal.version, s_cal.n_motors, CAL_BLOB_VERSION, VH_N_MOTORS);
        memset(&s_cal, 0, sizeof s_cal);
        return false;
    }

    s_ready = s_cal.complete != 0;
    int disabled = 0;
    int flipped = 0;
    for (int i = 0; i < VH_N_MOTORS; i++) {
        if (s_cal.motors[i].flags & VH_CAL_F_DISABLED) {
            disabled++;
        }
        /* Blobs written before VH_CAL_F_FLIPPED existed carry a flipped dir
         * with no flag; derive it so the override survives the next sweep. */
        int j = joint_for_motor(VH_MOTOR_IDS[i], 1);
        int8_t def = (j >= 0) ? default_dir_for_joint(j, 1) : 1;
        if (s_cal.motors[i].dir != def) {
            s_cal.motors[i].flags |= VH_CAL_F_FLIPPED;
            flipped++;
        }
        /* Tables written before CAL_BOUND_INSET existed can carry an end at
         * the encoder edge (wrist lo=39 on 2026-09-02). Inset on load too. */
        if (s_cal.motors[i].flags & VH_CAL_F_HIT_BOUND) {
            if (s_cal.motors[i].lo < TICK_MIN + CAL_BOUND_INSET) {
                ESP_LOGW(TAG, "motor %d: stored low end %u at the encoder edge -- using %u",
                         VH_MOTOR_IDS[i], (unsigned)s_cal.motors[i].lo, (unsigned)(TICK_MIN + CAL_BOUND_INSET));
                s_cal.motors[i].lo = TICK_MIN + CAL_BOUND_INSET;
            }
            if (s_cal.motors[i].hi > TICK_MAX - CAL_BOUND_INSET) {
                ESP_LOGW(TAG, "motor %d: stored high end %u at the encoder edge -- using %u",
                         VH_MOTOR_IDS[i], (unsigned)s_cal.motors[i].hi, (unsigned)(TICK_MAX - CAL_BOUND_INSET));
                s_cal.motors[i].hi = TICK_MAX - CAL_BOUND_INSET;
            }
        }
    }
    ESP_LOGI(TAG, "loaded calibration from NVS (%s, %d motor(s) disabled, %d dir override(s))",
             s_ready ? "complete" : "incomplete", disabled, flipped);
    return s_ready;
}

/* Flip one motor's direction and record whether it now differs from the
 * default map, so recalibration can reapply it. The flag is derived from the
 * comparison rather than toggled, so flipping twice lands back on "default"
 * and an already-flipped blob from before the flag existed is handled. */
static void flip_one(int i, int side_right)
{
    int id = VH_MOTOR_IDS[i];
    int j = joint_for_motor(id, side_right);
    int8_t def = (j >= 0) ? default_dir_for_joint(j, side_right) : 1;
    s_cal.motors[i].dir = (int8_t)(-s_cal.motors[i].dir);
    if (s_cal.motors[i].dir != def) {
        s_cal.motors[i].flags |= VH_CAL_F_FLIPPED;
    } else {
        s_cal.motors[i].flags &= (uint8_t)~VH_CAL_F_FLIPPED;
    }
}

bool motorcal_flip(int motor_id)
{
    if (!s_ready) {
        return false;
    }
    if (motor_id <= 0) {
        for (int i = 0; i < VH_N_MOTORS; i++) {
            flip_one(i, 1);
        }
        ESP_LOGW(TAG, "flipped tick direction on all %d motors", VH_N_MOTORS);
    } else {
        int mi = motor_index_of(motor_id);
        if (mi < 0) {
            return false;
        }
        flip_one(mi, 1);
        ESP_LOGW(TAG, "flipped tick direction on motor %d (now %d%s)",
                 motor_id, s_cal.motors[mi].dir,
                 (s_cal.motors[mi].flags & VH_CAL_F_FLIPPED) ? ", override kept across recalibration" : ", back to default");
    }
    return cal_save();
}

bool motorcal_span(int motor_id, uint16_t *out_lo, uint16_t *out_hi, int8_t *out_dir)
{
    int mi = motor_index_of(motor_id);
    if (mi < 0 || !s_ready) {
        return false;
    }
    if (out_lo) *out_lo = s_cal.motors[mi].lo;
    if (out_hi) *out_hi = s_cal.motors[mi].hi;
    if (out_dir) *out_dir = s_cal.motors[mi].dir;
    return true;
}

bool motorcal_clear(void)
{
    memset(&s_cal, 0, sizeof s_cal);
    s_ready = false;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return false;
    }
    esp_err_t err = nvs_erase_key(h, NVS_KEY);
    if (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    ESP_LOGW(TAG, "calibration cleared");
    return err == ESP_OK;
}

bool motorcal_ready(void)
{
    return s_ready;
}

bool motorcal_motor_disabled(int motor_id)
{
    if (!s_ready) {
        return false; /* uncalibrated: nothing is known to be dead */
    }
    int mi = motor_index_of(motor_id);
    if (mi < 0) {
        return false;
    }
    return (s_cal.motors[mi].flags & VH_CAL_F_DISABLED) != 0;
}

void motorcal_copy(vh_motor_cal_t *out)
{
    if (out) {
        memcpy(out, s_cal.motors, sizeof s_cal.motors);
    }
}

/* ------------------------------------------------------------ goal mapping */

uint16_t motorcal_ticks_for_joint(int joint_index, float deg, int side_right)
{
    const int *map = side_right ? VH_RIGHT_MAP : VH_LEFT_MAP;
    int motor = map[joint_index] < 0 ? -map[joint_index] : map[joint_index];
    int mi = motor_index_of(motor);
    if (mi < 0) {
        return 2048;
    }
    const vh_motor_cal_t *m = &s_cal.motors[mi];

    float lo_deg = VH_ROM_LO[joint_index];
    float hi_deg = VH_ROM_HI[joint_index];
    if (deg < lo_deg) deg = lo_deg;
    if (deg > hi_deg) deg = hi_deg;

    float span_deg = hi_deg - lo_deg;
    float fill = span_deg <= 0.0f ? 0.0f : (deg - lo_deg) / span_deg;

    /* Keep off the hard stops -- motor_cal.DEFAULT_MARGIN. */
    fill = CAL_MARGIN + fill * (1.0f - 2.0f * CAL_MARGIN);

    float span_ticks = (float)m->hi - (float)m->lo;
    float t = (m->dir >= 0) ? ((float)m->lo + fill * span_ticks)
                            : ((float)m->hi - fill * span_ticks);
    if (t < (float)TICK_MIN) t = (float)TICK_MIN;
    if (t > (float)TICK_MAX) t = (float)TICK_MAX;
    return (uint16_t)lroundf(t);
}

bool motorcal_deg_for_joint(int joint_index, uint16_t ticks, int side_right,
                            float *out_deg)
{
    const int *map = side_right ? VH_RIGHT_MAP : VH_LEFT_MAP;
    int motor = map[joint_index] < 0 ? -map[joint_index] : map[joint_index];
    int mi = motor_index_of(motor);
    if (mi < 0) {
        return false;
    }
    const vh_motor_cal_t *m = &s_cal.motors[mi];
    if (m->flags & VH_CAL_F_DISABLED) {
        return false;
    }

    float span_ticks = (float)m->hi - (float)m->lo;
    if (span_ticks <= 0.0f) {
        return false;
    }

    /* Undo motorcal_ticks_for_joint() term by term, in reverse. */
    float fill = (m->dir >= 0) ? (((float)ticks - (float)m->lo) / span_ticks)
                               : (((float)m->hi - (float)ticks) / span_ticks);

    /* Back out the stop margin. Note this can legitimately land outside
     * [0,1] -- the forward map keeps commands off the hard stops, but a
     * motor pushed by an external force can sit anywhere in the measured
     * span. Clamping to the ROM is right: reporting an angle outside the
     * joint's declared range would be a lie about a joint, not a
     * measurement. */
    fill = (fill - CAL_MARGIN) / (1.0f - 2.0f * CAL_MARGIN);
    if (fill < 0.0f) fill = 0.0f;
    if (fill > 1.0f) fill = 1.0f;

    float lo_deg = VH_ROM_LO[joint_index];
    float hi_deg = VH_ROM_HI[joint_index];
    *out_deg = lo_deg + fill * (hi_deg - lo_deg);
    return true;
}

/* ---------------------------------------------------------------- probing */

/* Winds one motor in `dir` until it stops making progress (or its load
 * spikes), and returns the tick it settled at. `hit_bound` is set if it ran
 * into 0/4095 first -- per CLAUDE.md, that means the joint needs more than
 * one servo turn and must be flagged rather than clamped and forgotten. */
static bool probe_stop(uint8_t id, int dir, uint16_t *out_tick, bool *hit_bound,
                        bool *no_load)
{
    uint16_t pos;
    if (!ft_read_present_position(id, &pos)) {
        ESP_LOGW(TAG, "motor %u: no position reply, skipping", id);
        return false;
    }

    *hit_bound = false;
    *no_load = false;
    int hits = 0;
    int creep_only = 0;
    int travelled = 0;
    uint16_t last = pos;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(CAL_MOTOR_BUDGET_MS);

    while (travelled < CAL_MAX_TRAVEL) {
        if ((int32_t)(xTaskGetTickCount() - deadline) >= 0) {
            ESP_LOGW(TAG, "motor %u: probe budget exhausted, giving up on this end", id);
            break;
        }
        int goal = (int)last + dir * CAL_STEP;
        if (goal <= TICK_MIN || goal >= TICK_MAX) {
            goal = goal <= TICK_MIN ? TICK_MIN : TICK_MAX;
            *hit_bound = true;
        }

        if (!ft_write_goal(id, (uint16_t)goal)) {
            ESP_LOGW(TAG, "motor %u: goal write failed mid-probe", id);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));

        uint16_t now;
        if (!ft_read_present_position(id, &now)) {
            /* A dropped reply is not a stall; don't count it as one. */
            continue;
        }

        int progress = (int)now - (int)last;
        if (progress < 0) progress = -progress;

        /* Stall = effort saturated AND position not advancing AND not moving.
         * Saturation alone is not a mechanical stop: load pins at whatever
         * TORQUE_LIMIT is set to during any hard hold, so `load >= 500` at a
         * 150 per-mille probe cap was unreachable by construction and this
         * loop was previously finding stops on the creep rule alone. */
        int16_t load = 0;
        bool have_load = ft_read_load(id, &load);
        int16_t mag = load < 0 ? (int16_t)(-load) : load;
        bool saturated = have_load && mag >= (int16_t)CAL_TORQUE_LIMIT;

        bool moving = false;
        bool have_moving = ft_read_moving(id, &moving);

        bool stopped = (progress < CAL_STALL_TICKS) && (!have_moving || !moving);

        if (stopped && saturated) {
            hits++;
            creep_only = 0;
        } else if (stopped) {
            creep_only++;   /* not loading up -- slack, or a soft limit */
            hits = 0;
        } else {
            hits = 0;
            creep_only = 0;
        }

        travelled += progress;
        last = now;

        if (hits >= CAL_STALL_HITS) {
            break;                      /* loaded stop: trust it */
        }
        if (creep_only >= CAL_STALL_HITS_NO_LOAD) {
            *no_load = true;            /* stopped advancing but never loaded */
            break;
        }
        if (*hit_bound) {
            /* Already pinned at the tick limit and still being asked for
             * more -- stop rather than grind. */
            break;
        }
    }

    *out_tick = last;
    return true;
}

/* Endpoint verification: approach `target` from `standoff` as a real goal at
 * pose torque and report where the motor actually settles.
 *
 * Three things this has to get right, each learned on this hand:
 *
 *  - Approach from a distance. The probe leaves the motor sitting on the last
 *    end it found, so the old code's verify of that end was a no-op that
 *    recorded the creep value as "verified".
 *  - Wait for the motor, not the clock. The old fixed 1.2 s wait recorded
 *    wherever the shaft was when the timer expired: every curl motor
 *    travelled the same ~1100-1200 ticks and stopped 170-350 short, which is
 *    a timer, not ten different tendons. Poll until the position stops
 *    changing over a real window, then classify: arrived near the goal, or
 *    stalled short (which is the reachable end at this torque).
 *  - Fail closed. Still moving at the budget, or a lost reply, is not an
 *    endpoint. The goal is rewritten to the present position so nothing keeps
 *    pulling, and the caller keeps the previous table for this motor or
 *    disables it -- never the creep value (inflated range -> stall -> heat)
 *    and never the in-flight tick (silently shrunk range).
 *
 * Runs at the probe's speed/acc, not pose speed: slamming a tendon into its
 * stop at 2200 ticks/s is not a test anyone wants. */
typedef enum { VERIFY_ARRIVED, VERIFY_STALLED, VERIFY_TIMEOUT, VERIFY_IO } verify_status_t;

#define CAL_VERIFY_ARRIVE_TOL 8          /* measured endpoint repeatability is ~8 ticks */
#define CAL_VERIFY_STABLE_READS 3        /* 3 x CAL_SETTLE_MS (~240 ms) of no progress */
#define CAL_VERIFY_BUDGET_MS 6000        /* a backstop for "something is wrong" */

static const char *verify_name(verify_status_t s)
{
    switch (s) {
    case VERIFY_ARRIVED: return "arrived";
    case VERIFY_STALLED: return "stalled";
    case VERIFY_TIMEOUT: return "timeout";
    default: return "no reply";
    }
}

/* Drive to `goal` and wait until the shaft stops moving. Returns where it
 * stopped in *out, or TIMEOUT / IO. */
static verify_status_t settle_at(uint8_t id, uint16_t goal, uint16_t *out)
{
    if (!ft_write_goal(id, goal)) {
        return VERIFY_IO;
    }
    uint16_t prev;
    if (!ft_read_present_position(id, &prev)) {
        return VERIFY_IO;
    }
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(CAL_VERIFY_BUDGET_MS);
    int stable = 0;
    while ((int32_t)(xTaskGetTickCount() - deadline) < 0) {
        vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));
        uint16_t cur;
        if (!ft_read_present_position(id, &cur)) {
            return VERIFY_IO;
        }
        int d = (int)cur - (int)prev;
        stable = (d <= 2 && d >= -2) ? stable + 1 : 0;
        prev = cur;
        if (stable >= CAL_VERIFY_STABLE_READS) {
            *out = cur;
            return (cur >= goal ? cur - goal : goal - cur) <= CAL_VERIFY_ARRIVE_TOL
                       ? VERIFY_ARRIVED : VERIFY_STALLED;
        }
    }
    *out = prev;
    return VERIFY_TIMEOUT;
}

static verify_status_t verify_endpoint(uint8_t id, uint16_t target, uint16_t standoff,
                                       uint16_t *landed)
{
    ft_set_torque_limit(id, VH_TORQUE_LIMIT_ARMED);
    ft_set_speed_acc(id, CAL_SPEED, CAL_ACC);

    uint16_t at = 0;
    verify_status_t st = settle_at(id, standoff, &at);
    if (st == VERIFY_TIMEOUT || st == VERIFY_IO) {
        ESP_LOGW(TAG, "motor %u: could not reach standoff %u (%s)", id, (unsigned)standoff, verify_name(st));
        uint16_t cur;
        if (ft_read_present_position(id, &cur)) {
            ft_write_goal(id, cur);          /* stop pulling on whatever it is stuck against */
        }
        return st;
    }

    st = settle_at(id, target, &at);
    if (st == VERIFY_TIMEOUT || st == VERIFY_IO) {
        uint16_t cur;
        if (ft_read_present_position(id, &cur)) {
            ft_write_goal(id, cur);
        }
        return st;
    }
    *landed = at;
    return st;
}

bool motorcal_run(int side_right)
{
    if (!ft_transport_ready()) {
        ESP_LOGE(TAG, "servo bus not ready -- refusing to calibrate");
        return false;
    }

    ESP_LOGW(TAG, "starting autocalibration -- the hand will move; keep hands clear");

    /* A cal_flip is a statement about how the motor is mounted, not about
     * this sweep, so it must outlive the sweep. Motor 14 (thumb_abd) on the
     * Touch is mounted opposite to the v2 map; the flip was lost the first
     * time the hand was recalibrated and the thumb went back to folding
     * across the palm in the open pose. */
    bool flipped[VH_N_MOTORS] = {0};
    vh_motor_cal_t prev[VH_N_MOTORS];
    bool had_prev = s_ready;
    memcpy(prev, s_cal.motors, sizeof prev);
    if (s_ready) {
        for (int i = 0; i < VH_N_MOTORS; i++) {
            flipped[i] = (s_cal.motors[i].flags & VH_CAL_F_FLIPPED) != 0;
        }
    }

    memset(&s_cal, 0, sizeof s_cal);
    s_cal.version = CAL_BLOB_VERSION;
    s_cal.n_motors = VH_N_MOTORS;
    s_ready = false;

    int measured = 0;
    int disabled = 0;

    /* Hold every motor where it is before probing any of them.
     *
     * Measured 2026-09-02, two sweeps from the same open posture: one end of
     * every motor repeated within a few ticks and the other end wandered by
     * 100-360 ticks (index_mcp lo 1091 -> 731, thumb_abd lo 666 -> 361,
     * thumb_cmc lo 745 -> 913, leaving the CMC a 72-tick span). The steady
     * end is a hard stop. The wandering end is where the probe stalls against
     * tendon tension, and that tension depends on where the coupled joints
     * are sitting -- which, with the rest of the hand limp, is wherever the
     * previous probes dragged them. So the stop the thumb CMC finds depends
     * on what the three thumb probes before it left behind.
     *
     * Holding the others at pose torque (the same ceiling verify_endpoint()
     * and every pose use, so a 300-permille verify cannot shove a neighbour
     * held at 150 off its goal) turns that soft end into a measurement
     * against a defined posture. Goal is parked at present position first so
     * enabling torque does not make anything chase a stale goal register. */
    int held = 0;
    for (int i = 0; i < VH_N_MOTORS; i++) {
        uint8_t id = (uint8_t)VH_MOTOR_IDS[i];
        uint16_t here;
        if (!ft_read_present_position(id, &here)) {
            continue;
        }
        ft_set_torque_limit(id, VH_TORQUE_LIMIT_ARMED);
        ft_set_speed_acc(id, CAL_SPEED, CAL_ACC);
        ft_write_goal(id, here);
        ft_write_u8(id, ADDR_TORQUE_ENABLE, 1);
        held++;
    }
    ESP_LOGI(TAG, "holding %d/%d motors at present position for the sweep", held, VH_N_MOTORS);

    for (int i = 0; i < VH_N_MOTORS; i++) {
        uint8_t id = (uint8_t)VH_MOTOR_IDS[i];
        int j = joint_for_motor(id, side_right);
        vh_motor_cal_t *m = &s_cal.motors[i];
        m->dir = (j >= 0) ? default_dir_for_joint(j, side_right) : 1;
        if (flipped[i]) {
            m->dir = (int8_t)(-m->dir);
            m->flags |= VH_CAL_F_FLIPPED;
        }

        /* One motor at a time, at low torque, per CLAUDE.md's hardware rules.
         * The others stay held (see above); this one drops to probe torque. */
        ft_set_torque_limit(id, CAL_TORQUE_LIMIT);
        ft_set_speed_acc(id, CAL_SPEED, CAL_ACC);
        ft_write_u8(id, ADDR_TORQUE_ENABLE, 1);
        vTaskDelay(pdMS_TO_TICKS(20));

        uint16_t a = 0, b = 0;
        bool bound_a = false, bound_b = false;
        bool noload_a = false, noload_b = false;
        bool ok_a = probe_stop(id, -1, &a, &bound_a, &noload_a);
        bool ok_b = ok_a && probe_stop(id, +1, &b, &bound_b, &noload_b);

        if (!ok_a || !ok_b) {
            m->flags |= VH_CAL_F_DISABLED;
            disabled++;
            ESP_LOGW(TAG, "motor %u: could not be measured, marked disabled", id);
            ft_write_u8(id, ADDR_TORQUE_ENABLE, 0);
            continue;
        }

        uint16_t crept_lo = a < b ? a : b;
        uint16_t crept_hi = a < b ? b : a;

        /* Verify both ends from a standoff in the middle of the crept span,
         * at pose torque, and keep where the motor actually settles. */
        uint16_t standoff = (uint16_t)(((int)crept_lo + (int)crept_hi) / 2);
        uint16_t v_lo = crept_lo, v_hi = crept_hi;
        verify_status_t s_lo = verify_endpoint(id, crept_lo, standoff, &v_lo);
        verify_status_t s_hi = verify_endpoint(id, crept_hi, standoff, &v_hi);
        ESP_LOGI(TAG, "motor %u: verify lo %u -> %u (%s), hi %u -> %u (%s)",
                 id, (unsigned)crept_lo, (unsigned)v_lo, verify_name(s_lo),
                 (unsigned)crept_hi, (unsigned)v_hi, verify_name(s_hi));

        bool unverified = (s_lo == VERIFY_TIMEOUT || s_lo == VERIFY_IO ||
                           s_hi == VERIFY_TIMEOUT || s_hi == VERIFY_IO);
        if (unverified) {
            /* Fail closed. A previous table for this motor is better than a
             * guess; no previous table means this motor sits out until a
             * human recalibrates it. */
            m->flags |= VH_CAL_F_UNVERIFIED;
            if (had_prev && !(prev[i].flags & VH_CAL_F_DISABLED)) {
                m->lo = prev[i].lo;
                m->hi = prev[i].hi;
                m->flags |= (uint8_t)(prev[i].flags & (VH_CAL_F_HIT_BOUND | VH_CAL_F_NO_LOAD));
                ESP_LOGW(TAG, "motor %u: endpoint unverified -- keeping previous table %u..%u",
                         id, (unsigned)m->lo, (unsigned)m->hi);
                measured++;
            } else {
                m->flags |= VH_CAL_F_DISABLED;
                disabled++;
                ESP_LOGW(TAG, "motor %u: endpoint unverified and no previous table -- disabled", id);
            }
            ft_write_u8(id, ADDR_TORQUE_ENABLE, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        uint16_t lo = v_lo < v_hi ? v_lo : v_hi;
        uint16_t hi = v_lo < v_hi ? v_hi : v_lo;

        int shrink = ((int)crept_hi - (int)crept_lo) - ((int)hi - (int)lo);
        if (shrink > 4 * CAL_STEP) {
            ESP_LOGW(TAG, "motor %u: crept span %d..%d but only %d..%d is reachable "
                          "at %d permille (%d ticks unreachable)",
                     id, (unsigned)crept_lo, (unsigned)crept_hi,
                     (unsigned)lo, (unsigned)hi, VH_TORQUE_LIMIT_ARMED, shrink);
        }

        if (bound_a || bound_b) {
            m->flags |= VH_CAL_F_HIT_BOUND;
            if (lo < TICK_MIN + CAL_BOUND_INSET) {
                ESP_LOGW(TAG, "motor %u: low end %u is at the encoder edge -- inset to %u",
                         id, (unsigned)lo, (unsigned)(TICK_MIN + CAL_BOUND_INSET));
                lo = TICK_MIN + CAL_BOUND_INSET;
            }
            if (hi > TICK_MAX - CAL_BOUND_INSET) {
                ESP_LOGW(TAG, "motor %u: high end %u is at the encoder edge -- inset to %u",
                         id, (unsigned)hi, (unsigned)(TICK_MAX - CAL_BOUND_INSET));
                hi = TICK_MAX - CAL_BOUND_INSET;
            }
        }
        m->lo = lo;
        m->hi = hi;
        if (noload_a || noload_b) {
            /* Stopped advancing but never loaded up: slack tendon, soft
             * limit, or a joint that is not actually being driven. */
            m->flags |= VH_CAL_F_NO_LOAD;
            ESP_LOGW(TAG, "motor %u: an endpoint never loaded up -- span suspect", id);
        }

        int span = (int)hi - (int)lo;
        if (span < CAL_MIN_LIVE_SPAN) {
            m->flags |= VH_CAL_F_DISABLED;
            disabled++;
            ESP_LOGW(TAG, "motor %u: span only %d ticks (<%d) -- jammed or dead, disabled",
                     id, span, CAL_MIN_LIVE_SPAN);
        } else {
            measured++;
            ESP_LOGI(TAG, "motor %u: lo=%u hi=%u span=%d dir=%d%s",
                     id, lo, hi, span, m->dir,
                     (m->flags & VH_CAL_F_HIT_BOUND) ? " (HIT BOUND)" : "");
        }

        /* Park mid-span so nothing is left leaning on a stop, and keep it
         * held there (verify_endpoint left the limit at pose torque) so the
         * motors still to come are measured against a fixed posture rather
         * than a limp neighbour. A motor that could not be measured is let
         * go: a jam handed any goal at all just sits at stall current. */
        if (!(m->flags & VH_CAL_F_DISABLED)) {
            ft_write_goal(id, (uint16_t)((lo + hi) / 2));
            vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS * 2));
        } else {
            ft_write_u8(id, ADDR_TORQUE_ENABLE, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Everything limp again before returning, the way the hand was found. */
    for (int i = 0; i < VH_N_MOTORS; i++) {
        ft_write_u8((uint8_t)VH_MOTOR_IDS[i], ADDR_TORQUE_ENABLE, 0);
    }

    s_cal.complete = (measured > 0) ? 1 : 0;
    s_ready = s_cal.complete != 0;

    ESP_LOGW(TAG, "autocalibration done: %d measured, %d disabled", measured, disabled);
    if (s_ready) {
        cal_save();
    }
    return s_ready;
}
