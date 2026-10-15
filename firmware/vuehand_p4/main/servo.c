#include "servo.h"

#include <math.h>
#include <string.h>

#include "feetech.h"
#include "motorcal.h"
#include "driver/temperature_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "servo";
static int s_right = 1;
static float s_current[VH_N_JOINTS];

#define ADDR_TORQUE_ENABLE 40

/* Ticks/s and x100-ticks/s^2 applied to every live motor whenever torque
 * comes on. Feetech servos default to unlimited speed/acceleration (register
 * value 0 -- NOT "stopped"), which is why the first real move looked like a
 * violent snap instead of a smooth motion. Matches vuehand/urt2_link.py's
 * LIVE_SPEED/LIVE_ACC, the proven-working reference implementation. */
#define LIVE_SPEED 2200
#define LIVE_ACC 80

/* Thermal protection, ported from the Python side that already learned this
 * the hard way (vuehand/touch.py's WARN_TEMP/CUT_TEMP/THERMAL_FLOOR_SCALE and
 * hand_link.py's IMITATE_TORQUE_LIMIT/_apply_thermal_throttle). Holding a pose
 * against tendon tension climbs roughly 30 C/hr, and the firmware previously
 * had no torque ceiling, no temperature reads, and no cutoff at all -- so an
 * armed hand left alone would cook until the servos' own ~70 C shutdown.
 *
 * Below VH_WARN_TEMP the limit is VH_TORQUE_LIMIT_ARMED. From there it ramps
 * linearly down to VH_THERMAL_FLOOR_SCALE of that by VH_CUT_TEMP, at which
 * point torque is cut outright and arming is refused until things cool back
 * under VH_WARN_TEMP. */
#define VH_WARN_TEMP 55
#define VH_CUT_TEMP 65
#define VH_THERMAL_FLOOR_SCALE 0.35f
/* Servo temperature moves slowly (~30 C/hr holding a pose), and the torque
 * ceiling caps how fast a stall can heat one, so there's no reason to poll
 * hard -- 2s was needless bus traffic and console noise. */
/* Temperature lags current, and current is the runaway signal -- so poll hard
 * while something is actually being held and idle along while limp. */
#define VH_THERMAL_POLL_ARMED_MS 500
#define VH_THERMAL_POLL_IDLE_MS 5000
#define VH_LIMIT_HYSTERESIS 10        /* only rewrite when it moves this much */

/* Serialises whole arm/disarm/limit-update *sequences*, not individual
 * packets. feetech.c's bus mutex makes each register write atomic, which is
 * not enough: servo_enable_torque() is a 17-motor sequence, and a thermal cut
 * landing in the middle of one would interleave per-motor and leave the hand
 * half-torqued with s_armed set by whichever finished last. Recursive because
 * servo_relax() -> servo_enable_torque() nests. */
static SemaphoreHandle_t s_seq_mux = NULL;

#define SEQ_LOCK()                                          \
    do {                                                    \
        if (s_seq_mux) {                                    \
            xSemaphoreTakeRecursive(s_seq_mux, portMAX_DELAY); \
        }                                                   \
    } while (0)

#define SEQ_UNLOCK()                                        \
    do {                                                    \
        if (s_seq_mux) {                                     \
            xSemaphoreGiveRecursive(s_seq_mux);             \
        }                                                   \
    } while (0)

/* Feetech STS ticks increase the opposite way from the Dynamixel-style
 * absolute ticks joint_to_ticks() produces -- matches urt2_link.TICK_SIGN. */
#define TICK_SIGN (-1)
#define TICK_MIN 0
#define TICK_MAX 4095

/* Uncalibrated fallback, ported from urt2_link.goal_targets(): mock-ratio
 * ticks are only ever used as a *delta*, applied relative to where the
 * servos actually sat when torque came on. Writing them as absolute goals
 * (what this file did before) commands roughly the bottom 11% of the servo's
 * 0..4095 travel -- i.e. yanks every tendon toward a hard stop -- because
 * VH_RATIO yields only ~4.56 ticks/deg. Replaced wholesale once real
 * per-motor calibration (data/motor_cal.json equivalent) lands. */
static uint16_t s_home_ticks[VH_N_MOTORS];  /* real present position at torque-on */
static int32_t s_home_cmd[VH_N_MOTORS];     /* mock-ratio ticks of the anchor pose */
static bool s_have_home = false;
static bool s_have_anchor = false;

static bool s_armed = false;
static uint8_t s_temp[VH_N_MOTORS];        /* last good Present Temperature, 0 = unknown */
static int32_t s_ma[VH_N_MOTORS];          /* last good Present Current, mA */
static TickType_t s_hold_deadline = 0;     /* auto-relax time, 0 = disabled */
static uint8_t s_prev_hottest = 0;         /* previous sweep, for a rising check */

/* How long the hand may stay armed with no new command before relaxing
 * itself. A fist holds every curl tendon against its stop; leaving one up
 * indefinitely is how motors get cooked. */
#define VH_HOLD_TIMEOUT_MS 30000
static uint16_t s_limit[VH_N_MOTORS];      /* torque limit currently written, per-mille */

/* Grasp force profiles. See the commentary in servo.h: these set *holding*
 * torque only. Reach torque is fixed at VH_TORQUE_LIMIT_ARMED because that
 * is the value motorcal verified every endpoint against.
 *
 * The numbers come from the Python side's hard-won range: ~150 permille is
 * what you want a hand doing near a person's fingers, ~300 is enough to keep
 * hold of a corn dog without deforming it, and 500+ is where these servos
 * start destroying themselves against a stop. "firm" sits under that with a
 * deliberately short leash. */
static const vh_grasp_profile_t s_profiles[] = {
    { "soft",   120, 60000 },
    { "medium", 220, VH_HOLD_TIMEOUT_MS },
    { "firm",   400, 10000 },
};
#define VH_N_PROFILES (sizeof s_profiles / sizeof s_profiles[0])
#define VH_GRASP_DEFAULT 1  /* medium */

/* A hard squeeze on an already-warm hand is how the last overheat started,
 * so anything above medium is refused above this temperature. */
#define VH_GRASP_HARD_MAX_TEMP 50

static const char *s_grasp_name = "medium";
static uint16_t s_hold_limit = 220;
static uint32_t s_hold_ms = VH_HOLD_TIMEOUT_MS;
/* While now < s_reach_until, motors run at reach torque so they can actually
 * travel; after it they settle to s_hold_limit. 0 = not moving. */
static TickType_t s_reach_until = 0;
static uint8_t s_hottest = 0;
static bool s_thermal_cut = false;
static uint8_t s_cut_temp = 0;
static uint8_t s_cut_id = 0;

/* The P4's own die temperature -- separate concern from the servos, but the
 * board runs unattended (and often on whatever surface is handy, which is
 * usually a poor one thermally), so it's worth having eyes on. Reported
 * alongside the servo temps; nothing is throttled off it. */
static temperature_sensor_handle_t s_tsens = NULL;
static float s_board_temp = 0.0f;
#define VH_BOARD_WARN_TEMP 80.0f

/* Which finger each motor belongs to, so a hot joint throttles its whole
 * digit rather than falling behind its neighbours mid-grasp -- the same
 * per-finger-group rule hand_link._apply_thermal_throttle() uses. Derived
 * from VH_JOINT_NAMES's fixed ordering: wrist, thumb x4, then index /
 * middle / ring / pinky x3 each. */
#define VH_N_GROUPS 6
static int s_group_of_motor[VH_N_MOTORS];
static uint8_t s_group_hot[VH_N_GROUPS];   /* hottest temp per finger group */

static int group_of_joint(int j)
{
    if (j == 0) return 0;            /* wrist */
    if (j <= 4) return 1;            /* thumb_cmc/abd/mcp/dip */
    return 2 + (j - 5) / 3;          /* index, middle, ring, pinky */
}

static int motor_index(int motor_id)
{
    for (int i = 0; i < VH_N_MOTORS; i++) {
        if (VH_MOTOR_IDS[i] == motor_id) {
            return i;
        }
    }
    return -1;
}

static float thermal_scale(uint8_t temp);

/* Rewrites every live motor's torque limit to `base` scaled by its finger
 * group's thermal derate. Caller must hold SEQ_LOCK.
 *
 * The hysteresis check means a burst of poses inside one reach window costs
 * zero extra packets -- only the first pose after a settle pays the 17
 * paced writes (~85 ms). That matters if streamed teleop lands on top of
 * this later. */
static void apply_limits_locked(uint16_t base)
{
    for (int i = 0; i < VH_N_MOTORS; i++) {
        if (motorcal_motor_disabled(VH_MOTOR_IDS[i])) {
            continue;
        }
        float scale = thermal_scale(s_group_hot[s_group_of_motor[i]]);
        uint16_t target = (uint16_t)lroundf((float)base * scale);
        int delta = (int)target - (int)s_limit[i];
        if (delta < 0) delta = -delta;
        if (delta < VH_LIMIT_HYSTERESIS) {
            continue;
        }
        if (ft_set_torque_limit((uint8_t)VH_MOTOR_IDS[i], target)) {
            s_limit[i] = target;
        }
    }
}

/* Gentle mode: close at the holding ceiling instead of reach torque.
 *
 * Reach torque (300) exists so poses arrive at endpoints that calibration
 * verified at that torque. A *grasp* does not need to arrive: on an object
 * the fingers stop on the object, on air they stop where friction stops them.
 * What reach torque adds to a grasp is 300 permille on whatever is in the
 * palm for 1.2 s before anyone knows what it is -- and when a person is
 * handing things over, that can be a finger. Two outside reviews of the
 * passive loop (2026-09-03) called this the primary pinch hazard. So a grasp
 * may ask to be gentle, and then the whole close runs at the profile's hold
 * ceiling. The cost is a fist that stops a little short on air; the hold
 * state afterwards is identical. */
static bool s_gentle = false;
static uint16_t s_gentle_limit = VH_GENTLE_DEFAULT;

/* The gentle close still has to MOVE the fingers: at the soft hold ceiling
 * (120) tendon friction stops the MCPs 300-440 ticks short of a fist and a can
 * sits in a shallow grip and slides out (measured 2026-09-03). The
 * calibration probes move every joint at 150; 180 is a margin above that and
 * still well under reach torque. Never below the hold ceiling, never above
 * reach torque. */
static uint16_t reach_limit(void)
{
    if (!s_gentle) {
        return VH_TORQUE_LIMIT_ARMED;
    }
    uint16_t g = s_gentle_limit;
    if (g < s_hold_limit) g = s_hold_limit;
    if (g > VH_TORQUE_LIMIT_ARMED) g = VH_TORQUE_LIMIT_ARMED;
    return g;
}

void servo_set_gentle(bool on) { s_gentle = on; }
bool servo_gentle(void) { return s_gentle; }
void servo_set_gentle_limit(uint16_t permille)
{
    if (permille < 40) permille = 40;
    if (permille > VH_TORQUE_LIMIT_ARMED) permille = VH_TORQUE_LIMIT_ARMED;
    s_gentle_limit = permille;
}
uint16_t servo_gentle_limit(void) { return s_gentle_limit; }

/* Which ceiling applies right now: reach while a pose is in flight, the
 * grasp profile's holding torque once it has settled. */
static uint16_t limit_base(void)
{
    if (s_reach_until != 0 &&
        (int32_t)(xTaskGetTickCount() - s_reach_until) < 0) {
        return reach_limit();
    }
    return s_hold_limit;
}

/* Called immediately before a goal write: raise to reach torque so the
 * motors can travel, open the reach window, and push the auto-relax
 * deadline out by this profile's leash. */
static void begin_move(void)
{
    if (!s_armed) {
        return;
    }
    SEQ_LOCK();
    s_reach_until = xTaskGetTickCount() + pdMS_TO_TICKS(VH_REACH_MS);
    apply_limits_locked(reach_limit());
    s_hold_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(s_hold_ms);
    SEQ_UNLOCK();
}

static int32_t joint_to_ticks(int joint_index, float deg)
{
    const int *map = s_right ? VH_RIGHT_MAP : VH_LEFT_MAP;
    int signed_id = map[joint_index];
    int sign = signed_id < 0 ? -1 : 1;
    float lo = VH_ROM_LO[joint_index];
    float hi = VH_ROM_HI[joint_index];
    if (deg < lo) deg = lo;
    if (deg > hi) deg = hi;
    float travel = sign < 0 ? (hi - deg) : (deg - lo);
    float rad = travel * VH_RATIO;
    return (int32_t)lroundf(rad * (float)VH_TICKS_PER_REV / (2.0f * 3.14159265f));
}

static void write_pose(const float *deg17)
{
    const int *map = s_right ? VH_RIGHT_MAP : VH_LEFT_MAP;

    /* Calibrated: absolute goals straight off each motor's measured
     * stop-to-stop span, which is what gives the hand its full range.
     * Mirrors urt2_link.goal_targets() taking the absolute branch when
     * motor_cal is complete. */
    if (motorcal_ready()) {
        uint8_t ids[VH_N_MOTORS];
        int32_t goals[VH_N_MOTORS];
        int n = 0;
        for (int j = 0; j < VH_N_JOINTS; j++) {
            int signed_id = map[j];
            int motor = signed_id < 0 ? -signed_id : signed_id;
            if (motor_index(motor) < 0) {
                continue;
            }
            if (motorcal_motor_disabled(motor)) {
                continue; /* jammed or dead: never command it */
            }
            ids[n] = (uint8_t)motor;
            goals[n] = (int32_t)motorcal_ticks_for_joint(j, deg17[j], s_right);
            n++;
        }
        if (n > 0) {
            begin_move();   /* reach torque up BEFORE the goal, not after */
            ft_sync_write_goal(ids, goals, n);
        }
        memcpy(s_current, deg17, sizeof s_current);
        return;
    }

    /* Uncalibrated fallback: relative moves off a real home reading. */
    int32_t cmd[VH_N_MOTORS] = {0};
    for (int j = 0; j < VH_N_JOINTS; j++) {
        int signed_id = map[j];
        int motor = signed_id < 0 ? -signed_id : signed_id;
        int mi = motor_index(motor);
        if (mi >= 0) {
            cmd[mi] = joint_to_ticks(j, deg17[j]);
        }
    }

    if (!s_have_home) {
        /* Never write absolute mock-ratio ticks -- see the comment on
         * s_home_ticks above. Torque has to come on first (that is what
         * reads the home reference). */
        ESP_LOGW(TAG, "no home reference yet -- suppressing goal write (enable torque first)");
        memcpy(s_current, deg17, sizeof s_current);
        return;
    }

    if (!s_have_anchor) {
        /* Anchor on this pose and deliberately do not move, matching
         * urt2_link.write_joints_live()'s "don't jump on that frame". */
        memcpy(s_home_cmd, cmd, sizeof s_home_cmd);
        s_have_anchor = true;
        memcpy(s_current, deg17, sizeof s_current);
        return;
    }

    uint8_t ids[VH_N_MOTORS];
    int32_t goals[VH_N_MOTORS];
    for (int i = 0; i < VH_N_MOTORS; i++) {
        int32_t t = (int32_t)s_home_ticks[i] + TICK_SIGN * (cmd[i] - s_home_cmd[i]);
        if (t < TICK_MIN) t = TICK_MIN;
        if (t > TICK_MAX) t = TICK_MAX;
        ids[i] = (uint8_t)VH_MOTOR_IDS[i];
        goals[i] = t;
    }
    begin_move();
    ft_sync_write_goal(ids, goals, VH_N_MOTORS);
    memcpy(s_current, deg17, sizeof s_current);
}

void servo_init(int side_right)
{
    if (!s_seq_mux) {
        s_seq_mux = xSemaphoreCreateRecursiveMutex();
    }
    s_right = side_right;
    for (int j = 0; j < VH_N_JOINTS; j++) {
        s_current[j] = 0.5f * (VH_ROM_LO[j] + VH_ROM_HI[j]);
    }
    for (int i = 0; i < VH_N_MOTORS; i++) {
        s_group_of_motor[i] = 0;
        s_limit[i] = 0;
        s_temp[i] = 0;
    }
    {
        const int *map = side_right ? VH_RIGHT_MAP : VH_LEFT_MAP;
        for (int j = 0; j < VH_N_JOINTS; j++) {
            int signed_id = map[j];
            int motor = signed_id < 0 ? -signed_id : signed_id;
            int mi = motor_index(motor);
            if (mi >= 0) {
                s_group_of_motor[mi] = group_of_joint(j);
            }
        }
    }
    /* TODO(feetech-cal): joint_to_ticks() still uses the generic
     * VH_RATIO/VH_TICKS_PER_REV formula, not this hand's measured per-motor
     * min/max/dir from data/motor_cal.json. That's the same "mock ratio"
     * CLAUDE.md warns doesn't cover full ROM. write_pose() therefore only
     * uses it as a relative delta off a real home reading (see TICK_SIGN
     * above), which is what urt2_link does uncalibrated. Real calibration
     * still needs a codegen step (extend vuehand/firmware_gen.py) or an
     * on-device autocal persisted to NVS. */
    /* Pacing between addressed packets is enforced in feetech.c now, so no
     * explicit delay here. */
    for (int i = 0; i < VH_N_MOTORS; i++) {
        ft_write_u8((uint8_t)VH_MOTOR_IDS[i], ADDR_TORQUE_ENABLE, 0);
    }

    /* Read the torque-enable register back rather than asserting success:
     * the old log line here printed a hardcoded motor count and claimed
     * "torque off" without ever checking either, which is exactly the kind
     * of thing that hid the boot-race bug for so long. */
    int replied = 0;
    int still_on = 0;
    for (int i = 0; i < VH_N_MOTORS; i++) {
        uint8_t on = 0;
        if (ft_read_torque_enable((uint8_t)VH_MOTOR_IDS[i], &on)) {
            replied++;
            if (on) {
                still_on++;
            }
        }
    }
    if (replied == VH_N_MOTORS && still_on == 0) {
        ESP_LOGI(TAG, "torque off confirmed on all %d motors", VH_N_MOTORS);
    } else {
        ESP_LOGW(TAG, "boot torque-off unconfirmed: %d/%d motors replied, %d still torqued",
                 replied, VH_N_MOTORS, still_on);
    }
}

int servo_side_right(void) { return s_right; }

void servo_enable_torque(bool on)
{
    SEQ_LOCK();
    if (on && s_thermal_cut) {
        ESP_LOGW(TAG, "refusing to arm: thermal cut latched (motor %d hit %d C) -- "
                      "let it cool below %d C", s_cut_id, s_cut_temp, VH_WARN_TEMP);
        SEQ_UNLOCK();
        return;
    }
    if (on) {
        /* Capture where the servos actually are, in raw ticks, before arming
         * -- this is the reference every subsequent goal is computed
         * relative to. Matches urt2_link.torque(True) doing
         * self.home_ticks = self.read_positions(); self.home_cmd = None. */
        int got = 0;
        for (int i = 0; i < VH_N_MOTORS; i++) {
            uint16_t ticks;
            if (ft_read_present_position((uint8_t)VH_MOTOR_IDS[i], &ticks)) {
                s_home_ticks[i] = ticks;
                got++;
            } else {
                s_home_ticks[i] = 2048; /* urt2_link's home_ticks.get(mid, 2048) */
            }
        }
        s_have_home = (got > 0);
        s_have_anchor = false;
        ESP_LOGI(TAG, "home ticks read from %d/%d motors", got, VH_N_MOTORS);
    } else {
        s_have_home = false;
        s_have_anchor = false;
    }

    /* Matches servo_init()'s loop just above (and the known-working Python
     * urt2_link.py, which waits ~8ms per motor for a status-packet reply):
     * back-to-back packets on this half-duplex bus with zero gap can collide
     * with the servos' own replies and get dropped. */
    for (int i = 0; i < VH_N_MOTORS; i++) {
        uint8_t id = (uint8_t)VH_MOTOR_IDS[i];
        if (on && motorcal_motor_disabled(VH_MOTOR_IDS[i])) {
            /* Leave a known-dead motor limp rather than parking it at stall
             * current -- urt2_link.torque() skips disabled ids the same way.
             * Torque-off still goes to everything. */
            continue;
        }
        if (on) {
            /* Park the goal at where this motor already is, BEFORE energising
             * it. Enabling torque makes a Feetech servo hold whatever is in
             * its goal register, which may be stale from a previous session
             * or a previous pose -- and we have already watched a dropped
             * goal write masquerade as a dead motor, so a stale goal on arm
             * is a real shape of bug, not a hypothetical one. */
            ft_write_goal(id, s_home_ticks[i]);
            ft_set_speed_acc(id, LIVE_SPEED, LIVE_ACC);
            /* Cap holding torque. Nothing wrote this register before, so
             * motors ran at whatever ceiling was last left in SRAM. Arming
             * gets the *holding* ceiling, not reach: goal == present
             * position here, so nothing has anywhere to travel. */
            ft_set_torque_limit(id, s_hold_limit);
            s_limit[i] = s_hold_limit;
        }
        ft_write_u8(id, ADDR_TORQUE_ENABLE, on ? 1 : 0);
    }
    s_armed = on;
    s_reach_until = 0;
    s_hold_deadline = on ? (xTaskGetTickCount() + pdMS_TO_TICKS(s_hold_ms)) : 0;
    SEQ_UNLOCK();
}

static float thermal_scale(uint8_t temp)
{
    /* Mirrors vuehand/touch.py thermal_scale(). temp 0 means "never read" --
     * treat as cool rather than throttling on missing data. */
    if (temp == 0 || temp < VH_WARN_TEMP) {
        return 1.0f;
    }
    if (temp >= VH_CUT_TEMP) {
        return VH_THERMAL_FLOOR_SCALE;
    }
    float frac = (float)(temp - VH_WARN_TEMP) / (float)(VH_CUT_TEMP - VH_WARN_TEMP);
    return 1.0f - frac * (1.0f - VH_THERMAL_FLOOR_SCALE);
}

static void thermal_task(void *arg)
{
    (void)arg;
    while (true) {
        /* Hard while holding something, idle while limp. Motor 10 went
         * 55->64 C in 2.5 minutes holding a fist; a 5s poll is far too coarse
         * to react to that, and temperature already lags current. */
        vTaskDelay(pdMS_TO_TICKS(s_armed ? VH_THERMAL_POLL_ARMED_MS
                                         : VH_THERMAL_POLL_IDLE_MS));

        /* Auto-relax: every overheat this session traces to somebody
         * forgetting to relax the hand after a test, including me. A held
         * pose is not a resting state -- it is every tendon against its stop.
         * Nobody should have to remember this. */
        if (s_armed && s_hold_deadline != 0 &&
            (int32_t)(xTaskGetTickCount() - s_hold_deadline) >= 0) {
            ESP_LOGW(TAG, "hold timeout: %lu s at %s grasp with no new command -- relaxing",
                     (unsigned long)(s_hold_ms / 1000), s_grasp_name);
            servo_enable_torque(false);
            continue;
        }

        if (s_tsens) {
            float board;
            if (temperature_sensor_get_celsius(s_tsens, &board) == ESP_OK) {
                s_board_temp = board;
                if (board >= VH_BOARD_WARN_TEMP) {
                    ESP_LOGW(TAG, "P4 die at %.1f C -- check airflow around the board", board);
                }
            }
        }

        if (!ft_transport_ready()) {
            continue;
        }

        /* One read per motor, each taking the bus lock on its own so a pose
         * in progress can interleave between them instead of stalling for
         * the whole sweep. */
        uint8_t hottest = 0;
        uint8_t hottest_id = 0;
        uint8_t group_hot[VH_N_GROUPS] = {0};
        for (int i = 0; i < VH_N_MOTORS; i++) {
            uint8_t t;
            int32_t ma;
            if (ft_read_current((uint8_t)VH_MOTOR_IDS[i], &ma)) {
                s_ma[i] = ma;
            }
            if (!ft_read_temp((uint8_t)VH_MOTOR_IDS[i], &t)) {
                continue;
            }
            s_temp[i] = t;
            int g = s_group_of_motor[i];
            if (t > group_hot[g]) {
                group_hot[g] = t;
            }
            if (t > hottest) {
                hottest = t;
                hottest_id = (uint8_t)VH_MOTOR_IDS[i];
            }
        }

        memcpy(s_group_hot, group_hot, sizeof s_group_hot);
        s_hottest = hottest;

        if (hottest == 0) {
            continue; /* nothing answered -- don't act on no data */
        }

        if (hottest >= VH_CUT_TEMP) {
            /* Below Feetech's own ~70 C shutdown: go limp before the servo
             * does it for us. Same reasoning as hand_link.py's hard cut. */
            if (!s_thermal_cut) {
                s_thermal_cut = true;
                s_cut_temp = hottest;
                s_cut_id = hottest_id;
                ESP_LOGE(TAG, "THERMAL CUT: motor %d at %d C -- dropping torque, "
                              "hand will not arm until under %d C",
                         hottest_id, hottest, VH_WARN_TEMP);
                servo_enable_torque(false);
            }
            continue;
        }

        if (s_thermal_cut) {
            if (hottest < VH_WARN_TEMP) {
                s_thermal_cut = false;
                ESP_LOGW(TAG, "thermal cut cleared, hottest now %d C -- safe to arm",
                         hottest);
            }
            continue; /* stay latched while still above warn */
        }

        if (!s_armed) {
            s_prev_hottest = hottest;
            continue; /* nothing being held, nothing to throttle */
        }

        /* The 0.35x floor is a brake, not an equilibrium. Motor 10 climbed
         * 55 -> 64 C over 2.5 minutes while already being throttled, because
         * scaling the cap does not stop I2R in a stalled hold. So if
         * temperature is still rising inside the warn band, let go rather
         * than ride the floor down into a cut. */
        if (hottest >= VH_WARN_TEMP && s_prev_hottest != 0 && hottest > s_prev_hottest) {
            ESP_LOGW(TAG, "still climbing at %d C (was %d) on motor %d -- releasing "
                          "instead of riding the throttle floor",
                     hottest, s_prev_hottest, hottest_id);
            s_prev_hottest = hottest;
            servo_enable_torque(false);
            continue;
        }
        s_prev_hottest = hottest;

        /* This is also what performs the reach -> hold settle: once the
         * reach window closes limit_base() returns the grasp profile's
         * holding torque and the same hysteresis check writes it down. */
        SEQ_LOCK(); /* must not interleave with an arm/disarm sequence */
        apply_limits_locked(limit_base());
        SEQ_UNLOCK();

        if (hottest >= VH_WARN_TEMP) {
            ESP_LOGW(TAG, "thermal throttle active: hottest motor %d at %d C",
                     hottest_id, hottest);
        }
    }
}

static int set_grasp_locked(const char *name, uint16_t hold, uint32_t hold_ms)
{
    if (hold > VH_GRASP_MAX_HOLD) hold = VH_GRASP_MAX_HOLD;
    if (hold < 40) hold = 40;

    /* Refuse a hard squeeze on a hand that is already warm. The overheat
     * this session came from holding a pose the motors could not reach; the
     * next most likely one is squeezing hard on top of residual heat. */
    if (hold > s_profiles[VH_GRASP_DEFAULT].hold_permille &&
        s_hottest != 0 && s_hottest >= VH_GRASP_HARD_MAX_TEMP) {
        ESP_LOGW(TAG, "refusing %s grasp: hottest motor already %d C (limit %d C)",
                 name, s_hottest, VH_GRASP_HARD_MAX_TEMP);
        return -2;
    }

    SEQ_LOCK();
    s_grasp_name = name;
    s_hold_limit = hold;
    s_hold_ms = hold_ms;
    if (s_armed) {
        /* Apply now rather than waiting up to a poll interval: if somebody
         * asks for a softer grip because the hand is squeezing something it
         * shouldn't, half a second is half a second too long. */
        apply_limits_locked(limit_base());
        s_hold_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(s_hold_ms);
    }
    SEQ_UNLOCK();
    ESP_LOGI(TAG, "grasp %s: hold %d permille, reach %d permille, leash %lu s",
             s_grasp_name, s_hold_limit, VH_TORQUE_LIMIT_ARMED,
             (unsigned long)(s_hold_ms / 1000));
    return 0;
}

int servo_set_grasp(const char *name)
{
    if (!name) {
        return -1;
    }
    for (size_t i = 0; i < VH_N_PROFILES; i++) {
        if (strcmp(name, s_profiles[i].name) == 0) {
            return set_grasp_locked(s_profiles[i].name,
                                    s_profiles[i].hold_permille,
                                    s_profiles[i].hold_ms);
        }
    }
    return -1;
}

int servo_set_grasp_permille(uint16_t permille)
{
    /* Inherit the leash of the nearest named profile at or above this
     * force, so a raw override can't get a gentle timeout for a hard grip. */
    uint32_t leash = s_profiles[VH_N_PROFILES - 1].hold_ms;
    const char *name = "custom";
    for (size_t i = 0; i < VH_N_PROFILES; i++) {
        if (permille <= s_profiles[i].hold_permille) {
            leash = s_profiles[i].hold_ms;
            break;
        }
    }
    return set_grasp_locked(name, permille, leash);
}

const char *servo_grasp_name(void) { return s_grasp_name; }
uint16_t servo_grasp_hold(void) { return s_hold_limit; }
uint32_t servo_grasp_hold_ms(void) { return s_hold_ms; }
uint8_t servo_hottest_temp(void) { return s_hottest; }

void servo_thermal_start(void)
{
    static bool started = false;
    if (started) {
        return;
    }
    /* Must match one of the chip's actual measurement ranges -- asking for
     * -10..100 spans several and fails with "Cannot select the correct
     * range". -10..80 is a real one and covers everything this board will
     * legitimately see. */
    temperature_sensor_config_t tsens_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&tsens_cfg, &s_tsens) != ESP_OK ||
        temperature_sensor_enable(s_tsens) != ESP_OK) {
        ESP_LOGW(TAG, "P4 die temperature sensor unavailable -- servo temps still monitored");
        s_tsens = NULL;
    }

    if (xTaskCreate(thermal_task, "thermal", 4096, NULL, 4, NULL) == pdTRUE) {
        started = true;
        ESP_LOGI(TAG, "thermal monitor running (warn %d C, cut %d C, reach %d / "
                      "hold %d permille at %s grasp)",
                 VH_WARN_TEMP, VH_CUT_TEMP, VH_TORQUE_LIMIT_ARMED,
                 s_hold_limit, s_grasp_name);
    } else {
        ESP_LOGE(TAG, "failed to start thermal monitor -- hand has NO overheat protection");
    }
}

void servo_temps_copy(uint8_t *out17, uint16_t *out_limits, int32_t *out_ma)
{
    if (out17) {
        memcpy(out17, s_temp, sizeof s_temp);
    }
    if (out_limits) {
        memcpy(out_limits, s_limit, sizeof s_limit);
    }
    if (out_ma) {
        memcpy(out_ma, s_ma, sizeof s_ma);
    }
}

float servo_board_temp(void)
{
    return s_board_temp;
}

bool servo_thermal_cut(uint8_t *out_hottest_temp, uint8_t *out_hottest_id)
{
    if (out_hottest_temp) {
        *out_hottest_temp = s_cut_temp;
    }
    if (out_hottest_id) {
        *out_hottest_id = s_cut_id;
    }
    return s_thermal_cut;
}

void servo_relax(void)
{
    servo_enable_torque(false);
}

void servo_current_copy(float *out17)
{
    memcpy(out17, s_current, sizeof s_current);
}

void servo_set_vec(const float *deg17, int t_ms)
{
    if (t_ms < 20) {
        write_pose(deg17);
        return;
    }
    int steps = t_ms / 10;
    if (steps < 2) steps = 2;
    float start[VH_N_JOINTS];
    memcpy(start, s_current, sizeof start);
    for (int s = 1; s <= steps; s++) {
        float a = (float)s / (float)steps;
        float mid[VH_N_JOINTS];
        for (int j = 0; j < VH_N_JOINTS; j++) {
            mid[j] = start[j] * (1.0f - a) + deg17[j] * a;
        }
        write_pose(mid);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
