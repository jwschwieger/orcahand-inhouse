#include <math.h>
#include <stdio.h>
#include <string.h>

#include "feetech.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "infer.h"
#include "jsonish.h"
#include "motorcal.h"
#include "nvs_flash.h"
#include "servo.h"

static const char *TAG = "vuehand";

/* Confirmed against a photo of the actual Waveshare ESP32-P4-Module-DEV-KIT
 * header in hand: GPIO11/12/13 (the old Dynamixel/ORCA-v2 dev defaults) are
 * NOT broken out on this board's header at all -- they're reserved
 * internally (PSRAM/flash bus). Swapped to GPIO20/21/8, which ARE present on
 * this board's header row. The bare "TXD"/"RXD" pins on that same header are
 * wired to the onboard USB-UART console chip -- do not reuse those for the
 * servo bus. Still verify the hand-side wire colors (orange/yellow) map to
 * TX/RX correctly before trusting this fully. Override at build time
 * (-D VH_SERVO_TX=<n> etc.) if your board differs. */
/* Which hand is on the bus. 1 = right (VH_RIGHT_MAP), 0 = left (VH_LEFT_MAP);
 * set from the VH_HAND_SIDE CMake cache variable (main/CMakeLists.txt). The
 * two maps route the same 17 joint names onto different motor IDs and signs,
 * so a right build on a left hand curls the wrong fingers and calibrates the
 * wrong direction on every mismatched motor. */
#ifndef VH_HAND_RIGHT
#define VH_HAND_RIGHT 1
#endif
#ifndef VH_SERVO_UART
#define VH_SERVO_UART 1
#endif
#ifndef VH_SERVO_TX
#define VH_SERVO_TX 20
#endif
#ifndef VH_SERVO_RX
#define VH_SERVO_RX 21
#endif
#ifndef VH_SERVO_RTS
#define VH_SERVO_RTS 8
#endif

static void reply(const char *json)
{
    printf("%s\n", json);
    fflush(stdout);
}

static void play_frames(float frames[][VH_N_JOINTS], int n, int duration_ms)
{
    if (n <= 0) {
        return;
    }
    servo_enable_torque(true);
    int slice = duration_ms / n;
    if (slice < 40) slice = 40;
    for (int i = 0; i < n; i++) {
        servo_set_vec(frames[i], slice);
    }
}

static void handle_line(char *line)
{
    char cmd[32] = {0};
    if (!json_get_str(line, "cmd", cmd, sizeof cmd)) {
        reply("{\"ok\":false,\"error\":\"no cmd\"}");
        return;
    }
    if (strcmp(cmd, "ping") == 0) {
        reply("{\"ok\":true,\"device\":\"vuehand-p4\"}");
        return;
    }
    if (strcmp(cmd, "status") == 0) {
        /* Pure local read of s_current -- no bus I/O, so this one doesn't
         * need the transport-ready gate below. */
        float cur[VH_N_JOINTS];
        servo_current_copy(cur);
        static char buf[512];
        int n = snprintf(buf, sizeof buf, "{\"ok\":true,\"vec\":[");
        for (int i = 0; i < VH_N_JOINTS; i++) {
            n += snprintf(buf + n, sizeof buf - n, "%s%.2f", i ? "," : "", cur[i]);
        }
        snprintf(buf + n, sizeof buf - n, "]}");
        reply(buf);
        return;
    }
    if (!ft_transport_ready()) {
        /* Real fix for a boot-race bug found in review: every command below
         * this point touches the servo bus, and on the USB backend the
         * adapter may not be open yet even seconds after boot. Previously
         * these would silently no-op (transport_write() returns false while
         * s_vcp is null) and still reply {"ok":true}. */
        reply("{\"ok\":false,\"error\":\"servo bus not ready yet\"}");
        return;
    }
    if (strcmp(cmd, "relax") == 0) {
        servo_relax();
        reply("{\"ok\":true,\"torque\":false}");
        return;
    }
    if (strcmp(cmd, "stop") == 0) {
        servo_relax();
        reply("{\"ok\":true,\"stopped\":true}");
        return;
    }
    if (strcmp(cmd, "torque") == 0) {
        bool on = true;
        json_get_bool(line, "on", &on);
        uint8_t cut_temp = 0, cut_id = 0;
        if (on && servo_thermal_cut(&cut_temp, &cut_id)) {
            static char buf[160];
            snprintf(buf, sizeof buf,
                     "{\"ok\":false,\"error\":\"thermal cut\",\"motor\":%u,\"temp\":%u}",
                     (unsigned)cut_id, (unsigned)cut_temp);
            reply(buf);
            return;
        }
        servo_enable_torque(on);
        reply(on ? "{\"ok\":true,\"torque\":true}" : "{\"ok\":true,\"torque\":false}");
        return;
    }
    if (strcmp(cmd, "temps") == 0) {
        uint8_t temps[VH_N_MOTORS];
        uint16_t limits[VH_N_MOTORS];
        int32_t mas[VH_N_MOTORS];
        servo_temps_copy(temps, limits, mas);
        uint8_t cut_temp = 0, cut_id = 0;
        bool cut = servo_thermal_cut(&cut_temp, &cut_id);
        static char buf[768];
        int n = snprintf(buf, sizeof buf, "{\"ok\":true,\"temp_c\":[");
        for (int i = 0; i < VH_N_MOTORS; i++) {
            n += snprintf(buf + n, sizeof buf - n, "%s%u", i ? "," : "", (unsigned)temps[i]);
        }
        n += snprintf(buf + n, sizeof buf - n, "],\"limit_permille\":[");
        for (int i = 0; i < VH_N_MOTORS; i++) {
            n += snprintf(buf + n, sizeof buf - n, "%s%u", i ? "," : "", (unsigned)limits[i]);
        }
        n += snprintf(buf + n, sizeof buf - n, "],\"ma\":[");
        for (int i = 0; i < VH_N_MOTORS; i++) {
            n += snprintf(buf + n, sizeof buf - n, "%s%ld", i ? "," : "", (long)mas[i]);
        }
        /* Integer, deliberately: %f drags in newlib's _dtoa_r, which is a
         * stack hog and is what crashed this handler the first time. */
        float board = servo_board_temp();
        int board_c = (int)(board < 0.0f ? board - 0.5f : board + 0.5f);
        snprintf(buf + n, sizeof buf - n,
                 "],\"board_c\":%d,\"cut\":%s,\"cut_motor\":%u,\"cut_temp\":%u,"
                 "\"grasp\":\"%s\",\"hold_permille\":%u,\"reach_permille\":%u,"
                 "\"leash_s\":%lu}",
                 board_c, cut ? "true" : "false",
                 (unsigned)cut_id, (unsigned)cut_temp,
                 servo_grasp_name(), (unsigned)servo_grasp_hold(),
                 (unsigned)VH_TORQUE_LIMIT_ARMED,
                 (unsigned long)(servo_grasp_hold_ms() / 1000));
        reply(buf);
        return;
    }
    if (strcmp(cmd, "grasp") == 0) {
        /* Grasp force as a *holding* torque limit rather than a pose: the
         * shape of a fist doesn't change how hard it squeezes, the ceiling
         * on how much the motor may push against the position error an
         * object creates does. Reach torque stays fixed at the value
         * calibration verified its endpoints against, so a soft grasp still
         * arrives where it was told to go instead of stalling short. */
        static char gbuf[224];
        char name[16];
        int permille = 0;
        int rc = 0;
        bool have_name = json_get_str(line, "name", name, sizeof name);
        bool have_raw = json_get_int(line, "permille", &permille);
        bool gentle = false;
        if (json_get_bool(line, "gentle", &gentle)) {
            servo_set_gentle(gentle);
        }
        int gentle_permille = 0;
        if (json_get_int(line, "gentle_permille", &gentle_permille)) {
            servo_set_gentle_limit((uint16_t)(gentle_permille < 0 ? 0 : gentle_permille));
            if (!have_name && !have_raw) {
                snprintf(gbuf, sizeof gbuf, "{\"ok\":true,\"gentle_permille\":%u,\"gentle\":%s}",
                         (unsigned)servo_gentle_limit(), servo_gentle() ? "true" : "false");
                reply(gbuf);
                return;
            }
        }
        if (have_name) {
            rc = servo_set_grasp(name);
        } else if (have_raw) {
            if (permille < 0) permille = 0;
            rc = servo_set_grasp_permille((uint16_t)permille);
        }
        if (rc == -1) {
            reply("{\"ok\":false,\"error\":\"unknown grasp; use soft, medium, firm, "
                  "or {\\\"permille\\\":N}\"}");
            return;
        }
        if (rc == -2) {
            snprintf(gbuf, sizeof gbuf,
                     "{\"ok\":false,\"error\":\"too hot for a hard grasp\","
                     "\"hottest_c\":%u,\"grasp\":\"%s\",\"hold_permille\":%u}",
                     (unsigned)servo_hottest_temp(), servo_grasp_name(),
                     (unsigned)servo_grasp_hold());
            reply(gbuf);
            return;
        }
        snprintf(gbuf, sizeof gbuf,
                 "{\"ok\":true,\"grasp\":\"%s\",\"hold_permille\":%u,"
                 "\"reach_permille\":%u,\"leash_s\":%lu,\"hottest_c\":%u}",
                 servo_grasp_name(), (unsigned)servo_grasp_hold(),
                 (unsigned)(servo_gentle() ? servo_gentle_limit() : VH_TORQUE_LIMIT_ARMED),
                 (unsigned long)(servo_grasp_hold_ms() / 1000),
                 (unsigned)servo_hottest_temp());
        reply(gbuf);
        return;
    }
    if (strcmp(cmd, "faults") == 0) {
        /* Every status reply carries the servo's own error byte, and the
         * packet parser had been stepping straight over it since this
         * firmware was written -- so all 17 motors have been reporting their
         * own faults on every read and nothing ever looked.
         *
         * Bits are reported RAW and undecoded on purpose. Protocol 1's
         * lineage suggests voltage / angle-sensor / overheat / current /
         * overload / instruction, but that mapping has not been checked
         * against the SCServo datasheet for these units, and a confidently
         * wrong bit label is worse than a hex number. Nonzero means the
         * servo is unhappy, which is actionable by itself.
         *
         * "last" is the most recent reply; "seen" is a sticky OR since boot
         * or the last {"cmd":"faults","clear":true}. The sticky one is the
         * useful column -- a fault that flickers during a grasp is gone by
         * the time anyone types a command.
         *
         * Motor 4 is the immediate question this exists to answer: span 0 on
         * three calibration runs, coldest motor on the hand, yet it answers
         * every register read. If it is reporting an angle-sensor or overload
         * fault, that distinguishes a dead winding from a mechanical jam
         * without taking anything apart. */
        bool clear = false;
        json_get_bool(line, "clear", &clear);
        if (clear) {
            ft_status_clear();
            reply("{\"ok\":true,\"cleared\":true}");
            return;
        }
        static char fbuf[512];
        int n = snprintf(fbuf, sizeof fbuf, "{\"ok\":true,\"last\":[");
        for (int i = 0; i < VH_N_MOTORS; i++) {
            n += snprintf(fbuf + n, sizeof fbuf - n, "%s%u", i ? "," : "",
                          (unsigned)ft_status_last((uint8_t)VH_MOTOR_IDS[i]));
        }
        n += snprintf(fbuf + n, sizeof fbuf - n, "],\"seen\":[");
        for (int i = 0; i < VH_N_MOTORS; i++) {
            n += snprintf(fbuf + n, sizeof fbuf - n, "%s%u", i ? "," : "",
                          (unsigned)ft_status_seen((uint8_t)VH_MOTOR_IDS[i]));
        }
        snprintf(fbuf + n, sizeof fbuf - n, "]}");
        reply(fbuf);
        return;
    }
    if (strcmp(cmd, "joints") == 0) {
        /* The shape the hand is ACTUALLY in, in degrees, versus the shape it
         * was last told to make. {"cmd":"status"} reports the latter -- it
         * echoes the commanded vector and has no idea whether the hand got
         * there. These two diverge exactly when something is in the way,
         * which is most of the interesting cases, so a controller that only
         * ever reads back its own commands is flying blind.
         *
         * Only meaningful with a calibration loaded: the tick -> degree map
         * is the inverse of the one poses are built from, so an uncalibrated
         * hand has no frame to report an angle in and this says so rather
         * than inventing one from the mock VH_RATIO. */
        if (!motorcal_ready()) {
            reply("{\"ok\":false,\"error\":\"no calibration; run {\\\"cmd\\\":\\\"calibrate\\\"}\"}");
            return;
        }
        static char jbuf[768];
        int n = snprintf(jbuf, sizeof jbuf, "{\"ok\":true,\"deg\":[");
        int right = servo_side_right();
        const int *map = right ? VH_RIGHT_MAP : VH_LEFT_MAP;
        for (int j = 0; j < VH_N_JOINTS; j++) {
            int motor = map[j] < 0 ? -map[j] : map[j];
            uint16_t ticks = 0;
            float deg = 0.0f;
            bool ok = ft_read_present_position((uint8_t)motor, &ticks) &&
                      motorcal_deg_for_joint(j, ticks, right, &deg);
            if (!ok) {
                n += snprintf(jbuf + n, sizeof jbuf - n, "%snull", j ? "," : "");
                continue;
            }
            /* Fixed point by hand: %f drags in newlib's _dtoa_r, which is a
             * stack hog and is what crashed this handler the first time. */
            int d10 = (int)lroundf(deg * 10.0f);
            const char *sg = d10 < 0 ? "-" : "";
            int a = d10 < 0 ? -d10 : d10;
            n += snprintf(jbuf + n, sizeof jbuf - n, "%s%s%d.%d",
                          j ? "," : "", sg, a / 10, a % 10);
        }
        snprintf(jbuf + n, sizeof jbuf - n, "]}");
        reply(jbuf);
        return;
    }
    if (strcmp(cmd, "grip") == 0) {
        /* Did the grasp close on something, or on air?
         *
         * A pose command is open-loop: it says where to go, never whether
         * the hand got there. The two registers that answer that are already
         * on the bus -- goal position vs present position gives the
         * positional error the object is creating, and Present Load gives
         * how hard the motor is pushing against it. A motor that arrived
         * (err ~0, low load) closed on nothing; one sitting at a large err
         * with load pinned near the holding ceiling has hold of something.
         *
         * This is the readback a higher layer needs to close a loop on, and
         * the reason it can trust the numbers is that the endpoints poses
         * are built from were verified reachable at reach torque -- before
         * that fix a large err meant "commanded somewhere impossible",
         * which is indistinguishable from a successful grip. */
        static char rbuf[768];
        int n = snprintf(rbuf, sizeof rbuf,
                         "{\"ok\":true,\"hold_permille\":%u,\"grasp\":\"%s\",\"err\":[",
                         (unsigned)servo_grasp_hold(), servo_grasp_name());
        int16_t loads[VH_N_MOTORS];
        bool have_load[VH_N_MOTORS];
        for (int i = 0; i < VH_N_MOTORS; i++) {
            uint8_t id = (uint8_t)VH_MOTOR_IDS[i];
            uint16_t goal = 0, pos = 0;
            bool ok = ft_read_u16(id, 42, &goal) && ft_read_present_position(id, &pos);
            if (ok) {
                n += snprintf(rbuf + n, sizeof rbuf - n, "%s%d", i ? "," : "",
                              (int)goal - (int)pos);
            } else {
                n += snprintf(rbuf + n, sizeof rbuf - n, "%snull", i ? "," : "");
            }
            have_load[i] = ft_read_load(id, &loads[i]);
        }
        n += snprintf(rbuf + n, sizeof rbuf - n, "],\"load_permille\":[");
        for (int i = 0; i < VH_N_MOTORS; i++) {
            if (have_load[i]) {
                n += snprintf(rbuf + n, sizeof rbuf - n, "%s%d", i ? "," : "", (int)loads[i]);
            } else {
                n += snprintf(rbuf + n, sizeof rbuf - n, "%snull", i ? "," : "");
            }
        }
        snprintf(rbuf + n, sizeof rbuf - n, "]}");
        reply(rbuf);
        return;
    }
    if (strcmp(cmd, "torque_status") == 0) {
        /* Real per-motor confirmation of ADDR_TORQUE_ENABLE, instead of
         * inferring torque state from a console reply or a hand-feel test. */
        char buf[256];
        int n = snprintf(buf, sizeof buf, "{\"ok\":true,\"torque\":[");
        for (int i = 0; i < VH_N_MOTORS; i++) {
            uint8_t on = 0;
            bool ok = ft_read_torque_enable((uint8_t)VH_MOTOR_IDS[i], &on);
            n += snprintf(buf + n, sizeof buf - n, "%s%s", i ? "," : "",
                          ok ? (on ? "1" : "0") : "null");
        }
        snprintf(buf + n, sizeof buf - n, "]}");
        reply(buf);
        return;
    }
    if (strcmp(cmd, "calibrate") == 0) {
        /* Physically drives every motor into both mechanical stops, one at a
         * time, at low torque. Blocking and slow -- the console will not
         * answer until it finishes. Explicit-only by design: see motorcal.h
         * on why this must not run at every boot. */
        reply("{\"ok\":true,\"starting\":\"calibrate\",\"warning\":\"hand will move, keep clear\"}");
        /* Go limp first. The sweep leaves every other motor in whatever
         * torque state it had, so calibrating from an armed pose meant two
         * things went wrong at once: the auto-relax leash fired mid-sweep and
         * dropped the motor under probe, and the thermal poll kept rewriting
         * the probe's 150 permille ceiling with the grasp's holding torque.
         * Relaxing also clears the leash, so nothing wakes up during the run. */
        servo_relax();
        bool ok = motorcal_run(1);
        vh_motor_cal_t cal[VH_N_MOTORS];
        motorcal_copy(cal);
        /* 17 motors x ~65 chars: 700 bytes truncated this reply after motor
         * 12 -- every calibration ever read back through this command lost
         * the pinky abduction and all four thumb motors. */
        static char buf[1400];
        int n = snprintf(buf, sizeof buf, "{\"ok\":%s,\"calibrated\":%s,\"motors\":[",
                         ok ? "true" : "false", motorcal_ready() ? "true" : "false");
        for (int i = 0; i < VH_N_MOTORS; i++) {
            n += snprintf(buf + n, sizeof buf - n,
                          "%s{\"id\":%d,\"lo\":%u,\"hi\":%u,\"span\":%d,\"dir\":%d,\"flags\":%u}",
                          i ? "," : "", VH_MOTOR_IDS[i], (unsigned)cal[i].lo,
                          (unsigned)cal[i].hi, (int)cal[i].hi - (int)cal[i].lo,
                          (int)cal[i].dir, (unsigned)cal[i].flags);
        }
        snprintf(buf + n, sizeof buf - n, "]}");
        reply(buf);
        return;
    }
    if (strcmp(cmd, "cal") == 0) {
        vh_motor_cal_t cal[VH_N_MOTORS];
        motorcal_copy(cal);
        static char buf[1400]; /* see "calibrate": 700 cut this off at motor 12 */
        int n = snprintf(buf, sizeof buf, "{\"ok\":true,\"calibrated\":%s,\"motors\":[",
                         motorcal_ready() ? "true" : "false");
        for (int i = 0; i < VH_N_MOTORS; i++) {
            n += snprintf(buf + n, sizeof buf - n,
                          "%s{\"id\":%d,\"lo\":%u,\"hi\":%u,\"span\":%d,\"dir\":%d,\"flags\":%u}",
                          i ? "," : "", VH_MOTOR_IDS[i], (unsigned)cal[i].lo,
                          (unsigned)cal[i].hi, (int)cal[i].hi - (int)cal[i].lo,
                          (int)cal[i].dir, (unsigned)cal[i].flags);
        }
        snprintf(buf + n, sizeof buf - n, "]}");
        reply(buf);
        return;
    }
    if (strcmp(cmd, "goto_tick") == 0) {
        /* Raw single-motor tick move, for establishing which end of a
         * measured span is flexion. Clamped to that motor's calibrated
         * [lo,hi] -- those ticks were physically reached during
         * autocalibration, so staying inside them cannot drive into
         * anything new. Probing torque limit, not the full armed limit. */
        int id = 1;
        int ticks = 0;
        json_get_int(line, "id", &id);
        if (!json_get_int(line, "ticks", &ticks)) {
            reply("{\"ok\":false,\"error\":\"need ticks\"}");
            return;
        }
        uint16_t lo = 0, hi = 0;
        int8_t dir = 0;
        if (!motorcal_span(id, &lo, &hi, &dir)) {
            reply("{\"ok\":false,\"error\":\"motor not calibrated\"}");
            return;
        }
        int clamped = ticks;
        if (clamped < (int)lo) clamped = (int)lo;
        if (clamped > (int)hi) clamped = (int)hi;

        uint16_t before = 0;
        int16_t load_before = 0;
        int32_t ma_before = 0;
        ft_read_present_position((uint8_t)id, &before);
        ft_read_load((uint8_t)id, &load_before);
        ft_read_current((uint8_t)id, &ma_before);
        ft_set_torque_limit((uint8_t)id, 150);
        ft_set_speed_acc((uint8_t)id, 600, 20);
        ft_torque_enable((uint8_t)id, true);
        ft_write_goal((uint8_t)id, (uint16_t)clamped);
        vTaskDelay(pdMS_TO_TICKS(900));
        uint16_t after = 0;
        int16_t load_after = 0;
        int32_t ma_after = 0;
        bool got = ft_read_present_position((uint8_t)id, &after);
        ft_read_load((uint8_t)id, &load_after);
        ft_read_current((uint8_t)id, &ma_after);

        /* Load/current at the commanded end is the thing that distinguishes a
         * real mechanical stop from unloaded winch travel on a slack tendon:
         * a real stop loads up, slack take-up doesn't. Reported rather than
         * acted on -- the stop detector stays as-is until we can see whether
         * a known-good motor's ends actually load. */
        static char buf[320];
        snprintf(buf, sizeof buf,
                 "{\"ok\":true,\"id\":%d,\"lo\":%u,\"hi\":%u,\"dir\":%d,"
                 "\"asked\":%d,\"sent\":%d,\"before\":%u,\"after\":%s%u,"
                 "\"load_before\":%d,\"load_after\":%d,"
                 "\"ma_before\":%ld,\"ma_after\":%ld,\"still_torqued\":true}",
                 id, (unsigned)lo, (unsigned)hi, (int)dir, ticks, clamped,
                 (unsigned)before, got ? "" : "-", (unsigned)after,
                 (int)load_before, (int)load_after,
                 (long)ma_before, (long)ma_after);
        reply(buf);
        return;
    }
    if (strcmp(cmd, "probe") == 0) {
        /* Is this motor trying and failing, or not trying at all?
         *
         * Built for motor 4 specifically. goto_tick can't answer it: it
         * clamps to the calibrated span, and a motor with span 0 has lo ==
         * hi, so the only reachable target is the tick it already sits on.
         * It also leaves torque on, which on a jammed motor is a stall with
         * nothing to release it -- the hold leash only arms via {"cmd":
         * "torque"} and the thermal throttle returns early when the hand
         * isn't armed, so only the 65 C hard cut would ever let go.
         *
         * This deliberately ignores the calibration and nudges a small fixed
         * displacement each way from wherever the shaft is, at a torque limit
         * well below the probing limit, then ALWAYS drops torque -- on every
         * exit path, including a motor that never answers.
         *
         * The discriminator is load and current, not position:
         *   moved            -> the calibration's span 0 was wrong
         *   stuck, load HIGH -> the motor is pushing and cannot move: a jam
         *   stuck, load ~0   -> the motor isn't even trying: open winding,
         *                       bad connector, or torque never engaged
         * Position alone can't separate the last two, which is why the first
         * three calibration runs left the question open. */
        int id = 0;
        if (!json_get_int(line, "id", &id) || id < 1 || id > 253) {
            reply("{\"ok\":false,\"error\":\"need id\"}");
            return;
        }
        uint16_t p0 = 0;
        if (!ft_read_present_position((uint8_t)id, &p0)) {
            reply("{\"ok\":false,\"error\":\"no position reply\"}");
            return;
        }

        const int NUDGE = 120;   /* ~10 deg of shaft, small on any joint */
        int up = (int)p0 + NUDGE;
        int down = (int)p0 - NUDGE;
        if (up > 4095) up = 4095;
        if (down < 0) down = 0;

        uint16_t pos[2] = {p0, p0};
        int16_t load[2] = {0, 0};
        int32_t ma[2] = {0, 0};
        int goals[2] = {up, down};

        ft_set_torque_limit((uint8_t)id, 100);
        ft_set_speed_acc((uint8_t)id, 300, 20);
        ft_torque_enable((uint8_t)id, true);
        for (int k = 0; k < 2; k++) {
            ft_write_goal((uint8_t)id, (uint16_t)goals[k]);
            vTaskDelay(pdMS_TO_TICKS(700));
            ft_read_present_position((uint8_t)id, &pos[k]);
            ft_read_load((uint8_t)id, &load[k]);
            ft_read_current((uint8_t)id, &ma[k]);
        }
        /* Back to where it started, then limp. Unconditional: a probe that
         * can leave a motor energised is not a probe, it is a hazard. */
        ft_write_goal((uint8_t)id, p0);
        vTaskDelay(pdMS_TO_TICKS(300));
        ft_torque_enable((uint8_t)id, false);

        static char pbuf[352];
        snprintf(pbuf, sizeof pbuf,
                 "{\"ok\":true,\"id\":%d,\"start\":%u,"
                 "\"asked_up\":%d,\"got_up\":%u,\"load_up\":%d,\"ma_up\":%ld,"
                 "\"asked_down\":%d,\"got_down\":%u,\"load_down\":%d,\"ma_down\":%ld,"
                 "\"moved\":%d,\"torque\":false}",
                 id, (unsigned)p0,
                 up, (unsigned)pos[0], (int)load[0], (long)ma[0],
                 down, (unsigned)pos[1], (int)load[1], (long)ma[1],
                 (int)((pos[0] > p0 ? pos[0] - p0 : p0 - pos[0]) +
                       (pos[1] > p0 ? pos[1] - p0 : p0 - pos[1])));
        reply(pbuf);
        return;
    }
    if (strcmp(cmd, "cal_flip") == 0) {
        /* dir is a mounting convention autocal cannot measure; this is the
         * same escape hatch MotorCal.flip() exists for. Omit "id" to flip
         * every motor. Re-saves to NVS. */
        int id = 0;
        json_get_int(line, "id", &id);
        bool ok = motorcal_flip(id);
        static char buf[120];
        snprintf(buf, sizeof buf,
                 ok ? "{\"ok\":true,\"flipped\":%d}"
                    : "{\"ok\":false,\"error\":\"no calibration to flip\",\"id\":%d}",
                 id);
        reply(buf);
        return;
    }
    if (strcmp(cmd, "protection") == 0) {
        /* Why did a servo on this hand reach 150 C without its own cutoff
         * firing? These are the registers that would explain it. Read only. */
        int id = 1;
        json_get_int(line, "id", &id);
        uint8_t blk[24] = {0};
        if (!ft_read_protection_block((uint8_t)id, blk)) {
            reply("{\"ok\":false,\"error\":\"no reply\"}");
            return;
        }
        static char buf[420];
        int n = snprintf(buf, sizeof buf,
                 "{\"ok\":true,\"id\":%d,\"max_temp_13\":%u,\"unloading_19\":%u,"
                 "\"prot_current_28\":%u,\"prot_torque_34\":%u,\"prot_time_35\":%u,"
                 "\"overload_torque_36\":%u,\"raw13_36\":[",
                 id, (unsigned)blk[0], (unsigned)blk[6],
                 (unsigned)(blk[15] | (blk[16] << 8)),
                 (unsigned)blk[21], (unsigned)blk[22], (unsigned)blk[23]);
        for (int i = 0; i < 24; i++) {
            n += snprintf(buf + n, sizeof buf - n, "%s%u", i ? "," : "", (unsigned)blk[i]);
        }
        snprintf(buf + n, sizeof buf - n, "]}");
        reply(buf);
        return;
    }
    if (strcmp(cmd, "cal_clear") == 0) {
        bool ok = motorcal_clear();
        reply(ok ? "{\"ok\":true,\"cleared\":true}"
                 : "{\"ok\":false,\"error\":\"could not clear NVS\"}");
        return;
    }
    if (strcmp(cmd, "read_pos") == 0) {
        /* First real bus *read* this firmware has ever done -- everything
         * before this was fire-and-forget writes. Standalone test command
         * for the new ft_read_present_position() path before building the
         * motor_cal autocal sweep on top of it. */
        int id = 1;
        json_get_int(line, "id", &id);
        uint16_t ticks = 0;
        bool ok = ft_read_present_position((uint8_t)id, &ticks);
        static char buf[96];
        if (ok) {
            snprintf(buf, sizeof buf, "{\"ok\":true,\"id\":%d,\"ticks\":%u}", id, (unsigned)ticks);
        } else {
            snprintf(buf, sizeof buf, "{\"ok\":false,\"id\":%d,\"error\":\"no reply\"}", id);
        }
        reply(buf);
        return;
    }

    int t_ms = 400;
    json_get_int(line, "t_ms", &t_ms);

    if (strcmp(cmd, "set_joints") == 0) {
        float vec[VH_N_JOINTS];
        int n = json_get_vec(line, "vec", vec, VH_N_JOINTS);
        if (n != VH_N_JOINTS) {
            reply("{\"ok\":false,\"error\":\"vec needs 17 floats\"}");
            return;
        }
        servo_enable_torque(true);
        servo_set_vec(vec, t_ms);
        reply("{\"ok\":true}");
        return;
    }

    char text[128] = {0};
    if (strcmp(cmd, "say") == 0) {
        if (!json_get_str(line, "text", text, sizeof text)) {
            reply("{\"ok\":false,\"error\":\"missing text\"}");
            return;
        }
    } else if (strcmp(cmd, "pose") == 0 || strcmp(cmd, "sequence") == 0) {
        if (!json_get_str(line, "name", text, sizeof text)) {
            reply("{\"ok\":false,\"error\":\"missing name\"}");
            return;
        }
        /* Optional: pick the grasp force in the same command as the pose,
         * so "close firmly" is one round trip instead of two. A refusal on
         * thermal grounds is not fatal here -- the pose still runs, just at
         * whatever force was already in effect. */
        {
            char g[16];
            if (json_get_str(line, "grasp", g, sizeof g)) {
                if (servo_set_grasp(g) == -1) {
                    reply("{\"ok\":false,\"error\":\"unknown grasp\"}");
                    return;
                }
            }
            bool gentle = false;
            if (json_get_bool(line, "gentle", &gentle)) {
                servo_set_gentle(gentle);
            }
        }
    } else {
        reply("{\"ok\":false,\"error\":\"unknown cmd\"}");
        return;
    }

    float cur[VH_N_JOINTS];
    servo_current_copy(cur);
    float frames[VH_N_KEYFRAMES][VH_N_JOINTS];
    int n_frames = 0;
    int duration = t_ms;
    vh_say(text, cur, frames, &n_frames, &duration);
    if (t_ms > 0) {
        duration = t_ms;
    }
    play_frames(frames, n_frames, duration);
    reply("{\"ok\":true,\"ran\":true}");
}

static void command_task(void *arg)
{
    (void)arg;
    char line[768];
    size_t n = 0;
    while (1) {
        int ch = getchar();
        if (ch == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (ch == '\n' || ch == '\r') {
            if (n > 0) {
                line[n] = 0;
                handle_line(line);
                n = 0;
            }
            continue;
        }
        if (n + 1 < sizeof line) {
            line[n++] = (char)ch;
        } else {
            n = 0; /* overlong line: drop it rather than truncate into a
                    * half-parsed command */
        }
    }
}

void app_main(void)
{
    /* The USB host and CDC-ACM layers are extremely chatty at DEBUG -- they
     * were invaluable while enumeration was broken, and unreadable now that
     * the thermal monitor polls every few seconds. Pinned quiet here as well
     * as in sdkconfig.defaults, so turning the global level back to DEBUG to
     * chase something else doesn't re-bury the console. */
    esp_log_level_set("cdc_acm", ESP_LOG_WARN);
    esp_log_level_set("cdc_acm_compliant", ESP_LOG_WARN);
    esp_log_level_set("USBH", ESP_LOG_WARN);
    esp_log_level_set("HUB", ESP_LOG_WARN);
    esp_log_level_set("ENUM", ESP_LOG_WARN);
    esp_log_level_set("FTDI", ESP_LOG_ERROR);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ft_init(VH_SERVO_UART, VH_SERVO_TX, VH_SERVO_RX, VH_SERVO_RTS, VH_BAUDRATE);

    /* Real fix for a boot-race bug found in review: on the USB backend the
     * adapter isn't open yet the instant ft_init() returns (device
     * acquisition happens in a background task), so servo_init()'s
     * torque-off writes were silently failing -- undermining the "always
     * boots limp" guarantee this whole function exists to provide. Wait
     * (bounded) for the transport to actually be usable first. */
    int waited_ms = 0;
    while (!ft_transport_ready() && waited_ms < 15000) {
        vTaskDelay(pdMS_TO_TICKS(200));
        waited_ms += 200;
    }
    if (!ft_transport_ready()) {
        ESP_LOGW(TAG, "transport still not ready after %dms -- servo_init()'s "
                      "torque-off will not reach real hardware yet", waited_ms);
    }
    servo_init(VH_HAND_RIGHT);
    ESP_LOGI(TAG, "hand side: %s (VH_HAND_SIDE build option)", VH_HAND_RIGHT ? "right" : "left");
    servo_thermal_start();
    if (motorcal_load()) {
        ESP_LOGI(TAG, "using measured per-motor calibration (full range available)");
    } else {
        ESP_LOGW(TAG, "no calibration stored -- poses are relative-only and cover a "
                      "fraction of the real range; run {\"cmd\":\"calibrate\"} once");
    }
    ESP_LOGI(TAG, "vuehand P4 ready — USB JSON commands, Feetech @ %d", VH_BAUDRATE);
    reply("{\"ok\":true,\"boot\":\"vuehand-p4\"}");

    /* Own task, own stack. app_main() runs on the main task, which ESP-IDF
     * gives ~3.5KB -- handle_line() builds JSON replies up to several hundred
     * bytes and (via status) formats floats, which overflowed it. Giving the
     * command loop 8KB removes the whole class of problem rather than
     * shaving individual call sites. */
    if (xTaskCreate(command_task, "cmd", 8192, NULL, 5, NULL) != pdTRUE) {
        ESP_LOGE(TAG, "failed to start command task -- no JSON console");
    }
}
