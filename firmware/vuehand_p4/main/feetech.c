#include "feetech.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "transport.h"

static const char *TAG = "feetech";

/* Ported 1:1 from vuehand/feetech_packet.py -- see that file's docstring for
 * the protocol reference (SMS/STS Protocol 1). No CRC16 and no byte-stuffing
 * here, unlike Dynamixel 2.0 in dynamixel.c: Feetech's checksum is a plain
 * inverted sum, which is genuinely simpler to get right than the Dynamixel
 * path this was cloned from.
 *
 * This file no longer talks to the UART peripheral directly -- it goes
 * through transport.h, which has two interchangeable backends (raw GPIO/UART,
 * or a USB-host VCP adapter). See transport.h for how the backend is picked. */

#define FT_HDR0 0xFF
#define FT_HDR1 0xFF
#define FT_INST_PING 0x01
#define FT_INST_READ 0x02
#define FT_INST_WRITE 0x03
#define FT_INST_SYNC_WRITE 0x83
#define FT_BROADCAST 0xFE

/* SMS/STS memory map -- matches vuehand/feetech_packet.py's ADDR_* constants.
 * Only what servo.c currently needs; add more here (mirroring the Python
 * side) if servo.c grows to use them, e.g. ADDR_TORQUE_LIMIT for the same
 * thermal back-off vuehand.hand_link does on the host side. */
#define ADDR_TORQUE_ENABLE 40
#define ADDR_GOAL_POSITION 42
#define ADDR_PRESENT_POSITION 56  /* u16 -- matches vuehand/feetech_packet.py */
#define ADDR_PRESENT_LOAD 60      /* u16, sign bit 10 -- ditto */
#define ADDR_ACC 41               /* u8, x100 ticks/s^2 */
#define ADDR_GOAL_SPEED 46        /* u16, ticks/s */
#define ADDR_TORQUE_LIMIT 48      /* u16, per-mille of max torque */
#define ADDR_PRESENT_TEMP 63      /* u8, degrees C */
#define ADDR_PRESENT_CURRENT 69   /* u16, sign bit 15, 6.5 mA/LSB */
#define ADDR_MOVING 66            /* u8, nonzero while travelling */

/* One bus, now several callers: the JSON command loop in main.c and the
 * thermal monitor task in servo.c both talk to these servos. A read is a
 * send-then-wait-for-reply *transaction* -- if another task's write lands in
 * the middle, the reply gets attributed to the wrong request. Recursive
 * because the public helpers below legitimately call each other
 * (ft_set_speed_acc -> ft_write_u8/u16). */
static SemaphoreHandle_t s_bus_mux = NULL;

#define BUS_LOCK()                                     \
    do {                                               \
        if (s_bus_mux) {                               \
            xSemaphoreTakeRecursive(s_bus_mux, portMAX_DELAY); \
        }                                              \
    } while (0)

#define BUS_UNLOCK()                                   \
    do {                                               \
        if (s_bus_mux) {                               \
            xSemaphoreGiveRecursive(s_bus_mux);        \
        }                                              \
    } while (0)

static uint8_t checksum(const uint8_t *body, size_t n)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < n; i++) {
        sum += body[i];
    }
    return (uint8_t)(~sum & 0xFF);
}

static bool tx_raw(const uint8_t *pkt, size_t n)
{
    ft_flush_rx();
    bool ok = transport_write(pkt, n);
    ft_flush_rx();
    return ok;
}

/* Minimum gap between consecutive *addressed* packets. Non-broadcast Feetech
 * instructions elicit a status reply, and on a half-duplex bus that reply
 * collides with whatever goes out next -- silently dropping it. Enforced here
 * rather than at each call site, because forgetting it is subtle and costly:
 * it first showed up as `relax` leaving motors torqued, then again as a
 * single-motor goal write being ignored while the 40-packet pose ramp looked
 * fine. Broadcast SYNC_WRITE is exempt: nothing replies to it. */
#define FT_MIN_GAP_MS 5

static void pace_addressed(uint8_t id)
{
    static TickType_t s_last_tx = 0;
    if (id == FT_BROADCAST) {
        return;
    }
    TickType_t now = xTaskGetTickCount();
    TickType_t since = now - s_last_tx;
    TickType_t gap = pdMS_TO_TICKS(FT_MIN_GAP_MS);
    if (s_last_tx != 0 && since < gap) {
        vTaskDelay(gap - since);
    }
    s_last_tx = xTaskGetTickCount();
}

/* expect_reply=false (the original behavior): flush any stray bytes both
 * before and after sending, since nothing reads a response. expect_reply=true
 * (new, for ft_read_bytes): flush only *before* sending -- flushing after
 * would throw away the very response we're about to wait for. */
static bool send_packet_ex(uint8_t id, uint8_t inst, const uint8_t *params, uint8_t nparams,
                            bool expect_reply)
{
    uint8_t raw[256];
    if ((size_t)nparams + 6 > sizeof raw) {
        return false;
    }
    size_t n = 0;
    raw[n++] = FT_HDR0;
    raw[n++] = FT_HDR1;
    raw[n++] = id;
    raw[n++] = (uint8_t)(nparams + 2); /* length = param count + 2, per feetech_packet.py */
    raw[n++] = inst;
    if (nparams) {
        memcpy(&raw[n], params, nparams);
        n += nparams;
    }
    /* checksum covers everything after the 2-byte header, before the checksum itself */
    raw[n] = checksum(&raw[2], n - 2);
    n += 1;
    pace_addressed(id);
    if (expect_reply) {
        ft_flush_rx();
        return transport_write(raw, n);
    }
    return tx_raw(raw, n);
}

static bool send_packet(uint8_t id, uint8_t inst, const uint8_t *params, uint8_t nparams)
{
    return send_packet_ex(id, inst, params, nparams, false);
}

void ft_flush_rx(void)
{
    transport_flush_rx();
}

void ft_init(int uart_num, int tx_gpio, int rx_gpio, int rts_gpio, int baud)
{
    if (!s_bus_mux) {
        s_bus_mux = xSemaphoreCreateRecursiveMutex();
    }
    transport_uart_cfg_t cfg = {
        .uart_num = uart_num,
        .tx_gpio = tx_gpio,
        .rx_gpio = rx_gpio,
        .rts_gpio = rts_gpio,
        .baud = baud,
    };
    if (!transport_init(&cfg)) {
        /* Logged by the transport backend itself; keep going rather than
         * hard-fail, matching the existing fire-and-forget design (nothing
         * here reads back a response to confirm success either way). */
    }
}

bool ft_ping(uint8_t id)
{
    BUS_LOCK();
    bool ok = send_packet(id, FT_INST_PING, NULL, 0);
    BUS_UNLOCK();
    return ok;
}

bool ft_write_u8(uint8_t id, uint8_t addr, uint8_t value)
{
    uint8_t p[2] = { addr, value };
    BUS_LOCK();
    bool ok = send_packet(id, FT_INST_WRITE, p, 2);
    BUS_UNLOCK();
    return ok;
}

bool ft_write_u16(uint8_t id, uint8_t addr, uint16_t value)
{
    uint8_t p[3] = { addr, (uint8_t)(value & 0xFF), (uint8_t)(value >> 8) };
    BUS_LOCK();
    bool ok = send_packet(id, FT_INST_WRITE, p, 3);
    BUS_UNLOCK();
    return ok;
}

bool ft_sync_write_goal(const uint8_t *ids, const int32_t *ticks, int n)
{
    if (n <= 0 || n > 17) {
        return false;
    }
    uint8_t params[2 + 17 * 3];
    params[0] = ADDR_GOAL_POSITION;
    params[1] = 2; /* bytes per motor: u16 goal position */
    size_t o = 2;
    for (int i = 0; i < n; i++) {
        int32_t t = ticks[i];
        if (t < 0) t = 0;
        if (t > 4095) t = 4095; /* matches feetech_packet.py's sync_write_goal_position clamp */
        params[o++] = ids[i];
        params[o++] = (uint8_t)(t & 0xFF);
        params[o++] = (uint8_t)((t >> 8) & 0xFF);
    }
    BUS_LOCK();
    bool ok = send_packet(FT_BROADCAST, FT_INST_SYNC_WRITE, params, (uint8_t)o);
    BUS_UNLOCK();
    return ok;
}

/* Looks for a complete, checksum-valid status packet for `id` anywhere in
 * buf[0..got). Status packet layout: FF FF ID LEN ERR PARAMS... CHECKSUM,
 * where LEN counts ERR + PARAMS + CHECKSUM, so the whole thing is LEN+4 bytes
 * from the first header byte.
 *
 * Returns true once one is found and its params are copied out. A truncated
 * packet returns false without consuming anything, so the caller can read
 * more and try again -- that is what makes parse-as-you-go work. */
/* Every status reply carries the servo's own error byte at offset 4 -- right
 * after the length, before the params. find_status() has been stepping over
 * it since this file was written, which means the servos have been reporting
 * their own faults on every single read and nothing has ever looked.
 *
 * The bit meanings are NOT asserted here. Protocol 1's heritage suggests
 * voltage / angle-sensor / overheat / current / overload / instruction, but
 * that ordering has not been checked against the SCServo datasheet for these
 * particular units, and guessing at a bit mapping is exactly the sort of
 * confident-and-wrong that has cost this project time already. So the raw
 * byte is what gets stored and reported; whoever decodes it should verify the
 * mapping first. A nonzero value means the servo is unhappy, which is
 * actionable on its own. */
#define FT_MAX_TRACKED_ID 32
static uint8_t s_status_last[FT_MAX_TRACKED_ID];  /* most recent reply */
static uint8_t s_status_seen[FT_MAX_TRACKED_ID];  /* sticky OR since clear */

static void note_status(uint8_t id, uint8_t status)
{
    if (id >= FT_MAX_TRACKED_ID) {
        return;
    }
    s_status_last[id] = status;
    uint8_t fresh = (uint8_t)(status & ~s_status_seen[id]);
    s_status_seen[id] |= status;
    if (fresh) {
        /* Only on a bit that hasn't been seen for this motor yet, so a
         * permanently faulted servo doesn't flood the console. */
        ESP_LOGW(TAG, "motor %u reports status 0x%02X (new bits 0x%02X)",
                 id, status, fresh);
    }
}

uint8_t ft_status_last(uint8_t id)
{
    return id < FT_MAX_TRACKED_ID ? s_status_last[id] : 0;
}

uint8_t ft_status_seen(uint8_t id)
{
    return id < FT_MAX_TRACKED_ID ? s_status_seen[id] : 0;
}

void ft_status_clear(void)
{
    memset(s_status_seen, 0, sizeof s_status_seen);
    memset(s_status_last, 0, sizeof s_status_last);
}

static bool find_status(const uint8_t *buf, size_t got, uint8_t id, uint8_t nreq, uint8_t *out)
{
    for (size_t i = 0; i + 3 < got; i++) {
        if (buf[i] != FT_HDR0 || buf[i + 1] != FT_HDR1) {
            continue;
        }
        uint8_t rid = buf[i + 2];
        uint8_t length = buf[i + 3];
        size_t nparams = length >= 2 ? (size_t)(length - 2) : 0;
        size_t total = 4 + (size_t)length;
        if (i + total > got) {
            return false; /* real header, reply still arriving -- read more */
        }
        if (checksum(&buf[i + 2], 3 + nparams) != buf[i + total - 1]) {
            continue; /* corrupt or a false FF FF -- keep scanning */
        }
        if (rid != id) {
            continue; /* someone else's reply (or a stale one) */
        }
        note_status(rid, buf[i + 4]);
        size_t copy_n = nreq < nparams ? nreq : nparams;
        memcpy(out, &buf[i + 5], copy_n);
        return true;
    }
    return false;
}

/* Total read budget stays ~20ms, but it is spent in short slices with a parse
 * attempt after each one, so a reply that lands in under a millisecond (the
 * normal case) returns immediately instead of waiting out the clock. The old
 * version asked for one 20ms read and therefore cost 20ms per call every
 * time -- fine for the 17 reads at boot, ruinous for a calibration sweep that
 * polls each motor repeatedly. */
#define FT_READ_SLICE_MS 2
#define FT_READ_SLICES 10

bool ft_read_bytes(uint8_t id, uint8_t addr, uint8_t nreq, uint8_t *out)
{
    uint8_t p[2] = { addr, nreq };
    bool ok = false;

    BUS_LOCK(); /* send + reply must stay one atomic transaction */
    if (send_packet_ex(id, FT_INST_READ, p, 2, true)) {
        uint8_t buf[64];
        size_t got = 0;
        for (int slice = 0; slice < FT_READ_SLICES && got < sizeof buf; slice++) {
            size_t n = transport_read(buf + got, sizeof buf - got, FT_READ_SLICE_MS);
            if (n == 0) {
                continue; /* nothing yet -- keep waiting out the budget */
            }
            got += n;
            if (find_status(buf, got, id, nreq, out)) {
                ok = true;
                break;
            }
        }
    }
    BUS_UNLOCK();
    return ok;
}

bool ft_read_u16(uint8_t id, uint8_t addr, uint16_t *out_value)
{
    uint8_t raw[2];
    if (!ft_read_bytes(id, addr, 2, raw)) {
        return false;
    }
    *out_value = (uint16_t)(raw[0] | (raw[1] << 8));
    return true;
}

bool ft_read_present_position(uint8_t id, uint16_t *out_ticks)
{
    return ft_read_u16(id, ADDR_PRESENT_POSITION, out_ticks);
}

bool ft_transport_ready(void)
{
    return transport_is_ready();
}

bool ft_set_speed_acc(uint8_t id, uint16_t speed, uint8_t acc)
{
    bool ok = ft_write_u8(id, ADDR_ACC, acc);
    return ft_write_u16(id, ADDR_GOAL_SPEED, speed) && ok;
}

bool ft_read_torque_enable(uint8_t id, uint8_t *out_on)
{
    return ft_read_bytes(id, ADDR_TORQUE_ENABLE, 1, out_on);
}

bool ft_read_temp(uint8_t id, uint8_t *out_celsius)
{
    return ft_read_bytes(id, ADDR_PRESENT_TEMP, 1, out_celsius);
}

bool ft_set_torque_limit(uint8_t id, uint16_t permille)
{
    if (permille > 1000) {
        permille = 1000;
    }
    return ft_write_u16(id, ADDR_TORQUE_LIMIT, permille);
}

bool ft_write_goal(uint8_t id, uint16_t ticks)
{
    if (ticks > 4095) {
        ticks = 4095;
    }
    return ft_write_u16(id, ADDR_GOAL_POSITION, ticks);
}

bool ft_read_load(uint8_t id, int16_t *out_permille)
{
    uint16_t raw;
    if (!ft_read_u16(id, ADDR_PRESENT_LOAD, &raw)) {
        return false;
    }
    /* Sign-magnitude: magnitude in the LOW 10 BITS, direction in bit 10.
     * Masking with ~(1<<10) (as feetech_packet.sign_magnitude does) leaves
     * bits 11..15 in the magnitude -- harmless on every value observed so
     * far, but wrong in principle. Masked properly here.
     *
     * Critically: load and TORQUE_LIMIT share the same 0..1000 per-mille
     * scale, so load can never exceed the cap currently written. Any
     * threshold above the active torque limit is unreachable by
     * construction -- which is exactly how CAL_LOAD_STALL=500 at 150 permille
     * probe torque became dead code. */
    *out_permille = (raw & (1u << 10)) ? (int16_t)(-(int16_t)(raw & 0x3FF))
                                       : (int16_t)(raw & 0x3FF);
    return true;
}

bool ft_torque_enable(uint8_t id, bool on)
{
    return ft_write_u8(id, ADDR_TORQUE_ENABLE, on ? 1 : 0);
}

bool ft_read_current(uint8_t id, int32_t *out_ma)
{
    uint16_t raw;
    if (!ft_read_u16(id, ADDR_PRESENT_CURRENT, &raw)) {
        return false;
    }
    /* Sign at bit 15, magnitude below it -- same sign-magnitude form as load
     * and present position, per feetech_packet.sign_magnitude(). 6.5 mA/LSB
     * done as *13/2 to stay in integer arithmetic. */
    uint16_t mask = 1u << 15;
    int32_t mag = (int32_t)(raw & ~mask & 0xFFFF);
    int32_t ma = (mag * 13) / 2;
    *out_ma = (raw & mask) ? -ma : ma;
    return true;
}

bool ft_read_protection_block(uint8_t id, uint8_t *out24)
{
    /* One READ of the whole 13..36 range rather than six separate packets:
     * fewer addressed transactions, and no pacing to get wrong. */
    return ft_read_bytes(id, 13, 24, out24);
}

bool ft_read_moving(uint8_t id, bool *out_moving)
{
    uint8_t v = 0;
    if (!ft_read_bytes(id, ADDR_MOVING, 1, &v)) {
        return false;
    }
    *out_moving = (v != 0);
    return true;
}
