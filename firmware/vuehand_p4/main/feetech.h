#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Feetech SMS/STS bus protocol (FF FF ... checksum) -- what the ORCA Touch's
 * servos speak, over the P4's own UART instead of a URT-2 USB bridge. Mirrors
 * dynamixel.h's shape so servo.c can swap between the two with minimal changes.
 * Wire format ported 1:1 from vuehand/feetech_packet.py -- keep them in sync. */

void ft_init(int uart_num, int tx_gpio, int rx_gpio, int rts_gpio, int baud);
bool ft_ping(uint8_t id);
bool ft_write_u8(uint8_t id, uint8_t addr, uint8_t value);
bool ft_write_u16(uint8_t id, uint8_t addr, uint16_t value);
bool ft_sync_write_goal(const uint8_t *ids, const int32_t *ticks, int n);
void ft_flush_rx(void);

/* Real reads, added alongside the write-only fire-and-forget path above.
 * Sends a READ instruction and waits (up to a short fixed timeout) for that
 * servo's status-packet reply, verifying its checksum before trusting it.
 * Returns false on no/garbled reply -- same "log, don't hard-fail" spirit as
 * everything else in this file. */
bool ft_read_bytes(uint8_t id, uint8_t addr, uint8_t n, uint8_t *out);
bool ft_read_u16(uint8_t id, uint8_t addr, uint16_t *out_value);

/* The servo's own error byte, harvested from every status reply. Nonzero
 * means the servo is reporting a fault about itself. Bit meanings are
 * deliberately not decoded here -- see the comment in feetech.c. */
uint8_t ft_status_last(uint8_t id);   /* most recent reply from this motor */
uint8_t ft_status_seen(uint8_t id);   /* sticky OR of everything since clear */
void ft_status_clear(void);

/* Convenience wrapper -- the address this project actually needs read-back
 * for right now (motor_cal's stall detection). */
bool ft_read_present_position(uint8_t id, uint16_t *out_ticks);

/* True once the underlying transport has a genuinely usable link. Check
 * before sending anything that matters -- see transport.h's comment on the
 * boot-race bug this exists to catch. */
bool ft_transport_ready(void);

/* Sets goal speed (ticks/s) and acceleration (x100 ticks/s^2) on one motor.
 * Call whenever torque comes on -- Feetech servos default to unlimited speed/
 * acceleration (register value 0), which is what made the first real move
 * feel like a violent snap instead of a smooth motion. Values ported from
 * vuehand/urt2_link.py's LIVE_SPEED/LIVE_ACC, the proven-working reference. */
bool ft_set_speed_acc(uint8_t id, uint16_t speed, uint8_t acc);

/* Reads back ADDR_TORQUE_ENABLE directly -- a real confirmation that torque
 * is (or isn't) actually on for a given motor, instead of inferring it from
 * "the write didn't error" or a hand-feel test. */
bool ft_read_torque_enable(uint8_t id, uint8_t *out_on);

/* Present Temperature (register 63), in whole degrees C. The ORCA Touch's
 * servos shut themselves down around 70 C; servo.c's thermal monitor uses
 * this to back off well before that. */
bool ft_read_temp(uint8_t id, uint8_t *out_celsius);

/* Torque Limit (register 48), in per-mille of max torque (0..1000). Feetech
 * treats this as a ceiling, not a commanded torque. Nothing wrote it before,
 * so motors ran at whatever limit was last left in SRAM -- typically max. */
bool ft_set_torque_limit(uint8_t id, uint16_t permille);

/* Goal Position (register 42) for a single motor, in absolute ticks
 * (0..4095). Normal posing uses ft_sync_write_goal() for all 17 at once; this
 * is for calibration, which drives one motor at a time. */
bool ft_write_goal(uint8_t id, uint16_t ticks);

/* Present Load (register 60), decoded from Feetech's sign-magnitude form
 * (sign is bit 10), in per-mille of max torque. Calibration uses a high
 * |load| as a stall signal alongside lack of positional progress. */
bool ft_read_load(uint8_t id, int16_t *out_permille);

/* Torque Enable (register 40) for a single motor. servo.c does this in bulk;
 * this is for calibration and single-motor diagnostics. */
bool ft_torque_enable(uint8_t id, bool on);

/* Present Current (register 69), in milliamps, sign-magnitude with the sign
 * at bit 15 and 6.5 mA per LSB. Load (register 60) is a processed view of the
 * same thing; read both so a disagreement is visible rather than assumed
 * away. Integer math throughout -- no %f anywhere near this firmware. */
bool ft_read_current(uint8_t id, int32_t *out_ma);

/* Reads the EEPROM protection block (registers 13..36) in one packet.
 * `out24` receives raw bytes, reg N at index N-13. READ ONLY -- these are
 * EEPROM and the hand's own overheat protection evidently did not fire at
 * 150 C, so we want to see them, not change them. Addresses are from an
 * external note, not vuehand/feetech_packet.py, so treat the interpretation
 * as unverified and the raw bytes as the actual evidence. */
bool ft_read_protection_block(uint8_t id, uint8_t *out24);

/* Moving flag (register 66). A stall is "effort saturated AND position not
 * advancing AND not moving" -- saturation alone is not a mechanical stop,
 * since load pins at the torque cap during any hard hold. */
bool ft_read_moving(uint8_t id, bool *out_moving);
