# OrcaHand (right Touch) — ESP32-P4 bus test, status 2026-10-19

## Where we stopped (read this first)

**Index finger servos 3 and 4 are being replaced.** New servos for both are prepared; the swap itself is in progress.

| | Old servo (still in the hand) | New servo (ready, not installed) |
|---|---|---|
| **ID 3 — index_mcp** (knuckle curl) | **Weak**: moves, but needs ~4x the effort of a healthy servo (load ~100/1000 vs ~24 for servo 9 at the same move). At a 100/1000 torque cap it pushes at the cap (0.4–0.75 A) and barely moves. Worked in the morning and in P4 sweep 1, gave out after stalling at 1.24 A in `pose open`. | ID set to 3, centered at tick 2048, verified (12.5 V, 26 °C, hits targets within 1 tick). |
| **ID 4 — index_abd** (side to side) | **Failing**: stick-slip (catches, load jumps to ~140, then jumps free), 0 ticks moved in both P4 sweeps and at a 100/1000 cap; ran hot (40 °C) on day one. | ID set to 4, centered at 2048, verified (12.5 V, 23–28 °C). |

**Where they are:** back side of the hand, **left column — servo 3 is row 2, servo 4 is row 3 (directly below 3)**. Found by wiggling each one (`servo-jog/wiggle.py`).

**One spare is bad:** it never answered on any ID/baud/protocol, and plugging it in tripped the adapter's power supply (servo 4 went silent until the PSU was replugged). Kept apart — check its plug's wire order against a good servo before ever reusing it.

**Swap procedure (each servo drives two spools wound opposite ways, so turning the servo can't slacken both tendons):**
1. Unplug the hand cable from the adapter (or kill bus power).
2. Undo the horn screw, lift horn + both spools off the shaft *with tendons still wound*, and hold the spools so they don't spin.
3. Fit the new servo (already centered at 2048), press the same horn + spools onto it without rotating them.
4. If the spools can't come off with tendons on: power the bus, slacken the tighter tendon with `servo-jog/unwind.py`, free it, then reverse for the other.

**After the swap:**
1. Hand cable back on the adapter, **jumper to A**, P4 USB-C to the PC (COM10).
2. `python servo-jog/p4.py COM10 '{"cmd":"temps"}'` — all 17 should report.
3. `{"cmd":"pose","name":"open"}`, then `{"cmd":"calibrate"}` (~8 min). Index motors 3/4 are currently stored as disabled; a clean sweep re-enables them.
4. Then move fingers one at a time.

### Second P4 calibration (2026-10-15, from `pose open`) — currently stored on the board

15 measured, 2 disabled (3, 4). Log: `logs/p4-calibration-2-2026-10-15.log`.

- **Wrist (1) now works**: 300–1654 (1354 ticks). Its low end hit the encoder edge at tick 34 and was inset to 300.
- Index_pip (2) only 893 ticks (was 1234) — the index was binding.
- Ring_mcp (7) 1509 (sweep 1 ran out of probe budget at 1026).
- All other motors within ~60 ticks of sweep 1.
- Thumb_abd (14) did not fold into the palm on `pose open`; no `cal_flip` needed so far.

## What to pull

| What | Where |
|---|---|
| **This repo** (servo jog server, P4 firmware for the Waveshare board, patches, calibration) | `https://github.com/jwschwieger/orcahand-inhouse`, branch **`ESP-Bus-Test`** |
| Hand server + `/control` slider page (PC-side, used before the P4) | `https://github.com/sitevue/OrcaHand-Corndog-Pickup`, branch `CornbotLocalControl` (7a929b52), then `git apply hand-server/control-page-servo-numbers.patch` from this repo |
| Upstream of the P4 firmware | `https://github.com/sitevue/playground.vuehand`, `firmware/vuehand_p4/` (Will's). The copy here is master 1a02536 + the left/right `VH_HAND_SIDE` option (b9738eb, not merged upstream) + the Waveshare changes in `firmware/waveshare-p4-changes.patch` |

## Hardware on the bench

- **Hand:** ORCA Touch, **right** hand, 17 Feetech STS servos (IDs 1–17), 1 Mbps.
- **Controller:** Waveshare **ESP32-P4-WIFI6** dev board (P4 silicon **v1.3**, 32 MB flash). Its USB-C is a CH343 USB-UART bridge on UART0 — that is the JSON command console (115200 baud).
- **Servo bus:** Waveshare **Bus Servo Adapter (A)**, jumper on **A** (UART mode), bus power from its DC jack.
- **Wiring:** P4 **GPIO2 → adapter TX**, **GPIO3 → adapter RX** (straight through, as Waveshare's wiki says — not crossed), plus GND. No direction pin; the adapter switches the half-duplex line itself and does **not** echo TX.

## What is working

- **P4 firmware builds, flashes and boots** on the Waveshare board (right-hand map, torque off at boot, thermal monitor running).
- **All 17 servos answer over the P4 → Waveshare UART link**: torque state, temperatures, error bytes (all zero) read back from every motor.
- **On-device calibration ran** (`{"cmd":"calibrate"}`, ~5 min) and is stored in NVS: **15 of 17 motors measured** with verified endpoints — curl joints 1026–1500 ticks, abduction/thumb_cmc 318–475 ticks. Full log in `logs/p4-calibration-2026-10-15.log` (motor 1's lines were missed by the log capture; its result is in the table below).
- **PC-side tools** (used with the hand on a USB adapter directly, before the P4):
  - `servo-jog/server.py` — raw per-servo slider page on http://127.0.0.1:8430, no calibration needed. Torque starts off, arming parks the goal at the present position, moves are clamped to ±800 ticks of the start position, torque limit 400/1000. `--side right` (default) labels the joints.
  - `servo-jog/diagnose.py` / `retest.py` — read-only register dump and small per-servo motion tests.
  - `servo-jog/set_id.py` — scan, and change the ID of a lone servo (refuses unless exactly one answers). New Feetech servos ship as ID 1, which clashes with the wrist.
  - `servo-jog/center.py` — move an *unmounted* servo to 2048 and back-check it.
  - `servo-jog/wiggle.py` — ±60-tick wiggle to find a servo by eye. `unwind.py` — stepped powered move that stops on stall.
  - `servo-jog/deep_scan.py` — Feetech ping at every baud + broadcast + Dynamixel 2.0 ping, for a servo that won't answer.
  - With the PC on the adapter (jumper **B**, adapter USB-C), the hand shows up as its own COM port (COM11 here).
  - Hand server `/control` page now shows the servo number next to every slider (the patch). Three host-side calibrations of this hand (right config) agree within a few degrees: `hand-server/calibration/.../calibration.yaml`.

## Still open

| Item | Status |
|---|---|
| Index servos 3 and 4 | Being replaced — see "Where we stopped" above. |
| Finger motion on the P4 | Only `pose open` so far. Next after the post-swap sweep: one finger at a time, small moves. |
| Firmware changes | Running on the bench board, but not reviewed upstream; this account has no push access to `sitevue/playground.vuehand`. |
| Day-one notes (superseded) | First P4 sweep disabled the wrist (encoder wrap) — fixed by sweeping from `pose open`. Let servos cool below ~40 °C between sweeps; a sweep leaves several at 44–52 °C (thermal cut 65 °C). |

## How to build and flash the P4

ESP-IDF v5.5.5 (`. ~/esp/esp-idf/export.ps1` on the Windows bench PC). From `firmware/vuehand_p4/`:

```
idf.py -B <build-dir> -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.waveshare_p4_wifi6" ^
  -D VH_SERVO_TRANSPORT=uart -D VH_HAND_SIDE=right ^
  -D VH_SERVO_TX=2 -D VH_SERVO_RX=3 -D VH_SERVO_RTS=-1 set-target esp32p4
idf.py -B <build-dir> build
idf.py -B <build-dir> -p COM10 flash
```

Use a short build path on Windows (we used `%LOCALAPPDATA%\vh_p4_ws`). `sdkconfig.waveshare_p4_wifi6` moves the console to UART0 and allows pre-v3 P4 silicon (`CONFIG_ESP32P4_SELECTS_REV_LESS_V3`); without it the board boots into nothing on the USB-C console.

Talk to it: `python servo-jog/p4.py COM10 '{"cmd":"ping"}' '{"cmd":"temps"}'` (`--boot` resets the board and prints the boot log). Command reference: `firmware/vuehand_p4/README.md`.

## Gotchas we hit

- **Jumper B = USB mode**: the adapter ignores its UART header. With the jumper on B the P4 sees 0/17 servos and no echo — it looks exactly like a wiring fault.
- Only one controller on the servo bus at a time (P4 *or* a PC USB adapter, never both).
- A servo that "doesn't answer" can also mean the bus PSU has tripped — rescan a known-good servo before blaming the new one.
- The old host calibration in `orcahand-touch-left-feetech` belongs to a different (left) hand; using it on this hand makes the slider page read joints at 300–400°.
