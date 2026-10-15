# OrcaHand (right Touch) — ESP32-P4 bus test, status 2026-10-15

## What to pull

| What | Where |
|---|---|
| **This repo** (servo jog server, P4 firmware for the Waveshare board, patches, calibration) | `https://github.com/sitevue/orcahand-inhouse`, branch **`ESP-Bus-Test`** |
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
  - Hand server `/control` page now shows the servo number next to every slider (the patch). Three host-side calibrations of this hand (right config) agree within a few degrees: `hand-server/calibration/.../calibration.yaml`.

## What is broken / not done

| Item | Status |
|---|---|
| **Motor 4 — index_abd** | **Likely failed.** On the P4 sweep it did not move at all: commanded both ways, stayed at tick 879, stayed cool (27 °C), so the firmware disabled it. Earlier the same day it still moved but ran hot (40 °C vs 25–32 for the rest) and stalled at ~1.3 A in one direction. Check by hand with power off: if the finger moves freely, or the horn turns without the finger following, it's the servo/gears. |
| **Motor 1 — wrist** | **Disabled, but probably not broken.** It sits near the encoder's 0/4095 wrap (lo 3914, hi 3795 — a negative span), so the sweep can't measure it. Needs a position offset or multi-turn mode before it can be calibrated. Don't command the wrist until then. |
| Second calibration sweep | Not run. Will's notes say: `{"cmd":"pose","name":"open"}`, wait, `calibrate`, run twice and keep the second. Let the servos cool below ~40 °C first — the first sweep left several at 44–52 °C (thermal cut is 65 °C). |
| Finger motion on the P4 | Not tried yet. Next step after the second sweep: one finger at a time, small moves. |
| Motor 14 — thumb_abd direction | Unchecked. On Will's hand it needed `{"cmd":"cal_flip","id":14}` or the open pose folds the thumb into the index finger. Watch the first `pose open`. |
| Firmware changes | Compiled and running, but not reviewed upstream; this account has no push access to `sitevue/playground.vuehand`. |

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
- The old host calibration in `orcahand-touch-left-feetech` belongs to a different (left) hand; using it on this hand makes the slider page read joints at 300–400°.
