# Vuehand ESP32-P4 firmware

The P4 is the servo brain. A host (Mac, Pi, or nothing at all) sends
**high-level JSON** over the P4's console USB; the P4 turns that into servo bus
traffic. It is meant to run standalone — calibration lives in NVS and thermal
protection runs on-device, so the hand is safe with no host attached.

Two servo backends, selected at configure time by the `VH_SERVO_TRANSPORT`
CMake cache variable:

| transport | bus | hand | status |
|---|---|---|---|
| `usb` | **Feetech SMS/STS Protocol 1, 1 Mbps** over a USB-serial adapter in the P4's USB-A host port | ORCA Hand **Touch** | **implemented, verified on hardware** |
| `uart` | Dynamixel Protocol 2.0, TTL half-duplex, 1 Mbps on GPIO | ORCA **v2** | implemented, not the path under active work |

## USB command port

Use the ESP32-P4 **USB Serial/JTAG** USB-C (the console port) — this is a
*different* USB link from the one carrying servo traffic. 115200 8N1, one JSON
object per line, typed straight into an attached `idf.py monitor`.

```json
{"cmd":"ping"}
{"cmd":"status"}
{"cmd":"torque","on":true}
{"cmd":"relax"}
{"cmd":"stop"}
{"cmd":"say","text":"grasp the corndog","t_ms":600}
{"cmd":"pose","name":"fist"}
{"cmd":"pose","name":"fist","grasp":"soft"}
{"cmd":"sequence","name":"wave"}
{"cmd":"set_joints","vec":[...17 floats, degrees...],"t_ms":400}
```

Every command that touches the bus is gated on the transport actually being
open. `transport_init()` returning is **not** the same as the adapter being
enumerated — that gap is roughly 5 s, and before it was closed the boot-time
torque-off writes went nowhere while the log claimed success.

### Calibration

```json
{"cmd":"calibrate"}       // runs the sweep, persists to NVS
{"cmd":"cal"}             // report the stored table
{"cmd":"cal_clear"}       // forget it
{"cmd":"goto_tick","id":3,"ticks":1200}
{"cmd":"read_pos","id":3}
{"cmd":"protection","id":10}
```

`calibrate` physically winds each motor into both mechanical stops, one at a
time, at a low torque limit, then **writes each measured endpoint back as a
real goal at reach torque and records where the motor actually lands.** That
last step is not optional bookkeeping: the sweep inches 40 ticks at a time
under rising antagonist tension and finds stops the motor can only *creep* to.
Nine motors on this hand had 187–852 ticks of travel that no goal could ever
reach. Poses built on the un-verified endpoints commanded motors into that dead
zone, where they stalled and heated instead of arriving.

**The other sixteen motors are held at pose torque while one is probed.** Each
joint has one hard mechanical stop and one *soft* end where the probe stalls
against tendon tension, and that tension depends on where the coupled joints
are sitting. Measured 2026-09-02 across two sweeps from the same posture with
the rest of the hand limp: every hard end repeated within a few ticks, every
soft end wandered 100–360 ticks (index_mcp lo 1091 → 731, thumb_abd lo
666 → 361, thumb_cmc lo 745 → 913 — a 72-tick span, thumb effectively frozen).
With the neighbours limp, each probe drags them somewhere new, and the thumb
CMC, probed last, inherits whatever the three thumb probes before it left
behind. Holding everything at its present position at the start, and parking
each finished motor mid-span *held* rather than releasing it, turns the soft
end into a measurement against a defined posture. First held sweep: thumb_cmc
523, thumb_abd 505, thumb_mcp 1142 — back to the best spans ever measured.
Two further held sweeps from the same `open` start: **all 34 endpoints
repeated within 8 ticks.**

Because the soft ends are posture-dependent, **the sweep must start from a
defined posture** — `{"cmd":"pose","name":"open"}`, wait, then `calibrate`.
The open pose is itself computed through the calibration that is loaded, so
the first held sweep after a bad calibration starts from a slightly different
physical posture than the next one (index_mcp's flexion end came out 728 with
a frozen-thumb calibration in force and 1122–1130 on every sweep since, the
thumb sitting ~300 ticks elsewhere). Run it twice after any large change and
keep the second.

The flip side: middle and ring abduction came out **smaller** held (362 / 242
vs 449–548 limp), because a neighbour that no longer yields is a real limit. A
limp neighbour gets pushed aside and inflates the range; at pose time that
inflated range commands a finger into a neighbour that is holding, which is a
stall. The smaller number is the reachable one. Abduction range is genuinely
posture-dependent on this hand.

**Endpoints are verified by waiting for the motor, not the clock.** The first
version waited a fixed 1.2 s and read wherever the shaft was; every curl
motor travelled the same ~1100–1200 ticks in that window and stopped
170–350 short, and the end the probe left the motor sitting on was "verified"
in place, a no-op. Now each end is approached from a standoff in the middle
of the crept span, at probe speed and pose torque, the position is polled
until it stops changing for ~240 ms, and the end is recorded as *arrived* or
*stalled* (stopped short: the reachable end). An end that is still moving at
6 s or loses its reply is **unverified**: the motor keeps its previous table
(flag 0x10) or is disabled if there was none. Never the creep value, never the
in-flight tick.

First settle-verified sweep, 2026-09-02 (run F vs. the fixed-delay run E):
every end `stalled` 15–25 ticks inside its crept value, none timed out, and
the curl motors gained 150–320 ticks of flexion each — index_mcp 1186 → 1465,
ring_mcp 1192 → 1509, middle_mcp 1202 → 1521, thumb_mcp 1112 → 1339 — while
the hi ends each moved in ~20 ticks because they were finally approached.
Abduction motors were unchanged within 14 ticks.

An end at the **encoder's edge** (`HIT_BOUND`) is inset by 300 ticks, at
sweep time and on load. Run F put the wrist's low end at tick 39: reachable,
but a push past zero wraps the reading to 4095 and the servo drives the wrong
way at full effort. Multi-turn mode is the proper fix, as a deliberate step.

`calibrate` relaxes the hand before it starts (it used to inherit the armed
state, so the auto-relax leash could fire mid-sweep and the thermal poll kept
rewriting the probe's ceiling) and leaves everything limp when it finishes.

`{"cmd":"cal_flip","id":N}` reverses the direction the joint angle maps onto
ticks for one motor, for a motor mounted the opposite way from the v2 map.
It is recorded as a per-motor override (`flags` bit 0x08) and reapplied by
every later sweep. **Motor 14 (thumb_abd) is flipped on this hand**: without
it the open pose folds the thumb across the palm and into the index finger's
path, and the index MCP calibrates 400 ticks short.

Deliberately **not** run on every boot — winding 17 tendon-driven motors into
hard stops at power-up would crush whatever is in the hand for no reason. Run
it once; the result is loaded from NVS at boot.

### Grasp force and thermal state

```json
{"cmd":"grasp"}                      // report current profile
{"cmd":"grasp","name":"soft"}        // soft | medium | firm
{"cmd":"grasp","permille":260}       // raw override, clamped
{"cmd":"grip"}                       // did it close on something, or on air?
{"cmd":"temps"}
{"cmd":"torque_status"}              // reads register 40 back, per motor
{"cmd":"faults"}                     // servo error byte per motor: last + sticky since boot
{"cmd":"faults","clear":true}
```

**Reach torque and holding torque are separate.** Reach is fixed at
`VH_TORQUE_LIMIT_ARMED` (300‰) for every profile, because that is the value
calibration verified each endpoint against — a profile that reached at 120‰
would be commanded to endpoints it cannot get to and would stall there. Reach
is a property of the hand's mechanics. Holding torque is the knob:

| profile | hold | auto-relax leash |
|---|---|---|
| `soft` | 120‰ | 60 s |
| `medium` (default) | 220‰ | 30 s |
| `firm` | 400‰ | 10 s |

Harder squeeze, shorter rope. Anything above `medium` is refused while a motor
is already at 50 °C or more. Once a motor has arrived at its goal there is no
position error, so the ceiling costs nothing; it bites only when something is
in the way, which is what grasp force means.

`grip` reports goal-minus-present error and load per motor. Error ≈ 0 with low
load is an empty fist; a large error with load pinned near the ceiling means
the hand has hold of something.

**Measured, 2026-09-02**, fist closed on a taped can, then the profile switched
from `soft` to `medium` on the armed hand without moving:

| motor | err (ticks) soft → medium | load soft → medium |
|---|---|---|
| 2 | −49 → −41 | 120 → 172 |
| 3 | −40 → −38 | 120 → 160 |
| 7 | +94 → +94 | −120 → −220 |
| 8 | −78 → −78 | 120 → 220 |
| 9 | −34 → −33 | 120 → 140 |
| 10 | −31 → −30 | 120 → 128 |
| 11 | +45 → +37 | −120 → −156 |
| 12 | −137 → −137 | 120 → 220 |
| 15 | −37 → −35 | 120 → 148 |

No error moved more than 8 ticks — the can sets the shape — while every load
climbed with the ceiling. The seven motors that were not pinned did not change
at all, which is what says the load reading is physical and not the written
limit echoed back. Two things worth knowing that fall out of it:

- **Load is position error × gain, clipped at the ceiling.** Every unclipped
  motor sits at ≈ 4.2‰ per tick of error (41 → 172, 38 → 160, 33 → 140,
  30 → 128). So the holding ceiling only bites on motors more than ≈ 29 ticks
  from goal at `soft` or ≈ 52 at `medium`; a firmer profile squeezes harder
  only on the fingers the object is already blocking most.
- **Relaxing is not dropping.** With torque off the closed fingers still
  cradled the can by geometry. Holding torque is what resists something being
  pulled out, not what stops it falling — which is why the leash can be short.

Cost: a 60 s `soft` hold took the curl motors from the mid-20s to the mid-30s
°C, peaking at 44. Soft is not free either.

`faults` reports the error byte each servo attaches to every status reply,
raw and undecoded (`last` = most recent reply, `seen` = sticky OR since boot
or the last `clear`). Validated on 2026-09-02 by accident: with the hand's
12 V unplugged and the servos running on the adapter's trickle, only three
motors answered and two of them flagged bit 0 (input voltage), one also bit 1
(angle sensor). The moment the rail came up `last` went to zero on all 17 and
`seen` kept the history — so an all-zero reading means "no faults", not
"wrong offset".

### What protects the hand

Learned the hard way — every one of these exists because of a specific incident:

- **Torque ceiling on arming.** Nothing wrote register 48 before, so motors ran
  at whatever was last left in SRAM.
- **Speed and acceleration limits** (`LIVE_SPEED` 2200 / `LIVE_ACC` 80). Feetech
  treats goal speed 0 as *unlimited*, not stopped. Software interpolation slows
  nothing without these.
- **Goal parked at present position before torque enable.** Enabling torque
  makes the servo chase whatever stale value is in its goal register.
- **Per-finger thermal throttle**, warn 55 °C → floor 0.35× by 65 °C, applied
  per digit so one hot joint doesn't fall behind its neighbours mid-grasp.
- **Release on a rising temperature inside the warn band.** Scaling a ceiling
  does not stop I²R in a stalled hold — motor 10 climbed 55 → 64 °C while
  already being throttled. The hand now lets go instead of riding the floor
  down into a cut.
- **Hard cut at 65 °C**, latched; arming is refused until everything is back
  under 55 °C. Below the servos' own ~70 °C shutdown, deliberately.
- **Auto-relax leash.** A held pose is not a resting state — it is every tendon
  against its stop. Every overheat this project has had traces to someone
  forgetting to relax the hand.
- **Disabled motors are never commanded.** A jammed motor handed an unreachable
  goal sits at stall current. Torque-*off* still goes to everything.
- **Packet pacing in the send path.** Addressed writes elicit a status reply
  that collides with the next outgoing packet on a half-duplex bus. Enforcing
  the gap per-callsite meant reintroducing the same dropped-write bug, which
  presented convincingly as a dead motor.

## Servo wiring — `usb` transport (ORCA Hand Touch)

A USB-to-serial adapter in one of the P4 dev kit's **USB-A host** ports, with
its TTL side on the hand's Feetech bus. The adapter on this bench is not
FTDI/CP210x/CH34x — it enumerates through the standards-compliant CDC-ACM
path. Enumeration is retried in a persistent task, so unplugging and replugging
the hand mid-session recovers on its own.

Power the hand from its own PSU. Never from the P4.

## Servo wiring — `uart` transport (ORCA v2, Dynamixel)

TTL half-duplex UART, 1 Mbps, Protocol 2.0. Do **not** leave the hand's U2D2
talking on the same data line at the same time.

| P4 pin (default) | Function | To the hand |
|---|---|---|
| GPIO20 | UART1 TX | through a 74LVC1T45 / MOSFET driver onto **DATA** |
| GPIO21 | UART1 RX | **DATA** (5 V tolerant buffer — Dynamixel TTL is 5 V) |
| GPIO8 | UART1 RTS | DE/RE of the TTL driver (RS485 half-duplex mode) |
| GND | ground | hand GND |
| — | power | **12 V 5 A** to the hand PSU, **not** from the P4 |

Overridable at configure time as `-D VH_SERVO_TX=<n>` (likewise `VH_SERVO_RX`,
`VH_SERVO_RTS`).

GPIO11/12/13 — the old ORCA-v2 dev defaults — are **not broken out** on this
Waveshare board's header at all; they are reserved internally for the
PSRAM/flash bus. Confirmed against the physical header. The XC330/XC430 **T**
models are TTL: use a level-shift or open-drain buffer, and never feed 12 V
into a P4 pin.

## Flash

Needs ESP-IDF ≥ 5.4 with P4 support.

```sh
uv run vuehand train          # writes model_weights.h + seed_poses.h + joints_table.h
uv run vuehand flash --port /dev/cu.usbmodemXXXX
```

Or:

```sh
cd firmware/vuehand_p4
idf.py set-target esp32p4
idf.py -D VH_SERVO_TRANSPORT=usb build      # 'uart' is the CMake default
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

**Left hand:** add `-D VH_HAND_SIDE=left` to the build (default `right`). It picks
`VH_LEFT_MAP` over `VH_RIGHT_MAP` at boot; the boot log prints `hand side: left`.
Like the transport it is a CMake cache variable and sticks for the build tree. A
right build on a left hand drives the wrong motor for most joints, so check the
log line before the first `calibrate`. Everything under *Known hardware state*
below (motor 14 flip, motor 4 jam) is this bench's right hand and does not carry
over � a new board starts with an empty NVS and needs its own sweep.

`VH_SERVO_TRANSPORT` is a CMake **cache** variable, so the `-D` sticks for the
rest of the build tree's life — subsequent `idf.py build` / `flash monitor`
calls keep the transport you last selected. Check it with
`grep VH_SERVO_TRANSPORT build/CMakeCache.txt` if you are unsure which backend
a binary has in it.

## Known hardware state on this bench

- **KNOWN ISSUE — motor 4 (`index_abd`) is a mechanical jam. The servo is
  fine; do not order a replacement.** Flagged `DISABLED` in the stored
  calibration and never commanded, so the hand is usable as-is with 16 of 17
  joints. Fixing it is a hardware job: check whether the index abduction
  tendon is off its pulley, wrapped or pinched, and whether the joint is
  fouled. Recheck with `{"cmd":"probe","id":4}` afterwards, then recalibrate
  to give it a real span.

  Evidence, so nobody re-opens this: span 0 on three separate calibration
  runs and the coldest motor on the hand (25 °C), yet it answers every
  register read. `{"cmd":"faults"}` reports zero on all 17 motors (and the
  byte is known to be live — see *Grasp force and thermal state*). With the
  whole hand limp, middle/ring/pinky abduction all back-drive freely by hand
  and index will not budge. And `{"cmd":"probe","id":4}`:

  ```
  start 295, asked 415 -> got 295, load -100, ma -416
             asked 175 -> got 295, load +100, ma +760, moved 0
  ```

  Load saturated at exactly the commanded 100‰ ceiling in **both** directions
  with correctly opposite signs, drawing 416 and 760 mA — so torque enable
  took, the winding conducts, and the motor is making real force at its full
  permitted ceiling while the shaft does not move one tick. That rules out
  the electrical failures.

  One honest limit: 100‰ proves immovable *at 100‰*, not at any torque.
  Separating "jammed" from "merely stiff" by cranking torque into a stuck
  joint is how motors cook, so that question goes to inspection rather than
  to a bigger number. Note for whoever opens it: 760 mA down versus 416 mA
  up at the same saturated load — the mechanism resists the two directions
  differently.
- **The `fist` pose collides the thumb with the index on an empty close.** thumb_abd 10° puts
  the thumb across the palm under the index MCP's path; on an empty fist both stall against each
  other (thumb_abd pinned at the ceiling 101 ticks short, index_mcp at 196‰). With an object in
  the palm the fingers stop on it first and the grasp reads clean. The same fist with thumb_abd
  45° is clean everywhere. To be fixed in the thumb pose values after the thumb CMC/MCP/DIP
  directions are checked one motor at a time, the way motor 14 was.
- **Motor 1 (wrist) is `HIT_BOUND`** — ran to tick 21 before stalling. It needs
  multi-turn mode enabled deliberately; left alone for now. It draws ~480 mA
  holding against its stop, roughly 10× every other motor at rest.
- **Abduction spans were not repeatable** between calibration runs (thumb_abd
  528 → 182 ticks across two runs). Diagnosed 2026-09-02: the soft end of each
  joint depends on where the coupled joints sit, and the sweep was leaving
  them limp. See *Calibration* — the sweep now holds the rest of the hand.
- Hardware backstops are intact and were verified by register read: `max_temp`
  (reg 13) = 80 °C on all 17 motors, protection current 450 raw ≈ 2.9 A at
  6.5 mA/LSB, mode register 33 = 0 (position mode) throughout.
