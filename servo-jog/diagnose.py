"""Servo health check: register dump + small per-servo motion test (+DELTA ticks and back).

Run: python diagnose.py [COM9] [--no-move]
"""
import sys
import time

from server import Bus, JOINTS, TORQUE_ENABLE, GOAL_POSITION, TORQUE_LIMIT, PRESENT_POSITION

DELTA = 40
port = next((a for a in sys.argv[1:] if not a.startswith("--")), "COM9")
move = "--no-move" not in sys.argv
bus = Bus(port)


def s16(v, sign_bit):
    return -(v & ((1 << sign_bit) - 1)) if v & (1 << sign_bit) else v


print(f"{'id':>2} {'joint':<11} {'pos':>5} {'volt':>5} {'temp':>4} {'stat':>4} {'curr':>5} {'minlim':>6} {'maxlim':>6} {'mode':>4} {'offs':>5} {'maxtq':>5}")
alive = []
for sid, name in JOINTS.items():
    eep = bus.read(sid, 9, 8)    # 9-10 min limit, 11-12 max limit, 13 max temp, 14 max V, 15 min V, 16 max torque(lo)
    ram = bus.read(sid, 56, 15)  # 56 pos, 58 speed, 60 load, 62 volt, 63 temp, 64 async, 65 status, 66 moving, 67-68, 69 current
    mode = bus.read(sid, 33, 1)
    offs = bus.read(sid, 31, 2)
    if eep is None or ram is None:
        print(f"{sid:>2} {name:<11}  NO REPLY")
        continue
    alive.append(sid)
    pos = ram[0] | ram[1] << 8
    curr = s16(ram[13] | ram[14] << 8, 15)
    print(f"{sid:>2} {name:<11} {pos:>5} {ram[6] / 10:>5.1f} {ram[7]:>4} {ram[9]:>#4x} {curr:>5} "
          f"{eep[0] | eep[1] << 8:>6} {eep[2] | eep[3] << 8:>6} {mode[0] if mode else '?':>4} "
          f"{s16(offs[0] | offs[1] << 8, 11) if offs else '?':>5} {eep[7]:>5}")

if not move:
    sys.exit()

print(f"\nMotion test: +{DELTA} ticks and back, torque limit 400")
for sid in alive:
    name = JOINTS[sid]
    p0 = bus.read_u16(sid, PRESENT_POSITION)
    bus.write(sid, TORQUE_LIMIT, [400 & 0xFF, 400 >> 8])
    bus.write(sid, GOAL_POSITION, [p0 & 0xFF, p0 >> 8])
    bus.write(sid, TORQUE_ENABLE, [1])
    target = p0 + DELTA
    bus.write(sid, GOAL_POSITION, [target & 0xFF, target >> 8])
    time.sleep(0.6)
    p1 = bus.read_u16(sid, PRESENT_POSITION)
    r = bus.read(sid, 65, 6)
    stat = r[0] if r else None
    curr = s16(r[4] | r[5] << 8, 15) if r else None
    bus.write(sid, GOAL_POSITION, [p0 & 0xFF, p0 >> 8])
    time.sleep(0.6)
    p2 = bus.read_u16(sid, PRESENT_POSITION)
    bus.write(sid, TORQUE_ENABLE, [0])
    out = p1 - p0 if p1 is not None else None
    back = p2 - p0 if p2 is not None else None
    ok = out is not None and abs(out - DELTA) <= 8 and back is not None and abs(back) <= 8
    print(f"{sid:>2} {name:<11} start {p0:>5}  out {out!s:>4} (want {DELTA})  back {back!s:>4}  "
          f"stat {stat!s:>4} curr {curr!s:>5}  {'ok' if ok else '<-- PROBLEM'}")
