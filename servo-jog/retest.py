"""Focused retest: +/-D ticks, 1.2 s settle, 3 repeats, peak current. Run: python retest.py COM9 4 9 2"""
import sys
import time

from server import Bus, JOINTS, TORQUE_ENABLE, GOAL_POSITION, TORQUE_LIMIT, PRESENT_POSITION

D = 80
bus = Bus(sys.argv[1])


def goto(sid, p, secs=1.2):
    bus.write(sid, GOAL_POSITION, [p & 0xFF, p >> 8])
    peak, t0 = 0, time.time()
    while time.time() - t0 < secs:
        r = bus.read(sid, 69, 2)
        if r:
            c = r[0] | r[1] << 8
            c = -(c & 0x7FFF) if c & 0x8000 else c
            peak = max(peak, abs(c))
        time.sleep(0.03)
    return bus.read_u16(sid, PRESENT_POSITION), peak


for sid in map(int, sys.argv[2:]):
    p0 = bus.read_u16(sid, PRESENT_POSITION)
    bus.write(sid, TORQUE_LIMIT, [400 & 0xFF, 400 >> 8])
    bus.write(sid, GOAL_POSITION, [p0 & 0xFF, p0 >> 8])
    bus.write(sid, TORQUE_ENABLE, [1])
    print(f"servo {sid} ({JOINTS[sid]}), home {p0}")
    for rep in range(3):
        line = []
        for tgt in (p0 + D, p0, p0 - D, p0):
            pos, peak = goto(sid, tgt)
            line.append(f"{tgt - p0:+4d}->{pos - p0:+4d} ({peak:>3}mA*6.5)")
        print("   " + "  ".join(line))
    bus.write(sid, TORQUE_ENABLE, [0])
    t = bus.read(sid, 63, 1)
    print(f"   temp now {t[0] if t else '?'} C")
