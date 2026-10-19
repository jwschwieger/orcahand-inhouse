"""Wiggle one servo so it can be found by eye: +/-amp ticks, `times` cycles, then torque off.

  python wiggle.py COM11 3 [--amp 60] [--times 3] [--torque 300]
"""
import argparse
import time

from server import Bus, TORQUE_ENABLE, GOAL_POSITION, TORQUE_LIMIT, PRESENT_POSITION

ap = argparse.ArgumentParser()
ap.add_argument("com")
ap.add_argument("id", type=int)
ap.add_argument("--amp", type=int, default=60)
ap.add_argument("--times", type=int, default=3)
ap.add_argument("--torque", type=int, default=300)
a = ap.parse_args()

bus, sid = Bus(a.com), a.id
p0 = bus.read_u16(sid, PRESENT_POSITION)
if p0 is None:
    raise SystemExit(f"servo {sid} not answering")
bus.write(sid, TORQUE_LIMIT, [a.torque & 0xFF, a.torque >> 8])
bus.write(sid, GOAL_POSITION, [p0 & 0xFF, p0 >> 8])
bus.write(sid, TORQUE_ENABLE, [1])
try:
    for _ in range(a.times):
        for p in (p0 + a.amp, p0 - a.amp):
            p = max(0, min(4095, p))
            bus.write(sid, GOAL_POSITION, [p & 0xFF, p >> 8])
            time.sleep(0.6)
    bus.write(sid, GOAL_POSITION, [p0 & 0xFF, p0 >> 8])
    time.sleep(0.6)
finally:
    bus.write(sid, TORQUE_ENABLE, [0])
    print(f"servo {sid} wiggled around {p0}, torque off at {bus.read_u16(sid, PRESENT_POSITION)}")
