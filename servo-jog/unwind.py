"""Powered unwind attempt on one servo: step `step` ticks at a time in direction `dir` (+1/-1),
up to `max` ticks, at torque limit `torque`. Stops when the shaft stops following (3 steps with
under 5 ticks of progress). Torque is always released at the end.

  python unwind.py COM11 3 -1 [--max 300] [--step 20] [--torque 300]
"""
import argparse
import time

from server import Bus, TORQUE_ENABLE, GOAL_POSITION, TORQUE_LIMIT, PRESENT_POSITION, PRESENT_LOAD

ap = argparse.ArgumentParser()
ap.add_argument("com")
ap.add_argument("id", type=int)
ap.add_argument("dir", type=int, choices=(-1, 1))
ap.add_argument("--max", type=int, default=300)
ap.add_argument("--step", type=int, default=20)
ap.add_argument("--torque", type=int, default=300)
a = ap.parse_args()

bus = Bus(a.com)
sid = a.id
p0 = bus.read_u16(sid, PRESENT_POSITION)
if p0 is None:
    raise SystemExit(f"servo {sid} not answering")
bus.write(sid, TORQUE_LIMIT, [a.torque & 0xFF, a.torque >> 8])
bus.write(sid, GOAL_POSITION, [p0 & 0xFF, p0 >> 8])
bus.write(sid, TORQUE_ENABLE, [1])
try:
    goal, stuck, pos = p0, 0, p0
    print(f"servo {sid}: start {p0}, dir {a.dir:+d}, torque {a.torque}")
    while abs(goal - p0) < a.max and stuck < 3:
        goal = max(0, min(4095, goal + a.dir * a.step))
        bus.write(sid, GOAL_POSITION, [goal & 0xFF, goal >> 8])
        time.sleep(0.35)
        new = bus.read_u16(sid, PRESENT_POSITION)
        load = bus.read_u16(sid, PRESENT_LOAD)
        if load is not None and load & 0x400:
            load = -(load & 0x3FF)
        stuck = stuck + 1 if new is None or abs(new - pos) < 5 else 0
        pos = new if new is not None else pos
        print(f"  goal {goal:>5}  pos {pos:>5}  load {load}")
finally:
    bus.write(sid, TORQUE_ENABLE, [0])
    end = bus.read_u16(sid, PRESENT_POSITION)
    print(f"  torque off. moved {end - p0:+d} ticks ({'stalled' if stuck >= 3 else 'reached max'})")
