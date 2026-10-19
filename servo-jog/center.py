"""Unmounted servo only: move to 2048 (mid-range) at low torque, small +/- check, then torque off at 2048."""
import sys, time
from server import Bus, TORQUE_ENABLE, GOAL_POSITION, TORQUE_LIMIT, PRESENT_POSITION
bus, sid = Bus(sys.argv[1]), int(sys.argv[2])
def go(p, t=1.5):
    bus.write(sid, GOAL_POSITION, [p & 0xFF, p >> 8]); time.sleep(t)
    return bus.read_u16(sid, PRESENT_POSITION)
p0 = bus.read_u16(sid, PRESENT_POSITION)
bus.write(sid, TORQUE_LIMIT, [400 & 0xFF, 400 >> 8])
bus.write(sid, GOAL_POSITION, [p0 & 0xFF, p0 >> 8])
bus.write(sid, TORQUE_ENABLE, [1])
try:
    print(f"servo {sid}: start {p0}")
    print(f"  -> 2048: {go(2048, 3)}")
    print(f"  -> 2128: {go(2128)}   -> 1968: {go(1968)}")
    print(f"  -> 2048: {go(2048)}")
finally:
    bus.write(sid, TORQUE_ENABLE, [0])
    print(f"  torque off, resting at {bus.read_u16(sid, PRESENT_POSITION)}")
