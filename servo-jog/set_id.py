"""Scan a Feetech STS bus and change one servo's ID. Refuses unless exactly one servo answers.

  python set_id.py COM11              # scan only
  python set_id.py COM11 --to 3       # change the lone servo's ID to 3
"""
import sys
import time

from server import Bus, PRESENT_POSITION

ID_REG, LOCK_REG = 5, 55  # EEPROM ID; EEPROM write lock (0 = unlocked)

port = sys.argv[1]
new_id = int(sys.argv[sys.argv.index("--to") + 1]) if "--to" in sys.argv else None
bus = Bus(port)

found = [i for i in range(0, 254) if bus.read_u16(i, PRESENT_POSITION) is not None]
print(f"servos answering on {port}: {found}")
if new_id is None:
    sys.exit()
if len(found) != 1:
    sys.exit(f"refusing: need exactly one servo on the bus, found {len(found)}")
old = found[0]
if not 1 <= new_id <= 253:
    sys.exit("new id must be 1..253")
if old == new_id:
    sys.exit(f"servo is already ID {new_id}")

bus.write(old, 40, [0])                 # torque off
bus.write(old, LOCK_REG, [0])           # unlock EEPROM
bus.write(old, ID_REG, [new_id])        # new ID (reply comes from the new ID, so not checked here)
time.sleep(0.05)
bus.write(new_id, LOCK_REG, [1])        # relock under the new ID
time.sleep(0.05)
after = [i for i in (old, new_id) if bus.read_u16(i, PRESENT_POSITION) is not None]
print(f"after change, answering: {after}")
print("OK" if after == [new_id] else "ID change NOT confirmed")
