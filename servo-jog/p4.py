"""Talk to the vuehand P4 firmware over its USB-C console.

  python p4.py COM10 --boot                     # reset, print the boot log
  python p4.py COM10 '{"cmd":"ping"}' ...       # send JSON lines, print replies
"""
import sys
import time

import serial

port, args = sys.argv[1], sys.argv[2:]
s = serial.Serial()
s.port, s.baudrate, s.timeout = port, 115200, 0.1
s.dtr = s.rts = False  # RTS drives EN on this board; asserting it holds the P4 in reset
s.open()


def drain(secs, until_ok=False):
    end = time.time() + secs
    while time.time() < end:
        line = s.readline()
        if line:
            text = line.decode(errors="replace").rstrip()
            print(text, flush=True)
            if until_ok and text.startswith('{"ok"'):
                return
            if not until_ok:
                end = max(end, time.time() + 0.3)


if "--boot" in args:
    s.rts = True
    time.sleep(0.1)
    s.rts = False
    drain(12)
    args = [a for a in args if a != "--boot"]
else:
    drain(0.3)
for cmd in args:
    print(f">>> {cmd}")
    s.write((cmd + "\n").encode())
    if '"calibrate"' in cmd:
        drain(900, until_ok=True)  # the sweep logs progress, then one final {"ok"...} reply
    else:
        drain(1.5)
s.close()
