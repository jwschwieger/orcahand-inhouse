"""Print the P4 console without resetting it; exit when `pattern` appears or after `secs`."""
import sys, time, serial
port, pattern, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
s = serial.Serial(); s.port, s.baudrate, s.timeout = port, 115200, 0.2
s.dtr = s.rts = False
s.open()
end = time.time() + secs
while time.time() < end:
    line = s.readline()
    if line:
        text = line.decode(errors="replace").rstrip()
        print(text, flush=True)
        if pattern in text:
            break
s.close()
