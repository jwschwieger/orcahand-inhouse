"""Exhaustive search for an unknown servo: Feetech PING to IDs 0-253 at every Feetech baud,
plus a Dynamixel Protocol 2.0 broadcast ping. Prints any reply, and any stray bytes at all.

  python deep_scan.py COM11
"""
import sys

import serial

port = sys.argv[1]
FEETECH_BAUDS = [1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400]


def crc16_dxl(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


for baud in FEETECH_BAUDS:
    s = serial.Serial(port, baud, timeout=0.015)
    hits, stray = [], 0
    for i in range(254):
        body = [i, 2, 0x01]  # PING
        s.reset_input_buffer()
        s.write(bytes([0xFF, 0xFF] + body + [(~sum(body)) & 0xFF]))
        r = s.read(6)
        if len(r) >= 5 and r[:2] == b"\xff\xff" and r[2] == i:
            hits.append(i)
        elif r:
            stray += len(r)
    # broadcast ping: every servo answers, whatever its ID
    s.reset_input_buffer()
    s.write(bytes([0xFF, 0xFF, 0xFE, 0x02, 0x01, (~(0xFE + 0x02 + 0x01)) & 0xFF]))
    s.timeout = 0.1
    bc = s.read(64)
    s.close()
    print(f"feetech {baud:>8}: ids {hits}  stray bytes {stray}  broadcast reply {bc.hex() or '-'}")

for baud in (57_600, 1_000_000, 115_200, 2_000_000):
    s = serial.Serial(port, baud, timeout=0.1)
    pkt = [0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x03, 0x00, 0x01]
    c = crc16_dxl(pkt)
    s.write(bytes(pkt + [c & 0xFF, c >> 8]))
    r = s.read(128)
    s.close()
    print(f"dynamixel2 {baud:>8}: broadcast reply {r.hex() or '-'}")
