"""Raw per-servo jog page for the OrcaHand (Feetech STS bus). No calibration involved.

Run:  python server.py [--com COM9] [--port 8430] [--window 800] [--torque 400] [--side right|left]
Open: http://127.0.0.1:8430

Safety rules built in:
  - Torque starts OFF on every servo. Enabling torque first sets goal = current
    position, so a servo never jumps when it is armed.
  - Goals are clamped to 0..4095 and to +/- --window ticks of the position each
    servo had when this server started ("home").
  - Torque limit (reg 48, RAM) is set to --torque (0-1000) before arming.
"""
from __future__ import annotations

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial
from serial.tools import list_ports

# STS register map
TORQUE_ENABLE, GOAL_POSITION, TORQUE_LIMIT, PRESENT_POSITION, PRESENT_LOAD = 40, 42, 48, 56, 60

# Motor id -> joint name, from OrcaHand-Corndog-Pickup hand/config/orcahand-touch-<side>-feetech/config.yaml
JOINT_MAPS = {
    "right": {
        1: "wrist", 2: "index_pip", 3: "index_mcp", 4: "index_abd", 5: "middle_abd", 6: "ring_abd",
        7: "ring_mcp", 8: "ring_pip", 9: "middle_pip", 10: "middle_mcp", 11: "pinky_pip",
        12: "pinky_mcp", 13: "pinky_abd", 14: "thumb_abd", 15: "thumb_mcp", 16: "thumb_dip",
        17: "thumb_cmc",
    },
    "left": {
        1: "wrist", 2: "middle_pip", 3: "ring_pip", 4: "ring_mcp", 5: "ring_abd", 6: "middle_abd",
        7: "index_abd", 8: "index_mcp", 9: "index_pip", 10: "thumb_cmc", 11: "thumb_dip",
        12: "thumb_mcp", 13: "thumb_abd", 14: "pinky_abd", 15: "pinky_mcp", 16: "pinky_pip",
        17: "middle_mcp",
    },
}
JOINTS = dict(JOINT_MAPS["right"])


class Bus:
    def __init__(self, port: str):
        self.s = serial.Serial(port, 1_000_000, timeout=0.02)
        self.lock = threading.Lock()

    def _txrx(self, sid: int, ins: int, params: list[int], reply_len: int) -> bytes | None:
        body = [sid, len(params) + 2, ins] + params
        pkt = bytes([0xFF, 0xFF] + body + [(~sum(body)) & 0xFF])
        for _ in range(3):
            self.s.reset_input_buffer()
            self.s.write(pkt)
            r = self.s.read(6 + reply_len)
            if len(r) == 6 + reply_len and r[:2] == b"\xff\xff" and r[2] == sid:
                return r[5:5 + reply_len]
            time.sleep(0.003)
        return None

    def read(self, sid: int, addr: int, n: int) -> bytes | None:
        with self.lock:
            return self._txrx(sid, 0x02, [addr, n], n)

    def write(self, sid: int, addr: int, data: list[int]) -> bool:
        with self.lock:
            return self._txrx(sid, 0x03, [addr] + data, 0) is not None

    def read_u16(self, sid: int, addr: int) -> int | None:
        r = self.read(sid, addr, 2)
        return None if r is None else r[0] | (r[1] << 8)


class Hand:
    def __init__(self, bus: Bus, window: int, torque: int):
        self.bus, self.window, self.torque = bus, window, torque
        self.home: dict[int, int] = {}
        self.goal: dict[int, int] = {}
        for sid in JOINTS:
            p = bus.read_u16(sid, PRESENT_POSITION)
            if p is not None:
                self.home[sid] = p
        for sid in self.home:
            bus.write(sid, TORQUE_ENABLE, [0])

    def limits(self, sid: int) -> tuple[int, int]:
        h = self.home[sid]
        return max(0, h - self.window), min(4095, h + self.window)

    def state(self) -> dict:
        out = {}
        for sid in self.home:
            pos = self.bus.read_u16(sid, PRESENT_POSITION)
            load = self.bus.read_u16(sid, PRESENT_LOAD)
            tq = self.bus.read(sid, TORQUE_ENABLE, 1)
            if load is not None and load & 0x400:  # sign bit 10
                load = -(load & 0x3FF)
            lo, hi = self.limits(sid)
            out[sid] = {"joint": JOINTS[sid], "pos": pos, "load": load,
                        "torque": None if tq is None else bool(tq[0]),
                        "home": self.home[sid], "min": lo, "max": hi, "goal": self.goal.get(sid)}
        return out

    def set_torque(self, sid: int, on: bool) -> bool:
        if not on:
            return self.bus.write(sid, TORQUE_ENABLE, [0])
        pos = self.bus.read_u16(sid, PRESENT_POSITION)
        if pos is None:
            return False
        self.bus.write(sid, TORQUE_LIMIT, [self.torque & 0xFF, self.torque >> 8])
        self.bus.write(sid, GOAL_POSITION, [pos & 0xFF, pos >> 8])
        self.goal[sid] = pos
        return self.bus.write(sid, TORQUE_ENABLE, [1])

    def move(self, sid: int, pos: int) -> int:
        lo, hi = self.limits(sid)
        pos = max(lo, min(hi, int(pos)))
        tq = self.bus.read(sid, TORQUE_ENABLE, 1)
        if not tq or not tq[0]:
            self.set_torque(sid, True)
        self.bus.write(sid, GOAL_POSITION, [pos & 0xFF, pos >> 8])
        self.goal[sid] = pos
        return pos


PAGE = (Path(__file__).parent / "index.html").read_bytes()


def make_handler(hand: Hand):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            b = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)
            elif self.path == "/api/state":
                self._json(hand.state())
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}")
            try:
                if self.path == "/api/move":
                    self._json({"goal": hand.move(int(body["id"]), int(body["pos"]))})
                elif self.path == "/api/torque":
                    ids = list(hand.home) if body.get("id") == "all" else [int(body["id"])]
                    self._json({"ok": all([hand.set_torque(i, bool(body["on"])) for i in ids])})
                else:
                    self._json({"error": "not found"}, 404)
            except (KeyError, ValueError) as e:
                self._json({"error": str(e)}, 400)

    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--com", default=None, help="serial port (default: first CH34x/USB serial port)")
    ap.add_argument("--port", type=int, default=8430)
    ap.add_argument("--window", type=int, default=800, help="max ticks either side of start position")
    ap.add_argument("--torque", type=int, default=400, help="torque limit 0-1000")
    ap.add_argument("--side", choices=JOINT_MAPS, default="right", help="which hand's joint labels to show")
    a = ap.parse_args()
    JOINTS.clear()
    JOINTS.update(JOINT_MAPS[a.side])
    com = a.com or next(p.device for p in list_ports.comports() if "USB" in (p.description or "").upper()
                        or "CH34" in (p.description or "").upper())
    hand = Hand(Bus(com), a.window, a.torque)
    print(f"{com}: {len(hand.home)} servos found, torque off. http://127.0.0.1:{a.port}", flush=True)
    ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(hand)).serve_forever()


if __name__ == "__main__":
    main()
