"""go2_nmea_reader — the Go2 field kit's GPS position, as one HEC event per GPS_PERIOD.

The GNSS is not on the robot: it is the IR1101's cellular module (P-LTEA7-EAL, Sierra EM7421),
which streams NMEA over UDP (`lte gps nmea ip udp <src> <this Jetson> <NMEA_PORT>`). This
process listens for it, keeps the latest fix, and writes `robot:gps` envelopes on stdout —
the same stream as the DDS reader, read by hec_shipper.py. Stdlib only: the Jetson has 3.8.

THE ONE TRAP: NMEA coordinates are ddmm.mmmm, not decimal degrees. 3438.166318 S is
34 + 38.166318/60 = -34.636105. Dividing by 100 instead gives a plausible number that is
wrong by ~30 km — the first fix (2026-10-07) is the regression test.

It accepts datagrams only from NMEA_SOURCE (the IR1101), so nothing else on the robot's
network can plant a position. It exits when its parent dies (run.sh), so a dead DDS reader
still takes the whole chain down for systemd to restart.

Getting the IR1101 to emit at all took a modem setting, not a config line: the antenna bias
was off (AT+WANT=0). See .claude/roadmap/GO2.md §5.5.
"""
from __future__ import annotations

import json
import os
import socket
import sys
import time

PORT = int(os.environ.get("NMEA_PORT", "0") or 0)
BIND = os.environ.get("NMEA_BIND", "0.0.0.0")  # noqa: S104  # filtered by NMEA_SOURCE below
SOURCE = os.environ.get("NMEA_SOURCE", "192.168.123.1")
PERIOD = float(os.environ.get("GPS_PERIOD", "5"))
STALE_S = float(os.environ.get("GPS_STALE_S", "30"))
ROBOT = os.environ.get("ROBOT_NAME", "go2")
INDEX = os.environ.get("HEC_INDEX", "")


def log(msg):
    print(f"[gps] {msg}", file=sys.stderr, flush=True)


def checksum_ok(sentence):
    """True for '$...*hh' whose XOR checksum matches. Damaged datagrams are dropped."""
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    body, _, given = sentence[1:].partition("*")
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(given[:2], 16)
    except ValueError:
        return False


def ddmm_to_decimal(value, hemisphere):
    """NMEA ddmm.mmmm (or dddmm.mmmm) plus N/S/E/W to signed decimal degrees, or None."""
    if not value or hemisphere not in ("N", "S", "E", "W"):
        return None
    try:
        raw = float(value)
    except ValueError:
        return None
    degrees = int(raw // 100)
    decimal = degrees + (raw - degrees * 100) / 60.0
    return -decimal if hemisphere in ("S", "W") else decimal


def _f(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


class GpsState:
    """The latest of each field, fed one sentence at a time, whatever the talker
    (GP GPS, GL GLONASS, GA Galileo, GN combined)."""

    def __init__(self):
        self.fix = 0
        self.lat = self.lon = None
        self.alt_m = self.hdop = None
        self.sats_used = {}          # talker -> satellites used in its fix, from GGA
        self.speed_kmh = self.course = None
        self.sats_view = {}          # talker -> satellites in view, from GSV
        self.last_fix_at = None      # monotonic time of the last sentence with a valid fix
        self.last_nmea_at = None     # monotonic time of the last valid sentence of any kind

    def feed(self, sentence, now):
        if not checksum_ok(sentence):
            return
        self.last_nmea_at = now
        f = sentence[1:].split("*")[0].split(",")
        talker, kind = f[0][:2], f[0][2:]
        if kind == "GGA" and len(f) >= 10:
            quality = int(f[6]) if f[6].isdigit() else 0
            if quality > 0:
                lat, lon = ddmm_to_decimal(f[2], f[3]), ddmm_to_decimal(f[4], f[5])
                if lat is not None and lon is not None:
                    self.fix, self.lat, self.lon = quality, lat, lon
                    if f[7].isdigit():  # each talker's GGA counts only its own system
                        self.sats_used[talker] = int(f[7])
                    self.hdop, self.alt_m = _f(f[8]), _f(f[9])
                    self.last_fix_at = now
            elif self.last_fix_at is None or now - self.last_fix_at > STALE_S:
                self.fix = 0
        elif kind == "RMC" and len(f) >= 9 and f[2] == "A":
            knots = _f(f[7])
            self.speed_kmh = None if knots is None else knots * 1.852
            self.course = _f(f[8])
        elif kind == "GSV" and len(f) >= 4 and f[3].isdigit():
            self.sats_view[talker] = int(f[3])

    def event(self, now):
        """The robot:gps event body. Position only while the fix is fresh."""
        fresh = self.last_fix_at is not None and now - self.last_fix_at <= STALE_S
        ev = {"fix": self.fix if fresh else 0, "sats_view": sum(self.sats_view.values()),
              "robot": ROBOT}
        if fresh:
            ev.update({"lat": round(self.lat, 7), "lon": round(self.lon, 7)})
            if self.sats_used:
                ev["sats_used"] = sum(self.sats_used.values())
            for key in ("alt_m", "hdop", "speed_kmh", "course"):
                val = getattr(self, key)
                if val is not None:
                    ev[key] = round(val, 2) if isinstance(val, float) else val
        if self.last_fix_at is not None:
            ev["fix_age_s"] = round(now - self.last_fix_at, 1)
        if self.last_nmea_at is not None:
            ev["nmea_age_s"] = round(now - self.last_nmea_at, 1)
        return ev


def envelope(body):
    env = {"time": round(time.time(), 3), "sourcetype": "robot:gps", "host": ROBOT,
           "event": body}
    if INDEX:
        env["index"] = INDEX
    return json.dumps(env, separators=(",", ":"))


def main():
    if PORT <= 0:
        log("NMEA_PORT unset: GPS reader disabled")
        return 0
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((BIND, PORT))
    sock.settimeout(1.0)
    parent = os.getppid()
    state = GpsState()
    log(f"listening udp {BIND}:{PORT} from {SOURCE}, period {PERIOD}s")
    next_emit, dropped = time.monotonic() + PERIOD, 0
    while True:
        if os.getppid() != parent:          # run.sh / the DDS reader is gone: so are we
            log("parent exited, stopping")
            return 0
        try:
            data, (src, _port) = sock.recvfrom(4096)
            if src != SOURCE:
                dropped += 1
                if dropped in (1, 100, 10000):
                    log(f"ignored datagram from {src} (only {SOURCE} is accepted), n={dropped}")
            else:
                now = time.monotonic()
                for line in data.decode("ascii", errors="replace").splitlines():
                    state.feed(line.strip(), now)
        except socket.timeout:
            pass
        now = time.monotonic()
        if now >= next_emit:
            next_emit = now + PERIOD
            if state.last_nmea_at is not None:      # nothing to report until NMEA arrives
                # One write per line, well under PIPE_BUF: lines from this process and the
                # DDS reader never interleave on the shared pipe.
                sys.stdout.write(envelope(state.event(now)) + "\n")
                sys.stdout.flush()


if __name__ == "__main__":
    sys.exit(main())
