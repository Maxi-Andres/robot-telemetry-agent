#!/usr/bin/env python3
"""
hec_shipper — reads HEC event envelopes (one JSON per line) on stdin and posts them to
Splunk in batches, with a disk spool so a dead link becomes a delay instead of a hole.

Runs on the robot next to telemetry_reader:  telemetry_reader | hec_shipper.py

Standard library only, so nothing has to be installed on the robot (Python 3.8 there).

Three properties that matter in the field:
  * Every event already carries its own `time`, set when it was read off DDS. Events that
    drain hours later still land at the correct timestamp in Splunk.
  * The spool is bounded and drops the OLDEST batch when full. Telemetry is perishable and
    the robot's disk is not ours to fill.
  * A daily byte cap that stops sending. The Splunk licence is shared with other users;
    this agent must not be able to eat it.
"""
import json
import os
import select
import ssl
import sys
import time
import urllib.error
import urllib.request

ROBOT = os.environ.get("ROBOT_NAME", "go2")
INDEX = os.environ.get("HEC_INDEX", "")
SELF_HEALTH_S = float(os.environ.get("SELF_HEALTH_S", "30"))
HEC_URL = os.environ.get("HEC_URL", "")
HEC_TOKEN = os.environ.get("HEC_TOKEN", "")
SPOOL_DIR = os.environ.get("SPOOL_DIR", "/var/tmp/robot-splunk-spool")
# Newest battery reading, for the relay to serve to the app. Empty disables it; the telemetry
# pipeline works exactly the same either way. See snapshot_battery().
BATTERY_FILE = os.environ.get("BATTERY_FILE", "/var/tmp/robot-battery.json")
SPOOL_MB = float(os.environ.get("SPOOL_MB", "50"))
DAILY_CAP = int(os.environ.get("DAILY_BYTE_CAP", str(150 * 1024 * 1024)))
BATCH_N = int(os.environ.get("BATCH_N", "20"))
BATCH_MS = float(os.environ.get("BATCH_MS", "2000"))
VERIFY_TLS = os.environ.get("VERIFY_TLS", "0") == "1"
TIMEOUT = float(os.environ.get("HTTP_TIMEOUT", "5"))

_ctx = ssl.create_default_context()
if not VERIFY_TLS:                      # Splunk ships a self-signed cert by default
    _ctx.check_hostname = False
    _ctx.verify_mode = ssl.CERT_NONE


def log(msg):
    print(f"[shipper] {msg}", file=sys.stderr, flush=True)


class Spool:
    """Bounded directory of pending batch files, oldest-first, dropped oldest-first."""

    def __init__(self, path, max_bytes):
        self.path = path
        self.max_bytes = max_bytes
        self.seq = 0
        os.makedirs(path, exist_ok=True)

    def files(self):
        return sorted(f for f in os.listdir(self.path) if f.endswith(".ndjson"))

    def size(self):
        return sum(os.path.getsize(os.path.join(self.path, f)) for f in self.files())

    def put(self, body):
        self.seq += 1
        name = f"{int(time.time()*1000):015d}-{self.seq:05d}.ndjson"
        with open(os.path.join(self.path, name), "wb") as fh:
            fh.write(body)
        dropped = 0
        while self.size() > self.max_bytes:
            oldest = self.files()
            if len(oldest) <= 1:
                break                    # never drop the batch we just wrote
            os.unlink(os.path.join(self.path, oldest[0]))
            dropped += 1
        if dropped:
            log(f"spool full ({self.max_bytes} B): dropped {dropped} oldest batch(es)")

    def pop_oldest(self):
        fs = self.files()
        if not fs:
            return None, None
        full = os.path.join(self.path, fs[0])
        with open(full, "rb") as fh:
            return full, fh.read()

    def drop(self, full):
        try:
            os.unlink(full)
        except FileNotFoundError:
            pass


class Sender:
    def __init__(self):
        self.sent_bytes = 0
        self.day = time.gmtime().tm_yday
        self.capped = False
        self.backoff = 1.0

    def _roll_day(self):
        today = time.gmtime().tm_yday
        if today != self.day:
            log(f"new UTC day: byte counter reset (was {self.sent_bytes} B)")
            self.day, self.sent_bytes, self.capped = today, 0, False

    def post(self, body):
        """True if Splunk accepted it. False means: spool it and retry later."""
        self._roll_day()
        if self.sent_bytes + len(body) > DAILY_CAP:
            if not self.capped:
                log(f"DAILY CAP reached ({DAILY_CAP} B) — dropping until UTC midnight")
                self.capped = True
            return True                  # deliberately not spooled: the cap is a stop
        req = urllib.request.Request(
            HEC_URL, data=body,
            headers={"Authorization": f"Splunk {HEC_TOKEN}",
                     "Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=TIMEOUT, context=_ctx) as r:
                r.read()
            self.sent_bytes += len(body)
            self.backoff = 1.0
            return True
        except urllib.error.HTTPError as e:
            detail = e.read()[:200].decode("utf-8", "replace")
            # 4xx will never succeed on retry (bad token, bad index): do not spool it.
            if 400 <= e.code < 500:
                log(f"HTTP {e.code} — NOT retrying: {detail}")
                return True
            log(f"HTTP {e.code}: {detail}")
        except Exception as e:
            log(f"post failed: {e}")
        self.backoff = min(self.backoff * 2, 60.0)
        return False


def snapshot_battery(line):
    """Keep the newest battery reading in a file the relay can serve.

    WHY HERE. The battery already crosses this process on its way to Splunk, so the app can
    have it without a second DDS subscriber, a second socket or a second service. The relay
    reads this file and folds it into its own /health, the way it already folds in the video
    publisher's — see `mjpeg_live()` there.

    WHY A FILE and not a socket: this process must never gain a listening port. It runs on the
    robot, it holds the HEC token, and its whole security story is that it only ever makes
    OUTBOUND connections. A file that one local service reads keeps that true.

    Written atomically (tmp + rename), so a reader can never see half a document.

    NEVER RAISES. Shipping telemetry is the job; a full disk or a read-only /var must not be
    able to stop it for the sake of a convenience field.
    """
    if not BATTERY_FILE:
        return
    try:
        ev = json.loads(line)
    except (ValueError, TypeError):
        return
    # Every level checked, not just the outer one: `{"event": "..."}` is valid JSON and made
    # this raise AttributeError inside the shipping loop — caught by
    # test_a_malformed_line_is_ignored_rather_than_raised, which is the whole point of it.
    if not isinstance(ev, dict):
        return
    inner = ev.get("event")
    if not isinstance(inner, dict):
        return
    bat = inner.get("battery")
    if not isinstance(bat, dict):
        return
    try:
        # `at` is OUR clock when the reading was taken, so a consumer can tell a live value
        # from one frozen since the robot went quiet. A battery percentage that never changes
        # looks perfectly healthy and is the easiest stale reading to miss.
        doc = json.dumps({**bat, "at": round(time.time(), 3), "robot": ROBOT},
                         separators=(",", ":"))
        tmp = BATTERY_FILE + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(doc)
        os.replace(tmp, BATTERY_FILE)
    except OSError:
        pass


def main():
    if not (HEC_URL and HEC_TOKEN):
        sys.exit("HEC_URL and HEC_TOKEN are required")
    # A token with whitespace in it is always a bad token file, but urllib reports it as
    # "Invalid header value", which sends you looking in the wrong place.
    if HEC_TOKEN.strip() != HEC_TOKEN or any(c.isspace() for c in HEC_TOKEN):
        sys.exit(f"HEC_TOKEN contains whitespace ({HEC_TOKEN!r}) — the token file is "
                 f"probably corrupt. Rewrite it with:\n"
                 f"  printf '%s' 'YOUR-TOKEN' > ~/.splunk_hec_token")
    spool = Spool(SPOOL_DIR, int(SPOOL_MB * 1024 * 1024))
    sender = Sender()
    log(f"up: url={HEC_URL} spool={SPOOL_DIR} cap={DAILY_CAP}B")

    batch, last_flush, next_drain = [], time.time(), 0.0
    next_self = time.time() + SELF_HEALTH_S

    def self_health():
        """Own telemetry: the reader cannot report these — the counters live here."""
        ev = {"time": round(time.time(), 3), "sourcetype": "robot:shipper",
              "host": ROBOT,
              "event": {"sent_bytes_today": sender.sent_bytes,
                        "byte_cap": DAILY_CAP,
                        "capped": sender.capped,
                        "spool_files": len(spool.files()),
                        "spool_bytes": spool.size(),
                        "robot": ROBOT}}
        if INDEX:
            ev["index"] = INDEX
        batch.append(json.dumps(ev, separators=(",", ":")))

    def flush():
        nonlocal batch
        if not batch:
            return
        body = "".join(batch).encode()
        batch = []
        if not sender.post(body):
            spool.put(body)

    def drain_one():
        """One spooled batch per pass, so intake is never blocked by a big backlog."""
        full, body = spool.pop_oldest()
        if body is None:
            return None
        if sender.post(body):
            spool.drop(full)
            return True
        return False

    while True:
        # select() rather than a blocking readline(): the loop MUST keep turning when no
        # data arrives, or a spool backlog never drains while DDS is silent (robot just
        # booted, link was down) — which is precisely when there is a backlog to drain.
        ready, _, _ = select.select([sys.stdin], [], [], 0.5)
        if ready:
            line = sys.stdin.readline()
            if not line:                 # reader exited
                flush()
                for _ in range(200):     # bounded best-effort drain on the way out
                    if drain_one() is not True:
                        break
                log("stdin closed, exiting")
                return
            line = line.strip()
            if line:
                snapshot_battery(line)
                batch.append(line)

        now = time.time()
        if now >= next_self:
            next_self = now + SELF_HEALTH_S
            self_health()

        if batch and (len(batch) >= BATCH_N or (now - last_flush) * 1000 >= BATCH_MS):
            flush()
            last_flush = now

        if now >= next_drain:
            r = drain_one()
            if r is True:
                next_drain = 0.0          # keep draining while it works
            elif r is False:
                next_drain = now + sender.backoff
            else:
                next_drain = now + 1.0


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        log("interrupted")
