#!/usr/bin/env python3
"""
Proof-of-concept telemetry collector: Unitree Go2 DDS -> Splunk HEC.

Runs inside the existing ROS2 Humble devcontainer, so it needs NO new dependency and
NO change on the robot. It is the throwaway step that proves the data path; the
production agent is a native C++ binary that runs on the robot's Jetson (see PLAN.md).

Reads the LOW-FREQUENCY topics on purpose: /lf/lowstate carries the same payload as
/lowstate at 20 Hz instead of 500 Hz (measured — see CENSO-GO2.md), then decimates
again to one event every PERIOD seconds.

Run it with --dry-run first: it prints the exact JSON it would send and sends nothing.

    source /workspace/setup.sh
    python3 telemetry_poc.py --dry-run

    HEC_URL=https://192.168.20.200:8088/services/collector/event \
    HEC_TOKEN=xxxx python3 telemetry_poc.py
"""
import argparse
import json
import os
import signal
import ssl
import sys
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import LowState, SportModeState

ROBOT = os.environ.get("ROBOT_NAME", "go2")
HEC_URL = os.environ.get("HEC_URL", "")
HEC_TOKEN = os.environ.get("HEC_TOKEN", "")
# Empty means: omit the field and let the token's default index decide. A token whose
# allowed-index list does not include the name you send is rejected with "Incorrect
# index" (code 7) — even for `main` — so omitting is the safe default.
INDEX = os.environ.get("HEC_INDEX", "")
PERIOD = float(os.environ.get("PERIOD", "3.0"))
# Self-imposed ceiling so this can never eat a shared Splunk licence.
DAILY_BYTE_CAP = int(os.environ.get("DAILY_BYTE_CAP", str(150 * 1024 * 1024)))
TEMP_WARN = float(os.environ.get("TEMP_WARN", "80"))

# Go2 leg joints, in the order the SDK reports them. Only the first 12 of the 20
# motor_state slots are real on this robot.
def _jsonable(o):
    """ROS2 array fields arrive as numpy scalars, which json cannot encode."""
    if hasattr(o, "item"):
        return o.item()
    raise TypeError(f"not JSON serializable: {type(o).__name__}")


JOINTS = ["FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
          "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"]


class Collector(Node):
    def __init__(self, dry_run):
        super().__init__("telemetry_poc")
        self.dry_run = dry_run
        self.low = None
        self.sport = None
        self.counts = {"lowstate": 0, "sportmodestate": 0}
        self.sent_bytes = 0
        self.events_sent = 0
        self.post_errors = 0
        self.capped = False
        self.prev = {}           # last seen discrete values, for change detection
        self.ctx = ssl.create_default_context()
        self.ctx.check_hostname = False
        self.ctx.verify_mode = ssl.CERT_NONE   # self-signed Splunk cert

        # BEST_EFFORT: matches Unitree's RELIABLE writers and never triggers a
        # retransmit storm if the link drops packets.
        self.create_subscription(LowState, "/lf/lowstate", self._on_low,
                                 qos_profile_sensor_data)
        self.create_subscription(SportModeState, "/lf/sportmodestate", self._on_sport,
                                 qos_profile_sensor_data)
        self.create_timer(PERIOD, self._emit)
        self.create_timer(10.0, self._emit_health)

    def _on_low(self, msg):
        self.low = msg
        self.counts["lowstate"] += 1

    def _on_sport(self, msg):
        self.sport = msg
        self.counts["sportmodestate"] += 1

    # ---------- event builders ----------

    def _vitals(self, m):
        bms, imu = m.bms_state, m.imu_state
        temps = [mo.temperature for mo in m.motor_state[:12]]
        return {
            "battery": {"soc": bms.soc, "current": bms.current, "cycles": bms.cycle,
                        "volt_mv": int(sum(bms.cell_vol)),
                        "mcu_ntc": int(max(bms.mcu_ntc)), "bq_ntc": int(max(bms.bq_ntc))},
            "power": {"volt": round(float(m.power_v), 2), "amp": round(float(m.power_a), 2)},
            "imu": {"roll": round(float(imu.rpy[0]), 4),
                    "pitch": round(float(imu.rpy[1]), 4),
                    "yaw": round(float(imu.rpy[2]), 4), "temp": int(imu.temperature)},
            "temp": {"motor_max": max(temps), "ntc1": m.temperature_ntc1,
                     "ntc2": m.temperature_ntc2},
            "foot_force": [int(v) for v in m.foot_force],
            "bit_flag": m.bit_flag,
        }

    def _motors(self, m):
        # Flattened per joint on purpose: an array cannot be charted in Splunk.
        out = {}
        for name, mo in zip(JOINTS, m.motor_state[:12]):
            out[name] = {"q": round(float(mo.q), 4), "tau": round(float(mo.tau_est), 3),
                         "temp": int(mo.temperature), "lost": int(mo.lost)}
        return out

    def _pose(self, s):
        return {
            "position": [round(float(v), 3) for v in s.position],
            "velocity": [round(float(v), 3) for v in s.velocity],
            "yaw_speed": round(float(s.yaw_speed), 3),
            "body_height": round(float(s.body_height), 3),
            "mode": s.mode, "gait_type": s.gait_type,
            "error_code": s.error_code,
        }

    # ---------- emission ----------

    def _emit(self):
        now = time.time()
        batch = []
        if self.low is not None:
            batch.append(self._event(now, "robot:vitals", self._vitals(self.low)))
            batch.append(self._event(now, "robot:motors", self._motors(self.low)))
        if self.sport is not None:
            batch.append(self._event(now, "robot:pose", self._pose(self.sport)))
            batch += self._discrete(now, self.sport, self.low)
        if batch:
            self._ship(batch)

    def _discrete(self, now, s, low):
        """One event per change of a discrete value — never decimated."""
        events = []
        watch = {"mode": s.mode, "gait_type": s.gait_type, "error_code": s.error_code}
        if low is not None:
            watch["motor_over_temp"] = max(
                mo.temperature for mo in low.motor_state[:12]) >= TEMP_WARN
        for key, val in watch.items():
            if key in self.prev and self.prev[key] != val:
                events.append(self._event(now, "robot:event", {
                    "kind": key, "prev": self.prev[key], "curr": val}))
            self.prev[key] = val
        return events

    def _emit_health(self):
        now = time.time()
        hz = {k: round(v / 10.0, 1) for k, v in self.counts.items()}
        self.counts = {k: 0 for k in self.counts}
        self._ship([self._event(now, "robot:health", {
            "topic_hz": hz,
            "dds_alive": hz["lowstate"] > 0,
            "sent_bytes_today": self.sent_bytes,
            "byte_cap": DAILY_BYTE_CAP,
            "capped": self.capped,
        })])

    def _event(self, ts, sourcetype, data):
        data["robot"] = ROBOT
        ev = {"time": round(ts, 3), "sourcetype": sourcetype,
              "host": ROBOT, "event": data}
        if INDEX:
            ev["index"] = INDEX
        return ev

    def _ship(self, batch):
        # HEC accepts many events in one POST as concatenated JSON objects.
        body = "".join(json.dumps(e, separators=(",", ":"),
                                 default=_jsonable) for e in batch).encode()

        if self.dry_run:
            for e in batch:
                print(json.dumps(e, separators=(",", ":"), default=_jsonable))
            sys.stdout.flush()
            self.sent_bytes += len(body)
            self.events_sent += len(batch)
            return

        if self.sent_bytes + len(body) > DAILY_BYTE_CAP:
            if not self.capped:
                self.get_logger().error(
                    f"daily byte cap reached ({DAILY_BYTE_CAP} B) — stopping sends")
                self.capped = True
            return

        req = urllib.request.Request(
            HEC_URL, data=body,
            headers={"Authorization": f"Splunk {HEC_TOKEN}",
                     "Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5, context=self.ctx) as r:
                r.read()
            self.sent_bytes += len(body)
            self.events_sent += len(batch)
        except Exception as exc:
            # A PoC drops on failure. The production agent spools to disk instead.
            self.post_errors += 1
            self.get_logger().warn(f"HEC post failed: {exc}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true",
                    help="print the events instead of posting them")
    args = ap.parse_args()
    if not args.dry_run and not (HEC_URL and HEC_TOKEN):
        sys.exit("HEC_URL and HEC_TOKEN are required unless --dry-run is given")

    # Make SIGTERM behave like Ctrl-C so `timeout`, `docker stop` or systemd end through
    # the normal cleanup path instead of tearing down rclpy mid-spin. Same trick as
    # unitree_ros2/robot_executor/g1_fsm_watch.py.
    signal.signal(signal.SIGTERM,
                  lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    rclpy.init()
    node = Collector(args.dry_run)
    print(f"[poc] robot={ROBOT} period={PERIOD}s "
          f"mode={'DRY-RUN' if args.dry_run else 'HEC -> ' + HEC_URL}", file=sys.stderr)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"[poc] events={node.events_sent} bytes={node.sent_bytes} "
              f"post_errors={node.post_errors}", file=sys.stderr)
        node.destroy_node()
        # SIGTERM (e.g. from `timeout` or systemd) can shut the context down before we
        # get here; calling shutdown twice raises and masks the real exit reason.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
