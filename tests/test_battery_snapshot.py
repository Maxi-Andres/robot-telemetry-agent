"""`hec_shipper.snapshot_battery` — the newest battery reading, for the app.

WHY IT IS WORTH TESTING. This function sits in the middle of the telemetry pipeline, on the
robot, in the process that holds the HEC token. Its whole contract is that it is INVISIBLE:
whatever it is handed, shipping must carry on. A snapshot that raises on a malformed line, or
on a read-only filesystem, would stop telemetry to add a convenience field — the worst trade
in this repo.

The second thing each test guards is the STALENESS marker. A battery percentage that stops
updating looks perfectly healthy; `at` is what lets a consumer tell a live reading from one
frozen since the robot went quiet.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "shipper"))

import hec_shipper


def line(**battery):
    return json.dumps({"time": 1.0, "sourcetype": "robot:vitals",
                       "event": {"battery": battery, "power": {}}})


def test_a_battery_event_is_written_atomically_with_a_timestamp(tmp_path, monkeypatch):
    out = tmp_path / "battery.json"
    monkeypatch.setattr(hec_shipper, "BATTERY_FILE", str(out))
    hec_shipper.snapshot_battery(line(soc=95, current=471, cycles=6))
    doc = json.loads(out.read_text())
    assert doc["soc"] == 95 and doc["current"] == 471
    assert doc["at"] > 0, "without `at` a frozen reading is indistinguishable from a live one"
    assert not list(tmp_path.glob("*.tmp")), "the temporary file must not be left behind"


def test_the_newest_reading_replaces_the_previous_one():
    """The file is a snapshot, not a log: a consumer asking for the battery wants the last
    value, not a history it has to parse."""
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "b.json"
        hec_shipper.BATTERY_FILE = str(out)
        try:
            hec_shipper.snapshot_battery(line(soc=95))
            hec_shipper.snapshot_battery(line(soc=94))
            assert json.loads(out.read_text())["soc"] == 94
        finally:
            hec_shipper.BATTERY_FILE = "/var/tmp/robot-battery.json"


def test_an_event_without_a_battery_writes_nothing(tmp_path, monkeypatch):
    """Most events on this stream are pose, health or the shipper's own counters. Writing on
    every line would be pointless disk traffic on an embedded machine."""
    out = tmp_path / "battery.json"
    monkeypatch.setattr(hec_shipper, "BATTERY_FILE", str(out))
    hec_shipper.snapshot_battery(json.dumps({"event": {"pose": {"x": 1}}}))
    assert not out.exists()


def test_a_malformed_line_is_ignored_rather_than_raised(tmp_path, monkeypatch):
    """THE one that matters: this runs inside the shipping loop. A line that is not JSON — a
    truncated write from the reader, a log line on the wrong stream — must not be able to stop
    telemetry."""
    monkeypatch.setattr(hec_shipper, "BATTERY_FILE", str(tmp_path / "b.json"))
    for bad in ("", "not json", "[1,2,3]", '{"event": "a string"}', '{"event":{"battery":7}}'):
        hec_shipper.snapshot_battery(bad)          # must not raise


def test_an_unwritable_path_is_ignored_rather_than_raised(monkeypatch):
    """A read-only or missing directory must cost the battery field, never the telemetry."""
    monkeypatch.setattr(hec_shipper, "BATTERY_FILE", "/nonexistent-dir/battery.json")
    hec_shipper.snapshot_battery(line(soc=50))     # must not raise


def test_it_can_be_turned_off(monkeypatch):
    """Empty BATTERY_FILE disables it, so a deployment that does not want the file on disk
    keeps the pipeline unchanged."""
    monkeypatch.setattr(hec_shipper, "BATTERY_FILE", "")
    hec_shipper.snapshot_battery(line(soc=50))     # must not raise, must not write
