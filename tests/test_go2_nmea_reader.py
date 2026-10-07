"""gps/go2_nmea_reader.py — NMEA from the IR1101 into robot:gps events.

Every test names the defect it catches. The sentences are the ones the IR1101 sent on
2026-10-07, so the regression values are the field kit's real first fix.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "gps"))

import go2_nmea_reader as r

FIRST_FIX = "$GAGGA,133556.00,3438.166318,S,05823.943146,W,1,00,0.9,49.7,M,23.0,M,,*41"
NO_FIX = "$GPGGA,,,,,,0,,,,,,,,*66"


def test_coordinates_are_converted_from_ddmm_not_divided_by_100():
    """Catches the ~30 km error: 3438.166318 S is -34.636105, not -34.38."""
    assert abs(r.ddmm_to_decimal("3438.166318", "S") - (-34.636105)) < 1e-6
    assert abs(r.ddmm_to_decimal("05823.943146", "W") - (-58.399052)) < 1e-6


def test_the_first_real_fix_parses():
    s = r.GpsState()
    s.feed(FIRST_FIX, now=100.0)
    ev = s.event(now=101.0)
    assert ev["fix"] == 1
    assert abs(ev["lat"] + 34.636105) < 1e-6 and abs(ev["lon"] + 58.399052) < 1e-6
    assert ev["hdop"] == 0.9 and ev["alt_m"] == 49.7


def test_a_damaged_sentence_is_dropped():
    """Catches a corrupted datagram planting a wrong position: the checksum must match."""
    bad = FIRST_FIX.replace("3438", "3538")
    s = r.GpsState()
    s.feed(bad, now=1.0)
    assert s.last_fix_at is None and s.event(now=2.0)["fix"] == 0


def test_no_fix_reports_no_position():
    s = r.GpsState()
    s.feed(NO_FIX, now=1.0)
    ev = s.event(now=2.0)
    assert ev["fix"] == 0 and "lat" not in ev


def test_a_stale_fix_stops_reporting_the_position():
    """Catches the map showing a frozen point as live after the GPS lost the sky."""
    s = r.GpsState()
    s.feed(FIRST_FIX, now=0.0)
    ev = s.event(now=r.STALE_S + 1)
    assert ev["fix"] == 0 and "lat" not in ev and ev["fix_age_s"] > r.STALE_S


def test_satellites_in_view_add_up_across_constellations():
    s = r.GpsState()
    for gsv in ("$GPGSV,3,1,09,05,,,31.9*55", "$GLGSV,1,1,04,76,,,36.8*5F"):
        body = gsv[1:].split("*")[0]
        calc = 0
        for ch in body:
            calc ^= ord(ch)
        s.feed(f"${body}*{calc:02X}", now=1.0)
    assert s.event(now=1.0)["sats_view"] == 13


def test_the_envelope_is_a_robot_gps_hec_event():
    import json
    env = json.loads(r.envelope({"fix": 0}))
    assert env["sourcetype"] == "robot:gps" and env["event"] == {"fix": 0}


def test_satellites_used_add_up_across_talkers():
    """Catches 'sats_used: 2' with 22 in view: each talker's GGA counts only its own system."""
    def sign(body):
        calc = 0
        for ch in body:
            calc ^= ord(ch)
        return f"${body}*{calc:02X}"
    s = r.GpsState()
    s.feed(sign("GPGGA,133556.00,3438.166318,S,05823.943146,W,1,07,0.9,49.7,M,23.0,M,,"), now=1.0)
    s.feed(sign("GAGGA,133556.00,3438.166318,S,05823.943146,W,1,02,0.9,49.7,M,23.0,M,,"), now=1.0)
    assert s.event(now=1.5)["sats_used"] == 9
