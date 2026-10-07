#!/usr/bin/env bash
# Wire the two halves:  DDS -> curated NDJSON -> batched HEC POST.
#
# The reader dying must take the shipper with it (and vice versa) so systemd restarts the
# whole chain rather than leaving half of it running: hence pipefail + the explicit wait.
set -uo pipefail
cd "$(dirname "$0")"

# ROBOT_MODEL picks the reader; the rest of the chain is the same for both robots.
ROBOT_MODEL="${ROBOT_MODEL:-go2}"
case "$ROBOT_MODEL" in
  go2) READER=./go2_telemetry_reader ;;
  g1)  READER=./g1_telemetry_reader ;;
  *)   echo "ROBOT_MODEL must be go2 or g1 (got '$ROBOT_MODEL')" >&2; exit 1 ;;
esac

export DDS_IFACE="${DDS_IFACE:-eth0}"          # the robot's internal bus
export ROBOT_NAME="${ROBOT_NAME:-$ROBOT_MODEL}"
export PERIOD="${PERIOD:-3.0}"
export HEC_INDEX="${HEC_INDEX:-$ROBOT_MODEL-robot-data}"
export HEC_URL="${HEC_URL:?set HEC_URL}"
export SPOOL_DIR="${SPOOL_DIR:-/var/tmp/robot-splunk-spool}"

# Token from a file by preference: it never appears in the process list or in shell
# history that way.
TOKEN_FILE="${TOKEN_FILE:-$HOME/.splunk_hec_token}"
if [ -z "${HEC_TOKEN:-}" ] && [ -r "$TOKEN_FILE" ]; then
  # tr, not plain cat: a trailing newline or a CR from a pasted value would end up in the
  # Authorization header and fail with a message that blames the header, not the file.
  HEC_TOKEN="$(tr -d '[:space:]' < "$TOKEN_FILE")"
fi
export HEC_TOKEN="${HEC_TOKEN:?set HEC_TOKEN or create $TOKEN_FILE}"

# GPS (the Go2 field kit's IR1101 streams NMEA over UDP): a second producer on the same pipe,
# only when NMEA_PORT is set. It is a child of the reader's process (the exec keeps the PID),
# and exits when that dies, so a dead DDS reader still ends the chain for systemd to restart.
if [ "${NMEA_PORT:-0}" != 0 ]; then
  ( python3 gps/go2_nmea_reader.py & exec "$READER" ) | python3 shipper/hec_shipper.py
else
  exec "$READER" | python3 shipper/hec_shipper.py
fi
