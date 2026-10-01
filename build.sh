#!/usr/bin/env bash
# Build the telemetry readers (go2_ and g1_) against the prebuilt Unitree SDK. No cmake, no ROS2.
# Works unchanged on x86_64 (dev box) and aarch64 (the robot's Jetson) because the SDK
# ships a static library for both.
set -euo pipefail
cd "$(dirname "$0")"

SDK="${UNITREE_SDK2_DIR:-$HOME/unitree_sdk2}"
ARCH="$(uname -m)"

if [ ! -f "$SDK/lib/$ARCH/libunitree_sdk2.a" ]; then
  echo "error: $SDK/lib/$ARCH/libunitree_sdk2.a not found" >&2
  echo "set UNITREE_SDK2_DIR to the unitree_sdk2 checkout" >&2
  exit 1
fi

INCS=(-I"$SDK/include" -I"$SDK/thirdparty/include" -I"$SDK/thirdparty/include/ddscxx")
LIBS=("$SDK/lib/$ARCH/libunitree_sdk2.a" -L"$SDK/thirdparty/lib/$ARCH" -lddscxx -lddsc
      -Wl,-rpath,"$SDK/thirdparty/lib/$ARCH" -lpthread)

# Telemetry: read-only, subscribes and nothing else. One binary per robot model — the two
# IDLs (unitree_go, unitree_hg) are different message types, not options of one reader.
g++ -O2 -std=c++17 src/go2_telemetry_reader.cpp -o go2_telemetry_reader "${INCS[@]}" "${LIBS[@]}"
echo "built ./go2_telemetry_reader ($ARCH, Go2)"
g++ -O2 -std=c++17 src/g1_telemetry_reader.cpp -o g1_telemetry_reader "${INCS[@]}" "${LIBS[@]}"
echo "built ./g1_telemetry_reader ($ARCH, G1)"

# Flush to disk before returning. The robot is powered off by its switch, not shut down: on
# 2026-10-01 a power-off right after a build left BOTH binaries at 0 bytes, and the service
# then died on every start until systemd gave up (start-limit-hit).
sync
