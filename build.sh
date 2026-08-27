#!/usr/bin/env bash
# Build telemetry_reader against the prebuilt Unitree SDK. No cmake, no ROS2.
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

# Telemetry: read-only, subscribes and nothing else.
g++ -O2 -std=c++17 src/telemetry_reader.cpp -o telemetry_reader "${INCS[@]}" "${LIBS[@]}"
echo "built ./telemetry_reader ($ARCH)"
