# robot-telemetry-agent

Telemetry agent that reads a Unitree robot's DDS state and posts curated events to Splunk
over HTTPS. **It runs on the robot's own high-level computer**, not on a server.

**Read-only.** It subscribes and nothing else. The remote command path — the only thing that
can move the robot — lives in a separate repo, `robot-command-relay`, so that it is named
for what it does and can be audited on its own. The two used to share this repo because they
share a build recipe and a deploy target; they never shared code.

## Why on the robot

DDS cannot be read across a subnet boundary on these robots. Measured on a Go2:

| From | DDS topics visible |
|---|---|
| The robot's own subnet (`192.168.123.0/24`) | **122** |
| Another subnet, routed (ping works, 1.3 ms) | **2** |
| Another subnet with explicit unicast DDS peers | **3** |

The robot advertises only `192.168.123.x` locators and its low-level controller has no
route off that subnet, so no peer list, route or NAT fixes it. The only machine guaranteed
to be L2-adjacent to the robot's DDS — wherever the robot goes — is the robot itself.

So: **DDS short and local, HTTPS long and routed.** The agent extracts fields next to the
robot and only HTTPS leaves it, which traverses NAT, Starlink and any VLAN.

Full reasoning: `robot-splunk-docs/RED-Y-DDS.md`. Plan: `robot-splunk-docs/PLAN.md`.

## Shape

```
rt/lf/lowstate ─┐
                ├─▶ telemetry_reader (C++) ─NDJSON─▶ hec_shipper.py ─HTTPS─▶ Splunk HEC
rt/lf/sport…   ─┘   curated fields, decimated        batch + disk spool
```

- **`src/telemetry_reader.cpp`** — native Unitree SDK, no ROS2. Subscribes to the `/lf/*`
  topics (20 Hz, same payload as the 500 Hz ones — 25x less traffic), decimates to one
  event set every `PERIOD`, emits one HEC envelope per line on stdout.
- **`shipper/hec_shipper.py`** — stdlib only (the robot has Python 3.8). Batches, retries,
  spools to disk when the link is down, and enforces a daily byte cap.
- **`poc/telemetry_poc.py`** — throwaway Python version that runs in the ROS2 devcontainer
  on a workstation. Used to validate the data contract; not for the robot.

## Deploy to the robot

Clone both repos on the robot; the SDK comes straight from Unitree's upstream (this project
does not patch it) and the robot has internet, so nothing needs copying from a workstation.

```bash
ssh unitree@<robot-jetson>
git clone https://github.com/unitreerobotics/unitree_sdk2.git ~/unitree_sdk2
git clone <this-repo> ~/robot-telemetry-agent
cd ~/robot-telemetry-agent && ./build.sh
```

Updating later:

```bash
git pull && ./build.sh && sudo systemctl restart robot-telemetry-agent
```

`./build.sh` is **not** optional after a pull: the binary is gitignored, so a pull brings
new source without rebuilding it.

The token lives in `~/.splunk_hec_token`, outside the repo — a pull never overwrites it and
a push never leaks it.

Step-by-step with success criteria for each stage: `robot-splunk-docs/IMPLEMENTACION.md`, Etapa D.

## Build

```bash
UNITREE_SDK2_DIR=~/unitree_sdk2 ./build.sh     # x86_64 or aarch64, same command
```

## Run

```bash
./telemetry_reader                 # dry run: prints the JSON, sends nothing
HEC_URL=https://<splunk>:8088/services/collector/event ./run.sh
```

The token is read from `~/.splunk_hec_token` (mode 600) so it never lands in the process
list or shell history.

## Configuration

| Variable | Default | Meaning |
|---|---|---|
| `DDS_IFACE` | `eth0` | Interface CycloneDDS binds to. **Required** — `Init(0, iface)` alone receives nothing |
| `ROBOT_NAME` | `go2` | Goes into every event as `robot` and `host` |
| `PERIOD` | `3.0` | Seconds between event sets |
| `HEC_INDEX` | *(empty)* | Omitted when empty, so the token's default index applies |
| `TEMP_WARN` | `80` | °C threshold for the over-temperature event |
| `DAILY_BYTE_CAP` | `150 MB` | Hard stop, resets at UTC midnight |
| `SPOOL_MB` | `50` | Disk spool ceiling; oldest batch dropped when full |
| `VERIFY_TLS` | `0` | `1` once Splunk has a real certificate |

## Events

`robot:vitals` (battery, power, IMU, temps) · `robot:motors` (12 joints, flattened per
joint so Splunk can chart them) · `robot:pose` · `robot:health` (per-topic Hz, `dds_alive`,
spool state) · `robot:event` (mode / gait / error-code changes, over-temperature, DDS link
up-down — never decimated).

Measured cost: **40 MB/day** at `PERIOD=3`.

## Rules this agent follows

- **Read-only.** It subscribes and nothing else; it publishes to no command topic. It
  cannot move the robot.
- **Resource-capped** by systemd (`MemoryMax=256M`, `CPUQuota=25%`, `Nice=10`) so it can
  never compete with the control stack.
- **Bounded disk use.** The spool has a ceiling and drops oldest-first; it can never fill
  the robot's disk.
- **Self-limiting on licence.** The daily byte cap exists because the Splunk licence is
  shared with other users.
