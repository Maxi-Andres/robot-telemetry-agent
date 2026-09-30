// g1_telemetry_reader — Unitree G1 DDS -> curated NDJSON on stdout.
//
// The G1 sibling of go2_telemetry_reader.cpp. Same contract (sourcetypes, envelope, cadence,
// health and change events), because the G1 dashboard reuses the Go2's searches — see
// ROADMAP §6.5 and robot-splunk-docs/dashboards/g1-telemetria-thousandeyes.xml. What
// differs is the robot, not the design:
//
//   * IDL unitree_hg, not unitree_go. The Go2 types on a G1 topic receive nothing.
//   * 29 joints, not 12, and TWO temperature sensors per motor. `temp` is the hotter one:
//     the dashboard colours a joint by its worst reading, and a threshold on the cooler
//     sensor would hide exactly the motor that is overheating.
//   * The battery is its own topic (rt/lf/bmsstate). The Go2 carries it inside LowState.
//   * No SportModeState pose on the /lf set; the machine mode lives in LowState.
//
// Runs on PC2 (the Jetson, .164): PC1 has no SSH, and PC2 is on the same internal bus, so
// it is L2-adjacent to PC1's DDS whatever network the robot is on — cable, WiFi or CURWB.
//
// Measured on the robot 2026-09-30, cable, standing: rt/lf/lowstate 20.0 Hz (rt/lowstate is
// 1039 Hz, same message), rt/lf/bmsstate 20.0 Hz, motors 0-28 live and 29-34 all zero.

#include "ndjson.hpp"

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include <algorithm>
#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <thread>

using namespace unitree::robot;
using LowState = unitree_hg::msg::dds_::LowState_;
using BmsState = unitree_hg::msg::dds_::BmsState_;

// motor_state[0..28] in the SDK's G1 29-DoF order, names without "_joint". These are the
// field prefixes the dashboard's motor panel expects; do not rename one without the other.
static const int NJOINT = 29;
static const char* JOINT[NJOINT] = {
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
    "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee",
    "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw"};

static std::mutex g_mu;
static LowState g_low;
static BmsState g_bms;
static bool g_have_low = false, g_have_bms = false;
static std::atomic<long> g_n_low{0}, g_n_bms{0};
static std::atomic<double> g_last_low{0};

static double g_period, g_health_period, g_temp_warn, g_down_after;

static int motor_temp(const LowState& m, int i) {
    const auto& t = m.motor_state()[i].temperature();
    return std::max<int>(t[0], t[1]);
}

static int motor_temp_max(const LowState& m) {
    int tmax = 0;
    for (int i = 0; i < NJOINT; ++i) tmax = std::max(tmax, motor_temp(m, i));
    return tmax;
}

// motorstate is a bit field of fault flags; any non-zero value is a motor reporting a
// problem. Counted, not decoded: the bit meanings are not documented in the SDK.
static int motor_err_count(const LowState& m) {
    int n = 0;
    for (int i = 0; i < NJOINT; ++i) n += m.motor_state()[i].motorstate() != 0;
    return n;
}

// ---------- event builders ----------

static std::string build_vitals(const LowState& m, const BmsState* bms) {
    const auto& imu = m.imu_state();

    // Unused sensor slots read 0, so a max over all twelve is the hottest real one.
    int tbat = 0;
    if (bms) for (auto v : bms->temperature()) tbat = std::max<int>(tbat, v);

    Obj o;
    if (bms) {
        long cell_sum = 0;
        int cells = 0;
        for (auto v : bms->cell_vol()) if (v) { cell_sum += v; ++cells; }

        Obj bat;
        bat.i("soc", bms->soc());
        bat.i("soh", bms->soh());
        bat.i("current", bms->current());          // mA, negative = discharging
        bat.i("cycles", bms->cycle());
        bat.i("volt_mv", cell_sum);
        bat.i("cells", cells);
        bat.i("temp_max", tbat);
        o.raw("battery", "{" + bat.s + "}");

        // Same names as the Go2's power.* so the charts are one search. bmsvoltage[0] is the
        // pack voltage in mV; it agrees with the cell sum within ~60 mV on the robot.
        Obj pw;
        pw.f("volt", bms->bmsvoltage()[0] / 1000.0, 2);
        pw.f("current", bms->current() / 1000.0, 2);
        o.raw("power", "{" + pw.s + "}");
    }

    Obj im;
    im.f("roll", imu.rpy()[0], 4);
    im.f("pitch", imu.rpy()[1], 4);
    im.f("yaw", imu.rpy()[2], 4);
    im.i("temp", imu.temperature());
    o.raw("imu", "{" + im.s + "}");

    Obj tp;
    tp.i("motor_max", motor_temp_max(m));
    if (bms) tp.i("bms_max", tbat);
    o.raw("temp", "{" + tp.s + "}");

    Obj mo;
    mo.i("err_count", motor_err_count(m));
    o.raw("motor", "{" + mo.s + "}");

    o.i("mode_machine", m.mode_machine());
    o.i("mode_pr", m.mode_pr());
    o.raw("robot", "\"" + g_robot + "\"");
    return "{" + o.s + "}";
}

static std::string build_motors(const LowState& m) {
    Obj o;
    for (int i = 0; i < NJOINT; ++i) {
        const auto& mo = m.motor_state()[i];
        Obj j;
        j.f("q", mo.q(), 4);
        j.f("tau", mo.tau_est(), 3);
        j.i("temp", motor_temp(m, i));
        if (mo.motorstate()) j.i("err", mo.motorstate());   // only when set: licence bytes
        o.raw(JOINT[i], "{" + j.s + "}");
    }
    o.raw("robot", "\"" + g_robot + "\"");
    return "{" + o.s + "}";
}

// ---------- discrete change detection ----------

static std::map<std::string, long long> g_prev;

static void emit_changes(const LowState& low) {
    std::map<std::string, long long> cur;
    cur["mode_machine"] = low.mode_machine();
    cur["mode_pr"] = low.mode_pr();
    cur["motor_over_temp"] = (motor_temp_max(low) >= g_temp_warn) ? 1 : 0;
    cur["motor_err_count"] = motor_err_count(low);
    for (const auto& kv : cur) {
        auto it = g_prev.find(kv.first);
        if (it != g_prev.end() && it->second != kv.second) {
            Obj o;
            o.raw("kind", "\"" + kv.first + "\"");
            o.i("prev", it->second);
            o.i("curr", kv.second);
            o.raw("robot", "\"" + g_robot + "\"");
            emit("robot:event", "{" + o.s + "}");
        }
        g_prev[kv.first] = kv.second;
    }
}

int main() {
    const std::string iface = env_s("DDS_IFACE", "eth0");
    g_robot = env_s("ROBOT_NAME", "g1");
    g_index = env_s("HEC_INDEX", "");
    g_period = env_d("PERIOD", 3.0);
    g_health_period = env_d("HEALTH_PERIOD", 10.0);
    g_temp_warn = env_d("TEMP_WARN", 80);
    g_down_after = env_d("DOWN_AFTER", 5.0);

    fprintf(stderr, "[g1-reader] iface=%s robot=%s period=%.1fs\n",
            iface.c_str(), g_robot.c_str(), g_period);

    // Binding CycloneDDS to the interface is not optional: Init(0, iface) alone does not
    // make the SDK receive anything. eth0 is PC2's side of the robot's internal bus, and it
    // keeps carrier with the external cable unplugged.
    ChannelFactory::Instance()->Init(0, iface);

    ChannelSubscriberPtr<LowState> sub_low(new ChannelSubscriber<LowState>("rt/lf/lowstate"));
    sub_low->InitChannel([](const void* msg) {
        std::lock_guard<std::mutex> lk(g_mu);
        g_low = *(const LowState*)msg;
        g_have_low = true;
        g_n_low++;
        g_last_low = now_s();
    }, 1);

    ChannelSubscriberPtr<BmsState> sub_bms(new ChannelSubscriber<BmsState>("rt/lf/bmsstate"));
    sub_bms->InitChannel([](const void* msg) {
        std::lock_guard<std::mutex> lk(g_mu);
        g_bms = *(const BmsState*)msg;
        g_have_bms = true;
        g_n_bms++;
    }, 1);

    double next_data = now_s() + g_period;
    double next_health = now_s() + g_health_period;
    int alive = -1;   // -1 = not determined yet

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const double t = now_s();

        if (t >= next_data) {
            next_data = t + g_period;
            LowState low;
            BmsState bms;
            bool hl, hb;
            {
                std::lock_guard<std::mutex> lk(g_mu);
                low = g_low; bms = g_bms;
                hl = g_have_low; hb = g_have_bms;
            }
            if (hl) {
                emit("robot:vitals", build_vitals(low, hb ? &bms : nullptr));
                emit("robot:motors", build_motors(low));
                emit_changes(low);
            }
        }

        if (t >= next_health) {
            next_health = t + g_health_period;
            const long nl = g_n_low.exchange(0), nb = g_n_bms.exchange(0);
            const double age = t - g_last_low.load();
            const bool now_alive = (g_last_low.load() > 0) && (age < g_down_after);

            Obj hz;
            hz.f("lowstate", nl / g_health_period, 1);
            hz.f("bmsstate", nb / g_health_period, 1);
            Obj o;
            o.raw("topic_hz", "{" + hz.s + "}");
            o.b("dds_alive", now_alive);
            o.f("last_sample_age", g_last_low.load() > 0 ? age : -1, 1);
            o.raw("robot", "\"" + g_robot + "\"");
            emit("robot:health", "{" + o.s + "}");

            // First determination is not a transition: emitting it would log a fake
            // "link came up" on every restart.
            if (alive != -1 && now_alive != (alive == 1)) {
                Obj e;
                e.raw("kind", "\"dds_link\"");
                e.i("prev", alive);
                e.i("curr", now_alive ? 1 : 0);
                e.raw("robot", "\"" + g_robot + "\"");
                emit("robot:event", "{" + e.s + "}");
            }
            alive = now_alive ? 1 : 0;
        }
    }
}
