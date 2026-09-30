// telemetry_reader — Unitree Go2 DDS -> curated NDJSON on stdout.
//
// Runs ON THE ROBOT (its high-level Jetson), which is the only machine guaranteed to be
// L2-adjacent to the robot's DDS no matter what network the robot is on. Measured proof
// that a routed reader cannot work: 122 topics from the robot's subnet, 2 from another
// one, 3 with explicit unicast peers. See RED-Y-DDS.md.
//
// Deliberately native SDK, not ROS2: the SDK ships a prebuilt aarch64 library, so this
// is a single binary with no runtime stack to install on the robot. ROS2's generic
// message introspection is useless here because we hand-pick fields anyway.
//
// Subscribes to the /lf/* (low-frequency) topics on purpose: rt/lf/lowstate carries the
// same payload as rt/lowstate at 20 Hz instead of 500 Hz — same 1.18 KB message, 25x
// less traffic (measured, see CENSO-GO2.md).
//
// Output is one HEC event envelope per line. Every value is numeric and every key is a
// literal, so no JSON string escaping is needed anywhere in this file.

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <thread>

using namespace unitree::robot;
using LowState = unitree_go::msg::dds_::LowState_;
using SportState = unitree_go::msg::dds_::SportModeState_;

// The Go2 has 12 real joints; motor_state[] carries 20 slots.
static const int NJOINT = 12;
static const char* JOINT[NJOINT] = {
    "FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"};

static std::mutex g_mu;
static LowState g_low;
static SportState g_sport;
static bool g_have_low = false, g_have_sport = false;
static std::atomic<long> g_n_low{0}, g_n_sport{0};
static std::atomic<double> g_last_low{0};

static std::string g_robot, g_index;
static double g_period, g_health_period, g_temp_warn, g_down_after;

static double now_s() {
    using namespace std::chrono;
    return duration<double>(system_clock::now().time_since_epoch()).count();
}

static const char* env_s(const char* k, const char* d) {
    const char* v = getenv(k);
    return (v && *v) ? v : d;
}
static double env_d(const char* k, double d) {
    const char* v = getenv(k);
    return (v && *v) ? atof(v) : d;
}

// ---------- minimal JSON emission ----------

// Trims trailing zeros so 0.1400 serialises as 0.14. Over 40 MB/day of events those
// bytes are licence cost, not cosmetics.
static std::string num(double v, int prec) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%.*f", prec, v);
    std::string s(buf);
    if (s.find('.') != std::string::npos) {
        while (s.back() == '0') s.pop_back();
        if (s.back() == '.') s.pop_back();
    }
    if (s == "-0") s = "0";
    return s;
}

struct Obj {
    std::string s;
    bool first = true;
    void key(const char* k) {
        if (!first) s += ',';
        first = false;
        s += '"'; s += k; s += "\":";
    }
    void i(const char* k, long long v) { key(k); s += std::to_string(v); }
    void f(const char* k, double v, int prec) { key(k); s += num(v, prec); }
    void b(const char* k, bool v) { key(k); s += v ? "true" : "false"; }
    void raw(const char* k, const std::string& v) { key(k); s += v; }
};

static void emit(const char* sourcetype, const std::string& body) {
    Obj e;
    e.f("time", now_s(), 3);
    e.raw("sourcetype", std::string("\"") + sourcetype + "\"");
    e.raw("host", "\"" + g_robot + "\"");
    if (!g_index.empty()) e.raw("index", "\"" + g_index + "\"");
    e.raw("event", body);
    printf("{%s}\n", e.s.c_str());
    fflush(stdout);
}

// ---------- event builders ----------

static std::string build_vitals(const LowState& m) {
    const auto& bms = m.bms_state();
    const auto& imu = m.imu_state();

    int tmax = 0;
    for (int i = 0; i < NJOINT; ++i)
        tmax = std::max<int>(tmax, m.motor_state()[i].temperature());

    long cell_sum = 0;
    for (auto v : bms.cell_vol()) cell_sum += v;
    int mcu = 0, bq = 0;
    for (auto v : bms.mcu_ntc()) mcu = std::max<int>(mcu, v);
    for (auto v : bms.bq_ntc()) bq = std::max<int>(bq, v);

    Obj bat;
    bat.i("soc", bms.soc());
    bat.i("current", bms.current());
    bat.i("cycles", bms.cycle());
    bat.i("volt_mv", cell_sum);
    bat.i("mcu_ntc", mcu);
    bat.i("bq_ntc", bq);

    Obj pw;
    pw.f("volt", m.power_v(), 2);
    pw.f("amp", m.power_a(), 2);

    Obj im;
    im.f("roll", imu.rpy()[0], 4);
    im.f("pitch", imu.rpy()[1], 4);
    im.f("yaw", imu.rpy()[2], 4);
    im.i("temp", imu.temperature());

    Obj tp;
    tp.i("motor_max", tmax);
    tp.i("ntc1", m.temperature_ntc1());
    tp.i("ntc2", m.temperature_ntc2());

    std::string ff = "[";
    for (int i = 0; i < 4; ++i) {
        if (i) ff += ',';
        ff += std::to_string(m.foot_force()[i]);
    }
    ff += ']';

    Obj o;
    o.raw("battery", "{" + bat.s + "}");
    o.raw("power", "{" + pw.s + "}");
    o.raw("imu", "{" + im.s + "}");
    o.raw("temp", "{" + tp.s + "}");
    o.raw("foot_force", ff);
    o.i("bit_flag", m.bit_flag());
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
        j.i("temp", mo.temperature());
        j.i("lost", mo.lost());
        o.raw(JOINT[i], "{" + j.s + "}");
    }
    o.raw("robot", "\"" + g_robot + "\"");
    return "{" + o.s + "}";
}

static std::string build_pose(const SportState& s) {
    Obj o;
    std::string pos = "[", vel = "[";
    for (int i = 0; i < 3; ++i) {
        if (i) { pos += ','; vel += ','; }
        pos += num(s.position()[i], 3);
        vel += num(s.velocity()[i], 3);
    }
    pos += ']'; vel += ']';
    o.raw("position", pos);
    o.raw("velocity", vel);
    o.f("yaw_speed", s.yaw_speed(), 3);
    o.f("body_height", s.body_height(), 3);
    o.i("mode", s.mode());
    o.i("gait_type", s.gait_type());
    o.i("error_code", s.error_code());
    o.raw("robot", "\"" + g_robot + "\"");
    return "{" + o.s + "}";
}

// ---------- discrete change detection ----------

static std::map<std::string, long long> g_prev;

static void emit_changes(const LowState* low, const SportState* sport, bool have_low,
                         bool have_sport) {
    std::map<std::string, long long> cur;
    if (have_sport) {
        cur["mode"] = sport->mode();
        cur["gait_type"] = sport->gait_type();
        cur["error_code"] = sport->error_code();
    }
    if (have_low) {
        int tmax = 0;
        for (int i = 0; i < NJOINT; ++i)
            tmax = std::max<int>(tmax, low->motor_state()[i].temperature());
        cur["motor_over_temp"] = (tmax >= g_temp_warn) ? 1 : 0;
    }
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
    g_robot = env_s("ROBOT_NAME", "go2");
    g_index = env_s("HEC_INDEX", "");
    g_period = env_d("PERIOD", 3.0);
    g_health_period = env_d("HEALTH_PERIOD", 10.0);
    g_temp_warn = env_d("TEMP_WARN", 80);
    g_down_after = env_d("DOWN_AFTER", 5.0);

    fprintf(stderr, "[reader] iface=%s robot=%s period=%.1fs\n",
            iface.c_str(), g_robot.c_str(), g_period);

    // Binding CycloneDDS to the interface is not optional: Init(0, iface) alone does not
    // make the SDK receive anything. Hard-won detail, same as robot-video-pipeline.
    ChannelFactory::Instance()->Init(0, iface);

    ChannelSubscriberPtr<LowState> sub_low(
        new ChannelSubscriber<LowState>("rt/lf/lowstate"));
    sub_low->InitChannel([](const void* msg) {
        std::lock_guard<std::mutex> lk(g_mu);
        g_low = *(const LowState*)msg;
        g_have_low = true;
        g_n_low++;
        g_last_low = now_s();
    }, 1);

    ChannelSubscriberPtr<SportState> sub_sport(
        new ChannelSubscriber<SportState>("rt/lf/sportmodestate"));
    sub_sport->InitChannel([](const void* msg) {
        std::lock_guard<std::mutex> lk(g_mu);
        g_sport = *(const SportState*)msg;
        g_have_sport = true;
        g_n_sport++;
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
            SportState sport;
            bool hl, hs;
            {
                std::lock_guard<std::mutex> lk(g_mu);
                low = g_low; sport = g_sport;
                hl = g_have_low; hs = g_have_sport;
            }
            if (hl) {
                emit("robot:vitals", build_vitals(low));
                emit("robot:motors", build_motors(low));
            }
            if (hs) emit("robot:pose", build_pose(sport));
            emit_changes(hl ? &low : nullptr, hs ? &sport : nullptr, hl, hs);
        }

        if (t >= next_health) {
            next_health = t + g_health_period;
            const long nl = g_n_low.exchange(0), ns = g_n_sport.exchange(0);
            const double age = t - g_last_low.load();
            const bool now_alive = (g_last_low.load() > 0) && (age < g_down_after);

            Obj hz;
            hz.f("lowstate", nl / g_health_period, 1);
            hz.f("sportmodestate", ns / g_health_period, 1);
            Obj o;
            o.raw("topic_hz", "{" + hz.s + "}");
            o.b("dds_alive", now_alive);
            o.f("last_sample_age", g_last_low.load() > 0 ? age : -1, 1);
            o.raw("robot", "\"" + g_robot + "\"");
            emit("robot:health", "{" + o.s + "}");

            // A gap becomes a datum instead of a silent hole in the dashboard. The very
            // first determination is not a transition, though: emitting it would log a
            // fake "link came up" on every restart.
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
