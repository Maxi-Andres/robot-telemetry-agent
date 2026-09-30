// ndjson.hpp — the output half every telemetry reader shares: env parsing and HEC
// envelope emission, one event per stdout line.
//
// Every value is numeric and every key is a literal, so no JSON string escaping is needed.
// The only strings written are the robot name and the index, both set by the operator.
#pragma once

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>

inline std::string g_robot, g_index;

inline double now_s() {
    using namespace std::chrono;
    return duration<double>(system_clock::now().time_since_epoch()).count();
}

inline const char* env_s(const char* k, const char* d) {
    const char* v = getenv(k);
    return (v && *v) ? v : d;
}
inline double env_d(const char* k, double d) {
    const char* v = getenv(k);
    return (v && *v) ? atof(v) : d;
}

// Trims trailing zeros so 0.1400 serialises as 0.14. Over 40 MB/day of events those
// bytes are licence cost, not cosmetics.
inline std::string num(double v, int prec) {
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

inline void emit(const char* sourcetype, const std::string& body) {
    Obj e;
    e.f("time", now_s(), 3);
    e.raw("sourcetype", std::string("\"") + sourcetype + "\"");
    e.raw("host", "\"" + g_robot + "\"");
    if (!g_index.empty()) e.raw("index", "\"" + g_index + "\"");
    e.raw("event", body);
    printf("{%s}\n", e.s.c_str());
    fflush(stdout);
}
