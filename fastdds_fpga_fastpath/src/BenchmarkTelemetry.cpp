// ============================================================================
// BenchmarkTelemetry.cpp
// ============================================================================

#include "BenchmarkTelemetry.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <time.h>

uint64_t monotonic_now_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

ProcMemInfo read_proc_mem_info() {
    ProcMemInfo info;
    std::ifstream f("/proc/self/status");
    if (!f.is_open()) return info;

    std::string line;
    while (std::getline(f, line)) {
        auto parse_kb = [&](const char* prefix, uint64_t& dst) {
            size_t len = std::strlen(prefix);
            if (line.compare(0, len, prefix) == 0) {
                std::istringstream iss(line.substr(len));
                iss >> dst;
                info.valid = true;
            }
        };
        parse_kb("VmRSS:", info.vm_rss_kb);
        parse_kb("VmHWM:", info.vm_hwm_kb);
        parse_kb("VmSize:", info.vm_size_kb);
        parse_kb("VmData:", info.vm_data_kb);
    }
    return info;
}

ProcStatCpu read_proc_stat_cpu() {
    ProcStatCpu s;
    std::ifstream f("/proc/stat");
    if (!f.is_open()) return s;

    std::string label;
    if (f >> label >> s.user >> s.nice >> s.system >> s.idle >> s.iowait >> s.irq >> s.softirq >> s.steal) {
        if (label == "cpu") s.valid = true;
    }
    return s;
}

ResourceSnapshot capture_resources() {
    ResourceSnapshot snap;
    snap.wall_ns = monotonic_now_ns();
    getrusage(RUSAGE_SELF, &snap.ru);
    snap.mem = read_proc_mem_info();
    snap.sys_cpu = read_proc_stat_cpu();
    return snap;
}

double tv_to_ms(const struct timeval& tv) {
    return static_cast<double>(tv.tv_sec) * 1000.0 + static_cast<double>(tv.tv_usec) / 1000.0;
}

double percentile_interp(const std::vector<double>& sorted, double pct) {
    if (sorted.empty()) return 0.0;
    if (sorted.size() == 1) return sorted[0];
    double rank = (pct / 100.0) * static_cast<double>(sorted.size() - 1);
    size_t lo = static_cast<size_t>(std::floor(rank));
    size_t hi = static_cast<size_t>(std::ceil(rank));
    if (hi >= sorted.size()) hi = sorted.size() - 1;
    double frac = rank - static_cast<double>(lo);
    return sorted[lo] + frac * (sorted[hi] - sorted[lo]);
}

LatencySummary compute_latency_summary(std::vector<double>& samples) {
    LatencySummary out;
    if (samples.empty()) return out;

    std::sort(samples.begin(), samples.end());
    out.count    = samples.size();
    out.min_us   = samples.front();
    out.max_us   = samples.back();
    out.avg_us   = std::accumulate(samples.begin(), samples.end(), 0.0) / static_cast<double>(samples.size());
    out.p50_us   = percentile_interp(samples, 50.0);
    out.p95_us   = percentile_interp(samples, 95.0);
    out.p99_us   = percentile_interp(samples, 99.0);
    out.p999_us  = percentile_interp(samples, 99.9);
    out.p9999_us = percentile_interp(samples, 99.99);
    return out;
}

void print_latency_summary(const char* label, const LatencySummary& s) {
    if (s.count == 0) return;
    std::cout << "  " << std::left << std::setw(19) << label << ": " << std::right
              << std::fixed << std::setprecision(1)
              << "Min=" << s.min_us << " us | "
              << "Avg=" << s.avg_us << " us | "
              << "p50=" << s.p50_us << " us | "
              << "p95=" << s.p95_us << " us | "
              << "p99=" << s.p99_us << " us | "
              << "p99.9=" << s.p999_us << " us | "
              << "p99.99=" << s.p9999_us << " us | "
              << "Max=" << s.max_us << " us\n";
}
