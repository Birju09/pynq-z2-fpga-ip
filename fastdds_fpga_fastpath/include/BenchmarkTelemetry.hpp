// ============================================================================
// BenchmarkTelemetry.hpp
//
// Linux Resource (/proc + getrusage) and Monotonic Latency Percentile Library.
// ============================================================================

#ifndef _BENCHMARK_TELEMETRY_HPP_
#define _BENCHMARK_TELEMETRY_HPP_

#include <cstddef>
#include <cstdint>
#include <sys/resource.h>
#include <sys/time.h>
#include <vector>

struct ProcMemInfo {
    uint64_t vm_rss_kb{0};
    uint64_t vm_hwm_kb{0};
    uint64_t vm_size_kb{0};
    uint64_t vm_data_kb{0};
    bool valid{false};
};

struct ProcStatCpu {
    uint64_t user{0};
    uint64_t nice{0};
    uint64_t system{0};
    uint64_t idle{0};
    uint64_t iowait{0};
    uint64_t irq{0};
    uint64_t softirq{0};
    uint64_t steal{0};
    bool valid{false};

    uint64_t total() const {
        return user + nice + system + idle + iowait + irq + softirq + steal;
    }
    uint64_t busy() const {
        return user + nice + system + irq + softirq + steal;
    }
};

struct ResourceSnapshot {
    uint64_t wall_ns{0};
    struct rusage ru{};
    ProcMemInfo mem{};
    ProcStatCpu sys_cpu{};
};

struct LatencySummary {
    size_t count{0};
    double min_us{0.0};
    double avg_us{0.0};
    double p50_us{0.0};
    double p95_us{0.0};
    double p99_us{0.0};
    double p999_us{0.0};
    double p9999_us{0.0};
    double max_us{0.0};
};

uint64_t monotonic_now_ns();
ProcMemInfo read_proc_mem_info();
ProcStatCpu read_proc_stat_cpu();
ResourceSnapshot capture_resources();
double tv_to_ms(const struct timeval& tv);
double percentile_interp(const std::vector<double>& sorted, double pct);
LatencySummary compute_latency_summary(std::vector<double>& samples);
void print_latency_summary(const char* label, const LatencySummary& s);

#endif // _BENCHMARK_TELEMETRY_HPP_
