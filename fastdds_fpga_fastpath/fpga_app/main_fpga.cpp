// ============================================================================
// main_fpga.cpp
//
// PYNQ-Z2 Fast DDS Benchmark & Fastpath Application:
// - FPGA Fastpath Mode (default):
//     * Control Plane : Fast DDS Participant & Discovery (over PS eth0)
//     * Data Plane    : Zero-copy coherent DDR DMA publishing via FPGA IP
//                       (over PL RMII PHY or internal peer emulator)
// - Pure Fast DDS Mode (--pure-dds / --no-ip):
//     * Control + Data Plane : 100% software Fast DDS Pub/Sub over Linux
//                              UDP/IP stack (no FPGA IP / DMA driver opened)
//     * Collects identical latency, CPU, and memory metrics for side-by-side
//       comparison against FPGA hardware offload.
// ============================================================================

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <sys/resource.h>
#include <time.h>
#include <unistd.h>

#include <fastdds/dds/subscriber/SampleInfo.hpp>

#include "BenchmarkTelemetry.hpp"
#include "FpgaFastpathBridge.hpp"
#include "PureDdsSession.hpp"

static std::atomic<bool> g_running{true};

static void sigint_handler(int) {
    g_running = false;
}

static uint32_t parse_ipv4(const std::string& ip_str) {
    struct in_addr addr;
    if (inet_pton(AF_INET, ip_str.c_str(), &addr) == 1) {
        return ntohl(addr.s_addr);
    }
    return 0;
}

static bool parse_mac(const std::string& mac_str, uint8_t mac[6]) {
    unsigned int m[6];
    if (sscanf(mac_str.c_str(), "%x:%x:%x:%x:%x:%x",
               &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6) {
        for (int i = 0; i < 6; i++) mac[i] = static_cast<uint8_t>(m[i]);
        return true;
    }
    return false;
}

// ============================================================================
// Sample Payload Population & Anti-Stale Verification
// ============================================================================

template <typename SampleT>
static void populate_sample(SampleT& fs, uint32_t iter, uint64_t now_ns) {
    fs.m_cycle_id = iter;
    fs.m_flags = 0x000000A5U ^ iter;
    fs.m_tx_timestamp_ns = now_ns;

    // Generate unique dynamic pattern (Rule 4: Anti-Stale Data Invariant)
    for (size_t p = 0; p < fs.m_payload.size(); p++) {
        fs.m_payload[p] = static_cast<uint32_t>((iter << 16) ^ (0x5A000000U + (p * 0x010103U)));
    }
}

template <typename SampleT>
static bool verify_sample_pattern(const SampleT& sample) {
    uint32_t cid = sample.m_cycle_id;
    for (size_t p = 0; p < 10; p++) {
        uint32_t expected = static_cast<uint32_t>((cid << 16) ^ (0x5A000000U + (p * 0x010103U)));
        if (sample.m_payload[p] != expected) {
            return false;
        }
    }
    if (sample.m_payload.size() > 10) {
        size_t last_p = sample.m_payload.size() - 1;
        uint32_t expected_last = static_cast<uint32_t>((cid << 16) ^ (0x5A000000U + (last_p * 0x010103U)));
        if (sample.m_payload[last_p] != expected_last) {
            return false;
        }
    }
    return true;
}

// ============================================================================
// Main Entrypoint
// ============================================================================

int main(int argc, char* argv[]) {
    std::signal(SIGINT, sigint_handler);
    std::signal(SIGTERM, sigint_handler);

    fastdds_fpga::BridgeConfig cfg;
    int target_rate_hz   = 1000;
    int total_samples    = 10000;
    bool sim_peer        = false;
    bool enable_rx       = true;
    bool skip_discovery  = false;
    bool pure_dds        = false;
    bool topic_explicit  = false;
    std::string csv_path = "";

    // Command-line parsing
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--device" && i + 1 < argc) {
            cfg.device_path = argv[++i];
        } else if (arg == "--topic" && i + 1 < argc) {
            cfg.topic_name = argv[++i];
            topic_explicit = true;
        } else if (arg == "--5mb") {
            cfg.large_5mb = true;
        } else if (arg == "--ring-depth" && i + 1 < argc) {
            cfg.ring_depth = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--domain" && i + 1 < argc) {
            cfg.domain_id = std::stoul(argv[++i]);
        } else if (arg == "--rate" && i + 1 < argc) {
            target_rate_hz = std::stoi(argv[++i]);
        } else if (arg == "--samples" && i + 1 < argc) {
            total_samples = std::stoi(argv[++i]);
        } else if ((arg == "--ps-ip" || arg == "--local-ip") && i + 1 < argc) {
            cfg.ps_ip = argv[++i];
        } else if (arg == "--pl-ip" && i + 1 < argc) {
            uint32_t ip = parse_ipv4(argv[++i]);
            if (ip != 0) cfg.pl_ip = ip;
        } else if (arg == "--peer-ip" && i + 1 < argc) {
            cfg.peer_ip = argv[++i];
            uint32_t pip = parse_ipv4(cfg.peer_ip);
            if (pip != 0) cfg.fallback_dst_ip = pip;
        } else if (arg == "--peer-mac" && i + 1 < argc) {
            if (parse_mac(argv[++i], cfg.override_peer_mac)) {
                cfg.has_peer_mac_override = true;
            }
        } else if (arg == "--sim-peer") {
            sim_peer = true;
            cfg.enable_peer_emu = true;
        } else if (arg == "--pure-dds" || arg == "--no-ip") {
            pure_dds = true;
        } else if (arg == "--ip-frag") {
            cfg.tx_ip_frag_mode = true;
        } else if (arg == "--frag-size" && i + 1 < argc) {
            cfg.frag_size_bytes = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--max-msg-size" && i + 1 < argc) {
            cfg.max_message_size = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--no-rx") {
            enable_rx = false;
        } else if (arg == "--no-wait") {
            skip_discovery = true;
        } else if (arg == "--csv" && i + 1 < argc) {
            csv_path = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --pure-dds          Run pure software Fast DDS Pub/Sub without FPGA IP (--no-ip alias)\n"
                      << "  --device <path>     AXI DMA device (default: /dev/axi_dma_dev, FPGA mode only)\n"
                      << "  --topic <name>      DDS Topic name (default: FastSampleTopic or LargeSample5MBTopic)\n"
                      << "  --5mb               Use 5 MiB LargeSample5MB IDL via DDR Ring\n"
                      << "  --ring-depth <K>    Per-endpoint DDR ring depth 1..16 (default: 2)\n"
                      << "  --domain <id>       DDS Domain ID (default: 0)\n"
                      << "  --rate <Hz>         Publish rate in Hz (default: 1000)\n"
                      << "  --samples <N>       Total samples to transmit (default: 10000)\n"
                      << "  --ps-ip <IP>        PS eth0 IP for Fast DDS (--local-ip alias)\n"
                      << "  --pl-ip <IP>        PL RMII PHY hardware IP (default: 192.168.1.201, FPGA mode)\n"
                      << "  --peer-ip <IP>      Remote PC IP for unicast SPDP discovery & fallback\n"
                      << "  --peer-mac <MAC>    Remote peer MAC address override (FPGA mode)\n"
                      << "  --sim-peer          Loopback mode (FPGA: PL peer emulator; Pure DDS: local UDPv4 echo peer)\n"
                      << "  --ip-frag           Use legacy IPv4 fragmentation instead of DATA_FRAG (FPGA mode)\n"
                      << "  --frag-size <B>     DATA_FRAG fragmentSize in bytes (default: derived 1400, FPGA mode)\n"
                      << "  --max-msg-size <B>  Fast DDS UDP maxMessageSize (default: 1468 -> 1400B frag)\n"
                      << "  --no-wait           Start immediately without waiting for discovery\n"
                      << "  --no-rx             Disable inbound receiver / RTT measurement\n"
                      << "  --csv <file>        Append summary metrics row to CSV file for comparison\n";
            return 0;
        }
    }

    if (target_rate_hz <= 0) target_rate_hz = 1;
    if (total_samples <= 0) total_samples = 1;
    if (cfg.large_5mb && !topic_explicit) {
        cfg.topic_name = "LargeSample5MBTopic";
    }

    // Shared RX telemetry state
    std::atomic<uint32_t> rx_count{0};
    std::atomic<uint32_t> rx_corrupted{0};
    std::atomic<uint32_t> rx_seq_gaps{0};
    uint32_t last_rx_seq = 0;

    std::vector<double> rtt_latencies_us;
    if (enable_rx) rtt_latencies_us.reserve(static_cast<size_t>(total_samples));
    std::mutex rtt_mutex;

    std::vector<double> tx_call_latencies_us;
    tx_call_latencies_us.reserve(static_cast<size_t>(total_samples));

    auto handle_rx_common = [&](uint32_t cid, uint64_t tx_ns, bool pattern_ok) {
        uint64_t now_ns = monotonic_now_ns();
        if (tx_ns > 0 && now_ns >= tx_ns) {
            double rtt_us = static_cast<double>(now_ns - tx_ns) / 1000.0;
            std::lock_guard<std::mutex> lock(rtt_mutex);
            rtt_latencies_us.push_back(rtt_us);
            if (last_rx_seq > 0 && cid > last_rx_seq + 1) {
                rx_seq_gaps += (cid - last_rx_seq - 1);
            }
            last_rx_seq = cid;
        }
        if (!pattern_ok) {
            rx_corrupted++;
        }
        rx_count++;
    };
    auto handle_rx_sample = [&](const FastSample& sample) {
        handle_rx_common(sample.m_cycle_id, sample.m_tx_timestamp_ns, verify_sample_pattern(sample));
    };
    auto handle_rx_sample_5mb = [&](const LargeSample5MB& sample) {
        handle_rx_common(sample.m_cycle_id, sample.m_tx_timestamp_ns, verify_sample_pattern(sample));
    };

    std::unique_ptr<fastdds_fpga::FpgaFastpathBridge> bridge;
    std::unique_ptr<PureDdsSession> pure_session;

    if (pure_dds) {
        std::cout << "=================================================================\n"
                  << "  PYNQ-Z2 Pure Fast DDS Software Baseline (No FPGA IP)\n"
                  << "=================================================================\n"
                  << "  Mode         : Pure Software Fast DDS (UDPv4, No FPGA DMA)\n"
                  << "  Topic        : " << cfg.topic_name << " (Domain " << cfg.domain_id << ")\n"
                  << "  Interface    : " << (cfg.ps_ip.empty() ? "All Interfaces" : cfg.ps_ip)
                  << (cfg.peer_ip.empty() ? "" : (" (Unicast Peer: " + cfg.peer_ip + ")")) << "\n"
                  << "  Publish Rate : " << target_rate_hz << " Hz\n"
                  << "  Total Samples: " << total_samples << " (4,096 B / sample)\n"
                  << "  Peer Mode    : " << (sim_peer ? "In-Process Local UDPv4 Echo Peer" : "Remote Fast DDS Peer") << "\n"
                  << "  Max Msg Size : " << cfg.max_message_size << " B\n"
                  << "=================================================================\n\n";

        pure_session = std::make_unique<PureDdsSession>(cfg, enable_rx, sim_peer, &g_running);
        if (!pure_session->init(handle_rx_sample, handle_rx_sample_5mb)) {
            std::cerr << "[-] Failed to initialize Pure Fast DDS session\n";
            return 1;
        }

        if (!skip_discovery) {
            pure_session->wait_for_match(20000);
        } else {
            std::cout << "[PureDDS] Skipping discovery wait (--no-wait).\n";
        }
    } else {
        bridge = std::make_unique<fastdds_fpga::FpgaFastpathBridge>(cfg);

        std::cout << "=================================================================\n"
                  << "  PYNQ-Z2 Fast DDS + Hardware Fastpath Publisher (Option 3)\n"
                  << "=================================================================\n"
                  << "  Mode         : FPGA Hardware Fastpath (Zero-Copy DMA)\n"
                  << "  Device       : " << cfg.device_path << "\n"
                  << "  Topic        : " << cfg.topic_name << " (Domain " << cfg.domain_id << ")\n"
                  << "  PS Discovery : " << (cfg.ps_ip.empty() ? "All Interfaces" : cfg.ps_ip)
                  << (cfg.peer_ip.empty() ? "" : (" (Unicast Peer: " + cfg.peer_ip + ")")) << "\n"
                  << "  PL RMII IP   : " << ((cfg.pl_ip >> 24) & 0xFF) << "."
                                         << ((cfg.pl_ip >> 16) & 0xFF) << "."
                                         << ((cfg.pl_ip >> 8) & 0xFF) << "."
                                         << (cfg.pl_ip & 0xFF) << "\n"
                  << "  Publish Rate : " << target_rate_hz << " Hz\n"
                  << "  Total Samples: " << total_samples << " (4,096 B / sample)\n"
                  << "  Wire Mode    : " << (cfg.enable_peer_emu ? "Internal Peer Emulator" : "Physical RMII Wire") << "\n"
                  << "  TX Frag Mode : " << (cfg.tx_ip_frag_mode ? "Legacy IPv4 Fragmentation"
                                                                 : ("RTPS DATA_FRAG (0x16), fragmentSize=" +
                                                                    std::to_string(bridge->effective_frag_size_bytes()) + " B")) << "\n"
                  << "=================================================================\n\n";

        if (!bridge->init()) {
            std::cerr << "[-] Failed to initialize FPGA Fastpath Bridge\n";
            return 1;
        }

        if (enable_rx) {
            if (cfg.large_5mb) {
                bridge->start_rx_worker_5mb([&](const LargeSample5MB& sample, const struct fpga_rx_metadata& meta) {
                    (void)meta;
                    handle_rx_sample_5mb(sample);
                });
            } else {
                bridge->start_rx_worker([&](const FastSample& sample, const struct fpga_rx_metadata& meta) {
                    (void)meta;
                    handle_rx_sample(sample);
                });
            }
        }

        // Discovery Phase
        if (sim_peer) {
            std::cout << "[FpgaApp] Operating in Peer Emulator mode. Configuring hardware loopback...\n";
            bridge->configure_default_endpoints();
        } else if (skip_discovery) {
            std::cout << "[FpgaApp] Skipping discovery wait. Applying default wire endpoints...\n";
            bridge->configure_default_endpoints();
        } else {
            if (!bridge->wait_for_match(20000)) {
                std::cerr << "[-] Warning: Proceeding with default physical wire endpoint settings\n";
                bridge->configure_default_endpoints();
            }
        }
    }

    const char* tag = pure_dds ? "[PureDDS]" : "[FpgaApp]";
    std::cout << "\n" << tag << " Commencing "
              << (pure_dds ? "pure software Fast DDS" : "zero-copy hardware wire")
              << " streaming at " << target_rate_hz << " Hz...\n";

    long period_ns = 1000000000L / target_rate_hz;
    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    std::unique_ptr<FastSample> sw_sample = cfg.large_5mb ? nullptr : std::make_unique<FastSample>();
    std::unique_ptr<LargeSample5MB> sw_sample_5mb = cfg.large_5mb ? std::make_unique<LargeSample5MB>() : nullptr;
    uint32_t published = 0;

    // Capture baseline CPU & Memory strictly before the streaming phase
    ResourceSnapshot res_before = capture_resources();

    for (int iter = 1; iter <= total_samples && g_running; iter++) {
        next_tick.tv_nsec += period_ns;
        while (next_tick.tv_nsec >= 1000000000L) {
            next_tick.tv_sec++;
            next_tick.tv_nsec -= 1000000000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_tick, nullptr);

        if (pure_dds) {
            uint64_t now_ns = monotonic_now_ns();
            bool ok = false;
            uint64_t t0 = 0, t1 = 0;
            if (cfg.large_5mb) {
                populate_sample(*sw_sample_5mb, static_cast<uint32_t>(iter), now_ns);
                t0 = monotonic_now_ns();
                ok = pure_session->publish_5mb(*sw_sample_5mb);
                t1 = monotonic_now_ns();
            } else {
                populate_sample(*sw_sample, static_cast<uint32_t>(iter), now_ns);
                t0 = monotonic_now_ns();
                ok = pure_session->publish(*sw_sample);
                t1 = monotonic_now_ns();
            }

            if (ok) {
                published++;
                tx_call_latencies_us.push_back(static_cast<double>(t1 - t0) / 1000.0);
            } else {
                std::cerr << "[-] Pure DDS publish error at iter " << iter << "\n";
                break;
            }
        } else if (cfg.large_5mb) {
            struct fpga_tx_loan loan;
            LargeSample5MB* ls = bridge->zero_copy_loan_5mb(&loan);
            if (ls) {
                uint64_t now_ns = monotonic_now_ns();
                populate_sample(*ls, static_cast<uint32_t>(iter), now_ns);

                uint64_t t0 = monotonic_now_ns();
                int rc = bridge->publish_loaned_5mb(loan);
                uint64_t t1 = monotonic_now_ns();

                if (rc == 0) {
                    published++;
                    tx_call_latencies_us.push_back(static_cast<double>(t1 - t0) / 1000.0);
                } else {
                    std::cerr << "[-] Hardware 5MB publish error at iter " << iter << ": " << rc << "\n";
                    break;
                }
            }
        } else {
            // Populate sample directly into coherent mmap DMA buffer (True Zero-Copy)
            FastSample* fs = bridge->zero_copy_loan();
            if (fs) {
                uint64_t now_ns = monotonic_now_ns();
                populate_sample(*fs, static_cast<uint32_t>(iter), now_ns);

                // Hardware Fastpath Publish: kicks MM2S and returns immediately
                uint64_t t0 = monotonic_now_ns();
                int rc = bridge->publish_loaned();
                uint64_t t1 = monotonic_now_ns();

                if (rc == 0) {
                    published++;
                    tx_call_latencies_us.push_back(static_cast<double>(t1 - t0) / 1000.0);
                } else {
                    std::cerr << "[-] Hardware publish error at iter " << iter << ": " << rc << "\n";
                    break;
                }
            }
        }

        if (iter % (target_rate_hz > 10 ? target_rate_hz : 10) == 0 || iter == total_samples) {
            std::cout << "\r" << tag << " Streamed " << iter << " / " << total_samples << " samples ("
                      << (iter * 100 / total_samples) << "%)" << std::flush;
        }
    }
    std::cout << "\n";

    // Capture end-of-streaming CPU & Memory immediately after the TX loop
    ResourceSnapshot res_after = capture_resources();

    // Allow a brief grace window for final in-flight RX echoes to arrive
    if (enable_rx && rx_count.load() < published && g_running) {
        auto drain_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
        while (rx_count.load() < published && std::chrono::steady_clock::now() < drain_deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    // Compute Elapsed & Resource Deltas
    double sample_bytes = cfg.large_5mb ? static_cast<double>(sizeof(LargeSample5MB)) : 4096.0;
    double elapsed_ms = static_cast<double>(res_after.wall_ns - res_before.wall_ns) / 1e6;
    double eff_rate   = (elapsed_ms > 0.0) ? (published * 1000.0 / elapsed_ms) : 0.0;
    double eff_mbps   = (elapsed_ms > 0.0) ? (published * sample_bytes * 8.0 / (elapsed_ms * 1000.0)) : 0.0;

    double user_cpu_ms  = tv_to_ms(res_after.ru.ru_utime) - tv_to_ms(res_before.ru.ru_utime);
    double sys_cpu_ms   = tv_to_ms(res_after.ru.ru_stime) - tv_to_ms(res_before.ru.ru_stime);
    double total_cpu_ms = user_cpu_ms + sys_cpu_ms;

    long num_cpus = sysconf(_SC_NPROCESSORS_ONLN);
    if (num_cpus < 1) num_cpus = 1;

    double proc_cpu_pct_1core = (elapsed_ms > 0.0) ? (total_cpu_ms / elapsed_ms) * 100.0 : 0.0;
    double proc_cpu_pct_sys   = proc_cpu_pct_1core / static_cast<double>(num_cpus);
    double cpu_us_per_sample  = (published > 0) ? (total_cpu_ms * 1000.0 / published) : 0.0;
    double user_us_per_sample = (published > 0) ? (user_cpu_ms * 1000.0 / published) : 0.0;
    double sys_us_per_sample  = (published > 0) ? (sys_cpu_ms * 1000.0 / published) : 0.0;

    long vol_csw   = res_after.ru.ru_nvcsw - res_before.ru.ru_nvcsw;
    long invol_csw = res_after.ru.ru_nivcsw - res_before.ru.ru_nivcsw;
    long min_flt   = res_after.ru.ru_minflt - res_before.ru.ru_minflt;
    long maj_flt   = res_after.ru.ru_majflt - res_before.ru.ru_majflt;

    double sys_total_cpu_pct = 0.0;
    double sys_softirq_pct   = 0.0;
    bool has_sys_cpu = (res_before.sys_cpu.valid && res_after.sys_cpu.valid);
    if (has_sys_cpu) {
        uint64_t d_total   = res_after.sys_cpu.total() - res_before.sys_cpu.total();
        uint64_t d_busy    = res_after.sys_cpu.busy() - res_before.sys_cpu.busy();
        uint64_t d_softirq = res_after.sys_cpu.softirq - res_before.sys_cpu.softirq;
        if (d_total > 0) {
            sys_total_cpu_pct = (static_cast<double>(d_busy) / static_cast<double>(d_total)) * 100.0;
            sys_softirq_pct   = (static_cast<double>(d_softirq) / static_cast<double>(d_total)) * 100.0;
        }
    }

    uint64_t peak_rss_kb = res_after.mem.valid ? res_after.mem.vm_hwm_kb
                                               : static_cast<uint64_t>(res_after.ru.ru_maxrss);
    int64_t rss_delta_kb = res_after.mem.valid && res_before.mem.valid
                               ? (static_cast<int64_t>(res_after.mem.vm_rss_kb) -
                                  static_cast<int64_t>(res_before.mem.vm_rss_kb))
                               : 0;

    LatencySummary tx_stats = compute_latency_summary(tx_call_latencies_us);
    LatencySummary rtt_stats;
    if (enable_rx) {
        std::lock_guard<std::mutex> lock(rtt_mutex);
        rtt_stats = compute_latency_summary(rtt_latencies_us);
    }

    // Query FPGA hardware diagnostics if in FPGA mode
    struct rtps_cycle_diag diag;
    std::memset(&diag, 0, sizeof(diag));
    bool has_diag = (!pure_dds && bridge) ? bridge->get_diagnostics(&diag) : false;

    const std::string mode_str = pure_dds
                                     ? (sim_peer ? "pure_dds_loopback" : "pure_dds")
                                     : (sim_peer ? "fpga_sim_peer" : "fpga_fastpath");

    std::cout << "\n=================================================================\n"
              << "  Transmission Summary & Benchmarks (" << (pure_dds ? "Pure Fast DDS" : "FPGA Fastpath") << ")\n"
              << "=================================================================\n"
              << "  Mode               : " << mode_str << "\n"
              << "  Total Samples Sent : " << published << "\n"
              << "  Elapsed Time       : " << std::fixed << std::setprecision(1) << elapsed_ms << " ms\n"
              << "  Effective Rate     : " << eff_rate << " samples/s (" << eff_mbps << " Mbps)\n";

    print_latency_summary("Publish Call Lat", tx_stats);

    if (enable_rx) {
        uint32_t rx_n = rx_count.load();
        double loss_pct = (published > 0 && published >= rx_n)
                              ? (static_cast<double>(published - rx_n) * 100.0 / published)
                              : 0.0;
        std::cout << "  Inbound RX Samples : " << rx_n
                  << " (Loss=" << std::fixed << std::setprecision(2) << loss_pct << "%"
                  << ", Corrupted=" << rx_corrupted.load()
                  << ", SeqGaps=" << rx_seq_gaps.load() << ")\n";
        print_latency_summary("Round-Trip Latency", rtt_stats);
    }

    std::cout << "-----------------------------------------------------------------\n"
              << "  CPU Usage Metrics (Streaming Window, " << num_cpus << " CPU core" << (num_cpus > 1 ? "s" : "") << ")\n"
              << "-----------------------------------------------------------------\n"
              << "  Process CPU Time   : Total=" << std::fixed << std::setprecision(2) << total_cpu_ms
              << " ms (User=" << user_cpu_ms << " ms, Sys=" << sys_cpu_ms << " ms)\n"
              << "  Process CPU Usage  : " << proc_cpu_pct_1core << "% (1-core) | "
              << proc_cpu_pct_sys << "% (system-wide)\n"
              << "  CPU Cost / Sample  : " << std::setprecision(2) << cpu_us_per_sample
              << " us/sample (User=" << user_us_per_sample << " us, Sys=" << sys_us_per_sample << " us)\n"
              << "  Context Switches   : Vol=" << vol_csw << ", Invol=" << invol_csw
              << " (" << std::setprecision(2)
              << (published > 0 ? static_cast<double>(vol_csw + invol_csw) / published : 0.0)
              << " / sample)\n";
    if (has_sys_cpu) {
        std::cout << "  System CPU (all)   : Busy=" << std::setprecision(2) << sys_total_cpu_pct
                  << "% | SoftIRQ=" << sys_softirq_pct << "%\n";
    }
    if (pure_dds && sim_peer) {
        std::cout << "  Note               : Includes in-process local UDPv4 echo peer participant.\n";
    }

    std::cout << "-----------------------------------------------------------------\n"
              << "  Memory Usage Metrics\n"
              << "-----------------------------------------------------------------\n";
    if (res_after.mem.valid) {
        std::cout << "  Process RSS (VmRSS): " << std::fixed << std::setprecision(2)
                  << (res_before.mem.vm_rss_kb / 1024.0) << " MB -> "
                  << (res_after.mem.vm_rss_kb / 1024.0) << " MB (Delta=" << rss_delta_kb << " KB)\n"
                  << "  Peak RSS (VmHWM)   : " << (peak_rss_kb / 1024.0) << " MB (" << peak_rss_kb << " KB)\n"
                  << "  Virtual (VmSize)   : " << (res_after.mem.vm_size_kb / 1024.0)
                  << " MB | Heap/Data (VmData): " << (res_after.mem.vm_data_kb / 1024.0) << " MB\n";
    } else {
        std::cout << "  Peak RSS (maxrss)  : " << std::fixed << std::setprecision(2)
                  << (peak_rss_kb / 1024.0) << " MB (" << peak_rss_kb << " KB)\n";
    }
    std::cout << "  Page Faults        : Minor=" << min_flt << ", Major=" << maj_flt << "\n";

    if (has_diag) {
        std::cout << "-----------------------------------------------------------------\n"
                  << "  FPGA Hardware Diagnostic Registers (PL RTL Counters)\n"
                  << "-----------------------------------------------------------------\n"
                  << "  stat_tx_samples    : " << diag.stat_tx_samples << "\n"
                  << "  stat_rx_samples    : " << diag.stat_rx_samples << "\n"
                  << "  stat_seq_gaps      : " << diag.stat_seq_gaps << "\n"
                  << "  stat_stale_drops   : " << diag.stat_stale_drops << "\n"
                  << "  stat_csum_errs     : " << diag.stat_csum_errs << "\n"
                  << "  stat_skipped_submsg: " << diag.stat_skipped_submsg << "\n"
                  << "  last_cycle_ticks   : " << diag.last_cycle_ticks << " (clock ticks)\n";
    }
    std::cout << "=================================================================\n";

    // Optional CSV summary export
    if (!csv_path.empty()) {
        bool write_header = false;
        {
            std::ifstream check(csv_path);
            write_header = !check.good() || (check.peek() == std::ifstream::traits_type::eof());
        }
        std::ofstream csv(csv_path, std::ios::app);
        if (csv.is_open()) {
            if (write_header) {
                csv << "mode,target_rate_hz,sent,rx,corrupted,seq_gaps,elapsed_ms,eff_rate_hz,eff_mbps,"
                    << "tx_avg_us,tx_p50_us,tx_p99_us,tx_max_us,"
                    << "rtt_min_us,rtt_avg_us,rtt_p50_us,rtt_p95_us,rtt_p99_us,rtt_p999_us,rtt_max_us,"
                    << "user_cpu_ms,sys_cpu_ms,total_cpu_ms,cpu_pct_1core,cpu_pct_sys,cpu_us_per_sample,"
                    << "vol_csw,invol_csw,sys_busy_pct,sys_softirq_pct,"
                    << "rss_before_kb,rss_after_kb,peak_rss_kb,vm_size_kb,min_flt,maj_flt\n";
            }
            csv << mode_str << "," << target_rate_hz << "," << published << "," << rx_count.load() << ","
                << rx_corrupted.load() << "," << rx_seq_gaps.load() << ","
                << std::fixed << std::setprecision(2)
                << elapsed_ms << "," << eff_rate << "," << eff_mbps << ","
                << tx_stats.avg_us << "," << tx_stats.p50_us << "," << tx_stats.p99_us << "," << tx_stats.max_us << ","
                << rtt_stats.min_us << "," << rtt_stats.avg_us << "," << rtt_stats.p50_us << ","
                << rtt_stats.p95_us << "," << rtt_stats.p99_us << "," << rtt_stats.p999_us << "," << rtt_stats.max_us << ","
                << user_cpu_ms << "," << sys_cpu_ms << "," << total_cpu_ms << ","
                << proc_cpu_pct_1core << "," << proc_cpu_pct_sys << "," << cpu_us_per_sample << ","
                << vol_csw << "," << invol_csw << "," << sys_total_cpu_pct << "," << sys_softirq_pct << ","
                << res_before.mem.vm_rss_kb << "," << res_after.mem.vm_rss_kb << ","
                << peak_rss_kb << "," << res_after.mem.vm_size_kb << "," << min_flt << "," << maj_flt << "\n";
            std::cout << "[Metrics] Appended benchmark row to: " << csv_path << "\n";
        } else {
            std::cerr << "[-] Warning: Could not open CSV file for writing: " << csv_path << "\n";
        }
    }

    if (pure_session) pure_session->stop();
    if (bridge) bridge->stop();
    return 0;
}
