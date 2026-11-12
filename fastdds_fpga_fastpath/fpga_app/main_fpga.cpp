// ============================================================================
// main_fpga.cpp
//
// PYNQ-Z2 FPGA Fastpath Application:
// - Control Plane : Fast DDS Participant & Discovery (over PS eth0)
// - Data Plane    : Pure zero-copy coherent DDR DMA publishing via FPGA Fastpath IP
//                   (over PL RMII PHY)
// ============================================================================

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <time.h>
#include <unistd.h>

#include "../include/FpgaFastpathBridge.hpp"

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

int main(int argc, char* argv[]) {
    std::signal(SIGINT, sigint_handler);
    std::signal(SIGTERM, sigint_handler);

    fastdds_fpga::BridgeConfig cfg;
    int target_rate_hz   = 1000;
    int total_samples    = 10000;
    bool sim_peer        = false;
    bool enable_rx       = true;
    bool skip_discovery  = false;

    // Command-line parsing
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--device" && i + 1 < argc) {
            cfg.device_path = argv[++i];
        } else if (arg == "--topic" && i + 1 < argc) {
            cfg.topic_name = argv[++i];
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
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --device <path>     AXI DMA device (default: /dev/axi_dma_dev)\n"
                      << "  --topic <name>      DDS Topic name (default: FastSampleTopic)\n"
                      << "  --domain <id>       DDS Domain ID (default: 0)\n"
                      << "  --rate <Hz>         Publish rate in Hz (default: 1000)\n"
                      << "  --samples <N>       Total samples to transmit (default: 10000)\n"
                      << "  --ps-ip <IP>        PS eth0 IP for Fast DDS Discovery (--local-ip alias)\n"
                      << "  --pl-ip <IP>        PL RMII PHY hardware IP (default: 192.168.1.201)\n"
                      << "  --peer-ip <IP>      Remote PC IP for unicast SPDP discovery & fallback\n"
                      << "  --peer-mac <MAC>    Remote peer MAC address override\n"
                      << "  --sim-peer          Enable PL internal peer emulator (loopback)\n"
                      << "  --ip-frag           Use legacy IPv4 fragmentation instead of DATA_FRAG (0x16)\n"
                      << "  --frag-size <B>     DATA_FRAG fragmentSize in bytes (default: derived 1400)\n"
                      << "  --max-msg-size <B>  Fast DDS UDP maxMessageSize (default: 1468 -> 1400B frag)\n"
                      << "  --no-wait           Start immediately without waiting for discovery\n"
                      << "  --no-rx             Disable S2MM receiver thread\n";
            return 0;
        }
    }

    fastdds_fpga::FpgaFastpathBridge bridge(cfg);

    std::cout << "=================================================================\n"
              << "  PYNQ-Z2 Fast DDS + Hardware Fastpath Publisher (Option 3)\n"
              << "=================================================================\n"
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
                                                                std::to_string(bridge.effective_frag_size_bytes()) + " B")) << "\n"
              << "=================================================================\n\n";

    if (!bridge.init()) {
        std::cerr << "[-] Failed to initialize FPGA Fastpath Bridge\n";
        return 1;
    }

    // Optional S2MM Inbound Receiver callback
    std::atomic<uint32_t> rx_count{0};
    std::vector<double> rtt_latencies_us;
    rtt_latencies_us.reserve(total_samples);
    std::mutex rtt_mutex;

    if (enable_rx) {
        bridge.start_rx_worker([&](const FastSample& sample, const struct fpga_rx_metadata& meta) {
            (void)meta;
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            uint64_t now_ns = static_cast<uint64_t>(now.tv_sec) * 1000000000ULL + now.tv_nsec;

            if (sample.m_tx_timestamp_ns > 0 && now_ns >= sample.m_tx_timestamp_ns) {
                double rtt_us = static_cast<double>(now_ns - sample.m_tx_timestamp_ns) / 1000.0;
                std::lock_guard<std::mutex> lock(rtt_mutex);
                rtt_latencies_us.push_back(rtt_us);
            }
            rx_count++;
        });
    }

    // Discovery Phase
    if (sim_peer) {
        std::cout << "[FpgaApp] Operating in Peer Emulator mode. Configuring hardware loopback...\n";
        bridge.configure_default_endpoints();
    } else if (skip_discovery) {
        std::cout << "[FpgaApp] Skipping discovery wait. Applying default wire endpoints...\n";
        bridge.configure_default_endpoints();
    } else {
        if (!bridge.wait_for_match(20000)) {
            std::cerr << "[-] Warning: Proceeding with default physical wire endpoint settings\n";
            bridge.configure_default_endpoints();
        }
    }

    std::cout << "\n[FpgaApp] Commencing zero-copy hardware wire streaming at "
              << target_rate_hz << " Hz...\n";

    long period_ns = 1000000000L / target_rate_hz;
    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    uint32_t published = 0;
    auto start_time = std::chrono::steady_clock::now();

    for (int iter = 1; iter <= total_samples && g_running; iter++) {
        next_tick.tv_nsec += period_ns;
        while (next_tick.tv_nsec >= 1000000000L) {
            next_tick.tv_sec++;
            next_tick.tv_nsec -= 1000000000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_tick, nullptr);

        // Populate sample directly into coherent mmap DMA buffer (True Zero-Copy)
        FastSample* fs = bridge.zero_copy_loan();
        if (fs) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            uint64_t now_ns = static_cast<uint64_t>(now.tv_sec) * 1000000000ULL + now.tv_nsec;

            fs->m_cycle_id = static_cast<uint32_t>(iter);
            fs->m_flags = 0x000000A5U ^ static_cast<uint32_t>(iter);
            fs->m_tx_timestamp_ns = now_ns;

            // Generate unique dynamic pattern (Rule 4: Anti-Stale Data Invariant)
            for (size_t p = 0; p < fs->m_payload.size(); p++) {
                fs->m_payload[p] = static_cast<uint32_t>((iter << 16) ^ (0x5A000000U + (p * 0x010103U)));
            }

            // Hardware Fastpath Publish: kicks MM2S and returns immediately (0 CPU serialization)
            int rc = bridge.publish_loaned();
            if (rc == 0) {
                published++;
            } else {
                std::cerr << "[-] Hardware publish error at iter " << iter << ": " << rc << "\n";
                break;
            }
        }

        if (iter % (target_rate_hz > 10 ? target_rate_hz : 10) == 0 || iter == total_samples) {
            std::cout << "\r[FpgaApp] Streamed " << iter << " / " << total_samples << " samples ("
                      << (iter * 100 / total_samples) << "%)" << std::flush;
        }
    }
    std::cout << "\n";

    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time).count();

    // Query hardware diagnostics
    struct rtps_cycle_diag diag;
    std::memset(&diag, 0, sizeof(diag));
    bool has_diag = bridge.get_diagnostics(&diag);

    std::cout << "\n=================================================================\n"
              << "  Transmission Summary & Benchmarks\n"
              << "=================================================================\n"
              << "  Total Samples Sent : " << published << "\n"
              << "  Elapsed Time       : " << elapsed_ms << " ms\n"
              << "  Effective Rate     : " << std::fixed << std::setprecision(1)
              << (elapsed_ms > 0 ? (published * 1000.0 / elapsed_ms) : 0.0) << " samples/s ("
              << (elapsed_ms > 0 ? (published * 4096.0 * 8.0 / (elapsed_ms * 1000.0)) : 0.0) << " Mbps)\n";

    if (enable_rx) {
        std::cout << "  Inbound RX Samples : " << rx_count.load() << "\n";
        std::lock_guard<std::mutex> lock(rtt_mutex);
        if (!rtt_latencies_us.empty()) {
            std::sort(rtt_latencies_us.begin(), rtt_latencies_us.end());
            double sum = std::accumulate(rtt_latencies_us.begin(), rtt_latencies_us.end(), 0.0);
            double avg = sum / rtt_latencies_us.size();
            double p50 = rtt_latencies_us[rtt_latencies_us.size() * 50 / 100];
            double p95 = rtt_latencies_us[rtt_latencies_us.size() * 95 / 100];
            double p99 = rtt_latencies_us[rtt_latencies_us.size() * 99 / 100];

            std::cout << "  Round-Trip Latency : Min=" << rtt_latencies_us.front() << " us | "
                      << "Avg=" << avg << " us | "
                      << "p50=" << p50 << " us | "
                      << "p95=" << p95 << " us | "
                      << "p99=" << p99 << " us | "
                      << "Max=" << rtt_latencies_us.back() << " us\n";
        }
    }

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

    bridge.stop();
    return 0;
}
