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

#include "../include/FpgaFastpathBridge.hpp"

static std::atomic<bool> g_running{true};

static void sigint_handler(int) {
    g_running = false;
}

static uint64_t monotonic_now_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
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
// Resource & Latency Telemetry Helpers (Linux /proc + getrusage)
// ============================================================================

struct ProcMemInfo {
    uint64_t vm_rss_kb{0};
    uint64_t vm_hwm_kb{0};
    uint64_t vm_size_kb{0};
    uint64_t vm_data_kb{0};
    bool valid{false};
};

static ProcMemInfo read_proc_mem_info() {
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

static ProcStatCpu read_proc_stat_cpu() {
    ProcStatCpu s;
    std::ifstream f("/proc/stat");
    if (!f.is_open()) return s;

    std::string label;
    if (f >> label >> s.user >> s.nice >> s.system >> s.idle >> s.iowait >> s.irq >> s.softirq >> s.steal) {
        if (label == "cpu") s.valid = true;
    }
    return s;
}

struct ResourceSnapshot {
    uint64_t wall_ns{0};
    struct rusage ru{};
    ProcMemInfo mem{};
    ProcStatCpu sys_cpu{};
};

static ResourceSnapshot capture_resources() {
    ResourceSnapshot snap;
    snap.wall_ns = monotonic_now_ns();
    getrusage(RUSAGE_SELF, &snap.ru);
    snap.mem = read_proc_mem_info();
    snap.sys_cpu = read_proc_stat_cpu();
    return snap;
}

static double tv_to_ms(const struct timeval& tv) {
    return static_cast<double>(tv.tv_sec) * 1000.0 + static_cast<double>(tv.tv_usec) / 1000.0;
}

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

// Sorted rank interpolation per benchmarking standards
static double percentile_interp(const std::vector<double>& sorted, double pct) {
    if (sorted.empty()) return 0.0;
    if (sorted.size() == 1) return sorted[0];
    double rank = (pct / 100.0) * static_cast<double>(sorted.size() - 1);
    size_t lo = static_cast<size_t>(std::floor(rank));
    size_t hi = static_cast<size_t>(std::ceil(rank));
    if (hi >= sorted.size()) hi = sorted.size() - 1;
    double frac = rank - static_cast<double>(lo);
    return sorted[lo] + frac * (sorted[hi] - sorted[lo]);
}

static LatencySummary compute_latency_summary(std::vector<double>& samples) {
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

static void print_latency_summary(const char* label, const LatencySummary& s) {
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

// ============================================================================
// Pure Software Fast DDS Pub/Sub Session (No FPGA IP / DMA Driver)
// ============================================================================

class PureDdsSession {
public:
    using RxCallback = std::function<void(const FastSample&)>;

    PureDdsSession(const fastdds_fpga::BridgeConfig& cfg, bool enable_rx, bool local_echo_peer)
        : cfg_(cfg), enable_rx_(enable_rx), local_echo_peer_(local_echo_peer) {}

    ~PureDdsSession() {
        stop();
    }

    bool init(RxCallback rx_cb) {
        rx_cb_ = std::move(rx_cb);

        if (!init_main_participant()) {
            std::cerr << "[-] PureDDS: Failed to initialize main DomainParticipant\n";
            return false;
        }

        if (local_echo_peer_) {
            if (!init_echo_peer_participant()) {
                std::cerr << "[-] PureDDS: Failed to initialize local echo peer DomainParticipant\n";
                return false;
            }
        }
        return true;
    }

    bool wait_for_match(uint32_t timeout_ms = 20000) {
        std::unique_lock<std::mutex> lock(match_mutex_);
        if (writer_matched_) return true;

        std::cout << "[PureDDS] Waiting for " << (local_echo_peer_ ? "Local Echo Peer" : "Remote Fast DDS Participant")
                  << " on topic '" << cfg_.topic_name << "' (timeout: " << timeout_ms / 1000 << "s)...\n";

        bool ok = cv_match_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this]() {
            return writer_matched_.load() || !g_running.load();
        });

        if (ok && writer_matched_) {
            std::cout << "[PureDDS] SUCCESS: Discovered & matched DDS subscriber endpoint!\n";
            return true;
        }
        std::cerr << "[PureDDS] WARNING: Discovery timeout reached without remote match.\n";
        return false;
    }

    bool publish(FastSample& sample) {
        if (!writer_) return false;
#if FASTDDS_V3
        return (writer_->write(&sample) == eprosima::fastdds::dds::RETCODE_OK);
#else
        return writer_->write(&sample);
#endif
    }

    void stop() {
        auto* factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();
        if (echo_participant_) {
            if (echo_publisher_ && echo_writer_) echo_publisher_->delete_datawriter(echo_writer_);
            if (echo_subscriber_ && echo_reader_) echo_subscriber_->delete_datareader(echo_reader_);
            if (echo_publisher_) echo_participant_->delete_publisher(echo_publisher_);
            if (echo_subscriber_) echo_participant_->delete_subscriber(echo_subscriber_);
            if (echo_topic_) echo_participant_->delete_topic(echo_topic_);
            factory->delete_participant(echo_participant_);
            echo_participant_ = nullptr;
            echo_publisher_   = nullptr;
            echo_subscriber_  = nullptr;
            echo_topic_       = nullptr;
            echo_writer_      = nullptr;
            echo_reader_      = nullptr;
        }

        if (rx_participant_) {
            if (subscriber_ && reader_) subscriber_->delete_datareader(reader_);
            if (subscriber_) rx_participant_->delete_subscriber(subscriber_);
            if (rx_topic_) rx_participant_->delete_topic(rx_topic_);
            factory->delete_participant(rx_participant_);
            rx_participant_ = nullptr;
            subscriber_     = nullptr;
            rx_topic_       = nullptr;
            reader_         = nullptr;
        }

        if (participant_) {
            if (publisher_ && writer_) publisher_->delete_datawriter(writer_);
            if (publisher_) participant_->delete_publisher(publisher_);
            if (topic_) participant_->delete_topic(topic_);
            factory->delete_participant(participant_);
            participant_ = nullptr;
            publisher_   = nullptr;
            topic_       = nullptr;
            writer_      = nullptr;
        }
    }

private:
    void configure_udp_transport(eprosima::fastdds::dds::DomainParticipantQos& pqos, bool log_cfg) {
        pqos.transport().use_builtin_transports = false;
        auto udp_transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
        if (!cfg_.ps_ip.empty()) {
            udp_transport->interfaceWhiteList.push_back(cfg_.ps_ip);
            if (log_cfg) {
                std::cout << "[PureDDS] Fast DDS UDPv4 bound to interface: " << cfg_.ps_ip << "\n";
            }
        }
        if (cfg_.max_message_size >= 100 && cfg_.max_message_size <= 65500U) {
            udp_transport->maxMessageSize = cfg_.max_message_size;
        }
        // Capped by net.core.rmem_max / wmem_max.
        udp_transport->receiveBufferSize = 4 * 1024 * 1024;
        udp_transport->sendBufferSize = 4 * 1024 * 1024;
        pqos.transport().user_transports.push_back(udp_transport);

        if (!cfg_.peer_ip.empty() && !local_echo_peer_) {
            struct in_addr peer_addr;
            if (inet_pton(AF_INET, cfg_.peer_ip.c_str(), &peer_addr) == 1) {
                uint32_t pip = ntohl(peer_addr.s_addr);
#if FASTDDS_V3
                eprosima::fastdds::rtps::Locator_t peer_loc;
#else
                eprosima::fastrtps::rtps::Locator_t peer_loc;
#endif
                peer_loc.kind = 1; // LOCATOR_KIND_UDPv4
                peer_loc.port = 0;
                peer_loc.address[12] = static_cast<uint8_t>((pip >> 24) & 0xFF);
                peer_loc.address[13] = static_cast<uint8_t>((pip >> 16) & 0xFF);
                peer_loc.address[14] = static_cast<uint8_t>((pip >> 8) & 0xFF);
                peer_loc.address[15] = static_cast<uint8_t>(pip & 0xFF);
                pqos.wire_protocol().builtin.initialPeersList.push_back(peer_loc);
                if (log_cfg) {
                    std::cout << "[PureDDS] Added Unicast SPDP Peer: " << cfg_.peer_ip << "\n";
                }
            }
        }
    }

    bool init_main_participant() {
        using namespace eprosima::fastdds::dds;

        DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
        pqos.name("PYNQZ2_PureDDS_Node");
        configure_udp_transport(pqos, true);

        participant_ = DomainParticipantFactory::get_instance()->create_participant(
            cfg_.domain_id, pqos, &part_listener_);
        if (!participant_) return false;

        type_.reset(new FastSamplePubSubType());
        type_.register_type(participant_);

        topic_ = participant_->create_topic(cfg_.topic_name, type_.get_type_name(), TOPIC_QOS_DEFAULT);
        if (!topic_) return false;

        publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (!publisher_) return false;

        DataWriterQos wqos = DATAWRITER_QOS_DEFAULT;
        wqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
        writer_ = publisher_->create_datawriter(topic_, wqos);
        if (!writer_) return false;

        if (enable_rx_) {
            return init_rx_participant();
        }
        return true;
    }

    // Separate participant so our own writer's samples are never delivered to the RX reader.
    bool init_rx_participant() {
        using namespace eprosima::fastdds::dds;

        DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
        pqos.name("PYNQZ2_PureDDS_RxNode");
        configure_udp_transport(pqos, false);

        rx_participant_ = DomainParticipantFactory::get_instance()->create_participant(
            cfg_.domain_id, pqos);
        if (!rx_participant_) return false;

        participant_->ignore_participant(rx_participant_->get_instance_handle());
        rx_participant_->ignore_participant(participant_->get_instance_handle());

        rx_type_.reset(new FastSamplePubSubType());
        rx_type_.register_type(rx_participant_);

        rx_topic_ = rx_participant_->create_topic(cfg_.topic_name, rx_type_.get_type_name(), TOPIC_QOS_DEFAULT);
        if (!rx_topic_) return false;

        subscriber_ = rx_participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (!subscriber_) return false;

        {
            DataReaderQos rqos = DATAREADER_QOS_DEFAULT;
            rqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
            // Default pool is too small for 4 KB DATA_FRAG samples arriving faster than the ARM drains them.
            rqos.history().kind = KEEP_LAST_HISTORY_QOS;
            rqos.history().depth = 512;
            rqos.resource_limits().max_samples = 1024;
            rqos.resource_limits().allocated_samples = 64;
            rqos.resource_limits().max_instances = 1;
            rqos.resource_limits().max_samples_per_instance = 1024;
            rqos.endpoint().history_memory_policy =
                eprosima::fastrtps::rtps::DYNAMIC_RESERVE_MEMORY_MODE;
            reader_ = subscriber_->create_datareader(rx_topic_, rqos, &reader_listener_);
            if (!reader_) return false;
        }
        return true;
    }

    bool init_echo_peer_participant() {
        using namespace eprosima::fastdds::dds;

        DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
        pqos.name("PYNQZ2_PureDDS_EchoPeer");
        configure_udp_transport(pqos, false);

        echo_participant_ = DomainParticipantFactory::get_instance()->create_participant(
            cfg_.domain_id, pqos);
        if (!echo_participant_) return false;

        echo_type_.reset(new FastSamplePubSubType());
        echo_type_.register_type(echo_participant_);

        echo_topic_ = echo_participant_->create_topic(
            cfg_.topic_name, echo_type_.get_type_name(), TOPIC_QOS_DEFAULT);
        if (!echo_topic_) return false;

        echo_publisher_ = echo_participant_->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (!echo_publisher_) return false;

        DataWriterQos wqos = DATAWRITER_QOS_DEFAULT;
        wqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
        echo_writer_ = echo_publisher_->create_datawriter(echo_topic_, wqos);
        if (!echo_writer_) return false;

        echo_subscriber_ = echo_participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (!echo_subscriber_) return false;

        DataReaderQos rqos = DATAREADER_QOS_DEFAULT;
        rqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
        echo_reader_ = echo_subscriber_->create_datareader(echo_topic_, rqos, &echo_reader_listener_);
        if (!echo_reader_) return false;

        std::cout << "[PureDDS] Local UDPv4 Echo Peer Participant initialized (--sim-peer mode).\n";
        return true;
    }

    template <typename Prefix>
    bool is_rx_prefix(const Prefix& prefix) const {
        return rx_participant_ && prefix == rx_participant_->guid().guidPrefix;
    }

    void notify_remote_reader_matched() {
        {
            std::lock_guard<std::mutex> lock(match_mutex_);
            writer_matched_ = true;
        }
        cv_match_.notify_all();
    }

    void on_main_data_available(eprosima::fastdds::dds::DataReader* reader) {
        using namespace eprosima::fastdds::dds;
        FastSample sample;
        SampleInfo info;
        auto local_prefix = participant_->guid().guidPrefix;

        while (reader->take_next_sample(&sample, &info) == ReturnCode_t::RETCODE_OK) {
            if (!info.valid_data) continue;
            // Ignore samples published by our own DataWriter; only count peer/echoed samples
            if (info.sample_identity.writer_guid().guidPrefix == local_prefix) continue;
            if (rx_cb_) rx_cb_(sample);
        }
    }

    void on_echo_data_available(eprosima::fastdds::dds::DataReader* reader) {
        using namespace eprosima::fastdds::dds;
        FastSample sample;
        SampleInfo info;
        auto echo_prefix = echo_participant_->guid().guidPrefix;

        while (reader->take_next_sample(&sample, &info) == ReturnCode_t::RETCODE_OK) {
            if (!info.valid_data) continue;
            if (info.sample_identity.writer_guid().guidPrefix == echo_prefix) continue;
            if (echo_writer_) {
                (void)echo_writer_->write(&sample);
            }
        }
    }

    class PurePartListener : public eprosima::fastdds::dds::DomainParticipantListener {
    public:
        explicit PurePartListener(PureDdsSession* parent) : parent_(parent) {}

#if FASTDDS_V3
        void on_data_reader_discovery(
            eprosima::fastdds::dds::DomainParticipant*,
            eprosima::fastdds::rtps::ReaderDiscoveryStatus reason,
            const eprosima::fastdds::dds::SubscriptionBuiltinTopicData& info,
            bool& should_be_ignored) override {
            should_be_ignored = false;
            if (reason == eprosima::fastdds::rtps::ReaderDiscoveryStatus::DISCOVERED_READER &&
                info.topic_name.to_string() == parent_->cfg_.topic_name &&
                info.guid.guidPrefix != parent_->participant_->guid().guidPrefix &&
                !parent_->is_rx_prefix(info.guid.guidPrefix)) {
                parent_->notify_remote_reader_matched();
            }
        }
#else
        void on_subscriber_discovery(
            eprosima::fastdds::dds::DomainParticipant*,
            eprosima::fastrtps::rtps::ReaderDiscoveryInfo&& info) override {
            if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER &&
                info.info.topicName().to_string() == parent_->cfg_.topic_name &&
                info.info.guid().guidPrefix != parent_->participant_->guid().guidPrefix &&
                !parent_->is_rx_prefix(info.info.guid().guidPrefix)) {
                parent_->notify_remote_reader_matched();
            }
        }
#endif
    private:
        PureDdsSession* parent_;
    };

    class MainReaderListener : public eprosima::fastdds::dds::DataReaderListener {
    public:
        explicit MainReaderListener(PureDdsSession* parent) : parent_(parent) {}
        void on_data_available(eprosima::fastdds::dds::DataReader* reader) override {
            parent_->on_main_data_available(reader);
        }
    private:
        PureDdsSession* parent_;
    };

    class EchoReaderListener : public eprosima::fastdds::dds::DataReaderListener {
    public:
        explicit EchoReaderListener(PureDdsSession* parent) : parent_(parent) {}
        void on_data_available(eprosima::fastdds::dds::DataReader* reader) override {
            parent_->on_echo_data_available(reader);
        }
    private:
        PureDdsSession* parent_;
    };

    fastdds_fpga::BridgeConfig cfg_;
    bool enable_rx_{true};
    bool local_echo_peer_{false};
    RxCallback rx_cb_;

    std::atomic<bool> writer_matched_{false};
    std::mutex match_mutex_;
    std::condition_variable cv_match_;

    eprosima::fastdds::dds::DomainParticipant* participant_{nullptr};
    eprosima::fastdds::dds::Publisher* publisher_{nullptr};
    eprosima::fastdds::dds::Subscriber* subscriber_{nullptr};
    eprosima::fastdds::dds::Topic* topic_{nullptr};
    eprosima::fastdds::dds::DataWriter* writer_{nullptr};
    eprosima::fastdds::dds::DataReader* reader_{nullptr};
    eprosima::fastdds::dds::TypeSupport type_;

    eprosima::fastdds::dds::DomainParticipant* rx_participant_{nullptr};
    eprosima::fastdds::dds::Topic* rx_topic_{nullptr};
    eprosima::fastdds::dds::TypeSupport rx_type_;

    eprosima::fastdds::dds::DomainParticipant* echo_participant_{nullptr};
    eprosima::fastdds::dds::Publisher* echo_publisher_{nullptr};
    eprosima::fastdds::dds::Subscriber* echo_subscriber_{nullptr};
    eprosima::fastdds::dds::Topic* echo_topic_{nullptr};
    eprosima::fastdds::dds::DataWriter* echo_writer_{nullptr};
    eprosima::fastdds::dds::DataReader* echo_reader_{nullptr};
    eprosima::fastdds::dds::TypeSupport echo_type_;

    PurePartListener part_listener_{this};
    MainReaderListener reader_listener_{this};
    EchoReaderListener echo_reader_listener_{this};
};

// ============================================================================
// Sample Payload Population & Anti-Stale Verification
// ============================================================================

static void populate_sample(FastSample& fs, uint32_t iter, uint64_t now_ns) {
    fs.m_cycle_id = iter;
    fs.m_flags = 0x000000A5U ^ iter;
    fs.m_tx_timestamp_ns = now_ns;

    // Generate unique dynamic pattern (Rule 4: Anti-Stale Data Invariant)
    for (size_t p = 0; p < fs.m_payload.size(); p++) {
        fs.m_payload[p] = static_cast<uint32_t>((iter << 16) ^ (0x5A000000U + (p * 0x010103U)));
    }
}

static bool verify_sample_pattern(const FastSample& sample) {
    uint32_t cid = sample.m_cycle_id;
    for (size_t p = 0; p < 10; p++) {
        uint32_t expected = static_cast<uint32_t>((cid << 16) ^ (0x5A000000U + (p * 0x010103U)));
        if (sample.m_payload[p] != expected) {
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
    std::string csv_path = "";

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
                      << "  --topic <name>      DDS Topic name (default: FastSampleTopic)\n"
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

    auto handle_rx_sample = [&](const FastSample& sample) {
        uint64_t now_ns = monotonic_now_ns();
        if (sample.m_tx_timestamp_ns > 0 && now_ns >= sample.m_tx_timestamp_ns) {
            double rtt_us = static_cast<double>(now_ns - sample.m_tx_timestamp_ns) / 1000.0;
            std::lock_guard<std::mutex> lock(rtt_mutex);
            rtt_latencies_us.push_back(rtt_us);
            if (last_rx_seq > 0 && sample.m_cycle_id > last_rx_seq + 1) {
                rx_seq_gaps += (sample.m_cycle_id - last_rx_seq - 1);
            }
            last_rx_seq = sample.m_cycle_id;
        }
        if (!verify_sample_pattern(sample)) {
            rx_corrupted++;
        }
        rx_count++;
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

        pure_session = std::make_unique<PureDdsSession>(cfg, enable_rx, sim_peer);
        if (!pure_session->init(handle_rx_sample)) {
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
            bridge->start_rx_worker([&](const FastSample& sample, const struct fpga_rx_metadata& meta) {
                (void)meta;
                handle_rx_sample(sample);
            });
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

    FastSample sw_sample;
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
            populate_sample(sw_sample, static_cast<uint32_t>(iter), now_ns);

            uint64_t t0 = monotonic_now_ns();
            bool ok = pure_session->publish(sw_sample);
            uint64_t t1 = monotonic_now_ns();

            if (ok) {
                published++;
                tx_call_latencies_us.push_back(static_cast<double>(t1 - t0) / 1000.0);
            } else {
                std::cerr << "[-] Pure DDS publish error at iter " << iter << "\n";
                break;
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
    double elapsed_ms = static_cast<double>(res_after.wall_ns - res_before.wall_ns) / 1e6;
    double eff_rate   = (elapsed_ms > 0.0) ? (published * 1000.0 / elapsed_ms) : 0.0;
    double eff_mbps   = (elapsed_ms > 0.0) ? (published * 4096.0 * 8.0 / (elapsed_ms * 1000.0)) : 0.0;

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
