// ============================================================================
// main_remote.cpp
//
// Remote Workstation Fast DDS Application:
// - Standard Fast DDS Participant (runs on standard PC / Linux laptop)
// - Subscribes to 4,096-byte FastSample messages sent by PYNQ-Z2 FPGA IP
// - Verifies sample data pattern integrity and measures wire latency
// - Optionally echoes or publishes samples back to the FPGA
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

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#if __has_include(<fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>)
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>
#else
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#endif

#include "../idl/FastSample.hpp"
#include "../idl/FastSamplePubSubTypes.hpp"

using namespace eprosima::fastdds::dds;

static std::atomic<bool> g_running{true};

static void sigint_handler(int) {
    g_running = false;
}

class RemoteNode {
public:
    RemoteNode(uint32_t domain_id, const std::string& topic_name,
               bool echo_mode, bool pub_mode, int pub_rate_hz,
               const std::string& interface_ip, const std::string& peer_ip)
        : domain_id_(domain_id), topic_name_(topic_name),
          echo_mode_(echo_mode), pub_mode_(pub_mode), pub_rate_hz_(pub_rate_hz),
          interface_ip_(interface_ip), peer_ip_(peer_ip),
          samples_received_(0), samples_sent_(0), corrupted_samples_(0), seq_gaps_(0),
          last_seq_(0) {}

    ~RemoteNode() {
        cleanup();
    }

    bool init() {
        DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
        pqos.name("Remote_Workstation_FastDDS_Node");

        if (!interface_ip_.empty()) {
            pqos.transport().use_builtin_transports = false;
            auto udp_transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
            udp_transport->interfaceWhiteList.push_back(interface_ip_);
            pqos.transport().user_transports.push_back(udp_transport);
            std::cout << "[RemoteNode] Bound Fast DDS to interface: " << interface_ip_ << "\n";
        }

        if (!peer_ip_.empty()) {
            struct in_addr peer_addr;
            if (inet_pton(AF_INET, peer_ip_.c_str(), &peer_addr) == 1) {
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
                std::cout << "[RemoteNode] Added Unicast SPDP Peer: " << peer_ip_ << "\n";
            }
        }

        participant_ = DomainParticipantFactory::get_instance()->create_participant(domain_id_, pqos);
        if (!participant_) {
            std::cerr << "[-] Error: Failed to create DomainParticipant\n";
            return false;
        }

        // Register FastSample Topic Data Type
        type_.reset(new FastSamplePubSubType());
        type_.register_type(participant_);

        topic_ = participant_->create_topic(topic_name_, type_.get_type_name(), TOPIC_QOS_DEFAULT);
        if (!topic_) {
            std::cerr << "[-] Error: Failed to create Topic\n";
            return false;
        }

        // Subscriber & DataReader
        subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (!subscriber_) return false;

        DataReaderQos rqos = DATAREADER_QOS_DEFAULT;
        rqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
        reader_ = subscriber_->create_datareader(topic_, rqos, &reader_listener_);
        if (!reader_) return false;

        // Publisher & DataWriter (used for echo or publish mode)
        if (echo_mode_ || pub_mode_) {
            publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT);
            if (!publisher_) return false;

            DataWriterQos wqos = DATAWRITER_QOS_DEFAULT;
            wqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
            writer_ = publisher_->create_datawriter(topic_, wqos, &writer_listener_);
            if (!writer_) return false;
        }

        return true;
    }

    void run(int max_samples) {
        if (pub_mode_) {
            run_publisher(max_samples);
        } else {
            run_subscriber(max_samples);
        }
    }

    void print_stats() {
        std::cout << "\n=================================================================\n"
                  << "  Remote Workstation Fast DDS Statistics\n"
                  << "=================================================================\n"
                  << "  Samples Received   : " << samples_received_.load() << "\n"
                  << "  Corrupted Samples  : " << corrupted_samples_.load() << "\n"
                  << "  Sequence Gaps      : " << seq_gaps_.load() << "\n";
        if (echo_mode_ || pub_mode_) {
            std::cout << "  Samples Sent/Echoed: " << samples_sent_.load() << "\n";
        }

        std::lock_guard<std::mutex> lock(lat_mutex_);
        if (!latencies_us_.empty()) {
            std::sort(latencies_us_.begin(), latencies_us_.end());
            double sum = std::accumulate(latencies_us_.begin(), latencies_us_.end(), 0.0);
            double avg = sum / latencies_us_.size();
            double p50 = latencies_us_[latencies_us_.size() * 50 / 100];
            double p95 = latencies_us_[latencies_us_.size() * 95 / 100];
            double p99 = latencies_us_[latencies_us_.size() * 99 / 100];

            std::cout << "  One-Way Latency    : Min=" << latencies_us_.front() << " us | "
                      << "Avg=" << std::fixed << std::setprecision(1) << avg << " us | "
                      << "p50=" << p50 << " us | "
                      << "p95=" << p95 << " us | "
                      << "p99=" << p99 << " us | "
                      << "Max=" << latencies_us_.back() << " us\n";
        }
        std::cout << "=================================================================\n";
    }

private:
    void run_subscriber(int max_samples) {
        std::cout << "[RemoteNode] Listening for FastSample packets on '" << topic_name_ << "'...\n";
        if (echo_mode_) {
            std::cout << "[RemoteNode] Echo Mode ACTIVE: incoming samples will be bounced back to FPGA.\n";
        }

        while (g_running && (max_samples <= 0 || samples_received_ < static_cast<uint32_t>(max_samples))) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void run_publisher(int max_samples) {
        std::cout << "[RemoteNode] Publishing FastSample packets to FPGA at "
                  << pub_rate_hz_ << " Hz...\n";

        long period_ns = 1000000000L / pub_rate_hz_;
        struct timespec next_tick;
        clock_gettime(CLOCK_MONOTONIC, &next_tick);

        FastSample sample;
        for (int iter = 1; iter <= max_samples && g_running; iter++) {
            next_tick.tv_nsec += period_ns;
            while (next_tick.tv_nsec >= 1000000000L) {
                next_tick.tv_sec++;
                next_tick.tv_nsec -= 1000000000L;
            }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_tick, nullptr);

            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            uint64_t now_ns = static_cast<uint64_t>(now.tv_sec) * 1000000000ULL + now.tv_nsec;

            sample.cycle_id(iter);
            sample.flags(0x5A5A0000U | iter);
            sample.tx_timestamp_ns(now_ns);

            for (size_t i = 0; i < sample.payload().size(); i++) {
                sample.payload()[i] = static_cast<uint32_t>((iter << 16) | (i + 1));
            }

#if FASTDDS_V3
            if (writer_->write(&sample) == RETCODE_OK) {
                samples_sent_++;
            }
#else
            if (writer_->write(&sample)) {
                samples_sent_++;
            }
#endif

            if (iter % (pub_rate_hz_ > 10 ? pub_rate_hz_ : 10) == 0 || iter == max_samples) {
                std::cout << "\r[RemoteNode] Published " << iter << " / " << max_samples << " samples" << std::flush;
            }
        }
        std::cout << "\n";
    }

    void on_data_received(DataReader* reader) {
        FastSample sample;
        SampleInfo info;

        while (reader->take_next_sample(&sample, &info) == ReturnCode_t::RETCODE_OK) {
            if (info.valid_data) {
                uint32_t cid = sample.cycle_id();
                uint64_t tx_ns = sample.tx_timestamp_ns();

                // Compute latency
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                uint64_t now_ns = static_cast<uint64_t>(now.tv_sec) * 1000000000ULL + now.tv_nsec;

                if (tx_ns > 0 && now_ns >= tx_ns) {
                    double lat_us = static_cast<double>(now_ns - tx_ns) / 1000.0;
                    std::lock_guard<std::mutex> lock(lat_mutex_);
                    latencies_us_.push_back(lat_us);
                }

                // Verify pattern integrity (Rule 4 Anti-Stale check)
                bool pattern_ok = true;
                for (size_t p = 0; p < 10; p++) {
                    uint32_t expected = static_cast<uint32_t>((cid << 16) ^ (0x5A000000U + (p * 0x010103U)));
                    if (sample.payload()[p] != expected) {
                        pattern_ok = false;
                        break;
                    }
                }
                if (!pattern_ok) corrupted_samples_++;

                // Sequence continuity check
                if (last_seq_ > 0 && cid > last_seq_ + 1) {
                    seq_gaps_ += (cid - last_seq_ - 1);
                }
                last_seq_ = cid;
                samples_received_++;

                // Echo back to FPGA if echo mode is enabled
                if (echo_mode_ && writer_) {
#if FASTDDS_V3
                    if (writer_->write(&sample) == RETCODE_OK) samples_sent_++;
#else
                    if (writer_->write(&sample)) samples_sent_++;
#endif
                }

                if (samples_received_ % 1000 == 0) {
                    std::cout << "[RemoteNode] Received " << samples_received_ << " samples (Last cycle_id: "
                              << cid << ", Corrupted: " << corrupted_samples_ << ")\n";
                }
            }
        }
    }

    void cleanup() {
        if (participant_) {
            if (publisher_ && writer_) publisher_->delete_datawriter(writer_);
            if (subscriber_ && reader_) subscriber_->delete_datareader(reader_);
            if (publisher_) participant_->delete_publisher(publisher_);
            if (subscriber_) participant_->delete_subscriber(subscriber_);
            if (topic_) participant_->delete_topic(topic_);
            DomainParticipantFactory::get_instance()->delete_participant(participant_);
            participant_ = nullptr;
        }
    }

    class ReaderListener : public DataReaderListener {
    public:
        explicit ReaderListener(RemoteNode* parent) : parent_(parent) {}
        void on_data_available(DataReader* reader) override {
            parent_->on_data_received(reader);
        }
        void on_subscription_matched(DataReader*, const SubscriptionMatchedStatus& info) override {
            if (info.current_count_change > 0) {
                std::cout << "[RemoteNode] Matched FPGA Publisher! Ready to receive.\n";
            }
        }
    private:
        RemoteNode* parent_;
    };

    class WriterListener : public DataWriterListener {
    public:
        void on_publication_matched(DataWriter*, const PublicationMatchedStatus& info) override {
            if (info.current_count_change > 0) {
                std::cout << "[RemoteNode] Matched FPGA Subscriber! Ready to transmit.\n";
            }
        }
    };

    uint32_t domain_id_;
    std::string topic_name_;
    bool echo_mode_;
    bool pub_mode_;
    int pub_rate_hz_;
    std::string interface_ip_;
    std::string peer_ip_;

    std::atomic<uint32_t> samples_received_;
    std::atomic<uint32_t> samples_sent_;
    std::atomic<uint32_t> corrupted_samples_;
    std::atomic<uint32_t> seq_gaps_;
    uint32_t last_seq_;

    std::vector<double> latencies_us_;
    std::mutex lat_mutex_;

    DomainParticipant* participant_{nullptr};
    Publisher* publisher_{nullptr};
    Subscriber* subscriber_{nullptr};
    Topic* topic_{nullptr};
    DataWriter* writer_{nullptr};
    DataReader* reader_{nullptr};
    TypeSupport type_;

    ReaderListener reader_listener_{this};
    WriterListener writer_listener_;
};

int main(int argc, char* argv[]) {
    std::signal(SIGINT, sigint_handler);
    std::signal(SIGTERM, sigint_handler);

    uint32_t domain_id = 0;
    std::string topic_name = "FastSampleTopic";
    bool echo_mode = false;
    bool pub_mode = false;
    int pub_rate_hz = 1000;
    int max_samples = 0; // 0 = run until Ctrl+C
    std::string interface_ip = "";
    std::string peer_ip = "";

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--topic" && i + 1 < argc) {
            topic_name = argv[++i];
        } else if (arg == "--domain" && i + 1 < argc) {
            domain_id = std::stoul(argv[++i]);
        } else if (arg == "--echo") {
            echo_mode = true;
        } else if (arg == "--publish") {
            pub_mode = true;
        } else if (arg == "--rate" && i + 1 < argc) {
            pub_rate_hz = std::stoi(argv[++i]);
        } else if (arg == "--samples" && i + 1 < argc) {
            max_samples = std::stoi(argv[++i]);
        } else if (arg == "--interface" && i + 1 < argc) {
            interface_ip = argv[++i];
        } else if (arg == "--peer-ip" && i + 1 < argc) {
            peer_ip = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --topic <name>      DDS Topic name (default: FastSampleTopic)\n"
                      << "  --domain <id>       DDS Domain ID (default: 0)\n"
                      << "  --interface <IP>    Bind Fast DDS to specific local NIC IP\n"
                      << "  --peer-ip <IP>      PYNQ-Z2 PS eth0 IP for unicast SPDP discovery\n"
                      << "  --echo              Bounce received samples back to FPGA\n"
                      << "  --publish           Publish mode: transmit samples from PC to FPGA\n"
                      << "  --rate <Hz>         Publish rate in Hz (default: 1000)\n"
                      << "  --samples <N>       Number of samples (default: infinite until Ctrl+C)\n";
            return 0;
        }
    }

    std::cout << "=================================================================\n"
              << "  Remote Workstation Fast DDS Application\n"
              << "=================================================================\n"
              << "  Topic        : " << topic_name << " (Domain " << domain_id << ")\n"
              << "  Interface    : " << (interface_ip.empty() ? "All Interfaces" : interface_ip) << "\n"
              << "  Unicast Peer : " << (peer_ip.empty() ? "Multicast Only" : peer_ip) << "\n"
              << "  Role         : " << (pub_mode ? "Publisher" : (echo_mode ? "Subscriber + Echo" : "Subscriber")) << "\n"
              << "  Target Samples: " << (max_samples > 0 ? std::to_string(max_samples) : "Continuous") << "\n"
              << "=================================================================\n\n";

    RemoteNode node(domain_id, topic_name, echo_mode, pub_mode, pub_rate_hz, interface_ip, peer_ip);
    if (!node.init()) {
        return 1;
    }

    node.run(max_samples);
    node.print_stats();

    return 0;
}
