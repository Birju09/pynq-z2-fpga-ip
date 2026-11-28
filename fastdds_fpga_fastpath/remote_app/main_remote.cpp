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
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <time.h>
#include <unistd.h>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include <fastdds/rtps/common/LocatorWithMask.hpp>
#include <fastdds/rtps/transport/TransportInterface.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>

#include "FastSample.hpp"
#include "FastSamplePubSubTypes.hpp"
#include "LargeSample5MB.hpp"
#include "LargeSample5MBPubSubTypes.hpp"

using namespace eprosima::fastdds::dds;

static std::atomic<bool> g_running{true};

static void sigint_handler(int) {
    g_running = false;
}

// Transport wrapper that redirects user-data packets destined for the PYNQ-Z2 PS IP
// (e.g. 192.168.2.20:7411) to the FPGA PL RMII IP:port (e.g. 192.168.2.201:7511),
// while leaving SPDP/SEDP metatraffic (even ports like 7400/7410) on PS eth0.
class FpgaRedirectTransport : public eprosima::fastdds::rtps::TransportInterface {
public:
    using Locator = eprosima::fastdds::rtps::Locator;
    using LocatorList = eprosima::fastdds::rtps::LocatorList;
    using LocatorSelector = eprosima::fastdds::rtps::LocatorSelector;
    using LocatorSelectorEntry = eprosima::fastdds::rtps::LocatorSelectorEntry;
    using PortParameters = eprosima::fastdds::rtps::PortParameters;
    using PropertyPolicy = eprosima::fastdds::rtps::PropertyPolicy;
    using SendResourceList = eprosima::fastdds::rtps::SendResourceList;
    using TransportDescriptorInterface = eprosima::fastdds::rtps::TransportDescriptorInterface;
    using TransportReceiverInterface = eprosima::fastdds::rtps::TransportReceiverInterface;
    using NetmaskFilterInfo = eprosima::fastdds::rtps::NetmaskFilterInfo;

    FpgaRedirectTransport(
            std::unique_ptr<TransportInterface> inner,
            uint32_t ps_ip,
            uint32_t pl_ip,
            uint16_t pl_port)
        : TransportInterface(LOCATOR_KIND_UDPv4),
          inner_(std::move(inner)),
          ps_ip_(ps_ip),
          pl_ip_(pl_ip),
          pl_port_(pl_port) {}

    bool init(const PropertyPolicy* properties = nullptr, const uint32_t& max_msg_size_no_frag = 0) override {
        return inner_->init(properties, max_msg_size_no_frag);
    }
    bool IsInputChannelOpen(const Locator& loc) const override { return inner_->IsInputChannelOpen(loc); }
    bool IsLocatorSupported(const Locator& loc) const override { return inner_->IsLocatorSupported(loc); }
    bool is_locator_allowed(const Locator& loc) const override { return inner_->is_locator_allowed(loc); }
    bool is_locator_reachable(const eprosima::fastdds::rtps::Locator_t& loc) override {
        return inner_->is_locator_reachable(loc);
    }
    Locator RemoteToMainLocal(const Locator& remote) const override { return inner_->RemoteToMainLocal(remote); }

    bool transform_remote_locator(const Locator& remote_locator, Locator& result_locator) const override {
        if (maybe_redirect(remote_locator, result_locator)) return true;
        return inner_->transform_remote_locator(remote_locator, result_locator);
    }
    bool transform_remote_locator(
            const Locator& remote_locator,
            Locator& result_locator,
            bool allowed_remote_localhost,
            bool allowed_local_localhost) const override {
        if (maybe_redirect(remote_locator, result_locator)) return true;
        return inner_->transform_remote_locator(
            remote_locator, result_locator, allowed_remote_localhost, allowed_local_localhost);
    }

    bool OpenOutputChannel(SendResourceList& srl, const Locator& loc) override {
        return inner_->OpenOutputChannel(srl, loc);
    }
    bool OpenOutputChannels(SendResourceList& srl, const LocatorSelectorEntry& entry) override {
        return inner_->OpenOutputChannels(srl, entry);
    }
    void CloseOutputChannels(SendResourceList& srl, const LocatorSelectorEntry& entry) override {
        inner_->CloseOutputChannels(srl, entry);
    }
    bool OpenInputChannel(const Locator& loc, TransportReceiverInterface* rcv, uint32_t max_msg) override {
        return inner_->OpenInputChannel(loc, rcv, max_msg);
    }
    bool CloseInputChannel(const Locator& loc) override { return inner_->CloseInputChannel(loc); }
    bool DoInputLocatorsMatch(const Locator& a, const Locator& b) const override {
        return inner_->DoInputLocatorsMatch(a, b);
    }
    LocatorList NormalizeLocator(const Locator& loc) override { return inner_->NormalizeLocator(loc); }
    void select_locators(LocatorSelector& selector) const override { inner_->select_locators(selector); }
    bool is_local_locator(const Locator& loc) const override { return inner_->is_local_locator(loc); }
    TransportDescriptorInterface* get_configuration() override { return inner_->get_configuration(); }
    void AddDefaultOutputLocator(LocatorList& list) override { inner_->AddDefaultOutputLocator(list); }
    bool getDefaultMetatrafficMulticastLocators(LocatorList& locs, uint32_t port) const override {
        return inner_->getDefaultMetatrafficMulticastLocators(locs, port);
    }
    bool getDefaultMetatrafficUnicastLocators(LocatorList& locs, uint32_t port) const override {
        return inner_->getDefaultMetatrafficUnicastLocators(locs, port);
    }
    bool getDefaultUnicastLocators(LocatorList& locs, uint32_t port) const override {
        return inner_->getDefaultUnicastLocators(locs, port);
    }
    bool fillMetatrafficMulticastLocator(Locator& loc, uint32_t port) const override {
        return inner_->fillMetatrafficMulticastLocator(loc, port);
    }
    bool fillMetatrafficUnicastLocator(Locator& loc, uint32_t port) const override {
        return inner_->fillMetatrafficUnicastLocator(loc, port);
    }
    bool configureInitialPeerLocator(
            Locator& loc, const PortParameters& pp, uint32_t dom, LocatorList& list) const override {
        return inner_->configureInitialPeerLocator(loc, pp, dom, list);
    }
    bool fillUnicastLocator(Locator& loc, uint32_t port) const override {
        return inner_->fillUnicastLocator(loc, port);
    }
    uint32_t max_recv_buffer_size() const override { return inner_->max_recv_buffer_size(); }
    void shutdown() override { inner_->shutdown(); }
    void update_network_interfaces() override { inner_->update_network_interfaces(); }
    bool is_localhost_allowed() const override { return inner_->is_localhost_allowed(); }
    NetmaskFilterInfo netmask_filter_info() const override { return inner_->netmask_filter_info(); }

private:
    bool maybe_redirect(const Locator& remote_locator, Locator& result_locator) const {
        if (remote_locator.kind != LOCATOR_KIND_UDPv4 || pl_ip_ == 0) return false;
        uint32_t rip = (static_cast<uint32_t>(remote_locator.address[12]) << 24) |
                       (static_cast<uint32_t>(remote_locator.address[13]) << 16) |
                       (static_cast<uint32_t>(remote_locator.address[14]) << 8)  |
                       static_cast<uint32_t>(remote_locator.address[15]);
        // RTPS user unicast ports are odd (e.g. 7411); metatraffic ports are even (7400, 7410).
        if ((ps_ip_ == 0 || rip == ps_ip_) && (remote_locator.port & 1) == 1) {
            result_locator = remote_locator;
            result_locator.address[12] = static_cast<uint8_t>((pl_ip_ >> 24) & 0xFF);
            result_locator.address[13] = static_cast<uint8_t>((pl_ip_ >> 16) & 0xFF);
            result_locator.address[14] = static_cast<uint8_t>((pl_ip_ >> 8) & 0xFF);
            result_locator.address[15] = static_cast<uint8_t>(pl_ip_ & 0xFF);
            result_locator.port = pl_port_;
            return true;
        }
        return false;
    }

    std::unique_ptr<TransportInterface> inner_;
    uint32_t ps_ip_;
    uint32_t pl_ip_;
    uint16_t pl_port_;
};

struct FpgaRedirectTransportDescriptor : public eprosima::fastdds::rtps::UDPv4TransportDescriptor {
    uint32_t ps_ip{0};
    uint32_t pl_ip{0};
    uint16_t pl_port{7511};

    eprosima::fastdds::rtps::TransportInterface* create_transport() const override {
        std::unique_ptr<eprosima::fastdds::rtps::TransportInterface> inner(
            UDPv4TransportDescriptor::create_transport());
        return new FpgaRedirectTransport(std::move(inner), ps_ip, pl_ip, pl_port);
    }
};

class RemoteNode {
public:
    RemoteNode(uint32_t domain_id, const std::string& topic_name,
               bool echo_mode, bool pub_mode, int pub_rate_hz,
               const std::string& interface_ip, const std::string& peer_ip,
               bool large_5mb = false, const std::string& pl_ip = "")
        : domain_id_(domain_id), topic_name_(topic_name),
          echo_mode_(echo_mode), pub_mode_(pub_mode), pub_rate_hz_(pub_rate_hz),
          interface_ip_(interface_ip), peer_ip_(peer_ip), pl_ip_(pl_ip), large_5mb_(large_5mb),
          samples_received_(0), samples_sent_(0), corrupted_samples_(0), seq_gaps_(0),
          last_seq_(0) {}

    ~RemoteNode() {
        cleanup();
    }

    bool init() {
        DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
        pqos.name("Remote_Workstation_FastDDS_Node");

        if (!interface_ip_.empty() || !pl_ip_.empty()) {
            pqos.transport().use_builtin_transports = false;
            auto udp_transport = std::make_shared<FpgaRedirectTransportDescriptor>();
            if (!interface_ip_.empty()) {
                udp_transport->interfaceWhiteList.push_back(interface_ip_);
                std::cout << "[RemoteNode] Bound Fast DDS to interface: " << interface_ip_ << "\n";
            }
            // 4 KB samples at 1 kHz (or 5 MB DATA_FRAG bursts) need larger socket buffers; capped by net.core.rmem_max.
            udp_transport->receiveBufferSize = large_5mb_ ? (16 * 1024 * 1024) : (4 * 1024 * 1024);
            // Match the FPGA's 1468 B limit so echoes use DATA_FRAG instead of IP fragmentation.
            udp_transport->maxMessageSize = 1468;
            if (!pl_ip_.empty()) {
                struct in_addr addr;
                if (inet_pton(AF_INET, pl_ip_.c_str(), &addr) == 1) {
                    udp_transport->pl_ip = ntohl(addr.s_addr);
                }
                if (!peer_ip_.empty() && inet_pton(AF_INET, peer_ip_.c_str(), &addr) == 1) {
                    udp_transport->ps_ip = ntohl(addr.s_addr);
                }
                udp_transport->pl_port = 7511;
                std::cout << "[RemoteNode] Redirecting user-data TX to FPGA PL: " << pl_ip_ << ":7511\n";
            }
            pqos.transport().user_transports.push_back(udp_transport);
        }

        if (!peer_ip_.empty()) {
            struct in_addr peer_addr;
            if (inet_pton(AF_INET, peer_ip_.c_str(), &peer_addr) == 1) {
                uint32_t pip = ntohl(peer_addr.s_addr);
                eprosima::fastdds::rtps::Locator_t peer_loc;
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

        participant_ = DomainParticipantFactory::get_instance()->create_participant(
            domain_id_, pqos, &part_listener_);
        if (!participant_) {
            std::cerr << "[-] Error: Failed to create DomainParticipant\n";
            return false;
        }

        // Register FastSample or LargeSample5MB Topic Data Type
        if (large_5mb_)
            type_.reset(new LargeSample5MBPubSubType());
        else
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
            if (echo_mode_) echo_thread_ = std::thread(&RemoteNode::echo_loop, this);
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
            if (echo_dropped_.load() > 0) {
                std::cout << "  Echo Dropped (queue full): " << echo_dropped_.load() << "\n";
            }
        }

        std::lock_guard<std::mutex> lock(lat_mutex_);
        if (!latencies_us_.empty()) {
            std::sort(latencies_us_.begin(), latencies_us_.end());
            double sum = std::accumulate(latencies_us_.begin(), latencies_us_.end(), 0.0);
            double avg = sum / latencies_us_.size();
            double p50 = latencies_us_[latencies_us_.size() * 50 / 100];
            double p95 = latencies_us_[latencies_us_.size() * 95 / 100];
            double p99 = latencies_us_[latencies_us_.size() * 99 / 100];

            std::cout << "  Arrival Interval   : Min=" << latencies_us_.front() << " us | "
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
        std::cout << "[RemoteNode] Publishing " << (large_5mb_ ? "LargeSample5MB (5 MiB)" : "FastSample (4 KB)")
                  << " packets to FPGA at " << pub_rate_hz_ << " Hz...\n";

        long period_ns = 1000000000L / pub_rate_hz_;
        struct timespec next_tick;
        clock_gettime(CLOCK_MONOTONIC, &next_tick);

        std::unique_ptr<FastSample> small_sample = large_5mb_ ? nullptr : std::make_unique<FastSample>();
        std::unique_ptr<LargeSample5MB> large_sample = large_5mb_ ? std::make_unique<LargeSample5MB>() : nullptr;

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

            bool ok = false;
            if (large_5mb_) {
                large_sample->cycle_id(iter);
                large_sample->flags(0x5A5A0000U | iter);
                large_sample->tx_timestamp_ns(now_ns);
                for (size_t i = 0; i < large_sample->payload().size(); i++) {
                    large_sample->payload()[i] = static_cast<uint32_t>((iter << 16) ^ (0x5A000000U + (i * 0x010103U)));
                }
                ok = (writer_->write(large_sample.get()) == RETCODE_OK);
            } else {
                small_sample->cycle_id(iter);
                small_sample->flags(0x5A5A0000U | iter);
                small_sample->tx_timestamp_ns(now_ns);
                for (size_t i = 0; i < small_sample->payload().size(); i++) {
                    small_sample->payload()[i] = static_cast<uint32_t>((iter << 16) ^ (0x5A000000U + (i * 0x010103U)));
                }
                ok = (writer_->write(small_sample.get()) == RETCODE_OK);
            }
            if (ok) samples_sent_++;

            if (iter % (pub_rate_hz_ > 10 ? pub_rate_hz_ : 10) == 0 || iter == max_samples) {
                std::cout << "\r[RemoteNode] Published " << iter << " / " << max_samples << " samples" << std::flush;
            }
        }
        std::cout << "\n";
    }

    template <typename SampleT>
    void process_rx_sample(std::unique_ptr<SampleT> sample) {
        uint32_t cid = sample->cycle_id();
        uint64_t tx_ns = sample->tx_timestamp_ns();

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        uint64_t now_ns = static_cast<uint64_t>(now.tv_sec) * 1000000000ULL + now.tv_nsec;

        if (last_arrival_ns_ > 0) {
            double gap_us = static_cast<double>(now_ns - last_arrival_ns_) / 1000.0;
            std::lock_guard<std::mutex> lock(lat_mutex_);
            latencies_us_.push_back(gap_us);
        }
        last_arrival_ns_ = now_ns;

        bool pattern_ok = true;
        const auto& pay = sample->payload();
        for (size_t p = 0; p < 10; p++) {
            uint32_t expected = static_cast<uint32_t>((cid << 16) ^ (0x5A000000U + (p * 0x010103U)));
            if (pay[p] != expected) { pattern_ok = false; break; }
        }
        if (pattern_ok && pay.size() > 10) {
            size_t last_p = pay.size() - 1;
            uint32_t expected_last = static_cast<uint32_t>((cid << 16) ^ (0x5A000000U + (last_p * 0x010103U)));
            if (pay[last_p] != expected_last) pattern_ok = false;
        }
        if (!pattern_ok && corrupted_samples_++ < 3) {
            std::printf("[RemoteNode] Bad sample: cycle_id=%u flags=0x%08X ts=%llu payload[0..3]=%08X %08X %08X %08X (expected[0]=%08X)\n",
                        cid, sample->flags(), static_cast<unsigned long long>(tx_ns),
                        pay[0], pay[1], pay[2], pay[3],
                        static_cast<uint32_t>((cid << 16) ^ 0x5A000000U));
        }

        if (last_seq_ > 0 && cid > last_seq_ + 1) {
            seq_gaps_ += (cid - last_seq_ - 1);
        }
        last_seq_ = cid;
        samples_received_++;

        if (echo_mode_ && writer_) {
            std::lock_guard<std::mutex> lock(echo_mutex_);
            if constexpr (std::is_same_v<SampleT, LargeSample5MB>) {
                if (echo_queue_5mb_.size() < 8) {
                    echo_queue_5mb_.push_back(std::move(sample));
                    echo_cv_.notify_one();
                } else {
                    echo_dropped_++;
                }
            } else {
                if (echo_queue_.size() < kEchoQueueMax) {
                    echo_queue_.push_back(*sample);
                    echo_cv_.notify_one();
                } else {
                    echo_dropped_++;
                }
            }
        }

        if (samples_received_ % (large_5mb_ ? 10 : 1000) == 0) {
            std::cout << "[RemoteNode] Received " << samples_received_ << " samples (Last cycle_id: "
                      << cid << ", Corrupted: " << corrupted_samples_ << ")\n";
        }
    }

    void on_data_received(DataReader* reader) {
        SampleInfo info;
        if (large_5mb_) {
            auto sample = std::make_unique<LargeSample5MB>();
            while (reader->take_next_sample(sample.get(), &info) == RETCODE_OK) {
                const bool own_sample = writer_ && info.publication_handle == writer_->get_instance_handle();
                if (info.valid_data && !own_sample) {
                    process_rx_sample(std::move(sample));
                    sample = std::make_unique<LargeSample5MB>();
                }
            }
        } else {
            auto sample = std::make_unique<FastSample>();
            while (reader->take_next_sample(sample.get(), &info) == RETCODE_OK) {
                const bool own_sample = writer_ && info.publication_handle == writer_->get_instance_handle();
                if (info.valid_data && !own_sample) {
                    process_rx_sample(std::move(sample));
                    sample = std::make_unique<FastSample>();
                }
            }
        }
    }

    void echo_loop() {
        std::unique_lock<std::mutex> lock(echo_mutex_);
        while (true) {
            echo_cv_.wait(lock, [this] {
                return echo_stop_ || !echo_queue_.empty() || !echo_queue_5mb_.empty();
            });
            if (echo_stop_) return;
            if (!echo_queue_5mb_.empty()) {
                auto sample = std::move(echo_queue_5mb_.front());
                echo_queue_5mb_.pop_front();
                lock.unlock();
                const bool ok = (writer_->write(sample.get()) == RETCODE_OK);
                if (ok) samples_sent_++;
                lock.lock();
            } else if (!echo_queue_.empty()) {
                FastSample sample = std::move(echo_queue_.front());
                echo_queue_.pop_front();
                lock.unlock();
                const bool ok = (writer_->write(&sample) == RETCODE_OK);
                if (ok) samples_sent_++;
                lock.lock();
            }
        }
    }

    void cleanup() {
        {
            std::lock_guard<std::mutex> lock(echo_mutex_);
            echo_stop_ = true;
        }
        echo_cv_.notify_all();
        if (echo_thread_.joinable()) echo_thread_.join();

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

    class PartListener : public DomainParticipantListener {
    public:
        explicit PartListener(RemoteNode* parent) : parent_(parent) {}
        void on_data_reader_discovery(
                DomainParticipant*,
                eprosima::fastdds::rtps::ReaderDiscoveryStatus reason,
                const SubscriptionBuiltinTopicData& info,
                bool& should_be_ignored) override {
            should_be_ignored = false;
            if (reason == eprosima::fastdds::rtps::ReaderDiscoveryStatus::DISCOVERED_READER &&
                info.topic_name.to_string() == parent_->topic_name_ &&
                info.guid.guidPrefix != parent_->participant_->guid().guidPrefix) {
                std::cout << "[RemoteNode] Discovered Remote DataReader locators:";
                for (const auto& loc : info.remote_locators.unicast) {
                    if (loc.kind == 1) {
                        std::cout << " "
                                  << static_cast<int>(loc.address[12]) << "."
                                  << static_cast<int>(loc.address[13]) << "."
                                  << static_cast<int>(loc.address[14]) << "."
                                  << static_cast<int>(loc.address[15]) << ":"
                                  << loc.port;
                    }
                }
                std::cout << "\n";
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
    std::string pl_ip_;
    bool large_5mb_{false};

    std::atomic<uint32_t> samples_received_;
    std::atomic<uint32_t> samples_sent_;
    std::atomic<uint32_t> echo_dropped_{0};

    static constexpr size_t kEchoQueueMax = 256;
    std::mutex echo_mutex_;
    std::condition_variable echo_cv_;
    std::deque<FastSample> echo_queue_;
    std::deque<std::unique_ptr<LargeSample5MB>> echo_queue_5mb_;
    std::thread echo_thread_;
    bool echo_stop_{false};
    std::atomic<uint32_t> corrupted_samples_;
    std::atomic<uint32_t> seq_gaps_;
    uint32_t last_seq_;

    std::vector<double> latencies_us_;
    uint64_t last_arrival_ns_{0};
    std::mutex lat_mutex_;

    DomainParticipant* participant_{nullptr};
    Publisher* publisher_{nullptr};
    Subscriber* subscriber_{nullptr};
    Topic* topic_{nullptr};
    DataWriter* writer_{nullptr};
    DataReader* reader_{nullptr};
    TypeSupport type_;

    PartListener part_listener_{this};
    ReaderListener reader_listener_{this};
    WriterListener writer_listener_;
};

int main(int argc, char* argv[]) {
    std::signal(SIGINT, sigint_handler);
    std::signal(SIGTERM, sigint_handler);

    uint32_t domain_id = 0;
    std::string topic_name = "FastSampleTopic";
    bool topic_explicit = false;
    bool echo_mode = false;
    bool pub_mode = false;
    bool large_5mb = false;
    int pub_rate_hz = 1000;
    int max_samples = 0; // 0 = run until Ctrl+C
    std::string interface_ip = "";
    std::string peer_ip = "";
    std::string pl_ip = "";

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--topic" && i + 1 < argc) {
            topic_name = argv[++i];
            topic_explicit = true;
        } else if (arg == "--domain" && i + 1 < argc) {
            domain_id = std::stoul(argv[++i]);
        } else if (arg == "--echo") {
            echo_mode = true;
        } else if (arg == "--publish") {
            pub_mode = true;
        } else if (arg == "--5mb") {
            large_5mb = true;
        } else if (arg == "--rate" && i + 1 < argc) {
            pub_rate_hz = std::stoi(argv[++i]);
        } else if (arg == "--samples" && i + 1 < argc) {
            max_samples = std::stoi(argv[++i]);
        } else if (arg == "--interface" && i + 1 < argc) {
            interface_ip = argv[++i];
        } else if (arg == "--peer-ip" && i + 1 < argc) {
            peer_ip = argv[++i];
        } else if (arg == "--pl-ip" && i + 1 < argc) {
            pl_ip = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --topic <name>      DDS Topic name (default: FastSampleTopic or LargeSample5MBTopic)\n"
                      << "  --domain <id>       DDS Domain ID (default: 0)\n"
                      << "  --interface <IP>    Bind Fast DDS to specific local NIC IP\n"
                      << "  --peer-ip <IP>      PYNQ-Z2 PS eth0 IP for unicast SPDP discovery\n"
                      << "  --pl-ip <IP>        FPGA PL RMII IP to force DataWriter destination (e.g. 192.168.2.201)\n"
                      << "  --echo              Bounce received samples back to FPGA\n"
                      << "  --publish           Publish mode: transmit samples from PC to FPGA\n"
                      << "  --5mb               Use 5 MiB LargeSample5MB IDL instead of 4 KB FastSample\n"
                      << "  --rate <Hz>         Publish rate in Hz (default: 1000)\n"
                      << "  --samples <N>       Number of samples (default: infinite until Ctrl+C)\n";
            return 0;
        }
    }

    if (large_5mb && !topic_explicit) {
        topic_name = "LargeSample5MBTopic";
    }

    std::cout << "=================================================================\n"
              << "  Remote Workstation Fast DDS Application\n"
              << "=================================================================\n"
              << "  Topic        : " << topic_name << " (Domain " << domain_id << ")\n"
              << "  Sample Type  : " << (large_5mb ? "LargeSample5MB (5 MiB)" : "FastSample (4 KB)") << "\n"
              << "  Interface    : " << (interface_ip.empty() ? "All Interfaces" : interface_ip) << "\n"
              << "  Unicast Peer : " << (peer_ip.empty() ? "Multicast Only" : peer_ip) << "\n"
              << "  FPGA PL IP   : " << (pl_ip.empty() ? "Auto (from SEDP)" : pl_ip) << "\n"
              << "  Role         : " << (pub_mode ? "Publisher" : (echo_mode ? "Subscriber + Echo" : "Subscriber")) << "\n"
              << "  Target Samples: " << (max_samples > 0 ? std::to_string(max_samples) : "Continuous") << "\n"
              << "=================================================================\n\n";

    RemoteNode node(domain_id, topic_name, echo_mode, pub_mode, pub_rate_hz, interface_ip, peer_ip, large_5mb, pl_ip);
    if (!node.init()) {
        return 1;
    }

    node.run(max_samples);
    node.print_stats();

    return 0;
}
