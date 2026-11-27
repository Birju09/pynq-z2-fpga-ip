// ============================================================================
// PureDdsSession.cpp
//
// Pure Software Fast DDS Pub/Sub Session Library Implementation.
// ============================================================================

#include "PureDdsSession.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <iostream>
#include <memory>

#include <fastdds/dds/builtin/topic/SubscriptionBuiltinTopicData.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/rtps/common/Locator.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>

#include "FastSamplePubSubTypes.hpp"
#include "LargeSample5MBPubSubTypes.hpp"

using namespace eprosima::fastdds::dds;

PureDdsSession::PureDdsSession(const fastdds_fpga::BridgeConfig& cfg,
                               bool enable_rx,
                               bool local_echo_peer,
                               const std::atomic<bool>* running_flag)
    : cfg_(cfg),
      enable_rx_(enable_rx),
      local_echo_peer_(local_echo_peer),
      running_flag_(running_flag) {}

PureDdsSession::~PureDdsSession() {
    stop();
}

bool PureDdsSession::init(RxCallback rx_cb, RxCallback5MB rx_cb_5mb) {
    rx_cb_ = std::move(rx_cb);
    rx_cb_5mb_ = std::move(rx_cb_5mb);

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

bool PureDdsSession::wait_for_match(uint32_t timeout_ms) {
    std::unique_lock<std::mutex> lock(match_mutex_);
    if (writer_matched_) return true;

    std::cout << "[PureDDS] Waiting for "
              << (local_echo_peer_ ? "Local Echo Peer" : "Remote Fast DDS Participant")
              << " on topic '" << cfg_.topic_name << "' (timeout: " << timeout_ms / 1000 << "s)...\n";

    bool ok = cv_match_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this]() {
        bool still_running = running_flag_ ? running_flag_->load() : true;
        return writer_matched_.load() || !still_running;
    });

    if (ok && writer_matched_) {
        std::cout << "[PureDDS] SUCCESS: Discovered & matched DDS subscriber endpoint!\n";
        return true;
    }
    std::cerr << "[PureDDS] WARNING: Discovery timeout reached without remote match.\n";
    return false;
}

bool PureDdsSession::publish(FastSample& sample) {
    if (!writer_) return false;
    return (writer_->write(&sample) == RETCODE_OK);
}

bool PureDdsSession::publish_5mb(LargeSample5MB& sample) {
    if (!writer_) return false;
    return (writer_->write(&sample) == RETCODE_OK);
}

void PureDdsSession::stop() {
    auto* factory = DomainParticipantFactory::get_instance();
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

void PureDdsSession::configure_udp_transport(DomainParticipantQos& pqos, bool log_cfg) {
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
    uint32_t sock_sz = cfg_.large_5mb ? (16 * 1024 * 1024) : (4 * 1024 * 1024);
    udp_transport->receiveBufferSize = sock_sz;
    udp_transport->sendBufferSize = sock_sz;
    pqos.transport().user_transports.push_back(udp_transport);

    if (!cfg_.peer_ip.empty() && !local_echo_peer_) {
        struct in_addr peer_addr;
        if (inet_pton(AF_INET, cfg_.peer_ip.c_str(), &peer_addr) == 1) {
            uint32_t pip = ntohl(peer_addr.s_addr);
            eprosima::fastdds::rtps::Locator_t peer_loc;
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

bool PureDdsSession::init_main_participant() {
    DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
    pqos.name("PYNQZ2_PureDDS_Node");
    configure_udp_transport(pqos, true);

    participant_ = DomainParticipantFactory::get_instance()->create_participant(
        cfg_.domain_id, pqos, &part_listener_);
    if (!participant_) return false;

    if (cfg_.large_5mb)
        type_.reset(new LargeSample5MBPubSubType());
    else
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

bool PureDdsSession::init_rx_participant() {
    DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
    pqos.name("PYNQZ2_PureDDS_RxNode");
    configure_udp_transport(pqos, false);

    rx_participant_ = DomainParticipantFactory::get_instance()->create_participant(
        cfg_.domain_id, pqos);
    if (!rx_participant_) return false;

    participant_->ignore_participant(rx_participant_->get_instance_handle());
    rx_participant_->ignore_participant(participant_->get_instance_handle());

    if (cfg_.large_5mb)
        rx_type_.reset(new LargeSample5MBPubSubType());
    else
        rx_type_.reset(new FastSamplePubSubType());
    rx_type_.register_type(rx_participant_);

    rx_topic_ = rx_participant_->create_topic(cfg_.topic_name, rx_type_.get_type_name(), TOPIC_QOS_DEFAULT);
    if (!rx_topic_) return false;

    subscriber_ = rx_participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
    if (!subscriber_) return false;

    DataReaderQos rqos = DATAREADER_QOS_DEFAULT;
    rqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
    rqos.history().kind = KEEP_LAST_HISTORY_QOS;
    rqos.history().depth = 512;
    rqos.resource_limits().max_samples = 1024;
    rqos.resource_limits().allocated_samples = 64;
    rqos.resource_limits().max_instances = 1;
    rqos.resource_limits().max_samples_per_instance = 1024;
    rqos.endpoint().history_memory_policy = eprosima::fastdds::rtps::DYNAMIC_RESERVE_MEMORY_MODE;
    reader_ = subscriber_->create_datareader(rx_topic_, rqos, &reader_listener_);
    return (reader_ != nullptr);
}

bool PureDdsSession::init_echo_peer_participant() {
    DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
    pqos.name("PYNQZ2_PureDDS_EchoPeer");
    configure_udp_transport(pqos, false);

    echo_participant_ = DomainParticipantFactory::get_instance()->create_participant(
        cfg_.domain_id, pqos);
    if (!echo_participant_) return false;

    if (cfg_.large_5mb)
        echo_type_.reset(new LargeSample5MBPubSubType());
    else
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

void PureDdsSession::notify_remote_reader_matched() {
    {
        std::lock_guard<std::mutex> lock(match_mutex_);
        writer_matched_ = true;
    }
    cv_match_.notify_all();
}

void PureDdsSession::on_main_data_available(DataReader* reader) {
    SampleInfo info;
    auto local_prefix = participant_->guid().guidPrefix;

    if (cfg_.large_5mb) {
        auto sample = std::make_unique<LargeSample5MB>();
        while (reader->take_next_sample(sample.get(), &info) == RETCODE_OK) {
            if (!info.valid_data) continue;
            if (info.sample_identity.writer_guid().guidPrefix == local_prefix) continue;
            if (rx_cb_5mb_) rx_cb_5mb_(*sample);
        }
    } else {
        FastSample sample;
        while (reader->take_next_sample(&sample, &info) == RETCODE_OK) {
            if (!info.valid_data) continue;
            if (info.sample_identity.writer_guid().guidPrefix == local_prefix) continue;
            if (rx_cb_) rx_cb_(sample);
        }
    }
}

void PureDdsSession::on_echo_data_available(DataReader* reader) {
    SampleInfo info;
    auto echo_prefix = echo_participant_->guid().guidPrefix;

    if (cfg_.large_5mb) {
        auto sample = std::make_unique<LargeSample5MB>();
        while (reader->take_next_sample(sample.get(), &info) == RETCODE_OK) {
            if (!info.valid_data) continue;
            if (info.sample_identity.writer_guid().guidPrefix == echo_prefix) continue;
            if (echo_writer_) (void)echo_writer_->write(sample.get());
        }
    } else {
        FastSample sample;
        while (reader->take_next_sample(&sample, &info) == RETCODE_OK) {
            if (!info.valid_data) continue;
            if (info.sample_identity.writer_guid().guidPrefix == echo_prefix) continue;
            if (echo_writer_) (void)echo_writer_->write(&sample);
        }
    }
}

void PureDdsSession::PurePartListener::on_data_reader_discovery(
    DomainParticipant*,
    eprosima::fastdds::rtps::ReaderDiscoveryStatus reason,
    const SubscriptionBuiltinTopicData& info,
    bool& should_be_ignored) {
    should_be_ignored = false;
    if (reason == eprosima::fastdds::rtps::ReaderDiscoveryStatus::DISCOVERED_READER &&
        info.topic_name.to_string() == parent_->cfg_.topic_name &&
        info.guid.guidPrefix != parent_->participant_->guid().guidPrefix &&
        !parent_->is_rx_prefix(info.guid.guidPrefix)) {
        parent_->notify_remote_reader_matched();
    }
}

void PureDdsSession::MainReaderListener::on_data_available(DataReader* reader) {
    parent_->on_main_data_available(reader);
}

void PureDdsSession::EchoReaderListener::on_data_available(DataReader* reader) {
    parent_->on_echo_data_available(reader);
}
