// ============================================================================
// PureDdsSession.hpp
//
// Pure Software Fast DDS Pub/Sub Session Library (No FPGA IP / DMA Driver).
// Supports both 4 KB FastSample and 5 MiB LargeSample5MB topics.
// ============================================================================

#ifndef _PURE_DDS_SESSION_HPP_
#define _PURE_DDS_SESSION_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "BridgeConfig.hpp"
#include "FastSample.hpp"
#include "LargeSample5MB.hpp"

class PureDdsSession {
public:
    using RxCallback    = std::function<void(const FastSample&)>;
    using RxCallback5MB = std::function<void(const LargeSample5MB&)>;

    PureDdsSession(const fastdds_fpga::BridgeConfig& cfg,
                   bool enable_rx,
                   bool local_echo_peer,
                   const std::atomic<bool>* running_flag = nullptr);

    ~PureDdsSession();

    bool init(RxCallback rx_cb, RxCallback5MB rx_cb_5mb = nullptr);
    bool wait_for_match(uint32_t timeout_ms = 20000);
    bool publish(FastSample& sample);
    bool publish_5mb(LargeSample5MB& sample);
    void stop();

private:
    void configure_udp_transport(eprosima::fastdds::dds::DomainParticipantQos& pqos, bool log_cfg);
    bool init_main_participant();
    bool init_rx_participant();
    bool init_echo_peer_participant();

    template <typename Prefix>
    bool is_rx_prefix(const Prefix& prefix) const {
        return rx_participant_ && prefix == rx_participant_->guid().guidPrefix;
    }

    void notify_remote_reader_matched();
    void on_main_data_available(eprosima::fastdds::dds::DataReader* reader);
    void on_echo_data_available(eprosima::fastdds::dds::DataReader* reader);

    class PurePartListener : public eprosima::fastdds::dds::DomainParticipantListener {
    public:
        explicit PurePartListener(PureDdsSession* parent) : parent_(parent) {}
        void on_data_reader_discovery(
            eprosima::fastdds::dds::DomainParticipant*,
            eprosima::fastdds::rtps::ReaderDiscoveryStatus reason,
            const eprosima::fastdds::dds::SubscriptionBuiltinTopicData& info,
            bool& should_be_ignored) override;
    private:
        PureDdsSession* parent_;
    };

    class MainReaderListener : public eprosima::fastdds::dds::DataReaderListener {
    public:
        explicit MainReaderListener(PureDdsSession* parent) : parent_(parent) {}
        void on_data_available(eprosima::fastdds::dds::DataReader* reader) override;
    private:
        PureDdsSession* parent_;
    };

    class EchoReaderListener : public eprosima::fastdds::dds::DataReaderListener {
    public:
        explicit EchoReaderListener(PureDdsSession* parent) : parent_(parent) {}
        void on_data_available(eprosima::fastdds::dds::DataReader* reader) override;
    private:
        PureDdsSession* parent_;
    };

    fastdds_fpga::BridgeConfig cfg_;
    bool enable_rx_{true};
    bool local_echo_peer_{false};
    const std::atomic<bool>* running_flag_{nullptr};
    RxCallback rx_cb_;
    RxCallback5MB rx_cb_5mb_;

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

#endif // _PURE_DDS_SESSION_HPP_
