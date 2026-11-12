// ============================================================================
// FpgaFastpathBridge.hpp
//
// Fast DDS Option 3 Hybrid Bridge:
// - Control Plane : Fast DDS handles standard SPDP/SEDP Discovery over PS eth0
// - Data Plane    : Direct zero-copy publishing & receiving via FPGA Fastpath IP
//                   over PL RMII PHY (bypasses software serializer & netstack)
// ============================================================================

#ifndef _FPGA_FASTPATH_BRIDGE_HPP_
#define _FPGA_FASTPATH_BRIDGE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

// Fast DDS C++ API
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#if __has_include(<fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>)
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>
#else
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#endif

#if __has_include(<fastdds/config.hpp>)
#include <fastdds/config.hpp>
#endif

#if defined(FASTDDS_VERSION_MAJOR) && (FASTDDS_VERSION_MAJOR >= 3)
#define FASTDDS_V3 1
#include <fastdds/dds/builtin/topic/SubscriptionBuiltinTopicData.hpp>
#include <fastdds/dds/builtin/topic/PublicationBuiltinTopicData.hpp>
#include <fastdds/rtps/common/Locator.hpp>
#else
#define FASTDDS_V3 0
#include <fastrtps/rtps/builtin/data/ParticipantProxyData.h>
#include <fastrtps/rtps/builtin/data/ReaderProxyData.h>
#include <fastrtps/rtps/builtin/data/WriterProxyData.h>
#include <fastrtps/rtps/common/Locator.h>
#endif

// Hardware Driver Headers
#define FastSample FpgaHardwareFastSample
#include "axi_rtps.h"
#include "rtps_fpga_transport.h"
#undef FastSample

#include "../idl/FastSample.hpp"
#include "../idl/FastSamplePubSubTypes.hpp"

namespace fastdds_fpga {

struct BridgeConfig {
    std::string device_path{"/dev/" XILINX_RTPS_DEV_NAME};
    uint32_t    domain_id{0};
    std::string topic_name{"FastSampleTopic"};

    // PS Control-Plane Discovery Network (Linux eth0)
    std::string ps_ip{""};                 // Empty = bind all interfaces; or e.g. "192.168.1.10"
    std::string peer_ip{""};               // Optional unicast SPDP peer IP, e.g. "192.168.1.100"

    // PL Data-Plane Hardware Identity (RMII PHY)
    uint32_t    pl_ip{0xC0A801C9};         // 192.168.1.201 (dedicated PL RMII IP)
    uint8_t     pl_mac[6]{0x02, 0x00, 0x5E, 0x10, 0x00, 0x01};
    uint16_t    local_src_port{7411};
    uint16_t    pl_rx_port{7511};          // Reader locator advertised via SEDP; must not collide with PS default ports 7410/7411
    uint16_t    default_dst_port{7410};
    uint32_t    fallback_dst_ip{0xC0A80164}; // 192.168.1.100

    bool        enable_peer_emu{false};    // false = Physical RMII wire (LAN8720), true = internal emulator
    bool        tx_ip_frag_mode{false};    // false = Option C DATA_FRAG default (0x16), true = Legacy IPv4 frag
    uint32_t    max_message_size{1468};    // Fast DDS UDP maxMessageSize (1468 - 68B RTPS/DATA_FRAG = 1400B)
    uint32_t    frag_size_bytes{0};        // Optional explicit DATA_FRAG fragmentSize override (0 = derive from max_message_size)
    uint8_t     override_peer_mac[6]{0};
    bool        has_peer_mac_override{false};
    uint32_t    pacing_timer_ticks{0};     // 0 = wire line rate
};

class FpgaFastpathBridge {
public:
    explicit FpgaFastpathBridge(const BridgeConfig& config)
        : config_(config), running_(true), writer_matched_(false), reader_matched_(false), fpga_(nullptr) {}

    ~FpgaFastpathBridge() {
        stop();
    }

    uint32_t effective_frag_size_bytes() const {
        uint32_t sz = config_.frag_size_bytes;
        if (sz == 0) {
            constexpr uint32_t kRtpsDataFragUdpHdrBytes = 68U; // 20B RTPS + 12B INFO_TS + 36B DATA_FRAG
            sz = (config_.max_message_size > kRtpsDataFragUdpHdrBytes)
                     ? (config_.max_message_size - kRtpsDataFragUdpHdrBytes)
                     : 1400U;
        }
        sz &= ~3U;
        if (sz < 4U) sz = 4U;
        if (sz > 1404U) sz = 1404U;
        return sz;
    }

    bool init() {
        std::cout << "[FpgaBridge] Opening FPGA RTPS Fastpath device: " << config_.device_path << "...\n";
        int rc = fpga_fastpath_open(config_.device_path.c_str(), 1, &fpga_);
        if (rc < 0 || !fpga_) {
            std::cerr << "[-] Error: fpga_fastpath_open failed: " << strerror(-rc) << " (" << rc << ")\n";
            return false;
        }

        std::cout << "[FpgaBridge] FPGA driver initialized (fd=" << fpga_->fd
                  << ", coherent mmap=" << (fpga_->use_mmap ? "ENABLED" : "DISABLED") << ")\n";

        // Pet watchdog immediately to clear any prior expired state
        pet_watchdog();

        // Start background watchdog pet thread (30 ms cadence < 100 ms HW limit)
        wdog_thread_ = std::thread([this]() {
            while (running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                pet_watchdog();
            }
        });

        // Initialize Fast DDS Participant for Discovery Plane (SPDP / SEDP)
        if (!init_fastdds()) {
            std::cerr << "[-] Error: Fast DDS discovery initialization failed\n";
            return false;
        }

        return true;
    }

    void stop() {
        running_ = false;
        cv_match_.notify_all();

        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
        if (wdog_thread_.joinable()) {
            wdog_thread_.join();
        }

        if (participant_) {
            if (publisher_ && writer_) {
                publisher_->delete_datawriter(writer_);
            }
            if (subscriber_ && reader_) {
                subscriber_->delete_datareader(reader_);
            }
            if (publisher_) {
                participant_->delete_publisher(publisher_);
            }
            if (subscriber_) {
                participant_->delete_subscriber(subscriber_);
            }
            if (topic_) {
                participant_->delete_topic(topic_);
            }
            eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
            participant_ = nullptr;
        }

        if (fpga_) {
            fpga_fastpath_drain_and_disable_writer(fpga_, 0);
            fpga_fastpath_drain_and_disable_reader(fpga_, 0);
            fpga_fastpath_close(fpga_);
            fpga_ = nullptr;
        }
    }

    bool wait_for_match(uint32_t timeout_ms = 15000) {
        std::unique_lock<std::mutex> lock(match_mutex_);
        if (writer_matched_) return true;

        std::cout << "[FpgaBridge] Waiting for Remote Fast DDS Participant on topic '"
                  << config_.topic_name << "' (timeout: " << timeout_ms / 1000 << "s)...\n";

        bool ok = cv_match_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this]() {
            return writer_matched_.load();
        });

        if (ok) {
            std::cout << "[FpgaBridge] SUCCESS: Discovered & matched remote DDS endpoint!\n";
        } else {
            std::cerr << "[FpgaBridge] WARNING: Discovery timeout reached without remote match.\n";
        }
        return ok;
    }

    // Configure FPGA using fallback/CLI settings while preserving wire vs emulator mode
    bool configure_default_endpoints() {
        if (!fpga_) return false;
        std::lock_guard<std::mutex> lock(hw_cfg_mutex_);

        fpga_->cfg.enable_peer_emu = config_.enable_peer_emu ? 1 : 0;
        fpga_->cfg.mode_pub_only   = config_.enable_peer_emu ? 0 : 1;
        fpga_->cfg.tx_ip_frag_mode = config_.tx_ip_frag_mode ? 1 : 0;
        fpga_->cfg.frag_size_bytes = effective_frag_size_bytes();

        uint32_t dst_ip = config_.fallback_dst_ip;
        uint8_t dst_mac[6] = {0, 0, 0, 0, 0, 0};
        if (config_.has_peer_mac_override) {
            std::memcpy(dst_mac, config_.override_peer_mac, 6);
        } else if (!config_.enable_peer_emu) {
            resolve_mac_from_arp(dst_ip, dst_mac);
        }

        struct fpga_writer_endpoint wep;
        std::memset(&wep, 0, sizeof(wep));
        wep.slot_idx         = 0;
        wep.lifecycle_state  = FPGA_LIFECYCLE_ACTIVE;
        wep.payload_words    = RTPS_FAST_SAMPLE_WORDS;
        wep.dst_ip           = dst_ip;
        wep.src_port         = config_.local_src_port;
        wep.dst_port         = config_.default_dst_port;
        std::memcpy(wep.dst_mac, dst_mac, 6);
        wep.writer_entity_id = 0x00000102;
        wep.reader_entity_id = 0x00000000;
        wep.seq_seed         = 1;
        wep.generation       = ++generation_;

        struct fpga_reader_endpoint rep;
        std::memset(&rep, 0, sizeof(rep));
        rep.slot_idx                = 0;
        rep.lifecycle_state         = FPGA_LIFECYCLE_ACTIVE;
        rep.expected_dst_port       = config_.pl_rx_port;
        rep.remote_guid_prefix[0]   = 0x01; rep.remote_guid_prefix[1] = 0x01;
        rep.remote_guid_prefix[2]   = 0x00; rep.remote_guid_prefix[3] = 0x02;
        rep.remote_guid_prefix[4]   = 0xBB; rep.remote_guid_prefix[5] = 0x00;
        rep.remote_guid_prefix[6]   = 0x00; rep.remote_guid_prefix[7] = 0x02;
        rep.remote_guid_prefix[8]   = 0x00; rep.remote_guid_prefix[9] = 0x00;
        rep.remote_guid_prefix[10]  = 0x00; rep.remote_guid_prefix[11] = 0x02;
        rep.remote_writer_entity_id = 0x00000202;
        rep.local_reader_entity_id  = 0x00000107;
        rep.expected_remote_src_ip  = dst_ip;
        rep.generation              = generation_;

        std::cout << "[FpgaBridge] Configuring Default Hardware Endpoints ("
                  << (config_.enable_peer_emu ? "Peer Emulator" : "Physical RMII Wire")
                  << "): PL_IP=" << ip_to_str(config_.pl_ip)
                  << " -> Dst=" << ip_to_str(dst_ip) << ":" << wep.dst_port << "\n";

        fpga_configure_writer(fpga_, &wep);
        fpga_configure_reader(fpga_, &rep);
        fpga_commit_endpoints(fpga_, generation_);
        return true;
    }

    // Direct Zero-Copy Loan: returns pointer directly into coherent DDR DMA buffer
    FastSample* zero_copy_loan() {
        if (!fpga_ || !fpga_->use_mmap || !fpga_->tx_buf) return nullptr;
        return reinterpret_cast<FastSample*>(fpga_->tx_buf);
    }

    // Direct Hardware Publish: kicks AXI DMA MM2S and returns immediately (Total CPU: < 0.3 us)
    int publish(const FastSample& sample) {
        if (!fpga_) return -ENODEV;
        return fpga_fastpath_publish(fpga_, 0, &sample, sizeof(FastSample), sample.m_tx_timestamp_ns, 5000000);
    }

    int publish_loaned() {
        if (!fpga_ || !fpga_->tx_buf) return -ENODEV;
        FastSample* fs = reinterpret_cast<FastSample*>(fpga_->tx_buf);
        return fpga_fastpath_publish(fpga_, 0, fs, sizeof(FastSample), fs->m_tx_timestamp_ns, 5000000);
    }

    // Starts an asynchronous zero-copy receive loop on FPGA S2MM
    void start_rx_worker(std::function<void(const FastSample&, const struct fpga_rx_metadata&)> callback) {
        rx_thread_ = std::thread([this, callback]() {
            std::cout << "[FpgaBridge] S2MM RX worker started.\n";
            while (running_) {
                const void* rx_payload = nullptr;
                size_t rx_len = 0;
                struct fpga_rx_metadata meta;
                std::memset(&meta, 0, sizeof(meta));

                int rc = fpga_fastpath_take(fpga_, 0, &rx_payload, &rx_len, &meta, 5000000);
                if (rc == 0 && rx_payload && rx_len >= sizeof(FastSample)) {
                    const FastSample* fs = reinterpret_cast<const FastSample*>(rx_payload);
                    callback(*fs, meta);
                    fpga_fastpath_release(fpga_, rx_payload);
                } else if (rc != -EAGAIN && rc != -ETIMEDOUT && running_) {
                    std::this_thread::sleep_for(std::chrono::microseconds(500));
                }
            }
            std::cout << "[FpgaBridge] S2MM RX worker stopped.\n";
        });
    }

    bool get_diagnostics(struct rtps_cycle_diag* diag) {
        if (!fpga_ || !diag) return false;
        return (ioctl(fpga_->fd, AXI_RTPS_IOC_GET_DIAG, diag) == 0);
    }

    bool is_matched() const { return writer_matched_.load(); }
    struct fpga_fastpath* get_raw_fpga() { return fpga_; }

private:
    void pet_watchdog() {
        if (fpga_ && fpga_->fd >= 0) {
            uint32_t pet = 1;
            ioctl(fpga_->fd, AXI_RTPS_IOC_PET_WATCHDOG, &pet);
        }
    }

    static std::string ip_to_str(uint32_t ip) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
                 (ip >> 24) & 0xFF, (ip >> 16) & 0xFF, (ip >> 8) & 0xFF, ip & 0xFF);
        return std::string(buf);
    }

    bool resolve_mac_from_arp(uint32_t ip_host_order, uint8_t mac_out[6]) {
        if (config_.has_peer_mac_override) {
            std::memcpy(mac_out, config_.override_peer_mac, 6);
            return true;
        }

        std::string ip_str = ip_to_str(ip_host_order);

        // Probe peer with a 1-byte UDP datagram to prime Linux ARP cache
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock >= 0) {
            struct sockaddr_in sin;
            std::memset(&sin, 0, sizeof(sin));
            sin.sin_family = AF_INET;
            sin.sin_port = htons(config_.default_dst_port);
            sin.sin_addr.s_addr = htonl(ip_host_order);
            char probe = 0;
            sendto(sock, &probe, 1, 0, (struct sockaddr*)&sin, sizeof(sin));
            close(sock);
            usleep(10000); // 10 ms for ARP resolution
        }

        std::ifstream arp_file("/proc/net/arp");
        if (arp_file.is_open()) {
            std::string line;
            std::getline(arp_file, line); // Skip header
            while (std::getline(arp_file, line)) {
                std::istringstream iss(line);
                std::string arp_ip, hw_type, flags, hw_address;
                if (iss >> arp_ip >> hw_type >> flags >> hw_address) {
                    if (arp_ip == ip_str && hw_address.length() == 17 && hw_address != "00:00:00:00:00:00") {
                        unsigned int m[6];
                        if (sscanf(hw_address.c_str(), "%x:%x:%x:%x:%x:%x",
                                   &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6) {
                            for (int i = 0; i < 6; i++) mac_out[i] = static_cast<uint8_t>(m[i]);
                            std::cout << "[FpgaBridge] Resolved Remote MAC for " << ip_str
                                      << " -> " << hw_address << "\n";
                            return true;
                        }
                    }
                }
            }
        }

        // Leave all-zeros so rtps_fastpath_ip.v substitutes ingress/ARP-learned MAC automatically
        std::memset(mac_out, 0, 6);
        std::cout << "[FpgaBridge] Note: MAC not in Linux ARP cache for " << ip_str
                  << "; FPGA hardware will auto-learn via PL ARP/ingress.\n";
        return false;
    }

    bool init_fastdds() {
        using namespace eprosima::fastdds::dds;

        DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
        pqos.name("PYNQZ2_FPGA_Fastpath_Node");

        // Configure UDPv4 transport for Discovery Plane (always apply maxMessageSize for DATA_FRAG alignment)
        {
            pqos.transport().use_builtin_transports = false;
#if FASTDDS_V3
            auto udp_transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
#else
            auto udp_transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
#endif
            if (!config_.ps_ip.empty()) {
                udp_transport->interfaceWhiteList.push_back(config_.ps_ip);
                std::cout << "[FpgaBridge] Fast DDS Discovery bound to PS interface: " << config_.ps_ip << "\n";
            }
            if (config_.max_message_size >= 100 && config_.max_message_size <= 65500U) {
                udp_transport->maxMessageSize = config_.max_message_size;
            }
            pqos.transport().user_transports.push_back(udp_transport);
        }

        // Optional: Add unicast SPDP peer locator (useful when multicast is filtered)
        if (!config_.peer_ip.empty()) {
            struct in_addr peer_addr;
            if (inet_pton(AF_INET, config_.peer_ip.c_str(), &peer_addr) == 1) {
                uint32_t pip = ntohl(peer_addr.s_addr);
#if FASTDDS_V3
                eprosima::fastdds::rtps::Locator_t peer_loc;
#else
                eprosima::fastrtps::rtps::Locator_t peer_loc;
#endif
                peer_loc.kind = 1; // LOCATOR_KIND_UDPv4
                peer_loc.port = 0; // Fast DDS probes all participant ID ports on this host
                peer_loc.address[12] = static_cast<uint8_t>((pip >> 24) & 0xFF);
                peer_loc.address[13] = static_cast<uint8_t>((pip >> 16) & 0xFF);
                peer_loc.address[14] = static_cast<uint8_t>((pip >> 8) & 0xFF);
                peer_loc.address[15] = static_cast<uint8_t>(pip & 0xFF);
                pqos.wire_protocol().builtin.initialPeersList.push_back(peer_loc);
                std::cout << "[FpgaBridge] Added Unicast SPDP Peer: " << config_.peer_ip << "\n";
            }
        }

        participant_ = DomainParticipantFactory::get_instance()->create_participant(
            config_.domain_id, pqos, &part_listener_);
        if (!participant_) return false;

        // Synchronize FPGA Local Identity with Fast DDS Participant GUID
        auto part_guid = participant_->guid();
        fpga_->cfg.local_guid_prefix[0] = (static_cast<uint32_t>(part_guid.guidPrefix.value[0]) << 24) |
                                          (static_cast<uint32_t>(part_guid.guidPrefix.value[1]) << 16) |
                                          (static_cast<uint32_t>(part_guid.guidPrefix.value[2]) << 8)  |
                                          (static_cast<uint32_t>(part_guid.guidPrefix.value[3]));
        fpga_->cfg.local_guid_prefix[1] = (static_cast<uint32_t>(part_guid.guidPrefix.value[4]) << 24) |
                                          (static_cast<uint32_t>(part_guid.guidPrefix.value[5]) << 16) |
                                          (static_cast<uint32_t>(part_guid.guidPrefix.value[6]) << 8)  |
                                          (static_cast<uint32_t>(part_guid.guidPrefix.value[7]));
        fpga_->cfg.local_guid_prefix[2] = (static_cast<uint32_t>(part_guid.guidPrefix.value[8]) << 24) |
                                          (static_cast<uint32_t>(part_guid.guidPrefix.value[9]) << 16) |
                                          (static_cast<uint32_t>(part_guid.guidPrefix.value[10]) << 8) |
                                          (static_cast<uint32_t>(part_guid.guidPrefix.value[11]));

        fpga_->cfg.local_ip             = config_.pl_ip;
        fpga_->cfg.local_mac_hi         = (static_cast<uint16_t>(config_.pl_mac[0]) << 8) | config_.pl_mac[1];
        fpga_->cfg.local_mac_lo         = (static_cast<uint32_t>(config_.pl_mac[2]) << 24) |
                                          (static_cast<uint32_t>(config_.pl_mac[3]) << 16) |
                                          (static_cast<uint32_t>(config_.pl_mac[4]) << 8)  |
                                          config_.pl_mac[5];
        fpga_->cfg.enable_peer_emu      = config_.enable_peer_emu ? 1 : 0;
        fpga_->cfg.mode_pub_only        = 1;
        fpga_->cfg.tx_ip_frag_mode      = config_.tx_ip_frag_mode ? 1 : 0;
        fpga_->cfg.frag_size_bytes      = effective_frag_size_bytes();
        fpga_->cfg.pacing_timer_ticks   = config_.pacing_timer_ticks;
        fpga_->cfg.local_identity_valid = 1;

        // Register Topic & Type
        type_.reset(new FastSamplePubSubType());
        type_.register_type(participant_);

        topic_ = participant_->create_topic(config_.topic_name, type_.get_type_name(), TOPIC_QOS_DEFAULT);
        if (!topic_) return false;

        // Create Publisher & DataWriter (Discovery Only; Data goes via FPGA)
        publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (!publisher_) return false;

        DataWriterQos wqos = DATAWRITER_QOS_DEFAULT;
        wqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
        writer_ = publisher_->create_datawriter(topic_, wqos, &writer_listener_);
        if (!writer_) return false;

        // Create Subscriber & DataReader
        subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (!subscriber_) return false;

        DataReaderQos rqos = DATAREADER_QOS_DEFAULT;
        rqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;

        // Remote writers send data to the PL RMII address, not the PS interface.
        {
#if FASTDDS_V3
            eprosima::fastdds::rtps::Locator_t rx_loc;
#else
            eprosima::fastrtps::rtps::Locator_t rx_loc;
#endif
            rx_loc.kind    = 1; // LOCATOR_KIND_UDPv4
            rx_loc.port    = config_.pl_rx_port;
            rx_loc.address[12] = static_cast<uint8_t>((config_.pl_ip >> 24) & 0xFF);
            rx_loc.address[13] = static_cast<uint8_t>((config_.pl_ip >> 16) & 0xFF);
            rx_loc.address[14] = static_cast<uint8_t>((config_.pl_ip >> 8) & 0xFF);
            rx_loc.address[15] = static_cast<uint8_t>(config_.pl_ip & 0xFF);
            rqos.endpoint().unicast_locator_list.push_back(rx_loc);
        }

        reader_ = subscriber_->create_datareader(topic_, rqos, &reader_listener_);
        if (!reader_) return false;

        return true;
    }

    void commit_discovered_remote_reader(
        uint32_t remote_ip,
        uint16_t remote_port,
        uint32_t remote_reader_entity_id) {
        std::lock_guard<std::mutex> lock(hw_cfg_mutex_);

        uint8_t dst_mac[6];
        resolve_mac_from_arp(remote_ip, dst_mac);

        struct fpga_writer_endpoint wep;
        std::memset(&wep, 0, sizeof(wep));
        wep.slot_idx        = 0;
        wep.lifecycle_state = FPGA_LIFECYCLE_ACTIVE;
        wep.payload_words   = RTPS_FAST_SAMPLE_WORDS;
        wep.dst_ip          = remote_ip;
        wep.src_port        = config_.local_src_port;
        wep.dst_port        = remote_port;
        std::memcpy(wep.dst_mac, dst_mac, 6);

        auto writer_guid = writer_->guid();
        wep.writer_entity_id = (static_cast<uint32_t>(writer_guid.entityId.value[0]) << 24) |
                               (static_cast<uint32_t>(writer_guid.entityId.value[1]) << 16) |
                               (static_cast<uint32_t>(writer_guid.entityId.value[2]) << 8)  |
                               static_cast<uint32_t>(writer_guid.entityId.value[3]);
        wep.reader_entity_id = remote_reader_entity_id;
        wep.seq_seed         = 1;
        wep.generation       = ++generation_;

        std::cout << "[FpgaBridge] SEDP Matched Remote Reader -> Programming FPGA Writer[0]: "
                  << ip_to_str(config_.pl_ip) << ":" << wep.src_port
                  << " -> Dst=" << ip_to_str(remote_ip) << ":" << remote_port
                  << " (WriterId=0x" << std::hex << wep.writer_entity_id
                  << ", ReaderId=0x" << wep.reader_entity_id << std::dec << ")\n";

        fpga_->cfg.enable_peer_emu = config_.enable_peer_emu ? 1 : 0;
        fpga_->cfg.mode_pub_only   = 1;
        fpga_->cfg.tx_ip_frag_mode = config_.tx_ip_frag_mode ? 1 : 0;
        fpga_->cfg.frag_size_bytes = effective_frag_size_bytes();
        fpga_configure_writer(fpga_, &wep);
        fpga_commit_endpoints(fpga_, generation_);

        {
            std::lock_guard<std::mutex> mlock(match_mutex_);
            writer_matched_ = true;
        }
        cv_match_.notify_all();
    }

    void commit_discovered_remote_writer(
        uint32_t remote_ip,
        const uint8_t remote_guid_prefix[12],
        uint32_t remote_writer_entity_id) {
        std::lock_guard<std::mutex> lock(hw_cfg_mutex_);

        struct fpga_reader_endpoint rep;
        std::memset(&rep, 0, sizeof(rep));
        rep.slot_idx                = 0;
        rep.lifecycle_state         = FPGA_LIFECYCLE_ACTIVE;
        rep.expected_dst_port       = config_.pl_rx_port;
        std::memcpy(rep.remote_guid_prefix, remote_guid_prefix, 12);
        rep.remote_writer_entity_id = remote_writer_entity_id;
        rep.expected_remote_src_ip  = remote_ip;

        auto rguid = reader_->guid();
        rep.local_reader_entity_id = (static_cast<uint32_t>(rguid.entityId.value[0]) << 24) |
                                     (static_cast<uint32_t>(rguid.entityId.value[1]) << 16) |
                                     (static_cast<uint32_t>(rguid.entityId.value[2]) << 8)  |
                                     static_cast<uint32_t>(rguid.entityId.value[3]);
        rep.generation = ++generation_;

        std::cout << "[FpgaBridge] SEDP Matched Remote Writer -> Programming FPGA Reader CAM[0]: "
                  << "Src=" << ip_to_str(remote_ip)
                  << " (RemoteWriterId=0x" << std::hex << rep.remote_writer_entity_id
                  << ", LocalReaderId=0x" << rep.local_reader_entity_id << std::dec << ")\n";

        fpga_configure_reader(fpga_, &rep);
        fpga_commit_endpoints(fpga_, generation_);
        reader_matched_ = true;
    }

    class BridgePartListener : public eprosima::fastdds::dds::DomainParticipantListener {
    public:
        explicit BridgePartListener(FpgaFastpathBridge* parent) : parent_(parent) {}

#if FASTDDS_V3
        void on_participant_discovery(
            eprosima::fastdds::dds::DomainParticipant*,
            eprosima::fastdds::rtps::ParticipantDiscoveryStatus reason,
            const eprosima::fastdds::rtps::ParticipantBuiltinTopicData& info,
            bool& should_be_ignored) override {
            should_be_ignored = false;
            if (reason == eprosima::fastdds::rtps::ParticipantDiscoveryStatus::DISCOVERED_PARTICIPANT &&
                info.guid.guidPrefix != parent_->participant_->guid().guidPrefix) {
                std::cout << "[FpgaBridge] SPDP Discovered Remote Participant: "
                          << info.participant_name.to_string() << "\n";
            }
        }

        void on_data_reader_discovery(
            eprosima::fastdds::dds::DomainParticipant*,
            eprosima::fastdds::rtps::ReaderDiscoveryStatus reason,
            const eprosima::fastdds::dds::SubscriptionBuiltinTopicData& info,
            bool& should_be_ignored) override {
            should_be_ignored = false;
            if (reason == eprosima::fastdds::rtps::ReaderDiscoveryStatus::DISCOVERED_READER &&
                info.topic_name.to_string() == parent_->config_.topic_name &&
                info.guid.guidPrefix != parent_->participant_->guid().guidPrefix) {
                uint32_t remote_ip = parent_->config_.fallback_dst_ip;
                uint16_t remote_port = parent_->config_.default_dst_port;
                for (const auto& loc : info.remote_locators.unicast) {
                    if (loc.kind == 1 /* LOCATOR_KIND_UDPv4 */) {
                        remote_ip = (static_cast<uint32_t>(loc.address[12]) << 24) |
                                    (static_cast<uint32_t>(loc.address[13]) << 16) |
                                    (static_cast<uint32_t>(loc.address[14]) << 8)  |
                                    static_cast<uint32_t>(loc.address[15]);
                        remote_port = static_cast<uint16_t>(loc.port);
                        break;
                    }
                }
                uint32_t eid = (static_cast<uint32_t>(info.guid.entityId.value[0]) << 24) |
                               (static_cast<uint32_t>(info.guid.entityId.value[1]) << 16) |
                               (static_cast<uint32_t>(info.guid.entityId.value[2]) << 8)  |
                               static_cast<uint32_t>(info.guid.entityId.value[3]);
                parent_->commit_discovered_remote_reader(remote_ip, remote_port, eid);
            }
        }

        void on_data_writer_discovery(
            eprosima::fastdds::dds::DomainParticipant*,
            eprosima::fastdds::rtps::WriterDiscoveryStatus reason,
            const eprosima::fastdds::dds::PublicationBuiltinTopicData& info,
            bool& should_be_ignored) override {
            should_be_ignored = false;
            if (reason == eprosima::fastdds::rtps::WriterDiscoveryStatus::DISCOVERED_WRITER &&
                info.topic_name.to_string() == parent_->config_.topic_name &&
                info.guid.guidPrefix != parent_->participant_->guid().guidPrefix) {
                uint32_t remote_ip = 0;
                for (const auto& loc : info.remote_locators.unicast) {
                    if (loc.kind == 1 /* LOCATOR_KIND_UDPv4 */) {
                        remote_ip = (static_cast<uint32_t>(loc.address[12]) << 24) |
                                    (static_cast<uint32_t>(loc.address[13]) << 16) |
                                    (static_cast<uint32_t>(loc.address[14]) << 8)  |
                                    static_cast<uint32_t>(loc.address[15]);
                        break;
                    }
                }
                uint32_t eid = (static_cast<uint32_t>(info.guid.entityId.value[0]) << 24) |
                               (static_cast<uint32_t>(info.guid.entityId.value[1]) << 16) |
                               (static_cast<uint32_t>(info.guid.entityId.value[2]) << 8)  |
                               static_cast<uint32_t>(info.guid.entityId.value[3]);
                parent_->commit_discovered_remote_writer(remote_ip, info.guid.guidPrefix.value, eid);
            }
        }
#else
        void on_participant_discovery(
            eprosima::fastdds::dds::DomainParticipant*,
            eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&& info) override {
            if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT &&
                info.info.m_guid.guidPrefix != parent_->participant_->guid().guidPrefix) {
                std::cout << "[FpgaBridge] SPDP Discovered Remote Participant: "
                          << info.info.m_participantName.to_string() << "\n";
            }
        }

        void on_subscriber_discovery(
            eprosima::fastdds::dds::DomainParticipant*,
            eprosima::fastrtps::rtps::ReaderDiscoveryInfo&& info) override {
            if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER &&
                info.info.topicName().to_string() == parent_->config_.topic_name &&
                info.info.guid().guidPrefix != parent_->participant_->guid().guidPrefix) {
                uint32_t remote_ip = parent_->config_.fallback_dst_ip;
                uint16_t remote_port = parent_->config_.default_dst_port;
                for (const auto& loc : info.info.remote_locators().unicast) {
                    if (loc.kind == 1 /* LOCATOR_KIND_UDPv4 */) {
                        remote_ip = (static_cast<uint32_t>(loc.address[12]) << 24) |
                                    (static_cast<uint32_t>(loc.address[13]) << 16) |
                                    (static_cast<uint32_t>(loc.address[14]) << 8)  |
                                    static_cast<uint32_t>(loc.address[15]);
                        remote_port = static_cast<uint16_t>(loc.port);
                        break;
                    }
                }
                auto guid = info.info.guid();
                uint32_t eid = (static_cast<uint32_t>(guid.entityId.value[0]) << 24) |
                               (static_cast<uint32_t>(guid.entityId.value[1]) << 16) |
                               (static_cast<uint32_t>(guid.entityId.value[2]) << 8)  |
                               static_cast<uint32_t>(guid.entityId.value[3]);
                parent_->commit_discovered_remote_reader(remote_ip, remote_port, eid);
            }
        }

        void on_publisher_discovery(
            eprosima::fastdds::dds::DomainParticipant*,
            eprosima::fastrtps::rtps::WriterDiscoveryInfo&& info) override {
            if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::DISCOVERED_WRITER &&
                info.info.topicName().to_string() == parent_->config_.topic_name &&
                info.info.guid().guidPrefix != parent_->participant_->guid().guidPrefix) {
                uint32_t remote_ip = 0;
                for (const auto& loc : info.info.remote_locators().unicast) {
                    if (loc.kind == 1 /* LOCATOR_KIND_UDPv4 */) {
                        remote_ip = (static_cast<uint32_t>(loc.address[12]) << 24) |
                                    (static_cast<uint32_t>(loc.address[13]) << 16) |
                                    (static_cast<uint32_t>(loc.address[14]) << 8)  |
                                    static_cast<uint32_t>(loc.address[15]);
                        break;
                    }
                }
                auto guid = info.info.guid();
                uint32_t eid = (static_cast<uint32_t>(guid.entityId.value[0]) << 24) |
                               (static_cast<uint32_t>(guid.entityId.value[1]) << 16) |
                               (static_cast<uint32_t>(guid.entityId.value[2]) << 8)  |
                               static_cast<uint32_t>(guid.entityId.value[3]);
                parent_->commit_discovered_remote_writer(remote_ip, guid.guidPrefix.value, eid);
            }
        }
#endif

    private:
        FpgaFastpathBridge* parent_;
    };

    class BridgeWriterListener : public eprosima::fastdds::dds::DataWriterListener {
    public:
        explicit BridgeWriterListener(FpgaFastpathBridge* parent) : parent_(parent) {}
        void on_publication_matched(eprosima::fastdds::dds::DataWriter*,
                                    const eprosima::fastdds::dds::PublicationMatchedStatus&) override {
            // Note: Remote endpoint matching is handled via BridgePartListener (filtered by guidPrefix)
        }
    private:
        FpgaFastpathBridge* parent_;
    };

    class BridgeReaderListener : public eprosima::fastdds::dds::DataReaderListener {
    public:
        explicit BridgeReaderListener(FpgaFastpathBridge* parent) : parent_(parent) {}
        void on_subscription_matched(eprosima::fastdds::dds::DataReader*,
                                     const eprosima::fastdds::dds::SubscriptionMatchedStatus&) override {
            // Note: Remote endpoint matching is handled via BridgePartListener (filtered by guidPrefix)
        }
    private:
        FpgaFastpathBridge* parent_;
    };

    BridgeConfig config_;
    std::atomic<bool> running_{false};
    std::atomic<bool> writer_matched_{false};
    std::atomic<bool> reader_matched_{false};
    uint32_t generation_{0};

    std::mutex match_mutex_;
    std::mutex hw_cfg_mutex_;
    std::condition_variable cv_match_;

    struct fpga_fastpath* fpga_{nullptr};
    std::thread rx_thread_;
    std::thread wdog_thread_;

    eprosima::fastdds::dds::DomainParticipant* participant_{nullptr};
    eprosima::fastdds::dds::Publisher* publisher_{nullptr};
    eprosima::fastdds::dds::Subscriber* subscriber_{nullptr};
    eprosima::fastdds::dds::Topic* topic_{nullptr};
    eprosima::fastdds::dds::DataWriter* writer_{nullptr};
    eprosima::fastdds::dds::DataReader* reader_{nullptr};
    eprosima::fastdds::dds::TypeSupport type_;

    BridgePartListener part_listener_{this};
    BridgeWriterListener writer_listener_{this};
    BridgeReaderListener reader_listener_{this};
};

} // namespace fastdds_fpga

#endif // _FPGA_FASTPATH_BRIDGE_HPP_
