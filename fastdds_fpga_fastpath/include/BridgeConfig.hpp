// ============================================================================
// BridgeConfig.hpp
//
// Shared configuration struct for both FPGA Fastpath Bridge and Pure Fast DDS
// benchmark sessions.
// ============================================================================

#ifndef _BRIDGE_CONFIG_HPP_
#define _BRIDGE_CONFIG_HPP_

#include <cstdint>
#include <string>

namespace fastdds_fpga {

struct BridgeConfig {
    std::string device_path{"/dev/axi_dma_dev"};
    uint32_t    domain_id{0};
    std::string topic_name{"FastSampleTopic"};

    // PS Control-Plane Discovery Network (Linux eth0)
    std::string ps_ip{""};                 // Empty = bind all interfaces; or e.g. "192.168.1.10"
    std::string peer_ip{""};               // Optional unicast SPDP peer IP, e.g. "192.168.1.100"

    // PL Data-Plane Hardware Identity (RMII PHY)
    uint32_t    pl_ip{0xC0A801C9};         // 192.168.1.201 (dedicated PL RMII IP)
    uint8_t     pl_mac[6]{0x02, 0x00, 0x5E, 0x10, 0x00, 0x01};
    uint16_t    local_src_port{7411};
    uint16_t    pl_rx_port{7511};          // Reader locator advertised via SEDP
    uint16_t    default_dst_port{7410};
    uint32_t    fallback_dst_ip{0xC0A80164}; // 192.168.1.100

    bool        enable_peer_emu{false};    // false = Physical RMII wire, true = internal emulator
    bool        tx_ip_frag_mode{false};    // false = Option C DATA_FRAG (0x16), true = Legacy IPv4 frag
    uint32_t    max_message_size{1468};    // Fast DDS UDP maxMessageSize (1468 - 68B = 1400B)
    uint32_t    frag_size_bytes{0};        // Optional explicit DATA_FRAG fragmentSize override
    uint8_t     override_peer_mac[6]{0};
    bool        has_peer_mac_override{false};
    uint32_t    pacing_timer_ticks{0};     // 0 = wire line rate
    bool        large_5mb{false};          // true = 5 MiB LargeSample5MB via DDR Ring
    uint32_t    ring_depth{2};             // DDR ring slot count (1..16)
};

} // namespace fastdds_fpga

#endif // _BRIDGE_CONFIG_HPP_
