// ============================================================================
// FastSample.hpp
//
// Generated / Handcrafted POD representation of FastSample.idl.
// Aligned to 4-byte boundaries with exact 4,096-byte hardware size.
// ============================================================================

#ifndef _FASTSAMPLE_HPP_
#define _FASTSAMPLE_HPP_

#include <cstdint>
#include <array>
#include <cstring>

#if defined(_WIN32)
#if defined(FASTSAMPLE_SOURCE)
#define FASTSAMPLE_DllAPI __declspec(dllexport)
#else
#define FASTSAMPLE_DllAPI __declspec(dllimport)
#endif
#else
#define FASTSAMPLE_DllAPI
#endif

#pragma pack(push, 4)
class FASTSAMPLE_DllAPI FastSample {
public:
    uint32_t m_cycle_id{0};
    uint32_t m_flags{0};
    uint64_t m_tx_timestamp_ns{0};
    std::array<uint32_t, 1020> m_payload{};

    FastSample() = default;
    ~FastSample() = default;
    FastSample(const FastSample& x) = default;
    FastSample(FastSample&& x) noexcept = default;
    FastSample& operator=(const FastSample& x) = default;
    FastSample& operator=(FastSample&& x) noexcept = default;

    inline void cycle_id(uint32_t _cycle_id) { m_cycle_id = _cycle_id; }
    inline uint32_t cycle_id() const { return m_cycle_id; }
    inline uint32_t& cycle_id() { return m_cycle_id; }

    inline void flags(uint32_t _flags) { m_flags = _flags; }
    inline uint32_t flags() const { return m_flags; }
    inline uint32_t& flags() { return m_flags; }

    inline void tx_timestamp_ns(uint64_t _tx_timestamp_ns) { m_tx_timestamp_ns = _tx_timestamp_ns; }
    inline uint64_t tx_timestamp_ns() const { return m_tx_timestamp_ns; }
    inline uint64_t& tx_timestamp_ns() { return m_tx_timestamp_ns; }

    inline void payload(const std::array<uint32_t, 1020>& _payload) { m_payload = _payload; }
    inline void payload(std::array<uint32_t, 1020>&& _payload) { m_payload = std::move(_payload); }
    inline const std::array<uint32_t, 1020>& payload() const { return m_payload; }
    inline std::array<uint32_t, 1020>& payload() { return m_payload; }

    // Direct C-pointer access for DMA zero-copy
    inline uint8_t* raw_data() { return reinterpret_cast<uint8_t*>(this); }
    inline const uint8_t* raw_data() const { return reinterpret_cast<const uint8_t*>(this); }
    static constexpr size_t raw_size() { return sizeof(FastSample); }
};
#pragma pack(pop)

static_assert(sizeof(FastSample) == 4096, "FastSample must be exactly 4,096 bytes (1,024 words)");

#endif // _FASTSAMPLE_HPP_
