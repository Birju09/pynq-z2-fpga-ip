// ============================================================================
// LargeSample5MB.hpp
//
// POD representation of LargeSample5MB.idl.
// Aligned to 4-byte boundaries with exact 5,242,880-byte (5 MiB) size.
// ============================================================================

#ifndef _LARGESAMPLE5MB_HPP_
#define _LARGESAMPLE5MB_HPP_

#include <array>
#include <cstdint>
#include <cstring>

#pragma pack(push, 4)
class LargeSample5MB {
public:
    uint32_t m_cycle_id{0};
    uint32_t m_flags{0};
    uint64_t m_tx_timestamp_ns{0};
    std::array<uint32_t, 1310716> m_payload{};

    LargeSample5MB() = default;
    ~LargeSample5MB() = default;
    LargeSample5MB(const LargeSample5MB& x) = default;
    LargeSample5MB(LargeSample5MB&& x) noexcept = default;
    LargeSample5MB& operator=(const LargeSample5MB& x) = default;
    LargeSample5MB& operator=(LargeSample5MB&& x) noexcept = default;

    inline void cycle_id(uint32_t _cycle_id) { m_cycle_id = _cycle_id; }
    inline uint32_t cycle_id() const { return m_cycle_id; }
    inline uint32_t& cycle_id() { return m_cycle_id; }

    inline void flags(uint32_t _flags) { m_flags = _flags; }
    inline uint32_t flags() const { return m_flags; }
    inline uint32_t& flags() { return m_flags; }

    inline void tx_timestamp_ns(uint64_t _tx_timestamp_ns) { m_tx_timestamp_ns = _tx_timestamp_ns; }
    inline uint64_t tx_timestamp_ns() const { return m_tx_timestamp_ns; }
    inline uint64_t& tx_timestamp_ns() { return m_tx_timestamp_ns; }

    inline void payload(const std::array<uint32_t, 1310716>& _payload) { m_payload = _payload; }
    inline void payload(std::array<uint32_t, 1310716>&& _payload) { m_payload = std::move(_payload); }
    inline const std::array<uint32_t, 1310716>& payload() const { return m_payload; }
    inline std::array<uint32_t, 1310716>& payload() { return m_payload; }

    inline uint8_t* raw_data() { return reinterpret_cast<uint8_t*>(this); }
    inline const uint8_t* raw_data() const { return reinterpret_cast<const uint8_t*>(this); }
    static constexpr size_t raw_size() { return sizeof(LargeSample5MB); }
};
#pragma pack(pop)

static_assert(sizeof(LargeSample5MB) == 5242880, "LargeSample5MB must be exactly 5,242,880 bytes (5 MiB)");

#endif // _LARGESAMPLE5MB_HPP_
