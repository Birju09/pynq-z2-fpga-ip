// ============================================================================
// LargeSample5MBPubSubTypes.cxx
//
// Fast DDS TopicDataType implementation for LargeSample5MB.
// Serializes Little-Endian CDR (CDR_LE, encapsulation 0x0001) matching
// the 5,242,884-byte wire layout produced by the FPGA Fastpath IP DDR Ring.
// ============================================================================

#include "LargeSample5MBPubSubTypes.hpp"
#include <cstring>

LargeSample5MBPubSubType::LargeSample5MBPubSubType() {
    set_name("LargeSample5MB");
    auto type_size = static_cast<uint32_t>(TOTAL_SERIALIZED_SIZE);
    max_serialized_type_size = type_size;
    is_compute_key_provided = false;
}

bool LargeSample5MBPubSubType::serialize(
    const void* const data,
    eprosima::fastdds::rtps::SerializedPayload_t& payload,
    eprosima::fastdds::dds::DataRepresentationId_t data_representation) {
    (void)data_representation;
    if (!data) return false;

    payload.reserve(static_cast<uint32_t>(TOTAL_SERIALIZED_SIZE));
    payload.encapsulation = 1; // CDR_LE
    payload.length = static_cast<uint32_t>(TOTAL_SERIALIZED_SIZE);

    payload.data[0] = 0x00;
    payload.data[1] = 0x01;
    payload.data[2] = 0x00;
    payload.data[3] = 0x00;

    std::memcpy(&payload.data[CDR_HEADER_SIZE], data, PAYLOAD_SIZE);
    return true;
}

bool LargeSample5MBPubSubType::deserialize(
    eprosima::fastdds::rtps::SerializedPayload_t& payload,
    void* data) {
    if (!data || payload.length < TOTAL_SERIALIZED_SIZE) return false;

    std::memcpy(data, &payload.data[CDR_HEADER_SIZE], PAYLOAD_SIZE);
    return true;
}

uint32_t LargeSample5MBPubSubType::calculate_serialized_size(
    const void* const data,
    eprosima::fastdds::dds::DataRepresentationId_t data_representation) {
    (void)data;
    (void)data_representation;
    return static_cast<uint32_t>(TOTAL_SERIALIZED_SIZE);
}

void* LargeSample5MBPubSubType::create_data() {
    return reinterpret_cast<void*>(new LargeSample5MB());
}

void LargeSample5MBPubSubType::delete_data(void* data) {
    delete reinterpret_cast<LargeSample5MB*>(data);
}

bool LargeSample5MBPubSubType::compute_key(
    eprosima::fastdds::rtps::SerializedPayload_t& payload,
    eprosima::fastdds::rtps::InstanceHandle_t& ihandle,
    bool force_md5) {
    (void)payload;
    (void)ihandle;
    (void)force_md5;
    return false;
}

bool LargeSample5MBPubSubType::compute_key(
    const void* const data,
    eprosima::fastdds::rtps::InstanceHandle_t& ihandle,
    bool force_md5) {
    (void)data;
    (void)ihandle;
    (void)force_md5;
    return false;
}
