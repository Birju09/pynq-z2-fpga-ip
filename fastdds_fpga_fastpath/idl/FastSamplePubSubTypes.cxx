// ============================================================================
// FastSamplePubSubTypes.cxx
//
// Fast DDS TopicDataType implementation for FastSample.
// Serializes Little-Endian CDR (CDR_LE, encapsulation 0x0001) matching
// the 4,100-byte wire layout produced by the FPGA Fastpath IP.
// ============================================================================

#include "FastSamplePubSubTypes.hpp"
#include <cstring>

FastSamplePubSubType::FastSamplePubSubType() {
    set_name("FastSample");
    auto type_size = static_cast<uint32_t>(TOTAL_SERIALIZED_SIZE);
    max_serialized_type_size = type_size;
    is_compute_key_provided = false;
}

bool FastSamplePubSubType::serialize(
    const void* const data,
    eprosima::fastdds::rtps::SerializedPayload_t& payload,
    eprosima::fastdds::dds::DataRepresentationId_t data_representation) {
    (void)data_representation;
    if (!data) return false;

    payload.reserve(static_cast<uint32_t>(TOTAL_SERIALIZED_SIZE));
    payload.encapsulation = 1; // CDR_LE
    payload.length = static_cast<uint32_t>(TOTAL_SERIALIZED_SIZE);

    // 4-byte CDR_LE encapsulation header: [0x00, 0x01, 0x00, 0x00]
    payload.data[0] = 0x00;
    payload.data[1] = 0x01;
    payload.data[2] = 0x00;
    payload.data[3] = 0x00;

    // 4,096-byte POD struct payload
    std::memcpy(&payload.data[CDR_HEADER_SIZE], data, PAYLOAD_SIZE);
    return true;
}

bool FastSamplePubSubType::deserialize(
    eprosima::fastdds::rtps::SerializedPayload_t& payload,
    void* data) {
    if (!data || payload.length < TOTAL_SERIALIZED_SIZE) return false;

    // Extract 4,096-byte FastSample payload directly after the 4-byte CDR header
    std::memcpy(data, &payload.data[CDR_HEADER_SIZE], PAYLOAD_SIZE);
    return true;
}

uint32_t FastSamplePubSubType::calculate_serialized_size(
    const void* const data,
    eprosima::fastdds::dds::DataRepresentationId_t data_representation) {
    (void)data;
    (void)data_representation;
    return static_cast<uint32_t>(TOTAL_SERIALIZED_SIZE);
}

void* FastSamplePubSubType::create_data() {
    return reinterpret_cast<void*>(new FastSample());
}

void FastSamplePubSubType::delete_data(void* data) {
    delete reinterpret_cast<FastSample*>(data);
}

bool FastSamplePubSubType::compute_key(
    eprosima::fastdds::rtps::SerializedPayload_t& payload,
    eprosima::fastdds::rtps::InstanceHandle_t& ihandle,
    bool force_md5) {
    (void)payload;
    (void)ihandle;
    (void)force_md5;
    return false;
}

bool FastSamplePubSubType::compute_key(
    const void* const data,
    eprosima::fastdds::rtps::InstanceHandle_t& ihandle,
    bool force_md5) {
    (void)data;
    (void)ihandle;
    (void)force_md5;
    return false;
}
