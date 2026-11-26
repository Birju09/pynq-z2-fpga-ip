// ============================================================================
// FastSamplePubSubTypes.hpp
//
// Fast DDS TopicDataType implementation for FastSample (Fast DDS 3.x).
// ============================================================================

#ifndef _FASTSAMPLE_PUBSUBTYPES_HPP_
#define _FASTSAMPLE_PUBSUBTYPES_HPP_

#include <fastdds/dds/topic/TopicDataType.hpp>
#include "FastSample.hpp"

class FastSamplePubSubType : public eprosima::fastdds::dds::TopicDataType {
public:
    typedef FastSample type;

    FastSamplePubSubType();
    ~FastSamplePubSubType() override = default;

    bool serialize(
        const void* const data,
        eprosima::fastdds::rtps::SerializedPayload_t& payload,
        eprosima::fastdds::dds::DataRepresentationId_t data_representation) override;

    bool deserialize(
        eprosima::fastdds::rtps::SerializedPayload_t& payload,
        void* data) override;

    uint32_t calculate_serialized_size(
        const void* const data,
        eprosima::fastdds::dds::DataRepresentationId_t data_representation) override;

    void* create_data() override;

    void delete_data(void* data) override;

    bool compute_key(
        eprosima::fastdds::rtps::SerializedPayload_t& payload,
        eprosima::fastdds::rtps::InstanceHandle_t& ihandle,
        bool force_md5 = false) override;

    bool compute_key(
        const void* const data,
        eprosima::fastdds::rtps::InstanceHandle_t& ihandle,
        bool force_md5 = false) override;

    static constexpr size_t PAYLOAD_SIZE = 4096;
    static constexpr size_t CDR_HEADER_SIZE = 4;
    static constexpr size_t TOTAL_SERIALIZED_SIZE = CDR_HEADER_SIZE + PAYLOAD_SIZE; // 4,100 B
};

#endif // _FASTSAMPLE_PUBSUBTYPES_HPP_
