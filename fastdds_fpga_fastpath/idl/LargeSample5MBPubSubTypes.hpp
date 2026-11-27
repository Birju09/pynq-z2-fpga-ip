// ============================================================================
// LargeSample5MBPubSubTypes.hpp
//
// Fast DDS TopicDataType implementation for LargeSample5MB (Fast DDS 3.x).
// ============================================================================

#ifndef _LARGESAMPLE5MB_PUBSUBTYPES_HPP_
#define _LARGESAMPLE5MB_PUBSUBTYPES_HPP_

#include <fastdds/dds/topic/TopicDataType.hpp>
#include "LargeSample5MB.hpp"

class LargeSample5MBPubSubType : public eprosima::fastdds::dds::TopicDataType {
public:
    typedef LargeSample5MB type;

    LargeSample5MBPubSubType();
    ~LargeSample5MBPubSubType() override = default;

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

    static constexpr size_t PAYLOAD_SIZE = 5242880;
    static constexpr size_t CDR_HEADER_SIZE = 4;
    static constexpr size_t TOTAL_SERIALIZED_SIZE = CDR_HEADER_SIZE + PAYLOAD_SIZE; // 5,242,884 B
};

#endif // _LARGESAMPLE5MB_PUBSUBTYPES_HPP_
