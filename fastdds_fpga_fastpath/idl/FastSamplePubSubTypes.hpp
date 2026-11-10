// ============================================================================
// FastSamplePubSubTypes.hpp
//
// Fast DDS TopicDataType implementation for FastSample.
// Compatible with Fast DDS 2.x and Fast DDS 3.x.
// ============================================================================

#ifndef _FASTSAMPLE_PUBSUBTYPES_HPP_
#define _FASTSAMPLE_PUBSUBTYPES_HPP_

#include <fastdds/dds/topic/TopicDataType.hpp>
#include "FastSample.hpp"

#if __has_include(<fastdds/config.hpp>)
#include <fastdds/config.hpp>
#endif

#if defined(FASTDDS_VERSION_MAJOR) && (FASTDDS_VERSION_MAJOR >= 3)
#define FASTDDS_V3 1
#else
#define FASTDDS_V3 0
#endif

class FastSamplePubSubType : public eprosima::fastdds::dds::TopicDataType {
public:
    typedef FastSample type;

    FastSamplePubSubType();
    ~FastSamplePubSubType() override = default;

#if FASTDDS_V3
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
#else
    bool serialize(
        void* data,
        eprosima::fastrtps::rtps::SerializedPayload_t* payload) override;

    bool deserialize(
        eprosima::fastrtps::rtps::SerializedPayload_t* payload,
        void* data) override;

    std::function<uint32_t()> getSerializedSizeProvider(void* data) override;

    void* createData() override;

    void deleteData(void* data) override;

    bool getKey(
        void* data,
        eprosima::fastrtps::rtps::InstanceHandle_t* ihandle,
        bool force_md5 = false) override;
#endif

    static constexpr size_t PAYLOAD_SIZE = 4096;
    static constexpr size_t CDR_HEADER_SIZE = 4;
    static constexpr size_t TOTAL_SERIALIZED_SIZE = CDR_HEADER_SIZE + PAYLOAD_SIZE; // 4,100 B
};

#endif // _FASTSAMPLE_PUBSUBTYPES_HPP_
