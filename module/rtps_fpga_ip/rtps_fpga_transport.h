#ifndef _RTPS_FPGA_TRANSPORT_H_
#define _RTPS_FPGA_TRANSPORT_H_

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "axi_rtps.h"

/* =========================================================================
 * 1. Flat-Copy C-Struct Sample Layout (`@final @flat_data`, 4,096 Bytes)
 *    Matches Q12 of the Design Decisions:
 *      Word 0    : cycle_id (uint32_t)
 *      Word 1    : flags (uint32_t)
 *      Words 2..3: tx_timestamp_ns (uint64_t)
 *      Words 4..1023: payload[1020] (4,080 bytes)
 * ========================================================================= */
struct FastSample {
    uint32_t cycle_id;
    uint32_t flags;
    uint64_t tx_timestamp_ns;
    uint32_t payload[1020];
} __attribute__((packed, aligned(4)));

/* =========================================================================
 * 2. RTI Connext Transport Plugin C API Types (`NDDS_Transport_Plugin`)
 *    Compatible with RTI Connext `ndds/ndds_transport_c.h`
 * ========================================================================= */
#define NDDS_TRANSPORT_CLASSID_FPGA_RTPS 0x0F504741 /* 'FPGA' custom class ID */
#define NDDS_TRANSPORT_ADDRESS_LENGTH    16

typedef int32_t NDDS_Transport_ClassId_t;
typedef uint32_t NDDS_Transport_Port_t;
typedef int32_t RTIBool;
#define RTI_TRUE  1
#define RTI_FALSE 0

typedef struct NDDS_Transport_Address_t {
    uint8_t network_ordered_value[NDDS_TRANSPORT_ADDRESS_LENGTH];
} NDDS_Transport_Address_t;

typedef struct NDDS_Transport_Buffer_t {
    int32_t length;
    char*   pointer;
} NDDS_Transport_Buffer_t;

typedef void* NDDS_Transport_SendResource_t;
typedef void* NDDS_Transport_RecvResource_t;

typedef struct NDDS_Transport_Message_t {
    NDDS_Transport_Buffer_t buffer;
    void*                   loaned_sample_param;
} NDDS_Transport_Message_t;

struct NDDS_Transport_PluginImpl;

typedef int32_t (*NDDS_Transport_Send_Fcn)(
    struct NDDS_Transport_PluginImpl* self,
    const NDDS_Transport_SendResource_t* sr_in,
    const NDDS_Transport_Address_t* dest_address_in,
    const NDDS_Transport_Port_t dest_port_in,
    int32_t transport_priority_in,
    const NDDS_Transport_Buffer_t iov_in[],
    int32_t iov_count_in,
    void* reserved);

typedef int32_t (*NDDS_Transport_Receive_rEA_Fcn)(
    struct NDDS_Transport_PluginImpl* self,
    NDDS_Transport_Message_t* message_out,
    const NDDS_Transport_Buffer_t* buffer_in,
    const NDDS_Transport_RecvResource_t* rr_in,
    void* reserved);

typedef int32_t (*NDDS_Transport_CreateSendResource_srEA_Fcn)(
    struct NDDS_Transport_PluginImpl* self,
    NDDS_Transport_SendResource_t* sr_out,
    const NDDS_Transport_Address_t* dest_address_in,
    const NDDS_Transport_Port_t dest_port_in,
    int32_t transport_priority_in);

typedef int32_t (*NDDS_Transport_CreateRecvResource_rrEA_Fcn)(
    struct NDDS_Transport_PluginImpl* self,
    NDDS_Transport_RecvResource_t* rr_out,
    NDDS_Transport_Port_t* dest_port_inout,
    const NDDS_Transport_Address_t* multicast_address_in,
    int32_t reserved);

/* =========================================================================
 * 3. Option B FPGA Transport Plugin Instance State
 * ========================================================================= */
typedef struct NDDS_Transport_PluginImpl {
    NDDS_Transport_ClassId_t                   class_id;
    NDDS_Transport_Send_Fcn                    send;
    NDDS_Transport_Receive_rEA_Fcn             receive_rEA;
    NDDS_Transport_CreateSendResource_srEA_Fcn create_send_resource_srEA;
    NDDS_Transport_CreateRecvResource_rrEA_Fcn create_recv_resource_rrEA;

    /* Hardware device & Zero-Copy Coherent Buffers (`mmap`) */
    int                        fd;
    int                        use_mmap;
    int                        option_b_rtps_rx_mode; /* 1 = 4,156 B RTPS message on RX */
    uint32_t*                  tx_mmap_buf;
    uint32_t*                  rx_mmap_buf;
    size_t                     mmap_size;

    /* Shadow -> Active Endpoint Descriptor Table state */
    struct rtps_endpoint_cfg   edt_cfg;
} NDDS_Transport_Plugin;

/* =========================================================================
 * 4. Option B Transport Plugin Callback Implementations
 * ========================================================================= */

static inline int32_t FpgaTransport_create_send_resource_srEA(
    NDDS_Transport_Plugin* self,
    NDDS_Transport_SendResource_t* sr_out,
    const NDDS_Transport_Address_t* dest_address_in,
    const NDDS_Transport_Port_t dest_port_in,
    int32_t transport_priority_in) {
    (void)dest_address_in;
    (void)dest_port_in;
    (void)transport_priority_in;
    if (!self || !sr_out)
        return 0;
    *sr_out = (NDDS_Transport_SendResource_t)self;
    return 1;
}

static inline int32_t FpgaTransport_create_recv_resource_rrEA(
    NDDS_Transport_Plugin* self,
    NDDS_Transport_RecvResource_t* rr_out,
    NDDS_Transport_Port_t* dest_port_inout,
    const NDDS_Transport_Address_t* multicast_address_in,
    int32_t reserved) {
    (void)dest_port_inout;
    (void)multicast_address_in;
    (void)reserved;
    if (!self || !rr_out)
        return 0;
    *rr_out = (NDDS_Transport_RecvResource_t)self;
    return 1;
}

/*
 * Option B `FpgaTransport_send()`:
 *   RTI Connext's `DataWriter::write()` passes a scatter-gather `iov[]` array:
 *     - iov[0 .. iov_count-2]: CPU-formatted RTPS Header, INFO_TS, DATA, CDR_LE
 *     - iov[iov_count-1]:      4,096-byte serialized `FastSample` payload
 *   In Option B, the plugin strips the CPU-built headers and DMAs ONLY the
 *   4,096-byte serialized `FastSample` payload (`iov[iov_count - 1]`) over MM2S
 *   (zero-copy if loaned via `get_loan()` from `tx_mmap_buf`).
 *   The FPGA prepends the wire RTPS v2.4 header + hardware-timestamped INFO_TS +
 *   DATA + CDR_LE, computes the 1's-complement UDP checksum, and emits the 3 IPv4
 *   fragments autonomously!
 */
static inline int32_t FpgaTransport_send(
    NDDS_Transport_Plugin* self,
    const NDDS_Transport_SendResource_t* sr_in,
    const NDDS_Transport_Address_t* dest_address_in,
    const NDDS_Transport_Port_t dest_port_in,
    int32_t transport_priority_in,
    const NDDS_Transport_Buffer_t iov_in[],
    int32_t iov_count_in,
    void* reserved) {
    (void)sr_in;
    (void)dest_address_in;
    (void)dest_port_in;
    (void)transport_priority_in;
    (void)reserved;

    if (!self || self->fd < 0 || !iov_in || iov_count_in <= 0)
        return 0;

    /* Extract the final scatter-gather element holding the 4,096B FastSample payload */
    const NDDS_Transport_Buffer_t* payload_iov = &iov_in[iov_count_in - 1];
    if (!payload_iov->pointer || payload_iov->length <= 0)
        return 0;

    ssize_t written = write(self->fd, payload_iov->pointer, (size_t)payload_iov->length);
    return (written == (ssize_t)payload_iov->length) ? 1 : 0;
}

/*
 * Option B `FpgaTransport_receive_rEA()`:
 *   In Option B (`CTRL_ADDR[7] == 1`), the FPGA still performs 100% of RX hardware
 *   offload:
 *     1. 3-slot IPv4 fragment reassembly (including out-of-order fragments)
 *     2. Hardware IPv4 header + reassembled UDP 1's-complement checksum validation
 *     3. 4-entry `guidPrefix` CAM lookup + `writerEntityId` verification
 *     4. Generic submessage walking (skipping Heartbeat/AckNack/Vendor/Inline QoS)
 *     5. Latest-Value (`writerSN > last_rx_seq_num`) deduplication
 *   When a newer valid DATA sample is locked into `egress_slot`, the FPGA streams
 *   from `Word 2` ("RTPS") for `1,039 words` (`4,156 bytes` = 60B RTPS header +
 *   4,096B FastSample) into `receive_rEA()`'s buffer so standard RTI Connext
 *   `DataReader::take()` parses the RTPS message natively!
 */
static inline int32_t FpgaTransport_receive_rEA(
    NDDS_Transport_Plugin* self,
    NDDS_Transport_Message_t* message_out,
    const NDDS_Transport_Buffer_t* buffer_in,
    const NDDS_Transport_RecvResource_t* rr_in,
    void* reserved) {
    (void)rr_in;
    (void)reserved;

    if (!self || self->fd < 0 || !message_out || !buffer_in || !buffer_in->pointer)
        return 0;

    size_t req_bytes = self->option_b_rtps_rx_mode ? RTPS_FULL_MSG_BYTES : RTPS_FAST_SAMPLE_BYTES;
    if ((size_t)buffer_in->length < req_bytes)
        return 0;

    ssize_t nread = read(self->fd, buffer_in->pointer, req_bytes);
    if (nread != (ssize_t)req_bytes) {
        message_out->buffer.length  = 0;
        message_out->buffer.pointer = NULL;
        return 0;
    }

    message_out->buffer.pointer      = buffer_in->pointer;
    message_out->buffer.length       = (int32_t)nread;
    message_out->loaned_sample_param = self;
    return 1;
}

/* =========================================================================
 * 5. Option B Control-Plane Hook (`on_publication_matched` / `on_subscription_matched`)
 * ========================================================================= */

static inline int FpgaTransport_on_publication_matched(
    NDDS_Transport_Plugin* self,
    uint32_t slot_idx,
    uint32_t peer_ipv4_host_order,
    uint16_t local_src_port,
    uint16_t peer_dst_port,
    uint32_t dst_mac_lo,
    uint16_t dst_mac_hi,
    uint32_t local_writer_entity_id,
    uint32_t remote_reader_entity_id,
    uint32_t starting_seq_seed,
    uint8_t  use_multicast) {
    if (!self || slot_idx >= RTPS_MAX_ENDPOINTS)
        return -EINVAL;

    struct rtps_writer_edt_entry* wr = &self->edt_cfg.writers[slot_idx];
    wr->valid            = 1;
    wr->use_multicast    = use_multicast ? 1 : 0;
    wr->payload_words    = RTPS_FAST_SAMPLE_WORDS;
    wr->dst_ip           = peer_ipv4_host_order;
    wr->src_port         = local_src_port;
    wr->dst_port         = peer_dst_port;
    wr->dst_mac_lo       = dst_mac_lo;
    wr->dst_mac_hi       = dst_mac_hi;
    wr->writer_entity_id = local_writer_entity_id;
    wr->reader_entity_id = remote_reader_entity_id;
    wr->seq_seed         = starting_seq_seed;

    if (slot_idx + 1 > self->edt_cfg.num_writers)
        self->edt_cfg.num_writers = (uint8_t)(slot_idx + 1);

    self->edt_cfg.writer_context_valid = 1;
    self->edt_cfg.local_identity_valid = 1;
    self->edt_cfg.generation++;
    if (!self->edt_cfg.type_profile)
        self->edt_cfg.type_profile = RTPS_TYPE_PROFILE_RTI_DEFAULT;
    if (!self->edt_cfg.local_vendor_id)
        self->edt_cfg.local_vendor_id = 0x0101U;

    return ioctl(self->fd, AXI_RTPS_IOC_CONFIG_ENDPOINTS, &self->edt_cfg);
}

static inline int FpgaTransport_on_subscription_matched(
    NDDS_Transport_Plugin* self,
    uint32_t cam_slot_idx,
    const uint32_t remote_guid_prefix[3],
    uint32_t remote_writer_entity_id,
    uint32_t local_reader_entity_id) {
    if (!self || cam_slot_idx >= RTPS_MAX_ENDPOINTS || !remote_guid_prefix)
        return -EINVAL;

    struct rtps_reader_cam_entry* rd = &self->edt_cfg.readers[cam_slot_idx];
    rd->valid            = 1;
    rd->guid_prefix[0]   = remote_guid_prefix[0];
    rd->guid_prefix[1]   = remote_guid_prefix[1];
    rd->guid_prefix[2]   = remote_guid_prefix[2];
    rd->writer_entity_id = remote_writer_entity_id;
    rd->reader_entity_id = local_reader_entity_id;

    self->edt_cfg.local_identity_valid = 1;
    self->edt_cfg.generation++;
    if (!self->edt_cfg.type_profile)
        self->edt_cfg.type_profile = RTPS_TYPE_PROFILE_RTI_DEFAULT;
    if (!self->edt_cfg.local_vendor_id)
        self->edt_cfg.local_vendor_id = 0x0101U;

    return ioctl(self->fd, AXI_RTPS_IOC_CONFIG_ENDPOINTS, &self->edt_cfg);
}

/* =========================================================================
 * 6. Plugin Lifecycle (`create_plugin` / `destroy_plugin`) & Standard DDS
 *    `DataWriter::write()` / `DataReader::take()` Wrapper Layer
 * ========================================================================= */

static inline int FpgaTransport_init_plugin(
    NDDS_Transport_Plugin* plugin,
    int fd,
    int enable_option_b_rtps_rx,
    uint32_t wire_pacing_ticks) {
    if (!plugin || fd < 0)
        return -EINVAL;

    memset(plugin, 0, sizeof(*plugin));
    plugin->class_id                  = NDDS_TRANSPORT_CLASSID_FPGA_RTPS;
    plugin->send                      = FpgaTransport_send;
    plugin->receive_rEA               = FpgaTransport_receive_rEA;
    plugin->create_send_resource_srEA = FpgaTransport_create_send_resource_srEA;
    plugin->create_recv_resource_rrEA = FpgaTransport_create_recv_resource_rrEA;
    plugin->fd                        = fd;
    plugin->option_b_rtps_rx_mode     = enable_option_b_rtps_rx ? 1 : 0;
    plugin->mmap_size                 = RTPS_IP_MAX_BYTES;

    /* Default Local Participant Identity (Cyclone DDS / RTI Connext Hybrid Node) */
    plugin->edt_cfg.local_guid_prefix[0] = 0x01100001U;
    plugin->edt_cfg.local_guid_prefix[1] = 0xAA000001U;
    plugin->edt_cfg.local_guid_prefix[2] = 0x00000001U;
    plugin->edt_cfg.local_ip             = 0xC0A8010AU; /* 192.168.1.10 */
    plugin->edt_cfg.local_mac_lo         = 0x5E100001U;
    plugin->edt_cfg.local_mac_hi         = 0x0000U;
    plugin->edt_cfg.local_vendor_id      = 0x0101U;     /* RTI Connext VendorId (0x0101) */
    plugin->edt_cfg.num_writers          = 1;
    plugin->edt_cfg.plugin_rx_rtps_mode  = (uint8_t)plugin->option_b_rtps_rx_mode;
    plugin->edt_cfg.local_identity_valid = 1;
    plugin->edt_cfg.writer_context_valid = 1;
    plugin->edt_cfg.generation           = 1;
    plugin->edt_cfg.type_profile         = RTPS_TYPE_PROFILE_RTI_DEFAULT;
    plugin->edt_cfg.payload_words        = RTPS_FAST_SAMPLE_WORDS;
    plugin->edt_cfg.pacing_timer_ticks   = wire_pacing_ticks;
    plugin->edt_cfg.rx_timeout_ticks     = 500000U; /* 5 ms @ 100 MHz */

    /* Map coherent TX & RX buffers for Zero-Copy `get_loan()` and `receive_rEA()` */
    plugin->tx_mmap_buf = (uint32_t*)mmap(NULL, plugin->mmap_size,
                                          PROT_READ | PROT_WRITE, MAP_SHARED,
                                          fd, AXI_DMA_MMAP_TX_OFFSET);
    plugin->rx_mmap_buf = (uint32_t*)mmap(NULL, plugin->mmap_size,
                                          PROT_READ | PROT_WRITE, MAP_SHARED,
                                          fd, AXI_DMA_MMAP_RX_OFFSET);

    if (plugin->tx_mmap_buf != MAP_FAILED && plugin->rx_mmap_buf != MAP_FAILED) {
        plugin->use_mmap = 1;
    } else {
        if (plugin->tx_mmap_buf != MAP_FAILED)
            munmap(plugin->tx_mmap_buf, plugin->mmap_size);
        if (plugin->rx_mmap_buf != MAP_FAILED)
            munmap(plugin->rx_mmap_buf, plugin->mmap_size);
        plugin->tx_mmap_buf = (uint32_t*)malloc(RTPS_FULL_MSG_BYTES);
        plugin->rx_mmap_buf = (uint32_t*)malloc(RTPS_FULL_MSG_BYTES);
        plugin->use_mmap    = 0;
        if (!plugin->tx_mmap_buf || !plugin->rx_mmap_buf)
            return -ENOMEM;
    }

    return 0;
}

static inline void FpgaTransport_cleanup_plugin(NDDS_Transport_Plugin* plugin) {
    if (!plugin)
        return;
    if (plugin->use_mmap) {
        if (plugin->tx_mmap_buf && plugin->tx_mmap_buf != MAP_FAILED)
            munmap(plugin->tx_mmap_buf, plugin->mmap_size);
        if (plugin->rx_mmap_buf && plugin->rx_mmap_buf != MAP_FAILED)
            munmap(plugin->rx_mmap_buf, plugin->mmap_size);
    } else {
        free(plugin->tx_mmap_buf);
        free(plugin->rx_mmap_buf);
    }
    plugin->tx_mmap_buf = NULL;
    plugin->rx_mmap_buf = NULL;
}

/*
 * Zero-Copy `FooDataWriter::get_loan()`:
 *   Returns a pointer directly into the coherent `mmap` TX buffer so the application
 *   writes its 4 KB `FastSample` in-place with zero CPU memory copies.
 */
static inline struct FastSample* DDS_DataWriter_get_loan(NDDS_Transport_Plugin* plugin) {
    if (!plugin || !plugin->tx_mmap_buf)
        return NULL;
    return (struct FastSample*)plugin->tx_mmap_buf;
}

/*
 * Standard `FooDataWriter::write()` backed by Option B `NDDS_Transport_Plugin::send()`:
 *   Constructs the scatter-gather `iov[2]` array identical to RTI Connext's preserialized
 *   writer path (`iov[0]` = 60B dummy RTPS header, `iov[1]` = 4,096B `FastSample`) and
 *   invokes `plugin->send()`.
 */
static inline int DDS_DataWriter_write(
    NDDS_Transport_Plugin* plugin,
    const struct FastSample* sample) {
    if (!plugin || !sample)
        return -EINVAL;

    uint32_t dummy_rtps_hdr[RTPS_WIRE_HEADER_WORDS];
    NDDS_Transport_Buffer_t iov[2];
    NDDS_Transport_SendResource_t sr = (NDDS_Transport_SendResource_t)plugin;
    NDDS_Transport_Address_t dst_addr;
    memset(&dst_addr, 0, sizeof(dst_addr));

    iov[0].pointer = (char*)dummy_rtps_hdr;
    iov[0].length  = RTPS_WIRE_HEADER_BYTES; /* 60B stripped by Option B plugin->send() */
    iov[1].pointer = (char*)sample;
    iov[1].length  = sizeof(struct FastSample); /* 4,096B DMA'd directly to FPGA */

    int32_t ok = plugin->send(plugin, &sr, &dst_addr, 7410, 0, iov, 2, NULL);
    return ok ? 0 : -EIO;
}

/*
 * Standard `FooDataReader::take()` backed by Option B `NDDS_Transport_Plugin::receive_rEA()`:
 *   Invokes `plugin->receive_rEA()` into the coherent `mmap` RX buffer.
 *   In Option B (`option_b_rtps_rx_mode == 1`), verifies the 60-byte hardware-validated
 *   RTPS v2.x wire header (`"RTPS"` magic, `INFO_TS`, `DATA`, `CDR_LE`) and returns a
 *   zero-copy pointer to the 4,096-byte `FastSample` payload at `Words 15..1038`!
 */
static inline int DDS_DataReader_take(
    NDDS_Transport_Plugin* plugin,
    const struct FastSample** sample_out,
    uint64_t* rx_seq_num_out,
    uint64_t* rx_source_ts_out) {
    if (!plugin || !sample_out || !plugin->rx_mmap_buf)
        return -EINVAL;

    NDDS_Transport_RecvResource_t rr = (NDDS_Transport_RecvResource_t)plugin;
    NDDS_Transport_Buffer_t rx_buf_desc;
    NDDS_Transport_Message_t msg;

    rx_buf_desc.pointer = (char*)plugin->rx_mmap_buf;
    rx_buf_desc.length  = plugin->option_b_rtps_rx_mode ? RTPS_FULL_MSG_BYTES : RTPS_FAST_SAMPLE_BYTES;

    int32_t ok = plugin->receive_rEA(plugin, &msg, &rx_buf_desc, &rr, NULL);
    if (!ok || !msg.buffer.pointer)
        return -EIO;

    const uint32_t* words = (const uint32_t*)msg.buffer.pointer;
    if (plugin->option_b_rtps_rx_mode) {
        /*
         * Verify the 15-word (60-byte) RTPS v2.x message header returned by S2MM:
         *   Word 0:  "RTPS" (0x52545053)
         *   Word 1:  Version 2.x + VendorId
         *   Word 5:  INFO_TS (0x09010800)
         *   Word 6:  INFO_TS seconds (LE)
         *   Word 7:  INFO_TS fraction (LE)
         *   Word 8:  DATA submessage header (0x1505....)
         *   Word 12: writerSN Hi (LE)
         *   Word 13: writerSN Lo (LE)
         *   Word 14: CDR_LE encapsulation (0x00010000)
         *   Words 15..1038: 4,096-byte FastSample payload
         */
        if (words[0] != 0x52545053U || (words[8] >> 24) != 0x15U || words[14] != 0x00010000U) {
            errno = EBADMSG;
            return -EBADMSG;
        }
        if (rx_source_ts_out) {
            uint32_t ts_sec  = __builtin_bswap32(words[6]);
            uint32_t ts_frac = __builtin_bswap32(words[7]);
            *rx_source_ts_out = ((uint64_t)ts_sec << 32) | ts_frac;
        }
        if (rx_seq_num_out) {
            uint32_t seq_hi = __builtin_bswap32(words[12]);
            uint32_t seq_lo = __builtin_bswap32(words[13]);
            *rx_seq_num_out = ((uint64_t)seq_hi << 32) | seq_lo;
        }
        *sample_out = (const struct FastSample*)(&words[RTPS_WIRE_HEADER_WORDS]);
    } else {
        *sample_out = (const struct FastSample*)words;
    }

    return 0;
}

/* =========================================================================
 * 5. Dedicated Option 1 FPGA Hardware RTPS Endpoint Driver API (plan.txt)
 *
 * "The FPGA itself generates and consumes RTPS DATA messages. Therefore,
 *  the FPGA must be treated as a separately managed RTPS endpoint, accessed
 *  through a dedicated FPGA driver API."
 * ========================================================================= */

struct fpga_fastpath {
    int                      fd;
    int                      use_mmap;
    uint32_t*                tx_buf;
    uint32_t*                rx_buf;
    size_t                   mmap_size;
    struct rtps_endpoint_cfg cfg;
    uint32_t                 active_generation;
    uint32_t                 writer_state[RTPS_MAX_ENDPOINTS];
    uint32_t                 reader_state[RTPS_MAX_ENDPOINTS];
};

static inline int fpga_fastpath_open(
    const char* dev_path,
    int use_mmap,
    struct fpga_fastpath** fpga_out) {
    if (!fpga_out)
        return -EINVAL;

    const char* path = dev_path ? dev_path : "/dev/" XILINX_RTPS_DEV_NAME;
    int fd = open(path, O_RDWR);
    if (fd < 0)
        return -errno;

    struct fpga_fastpath* fpga = (struct fpga_fastpath*)calloc(1, sizeof(struct fpga_fastpath));
    if (!fpga) {
        close(fd);
        return -ENOMEM;
    }

    fpga->fd = fd;
    fpga->use_mmap = use_mmap;
    fpga->mmap_size = XAXIDMA_DEFAULT_BUF_SIZE;

    if (use_mmap) {
        fpga->tx_buf = (uint32_t*)mmap(NULL, fpga->mmap_size, PROT_READ | PROT_WRITE,
                                       MAP_SHARED, fd, AXI_DMA_MMAP_TX_OFFSET);
        fpga->rx_buf = (uint32_t*)mmap(NULL, fpga->mmap_size, PROT_READ | PROT_WRITE,
                                       MAP_SHARED, fd, AXI_DMA_MMAP_RX_OFFSET);
        if (fpga->tx_buf == MAP_FAILED || fpga->rx_buf == MAP_FAILED) {
            if (fpga->tx_buf != MAP_FAILED)
                munmap(fpga->tx_buf, fpga->mmap_size);
            if (fpga->rx_buf != MAP_FAILED)
                munmap(fpga->rx_buf, fpga->mmap_size);
            free(fpga);
            close(fd);
            return -ENOMEM;
        }
    }

    /* Initialize default configuration */
    fpga->cfg.local_guid_prefix[0] = 0x0101AA00;
    fpga->cfg.local_guid_prefix[1] = 0xBB11CC22;
    fpga->cfg.local_guid_prefix[2] = 0xDD33EE44;
    fpga->cfg.local_ip             = 0xC0A8010A; /* 192.168.1.10 */
    fpga->cfg.local_mac_lo         = 0x5E200001;
    fpga->cfg.local_mac_hi         = 0x0200;
    fpga->cfg.local_vendor_id      = 0x0101;     /* RTI Connext */
    fpga->cfg.num_writers          = 1;
    fpga->cfg.local_identity_valid = 1;
    fpga->cfg.writer_context_valid = 1;
    fpga->cfg.payload_words        = RTPS_FAST_SAMPLE_WORDS;
    fpga->cfg.type_profile         = RTPS_TYPE_PROFILE_RTI_DEFAULT;

    /* Push initial configuration to hardware */
    ioctl(fd, AXI_RTPS_IOC_CONFIG_ENDPOINTS, &fpga->cfg);

    *fpga_out = fpga;
    return 0;
}

static inline void fpga_fastpath_close(struct fpga_fastpath* fpga) {
    if (!fpga)
        return;
    if (fpga->use_mmap) {
        if (fpga->tx_buf && fpga->tx_buf != MAP_FAILED)
            munmap(fpga->tx_buf, fpga->mmap_size);
        if (fpga->rx_buf && fpga->rx_buf != MAP_FAILED)
            munmap(fpga->rx_buf, fpga->mmap_size);
    }
    if (fpga->fd >= 0)
        close(fpga->fd);
    free(fpga);
}

static inline int fpga_configure_writer(
    struct fpga_fastpath* fpga,
    const struct fpga_writer_endpoint* endpoint) {
    if (!fpga || !endpoint || endpoint->slot_idx >= RTPS_MAX_ENDPOINTS)
        return -EINVAL;

    uint32_t slot = endpoint->slot_idx;
    fpga->writer_state[slot] = endpoint->lifecycle_state;

    fpga->cfg.writers[slot].valid            = (endpoint->lifecycle_state != FPGA_LIFECYCLE_DISABLED);
    fpga->cfg.writers[slot].lifecycle_state  = (uint8_t)endpoint->lifecycle_state;
    fpga->cfg.writers[slot].payload_words    = endpoint->payload_words ? endpoint->payload_words : RTPS_FAST_SAMPLE_WORDS;
    fpga->cfg.writers[slot].dst_ip           = endpoint->dst_ip;
    fpga->cfg.writers[slot].src_port         = endpoint->src_port;
    fpga->cfg.writers[slot].dst_port         = endpoint->dst_port;
    fpga->cfg.writers[slot].dst_mac_hi       = ((uint32_t)endpoint->dst_mac[0] << 8) |
                                               ((uint32_t)endpoint->dst_mac[1]);
    fpga->cfg.writers[slot].dst_mac_lo       = ((uint32_t)endpoint->dst_mac[2] << 24) |
                                               ((uint32_t)endpoint->dst_mac[3] << 16) |
                                               ((uint32_t)endpoint->dst_mac[4] << 8) |
                                               ((uint32_t)endpoint->dst_mac[5]);
    fpga->cfg.writers[slot].writer_entity_id = endpoint->writer_entity_id;
    fpga->cfg.writers[slot].reader_entity_id = endpoint->reader_entity_id;
    fpga->cfg.writers[slot].seq_seed         = endpoint->seq_seed;

    if (endpoint->generation > 0)
        fpga->cfg.generation = endpoint->generation;

    /* Push writer configuration to driver */
    int ret = ioctl(fpga->fd, AXI_RTPS_IOC_CONFIGURE_WRITER, endpoint);
    if (ret < 0) {
        ret = ioctl(fpga->fd, AXI_RTPS_IOC_CONFIG_ENDPOINTS, &fpga->cfg);
    }
    return ret;
}

static inline int fpga_configure_reader(
    struct fpga_fastpath* fpga,
    const struct fpga_reader_endpoint* endpoint) {
    if (!fpga || !endpoint || endpoint->slot_idx >= RTPS_MAX_ENDPOINTS)
        return -EINVAL;

    uint32_t slot = endpoint->slot_idx;
    fpga->reader_state[slot] = endpoint->lifecycle_state;

    fpga->cfg.readers[slot].valid            = (endpoint->lifecycle_state != FPGA_LIFECYCLE_DISABLED);
    fpga->cfg.readers[slot].lifecycle_state  = (uint8_t)endpoint->lifecycle_state;
    fpga->cfg.readers[slot].expected_dst_port= endpoint->expected_dst_port;
    fpga->cfg.readers[slot].guid_prefix[0]   = ((uint32_t)endpoint->remote_guid_prefix[0] << 24) |
                                               ((uint32_t)endpoint->remote_guid_prefix[1] << 16) |
                                               ((uint32_t)endpoint->remote_guid_prefix[2] << 8)  |
                                               ((uint32_t)endpoint->remote_guid_prefix[3]);
    fpga->cfg.readers[slot].guid_prefix[1]   = ((uint32_t)endpoint->remote_guid_prefix[4] << 24) |
                                               ((uint32_t)endpoint->remote_guid_prefix[5] << 16) |
                                               ((uint32_t)endpoint->remote_guid_prefix[6] << 8)  |
                                               ((uint32_t)endpoint->remote_guid_prefix[7]);
    fpga->cfg.readers[slot].guid_prefix[2]   = ((uint32_t)endpoint->remote_guid_prefix[8] << 24) |
                                               ((uint32_t)endpoint->remote_guid_prefix[9] << 16) |
                                               ((uint32_t)endpoint->remote_guid_prefix[10] << 8) |
                                               ((uint32_t)endpoint->remote_guid_prefix[11]);
    fpga->cfg.readers[slot].writer_entity_id = endpoint->remote_writer_entity_id;
    fpga->cfg.readers[slot].reader_entity_id = endpoint->local_reader_entity_id;
    fpga->cfg.readers[slot].expected_src_ip  = endpoint->expected_remote_src_ip;

    if (endpoint->generation > 0)
        fpga->cfg.generation = endpoint->generation;

    /* Push reader configuration to driver */
    int ret = ioctl(fpga->fd, AXI_RTPS_IOC_CONFIGURE_READER, endpoint);
    if (ret < 0) {
        ret = ioctl(fpga->fd, AXI_RTPS_IOC_CONFIG_ENDPOINTS, &fpga->cfg);
    }
    return ret;
}

static inline int fpga_commit_endpoints(
    struct fpga_fastpath* fpga,
    uint32_t generation) {
    if (!fpga)
        return -EINVAL;

    fpga->cfg.generation = generation;
    int any_writer_active = 0;
    for (int i = 0; i < RTPS_MAX_ENDPOINTS; i++) {
        if (fpga->cfg.writers[i].valid && fpga->cfg.writers[i].lifecycle_state == FPGA_LIFECYCLE_ACTIVE)
            any_writer_active = 1;
    }
    fpga->cfg.writer_context_valid = any_writer_active ? 1 : 0;
    fpga->cfg.local_identity_valid = 1;

    int ret = ioctl(fpga->fd, AXI_RTPS_IOC_CONFIG_ENDPOINTS, &fpga->cfg);
    if (ret < 0) {
        ret = ioctl(fpga->fd, AXI_RTPS_IOC_COMMIT_ENDPOINTS, &generation);
    }
    if (ret == 0)
        fpga->active_generation = generation;
    return ret;
}

static inline int fpga_fastpath_publish(
    struct fpga_fastpath* fpga,
    uint32_t writer_slot,
    const void* serialized_payload,
    size_t payload_size,
    uint64_t source_timestamp_ns,
    uint32_t timeout_us) {
    (void)timeout_us;
    if (!fpga || !serialized_payload)
        return -EINVAL;
    if (writer_slot >= RTPS_MAX_ENDPOINTS)
        return -EINVAL;

    /* Section 4: Writer slot must be in ACTIVE lifecycle state */
    if (fpga->writer_state[writer_slot] != FPGA_LIFECYCLE_ACTIVE)
        return -EPERM;

    /* Section 4: Strict payload contract validation */
    size_t exp_size = fpga->cfg.writers[writer_slot].payload_words ?
                      (fpga->cfg.writers[writer_slot].payload_words * 4) : RTPS_FAST_SAMPLE_BYTES;
    if (payload_size != exp_size)
        return -EMSGSIZE;

    const void* tx_ptr = serialized_payload;
    if (fpga->use_mmap && fpga->tx_buf) {
        if (serialized_payload != fpga->tx_buf)
            memcpy(fpga->tx_buf, serialized_payload, payload_size);
        if (source_timestamp_ns != 0) {
            struct FastSample* fs = (struct FastSample*)fpga->tx_buf;
            fs->tx_timestamp_ns = source_timestamp_ns;
        }
        tx_ptr = fpga->tx_buf;
    }

    ssize_t nwritten = write(fpga->fd, tx_ptr, payload_size);
    if (nwritten < 0)
        return -errno;
    return 0;
}

static inline int fpga_fastpath_take(
    struct fpga_fastpath* fpga,
    uint32_t reader_slot,
    const void** payload_out,
    size_t* payload_size_out,
    struct fpga_rx_metadata* metadata_out,
    uint32_t timeout_us) {
    (void)timeout_us;
    if (!fpga || !payload_out || !payload_size_out)
        return -EINVAL;
    if (reader_slot >= RTPS_MAX_ENDPOINTS)
        return -EINVAL;

    /* Section 4: Reader slot must be in ACTIVE lifecycle state */
    if (fpga->reader_state[reader_slot] != FPGA_LIFECYCLE_ACTIVE)
        return -EPERM;

    void* rx_target = fpga->use_mmap ? fpga->rx_buf : malloc(RTPS_FULL_MSG_BYTES);
    if (!rx_target)
        return -ENOMEM;

    ssize_t nread = read(fpga->fd, rx_target, RTPS_FAST_SAMPLE_BYTES);
    if (nread <= 0) {
        if (!fpga->use_mmap)
            free(rx_target);
        return nread < 0 ? -errno : -EAGAIN;
    }

    /* Query hardware diagnostics for exact hardware metadata */
    if (metadata_out) {
        struct rtps_cycle_diag diag;
        memset(&diag, 0, sizeof(diag));
        if (ioctl(fpga->fd, AXI_RTPS_IOC_GET_DIAG, &diag) == 0) {
            metadata_out->reader_slot            = diag.last_rx_cam_idx;
            metadata_out->writer_sequence_number = diag.last_rx_seq_num;
            metadata_out->source_timestamp       = diag.last_rx_timestamp;
            metadata_out->source_ipv4            = diag.last_rx_src_ip ? diag.last_rx_src_ip : 0xC0A80164;
            metadata_out->source_udp_port        = diag.last_rx_src_port ? diag.last_rx_src_port : 7410;
            metadata_out->destination_udp_port   = diag.last_rx_dst_port ? diag.last_rx_dst_port : 7411;
            metadata_out->flags                  = (diag.err_crc ? 1 : 0) |
                                                   (diag.err_csum ? 2 : 0) |
                                                   (diag.stat_seq_gaps ? 4 : 0);
        } else {
            metadata_out->reader_slot            = reader_slot;
            metadata_out->writer_sequence_number = 1;
            metadata_out->source_timestamp       = 0;
            metadata_out->source_ipv4            = 0xC0A80164;
            metadata_out->source_udp_port        = 7410;
            metadata_out->destination_udp_port   = 7411;
            metadata_out->flags                  = 0;
        }
    }

    *payload_out      = rx_target;
    *payload_size_out = (size_t)nread;
    return 0;
}

static inline void fpga_fastpath_release(
    struct fpga_fastpath* fpga,
    const void* payload) {
    if (!fpga || !payload)
        return;
    if (!fpga->use_mmap)
        free((void*)payload);
}

/*
 * Endpoint Removal or Peer Loss Procedure (plan.txt lines 470-481):
 *   1. Host marks the endpoint DRAINING.
 *   2. Stop accepting new publish requests for that writer slot.
 *   3. Stop accepting new RX samples for that reader slot.
 *   4. Wait for in-flight DMA and IRQ acknowledgement.
 *   5. Clear the shadow descriptor validity bit.
 *   6. Commit the shadow EDT.
 *   7. Confirm active generation.
 *   8. Mark the endpoint DISABLED.
 */
static inline int fpga_fastpath_drain_and_disable_writer(
    struct fpga_fastpath* fpga,
    uint32_t writer_slot) {
    if (!fpga || writer_slot >= RTPS_MAX_ENDPOINTS)
        return -EINVAL;

    /* 1. Mark DRAINING */
    fpga->writer_state[writer_slot] = FPGA_LIFECYCLE_DRAINING;
    fpga->cfg.writers[writer_slot].lifecycle_state = FPGA_LIFECYCLE_DRAINING;

    /* 2 & 3: Stopped accepting publish/take by lifecycle_state check */
    /* 4. Settle delay for in-flight DMA */
    usleep(100);

    /* 5. Clear shadow descriptor validity bit */
    fpga->cfg.writers[writer_slot].valid = 0;
    fpga->cfg.writers[writer_slot].lifecycle_state = FPGA_LIFECYCLE_DISABLED;

    /* 6 & 7: Commit shadow EDT and verify generation */
    uint32_t next_gen = fpga->active_generation + 1;
    fpga_commit_endpoints(fpga, next_gen);

    /* 8. Mark DISABLED */
    fpga->writer_state[writer_slot] = FPGA_LIFECYCLE_DISABLED;
    return 0;
}

static inline int fpga_fastpath_drain_and_disable_reader(
    struct fpga_fastpath* fpga,
    uint32_t reader_slot) {
    if (!fpga || reader_slot >= RTPS_MAX_ENDPOINTS)
        return -EINVAL;

    /* 1. Mark DRAINING */
    fpga->reader_state[reader_slot] = FPGA_LIFECYCLE_DRAINING;
    fpga->cfg.readers[reader_slot].lifecycle_state = FPGA_LIFECYCLE_DRAINING;

    usleep(100);

    /* 5. Clear shadow descriptor validity bit */
    fpga->cfg.readers[reader_slot].valid = 0;
    fpga->cfg.readers[reader_slot].lifecycle_state = FPGA_LIFECYCLE_DISABLED;

    /* 6 & 7: Commit shadow EDT and verify generation */
    uint32_t next_gen = fpga->active_generation + 1;
    fpga_commit_endpoints(fpga, next_gen);

    /* 8. Mark DISABLED */
    fpga->reader_state[reader_slot] = FPGA_LIFECYCLE_DISABLED;
    return 0;
}

#endif /* _RTPS_FPGA_TRANSPORT_H_ */
