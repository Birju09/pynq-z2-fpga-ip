#ifndef _RTPS_PCAP_CAPTURE_H_
#define _RTPS_PCAP_CAPTURE_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <arpa/inet.h>

#pragma pack(push, 1)

/* Standard Libpcap Global Header */
struct pcap_file_header {
    uint32_t magic;         /* 0xa1b2c3d4 */
    uint16_t version_major; /* 2 */
    uint16_t version_minor; /* 4 */
    int32_t  thiszone;      /* GMT to local correction */
    uint32_t sigfigs;       /* accuracy of timestamps */
    uint32_t snaplen;       /* max length of captured packets (65535) */
    uint32_t network;       /* data link type: 1 = DLT_EN10MB (Ethernet) */
};

/* Standard Libpcap Record Header */
struct pcap_pkthdr {
    uint32_t ts_sec;   /* timestamp seconds */
    uint32_t ts_usec;  /* timestamp microseconds */
    uint32_t incl_len; /* number of octets of packet saved in file */
    uint32_t orig_len; /* actual length of packet */
};

/* Ethernet II Header (14 bytes) */
struct eth_hdr {
    uint8_t  dst_mac[6];
    uint8_t  src_mac[6];
    uint16_t ether_type; /* 0x0800 for IPv4 */
};

/* IPv4 Header without options (20 bytes) */
struct ip_hdr {
    uint8_t  version_ihl; /* 0x45 */
    uint8_t  tos;         /* 0x00 */
    uint16_t total_len;   /* Network byte order */
    uint16_t id;          /* Identification */
    uint16_t flags_frag;  /* Flags + fragment offset */
    uint8_t  ttl;         /* 64 */
    uint8_t  protocol;    /* 17 for UDP */
    uint16_t checksum;    /* Header checksum */
    uint32_t src_ip;      /* Network byte order */
    uint32_t dst_ip;      /* Network byte order */
};

/* UDP Header (8 bytes) */
struct udp_hdr {
    uint16_t src_port; /* Network byte order */
    uint16_t dst_port; /* Network byte order */
    uint16_t length;   /* UDP len = 8 + payload_bytes */
    uint16_t checksum; /* UDP checksum (0 or 1's comp) */
};

#pragma pack(pop)

/* Calculate IPv4 16-bit 1's complement checksum */
static inline uint16_t pcap_csum16(const void* buf, size_t bytes) {
    const uint16_t* ptr = (const uint16_t*)buf;
    uint32_t sum = 0;
    while (bytes > 1) {
        sum += *ptr++;
        bytes -= 2;
    }
    if (bytes == 1) {
        sum += *(const uint8_t*)ptr;
    }
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return (uint16_t)(~sum);
}

typedef struct rtps_pcap_dumper {
    FILE* fp;
    uint32_t pkt_count;
} rtps_pcap_dumper_t;

static inline int rtps_pcap_open(rtps_pcap_dumper_t* dumper, const char* filename) {
    if (!dumper || !filename) return -1;
    dumper->fp = fopen(filename, "wb");
    if (!dumper->fp) return -1;

    struct pcap_file_header hdr;
    hdr.magic         = 0xa1b2c3d4;
    hdr.version_major = 2;
    hdr.version_minor = 4;
    hdr.thiszone      = 0;
    hdr.sigfigs       = 0;
    hdr.snaplen       = 65535;
    hdr.network       = 1; /* LINKTYPE_ETHERNET */

    if (fwrite(&hdr, sizeof(hdr), 1, dumper->fp) != 1) {
        fclose(dumper->fp);
        dumper->fp = NULL;
        return -1;
    }
    fflush(dumper->fp);
    dumper->pkt_count = 0;
    return 0;
}

static inline void rtps_pcap_close(rtps_pcap_dumper_t* dumper) {
    if (dumper && dumper->fp) {
        fflush(dumper->fp);
        fclose(dumper->fp);
        dumper->fp = NULL;
    }
}

/*
 * Dump complete RTPS over UDP packet:
 * [Ethernet II (14B)][IPv4 (20B)][UDP (8B)][RTPS Payload (length bytes)]
 */
static inline int rtps_pcap_dump_udp(rtps_pcap_dumper_t* dumper,
                                     const struct timespec* ts,
                                     const uint8_t src_mac[6],
                                     const uint8_t dst_mac[6],
                                     uint32_t src_ip,
                                     uint32_t dst_ip,
                                     uint16_t src_port,
                                     uint16_t dst_port,
                                     uint16_t ip_id,
                                     const void* rtps_payload,
                                     uint32_t rtps_len) {
    if (!dumper || !dumper->fp || !rtps_payload || rtps_len == 0)
        return -1;

    uint32_t total_pkt_len = sizeof(struct eth_hdr) + sizeof(struct ip_hdr) +
                             sizeof(struct udp_hdr) + rtps_len;

    struct pcap_pkthdr rec_hdr;
    rec_hdr.ts_sec   = (uint32_t)ts->tv_sec;
    rec_hdr.ts_usec  = (uint32_t)(ts->tv_nsec / 1000);
    rec_hdr.incl_len = total_pkt_len;
    rec_hdr.orig_len = total_pkt_len;

    struct eth_hdr eth;
    memcpy(eth.dst_mac, dst_mac, 6);
    memcpy(eth.src_mac, src_mac, 6);
    eth.ether_type = htons(0x0800);

    struct ip_hdr ip;
    ip.version_ihl = 0x45;
    ip.tos         = 0x00;
    ip.total_len   = htons((uint16_t)(sizeof(struct ip_hdr) + sizeof(struct udp_hdr) + rtps_len));
    ip.id          = htons(ip_id);
    ip.flags_frag  = htons(0x4000); /* Don't Fragment */
    ip.ttl         = 64;
    ip.protocol    = 17;            /* UDP */
    ip.checksum    = 0;
    ip.src_ip      = htonl(src_ip);
    ip.dst_ip      = htonl(dst_ip);
    ip.checksum    = pcap_csum16(&ip, sizeof(ip));

    struct udp_hdr udp;
    udp.src_port = htons(src_port);
    udp.dst_port = htons(dst_port);
    udp.length   = htons((uint16_t)(sizeof(struct udp_hdr) + rtps_len));
    udp.checksum = 0; /* Optional in IPv4 UDP */

    if (fwrite(&rec_hdr, sizeof(rec_hdr), 1, dumper->fp) != 1 ||
        fwrite(&eth, sizeof(eth), 1, dumper->fp) != 1 ||
        fwrite(&ip, sizeof(ip), 1, dumper->fp) != 1 ||
        fwrite(&udp, sizeof(udp), 1, dumper->fp) != 1 ||
        fwrite(rtps_payload, 1, rtps_len, dumper->fp) != rtps_len) {
        return -1;
    }

    dumper->pkt_count++;
    return 0;
}

/*
 * Dump 3-Fragment wire format:
 * Splits a full RTPS UDP datagram into 3 IPv4 fragments identical to the FPGA hardware wire framing:
 * - Fragment 0: MF=1, Offset=0   (1480B payload: 8B UDP + 1472B RTPS)
 * - Fragment 1: MF=1, Offset=185 (1480B payload: RTPS +1472..+2951)
 * - Fragment 2: MF=0, Offset=370 (Remaining payload bytes)
 */
static inline int rtps_pcap_dump_fragments(rtps_pcap_dumper_t* dumper,
                                           const struct timespec* ts,
                                           const uint8_t src_mac[6],
                                           const uint8_t dst_mac[6],
                                           uint32_t src_ip,
                                           uint32_t dst_ip,
                                           uint16_t src_port,
                                           uint16_t dst_port,
                                           uint16_t ip_id,
                                           const void* rtps_payload,
                                           uint32_t rtps_len) {
    if (!dumper || !dumper->fp || !rtps_payload || rtps_len == 0)
        return -1;

    uint32_t udp_total_len = (uint32_t)sizeof(struct udp_hdr) + rtps_len;
    uint8_t* dgram_buf = (uint8_t*)malloc(udp_total_len);
    if (!dgram_buf) return -1;

    struct udp_hdr* udp = (struct udp_hdr*)dgram_buf;
    udp->src_port = htons(src_port);
    udp->dst_port = htons(dst_port);
    udp->length   = htons((uint16_t)udp_total_len);
    udp->checksum = 0; /* Wireshark happily reassembles with 0 */
    memcpy(dgram_buf + sizeof(struct udp_hdr), rtps_payload, rtps_len);

    /* Fragment parameters matching PL fabric */
    uint32_t frag_offsets[3] = {0, 1480, 2960};
    uint32_t frag_lens[3];
    frag_lens[0] = 1480;
    frag_lens[1] = 1480;
    frag_lens[2] = (udp_total_len > 2960) ? (udp_total_len - 2960) : 0;

    struct eth_hdr eth;
    memcpy(eth.dst_mac, dst_mac, 6);
    memcpy(eth.src_mac, src_mac, 6);
    eth.ether_type = htons(0x0800);

    for (int f = 0; f < 3; f++) {
        if (frag_lens[f] == 0) continue;

        uint32_t total_pkt_len = sizeof(struct eth_hdr) + sizeof(struct ip_hdr) + frag_lens[f];
        struct pcap_pkthdr rec_hdr;
        rec_hdr.ts_sec   = (uint32_t)ts->tv_sec;
        rec_hdr.ts_usec  = (uint32_t)(ts->tv_nsec / 1000) + f; /* slight offset in us */
        rec_hdr.incl_len = total_pkt_len;
        rec_hdr.orig_len = total_pkt_len;

        struct ip_hdr ip;
        ip.version_ihl = 0x45;
        ip.tos         = 0x00;
        ip.total_len   = htons((uint16_t)(sizeof(struct ip_hdr) + frag_lens[f]));
        ip.id          = htons(ip_id);
        uint16_t mf_flag = (f < 2) ? 0x2000 : 0x0000;
        uint16_t off_8b  = (uint16_t)(frag_offsets[f] / 8);
        ip.flags_frag  = htons(mf_flag | (off_8b & 0x1FFF));
        ip.ttl         = 64;
        ip.protocol    = 17;
        ip.checksum    = 0;
        ip.src_ip      = htonl(src_ip);
        ip.dst_ip      = htonl(dst_ip);
        ip.checksum    = pcap_csum16(&ip, sizeof(ip));

        if (fwrite(&rec_hdr, sizeof(rec_hdr), 1, dumper->fp) != 1 ||
            fwrite(&eth, sizeof(eth), 1, dumper->fp) != 1 ||
            fwrite(&ip, sizeof(ip), 1, dumper->fp) != 1 ||
            fwrite(dgram_buf + frag_offsets[f], 1, frag_lens[f], dumper->fp) != frag_lens[f]) {
            free(dgram_buf);
            return -1;
        }
        dumper->pkt_count++;
    }

    free(dgram_buf);
    return 0;
}

#endif /* _RTPS_PCAP_CAPTURE_H_ */

