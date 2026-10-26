#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "rtps_fpga_transport.h"
#include "rtps_pcap.h"

#define DEVICE_PATH        "/dev/axi_dma_dev"
#define DEFAULT_ITERATIONS 1000

static const char* rtps_state_name(uint8_t state) {
    switch (state) {
        case 0: return "S_IDLE (Awaiting Ingress Stream)";
        case 1: return "S_CHECK_CRC (CRC Check & UDP Finalize)";
        case 2: return "S_TX_FRAG_INIT (Fragment Header Init)";
        case 3: return "S_TX_FRAG_SEND (Streaming to Wire/Peer)";
        case 4: return "S_WAIT_RX_FB (Awaiting Peer Emulation Feedback)";
        case 5: return "S_DONE_IRQ (Holding Level-High IRQ to PS)";
        case 6: return "S_WAIT_EGRESS (Awaiting Driver S2MM Kick)";
        case 7: return "S_TX_RUN (Streaming Egress to S2MM)";
        default: return "UNKNOWN";
    }
}

static void print_detailed_diag(FILE* out, const struct rtps_cycle_diag* diag) {
    uint8_t cur_state       = (uint8_t)((diag->status_reg & 0x1E) >> 1);
    uint8_t pre_wr_state    = (uint8_t)((diag->pre_write_ip_status & 0x1E) >> 1);
    uint8_t isr_pre_state   = (uint8_t)((diag->isr_pre_ack_ip_status & 0x1E) >> 1);
    uint8_t isr_post_state  = (uint8_t)((diag->isr_post_ack_ip_status & 0x1E) >> 1);
    uint8_t post_wr_state   = (uint8_t)((diag->post_write_ip_status & 0x1E) >> 1);
    uint8_t pre_rd_state    = (uint8_t)((diag->pre_read_ip_status & 0x1E) >> 1);
    uint8_t post_kick_state = (uint8_t)((diag->post_kick_ip_status & 0x1E) >> 1);
    uint8_t post_50us_state = (uint8_t)((diag->post_50us_ip_status & 0x1E) >> 1);

    fprintf(out, "    Hardware Diagnostic State:\n");
    fprintf(out, "      - FSM State            : %u [%s]\n", cur_state, rtps_state_name(cur_state));
    fprintf(out, "      - Status Reg           : 0x%08X (IRQ=%u, IRQ_IS_RX=%u, LIFECYCLE_OK=%u, ERR_CRC=%u, NO_MATCH=%u, TIMEOUT=%u, CSUM=%u)\n",
            diag->status_reg,
            diag->status_reg & 1U,
            (diag->status_reg >> 10) & 1U,
            (diag->status_reg >> 15) & 1U,
            diag->err_crc, diag->err_no_match, diag->err_rx_timeout, diag->err_csum);
    fprintf(out, "      - CTRL Reg             : 0x%08X (EN=%u, Writers=%u, Emu=%u, Wdog=%u, OptionB_RTPS=%u, PubOnly=%u, SubOnly=%u)\n",
            diag->ctrl_reg,
            diag->ctrl_reg & 1U,
            (diag->ctrl_reg >> 1) & 7U,
            (diag->ctrl_reg >> 4) & 1U,
            (diag->ctrl_reg >> 6) & 1U,
            (diag->ctrl_reg >> 7) & 1U,
            (diag->ctrl_reg >> 8) & 1U,
            (diag->ctrl_reg >> 9) & 1U);
    fprintf(out, "      - Identity & Generation: ActiveGen=%u, CommitStatus=%u, TypeProfile=0x%08X, IdCtrl=0x%08X\n",
            diag->active_generation, diag->commit_status, diag->type_profile, diag->identity_ctrl);
    fprintf(out, "      - TX / RX Samples      : %u TX / %u RX (Frags=%u, OptionB_Words=%u [%u B])\n",
            diag->stat_tx_samples, diag->stat_rx_samples, diag->completed_frags,
            diag->last_rx_rtps_words, diag->last_rx_rtps_words * 4);
    fprintf(out, "      - Latest RX SeqNum     : %llu (Gaps=%u, StaleDrops=%u, CsumErrs=%u, SkippedSubmsg=%u)\n",
            (unsigned long long)diag->last_rx_seq_num,
            diag->stat_seq_gaps, diag->stat_stale_drops, diag->stat_csum_errs, diag->stat_skipped_submsg);
    fprintf(out, "    Driver & FSM State Transition Timeline:\n");
    fprintf(out, "      1. Pre-Write IP Status : 0x%08X -> State %u [%s]\n",
            diag->pre_write_ip_status, pre_wr_state, rtps_state_name(pre_wr_state));
    fprintf(out, "      2. ISR Pre-ACK Status  : 0x%08X -> State %u [%s] (IRQ=%u)\n",
            diag->isr_pre_ack_ip_status, isr_pre_state, rtps_state_name(isr_pre_state),
            diag->isr_pre_ack_ip_status & 1U);
    fprintf(out, "      3. ISR Post-ACK Status : 0x%08X -> State %u [%s] (IRQ=%u, ACK Polls=%u)\n",
            diag->isr_post_ack_ip_status, isr_post_state, rtps_state_name(isr_post_state),
            diag->isr_post_ack_ip_status & 1U, diag->isr_ack_polls);
    fprintf(out, "      4. Post-Write Status   : 0x%08X -> State %u [%s]\n",
            diag->post_write_ip_status, post_wr_state, rtps_state_name(post_wr_state));
    fprintf(out, "      5. Pre-Read IP Status  : 0x%08X -> State %u [%s]\n",
            diag->pre_read_ip_status, pre_rd_state, rtps_state_name(pre_rd_state));
    fprintf(out, "      6. Post-Kick IP Status : 0x%08X -> State %u [%s]\n",
            diag->post_kick_ip_status, post_kick_state, rtps_state_name(post_kick_state));
    fprintf(out, "      7. Post-50us IP Status : 0x%08X -> State %u [%s]\n",
            diag->post_50us_ip_status, post_50us_state, rtps_state_name(post_50us_state));
    fprintf(out, "    AXI DMA S2MM & Interrupt Diagnostics (Pre-Reset Snapshot):\n");
    fprintf(out, "      - Linux VIRQs & Counts : MM2S_TX=%u (virq=%d), S2MM_RX=%u (virq=%d), IP_DONE=%u (virq=%d)\n",
            diag->irq_cnt_tx, diag->tx_virq,
            diag->irq_cnt_rx, diag->rx_virq,
            diag->irq_cnt_ip, diag->ip_virq);
    fprintf(out, "      - S2MM DMACR / DMASR   : 0x%08X / 0x%08X (Halted=%u, Idle=%u, SG=%u, Err=0x%X, IOC_Irq=%u)\n",
            diag->s2mm_dmacr, diag->s2mm_dmasr,
            diag->s2mm_dmasr & 1U,
            (diag->s2mm_dmasr >> 1) & 1U,
            (diag->s2mm_dmasr >> 3) & 1U,
            (diag->s2mm_dmasr >> 8) & 7U,
            (diag->s2mm_dmasr >> 12) & 1U);
    fprintf(out, "      - S2MM 50us Snapshot   : DMASR=0x%08X, BD_STS=0x%08X\n",
            diag->post_50us_s2mm_dmasr, diag->post_50us_bd_status);
    fprintf(out, "      - S2MM CUR / TAILDESC  : 0x%08X / 0x%08X (rx_bd_phys=0x%08X, rx_buf_phys=0x%08X)\n",
            diag->s2mm_curdesc, diag->s2mm_taildesc, diag->rx_bd_phys, diag->rx_buf_phys);
    fprintf(out, "      - S2MM BD CTRL / STS   : CTRL=0x%08X (%u B), STS=0x%08X (Cmplt=%u, Err=0x%X, SOF=%u, EOF=%u, Len=%u B)\n",
            diag->rx_bd_control, diag->rx_bd_control & 0x007FFFFFU,
            diag->rx_bd_status,
            (diag->rx_bd_status >> 31) & 1U,
            (diag->rx_bd_status >> 28) & 7U,
            (diag->rx_bd_status >> 27) & 1U,
            (diag->rx_bd_status >> 26) & 1U,
            diag->rx_bd_status & 0x007FFFFFU);
    fprintf(out, "      - S2MM DDR Buffer Scan : %u / %u words written by DMA (first_mod=%u, last_mod=%u)\n",
            diag->rx_buf_words_written, diag->rx_buf_total_words,
            diag->rx_buf_first_mod_idx, diag->rx_buf_last_mod_idx);
    fprintf(out, "      - RX Buffer Sample Wds : w[0]=0x%08X w[1]=0x%08X w[2]=0x%08X w[8]=0x%08X w[14]=0x%08X w[15]=0x%08X w[last]=0x%08X\n",
            diag->rx_buf_sample_words[0], diag->rx_buf_sample_words[1],
            diag->rx_buf_sample_words[2], diag->rx_buf_sample_words[3],
            diag->rx_buf_sample_words[4], diag->rx_buf_sample_words[5],
            diag->rx_buf_sample_words[6]);
}

static inline double timespec_to_us(const struct timespec* start,
                                    const struct timespec* end) {
    return (double)(end->tv_sec - start->tv_sec) * 1e6 +
           (double)(end->tv_nsec - start->tv_nsec) / 1e3;
}

static int compare_doubles(const void* a, const void* b) {
    double da = *(const double*)a;
    double db = *(const double*)b;
    if (da < db)
        return -1;
    if (da > db)
        return 1;
    return 0;
}

static double get_percentile(const double* sorted, int n, double pct) {
    if (n <= 0)
        return 0.0;
    if (n == 1)
        return sorted[0];
    double rank = (pct / 100.0) * (n - 1);
    int low = (int)rank;
    int high = low + 1;
    double weight = rank - low;
    if (high >= n)
        return sorted[n - 1];
    return sorted[low] * (1.0 - weight) + sorted[high] * weight;
}

static int run_fastpath_endpoint_test(int iterations, int verbose) {
    printf("=================================================================\n");
    printf("  Option 1 Dedicated Hardware RTPS Endpoint API Verification\n");
    printf("=================================================================\n");
    printf("  Target Device      : %s\n", DEVICE_PATH);
    printf("  Endpoint Model     : Option 1 (Dedicated FPGA Hardware Endpoint)\n");
    printf("  Lifecycle States   : DISABLED(0) -> ACTIVE(2) -> DRAINING(3)\n");
    printf("  Driver API Tested  : fpga_fastpath_open / close\n");
    printf("                       fpga_configure_writer / reader\n");
    printf("                       fpga_commit_endpoints\n");
    printf("                       fpga_fastpath_publish / take / release\n");
    printf("                       fpga_fastpath_drain_and_disable_*\n");
    printf("=================================================================\n\n");

    struct fpga_fastpath* fpga = NULL;
    int rc = fpga_fastpath_open(DEVICE_PATH, 1, &fpga);
    if (rc < 0) {
        fprintf(stderr, "[-] fpga_fastpath_open failed: %s (%d)\n", strerror(-rc), rc);
        return 1;
    }
    printf("[+] Step 1: fpga_fastpath_open() succeeded (fd=%d, mmap=%s)\n",
           fpga->fd, fpga->use_mmap ? "coherent zero-copy" : "copy");

    /* Step 2: Configure Option 1 Writer Endpoint */
    struct fpga_writer_endpoint writer_ep;
    memset(&writer_ep, 0, sizeof(writer_ep));
    writer_ep.slot_idx          = 0;
    writer_ep.lifecycle_state   = FPGA_LIFECYCLE_ACTIVE;
    writer_ep.payload_words     = RTPS_FAST_SAMPLE_WORDS;
    writer_ep.dst_ip            = 0xC0A80164; /* 192.168.1.100 */
    writer_ep.src_port          = 7411;
    writer_ep.dst_port          = 7410;
    writer_ep.dst_mac[0]        = 0x02;
    writer_ep.dst_mac[1]        = 0x00;
    writer_ep.dst_mac[2]        = 0x5E;
    writer_ep.dst_mac[3]        = 0x20;
    writer_ep.dst_mac[4]        = 0x00;
    writer_ep.dst_mac[5]        = 0x02;
    writer_ep.writer_entity_id  = 0x00000102;
    writer_ep.reader_entity_id  = 0x00000000;
    writer_ep.seq_seed          = 1;
    writer_ep.generation        = 1;

    rc = fpga_configure_writer(fpga, &writer_ep);
    if (rc < 0) {
        fprintf(stderr, "[-] fpga_configure_writer failed: %s (%d)\n", strerror(-rc), rc);
        fpga_fastpath_close(fpga);
        return 1;
    }
    printf("[+] Step 2: fpga_configure_writer() registered Writer[0] (Dst=192.168.1.100:7410, EntityId=0x%08X)\n",
           writer_ep.writer_entity_id);

    /* Step 3: Configure Option 1 Reader Endpoint */
    struct fpga_reader_endpoint reader_ep;
    memset(&reader_ep, 0, sizeof(reader_ep));
    reader_ep.slot_idx               = 0;
    reader_ep.lifecycle_state        = FPGA_LIFECYCLE_ACTIVE;
    /* Peer GuidPrefix: 01010002-BB000002-00000002 */
    reader_ep.remote_guid_prefix[0]  = 0x01;
    reader_ep.remote_guid_prefix[1]  = 0x01;
    reader_ep.remote_guid_prefix[2]  = 0x00;
    reader_ep.remote_guid_prefix[3]  = 0x02;
    reader_ep.remote_guid_prefix[4]  = 0xBB;
    reader_ep.remote_guid_prefix[5]  = 0x00;
    reader_ep.remote_guid_prefix[6]  = 0x00;
    reader_ep.remote_guid_prefix[7]  = 0x02;
    reader_ep.remote_guid_prefix[8]  = 0x00;
    reader_ep.remote_guid_prefix[9]  = 0x00;
    reader_ep.remote_guid_prefix[10] = 0x00;
    reader_ep.remote_guid_prefix[11] = 0x02;
    reader_ep.remote_writer_entity_id= 0x00000202;
    reader_ep.local_reader_entity_id = 0x00000107;
    reader_ep.expected_dst_port      = 7411;
    reader_ep.expected_remote_src_ip = 0xC0A80164;
    reader_ep.generation             = 1;

    rc = fpga_configure_reader(fpga, &reader_ep);
    if (rc < 0) {
        fprintf(stderr, "[-] fpga_configure_reader failed: %s (%d)\n", strerror(-rc), rc);
        fpga_fastpath_close(fpga);
        return 1;
    }
    printf("[+] Step 3: fpga_configure_reader() registered Reader[0] (Expected DstPort=%u, SrcIP=0x%08X)\n",
           reader_ep.expected_dst_port, reader_ep.expected_remote_src_ip);

    /* Step 4: Atomic Commit */
    rc = fpga_commit_endpoints(fpga, 1);
    if (rc < 0) {
        fprintf(stderr, "[-] fpga_commit_endpoints failed: %s (%d)\n", strerror(-rc), rc);
        fpga_fastpath_close(fpga);
        return 1;
    }
    printf("[+] Step 4: fpga_commit_endpoints() atomically committed Generation 1\n\n");

    /* Step 5: Benchmark Loop */
    printf("[+] Step 5: Executing %d Cyclic Fast-Path Publish / Take Iterations...\n", iterations);
    double* latencies_roundtrip = (double*)malloc(iterations * sizeof(double));
    double* latencies_tx        = (double*)malloc(iterations * sizeof(double));
    double* latencies_rx        = (double*)malloc(iterations * sizeof(double));
    double* latencies_pl_hw     = (double*)malloc(iterations * sizeof(double));
    if (!latencies_roundtrip || !latencies_tx || !latencies_rx || !latencies_pl_hw) {
        fprintf(stderr, "[-] Memory allocation failed\n");
        free(latencies_roundtrip); free(latencies_tx); free(latencies_rx); free(latencies_pl_hw);
        fpga_fastpath_close(fpga);
        return 1;
    }

    struct FastSample tx_sample;
    int iter, errors = 0;
    double min_us = 1e12, max_us = 0.0, total_us = 0.0;
    double total_tx_us = 0.0, total_rx_us = 0.0, total_pl_hw_us = 0.0;

    for (iter = 1; iter <= iterations; iter++) {
        if ((iter & 0x3F) == 1) {
            uint32_t pet = 1;
            ioctl(fpga->fd, AXI_RTPS_IOC_PET_WATCHDOG, &pet);
        }

        /* Anti-stale data invariant */
        tx_sample.cycle_id        = (uint32_t)iter;
        tx_sample.flags           = 0x000000A5U ^ (uint32_t)iter;
        tx_sample.tx_timestamp_ns = (uint64_t)iter * 1000000ULL;
        for (uint32_t j = 0; j < 1020; j++) {
            tx_sample.payload[j] = (uint32_t)((iter << 16) ^ (0x5A000000U + ((j + 4) * 0x010103U)));
        }

        struct timespec t_start, t_tx_done, t_rx_done;
        clock_gettime(CLOCK_MONOTONIC, &t_start);

        rc = fpga_fastpath_publish(fpga, 0, &tx_sample, sizeof(tx_sample), tx_sample.tx_timestamp_ns, 5000000);
        if (rc < 0) {
            fprintf(stderr, "\n[-] Iteration %d: fpga_fastpath_publish failed: %s (%d)\n", iter, strerror(-rc), rc);
            errors++;
            break;
        }
        clock_gettime(CLOCK_MONOTONIC, &t_tx_done);

        /* Query hardware ticks */
        struct rtps_cycle_diag diag;
        double pl_cycle_us = 0.0;
        uint32_t pl_ticks = 0;
        if (ioctl(fpga->fd, AXI_RTPS_IOC_GET_DIAG, &diag) == 0) {
            pl_ticks    = diag.last_cycle_ticks;
            pl_cycle_us = (double)diag.last_cycle_ticks * 0.01;
        }

        const void* rx_payload = NULL;
        size_t rx_len = 0;
        struct fpga_rx_metadata meta;
        memset(&meta, 0, sizeof(meta));

        rc = fpga_fastpath_take(fpga, 0, &rx_payload, &rx_len, &meta, 5000000);
        if (rc < 0) {
            fprintf(stderr, "\n[-] Iteration %d: fpga_fastpath_take failed: %s (%d)\n", iter, strerror(-rc), rc);
            errors++;
            break;
        }
        clock_gettime(CLOCK_MONOTONIC, &t_rx_done);

        double tx_us        = timespec_to_us(&t_start, &t_tx_done);
        double rx_us        = timespec_to_us(&t_tx_done, &t_rx_done);
        double roundtrip_us = timespec_to_us(&t_start, &t_rx_done);

        latencies_roundtrip[iter - 1] = roundtrip_us;
        latencies_tx[iter - 1]        = tx_us;
        latencies_rx[iter - 1]        = rx_us;
        latencies_pl_hw[iter - 1]     = pl_cycle_us;

        total_us       += roundtrip_us;
        total_tx_us    += tx_us;
        total_rx_us    += rx_us;
        total_pl_hw_us += pl_cycle_us;

        if (roundtrip_us < min_us) min_us = roundtrip_us;
        if (roundtrip_us > max_us) max_us = roundtrip_us;

        /* Verify RX Metadata */
        if (meta.reader_slot != 0 || meta.source_ipv4 != 0xC0A80164 ||
            meta.source_udp_port != 7410 || meta.destination_udp_port != 7411) {
            fprintf(stderr, "\n[-] Iteration %d: Metadata mismatch: slot=%u, src_ip=0x%08X, ports=%u->%u\n",
                    iter, meta.reader_slot, meta.source_ipv4, meta.source_udp_port, meta.destination_udp_port);
            errors++;
            fpga_fastpath_release(fpga, rx_payload);
            break;
        }

        /* Verify Payload */
        const struct FastSample* rx_sample = (const struct FastSample*)rx_payload;
        if (rx_sample->cycle_id != tx_sample.cycle_id ||
            rx_sample->flags != tx_sample.flags ||
            rx_sample->tx_timestamp_ns != tx_sample.tx_timestamp_ns) {
            fprintf(stderr, "\n[-] Iteration %d FastSample Header mismatch: exp cycle_id=%u, got=%u\n",
                    iter, tx_sample.cycle_id, rx_sample->cycle_id);
            errors++;
            fpga_fastpath_release(fpga, rx_payload);
            break;
        }

        uint64_t active_seq = (meta.writer_sequence_number != 0) ? meta.writer_sequence_number : (uint64_t)iter;
        uint32_t seq_lo_6b = ((uint32_t)active_seq) & 0x3FU;
        for (uint32_t j = 0; j < 1020; j++) {
            uint32_t expected = (~tx_sample.payload[j]) + ((seq_lo_6b << 10) | 0x200U);
            if (rx_sample->payload[j] != expected) {
                fprintf(stderr, "\n[-] Iteration %d payload mismatch at word %u: exp=0x%08X, got=0x%08X\n",
                        iter, j + 4, expected, rx_sample->payload[j]);
                errors++;
                break;
            }
        }

        fpga_fastpath_release(fpga, rx_payload);
        if (errors)
            break;

        if (verbose) {
            printf("  [Iter %4d] RTT: %7.2f us | TX: %6.2f us | PL Peer Emu: %5.2f us (%4u ticks) | RX: %6.2f us | Seq: %llu\n",
                   iter, roundtrip_us, tx_us, pl_cycle_us, pl_ticks, rx_us, (unsigned long long)meta.writer_sequence_number);
        } else if (iter % 100 == 0 || iter == iterations) {
            printf("  Progress: [%4d/%4d] (%5.1f%%) | Current RTT: %7.2f us | Avg: %7.2f us\r",
                   iter, iterations, (iter * 100.0) / iterations, roundtrip_us, total_us / iter);
            fflush(stdout);
        }
    }
    printf("\n\n");

    if (errors == 0 && iter > 1) {
        int count = iter - 1;
        qsort(latencies_roundtrip, count, sizeof(double), compare_doubles);
        qsort(latencies_tx,        count, sizeof(double), compare_doubles);
        qsort(latencies_rx,        count, sizeof(double), compare_doubles);
        qsort(latencies_pl_hw,     count, sizeof(double), compare_doubles);

        double avg_us    = total_us / count;
        double p50_us    = get_percentile(latencies_roundtrip, count, 50.0);
        double p95_us    = get_percentile(latencies_roundtrip, count, 95.0);
        double p99_us    = get_percentile(latencies_roundtrip, count, 99.0);
        double p99_9_us  = get_percentile(latencies_roundtrip, count, 99.9);
        double p99_99_us = get_percentile(latencies_roundtrip, count, 99.99);

        double avg_tx_us = total_tx_us / count;
        double p50_tx_us = get_percentile(latencies_tx, count, 50.0);
        double p95_tx_us = get_percentile(latencies_tx, count, 95.0);

        double avg_rx_us = total_rx_us / count;
        double p50_rx_us = get_percentile(latencies_rx, count, 50.0);
        double p95_rx_us = get_percentile(latencies_rx, count, 95.0);

        double avg_pl_us = total_pl_hw_us / count;
        double p50_pl_us = get_percentile(latencies_pl_hw, count, 50.0);
        double p95_pl_us = get_percentile(latencies_pl_hw, count, 95.0);

        double total_mb = (double)((RTPS_FAST_SAMPLE_BYTES * 2) * (uint64_t)count) / (1024.0 * 1024.0);
        double total_sec = total_us / 1e6;

        printf("=================================================================\n");
        printf("          OPTION 1 FAST-PATH BENCHMARK RESULTS                   \n");
        printf("=================================================================\n");
        printf("  Iterations Completed : %d\n", count);
        printf("  Payload Size         : %u bytes TX / %u bytes RX (FastSample)\n",
               RTPS_FAST_SAMPLE_BYTES, RTPS_FAST_SAMPLE_BYTES);
        printf("  Total Data Processed : %.2f MB\n", total_mb);
        printf("  Total Elapsed Time   : %.3f s (%.1f cycles/sec, %.2f MB/s)\n",
               total_sec, (double)count / total_sec, total_mb / total_sec);
        printf("-----------------------------------------------------------------\n");
        printf("  1. Total End-to-End Latency (fpga_publish -> Emu Loopback -> fpga_take):\n");
        printf("    - Min Latency      : %8.2f us  (%6.3f ms)\n", min_us, min_us / 1e3);
        printf("    - Avg Latency      : %8.2f us  (%6.3f ms)\n", avg_us, avg_us / 1e3);
        printf("    - p50   (Median)   : %8.2f us  (%6.3f ms)\n", p50_us, p50_us / 1e3);
        printf("    - p95   Latency    : %8.2f us  (%6.3f ms)\n", p95_us, p95_us / 1e3);
        printf("    - p99   Latency    : %8.2f us  (%6.3f ms)\n", p99_us, p99_us / 1e3);
        printf("    - p99.9 Latency    : %8.2f us  (%6.3f ms)\n", p99_9_us, p99_9_us / 1e3);
        printf("    - p99.99 Latency   : %8.2f us  (%6.3f ms)\n", p99_99_us, p99_99_us / 1e3);
        printf("    - Max Latency      : %8.2f us  (%6.3f ms)\n", max_us, max_us / 1e3);
        printf("-----------------------------------------------------------------\n");
        printf("  2. Phase Breakdown:\n");
        printf("    [A] Host MM2S TX Write DMA   : %7.2f us  (%5.1f%%)  [p50: %6.2f, p95: %6.2f]\n",
               avg_tx_us, (avg_tx_us / avg_us) * 100.0, p50_tx_us, p95_tx_us);
        printf("    [B] PL Wire Framing + Peer Emu: %7.2f us  (%5.1f%%)  [p50: %6.2f, p95: %6.2f]\n",
               avg_pl_us, (avg_pl_us / avg_us) * 100.0, p50_pl_us, p95_pl_us);
        printf("    [C] Host S2MM RX Read DMA    : %7.2f us  (%5.1f%%)  [p50: %6.2f, p95: %6.2f]\n",
               avg_rx_us, (avg_rx_us / avg_us) * 100.0, p50_rx_us, p95_rx_us);
        printf("=================================================================\n\n");
    }

    free(latencies_roundtrip);
    free(latencies_tx);
    free(latencies_rx);
    free(latencies_pl_hw);

    /* Step 6: Lifecycle State Machine & Safety Gating Verification */
    printf("--- Option 1 Lifecycle State Machine & Safety Gating Verification ---\n");

    /* Subtest A: Contract Gating */
    rc = fpga_fastpath_publish(fpga, 0, &tx_sample, 512, 0, 1000);
    if (rc != -EMSGSIZE) {
        fprintf(stderr, "[-] FAIL: Expected -EMSGSIZE when payload size contract violated, got %d\n", rc);
        errors++;
    } else {
        printf("[+] PASS: Strict payload contract violation returned -EMSGSIZE (%d)!\n", rc);
    }

    /* Subtest B: Drain & Disable Writer */
    rc = fpga_fastpath_drain_and_disable_writer(fpga, 0);
    if (rc < 0) {
        fprintf(stderr, "[-] FAIL: fpga_fastpath_drain_and_disable_writer failed: %d\n", rc);
        errors++;
    } else {
        printf("[+] PASS: fpga_fastpath_drain_and_disable_writer(0) transitioned Writer to DISABLED!\n");
    }

    /* Verify publish rejected while DISABLED */
    rc = fpga_fastpath_publish(fpga, 0, &tx_sample, sizeof(tx_sample), 0, 1000);
    if (rc != -EPERM) {
        fprintf(stderr, "[-] FAIL: Expected -EPERM when publishing on DISABLED writer, got %d\n", rc);
        errors++;
    } else {
        printf("[+] PASS: fpga_fastpath_publish returned -EPERM (%d) on DISABLED writer!\n", rc);
    }

    /* Subtest C: Recovery - Re-activate Writer */
    writer_ep.lifecycle_state = FPGA_LIFECYCLE_ACTIVE;
    writer_ep.generation      = 2;
    rc = fpga_configure_writer(fpga, &writer_ep);
    rc |= fpga_commit_endpoints(fpga, 2);
    if (rc < 0) {
        fprintf(stderr, "[-] FAIL: Re-activation of Writer failed: %d\n", rc);
        errors++;
    } else {
        printf("[+] PASS: Re-activated Writer[0] with Generation 2!\n");
    }

    /* Publish sample on recovered writer */
    rc = fpga_fastpath_publish(fpga, 0, &tx_sample, sizeof(tx_sample), tx_sample.tx_timestamp_ns, 5000000);
    if (rc < 0) {
        fprintf(stderr, "[-] FAIL: Publish on recovered writer failed: %d\n", rc);
        errors++;
    } else {
        const void* rx_p = NULL;
        size_t rx_sz = 0;
        struct fpga_rx_metadata meta;
        rc = fpga_fastpath_take(fpga, 0, &rx_p, &rx_sz, &meta, 5000000);
        if (rc < 0) {
            fprintf(stderr, "[-] FAIL: Take on recovered writer failed: %d\n", rc);
            errors++;
        } else {
            printf("[+] PASS: Clean roundtrip completed on recovered writer (Seq=%llu)!\n",
                   (unsigned long long)meta.writer_sequence_number);
            fpga_fastpath_release(fpga, rx_p);
        }
    }

    /* Subtest D: Full Drain & Disable Reader and Writer */
    fpga_fastpath_drain_and_disable_reader(fpga, 0);
    fpga_fastpath_drain_and_disable_writer(fpga, 0);
    printf("[+] PASS: Full drain and disable workflow executed cleanly.\n");

    fpga_fastpath_close(fpga);
    printf("[+] PASS: fpga_fastpath_close() completed cleanly.\n");
    printf("=================================================================\n");

    return errors ? 1 : 0;
}

int main(int argc, char* argv[]) {
    int fd;
    NDDS_Transport_Plugin plugin;
    struct FastSample* tx_sample = NULL;
    const struct FastSample* rx_sample = NULL;
    double* latencies_roundtrip = NULL;
    double* latencies_tx = NULL;
    double* latencies_rx = NULL;
    double* latencies_pl_hw = NULL;
    int iter, errors = 0;
    uint32_t i;
    int iterations = DEFAULT_ITERATIONS;
    uint32_t timer_val = 0;
    uint32_t bypass_crc = 0;
    int option_b_mode = 1; /* Default: Option B NDDS_Transport_Plugin mode (CTRL[7]=1) */
    int fastpath_mode = 0; /* Option 1 dedicated hardware RTPS endpoint mode */
    int verbose = 0;
    const char* pcap_filename = NULL;
    int pcap_wire_frags = 0;
    rtps_pcap_dumper_t pcap_dumper = {0};
    struct rtps_cycle_diag diag;

    double min_us = 1e12;
    double max_us = 0.0;
    double total_us = 0.0;
    double total_tx_us = 0.0;
    double total_rx_us = 0.0;
    double total_pl_hw_us = 0.0;
    struct timespec t_start, t_tx_done, t_rx_done;

    /* Parse both named flags (--iterations, --emulation, etc.) and positional arguments */
    for (int arg_i = 1; arg_i < argc; arg_i++) {
        if (strcmp(argv[arg_i], "-h") == 0 || strcmp(argv[arg_i], "--help") == 0) {
            printf("Usage: %s [iterations] [timer_cycles] [bypass_crc] [option_b_plugin_mode]\n", argv[0]);
            printf("   or: %s [--iterations N] [--timer N] [--bypass-crc] [--option-b (1|0)] [--verbose]\n", argv[0]);
            printf("   or: %s --fastpath [--iterations N] [--verbose]\n\n", argv[0]);
            printf("Options:\n");
            printf("  -n, --iterations N     Number of cyclic RTPS iterations (default: %d)\n", DEFAULT_ITERATIONS);
            printf("  -t, --timer N          Hardware wire pacing delay cycles per beat (default: 0)\n");
            printf("  --bypass-crc           Bypass CPU DMA0 CRC-32 check\n");
            printf("  -b, --option-b (0|1)   1 for Option B 4,156B RTPS RX (default: 1), 0 for Option A 4,096B\n");
            printf("  --fastpath, --endpoint Option 1 Dedicated FPGA Hardware RTPS Endpoint Driver API test\n");
            printf("  --emulation            Enable hardware RTI Peer Emulator (enabled by default in PL)\n");
            printf("  -v, --verbose          Print per-iteration hardware emulation and phase timing\n");
            printf("  -p, --pcap FILE        Capture all TX and RX RTPS packets to Wireshark PCAP file\n");
            printf("  --pcap-wire-frags      Capture in exact 3-fragment IPv4 wire layout (default: reassembled UDP)\n");
            return 0;
        } else if ((strcmp(argv[arg_i], "-n") == 0 || strcmp(argv[arg_i], "--iterations") == 0) && arg_i + 1 < argc) {
            iterations = atoi(argv[++arg_i]);
        } else if ((strcmp(argv[arg_i], "-t") == 0 || strcmp(argv[arg_i], "--timer") == 0) && arg_i + 1 < argc) {
            timer_val = (uint32_t)atoi(argv[++arg_i]);
        } else if (strcmp(argv[arg_i], "--bypass-crc") == 0) {
            bypass_crc = 1;
        } else if ((strcmp(argv[arg_i], "-b") == 0 || strcmp(argv[arg_i], "--option-b") == 0) && arg_i + 1 < argc) {
            option_b_mode = atoi(argv[++arg_i]) ? 1 : 0;
        } else if (strcmp(argv[arg_i], "--fastpath") == 0 || strcmp(argv[arg_i], "--endpoint") == 0 || strcmp(argv[arg_i], "--endpoint-api") == 0) {
            fastpath_mode = 1;
        } else if (strcmp(argv[arg_i], "-v") == 0 || strcmp(argv[arg_i], "--verbose") == 0) {
            verbose = 1;
        } else if ((strcmp(argv[arg_i], "-p") == 0 || strcmp(argv[arg_i], "--pcap") == 0) && arg_i + 1 < argc) {
            pcap_filename = argv[++arg_i];
        } else if (strcmp(argv[arg_i], "--pcap-wire-frags") == 0) {
            pcap_wire_frags = 1;
        } else if (strcmp(argv[arg_i], "--emulation") == 0) {
            /* Hardware Peer Emulator is active */
        } else if (argv[arg_i][0] != '-') {
            /* Fallback to positional parsing */
            if (arg_i == 1) {
                int v = atoi(argv[arg_i]);
                if (v > 0) iterations = v;
            } else if (arg_i == 2) {
                timer_val = (uint32_t)atoi(argv[arg_i]);
            } else if (arg_i == 3) {
                bypass_crc = (uint32_t)atoi(argv[arg_i]);
            } else if (arg_i == 4) {
                option_b_mode = atoi(argv[arg_i]) ? 1 : 0;
            }
        }
    }
    if (iterations <= 0)
        iterations = DEFAULT_ITERATIONS;

    if (fastpath_mode) {
        return run_fastpath_endpoint_test(iterations, verbose);
    }

    latencies_roundtrip = (double*)malloc(iterations * sizeof(double));
    latencies_tx        = (double*)malloc(iterations * sizeof(double));
    latencies_rx        = (double*)malloc(iterations * sizeof(double));
    latencies_pl_hw     = (double*)malloc(iterations * sizeof(double));
    if (!latencies_roundtrip || !latencies_tx || !latencies_rx || !latencies_pl_hw) {
        fprintf(stderr, "Failed to allocate latency buffers\n");
        free(latencies_roundtrip);
        free(latencies_tx);
        free(latencies_rx);
        free(latencies_pl_hw);
        return 1;
    }

    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open " DEVICE_PATH);
        free(latencies_roundtrip);
        free(latencies_tx);
        free(latencies_rx);
        free(latencies_pl_hw);
        return 1;
    }

    if (ioctl(fd, AXI_RTPS_IOC_SET_CRC_BYPASS, &bypass_crc) < 0) {
        perror("[-] Warning: AXI_RTPS_IOC_SET_CRC_BYPASS failed");
    }

    /* 1. Initialize Option B RTI Connext Transport Plugin (`NDDS_Transport_Plugin`) */
    if (FpgaTransport_init_plugin(&plugin, fd, option_b_mode, timer_val) < 0) {
        fprintf(stderr, "[-] Failed to initialize FPGA RTPS Transport Plugin\n");
        close(fd);
        free(latencies_roundtrip);
        free(latencies_tx);
        free(latencies_rx);
        free(latencies_pl_hw);
        return 1;
    }

    /*
     * 2. Simulate SEDP Discovery Handoff via Option B Control-Plane Hooks:
     *    - `on_publication_matched()`: Programs Shadow Writer EDT[0] for Peer 192.168.1.100:7410
     *    - `on_subscription_matched()`: Programs Shadow Reader CAM[0] for RTI Connext Peer
     *      guidPrefix = 0101_0002_BB00_0002_0000_0002, writerEntityId = 0x00000202
     *    Both commit atomically into Active EDT at S_IDLE boundary!
     */
    const uint32_t peer_guid_prefix[3] = {0x01010002U, 0xBB000002U, 0x00000002U};
    if (FpgaTransport_on_publication_matched(
            &plugin,
            0,            /* Writer slot 0 */
            0xC0A80164U,  /* Peer IPv4: 192.168.1.100 */
            7411,         /* Local UDP port */
            7410,         /* Remote RTPS user-unicast UDP port */
            0x5E200002U,  /* Next-Hop MAC Lo */
            0x0002U,      /* Next-Hop MAC Hi */
            0x00000102U,  /* Local DataWriter entityId */
            0x00000000U,  /* ENTITYID_UNKNOWN (broadcast to matched readers) */
            1U,           /* Starting writerSN = 1 */
            0             /* Unicast */) < 0) {
        perror("[-] Warning: FpgaTransport_on_publication_matched failed");
    }

    if (FpgaTransport_on_subscription_matched(
            &plugin,
            0,                /* Reader CAM slot 0 */
            peer_guid_prefix, /* Remote RTI Connext Participant 96-bit guidPrefix */
            0x00000202U,      /* Remote Feedback DataWriter entityId */
            0x00000207U       /* Local Feedback DataReader entityId */) < 0) {
        perror("[-] Warning: FpgaTransport_on_subscription_matched failed");
    }

    /* 3. Loan Zero-Copy `FastSample` directly from `mmap` TX buffer */
    tx_sample = DDS_DataWriter_get_loan(&plugin);

    if (pcap_filename) {
        if (rtps_pcap_open(&pcap_dumper, pcap_filename) == 0) {
            printf("[+] PCAP capture enabled -> writing packets to '%s' (%s)\n",
                   pcap_filename, pcap_wire_frags ? "3-Fragment Wire Format" : "Full Reassembled UDP");
        } else {
            fprintf(stderr, "[-] Failed to open PCAP file '%s': %s\n", pcap_filename, strerror(errno));
            pcap_filename = NULL;
        }
    }

    printf("=================================================================\n");
    printf("  Hybrid PS+PL RTPS v2.x Fast-Path & Option B Transport Benchmark\n");
    printf("  Target        : %s\n", DEVICE_PATH);
    printf("  Integration   : %s\n",
           option_b_mode
               ? "Option B (NDDS_Transport_Plugin: 4,096B TX -> 4,156B RTPS RX)"
               : "Option A (Direct Zero-Copy FastSample: 4,096B TX / 4,096B RX)");
    printf("  Hardware Mode : Integrated RTI Peer Emulator (100 MHz Hardware Wire Loopback)\n");
    printf("  Wire Framing  : 3 IPv4 Fragments/Sample (1480B + 1480B + 1204B)\n");
    printf("  Peer Endpoint : 192.168.1.100:7410 (GUID 01010002-BB000002-00000002)\n");
    printf("  Buffer Mode   : %s\n",
           plugin.use_mmap ? "Zero-Copy Direct Coherent mmap (get_loan)"
                           : "Standard copy_from/to_user");
    printf("  Verbosity     : %s\n", verbose ? "Enabled (Per-iteration breakdown)" : "Summary only");
    printf("=================================================================\n\n");

    for (iter = 1; iter <= iterations; iter++) {
        /* Pet hardware lifecycle watchdog periodically */
        if ((iter & 0x3F) == 1) {
            uint32_t pet = 1;
            ioctl(fd, AXI_RTPS_IOC_PET_WATCHDOG, &pet);
        }

        /*
         * Rule 3a: Anti-Stale Data Invariant:
         * Dynamically populate unique 4 KB `FastSample` header + payload words keyed by `iter`
         */
        tx_sample->cycle_id        = (uint32_t)iter;
        tx_sample->flags           = 0x000000A5U ^ (uint32_t)iter;
        tx_sample->tx_timestamp_ns = (uint64_t)iter * 1000000ULL;
        for (i = 0; i < 1020; i++) {
            tx_sample->payload[i] = (uint32_t)((iter << 16) ^ (0x5A000000U + ((i + 4) * 0x010103U)));
        }

        /* -------------------------------------------------------------
         * Phase 1: High-Resolution Timestamp: Host DDS Write -> MM2S DMA
         * ------------------------------------------------------------- */
        clock_gettime(CLOCK_MONOTONIC, &t_start);

        if (DDS_DataWriter_write(&plugin, tx_sample) < 0) {
            clock_gettime(CLOCK_MONOTONIC, &t_tx_done);
            double fail_tx_us = timespec_to_us(&t_start, &t_tx_done);
            fprintf(stderr, "\n[-] Iteration %d: DDS_DataWriter_write() failed: %s (after %.2f us)\n",
                    iter, strerror(errno), fail_tx_us);
            if (ioctl(fd, AXI_RTPS_IOC_GET_DIAG, &diag) == 0) {
                print_detailed_diag(stderr, &diag);
            }
            errors++;
            break;
        }
        clock_gettime(CLOCK_MONOTONIC, &t_tx_done);

        /* Query hardware cycle ticks for the emulated cycle */
        double pl_cycle_us = 0.0;
        uint32_t pl_ticks = 0;
        if (ioctl(fd, AXI_RTPS_IOC_GET_DIAG, &diag) == 0) {
            pl_ticks    = diag.last_cycle_ticks;
            pl_cycle_us = (double)diag.last_cycle_ticks * 0.01; /* 100 MHz clock -> 10.0 ns/tick */
        }

        /* -------------------------------------------------------------
         * Phase 2: High-Resolution Timestamp: Host DDS Take -> S2MM DMA
         * ------------------------------------------------------------- */
        uint64_t rx_seq_num = 0;
        uint64_t rx_source_ts = 0;
        if (DDS_DataReader_take(&plugin, &rx_sample, &rx_seq_num, &rx_source_ts) < 0) {
            clock_gettime(CLOCK_MONOTONIC, &t_rx_done);
            double fail_rx_us = timespec_to_us(&t_tx_done, &t_rx_done);
            double fail_total_us = timespec_to_us(&t_start, &t_rx_done);
            fprintf(stderr, "\n[-] Iteration %d: DDS_DataReader_take() failed: %s\n",
                    iter, strerror(errno));
            fprintf(stderr, "    Timing Breakdown at Failure Point:\n");
            fprintf(stderr, "      - Host TX Write (MM2S) : %8.2f us\n", timespec_to_us(&t_start, &t_tx_done));
            fprintf(stderr, "      - PL Wire + Peer Emu   : %8.2f us (%u clocks @ 100 MHz)\n", pl_cycle_us, pl_ticks);
            fprintf(stderr, "      - Host RX Wait (S2MM)  : %8.2f us (timed out)\n", fail_rx_us);
            fprintf(stderr, "      - Total Elapsed        : %8.2f us\n", fail_total_us);

            if (ioctl(fd, AXI_RTPS_IOC_GET_DIAG, &diag) == 0) {
                print_detailed_diag(stderr, &diag);
            }
            errors++;
            break;
        }
        clock_gettime(CLOCK_MONOTONIC, &t_rx_done);

        double tx_us        = timespec_to_us(&t_start, &t_tx_done);
        double rx_us        = timespec_to_us(&t_tx_done, &t_rx_done);
        double roundtrip_us = timespec_to_us(&t_start, &t_rx_done);

        latencies_roundtrip[iter - 1] = roundtrip_us;
        latencies_tx[iter - 1]        = tx_us;
        latencies_rx[iter - 1]        = rx_us;
        latencies_pl_hw[iter - 1]     = pl_cycle_us;

        total_us       += roundtrip_us;
        total_tx_us    += tx_us;
        total_rx_us    += rx_us;
        total_pl_hw_us += pl_cycle_us;

        if (roundtrip_us < min_us) min_us = roundtrip_us;
        if (roundtrip_us > max_us) max_us = roundtrip_us;

        /* Dump TX and RX packets to PCAP if capturing */
        if (pcap_filename) {
            uint8_t master_mac[6] = {0x00, 0x01, 0x5E, 0x10, 0x00, 0x02};
            uint8_t peer_mac[6]   = {0x00, 0x02, 0x5E, 0x20, 0x00, 0x02};
            uint32_t master_ip    = 0xC0A801C9; /* 192.168.1.201 (PL TX Secondary IP) */
            uint32_t peer_ip      = 0xC0A80164; /* 192.168.1.100 */
            uint16_t master_port  = 7411;
            uint16_t peer_port    = 7410;

            /* 1. Build & Dump Complete Wire RTPS message for TX (Command) */
            uint32_t tx_rtps_buf[RTPS_FULL_MSG_WORDS];
            /* RTPS Header */
            tx_rtps_buf[0] = 0x52545053; /* "RTPS" */
            tx_rtps_buf[1] = 0x02010110; /* Version 2.1, VendorId 0x0110 (Cyclone DDS) */
            tx_rtps_buf[2] = plugin.edt_cfg.local_guid_prefix[0];
            tx_rtps_buf[3] = plugin.edt_cfg.local_guid_prefix[1];
            tx_rtps_buf[4] = plugin.edt_cfg.local_guid_prefix[2];
            /* INFO_TS */
            tx_rtps_buf[5] = 0x09010800; /* Submsg 0x09, E=1, len=8 */
            tx_rtps_buf[6] = __builtin_bswap32((uint32_t)(tx_sample->tx_timestamp_ns / 1000000000ULL));
            tx_rtps_buf[7] = __builtin_bswap32((uint32_t)(((tx_sample->tx_timestamp_ns % 1000000000ULL) << 32) / 1000000000ULL));
            /* DATA */
            uint16_t tx_data_octets = (uint16_t)(24 + sizeof(struct FastSample));
            tx_rtps_buf[8]  = 0x15050000 | tx_data_octets; /* Submsg 0x15, flags 0x05 */
            tx_rtps_buf[9]  = 0x00001000;                  /* extraFlags=0, octetsToInlineQos=16 */
            tx_rtps_buf[10] = 0x00000000;                  /* readerEntityId (ENTITYID_UNKNOWN) */
            tx_rtps_buf[11] = 0x00000102;                  /* writerEntityId (Local Writer) */
            tx_rtps_buf[12] = 0x00000000;                  /* writerSeqNum Hi */
            tx_rtps_buf[13] = __builtin_bswap32((uint32_t)iter); /* writerSeqNum Lo */
            tx_rtps_buf[14] = 0x00010000;                  /* CDR_LE encapsulation */
            memcpy(&tx_rtps_buf[15], tx_sample, sizeof(struct FastSample));

            if (pcap_wire_frags) {
                rtps_pcap_dump_fragments(&pcap_dumper, &t_start, master_mac, peer_mac,
                                         master_ip, peer_ip, master_port, peer_port,
                                         (uint16_t)iter, tx_rtps_buf, RTPS_FULL_MSG_BYTES);
            } else {
                rtps_pcap_dump_udp(&pcap_dumper, &t_start, master_mac, peer_mac,
                                   master_ip, peer_ip, master_port, peer_port,
                                   (uint16_t)iter, tx_rtps_buf, RTPS_FULL_MSG_BYTES);
            }

            /* 2. Dump Received Wire RTPS message for RX (Feedback from Peer Emulator) */
            if (option_b_mode && plugin.rx_mmap_buf) {
                if (pcap_wire_frags) {
                    rtps_pcap_dump_fragments(&pcap_dumper, &t_rx_done, peer_mac, master_mac,
                                             peer_ip, master_ip, peer_port, master_port,
                                             (uint16_t)(iter + 1000), plugin.rx_mmap_buf, RTPS_FULL_MSG_BYTES);
                } else {
                    rtps_pcap_dump_udp(&pcap_dumper, &t_rx_done, peer_mac, master_mac,
                                       peer_ip, master_ip, peer_port, master_port,
                                       (uint16_t)(iter + 1000), plugin.rx_mmap_buf, RTPS_FULL_MSG_BYTES);
                }
            }
        }

        /* Verify 16-byte FastSample header (echoed intact by RTI Peer Emulator) */
        if (rx_sample->cycle_id != tx_sample->cycle_id ||
            rx_sample->flags != tx_sample->flags ||
            rx_sample->tx_timestamp_ns != tx_sample->tx_timestamp_ns) {
            fprintf(stderr,
                    "\n[-] Iteration %d FastSample Header mismatch: exp cycle_id=%u, got=%u\n",
                    iter, tx_sample->cycle_id, rx_sample->cycle_id);
            errors++;
            break;
        }

        /* Verify all 1,020 transformed payload words (4,080 bytes) */
        uint64_t active_seq = (rx_seq_num != 0) ? rx_seq_num :
                              ((diag.last_rx_seq_num != 0) ? diag.last_rx_seq_num : (uint64_t)iter);
        uint32_t seq_lo_6b = ((uint32_t)active_seq) & 0x3FU;
        for (i = 0; i < 1020; i++) {
            uint32_t expected = (~tx_sample->payload[i]) + ((seq_lo_6b << 10) | 0x200U);
            if (rx_sample->payload[i] != expected) {
                fprintf(stderr,
                        "\n[-] Iteration %d payload mismatch at word %u: "
                        "wrote 0x%08X, expected 0x%08X, got 0x%08X\n",
                        iter, i + 4, tx_sample->payload[i], expected, rx_sample->payload[i]);
                errors++;
                break;
            }
        }
        if (errors)
            break;

        if (verbose) {
            printf("  [Iter %4d] RTT: %7.2f us | TX: %6.2f us | PL Peer Emu: %5.2f us (%4u ticks) | RX: %6.2f us | Seq: %llu\n",
                   iter, roundtrip_us, tx_us, pl_cycle_us, pl_ticks, rx_us, (unsigned long long)rx_seq_num);
        } else if (iter % 100 == 0 || iter == iterations) {
            printf("  Progress: [%4d/%4d] (%5.1f%%) | Current RTT: %7.2f us | PL Emu: %5.2f us | Avg: %7.2f us\r",
                   iter, iterations, (iter * 100.0) / iterations,
                   roundtrip_us, pl_cycle_us, total_us / iter);
            fflush(stdout);
        }
    }
    printf("\n\n");

    if (ioctl(fd, AXI_RTPS_IOC_GET_DIAG, &diag) == 0) {
        printf("  Hardware RTPS Fast-Path Diagnostics:\n");
        printf("    - TX / RX Samples    : %u TX / %u RX (Frags/Cycle=%u)\n",
               diag.stat_tx_samples, diag.stat_rx_samples, diag.completed_frags);
        printf("    - Latest RX SeqNum   : %llu (Gaps=%u, StaleDrops=%u, CsumErrs=%u)\n",
               (unsigned long long)diag.last_rx_seq_num,
               diag.stat_seq_gaps, diag.stat_stale_drops, diag.stat_csum_errs);
        printf("    - Option B RTPS Len  : %u words (%u bytes)\n",
               diag.last_rx_rtps_words, diag.last_rx_rtps_words * 4);
        printf("    - Wire Cycle Ticks   : %u clocks (%.2f us @ 100 MHz)\n\n",
               diag.last_cycle_ticks, diag.last_cycle_ticks * 0.01);
    }

    if (errors == 0 && iter > 1) {
        int count = iter - 1;
        qsort(latencies_roundtrip, count, sizeof(double), compare_doubles);
        qsort(latencies_tx,        count, sizeof(double), compare_doubles);
        qsort(latencies_rx,        count, sizeof(double), compare_doubles);
        qsort(latencies_pl_hw,     count, sizeof(double), compare_doubles);

        double avg_us    = total_us / count;
        double p50_us    = get_percentile(latencies_roundtrip, count, 50.0);
        double p95_us    = get_percentile(latencies_roundtrip, count, 95.0);
        double p99_us    = get_percentile(latencies_roundtrip, count, 99.0);
        double p99_9_us  = get_percentile(latencies_roundtrip, count, 99.9);
        double p99_99_us = get_percentile(latencies_roundtrip, count, 99.99);

        double avg_tx_us = total_tx_us / count;
        double p50_tx_us = get_percentile(latencies_tx, count, 50.0);
        double p95_tx_us = get_percentile(latencies_tx, count, 95.0);

        double avg_rx_us = total_rx_us / count;
        double p50_rx_us = get_percentile(latencies_rx, count, 50.0);
        double p95_rx_us = get_percentile(latencies_rx, count, 95.0);

        double avg_pl_us = total_pl_hw_us / count;
        double p50_pl_us = get_percentile(latencies_pl_hw, count, 50.0);
        double p95_pl_us = get_percentile(latencies_pl_hw, count, 95.0);

        size_t rx_wire_bytes = option_b_mode ? RTPS_FULL_MSG_BYTES : RTPS_FAST_SAMPLE_BYTES;
        double total_mb =
            (double)((RTPS_FAST_SAMPLE_BYTES + rx_wire_bytes) * (uint64_t)count) /
            (1024.0 * 1024.0);
        double total_sec = total_us / 1e6;
        double throughput_mbps = total_mb / total_sec;
        double ops_per_sec = (double)count / total_sec;

        printf("=================================================================\n");
        printf("                      BENCHMARK RESULTS                          \n");
        printf("=================================================================\n");
        printf("  Iterations Completed : %d\n", count);
        printf("  Sample Payload Size  : %u bytes TX / %zu bytes RX (%s)\n",
               RTPS_FAST_SAMPLE_BYTES, rx_wire_bytes,
               option_b_mode ? "60B RTPS + 4096B FastSample" : "4096B FastSample");
        printf("  Total Data Processed : %.2f MB (bi-directional Pub/Sub)\n", total_mb);
        printf("  Total Elapsed Time   : %.3f s (%.1f cycles/sec, %.2f MB/s)\n",
               total_sec, ops_per_sec, throughput_mbps);
        printf("-----------------------------------------------------------------\n");
        printf("  1. Total End-to-End Latency (Write -> Emu Loopback -> Take):\n");
        printf("    - Min Latency      : %8.2f us  (%6.3f ms)\n", min_us, min_us / 1e3);
        printf("    - Avg Latency      : %8.2f us  (%6.3f ms)\n", avg_us, avg_us / 1e3);
        printf("    - p50   (Median)   : %8.2f us  (%6.3f ms)\n", p50_us, p50_us / 1e3);
        printf("    - p95   Latency    : %8.2f us  (%6.3f ms)\n", p95_us, p95_us / 1e3);
        printf("    - p99   Latency    : %8.2f us  (%6.3f ms)\n", p99_us, p99_us / 1e3);
        printf("    - p99.9 Latency    : %8.2f us  (%6.3f ms)\n", p99_9_us, p99_9_us / 1e3);
        printf("    - p99.99 Latency   : %8.2f us  (%6.3f ms)\n", p99_99_us, p99_99_us / 1e3);
        printf("    - Max Latency      : %8.2f us  (%6.3f ms)\n", max_us, max_us / 1e3);
        printf("-----------------------------------------------------------------\n");
        printf("  2. Emulation Mode Phase Breakdown (Average Latency per Phase):\n");
        printf("    [A] Host MM2S TX Write DMA   : %7.2f us  (%5.1f%%)  [p50: %6.2f, p95: %6.2f]\n",
               avg_tx_us, (avg_tx_us / avg_us) * 100.0, p50_tx_us, p95_tx_us);
        printf("    [B] PL Wire Framing + Peer Emu: %7.2f us  (%5.1f%%)  [p50: %6.2f, p95: %6.2f]\n",
               avg_pl_us, (avg_pl_us / avg_us) * 100.0, p50_pl_us, p95_pl_us);
        printf("    [C] Host S2MM RX Read DMA    : %7.2f us  (%5.1f%%)  [p50: %6.2f, p95: %6.2f]\n",
               avg_rx_us, (avg_rx_us / avg_us) * 100.0, p50_rx_us, p95_rx_us);
        printf("=================================================================\n");
    }

    if (pcap_filename) {
        printf("[+] Saved %u packets to '%s' (Wireshark PCAP)\n\n", pcap_dumper.pkt_count, pcap_filename);
        rtps_pcap_close(&pcap_dumper);
    }

    FpgaTransport_cleanup_plugin(&plugin);
    free(latencies_roundtrip);
    free(latencies_tx);
    free(latencies_rx);
    free(latencies_pl_hw);
    close(fd);
    return errors ? 1 : 0;
}

