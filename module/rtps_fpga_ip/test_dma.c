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

#define DEVICE_PATH        "/dev/axi_dma_dev"
#define DEFAULT_ITERATIONS 1000

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

int main(int argc, char* argv[]) {
    int fd;
    NDDS_Transport_Plugin plugin;
    struct FastSample* tx_sample = NULL;
    const struct FastSample* rx_sample = NULL;
    double* latencies = NULL;
    int iter, errors = 0;
    uint32_t i;
    int iterations = DEFAULT_ITERATIONS;
    uint32_t timer_val = 0;
    uint32_t bypass_crc = 0;
    int option_b_mode = 1; /* Default: Option B NDDS_Transport_Plugin mode (CTRL[7]=1) */
    struct rtps_cycle_diag diag;

    double min_us = 1e12;
    double max_us = 0.0;
    double total_us = 0.0;
    struct timespec start, end;

    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        printf("Usage: %s [iterations] [timer_cycles] [bypass_crc] [option_b_plugin_mode]\n\n",
               argv[0]);
        printf("Arguments:\n");
        printf("  iterations:           Number of cyclic 1 kHz RTPS iterations (default: %d)\n",
               DEFAULT_ITERATIONS);
        printf("  timer_cycles:         Hardware wire pacing delay cycles per beat (default: 0)\n");
        printf("  bypass_crc:           1 to bypass CPU DMA0 CRC-32 check (default: 0)\n");
        printf("  option_b_plugin_mode: 1 for Option B NDDS_Transport_Plugin 4,156B RTPS RX,\n");
        printf("                        0 for Option A raw 4,096B FastSample RX (default: 1)\n");
        return 0;
    }

    if (argc >= 2) {
        iterations = atoi(argv[1]);
        if (iterations <= 0)
            iterations = DEFAULT_ITERATIONS;
    }
    if (argc >= 3) {
        timer_val = (uint32_t)atoi(argv[2]);
    }
    if (argc >= 4) {
        bypass_crc = (uint32_t)atoi(argv[3]);
    }
    if (argc >= 5) {
        option_b_mode = atoi(argv[4]) ? 1 : 0;
    }

    latencies = (double*)malloc(iterations * sizeof(double));
    if (!latencies) {
        fprintf(stderr, "Failed to allocate latency buffer\n");
        return 1;
    }

    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open " DEVICE_PATH);
        free(latencies);
        return 1;
    }

    if (ioctl(fd, AXI_RTPS_IOC_SET_CRC_BYPASS, &bypass_crc) < 0) {
        perror("[-] Warning: AXI_RTPS_IOC_SET_CRC_BYPASS failed");
    }

    /* 1. Initialize Option B RTI Connext Transport Plugin (`NDDS_Transport_Plugin`) */
    if (FpgaTransport_init_plugin(&plugin, fd, option_b_mode, timer_val) < 0) {
        fprintf(stderr, "[-] Failed to initialize FPGA RTPS Transport Plugin\n");
        close(fd);
        free(latencies);
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

    printf("=================================================================\n");
    printf("  Hybrid PS+PL RTPS v2.x Fast-Path & Option B Transport Benchmark\n");
    printf("  Target        : %s\n", DEVICE_PATH);
    printf("  Integration   : %s\n",
           option_b_mode
               ? "Option B (NDDS_Transport_Plugin: 4,096B TX -> 4,156B RTPS RX)"
               : "Option A (Direct Zero-Copy FastSample: 4,096B TX / 4,096B RX)");
    printf("  Wire Framing  : 3 IPv4 Fragments/Sample (1480B + 1480B + 1204B)\n");
    printf("  Peer Endpoint : 192.168.1.100:7410 (GUID 01010002-BB000002-00000002)\n");
    printf("  Buffer Mode   : %s\n",
           plugin.use_mmap ? "Zero-Copy Direct Coherent mmap (get_loan)"
                           : "Standard copy_from/to_user");
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

        clock_gettime(CLOCK_MONOTONIC, &start);

        /*
         * Standard DDS Write -> invokes `plugin.send()` -> strips dummy CPU RTPS header
         * and DMAs the 4,096-byte `FastSample` into the FPGA Fast-Path IP.
         */
        if (DDS_DataWriter_write(&plugin, tx_sample) < 0) {
            fprintf(stderr, "\n[-] Iteration %d: DDS_DataWriter_write() failed: %s\n",
                    iter, strerror(errno));
            errors++;
            break;
        }

        /*
         * Standard DDS Take -> invokes `plugin.receive_rEA()` -> receives the 4,156-byte
         * hardware-validated RTPS v2.x message (Option B) or 4,096-byte FastSample (Option A).
         */
        uint64_t rx_seq_num = 0;
        uint64_t rx_source_ts = 0;
        if (DDS_DataReader_take(&plugin, &rx_sample, &rx_seq_num, &rx_source_ts) < 0) {
            fprintf(stderr, "\n[-] Iteration %d: DDS_DataReader_take() failed: %s\n",
                    iter, strerror(errno));
            errors++;
            break;
        }

        clock_gettime(CLOCK_MONOTONIC, &end);

        double elapsed_us = timespec_to_us(&start, &end);
        latencies[iter - 1] = elapsed_us;
        if (elapsed_us < min_us)
            min_us = elapsed_us;
        if (elapsed_us > max_us)
            max_us = elapsed_us;
        total_us += elapsed_us;

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
        uint32_t seq_lo_6b = ((uint32_t)iter) & 0x3FU;
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

        if (iter % 100 == 0 || iter == iterations) {
            printf("  Progress: [%4d/%4d] (%5.1f%%) | Current: %7.2f us | Avg: %7.2f us\r",
                   iter, iterations, (iter * 100.0) / iterations,
                   elapsed_us, total_us / iter);
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

    if (errors == 0) {
        qsort(latencies, iterations, sizeof(double), compare_doubles);

        double avg_us    = total_us / iterations;
        double p50_us    = get_percentile(latencies, iterations, 50.0);
        double p95_us    = get_percentile(latencies, iterations, 95.0);
        double p99_us    = get_percentile(latencies, iterations, 99.0);
        double p99_9_us  = get_percentile(latencies, iterations, 99.9);
        double p99_99_us = get_percentile(latencies, iterations, 99.99);

        size_t rx_wire_bytes = option_b_mode ? RTPS_FULL_MSG_BYTES : RTPS_FAST_SAMPLE_BYTES;
        double total_mb =
            (double)((RTPS_FAST_SAMPLE_BYTES + rx_wire_bytes) * (uint64_t)iterations) /
            (1024.0 * 1024.0);
        double total_sec = total_us / 1e6;
        double throughput_mbps = total_mb / total_sec;
        double ops_per_sec = (double)iterations / total_sec;

        printf("=================================================================\n");
        printf("                      BENCHMARK RESULTS                          \n");
        printf("=================================================================\n");
        printf("  Iterations Completed : %d\n", iterations);
        printf("  Sample Payload Size  : %u bytes TX / %zu bytes RX (%s)\n",
               RTPS_FAST_SAMPLE_BYTES, rx_wire_bytes,
               option_b_mode ? "60B RTPS + 4096B FastSample" : "4096B FastSample");
        printf("  Total Data Processed : %.2f MB (bi-directional Pub/Sub)\n",
               total_mb);
        printf("  Total Elapsed Time   : %.3f s (%.1f cycles/sec, %.2f MB/s)\n",
               total_sec, ops_per_sec, throughput_mbps);
        printf("-----------------------------------------------------------------\n");
        printf("  Roundtrip DDS Write->Take Latency (MM2S + 3-Frag RTPS + S2MM):\n");
        printf("    - Min Latency      : %8.2f us  (%6.3f ms)\n", min_us, min_us / 1e3);
        printf("    - Avg Latency      : %8.2f us  (%6.3f ms)\n", avg_us, avg_us / 1e3);
        printf("    - p50   (Median)   : %8.2f us  (%6.3f ms)\n", p50_us, p50_us / 1e3);
        printf("    - p95   Latency    : %8.2f us  (%6.3f ms)\n", p95_us, p95_us / 1e3);
        printf("    - p99   Latency    : %8.2f us  (%6.3f ms)\n", p99_us, p99_us / 1e3);
        printf("    - p99.9 Latency    : %8.2f us  (%6.3f ms)\n", p99_9_us, p99_9_us / 1e3);
        printf("    - p99.99 Latency   : %8.2f us  (%6.3f ms)\n", p99_99_us, p99_99_us / 1e3);
        printf("    - Max Latency      : %8.2f us  (%6.3f ms)\n", max_us, max_us / 1e3);
        printf("=================================================================\n");
    }

    FpgaTransport_cleanup_plugin(&plugin);
    free(latencies);
    close(fd);
    return errors ? 1 : 0;
}

