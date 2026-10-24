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
    int verbose = 0;
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
            printf("   or: %s [--iterations N] [--timer N] [--bypass-crc] [--option-b (1|0)] [--verbose]\n\n", argv[0]);
            printf("Options:\n");
            printf("  -n, --iterations N     Number of cyclic RTPS iterations (default: %d)\n", DEFAULT_ITERATIONS);
            printf("  -t, --timer N          Hardware wire pacing delay cycles per beat (default: 0)\n");
            printf("  --bypass-crc           Bypass CPU DMA0 CRC-32 check\n");
            printf("  -b, --option-b (0|1)   1 for Option B 4,156B RTPS RX (default: 1), 0 for Option A 4,096B\n");
            printf("  --emulation            Enable hardware RTI Peer Emulator (enabled by default in PL)\n");
            printf("  -v, --verbose          Print per-iteration hardware emulation and phase timing\n");
            return 0;
        } else if ((strcmp(argv[arg_i], "-n") == 0 || strcmp(argv[arg_i], "--iterations") == 0) && arg_i + 1 < argc) {
            iterations = atoi(argv[++arg_i]);
        } else if ((strcmp(argv[arg_i], "-t") == 0 || strcmp(argv[arg_i], "--timer") == 0) && arg_i + 1 < argc) {
            timer_val = (uint32_t)atoi(argv[++arg_i]);
        } else if (strcmp(argv[arg_i], "--bypass-crc") == 0) {
            bypass_crc = 1;
        } else if ((strcmp(argv[arg_i], "-b") == 0 || strcmp(argv[arg_i], "--option-b") == 0) && arg_i + 1 < argc) {
            option_b_mode = atoi(argv[++arg_i]) ? 1 : 0;
        } else if (strcmp(argv[arg_i], "-v") == 0 || strcmp(argv[arg_i], "--verbose") == 0) {
            verbose = 1;
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
                uint8_t cur_state = (uint8_t)((diag.status_reg & 0x1E) >> 1);
                fprintf(stderr, "    Hardware Diagnostic State:\n");
                fprintf(stderr, "      - FSM State         : %u [%s]\n", cur_state, rtps_state_name(cur_state));
                fprintf(stderr, "      - Status Reg        : 0x%08X (IRQ=%u, ERR_CRC=%u, ERR_TIMEOUT=%u)\n",
                        diag.status_reg, diag.status_reg & 1, diag.err_crc, diag.err_rx_timeout);
                fprintf(stderr, "      - Wire Cycle Clocks : %u ticks (%.2f us @ 100 MHz)\n",
                        diag.last_cycle_ticks, diag.last_cycle_ticks * 0.01);
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
                uint8_t cur_state = (uint8_t)((diag.status_reg & 0x1E) >> 1);
                fprintf(stderr, "    Hardware Diagnostic State:\n");
                fprintf(stderr, "      - FSM State            : %u [%s]\n", cur_state, rtps_state_name(cur_state));
                fprintf(stderr, "      - Status Reg           : 0x%08X (IRQ=%u, ERR_CRC=%u, NO_MATCH=%u, TIMEOUT=%u, CSUM=%u)\n",
                        diag.status_reg, diag.status_reg & 1, diag.err_crc, diag.err_no_match, diag.err_rx_timeout, diag.err_csum);
                fprintf(stderr, "      - TX / RX Samples      : %u TX / %u RX\n", diag.stat_tx_samples, diag.stat_rx_samples);
                fprintf(stderr, "      - Latest RX SeqNum     : %llu\n", (unsigned long long)diag.last_rx_seq_num);
                fprintf(stderr, "      - Stale Drops / Csum   : %u drops, %u csum errors\n", diag.stat_stale_drops, diag.stat_csum_errs);
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

    FpgaTransport_cleanup_plugin(&plugin);
    free(latencies_roundtrip);
    free(latencies_tx);
    free(latencies_rx);
    free(latencies_pl_hw);
    close(fd);
    return errors ? 1 : 0;
}

