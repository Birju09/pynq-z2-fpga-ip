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

#include "axi_dma.h"

#define DEVICE_PATH "/dev/axi_dma_dev"
#define DEFAULT_ITERATIONS      1000
#define DEFAULT_NUM_SLAVES      40
#define DEFAULT_BYTES_PER_SLAVE 128 /* 40 * 128 = 5,120 bytes (5 KB = 1,280 words) */

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

/*
 * Stack-Agnostic EtherCAT OP-State Handoff Builder:
 * Given the discovered slave count and PDO size per slave (from SOEM or IgH
 * EtherLab as the Master transitions Safe-Op -> Op), partitions the logical
 * Process Data Image into Ethernet MTU frames (<= 1,484 bytes) and computes
 * each frame's expected Working Counter (WKC).
 */
static int build_ecat_op_config(struct ecat_master_cfg* cfg,
                                uint32_t num_slaves,
                                uint32_t bytes_per_slave,
                                uint32_t wire_timer_ticks) {
    uint32_t total_bytes = num_slaves * bytes_per_slave;
    uint32_t max_slaves_per_frame = ECAT_MAX_FRAME_PAYLOAD_BYTES / bytes_per_slave;
    uint32_t rem_slaves = num_slaves;
    uint32_t cur_logical_addr = 0;
    uint32_t cur_word_offset = 0;
    uint32_t desc_idx = 0;

    if (max_slaves_per_frame == 0)
        max_slaves_per_frame = 1;

    memset(cfg, 0, sizeof(*cfg));
    cfg->total_pdo_words = total_bytes / sizeof(uint32_t);
    cfg->wire_timer_ticks = wire_timer_ticks;
    cfg->slave_mask_lo = 0xFFFFFFFFU;
    cfg->slave_mask_hi = 0x000000FFU;

    while (rem_slaves > 0 && desc_idx < ECAT_MAX_DESCRIPTORS) {
        uint32_t slaves_in_frame = (rem_slaves > max_slaves_per_frame)
                                       ? max_slaves_per_frame
                                       : rem_slaves;
        uint32_t frame_bytes = slaves_in_frame * bytes_per_slave;
        uint32_t frame_words = frame_bytes / sizeof(uint32_t);

        cfg->descs[desc_idx].logical_addr = cur_logical_addr;
        cfg->descs[desc_idx].payload_words = (uint16_t)frame_words;
        cfg->descs[desc_idx].ecat_cmd = ECAT_CMD_LRW;
        cfg->descs[desc_idx].datagram_idx = (uint8_t)desc_idx;
        cfg->descs[desc_idx].pdi_word_offset = (uint16_t)cur_word_offset;
        cfg->descs[desc_idx].expected_wkc = (uint16_t)(slaves_in_frame * 3);

        cur_logical_addr += frame_bytes;
        cur_word_offset += frame_words;
        rem_slaves -= slaves_in_frame;
        desc_idx++;
    }

    cfg->num_descriptors = desc_idx;
    return (rem_slaves == 0) ? 0 : -1;
}

int main(int argc, char* argv[]) {
    int fd;
    uint32_t* tx_buf = NULL;
    uint32_t* rx_buf = NULL;
    double* latencies;
    ssize_t bytes_written, bytes_read;
    int iter, errors = 0;
    uint32_t i;
    int iterations = DEFAULT_ITERATIONS;
    uint32_t num_slaves = DEFAULT_NUM_SLAVES;
    uint32_t bytes_per_slave = DEFAULT_BYTES_PER_SLAVE;
    uint32_t timer_val = 0;
    uint32_t bypass_crc = 0;
    int use_mmap = 0;
    size_t mmap_size = BRAM_IP_MAX_BYTES;
    struct ecat_master_cfg ecat_cfg;
    struct ecat_cycle_diag ecat_diag;

    double min_us = 1e12;
    double max_us = 0.0;
    double total_us = 0.0;
    struct timespec start, end;

    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        printf("Usage: %s [iterations] [num_slaves] [bytes_per_slave] [timer_cycles] [bypass_crc]\n\n",
               argv[0]);
        printf("Arguments:\n");
        printf("  iterations:      Number of cyclic OP iterations (default: %d)\n",
               DEFAULT_ITERATIONS);
        printf("  num_slaves:      Number of EtherCAT slaves on ring, 1-40 (default: %d)\n",
               DEFAULT_NUM_SLAVES);
        printf("  bytes_per_slave: PDO bytes per slave, multiple of 4 (default: %d -> 5 KB total)\n",
               DEFAULT_BYTES_PER_SLAVE);
        printf("  timer_cycles:    Hardware wire pacing delay cycles per beat (default: 0)\n");
        printf("  bypass_crc:      1 to bypass CPU DMA CRC check (default: 0)\n");
        return 0;
    }

    if (argc >= 2) {
        iterations = atoi(argv[1]);
        if (iterations <= 0)
            iterations = DEFAULT_ITERATIONS;
    }
    if (argc >= 3) {
        num_slaves = (uint32_t)atoi(argv[2]);
        if (num_slaves == 0 || num_slaves > 40) {
            fprintf(stderr, "[-] Error: num_slaves must be between 1 and 40.\n");
            return 1;
        }
    }
    if (argc >= 4) {
        bytes_per_slave = (uint32_t)atoi(argv[3]);
        if (bytes_per_slave == 0 || (bytes_per_slave % 4) != 0) {
            fprintf(stderr, "[-] Error: bytes_per_slave must be a non-zero multiple of 4.\n");
            return 1;
        }
    }
    if (argc >= 5) {
        timer_val = (uint32_t)atoi(argv[4]);
    }
    if (argc >= 6) {
        bypass_crc = (uint32_t)atoi(argv[5]);
    }

    if (build_ecat_op_config(&ecat_cfg, num_slaves, bytes_per_slave, timer_val) < 0) {
        fprintf(stderr, "[-] Error: PDO size exceeds 8-frame descriptor table capacity.\n");
        return 1;
    }

    uint32_t payload_len = ecat_cfg.total_pdo_words;
    uint32_t payload_bytes = payload_len * sizeof(uint32_t);
    uint32_t words_per_slave = bytes_per_slave / sizeof(uint32_t);

    latencies = malloc(iterations * sizeof(double));
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

    /* Hand off dynamic Slave & PDO Descriptor Table to PL Master IP (Safe-Op -> OP) */
    if (ioctl(fd, AXI_DMA_IOC_CONFIG_ECAT, &ecat_cfg) < 0) {
        perror("[-] Warning: AXI_DMA_IOC_CONFIG_ECAT failed");
    }
    if (ioctl(fd, AXI_DMA_IOC_SET_CRC_BYPASS, &bypass_crc) < 0) {
        perror("[-] Warning: AXI_DMA_IOC_SET_CRC_BYPASS failed");
    }

    tx_buf = mmap(NULL, mmap_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                  AXI_DMA_MMAP_TX_OFFSET);
    rx_buf = mmap(NULL, mmap_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                  AXI_DMA_MMAP_RX_OFFSET);

    if (tx_buf != MAP_FAILED && rx_buf != MAP_FAILED) {
        use_mmap = 1;
    } else {
        if (tx_buf != MAP_FAILED)
            munmap(tx_buf, mmap_size);
        if (rx_buf != MAP_FAILED)
            munmap(rx_buf, mmap_size);
        tx_buf = malloc(payload_bytes);
        rx_buf = malloc(payload_bytes);
        if (!tx_buf || !rx_buf) {
            free(tx_buf);
            free(rx_buf);
            free(latencies);
            close(fd);
            return 1;
        }
    }

    printf("=================================================================\n");
    printf("  Hybrid PS+PL EtherCAT Cyclic Master Benchmark (OP State)\n");
    printf("  Target        : %s\n", DEVICE_PATH);
    printf("  Topology      : %u Slaves x %u Bytes/Slave = %u Bytes (%u Words)\n",
           num_slaves, bytes_per_slave, payload_bytes, payload_len);
    printf("  EtherCAT Ring : %u Frames/Cycle | Expected Total WKC = %u\n",
           ecat_cfg.num_descriptors, num_slaves * 3);
    for (i = 0; i < ecat_cfg.num_descriptors; i++) {
        printf("    - Frame %u   : LogicalAddr=0x%04X, Len=%4uB (%3uW), ExpWKC=%2u\n",
               i, ecat_cfg.descs[i].logical_addr,
               ecat_cfg.descs[i].payload_words * 4,
               ecat_cfg.descs[i].payload_words,
               ecat_cfg.descs[i].expected_wkc);
    }
    printf("  Buffer Mode   : %s\n",
           use_mmap ? "Zero-Copy Direct Coherent mmap" : "Standard copy_from/to_user");
    printf("=================================================================\n\n");

    for (iter = 1; iter <= iterations; iter++) {
        /* Anti-stale data invariant: unique payload pattern keyed by iteration */
        for (i = 0; i < payload_len; i++) {
            tx_buf[i] = (uint32_t)((iter << 16) ^ (0x12340000U + i));
        }
        memset(rx_buf, 0, payload_bytes);

        clock_gettime(CLOCK_MONOTONIC, &start);

        bytes_written = write(fd, tx_buf, payload_bytes);
        if (bytes_written != (ssize_t)payload_bytes) {
            fprintf(stderr, "\n[-] Iteration %d: write() failed: %s\n",
                    iter, strerror(errno));
            errors++;
            break;
        }

        bytes_read = read(fd, rx_buf, payload_bytes);
        if (bytes_read != (ssize_t)payload_bytes) {
            fprintf(stderr, "\n[-] Iteration %d: read() failed: %s\n",
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

        /* Verify every word across all slaves against 40-slave emulator formula */
        for (i = 0; i < payload_len; i++) {
            uint32_t slave_id = i / words_per_slave;
            uint32_t expected = (~tx_buf[i]) + ((slave_id << 10) | 0x100U);
            if (rx_buf[i] != expected) {
                fprintf(stderr,
                        "\n[-] Iteration %d mismatch at word %u (Slave %u): "
                        "wrote 0x%08X, expected 0x%08X, got 0x%08X\n",
                        iter, i, slave_id, tx_buf[i], expected, rx_buf[i]);
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

    if (ioctl(fd, AXI_DMA_IOC_GET_ECAT_DIAG, &ecat_diag) == 0) {
        printf("  Hardware EtherCAT Cycle Diagnostics:\n");
        printf("    - Total Actual WKC : %u / %u (WKC_ERR=%u, FCS_ERR=%u)\n",
               ecat_diag.total_actual_wkc, num_slaves * 3,
               ecat_diag.wkc_err, ecat_diag.crc_err);
        printf("    - Wire Cycle Ticks : %u clocks (%.2f us @ 100 MHz)\n\n",
               ecat_diag.last_cycle_ticks, ecat_diag.last_cycle_ticks * 0.01);
    }

    if (errors == 0) {
        qsort(latencies, iterations, sizeof(double), compare_doubles);

        double avg_us = total_us / iterations;
        double p50_us = get_percentile(latencies, iterations, 50.0);
        double p95_us = get_percentile(latencies, iterations, 95.0);
        double p99_us = get_percentile(latencies, iterations, 99.0);
        double p99_9_us = get_percentile(latencies, iterations, 99.9);
        double p99_99_us = get_percentile(latencies, iterations, 99.99);

        double total_mb =
            (double)(payload_bytes * 2ULL * iterations) / (1024.0 * 1024.0);
        double total_sec = total_us / 1e6;
        double throughput_mbps = total_mb / total_sec;
        double ops_per_sec = (double)iterations / total_sec;

        printf("=================================================================\n");
        printf("                      BENCHMARK RESULTS                          \n");
        printf("=================================================================\n");
        printf("  Iterations Completed : %d\n", iterations);
        printf("  Process Data Image   : %u bytes (%u words across %u slaves)\n",
               payload_bytes, payload_len, num_slaves);
        printf("  Total Data Processed : %.2f MB (TxPDO + RxPDO bi-directional)\n",
               total_mb);
        printf("  Total Elapsed Time   : %.3f s (%.1f cycles/sec, %.2f MB/s)\n",
               total_sec, ops_per_sec, throughput_mbps);
        printf("-----------------------------------------------------------------\n");
        printf("  Roundtrip Robotics Loop Latency (MM2S + 4-Frame ECAT + S2MM):\n");
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

    if (use_mmap) {
        munmap(tx_buf, mmap_size);
        munmap(rx_buf, mmap_size);
    } else {
        free(tx_buf);
        free(rx_buf);
    }
    free(latencies);
    close(fd);
    return errors ? 1 : 0;
}

