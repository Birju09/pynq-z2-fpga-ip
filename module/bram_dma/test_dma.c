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
#define PAYLOAD_WORDS 1024 /* Words 0 .. 1023 */
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
    uint32_t* tx_buf = NULL;
    uint32_t* rx_buf = NULL;
    double* latencies;
    ssize_t bytes_written, bytes_read;
    int i, iter, errors = 0;
    int iterations = DEFAULT_ITERATIONS;
    uint32_t payload_len = PAYLOAD_WORDS;
    uint32_t timer_val = 0; // Default: 0 delay cycles for max throughput
    uint32_t bypass_crc = 0; // Default: 0 (verify CRC)
    int use_mmap = 0;
    size_t mmap_size = BRAM_IP_MAX_BYTES;

    double min_us = 1e12;
    double max_us = 0.0;
    double total_us = 0.0;
    struct timespec start, end;

    // Handle Help / Usage Flag
    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        printf("Usage: %s [iterations] [payload_words] [timer_cycles] [bypass_crc]\n\n", argv[0]);
        printf("Arguments:\n");
        printf("  iterations:     Number of test iterations (default: %d)\n", DEFAULT_ITERATIONS);
        printf("  payload_words:  Payload size in 32-bit words, 1-%d (default: %d)\n",
               BRAM_IP_MAX_WORDS, PAYLOAD_WORDS);
        printf("  timer_cycles:   Hardware compute delay cycles per word (default: %d)\n", 0);
        printf("  bypass_crc:     1 to bypass CPU CRC check for max throughput (default: %d)\n", 0);
        return 0;
    }

    // Argument 1: Iterations (optional)
    if (argc >= 2) {
        iterations = atoi(argv[1]);
        if (iterations <= 0) {
            fprintf(stderr, "[-] Warning: Invalid iterations '%s', using default: %d\n",
                    argv[1], DEFAULT_ITERATIONS);
            iterations = DEFAULT_ITERATIONS;
        }
    }

    // Argument 2: Payload words (optional)
    if (argc >= 3) {
        payload_len = atoi(argv[2]);
        if (payload_len <= 0 || payload_len > BRAM_IP_MAX_WORDS) {
            fprintf(stderr, "[-] Error: Invalid payload words '%s'. Must be between 1 and %d words.\n",
                    argv[2], BRAM_IP_MAX_WORDS);
            return 1;
        }
    }

    // Argument 3: Hardware timer cycles per word (optional)
    if (argc >= 4) {
        timer_val = (uint32_t)atoi(argv[3]);
    }

    // Argument 4: CRC bypass (optional)
    if (argc >= 5) {
        bypass_crc = (uint32_t)atoi(argv[4]);
    }

    uint32_t payload_bytes = payload_len * sizeof(uint32_t);

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

    // Configure hardware compute timer via IOCTL
    if (ioctl(fd, AXI_DMA_IOC_SET_TIMER, &timer_val) < 0) {
        perror("[-] Warning: Failed to set timer via ioctl");
    }

    // Configure payload length in hardware via IOCTL
    if (ioctl(fd, AXI_DMA_IOC_SET_PAYLOAD_LEN, &payload_len) < 0) {
        perror("[-] Warning: Failed to set payload length via ioctl");
    }

    // Configure CRC bypass via IOCTL
    if (ioctl(fd, AXI_DMA_IOC_SET_CRC_BYPASS, &bypass_crc) < 0) {
        perror("[-] Warning: Failed to set CRC bypass via ioctl");
    }

    /* Try zero-copy mmap of coherent DMA buffers */
    tx_buf = mmap(NULL, mmap_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, AXI_DMA_MMAP_TX_OFFSET);
    rx_buf = mmap(NULL, mmap_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, AXI_DMA_MMAP_RX_OFFSET);

    if (tx_buf != MAP_FAILED && rx_buf != MAP_FAILED) {
        use_mmap = 1;
    } else {
        printf("[-] Warning: mmap failed, falling back to malloc (copy mode)\n");
        if (tx_buf != MAP_FAILED) munmap(tx_buf, mmap_size);
        if (rx_buf != MAP_FAILED) munmap(rx_buf, mmap_size);
        tx_buf = malloc(payload_bytes);
        rx_buf = malloc(payload_bytes);
        if (!tx_buf || !rx_buf) {
            fprintf(stderr, "Failed to allocate memory\n");
            free(tx_buf);
            free(rx_buf);
            free(latencies);
            close(fd);
            return 1;
        }
    }

    printf(
        "=================================================================\n");
    printf("  Store-and-Forward BRAM Accelerator Benchmark & Verification\n");
    printf("  Target      : %s\n", DEVICE_PATH);
    printf("  Payload     : %u words (%u bytes)\n", payload_len, payload_bytes);
    printf("  Iterations  : %d | Timer Delay: %u cycles/word\n", iterations, timer_val);
    printf("  Buffer Mode : %s\n", use_mmap ? "Zero-Copy Direct Coherent mmap" : "Standard copy_from/to_user");
    printf("  CRC Checking: %s\n", bypass_crc ? "Bypassed (Zero CPU overhead)" : "Hardware + SW Verified");
    printf(
        "================================================================="
        "\n\n");

    printf("Running benchmark over %d iterations...\n", iterations);

    for (iter = 1; iter <= iterations; iter++) {
        /* Populate unique pattern per iteration to prevent stale cache/BRAM
         * hits */
        for (i = 0; i < payload_len; i++) {
            tx_buf[i] = (uint32_t)((iter << 16) ^ (0x12340000 + i));
        }
        memset(rx_buf, 0, payload_bytes);

        /* Measure complete roundtrip: MM2S -> Ingress -> Compute -> Egress ->
         * S2MM */
        clock_gettime(CLOCK_MONOTONIC, &start);

        bytes_written = write(fd, tx_buf, payload_bytes);
        if (bytes_written != payload_bytes) {
            fprintf(stderr,
                    "\n[-] Iteration %d: write() failed (wrote %zd, expected "
                    "%d): %s\n",
                    iter, bytes_written, payload_bytes, strerror(errno));
            errors++;
            break;
        }

        bytes_read = read(fd, rx_buf, payload_bytes);
        if (bytes_read != payload_bytes) {
            fprintf(stderr,
                    "\n[-] Iteration %d: read() failed (read %zd, expected "
                    "%d): %s\n",
                    iter, bytes_read, payload_bytes, strerror(errno));
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

        /* Verify computed result against hardware formula: (~tx_word) + 0x1000
         */
        for (i = 0; i < payload_len; i++) {
            uint32_t expected = (~tx_buf[i]) + 0x1000;
            if (rx_buf[i] != expected) {
                fprintf(stderr,
                        "\n[-] Iteration %d mismatch at word %d: wrote 0x%08X, "
                        "expected 0x%08X, got 0x%08X\n",
                        iter, i, tx_buf[i], expected, rx_buf[i]);
                errors++;
                break;
            }
        }

        if (errors)
            break;

        /* Periodic progress indicator */
        if (iter % 100 == 0 || iter == iterations) {
            printf(
                "  Progress: [%4d/%4d] (%5.1f%%) | Current: %7.2f us | Running "
                "Avg: %7.2f us\r",
                iter, iterations, (iter * 100.0) / iterations, elapsed_us,
                total_us / iter);
            fflush(stdout);
        }
    }
    printf("\n\n");

    if (errors == 0) {
        /* Sort latencies to compute percentiles */
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

        printf(
            "================================================================="
            "\n");
        printf(
            "                      BENCHMARK RESULTS                          "
            "\n");
        printf(
            "================================================================="
            "\n");
        printf("  Iterations Completed : %d\n", iterations);
        printf("  Payload per Transfer : %d bytes (%d words)\n", payload_bytes,
               payload_len);
        printf("  Total Data Processed : %.2f MB (Tx + Rx bi-directional)\n",
               total_mb);
        printf("  Total Elapsed Time   : %.3f s\n", total_sec);
        printf(
            "-----------------------------------------------------------------"
            "\n");
        printf("  Roundtrip Latency Profile (Write + Compute + Read):\n");
        printf("    - Min Latency      : %8.2f us  (%6.3f ms)\n", min_us,
               min_us / 1e3);
        printf("    - Avg Latency      : %8.2f us  (%6.3f ms)\n", avg_us,
               avg_us / 1e3);
        printf("    - p50   (Median)   : %8.2f us  (%6.3f ms)\n", p50_us,
               p50_us / 1e3);
        printf("    - p95   Latency    : %8.2f us  (%6.3f ms)\n", p95_us,
               p95_us / 1e3);
        printf("    - p99   Latency    : %8.2f us  (%6.3f ms)\n", p99_us,
               p99_us / 1e3);
        printf("    - p99.9 Latency    : %8.2f us  (%6.3f ms)\n", p99_9_us,
               p99_9_us / 1e3);
        printf("    - p99.99 Latency   : %8.2f us  (%6.3f ms)\n", p99_99_us,
               p99_99_us / 1e3);
        printf("    - Max Latency      : %8.2f us  (%6.3f ms)\n", max_us,
               max_us / 1e3);
        printf(
            "-----------------------------------------------------------------"
            "\n");
        printf("  Performance Throughput:\n");
        printf("    - Transaction Rate : %8.1f ops/sec\n", ops_per_sec);
        printf("    - Effective B/W    : %8.2f MB/s\n", throughput_mbps);
        printf(
            "================================================================="
            "\n");
        printf(
            "===> TEST PASSED! All %d iterations verified with zero errors! "
            "<===\n\n",
            iterations);
    } else {
        printf(
            "================================================================="
            "\n");
        printf("===> TEST FAILED at iteration %d with %d error(s)! <===\n",
               iter, errors);
        printf(
            "================================================================="
            "\n\n");
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
