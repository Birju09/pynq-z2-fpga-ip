#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define DEVICE_PATH "/dev/axi_dma_dev"
#define PAYLOAD_WORDS 1022                /* Words 0 .. 1021 */
#define PAYLOAD_BYTES (PAYLOAD_WORDS * 4) /* 4088 bytes */
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
    uint32_t* tx_buf;
    uint32_t* rx_buf;
    double* latencies;
    ssize_t bytes_written, bytes_read;
    int i, iter, errors = 0;
    int iterations = DEFAULT_ITERATIONS;

    double min_us = 1e12;
    double max_us = 0.0;
    double total_us = 0.0;
    struct timespec start, end;

    if (argc > 1) {
        iterations = atoi(argv[1]);
        if (iterations <= 0)
            iterations = DEFAULT_ITERATIONS;
    }

    tx_buf = malloc(PAYLOAD_BYTES);
    rx_buf = malloc(PAYLOAD_BYTES);
    latencies = malloc(iterations * sizeof(double));
    if (!tx_buf || !rx_buf || !latencies) {
        fprintf(stderr, "Failed to allocate memory\n");
        free(tx_buf);
        free(rx_buf);
        free(latencies);
        return 1;
    }

    printf(
        "=================================================================\n");
    printf("  Store-and-Forward BRAM Accelerator Benchmark & Verification\n");
    printf("  Target: %s | Payload: %d words (%d bytes)\n", DEVICE_PATH,
           PAYLOAD_WORDS, PAYLOAD_BYTES);
    printf("  Iterations: %d\n", iterations);
    printf(
        "================================================================="
        "\n\n");

    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open " DEVICE_PATH);
        free(tx_buf);
        free(rx_buf);
        free(latencies);
        return 1;
    }

    printf("Running benchmark over %d iterations...\n", iterations);

    for (iter = 1; iter <= iterations; iter++) {
        /* Populate unique pattern per iteration to prevent stale cache/BRAM
         * hits */
        for (i = 0; i < PAYLOAD_WORDS; i++) {
            tx_buf[i] = (uint32_t)((iter << 16) ^ (0x12340000 + i));
        }
        memset(rx_buf, 0, PAYLOAD_BYTES);

        /* Measure complete roundtrip: MM2S -> Ingress -> Compute -> Egress ->
         * S2MM */
        clock_gettime(CLOCK_MONOTONIC, &start);

        bytes_written = write(fd, tx_buf, PAYLOAD_BYTES);
        if (bytes_written != PAYLOAD_BYTES) {
            fprintf(stderr,
                    "\n[-] Iteration %d: write() failed (wrote %zd, expected "
                    "%d): %s\n",
                    iter, bytes_written, PAYLOAD_BYTES, strerror(errno));
            errors++;
            break;
        }

        bytes_read = read(fd, rx_buf, PAYLOAD_BYTES);
        if (bytes_read != PAYLOAD_BYTES) {
            fprintf(stderr,
                    "\n[-] Iteration %d: read() failed (read %zd, expected "
                    "%d): %s\n",
                    iter, bytes_read, PAYLOAD_BYTES, strerror(errno));
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
        for (i = 0; i < PAYLOAD_WORDS; i++) {
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
            (double)(PAYLOAD_BYTES * 2ULL * iterations) / (1024.0 * 1024.0);
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
        printf("  Payload per Transfer : %d bytes (%d words)\n", PAYLOAD_BYTES,
               PAYLOAD_WORDS);
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

    close(fd);
    free(tx_buf);
    free(rx_buf);
    free(latencies);
    return errors ? 1 : 0;
}
