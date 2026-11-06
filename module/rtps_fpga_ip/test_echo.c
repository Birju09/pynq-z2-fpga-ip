// Build: gcc -O2 -pthread -o test_echo test.c
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define SLOTS 1024 /* power of two */
#define SLOT_SZ 9216

static struct { uint16_t len; struct sockaddr_in peer; uint8_t data[SLOT_SZ]; } ring[SLOTS];
static _Atomic unsigned head; /* RX-owned producer index */
static _Atomic unsigned tail; /* TX-owned consumer index */

static int sock;
static struct sockaddr_in dest;

static void put_be32(uint8_t *p, uint32_t v) {
    p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v;
}

static void put_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void *rx_thread(void *arg) {
    (void)arg;
    uint32_t echo_seq = 0;
    for (;;) {
        unsigned producer = atomic_load_explicit(&head, memory_order_relaxed);
        while (producer - atomic_load_explicit(&tail, memory_order_acquire) == SLOTS) {
            sched_yield();
        }

        unsigned i = producer & (SLOTS - 1);
        uint8_t *b = ring[i].data;
        socklen_t addrlen = sizeof(ring[i].peer);
        ssize_t n = recvfrom(sock, b, SLOT_SZ, 0, (struct sockaddr *)&ring[i].peer, &addrlen);
        if (n < 0) { perror("recvfrom"); continue; }
        ring[i].peer.sin_port = htons(7411); /* Respond to FPGA S2MM listener port */

        if (n >= 56 && memcmp(b, "RTPS", 4) == 0) {
            /* Patch GuidPrefix to match Reader CAM[0]: 01010002-BB000002-00000002 */
            put_be32(b + 8,  0x01010002);
            put_be32(b + 12, 0xBB000002);
            put_be32(b + 16, 0x00000002);

            /* DATA submessage: readerEntityId = ENTITYID_UNKNOWN (0) */
            put_be32(b + 40, 0x00000000);

            /* DATA submessage: remote writerEntityId = 0x00000202 */
            put_be32(b + 44, 0x00000202);

            /*
             * Monotonically incrementing sequence number (Little-Endian per RTPS E=1 flag).
             * Word +4 is SeqNum Hi (bytes 48..51 = 0), Word +5 is SeqNum Lo (bytes 52..55 = ++echo_seq).
             * Without this, out-of-order or stale sequence numbers are dropped by the FPGA latest-value filter!
             */
            echo_seq++;
            put_le32(b + 48, 0);
            put_le32(b + 52, echo_seq);

            if (echo_seq == 1) {
                char ip_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &ring[i].peer.sin_addr, ip_str, sizeof(ip_str));
                printf("[+] Received first sample from %s (%d B) -> Echoing to %s:7411 (Seq 1)\n",
                       ip_str, (int)n, ip_str);
                fflush(stdout);
            }
        }
        ring[i].len = (uint16_t)n;

        atomic_store_explicit(&head, producer + 1, memory_order_release);
    }
    return NULL;
}

static void *tx_thread(void *arg) {
    (void)arg;
    unsigned long count = 0;
    for (;;) {
        unsigned consumer = atomic_load_explicit(&tail, memory_order_relaxed);
        while (atomic_load_explicit(&head, memory_order_acquire) == consumer) {
            sched_yield();
        }

        unsigned i = consumer & (SLOTS - 1);
        unsigned len = ring[i].len;
        if (sendto(sock, ring[i].data, len, 0,
                   (struct sockaddr *)&ring[i].peer, sizeof(ring[i].peer)) < 0)
            perror("sendto");

        atomic_store_explicit(&tail, consumer + 1, memory_order_release);

        count++;
        if (count == 1 || count % 100 == 0) {
            printf("Echoed %lu RTPS samples (%u B)\n", count, len);
            fflush(stdout);
        }
    }
    return NULL;
}

int main(void) {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return 1; }

    int one = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);

    int buf_sz = 8 * 1024 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &buf_sz, sizeof buf_sz);
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &buf_sz, sizeof buf_sz);

    struct sockaddr_in local = {0};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(7410);
    if (bind(sock, (struct sockaddr *)&local, sizeof local) < 0) {
        perror("bind");
        return 1;
    }

    dest.sin_family = AF_INET;
    dest.sin_port = htons(7411);
    inet_pton(AF_INET, "192.168.1.10", &dest.sin_addr);

    printf("[+] Diagnostic RTPS Echo Server listening on 0.0.0.0:7410...\n");

    pthread_t rx, tx;
    pthread_create(&rx, NULL, rx_thread, NULL);
    pthread_create(&tx, NULL, tx_thread, NULL);
    pthread_join(rx, NULL);
    pthread_join(tx, NULL);
    return 0;
}
