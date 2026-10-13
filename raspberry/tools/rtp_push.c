/*
 * rtp_push.c — Annex B H.264 on stdin -> RFC 6184 RTP over UDP, no server in between.
 * C port of rtp_push.py (same behaviour, ~10x less CPU). Build on the Pi:
 *     gcc -O2 -o rtp_push rtp_push.c
 * Usage: rpicam-vid ... --inline --flush -o - | ./rtp_push --host 10.42.0.173 --port 5000 [--pt 96] [--mtu 1400] [--stats 5]
 *
 * - no parsing delay: a frame is sent as soon as its bytes arrive (writer uses stdbuf -o0, one
 *   write per frame); a short quiet time (COALESCE_US) only catches the rare split write.
 * - access-unit boundaries are also detected in the bitstream (SPS/PPS/AUD, or a slice with
 *   first_mb_in_slice == 0 after another slice), so two frames in one read are still split.
 * - SPS/PPS are cached and re-sent in front of every IDR, so the viewer can join at any keyframe.
 * - RTP timestamp = send time on a 90 kHz clock; marker bit on the last packet of each frame.
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define BUF_SZ (4u << 20)
#define MAX_NALS 512
#define COALESCE_US 400
#define MAX_COALESCE_ROUNDS 8

static int sock_fd = -1;
static struct sockaddr_in dst;
static int pt = 96, mtu = 1400;
static uint32_t ssrc;
static uint16_t seq;
static uint8_t pkt[2048];
static uint64_t st_packets, st_bytes, st_frames, st_splits, st_tails;

static uint8_t *sps = NULL, *pps = NULL;
static size_t sps_len = 0, pps_len = 0;

typedef struct { const uint8_t *p; size_t len; } nal_t;

static uint32_t rtp_ts(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 90000u + (uint64_t)ts.tv_nsec * 9 / 100000);
}

static void send_pkt(const uint8_t *payload, size_t len, uint32_t ts, int marker) {
    pkt[0] = 0x80;
    pkt[1] = (uint8_t)(pt | (marker ? 0x80 : 0));
    pkt[2] = seq >> 8; pkt[3] = seq & 0xff;
    pkt[4] = ts >> 24; pkt[5] = ts >> 16; pkt[6] = ts >> 8; pkt[7] = ts;
    pkt[8] = ssrc >> 24; pkt[9] = ssrc >> 16; pkt[10] = ssrc >> 8; pkt[11] = ssrc;
    memcpy(pkt + 12, payload, len);
    sendto(sock_fd, pkt, 12 + len, 0, (struct sockaddr *)&dst, sizeof dst);
    seq++;
    st_packets++;
    st_bytes += 12 + len;
}

static void send_nal(const uint8_t *nal, size_t len, uint32_t ts, int last_of_au) {
    size_t max = (size_t)mtu - 12;
    if (len <= max) { send_pkt(nal, len, ts, last_of_au); return; }
    uint8_t hdr = nal[0];
    uint8_t fu_ind = (hdr & 0xE0) | 28;
    uint8_t typ = hdr & 0x1F;
    size_t pos = 1, chunk = max - 2;
    int first = 1;
    static uint8_t frag[2048];
    while (pos < len) {
        size_t n = len - pos < chunk ? len - pos : chunk;
        int end = (pos + n == len);
        frag[0] = fu_ind;
        frag[1] = (uint8_t)((first ? 0x80 : 0) | (end ? 0x40 : 0) | typ);
        memcpy(frag + 2, nal + pos, n);
        send_pkt(frag, n + 2, ts, last_of_au && end);
        pos += n;
        first = 0;
    }
}

static void emit_au(nal_t *nals, int n) {
    if (n == 0) return;
    int has_sps = 0, first_vcl_type = 0;
    for (int i = 0; i < n; i++) {
        int t = nals[i].p[0] & 0x1F;
        if (t == 7) has_sps = 1;
        if (!first_vcl_type && (t == 1 || t == 5)) first_vcl_type = t;
    }
    uint32_t ts = rtp_ts();
    if (first_vcl_type == 5 && !has_sps && sps && pps) {
        send_nal(sps, sps_len, ts, 0);
        send_nal(pps, pps_len, ts, 0);
    }
    for (int i = 0; i < n; i++) send_nal(nals[i].p, nals[i].len, ts, i == n - 1);
    st_frames++;
}

static int is_new_au(const uint8_t *nal, size_t len, int au_has_slice) {
    int t = nal[0] & 0x1F;
    if (t == 7 || t == 8 || t == 9) return au_has_slice;
    if ((t == 1 || t == 5) && len > 1) return au_has_slice && (nal[1] & 0x80);
    return 0;
}

/* find next 00 00 01 at or after i; returns index of the 01 byte + 1 (payload start) or -1 */
static ssize_t find_start(const uint8_t *b, size_t n, size_t i) {
    for (; i + 2 < n; i++)
        if (b[i] == 0 && b[i + 1] == 0 && b[i + 2] == 1) return (ssize_t)(i + 3);
    return -1;
}

int main(int argc, char **argv) {
    const char *host = NULL; int port = 0; double stats_s = 5.0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--host") && i + 1 < argc) host = argv[++i];
        else if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pt") && i + 1 < argc) pt = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--mtu") && i + 1 < argc) mtu = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--stats") && i + 1 < argc) stats_s = atof(argv[++i]);
    }
    if (!host || !port) { fprintf(stderr, "usage: rtp_push --host IP --port N [--pt 96] [--mtu 1400] [--stats 5]\n"); return 2; }

    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    int sndbuf = 4 << 20; setsockopt(sock_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof sndbuf);
    int tos = 0xB8; setsockopt(sock_fd, IPPROTO_IP, IP_TOS, &tos, sizeof tos);   /* DSCP EF */
    memset(&dst, 0, sizeof dst);
    dst.sin_family = AF_INET; dst.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, host, &dst.sin_addr) != 1) { fprintf(stderr, "bad host %s\n", host); return 2; }

    srand((unsigned)time(NULL) ^ (unsigned)getpid());
    ssrc = ((uint32_t)rand() << 16) ^ (uint32_t)rand();
    seq = (uint16_t)rand();

    fcntl(0, F_SETPIPE_SZ, 1 << 20);
    uint8_t *buf = malloc(BUF_SZ);
    size_t carry = 0;                 /* bytes of trailing non-VCL NALs kept for the next frame */
    nal_t nals[MAX_NALS];
    struct timespec t_stats; clock_gettime(CLOCK_MONOTONIC, &t_stats);

    for (;;) {
        ssize_t r = read(0, buf + carry, BUF_SZ - carry);
        if (r <= 0) { if (r < 0 && errno == EINTR) continue; break; }
        size_t n = carry + (size_t)r;
        for (int round = 0; round < MAX_COALESCE_ROUNDS && n < BUF_SZ; round++) {
            struct pollfd pfd = { .fd = 0, .events = POLLIN };
            if (poll(&pfd, 1, 0) <= 0) {                       /* quiet-time check in microseconds */
                struct timespec w = { 0, COALESCE_US * 1000L };
                nanosleep(&w, NULL);
                if (poll(&pfd, 1, 0) <= 0) break;
            }
            ssize_t m = read(0, buf + n, BUF_SZ - n);
            if (m <= 0) break;
            n += (size_t)m;
        }
        if (carry == 0 && !(n >= 3 && buf[0] == 0 && buf[1] == 0 && (buf[2] == 1 || (n >= 4 && buf[2] == 0 && buf[3] == 1))))
            st_tails++;                                         /* continuation of an already-sent NAL */

        /* split into NALs */
        int cnt = 0, au_has_slice = 0;
        size_t first_nal_off = 0;
        ssize_t s = find_start(buf, n, 0);
        while (s >= 0 && cnt < MAX_NALS) {
            ssize_t next = find_start(buf, n, (size_t)s);
            size_t end = next >= 0 ? (size_t)next - 3 : n;
            while (end > (size_t)s && buf[end - 1] == 0 && next >= 0) end--;   /* 4-byte start codes */
            if (end > (size_t)s) {
                const uint8_t *nal = buf + s; size_t len = end - (size_t)s;
                int t = nal[0] & 0x1F;
                if (t == 7) { free(sps); sps = malloc(len); memcpy(sps, nal, len); sps_len = len; }
                else if (t == 8) { free(pps); pps = malloc(len); memcpy(pps, nal, len); pps_len = len; }
                if (is_new_au(nal, len, au_has_slice)) {
                    emit_au(nals, cnt); st_splits++;
                    cnt = 0; au_has_slice = 0; first_nal_off = (size_t)s - 3 - (buf[s - 4] == 0 && s >= 4 ? 1 : 0);
                }
                if (cnt == 0) first_nal_off = (size_t)s - 3 - ((s >= 4 && buf[s - 4] == 0) ? 1 : 0);
                nals[cnt].p = nal; nals[cnt].len = len; cnt++;
                if (t == 1 || t == 5) au_has_slice = 1;
            }
            s = next;
        }
        if (au_has_slice) { emit_au(nals, cnt); carry = 0; }
        else if (cnt > 0) {                                     /* headers only: keep for the next frame */
            size_t keep = n - first_nal_off;
            if (keep < (1u << 16)) { memmove(buf, buf + first_nal_off, keep); carry = keep; } else carry = 0;
        } else carry = 0;

        if (stats_s > 0) {
            struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
            double dt = (now.tv_sec - t_stats.tv_sec) + (now.tv_nsec - t_stats.tv_nsec) / 1e9;
            if (dt >= stats_s) {
                fprintf(stderr, "rtp_push %s:%d  %.1f fps  %.0f pkt/s  %.0f kbit/s  au-splits %llu  split-tails %llu\n",
                        host, port, st_frames / dt, st_packets / dt, st_bytes * 8 / dt / 1000,
                        (unsigned long long)st_splits, (unsigned long long)st_tails);
                st_frames = st_packets = st_bytes = 0; t_stats = now;
            }
        }
    }
    return 0;
}
