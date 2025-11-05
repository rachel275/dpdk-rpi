// app/matrix-rx/matrix_rx.c
#include <arpa/inet.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <rte_cycles.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_ether.h>
#include <rte_mbuf.h>
#include <rte_prefetch.h>

#define RX_DESC 256
#define TX_DESC 256
#define BURST   32

// Use the same custom Ethertype as the sender
#define ETHERTYPE_MATRIX 0x88BD

// How far ahead to prefetch packet data
#define PREFETCH_OFFSET 4

#define MARK(tag)                                                                              \
    do {                                                                                       \
        fprintf(stderr, "APP:%s\n", tag);                                                      \
        fflush(stderr);                                                                         \
    } while (0)

static uint16_t g_port_id = 0;
static struct rte_ether_addr g_my_mac;

/* ---------- mempool warm-up ---------- */
static void warmup_mempool(struct rte_mempool *mp, unsigned total)
{
    enum { STEP = 256 };
    struct rte_mbuf *batch[STEP];

    unsigned left = total;
    while (left) {
        unsigned n = left > STEP ? STEP : left;
        if (rte_pktmbuf_alloc_bulk(mp, batch, n) == 0) {
            for (unsigned i = 0; i < n; i++) {
                char *d = rte_pktmbuf_mtod(batch[i], char *);
                d[0] = 0; /* touch first cacheline */
            }
            for (unsigned i = 0; i < n; i++)
                rte_pktmbuf_free(batch[i]);
        }
        left -= n;
    }
}

/* ---------- TX: send an ACK back to source ---------- */
static inline void send_ack(uint64_t counter, const struct rte_ether_addr *dst, struct rte_mempool *mp)
{
    struct rte_mbuf *m = rte_pktmbuf_alloc(mp);
    if (unlikely(!m))
        return;

    const uint16_t pay  = 8; /* "MXOK"(4) + counter(4) */
    const uint16_t need = sizeof(struct rte_ether_hdr) + pay;

    if (!rte_pktmbuf_append(m, need)) {
        rte_pktmbuf_free(m);
        return;
    }

    struct rte_ether_hdr *eth = rte_pktmbuf_mtod(m, struct rte_ether_hdr *);
    rte_ether_addr_copy(dst, &eth->dst_addr);
    rte_ether_addr_copy(&g_my_mac, &eth->src_addr);
    eth->ether_type = htons(ETHERTYPE_MATRIX);

    uint8_t *p = (uint8_t *)(eth + 1);
    memcpy(p, "MXOK", 4);
    uint32_t c = htonl((uint32_t)(counter & 0xffffffffu));
    memcpy(p + 4, &c, 4);

    struct rte_mbuf *txpkts[1] = { m };
    if (rte_eth_tx_burst(g_port_id, 0, txpkts, 1) != 1)
        rte_pktmbuf_free(m);
}

/* ---------- RX: parse matrix frames ---------- */
static int handle_pkt(struct rte_mbuf *m, struct rte_mempool *mp, uint64_t *seen)
{
    struct rte_ether_hdr *eth = rte_pktmbuf_mtod(m, struct rte_ether_hdr *);
    if (ntohs(eth->ether_type) != ETHERTYPE_MATRIX)
        return 0;

    uint8_t *p = (uint8_t *)(eth + 1);
    const uint32_t need_hdr = 12; // "MX01"(4) + rows(2) + cols(2) + type(1) + pad(3)
    const uint32_t len      = rte_pktmbuf_pkt_len(m);

    if (len < sizeof(*eth) + need_hdr)
        return 0;
    if (memcmp(p, "MX01", 4) != 0)
        return 0;

    uint16_t rows = (uint16_t)((p[4] << 8) | p[5]);
    uint16_t cols = (uint16_t)((p[6] << 8) | p[7]);
    uint8_t  et   = p[8]; // 1=float32
    uint8_t *data = p + need_hdr;

    uint32_t el_sz = (et == 1) ? 4u : 0u;
    if (!el_sz)
        return 0;

    uint32_t need = (uint32_t)rows * cols * el_sz;
    if (len < sizeof(*eth) + need_hdr + need)
        return 0;

    float f0 = 0.f, f1 = 0.f;
    if (et == 1 && cols >= 2) {
        memcpy(&f0, data + 0, 4);
        memcpy(&f1, data + 4, 4);
    }
    if (((*seen) & 0x3F) == 0) {
        printf("Matrix %ux%u (type=%u): first two elems = %.3f, %.3f\n",
               rows, cols, et, f0, f1);
    }

    /* increment counter and ACK back to sender MAC */
    (*seen)++;
    send_ack(*seen, &eth->src_addr, mp);
    return 1;
}

int main(int argc, char **argv)
{
    MARK("start");
    int ret = rte_eal_init(argc, argv);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "EAL init failed\n");
    argc -= ret;
    argv += ret;
    MARK("after_eal");

    /* make printf show up immediately */
    setvbuf(stdout, NULL, _IONBF, 0);

    printf(">>> APP: after EAL init (argc=%d)\n", argc);
    printf("ethdevs avail: %u\n", rte_eth_dev_count_avail());
    uint16_t pid;
    RTE_ETH_FOREACH_DEV(pid) {
        char name[RTE_ETH_NAME_MAX_LEN] = {0};
        rte_eth_dev_get_name_by_port(pid, name);
        printf("  port %u: %s\n", pid, name);
    }

    /* Try your expected vdev name first; fall back to AF_PACKET for Pi */
    if (rte_eth_dev_get_port_by_name("net_macb0", &g_port_id) != 0) {
        printf("No vdev named net_macb0; trying net_af_packet0...\n");
        if (rte_eth_dev_get_port_by_name("net_af_packet0", &g_port_id) != 0)
            rte_exit(EXIT_FAILURE, "No usable ethdev found\n");
    }
    printf("Using port %u\n", g_port_id);

    struct rte_eth_conf conf;
    memset(&conf, 0, sizeof(conf));
    conf.rxmode.mq_mode = RTE_ETH_MQ_RX_NONE;
    conf.txmode.mq_mode = RTE_ETH_MQ_TX_NONE;

    /* Ask PMD what it supports */
    struct rte_eth_dev_info di;
    if (rte_eth_dev_info_get(g_port_id, &di) == 0) {
        printf("Port %u driver=%s max_rxq=%u max_txq=%u\n",
               g_port_id, di.driver_name ? di.driver_name : "?",
               di.max_rx_queues, di.max_tx_queues);
    }
    MARK("before_dev_configure");

    /* Start with our preferred ring sizes, let PMD adjust */
    uint16_t nb_rx_desc = RX_DESC;
    uint16_t nb_tx_desc = TX_DESC;

    if (rte_eth_dev_configure(g_port_id, 1, 1, &conf) < 0)
        rte_exit(EXIT_FAILURE, "dev_configure failed\n");
    MARK("after_dev_configure");

    /* Let the PMD clamp/align descriptor counts */
    if (rte_eth_dev_adjust_nb_rx_tx_desc(g_port_id, &nb_rx_desc, &nb_tx_desc) == 0) {
        if (nb_rx_desc != RX_DESC || nb_tx_desc != TX_DESC) {
            printf("Adjusted desc: RX %u->%u, TX %u->%u\n",
                   (unsigned)RX_DESC, (unsigned)nb_rx_desc,
                   (unsigned)TX_DESC, (unsigned)nb_tx_desc);
        }
    }

    /* Mempool */
    int socket = rte_eth_dev_socket_id(g_port_id);
    if (socket < 0)
        socket = rte_socket_id();

    MARK("before_mempool");
    struct rte_mempool *mp = rte_pktmbuf_pool_create("mp", 8192, 512, 0, 2048, socket);
    if (!mp)
        rte_exit(EXIT_FAILURE, "mempool create failed\n");
    MARK("after_mempool");

    warmup_mempool(mp, 8192);

    /* RX queue 0 */
    struct rte_eth_rxconf rxq_conf;
    memset(&rxq_conf, 0, sizeof(rxq_conf));
    MARK("before_rxq");
    if (rte_eth_rx_queue_setup(g_port_id, 0, nb_rx_desc, socket, &rxq_conf, mp) < 0)
        rte_exit(EXIT_FAILURE, "rxq setup failed\n");
    MARK("after_rxq");

    /* TX queue 0 */
    struct rte_eth_txconf txq_conf;
    memset(&txq_conf, 0, sizeof(txq_conf));
    MARK("before_txq");
    if (rte_eth_tx_queue_setup(g_port_id, 0, nb_tx_desc, socket, &txq_conf) < 0)
        rte_exit(EXIT_FAILURE, "txq setup failed\n");
    MARK("after_txq");

    /* Start device */
    MARK("before_dev_start");
    if (rte_eth_dev_start(g_port_id) < 0)
        rte_exit(EXIT_FAILURE, "dev_start failed\n");
    MARK("after_dev_start");

    /* Promisc + MAC */
    if (rte_eth_promiscuous_enable(g_port_id) != 0)
        fprintf(stderr, "warn: promiscuous_enable failed\n");
    if (rte_eth_macaddr_get(g_port_id, &g_my_mac) != 0)
        memset(&g_my_mac, 0, sizeof(g_my_mac));

    fprintf(stderr,
            "APP:ready MAC %02x:%02x:%02x:%02x:%02x:%02x (Ethertype 0x%04x) on port %u\n",
            g_my_mac.addr_bytes[0], g_my_mac.addr_bytes[1], g_my_mac.addr_bytes[2],
            g_my_mac.addr_bytes[3], g_my_mac.addr_bytes[4], g_my_mac.addr_bytes[5],
            ETHERTYPE_MATRIX, g_port_id);

    uint64_t seen = 0;
    uint64_t last_stats = rte_get_timer_cycles();
    const uint64_t stats_every = rte_get_timer_hz(); // ~1 second

    for (;;) {
        struct rte_mbuf *pkts[BURST];
        const uint16_t n = rte_eth_rx_burst(g_port_id, 0, pkts, BURST);
        if (n == 0) {
            rte_pause();
            goto do_stats;
        }

        /* Prefetch first few */
        const uint16_t pre = (n > PREFETCH_OFFSET) ? PREFETCH_OFFSET : n;
        for (uint16_t i = 0; i < pre; i++)
            rte_prefetch0(rte_pktmbuf_mtod(pkts[i], void *));

        /* Rolling prefetch + process */
        for (uint16_t i = 0; i + PREFETCH_OFFSET < n; i++) {
            rte_prefetch0(rte_pktmbuf_mtod(pkts[i + PREFETCH_OFFSET], void *));
            (void)handle_pkt(pkts[i], mp, &seen);
            rte_pktmbuf_free(pkts[i]);
        }
        for (uint16_t i = (n > PREFETCH_OFFSET ? n - PREFETCH_OFFSET : 0); i < n; i++) {
            (void)handle_pkt(pkts[i], mp, &seen);
            rte_pktmbuf_free(pkts[i]);
        }

    do_stats:
        /* ~1 Hz stats */
        const uint64_t now = rte_get_timer_cycles();
        if (now - last_stats >= stats_every) {
            struct rte_eth_stats st;
            if (rte_eth_stats_get(g_port_id, &st) == 0) {
                printf("[stats] RX=%" PRIu64 " TX=%" PRIu64
                       "  RX-err=%" PRIu64 "  miss=%" PRIu64
                       "  no_mbuf=%" PRIu64 "  seen=%" PRIu64 "\n",
                       st.ipackets, st.opackets, st.ierrors, st.imissed, st.rx_nombuf, seen);
            }
            last_stats = now;
        }
    }
}

