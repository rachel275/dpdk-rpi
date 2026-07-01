// Logging control: export MACB_RXTX_TRACE=0..3   (default: 1)

#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h> /* getenv, atoi */

#include <rte_branch_prediction.h>
#include <rte_byteorder.h>
#include <rte_io.h>
#include <rte_log.h>
#include <rte_mbuf.h>

#include "macb_hw.h"
#include "dma_sync_user.h"

//#include "dma_sync_uapi.h"   /* userspace DMA sync helper ioctl API */
/* Ensure logtype exists for RTE_LOG(..., PMD, ...) */
#ifndef RTE_LOGTYPE_PMD
#define RTE_LOGTYPE_PMD RTE_LOGTYPE_USER1
#endif

#define MACB_LOGI(fmt, ...) RTE_LOG(INFO, PMD, "macb: " fmt, ##__VA_ARGS__)
#define MACB_LOGE(fmt, ...) RTE_LOG(ERR, PMD, "macb: " fmt, ##__VA_ARGS__)
#define MACB_LOGD(fmt, ...) RTE_LOG(DEBUG, PMD, "macb: " fmt, ##__VA_ARGS__)

/* ================= RX bits (MACB/GEM) =================
 * ADDR.bit0 = USED/OWN (0 = owned by HW / armed, 1 = SW owns / frame completed)
 * ADDR.bit1 = WRAP (end of ring)
 * CTRL: SOF/EOF + length (valid when SW owns)
 */
#ifndef RX_SOF
#define RX_SOF (1u << 14)
#endif
#ifndef RX_EOF
#define RX_EOF (1u << 15)
#endif
#ifndef RX_LEN_MASK
#define RX_LEN_MASK 0x1FFFu
#endif
#ifndef RX_WRAP
#define RX_WRAP (1u << 1) /* ADDR bit1 */
#endif
#ifndef RX_USED
#define RX_USED (1u << 0) /* ADDR bit0: 1=SW owns (complete) */
#endif
#define RX_OWN RX_USED /* alias to match your comments */

/* ================ TX bits (GEM standard layout) =================
 * CTRL.bit31 = USED (1 = FREE/owned by SW, 0 = BUSY/owned by HW)
 * CTRL.bit30 = WRAP
 * CTRL.bit15 = LAST
 * CTRL.[13:0] = length
 */

#define TX_USED     (1u << 14)   // 1 = free/SW owns
//#define TX_WRAP     (1u << 30)
#define TX_LAST     (1u << 15)
#define TX_LEN_MASK 0x07FFu //on older MACB

//#define TX_LEN_MASK 0x3FFFu     // or 0x07FFu on older MACB
//#define TX_USED 0x00004000u
//#define TX_WRAP 0x00004000u
//#define TX_LAST 0x00004000u

#ifndef NCR_TSTART
#define NCR_TSTART (1u << 9)
#endif

#ifndef RTE_LOGTYPE_PMD
#define RTE_LOGTYPE_PMD RTE_LOGTYPE_USER1
#endif
#define MACB_DBG(fmt, ...) RTE_LOG(INFO, PMD, "macb: " fmt, ##__VA_ARGS__)

#define MACB_RXTX_TRACE 2

/* ================= DMA sync wrappers (volatile-safe) ================= */
static inline void sync_desc_to_dev(int fd, const volatile void *p, size_t len) {
    if (fd >= 0) dma_sync_to_dev(fd, (void *)(uintptr_t)p, len);
}

static inline void sync_desc_from_dev(int fd, const volatile void *p, size_t len) {
    if (fd >= 0) dma_sync_from_dev(fd, (void *)(uintptr_t)p, len);
}

static inline void sync_buf_to_dev(int fd, const void *p, size_t len) {
    if (fd >= 0 && len) dma_sync_to_dev(fd, (void *)(uintptr_t)p, len);
}

static inline void sync_buf_from_dev(int fd, const void *p, size_t len) {
    if (fd >= 0 && len) dma_sync_from_dev(fd, (void *)(uintptr_t)p, len);
}

/* ================= Globals from ethdev ================= */
extern uint64_t g_bus_ofs; /* CPU-phys -> device bus offset */
#define BUS_IOVA(x) ((rte_iova_t)((rte_iova_t)(x) + (rte_iova_t)g_bus_ofs))

struct macb_txq;
__attribute__((weak)) void macb_dump_regs_txq(struct macb_txq *txq, const char *tag) {
    RTE_SET_USED(txq);
    RTE_SET_USED(tag);
}


/* === lightweight SW stats helpers === */
static inline void macb_sw_tx_add(struct macb_adapter *ad, uint32_t n, uint32_t bytes_per_pkt) {
    ad->sw.tx_pkts += n;
    ad->sw.tx_bytes += (uint64_t)n * (uint64_t)bytes_per_pkt;
}
static inline void macb_sw_rx_add(struct macb_adapter *ad, uint32_t n, uint32_t bytes_total) {
    ad->sw.rx_pkts += n;
    ad->sw.rx_bytes += (uint64_t)bytes_total;
}

static inline void macb_sync_rx_desc_from_dev(struct macb_rxq *rxq, uint16_t i) {
    sync_desc_from_dev(rxq->sync_fd, &rxq->ring[i], sizeof(rxq->ring[i]));
}

/* === RX OWN/USED polarity (1 = SW owns/completed) ======================== */
#ifndef MACB_RX_USED_IS_HW_OWNED
#define MACB_RX_USED_IS_HW_OWNED 0 /* 0: bit0==1 means SW owns (typical MACB) */
#endif

static inline int rx_hw_owns(uint32_t daddr_raw) {
    return (daddr_raw & RX_USED) == 0; // 1 when HW owns (not ready)
}

void macb_rx_probe_once(struct macb_rxq *rxq) {
    struct macb_adapter *ad = rxq->ad;
#ifdef MACB_RSR
    uint32_t rsr = macb_readl(&ad->hw, MACB_RSR);
#else
    uint32_t rsr = 0;
#endif

    for (int i = 0; i < 2 && i < rxq->nb_desc; ++i) {
        /* fresh view of descriptor i */
        volatile struct macb_desc *dv = &rxq->ring[i];
        sync_desc_from_dev(rxq->sync_fd, dv, sizeof(*dv));
        rte_io_mb();

        uint32_t addr = rxq->ring[i].addr;
        uint32_t ctrl = rxq->ring[i].ctrl;
        int own_hw = rx_hw_owns(addr);

        RTE_LOG(INFO, PMD,
                "macb: RX[probe] d%03d: addr=%08x ctrl=%08x OWN=%s WRAP=%d len=%u sof=%d eof=%d\n",
                i, addr, ctrl, own_hw ? "HW" : "SW", !!(addr & RX_WRAP), (ctrl & RX_LEN_MASK),
                !!(ctrl & RX_SOF), !!(ctrl & RX_EOF));
    }

    MACB_DBG("RX[probe] RSR=%08x (REC=%d BNA=%d OVR=%d)\n", rsr, !!(rsr & 0x1), !!(rsr & 0x2),
             !!(rsr & 0x4));
}

void macb_log_regs_full(struct macb_adapter *ad, const char *tag) {
    uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
    uint32_t ncfgr = macb_readl(&ad->hw, MACB_NCFGR);
    uint32_t nsr = macb_readl(&ad->hw, MACB_NSR);
#ifdef MACB_TSR
    uint32_t tsr = macb_readl(&ad->hw, MACB_TSR);
#endif
    uint32_t rbqph = 0, rbqp = macb_readl(&ad->hw, MACB_RBQP);
    uint32_t tbqph = 0, tbqp = macb_readl(&ad->hw, MACB_TBQP);
#ifdef MACB_RBQPH
    rbqph = macb_readl(&ad->hw, MACB_RBQPH);
#endif
#ifdef MACB_TBQPH
    tbqph = macb_readl(&ad->hw, MACB_TBQPH);
#endif
#ifdef GEM_DMACFG
    uint32_t dmacfg = macb_readl(&ad->hw, GEM_DMACFG);
#else
    uint32_t dmacfg = 0xFFFFFFFF;
#endif

    RTE_LOG(INFO, PMD,
            "macb: [%s] NCR=0x%08x NCFGR=0x%08x NSR=0x%08x TSR=0x%08x RBQP=%08x:%08x TBQP=%08x:%08x DMACFG=0x%08x\n",
            tag, ncr, ncfgr, nsr,
#ifdef MACB_TSR
            tsr,
#else
            0u,
#endif
            rbqph, rbqp, tbqph, tbqp, dmacfg);
}

/* ================== tracing helpers ================== */
#ifndef MACB_RXTX_TRACE_DEFAULT
#define MACB_RXTX_TRACE_DEFAULT 1
#endif
static int macb_trace_level = MACB_RXTX_TRACE_DEFAULT;
static inline void macb_trace_init_once(void) {
    static int inited;
    if (!inited) {
        const char *e = getenv("MACB_RXTX_TRACE");
        macb_trace_level = (e && *e) ? atoi(e) : MACB_RXTX_TRACE_DEFAULT;
        inited = 1;
    }
}
#define RT_LVL(lv, fmt, ...)                                                                 \
    do {                                                                                     \
        macb_trace_init_once();                                                               \
        if (macb_trace_level >= (lv)) RTE_LOG(INFO, EAL, "macb[rxtx]: " fmt, ##__VA_ARGS__); \
    } while (0)
#define RTI(fmt, ...) RT_LVL(1, fmt, ##__VA_ARGS__)
#define RTV(fmt, ...) RT_LVL(2, fmt, ##__VA_ARGS__)
#define RTD(fmt, ...) RT_LVL(3, fmt, ##__VA_ARGS__)

/* --- lightweight throttle so RX hot path doesn't spam --- */
static inline int macb_tick(unsigned step) {
    static uint32_t t;
    return ((t++ & (step - 1)) == 0); /* step must be power of 2, e.g. 0xFF */
}

/* assumes RX_WRAP is bit1 and RX_USED is bit0 in the address word */
static inline uint32_t rx_arm_addr_word(uint32_t iova, uint32_t wrap) {
    uint32_t w = (iova & ~0x3u); /* strip low 2 control bits */
    if (wrap) w |= RX_WRAP;      /* keep WRAP (bit1) if needed */
    /* DO NOT set RX_USED here; USED==0 hands ownership to HW */
    return w;
}

static inline uint32_t rx_sw_addr_word_hold(uint32_t prev_addr) {
    /* keep WRAP, force SW ownership (opposite of rx_arm_addr_word) */
    uint32_t aw = prev_addr & ~RX_USED;
    if (!MACB_RX_USED_IS_HW_OWNED) aw |= RX_USED; /* bit0=1 → SW owns (when HW owns on 0) */
    return aw;
}

/* Dump 'count' RX descriptors starting at rxq->cons with a custom tag. */
static inline void macb_dump_rx_window(struct macb_rxq *rxq, const char *tag, uint16_t count) {
    uint16_t idx = rxq->cons;
    uint16_t n = (count > rxq->nb_desc) ? rxq->nb_desc : count;

    for (uint16_t i = 0; i < n; i++) {
        volatile struct macb_desc *dv = &rxq->ring[idx];
        struct macb_desc d = (struct macb_desc){0};
        sync_desc_from_dev(rxq->sync_fd, dv, sizeof(d));
        memcpy(&d, (const void *)dv, sizeof(d));

        int own_hw = rx_hw_owns(d.addr);
        RTE_LOG(INFO, PMD,
                "macb: RX[%s] d%03u: addr=0x%08x ctrl=0x%08x OWN=%s WRAP=%d LEN=%u SOF=%d EOF=%d\n",
                tag, idx, d.addr, d.ctrl, own_hw ? "HW" : "SW", !!(d.addr & RX_WRAP),
                (d.ctrl & RX_LEN_MASK), !!(d.ctrl & RX_SOF), !!(d.ctrl & RX_EOF));

        if (++idx == rxq->nb_desc) idx = 0;
    }
}

/* ================= RX init ================= */
void macb_rx_init(struct macb_rxq *rxq) {
    const uint16_t nb = rxq->nb_desc;
    uint16_t armed = 0, held = 0;

    for (uint16_t i = 0; i < nb; i++) {
        struct macb_desc *d = &rxq->ring[i];
        const uint32_t wrap = (i == nb - 1) ? RX_WRAP : 0;

        if (rxq->sw_ring[i]) {
            rte_pktmbuf_free(rxq->sw_ring[i]);
            rxq->sw_ring[i] = NULL;
        }

        struct rte_mbuf *m = rte_pktmbuf_alloc(rxq->mp);
        if (likely(m)) {
            rte_iova_t bi = rte_mbuf_data_iova(m);
            uint32_t da = (uint32_t)BUS_IOVA(bi);

            d->ctrl = 0;
            d->addr = rx_arm_addr_word(da, wrap); /* arm for HW (proper polarity) */
            rxq->sw_ring[i] = m;
            armed++;
        } else {
            d->ctrl = 0;
            d->addr = rx_sw_addr_word_hold((d->addr & ~0x3u) | wrap); /* SW holds */
            held++;
        }
        sync_desc_to_dev(rxq->sync_fd, d, sizeof(*d));
        /* after: d->addr = (da & ~0x3u) | wrap; and sync_desc_to_dev(...) */
        {
            /* Read back what the device will see */
            struct macb_desc chk = {0};
            sync_desc_from_dev(rxq->sync_fd, d, sizeof(chk));
            memcpy(&chk, (const void *)d, sizeof(chk));

            if (chk.addr & RX_USED) {
                /* This should never be set when arming! Trace and auto-heal. */
                RTE_LOG(WARNING, PMD,
                        "macb: RX[init-fix] d%03u had USED=1 after arm (addr=%08x). Forcing USED=0.\n",
                        i, chk.addr);
                uint32_t addr_fix = chk.addr & ~RX_USED;
                /* write back in place */
                d->addr = addr_fix;
                rte_wmb();
                sync_desc_to_dev(rxq->sync_fd, d, sizeof(*d));
                rte_io_wmb();
            }

            RTE_LOG(INFO, PMD, "macb: RX[init-verify] d%03u armed -> addr=%08x (USED=%u WRAP=%u)\n",
                    i, d->addr, !!(d->addr & RX_USED), !!(d->addr & RX_WRAP));
        }
    }

    if (rxq->sync_fd >= 0) sync_desc_to_dev(rxq->sync_fd, rxq->ring, nb * sizeof(struct macb_desc));
    rte_io_wmb();

    rxq->cons = 0;
    MACB_DBG("RX init complete: armed(HW)=%u held(SW)=%u / %u\n", armed, held, nb);
    {
        volatile struct macb_desc *dv = &rxq->ring[0];
        sync_desc_from_dev(rxq->sync_fd, dv, sizeof(*dv));
        uint32_t a = dv->addr;
        if (!rx_hw_owns(a)) {
            RTE_LOG(WARNING, PMD,
                    "macb: RX sanity: d000 OWN=%s (expected HW). If this is bring-up, check MACB_RX_USED_IS_HW_OWNED define.\n",
                    rx_hw_owns(a) ? "HW" : "SW");
        }
    }
}
static inline int macb_rx_desc_ready(const volatile struct macb_desc *dv) {
    return (dv->addr & RX_USED) != 0; /* 1 == ready for SW */
}

uint16_t macb_rx_burst(void *queue, struct rte_mbuf **rx_pkts, uint16_t nb_pkts) {
    struct macb_rxq *rxq = (struct macb_rxq *)queue;
    uint16_t nb = 0;
    rte_iova_t mp_min = RTE_BAD_IOVA, mp_max = 0;

    /* Early exit if no packets are expected */
    if (unlikely(nb_pkts == 0)) return 0;

    while (nb < nb_pkts) {
        const uint16_t i = rxq->cons;
        volatile struct macb_desc *dv = &rxq->ring[i];

        /* Synchronize descriptor with device */
        sync_desc_from_dev(rxq->sync_fd, (const void *)dv, sizeof(*dv));
        rte_io_mb();

        uint32_t w0 = ((volatile uint32_t *)dv)[0];
        uint32_t w1 = ((volatile uint32_t *)dv)[1];

        /* --- Autodetect layout once: which word is the address? ----------- */
        static int rx_layout = -1;
        if (rx_layout < 0) {
            int w0_in = (w0 >= mp_min && w0 < mp_max);
            int w1_in = (w1 >= mp_min && w1 < mp_max);
            if (w0_in && !w1_in)
                rx_layout = 0; /* w0=addr, w1=status */
            else if (!w0_in && w1_in)
                rx_layout = 1; /* w0=status, w1=addr */
            else
                rx_layout = 0;
        }

        uint32_t daddr_raw = (rx_layout == 1) ? w1 : w0;
        uint32_t dstat = (rx_layout == 1) ? w0 : w1;
        uint32_t daddr_iova = daddr_raw & ~0x3u;

        /* If no valid frame, skip descriptor */
        if (unlikely((dstat & RX_SOF) == 0 || (dstat & RX_EOF) == 0 || !(dstat & RX_USED))) {
            RTE_LOG(DEBUG, PMD,
                    "RX[%u] incomplete frame: stat=0x%08x (sof=%u eof=%u)\n", i, dstat,
                    !!(dstat & RX_SOF), !!(dstat & RX_EOF));
            break;
        }

        uint16_t len = (uint16_t)(dstat & RX_LEN_MASK);
        if (unlikely(len == 0)) {
            RTE_LOG(DEBUG, PMD, "RX[%u] zero length frame\n", i);
            break;
        }

        struct rte_mbuf *m = rxq->sw_ring[i];
        if (unlikely(!m)) {
            RTE_LOG(WARNING, PMD, "RX[%u] no mbuf available, skipping\n", i);
            break;
        }

        /* Invalidate the payload before CPU access */
        sync_buf_from_dev(rxq->sync_fd, rte_pktmbuf_mtod(m, void *), len);
        rte_io_mb();

        m->data_off = RTE_PKTMBUF_HEADROOM;
        m->pkt_len = len;
        m->data_len = len;
        m->port = rxq->port_id;

        /* Store the received packet */
        rx_pkts[nb++] = m;
        rxq->sw_ring[i] = NULL;
        RTE_LOG(DEBUG, PMD, "RX[%u] frame received (len=%u)\n", i, len);

        /* Re-arm descriptor with a fresh mbuf for the hardware */
        struct rte_mbuf *nm = rte_pktmbuf_alloc(rxq->mp);
        if (!nm) {
            RTE_LOG(WARNING, PMD, "RX[%u] no buffer available to re-arm\n", i);
            break;
        }
        rxq->sw_ring[i] = nm;

        rte_iova_t di = rte_mbuf_data_iova(nm);
        uint32_t addr_word =
            rx_arm_addr_word((uint32_t)BUS_IOVA(di), (i == rxq->nb_desc - 1) ? RX_WRAP : 0);

        /* Invalidate the buffer before giving it back to NIC */
        sync_buf_to_dev(rxq->sync_fd, nm->buf_addr, nm->buf_len);
        rte_io_mb();

        if (rx_layout == 1) {
            ((volatile uint32_t *)dv)[0] = 0;          /* w0=status */
            ((volatile uint32_t *)dv)[1] = addr_word;  /* w1=addr */
        } else {
            ((volatile uint32_t *)dv)[0] = addr_word;  /* w0=addr */
            ((volatile uint32_t *)dv)[1] = 0;          /* w1=status */
        }

        rte_wmb();
        sync_desc_to_dev(rxq->sync_fd, (const void *)dv, sizeof(*dv));
        rte_io_wmb();

        RTE_LOG(DEBUG, PMD, "RX[%u] descriptor re-armed with new mbuf\n", i);
    }

    return nb;
}

/* ================= TX reclaim ================= */
static inline uint16_t macb_tx_reclaim(struct macb_txq *txq) {
    uint16_t reclaimed = 0;

    while (reclaimed < txq->nb_desc) {
        struct macb_desc *d = &txq->ring[txq->cons];

        /* Fresh view of descriptor */
        sync_desc_from_dev(txq->sync_fd, d, sizeof(*d));

        /* Only reclaim entries we actually queued earlier */
        if (txq->sw_ring[txq->cons] == NULL) break;

        /* HW done => CTRL.USED == 1 */
        if ((d->ctrl & TX_USED) == 0) break;

        rte_pktmbuf_free(txq->sw_ring[txq->cons]);
        txq->sw_ring[txq->cons] = NULL;

        /* Reset FREE descriptor (USED=1, preserve WRAP), zero addr */
        const uint32_t wrap = (d->ctrl); // & TX_WRAP);
        d->addr = 0;
        d->ctrl = (TX_USED | wrap);

        sync_desc_to_dev(txq->sync_fd, d, sizeof(*d));

        txq->cons = (uint16_t)((txq->cons + 1) % txq->nb_desc);
        reclaimed++;
    }

    if (reclaimed) RTI("TX reclaim: %u descriptors (cons=%u)\n", reclaimed, txq->cons);

    return reclaimed;
}

/* ---- TX burst: single-queue, single-seg mbufs ---- */
uint16_t macb_tx_burst(void *tx_queue, struct rte_mbuf **tx_pkts, uint16_t nb_pkts) {
    struct macb_txq *txq = (struct macb_txq *)tx_queue;
    struct macb_adapter *ad = txq->ad;
    const uint16_t nb_desc = txq->nb_desc;
    uint16_t sent = 0;

    if (nb_pkts == 0 || nb_desc == 0) return 0;

    /* Make cons/prod visible to both reclaim and free-space math */
    uint16_t cons = txq->cons;
    uint16_t prod = txq->prod;

    /* -------- reclaim completed descriptors -------- */
    while (cons != prod) {
        volatile struct macb_desc *dv = &txq->ring[cons];
        struct macb_desc *d = (struct macb_desc *)(uintptr_t)dv;

        sync_desc_from_dev(txq->sync_fd, d, sizeof(*d));
        uint32_t ctrl = d->ctrl;
	bool done = (ctrl & TX_USED) != 0;
        RTE_LOG(INFO, PMD, "macb TX[reclaim] ctrl=0x%08x, TXUSED=%d \n", ctrl, (int)done);
        if ((ctrl & TX_USED) == 0) break;

        if (txq->sw_ring[cons]) {
            rte_pktmbuf_free(txq->sw_ring[cons]);
            txq->sw_ring[cons] = NULL;
        }

        /* keep WRAP; mark FREE */
        d->ctrl = (ctrl) | TX_USED; // & TX_WRAP) | TX_USED;
        sync_desc_to_dev(txq->sync_fd, d, sizeof(*d));

#if MACB_RXTX_TRACE >= 2
        RTE_LOG(INFO, PMD, "macb: TX[reclaim] d%03u: ctrl=0x%08x (USED=1 WRAP=%d)\n", cons, ctrl,
                !!(ctrl)); // & TX_WRAP));
#endif
        if (++cons == nb_desc) cons = 0;
    }
    txq->cons = cons;

    /* -------- compute free space -------- */
    uint16_t used = (uint16_t)((prod - cons) & (nb_desc - 1));
    uint16_t free = (uint16_t)(nb_desc - 1 - used);
    if (free == 0) return 0;

    if (nb_pkts > free) nb_pkts = free;

    uint16_t first = prod;

    /* -------- enqueue & publish -------- */
    for (; sent < nb_pkts; sent++) {
        struct rte_mbuf *m = tx_pkts[sent];
        const uint16_t len = (uint16_t)m->pkt_len;
        rte_iova_t biova = rte_mbuf_data_iova(m);

        volatile struct macb_desc *dv = &txq->ring[prod];
        struct macb_desc *d = (struct macb_desc *)(uintptr_t)dv;

        d->addr = (uint32_t)BUS_IOVA(biova);
	
	rte_wmb();
	rte_io_wmb();


        uint32_t ctrl = ((uint32_t)len & TX_LEN_MASK) | TX_LAST;
        if (prod == (nb_desc - 1)) ctrl; // |= TX_WRAP;
        /* Hand to HW: ensure USED bit (31) is 0 */
        d->ctrl = ctrl; /* bit31 implicitly 0 */

        txq->sw_ring[prod] = m;

        /* cache maintenance (non-coherent) */
        sync_buf_to_dev(txq->sync_fd, rte_pktmbuf_mtod(m, void *), len);
        sync_desc_to_dev(txq->sync_fd, d, sizeof(*d));
	
	rte_wmb();
	rte_io_wmb();

        RTE_LOG(INFO, PMD, "TX[publish] addr=0x%08x ctrl=0x%08x\n", d->addr, d->ctrl);

#if MACB_RXTX_TRACE >= 2
        struct macb_desc chk = {0};
        sync_desc_from_dev(txq->sync_fd, d, sizeof(*d));
        memcpy(&chk, (const void *)d, sizeof(chk));
        RTE_LOG(INFO, PMD, "macb: TX[pub-check] d%03u: addr=0x%08x ctrl=0x%08x (LEN=%u WRAP=%d)\n",
                prod, chk.addr, chk.ctrl, (chk.ctrl & TX_LEN_MASK), !!(chk.ctrl)); // & TX_WRAP));
#endif
        if (++prod == nb_desc) prod = 0;
    }

    /* publish all writes before kick */
    rte_io_wmb();

    /* kick TX (safe no-op if already running) */
    macb_writel(&ad->hw, MACB_NCR, macb_readl(&ad->hw, MACB_NCR) | NCR_TSTART);
    macb_dump_regs_txq(txq, "post-kick");
#if MACB_RXTX_TRACE >= 2
    if (sent) {
        uint16_t max_check = sent < 8 ? sent : 8;
        for (uint16_t i = 0; i < max_check; i++) {
            uint16_t idx = first + i;
            if (idx >= nb_desc) idx -= nb_desc;

            volatile struct macb_desc *dv = &txq->ring[idx];
            struct macb_desc *d = (struct macb_desc *)(uintptr_t)dv;
            struct macb_desc chk;

            sync_desc_from_dev(txq->sync_fd, d, sizeof(*d));
            memcpy(&chk, (const void *)d, sizeof(chk));

            RTE_LOG(INFO, PMD,
                    "macb: TX[pub-check] d%03u: addr=0x%08x ctrl=0x%08x (LEN=%u WRAP=%d)\n", idx,
                    chk.addr, chk.ctrl, (chk.ctrl & TX_LEN_MASK), !!(chk.ctrl)); // & TX_WRAP));
        }
    }
#endif

    txq->prod = prod;
    macb_sw_tx_add(ad, sent, 0); /* bytes accounted elsewhere if you want */
    return sent;
}

