// drivers/net/macb/macb_rxtx.c — RX/TX datapath for RP1/Cadence GEM
#include <string.h>

#include <rte_branch_prediction.h>
#include <rte_io.h>
#include <rte_log.h>
#include <rte_mbuf.h>
#include "macb_hw.h"
#include "macb_dma_sync.h"

/* Canonical RX descriptor layout: w0 = addr word, w1 = status/ctrl word */
#define RX_PICK_ADDR(w0, w1) (w0)
#define RX_PICK_STAT(w0, w1) (w1)

struct macb_txq; struct macb_rxq; struct macb_adapter;
struct macb_desc;

/* ---- TX descriptor helpers ---- */

static inline size_t
macb_tx_stride_bytes(const struct macb_txq *txq, unsigned cap)
{
    return txq->desc_stride ? (size_t)txq->desc_stride
                            : (size_t)macb_desc_stride_bytes(cap);
}

/* Return pointer to first word (w0) of descriptor idx */
static inline volatile uint32_t *
macb_tx_desc_words(void *ring, unsigned cap, uint16_t idx)
{
    volatile struct macb_desc *dv = (volatile struct macb_desc *)macb_desc_at(ring, cap, idx);
    return (volatile uint32_t *)dv; /* w0,w1,... */
}

static inline void
macb_tx_desc_sync_to(const struct macb_txq *txq, volatile void *dv, unsigned cap)
{
    sync_desc_to_dev_v(txq->sync_fd, dv, macb_tx_stride_bytes(txq, cap));
}

static inline void
macb_tx_desc_sync_from(const struct macb_txq *txq, volatile void *dv, unsigned cap)
{
    sync_desc_from_dev_v(txq->sync_fd, dv, macb_tx_stride_bytes(txq, cap));
}

/* ------------------------------------------------------------------------- */
/* Ring index helpers                                                        */
/* ------------------------------------------------------------------------- */
static inline uint16_t macb_ring_next(uint16_t i, uint16_t n)
{
    i++;
    return (i == n) ? 0 : i;
}

static inline uint16_t macb_ring_count(uint16_t head, uint16_t tail, uint16_t n)
{
    return (head >= tail) ? (uint16_t)(head - tail)
                          : (uint16_t)(n - tail + head);
}

/* Keep 1 slot empty to distinguish full vs empty */
static inline uint16_t macb_ring_space(uint16_t head, uint16_t tail, uint16_t n)
{
    uint16_t used = macb_ring_count(head, tail, n);
    /* usable slots = n - 1 - used */
    return (used >= (uint16_t)(n)) ? 0 : (uint16_t)(n- used);
}

void
macb_rx_publish_desc(struct macb_rxq *rxq,
                     volatile struct macb_desc *dv,
                     uint64_t bus_addr, uint32_t wrap)
{
    const unsigned cap = rxq->hw_dma_cap;

    /* CRITICAL: use the actual ring stride (24 on your setup) */
    const size_t stride = rxq->desc_stride ?
                          (size_t)rxq->desc_stride :
                          (size_t)macb_desc_stride_bytes(cap);

    volatile uint32_t *w = (volatile uint32_t *)dv;

    const uint32_t addr_lo = ((uint32_t)(bus_addr & ~0x3ull)) |
                             (wrap ? RX_WRAP : 0);
    const uint32_t addr_hi = (uint32_t)(bus_addr >> 32);

    /* 1) Clear the FULL descriptor (all 24 bytes) */
    for (size_t k = 0; k < (stride / 4); k++)
        w[k] = 0;

    /*
     * 2) If HW consumes addr_hi in the extended format, it MUST be correct.
     * With 32-bit bus, addr_hi must be 0.
     */
    if (stride >= 16) {
        /* In the common GEM extended RX desc, w2 is addr_hi. */
        w[2] = addr_hi;
    }

    /* 3) Publish order: STAT first, then ADDR last (w0=addr, w1=stat) */
    w[1] = 0;        /* STAT */
    rte_io_wmb();
    w[0] = addr_lo;  /* ADDR (USED=0 => HW owns) */

    /* 4) Sync FULL stride so device sees cleared w2..w5 */
    sync_desc_to_dev_v(rxq->sync_fd, dv, stride);
    rte_io_wmb();
}



static inline void
macb_rx_refill(struct macb_rxq *rxq)
{
    const uint16_t nb = rxq->nb_desc;
    const unsigned cap = rxq->hw_dma_cap;

    uint16_t space = macb_ring_space(rxq->rx_head, rxq->rx_tail, nb);
    if (space == 0)
        return;

    if (rxq->free_thresh && space < rxq->free_thresh)
        return;

    while (space) {
        const uint16_t i = rxq->rx_head;

        if (rxq->sw_ring[i] != NULL)
            break;

        struct rte_mbuf *m = rte_pktmbuf_alloc(rxq->mp);
        if (m == NULL)
            break;

        const rte_iova_t bi = rte_mbuf_data_iova(m);
        if (bi == RTE_BAD_IOVA) {
            rte_pktmbuf_free(m);
            break;
        }

        /* Clean buffer to RAM before handing to HW (non-coherent DMA) */
        void *data = rte_pktmbuf_mtod(m, void *);
        size_t room = rte_pktmbuf_data_room_size(rxq->mp) - RTE_PKTMBUF_HEADROOM;
        (void)sync_buf_to_dev_s(rxq->sync_fd, data, room < 128 ? room : 128);
        rte_io_wmb();

        const uint64_t bus  = (uint64_t)BUS_IOVA(bi);
        const uint32_t wrap = (i == (nb - 1)) ? RX_WRAP : 0;

        volatile struct macb_desc *dv = macb_desc_at(rxq->ring, cap, i);
        macb_rx_publish_desc(rxq, dv, bus, wrap);

        rxq->sw_ring[i] = m;
        rxq->rx_head = macb_ring_next(rxq->rx_head, nb);
        space--;
    }
}


static inline void macb_rx_program_rbqp(struct macb_rxq *rxq)
{
    struct macb_adapter *ad = rxq->ad;
    uint64_t base = rxq->ring_iova ? rxq->ring_iova
                                   : (rte_iova_t)(uintptr_t)rxq->ring;
    uint64_t bus  = BUS_IOVA(base);
    uint32_t lo   = (uint32_t)bus;
#ifdef MACB_RBQPH
    uint32_t hi   = (uint32_t)(bus >> 32);
    macb_writel(&ad->hw, MACB_RBQPH, hi);
#endif
    macb_writel(&ad->hw, MACB_RBQP, lo);
    rte_io_wmb();
}

/* ------------------------------ TX helpers -------------------------------- */
static inline void macb_tx_kick(struct macb_txq *txq)
{
    struct macb_adapter *ad = txq->ad;
    rte_wmb();
    uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
    ncr |= NCR_TXEN;    macb_writel(&ad->hw, MACB_NCR, ncr);
    ncr |= NCR_TSTART;  macb_writel(&ad->hw, MACB_NCR, ncr);
}



/* ------------------------------ TX reclaim -------------------------------- */
static inline uint16_t
macb_tx_reclaim(struct macb_txq *txq)
{
    const uint16_t nb    = txq->nb_desc;
    const uint16_t cons0 = txq->cons;
    const uint16_t prod0 = txq->prod;

    const uint16_t outstanding =
        (prod0 >= cons0) ? (uint16_t)(prod0 - cons0)
                         : (uint16_t)(nb - cons0 + prod0);

    const unsigned cap = txq->hw_dma_cap;
    const size_t stride = txq->desc_stride ? txq->desc_stride
                                      : (size_t)macb_desc_stride_bytes(cap);

    uint16_t reclaimed = 0;

    while (reclaimed < outstanding) {
        const uint16_t cons = txq->cons;

        volatile uint32_t *w = macb_tx_desc_words(txq->ring, cap, cons);
        macb_tx_desc_sync_from(txq, w, cap);
        const uint32_t ctrl = w[1];

        if ((ctrl & TX_USED) == 0)
            break;

        if (txq->sw_ring[cons]) {
            rte_pktmbuf_free(txq->sw_ring[cons]);
            txq->sw_ring[cons] = NULL;
        }

        const uint32_t wrap = (cons == (nb - 1)) ? TX_WRAP : 0;

        /* Clear whole slot so ext words don't retain stale garbage */
        memset((void *)(uintptr_t)w, 0, stride);
        w[0] = 0;
        w[1] = TX_USED | wrap;

        macb_tx_desc_sync_to(txq, w, cap);

        txq->cons = (uint16_t)((cons + 1) % nb);
        reclaimed++;
    }


    return reclaimed;
}



static inline int
macb_rx_rearm_slot_swap(struct macb_rxq *rxq, uint16_t i, struct rte_mbuf **out_old)
{
    const unsigned cap = rxq->hw_dma_cap;
    volatile struct macb_desc *dv = macb_desc_at(rxq->ring, cap, i);

    struct rte_mbuf *m_old = rxq->sw_ring[i];
    if (unlikely(m_old == NULL))
        return -1;

    struct rte_mbuf *m_new = rte_pktmbuf_alloc(rxq->mp);
    if (unlikely(m_new == NULL))
        return -2;

    rte_iova_t diova = rte_mbuf_data_iova(m_new);
    if (unlikely(diova == RTE_BAD_IOVA)) {
        rte_pktmbuf_free(m_new);
        return -3;
    }

    /* Clean new buffer to RAM before handing to HW (non-coherent DMA) */
    void *data = rte_pktmbuf_mtod(m_new, void *);
    size_t room = rte_pktmbuf_data_room_size(rxq->mp) - RTE_PKTMBUF_HEADROOM;
    (void)sync_buf_to_dev_s(rxq->sync_fd, data, room < 128 ? room : 128);
    rte_io_wmb();

    const uint64_t bus = (uint64_t)BUS_IOVA(diova);
    const uint32_t wrap = (i == (rxq->nb_desc - 1)) ? 1u : 0u;

    /* publish new buffer into the SAME descriptor slot */
    macb_rx_publish_desc(rxq, dv, bus, wrap);

    /* Swap */
    rxq->sw_ring[i] = m_new;
    *out_old = m_old;
    return 0;
}


/* ------------------------------- RX init ---------------------------------- */


/* Fresh RX init that *always* clears any extension words present in the ring,
 * based on stride (not on cap flags), so w2..w5 can never contain poison.
 */
void macb_rx_init(struct macb_rxq *rxq)
{
    const uint16_t nb = rxq->nb_desc;
    const unsigned cap = rxq->hw_dma_cap;
    const size_t stride = rxq->desc_stride ?
                          rxq->desc_stride :
                          (size_t)macb_desc_stride_bytes(cap);

    rxq->rx_tail = 0;
    rxq->rx_head = 0;

    /* 1) Reset ring to SW-owned (USED=1), keep WRAP on last, and zero extras */
    for (uint16_t i = 0; i < nb; i++) {
        const uint32_t wrap = (i == nb - 1) ? RX_WRAP : 0;
        volatile struct macb_desc *dv = macb_desc_at(rxq->ring, cap, i);
        volatile uint32_t *w = (volatile uint32_t *)dv;

        if (rxq->sw_ring[i]) {
            rte_pktmbuf_free(rxq->sw_ring[i]);
            rxq->sw_ring[i] = NULL;
        }

        w[0] = RX_USED | wrap; /* ADDR (SW owns; USED=1 means SW) */
        w[1] = 0;              /* STAT */

        /* Critical: clear any extra words that exist for this stride.
         * Do NOT rely on cap bits; we've seen stride=24 rings where HW
         * consumes w2..w5 even if cap flags disagree.
         */
        if (stride >= 16) { w[2] = 0; w[3] = 0; }
        if (stride >= 24) { w[4] = 0; w[5] = 0; }
    }

    /* Make reset ring visible to device */
    sync_desc_to_dev_v(rxq->sync_fd, rxq->ring, (size_t)nb * stride);
    rte_io_wmb();

    /* 2) Program ring base */
    macb_rx_program_rbqp(rxq);
    rte_io_wmb();

    /* 3) Seed buffers: call refill until it stops making progress */
    for (;;) {
        const uint16_t head0 = rxq->rx_head;
        macb_rx_refill(rxq);
        if (rxq->rx_head == head0)
            break;
    }

    /* 4) Flush whole ring after posting (refill likely wrote descriptors) */
    sync_desc_to_dev_v(rxq->sync_fd, rxq->ring, (size_t)nb * stride);
    rte_io_wmb();
}

uint16_t macb_rx_burst(void *queue, struct rte_mbuf **rx_pkts, uint16_t nb_pkts)
{
    fprintf(stderr, "ENTER RX\n");
    struct macb_rxq *rxq = (struct macb_rxq *)queue;
    uint16_t nb = 0;

    if (unlikely(nb_pkts == 0))
        return 0;

    while (nb < nb_pkts) {
        const uint16_t i = rxq->rx_tail;
        const unsigned cap = rxq->hw_dma_cap;
        volatile struct macb_desc *dv = macb_desc_at(rxq->ring, cap, i);

        /* Sync this descriptor from device and read */
        const size_t desc_stride = rxq->desc_stride
                                   ? (size_t)rxq->desc_stride
                                   : (size_t)macb_desc_stride_bytes(cap);
        sync_desc_from_dev_v(rxq->sync_fd, dv, desc_stride);
        rte_io_rmb();
        const uint32_t addr = dv->addr;
        const uint32_t stat = dv->ctrl;

        /* Not ready until HW sets USED in addr word */
        if ((addr & RX_USED) == 0)
            break;

        const int has_eof = !!(stat & RX_EOF);
        const int has_sof = !!(stat & RX_SOF);
        uint16_t len = (uint16_t)(stat & RX_LEN_MASK);

        /* We must have an mbuf mapped for this slot */
        struct rte_mbuf *m_old = NULL;
        if (unlikely(rxq->sw_ring[i] == NULL)) {
            /* Can't safely progress; this is a ring bookkeeping bug */
            rxq->rx_tail = macb_ring_next(rxq->rx_tail, rxq->nb_desc);
            continue;
        }

        /* Swap-rearm FIRST: keeps HW fed and prevents BNA */
        if (unlikely(macb_rx_rearm_slot_swap(rxq, i, &m_old) != 0 || m_old == NULL)) {
            /* No replacement buffer => stop draining to avoid creating holes */
            break;
        }

        /* Compute room from the OLD mbuf for clamping */
        size_t room   = m_old->buf_len - RTE_PKTMBUF_HEADROOM;

        /* Clamp */
        if (unlikely(len > room))
            len = (uint16_t)room;


        /* If incomplete/empty, drop but DO NOT break the ring (we already re-armed) */
        if (unlikely(!has_sof || !has_eof || len == 0)) {
            rte_pktmbuf_free(m_old);
            rxq->rx_tail = macb_ring_next(rxq->rx_tail, rxq->nb_desc);
            continue;
        }

        /* Fill mbuf metadata for OLD mbuf */
        m_old->data_off = RTE_PKTMBUF_HEADROOM;
        m_old->data_len = len;
        m_old->pkt_len  = len;
        m_old->port     = rxq->port_id;

        /* Debug: one log line per received frame.
         * buf   — buffer physical address (addr word masked to RX_ADDR_MASK)
         * stat  — raw status/ctrl word (w1); inspect with the register map below
         * len   — frame length extracted from stat[12:0]
         * sof/eof — Start/End-of-Frame flags (stat bits 14/15); always 1 here
         * bcast — broadcast frame      (stat bit 21, GEM rx_w_broadcast_frame)
         * mhash — multicast hash match (stat bit 22, GEM rx_w_mult_hash_match)
         * uhash — unicast hash match   (stat bit 23, GEM rx_w_uni_hash_match)
         * sa    — specific-address register match nibble (stat bits [31:28],
         *         maps to SA4/SA3/SA2/SA1 matching, GEM rx_w_add_match[4:1])
         */
        RTE_LOG(INFO, MACB,
            "macb rx[%u]: buf=0x%08x stat=0x%08x len=%u "
            "sof=%d eof=%d bcast=%d mhash=%d uhash=%d sa=0x%x\n",
            (unsigned)i,
            addr & RX_ADDR_MASK,
            stat,
            (unsigned)len,
            has_sof, has_eof,
            !!(stat & (1u << 21)),
            !!(stat & (1u << 22)),
            !!(stat & (1u << 23)),
            (stat >> 28) & 0xFu);

        rx_pkts[nb++] = m_old;

        rxq->rx_tail = macb_ring_next(rxq->rx_tail, rxq->nb_desc);
    }

    macb_rx_refill(rxq);

    return nb;
}


/* ------------------------------- TX burst --------------------------------- */
uint16_t
macb_tx_burst(void *tx_queue, struct rte_mbuf **tx_pkts, uint16_t nb_pkts)
{
    struct macb_txq *txq = (struct macb_txq *)tx_queue;
    struct macb_adapter *ad = txq->ad;
    const uint16_t nb_desc = txq->nb_desc;
    uint16_t sent = 0;

    if (nb_pkts == 0 || nb_desc == 0)
        return 0;

    (void)macb_tx_reclaim(txq);

    uint16_t cons = txq->cons;
    uint16_t prod = txq->prod;
    uint16_t used = (prod >= cons) ? (uint16_t)(prod - cons)
                                   : (uint16_t)(nb_desc - cons + prod);
    uint16_t free = (uint16_t)(nb_desc - 1 - used); /* one-slot guard */

    if (free == 0)
        return 0;
    if (nb_pkts > free)
        nb_pkts = free;

    const unsigned cap = txq->hw_dma_cap;
    const size_t stride = txq->desc_stride ? txq->desc_stride
                                      : (size_t)macb_desc_stride_bytes(cap);

    for (; sent < nb_pkts; sent++) {
        struct rte_mbuf *m = tx_pkts[sent];
        const uint16_t len = (uint16_t)m->pkt_len;
        const rte_iova_t biova = rte_mbuf_data_iova(m);

        const uint32_t wrap = (prod == (nb_desc - 1)) ? TX_WRAP : 0;
        const uint32_t ctrl = wrap | TX_LAST | ((uint32_t)len & TX_LEN_MASK);

        /* Clean payload before DMA reads */
        sync_buf_to_dev_s(txq->sync_fd, rte_pktmbuf_mtod(m, void *), len);
        rte_wmb();

        volatile uint32_t *w = macb_tx_desc_words(txq->ring, cap, prod);

        /* Clear entire slot so extension words don't retain stale values */
        memset((void *)(uintptr_t)w, 0, stride);

        w[0] = (uint32_t)BUS_IOVA(biova);
        rte_io_wmb();
        w[1] = ctrl;

        macb_tx_desc_sync_to(txq, w, cap);

        txq->sw_ring[prod] = m;

        if (++prod == nb_desc)
            prod = 0;
    }

    rte_io_wmb();
    macb_tx_kick(txq);

    txq->prod = prod;
    (void)macb_tx_reclaim(txq);
    ad->sw.tx_pkts += sent;
    return sent;
}