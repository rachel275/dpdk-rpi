// drivers/net/macb/macb_rxtx.c
// Lightweight RX/TX for MACB/GEM (RP1). Trace via MACB_RXTX_TRACE=0..3.
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <rte_branch_prediction.h>
#include <rte_cycles.h>
#include <rte_io.h>
#include <rte_log.h>
#include <rte_mbuf.h>
#include <rte_hexdump.h>
#include "macb_hw.h"
#include "macb_rx_audit.h"
#include "dma_sync_uapi.h"   /* struct dma_sync_range + DMA_SYNC_* ioctls */
#include "macb_dma_sync.h"

#ifndef RTE_LOGTYPE_PMD
#define RTE_LOGTYPE_PMD RTE_LOGTYPE_USER1
#endif

#ifndef SPD100
# define SPD100 SPD
#endif

/* ====== Layout: choose which word holds ADDR vs STAT (compile-time pin) ==== */
#if defined(MACB_PINNED_RX_LAYOUT_W1_ADDR_W0_STAT)
# define RX_PICK_ADDR(w0, w1) (w1)
# define RX_PICK_STAT(w0, w1) (w0)
# define RX_LAYOUT_STR        "w1=addr,w0=stat"
#else
# define RX_PICK_ADDR(w0, w1) (w0)
# define RX_PICK_STAT(w0, w1) (w1)
# define RX_LAYOUT_STR        "w0=addr,w1=stat"
#endif

/* ------------------------------- TX bits ---------------------------------- */
#ifndef MACB_TSR_UBR
# define MACB_TSR_UBR    0x00000001u
#endif
#ifndef MACB_TSR_TXGO
# define MACB_TSR_TXGO   0x00000008u
#endif
#ifndef MACB_TSR_TFC
# define MACB_TSR_TFC    0x00000010u
#endif
#ifndef MACB_TSR_TXCOMP
# define MACB_TSR_TXCOMP 0x00000020u
#endif
#ifndef MACB_TSR_HRESP
# define MACB_TSR_HRESP  0x00000100u
#endif
#ifndef MACB_TSR_COL
# define MACB_TSR_COL    (1u << 1)
#endif
#ifndef MACB_TSR_RLE
# define MACB_TSR_RLE    (1u << 2)
#endif
#ifndef MACB_TSR_BEX
# define MACB_TSR_BEX    (1u << 4)
#endif
#ifndef MACB_TSR_UND
# define MACB_TSR_UND    (1u << 6)
#endif

#if   defined(NCR_TXEN)
# define MACB_NCR_TXEN   NCR_TXEN
#elif defined(MACB_NCR_TXEN)
# define MACB_NCR_TXEN   MACB_NCR_TXEN
#elif defined(GEM_NCR_TXEN)
# define MACB_NCR_TXEN   GEM_NCR_TXEN
#else
# define MACB_NCR_TXEN   (1u << 3)
#endif
#if   defined(NCR_TSTART)
# define MACB_NCR_TSTART NCR_TSTART
#elif defined(MACB_NCR_TSTART)
# define MACB_NCR_TSTART MACB_NCR_TSTART
#elif defined(GEM_NCR_TSTART)
# define MACB_NCR_TSTART GEM_NCR_TSTART
#elif defined(MACB_BIT_TSTART)
# define MACB_NCR_TSTART MACB_BIT_TSTART
#else
# define MACB_NCR_TSTART (1u << 9)
#endif
#if !defined(MACB_TBQPH) && defined(MACB_TBQP_HI)
# define MACB_TBQPH MACB_TBQP_HI
#endif

/* ------------------------------- RX bits ---------------------------------- */
#ifndef MACB_RSR
#define MACB_RSR       0x20
#endif
#ifndef MACB_RSR_REC
# define MACB_RSR_REC  (1u << 0)
#endif
#ifndef MACB_RSR_BNA
# define MACB_RSR_BNA  (1u << 1)
#endif
#ifndef MACB_RSR_OVR
# define MACB_RSR_OVR  (1u << 2)
#endif
#ifndef MACB_RSR_HRESP
# define MACB_RSR_HRESP (1u << 3) /* if present */
#endif
#ifndef MACB_RSR_BEX
# define MACB_RSR_BEX   0 /* if present on SoC, define accordingly */
#endif

#ifndef MACB_NCR_RXEN
# ifdef NCR_RXEN
#  define MACB_NCR_RXEN NCR_RXEN
# elif defined(MACB_NCR_RE)
#  define MACB_NCR_RXEN MACB_NCR_RE
# elif defined(GEM_NCR_RXEN)
#  define MACB_NCR_RXEN GEM_NCR_RXEN
# else
#  define MACB_NCR_RXEN (1u << 2)
# endif
#endif

#ifndef MACB_NCFGR_FD
# ifdef GEM_NCFGR_FD
#  define MACB_NCFGR_FD GEM_NCFGR_FD
# else
#  define MACB_NCFGR_FD (1u << 1)
# endif
#endif
#ifndef MACB_NCFGR_SPD
# ifdef GEM_NCFGR_SPD
#  define MACB_NCFGR_SPD GEM_NCFGR_SPD
# else
#  define MACB_NCFGR_SPD (1u << 0)
# endif
#endif
#ifndef MACB_NCFGR_GBE
# ifdef GEM_NCFGR_GBE
#  define MACB_NCFGR_GBE GEM_NCFGR_GBE
# else
#  define MACB_NCFGR_GBE (1u << 10)
# endif
#endif
#ifndef MACB_NCFGR_CAF
# ifdef GEM_NCFGR_CAF
#  define MACB_NCFGR_CAF GEM_NCFGR_CAF
# else
#  define MACB_NCFGR_CAF (1u << 4)
# endif
#endif
#ifndef MACB_NCFGR_NBC
# ifdef GEM_NCFGR_NBC
#  define MACB_NCFGR_NBC GEM_NCFGR_NBC
# else
#  define MACB_NCFGR_NBC (1u << 5)
# endif
#endif

/* Default: do NOT re-prime RBQP in the background. Opt-in with -DMACB_RX_REPRIME=1 */
#ifndef MACB_RX_REPRIME
#define MACB_RX_REPRIME 0
#endif

#ifndef MACB_RX_RECOVERY_LIGHT_ONLY
#define MACB_RX_RECOVERY_LIGHT_ONLY 0
#endif

/* ------------------------------------------------------------------------- */
struct macb_txq; struct macb_rxq; struct macb_adapter;
struct macb_desc;

/* ---- TX descriptor helpers (stride/cap aware, no external symbols) ---- */

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
/* Bus offset (exported by ethdev)                                           */
/* ------------------------------------------------------------------------- */
extern uint64_t g_bus_ofs;
#define BUS_IOVA(x) ((rte_iova_t)((rte_iova_t)(x) + (rte_iova_t)g_bus_ofs))

/* ------------------------------------------------------------------------- */
/* Tracing                                                                   */
/* ------------------------------------------------------------------------- */
#ifndef MACB_RXTX_TRACE
#define MACB_RXTX_TRACE 0
#endif
#define MACB_RXTX_TRACE_DEFAULT 1
static int macb_trace_level = MACB_RXTX_TRACE_DEFAULT;
static inline void macb_trace_init_once(void) {
    static int inited;
    if (!inited) {
        const char *e = getenv("MACB_RXTX_TRACE");
        macb_trace_level = (e && *e) ? atoi(e) : MACB_RXTX_TRACE_DEFAULT;
        inited = 1;
    }
}
#define RT_LVL(lv, fmt, ...) do { \
    macb_trace_init_once(); \
    if (macb_trace_level >= (lv)) RTE_LOG(INFO, PMD, "macb[rxtx]: " fmt, ##__VA_ARGS__); \
} while (0)
#define RTI(fmt, ...) RT_LVL(1, fmt, ##__VA_ARGS__)
#define RTV(fmt, ...) RT_LVL(2, fmt, ##__VA_ARGS__)
#define RTD(fmt, ...) RT_LVL(3, fmt, ##__VA_ARGS__)

/* ------------------------------------------------------------------------- *
 * Testing
 * ---------------------------------------------------------------------------*/
#define MY_TEST_ETYPE 0x88B5
#define MY_MAGIC      0x4D414342u /* "MACB" */

static inline int
macb_is_my_test_pkt(struct rte_mbuf *m)
{
    /* We need Ethernet header (14) + magic(4) + seq(4) = 22 bytes */
    if (unlikely(rte_pktmbuf_data_len(m) < sizeof(struct rte_ether_hdr) + 8))
        return 0;

    struct rte_ether_hdr *eth = rte_pktmbuf_mtod(m, struct rte_ether_hdr *);
    if (eth->ether_type != rte_cpu_to_be_16(MY_TEST_ETYPE))
        return 0;

    const uint8_t *p = (const uint8_t *)(eth + 1);
    uint32_t magic;
    memcpy(&magic, p, sizeof(magic));
    if (magic != rte_cpu_to_be_32(MY_MAGIC))
        return 0;

    return 1;
}

static inline void macb_rx_check_rbqp_stride(struct macb_rxq *rxq)
{
    const size_t stride = macb_desc_stride_bytes(rxq->hw_dma_cap);

    uint64_t base = rxq->ring_iova ? rxq->ring_iova : (rte_iova_t)(uintptr_t)rxq->ring;
    uint32_t base_lo = (uint32_t)BUS_IOVA(base);

    uint32_t rbqp_lo = macb_readl(&rxq->ad->hw, MACB_RBQP);
    uint32_t ofs = rbqp_lo - base_lo;

    if (ofs % stride) {
        RTE_LOG(ERR, PMD, "RBQP stride mismatch: base=%08x rbqp=%08x ofs=%u stride=%zu ofs%%stride=%u cap=0x%x\n",
                base_lo, rbqp_lo, ofs, stride, (unsigned)(ofs % stride), rxq->hw_dma_cap);
    } else {
    
        RTE_LOG(INFO, PMD, "RBQP stride match: base=%08x rbqp=%08x ofs=%u stride=%zu ofs%%stride=%u cap=0x%x\n",
                base_lo, rbqp_lo, ofs, stride, (unsigned)(ofs % stride), rxq->hw_dma_cap);
    }
}

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */
static inline int rx_hw_owns_addr_word(uint32_t addr_word) { return (addr_word & RX_USED) == 0; }
static inline int rx_sw_owns_addr_word(uint32_t addr_word) { return (addr_word & RX_USED) != 0; }

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


static inline void macb_dump_mbuf(const struct rte_mbuf *m, const char *tag)
{
    uint32_t cap = rte_pktmbuf_pkt_len(m);
    if (cap > 128) cap = 128;
    uint8_t tmp[128];
    const void *p = rte_pktmbuf_read(m, 0, cap, tmp);
    if (!p) {
        RTE_LOG(WARNING, PMD, "%s: rte_pktmbuf_read failed (pkt_len=%u)\n",
                tag, rte_pktmbuf_pkt_len(m));
        return;
    }

    RTE_LOG(INFO, PMD, "%s: pkt_len=%u data_len=%u nb_segs=%u\n",
            tag, rte_pktmbuf_pkt_len(m), rte_pktmbuf_data_len(m), m->nb_segs);

    rte_hexdump(stderr, tag, p, cap);
}



static inline void
macb_dump_region(struct macb_rxq *rxq, const uint8_t *base, uint32_t buflen,
                 const char *name, uint32_t start, uint32_t nbytes)
{
    if (start >= buflen || nbytes == 0)
        return;
    if (start + nbytes > buflen)
        nbytes = buflen - start;

    sync_buf_from_dev_s(rxq->sync_fd, base + start, nbytes);
    rte_io_rmb();
    rte_hexdump(stderr, name, base + start, nbytes);
}

static inline void
macb_dump_mbuf_window(struct macb_rxq *rxq, const char *tag,
                      struct rte_mbuf *m, uint16_t claimed_len,
                      uint32_t around)
{
    if (rxq == NULL || m == NULL)
        return;

    const uint8_t *base = (const uint8_t *)m->buf_addr;
    const uint32_t buflen = m->buf_len;

    if (base == NULL || buflen == 0) {
        RTE_LOG(INFO, PMD, "RX[mbufdump:%s] m=%p invalid buf\n", tag ? tag : "-", (void*)m);
        return;
    }

    const uint32_t off = (m->data_off < buflen) ? m->data_off : 0;
    const uint32_t hr  = (RTE_PKTMBUF_HEADROOM < buflen) ? RTE_PKTMBUF_HEADROOM : 0;

    RTE_LOG(INFO, PMD,
        "RX[mbufdump:%s] m=%p buf=%p buflen=%u data_off=%u data_len=%u pkt_len=%u claimed_len=%u around=%u\n",
        tag ? tag : "-", (void*)m, m->buf_addr, buflen,
        m->data_off, m->data_len, m->pkt_len, claimed_len, around);

    /* Around m->data_off */
    {
        uint32_t s = (off > 64) ? (off - 64) : 0;
        macb_dump_region(rxq, base, buflen, "  data@data_off", s, 256);
    }

    /* Around requested offset */
    if (around < buflen) {
        uint32_t s = (around > 64) ? (around - 64) : 0;
        macb_dump_region(rxq, base, buflen, "  data@around", s, 256);
    }

    /* Around headroom */
    {
        uint32_t s = (hr > 64) ? (hr - 64) : 0;
        macb_dump_region(rxq, base, buflen, "  data@headroom", s, 256);
    }
}

static inline void
macb_dump_rx_desc_raw(struct macb_rxq *rxq, uint16_t idx, const char *tag)
{
    volatile uint32_t *w =
        (volatile uint32_t *)((uint8_t *)rxq->ring + (size_t)idx * (size_t)rxq->desc_stride);

    /* Pull the whole descriptor from device-visible memory before printing */
    sync_desc_from_dev(rxq->sync_fd, (void *)(uintptr_t)w, rxq->desc_stride);
    rte_io_rmb();

    uint32_t nwords = rxq->desc_stride / 4;
    RTE_LOG(INFO, PMD, "macb: RXDESC[%s] i=%u stride=%u:\n",
            tag, idx, rxq->desc_stride);

    for (uint32_t j = 0; j < nwords; j++) {
        RTE_LOG(INFO, PMD, "macb:   w%u=%08x\n", j, w[j]);
    }
}

static inline void
macb_rx_print_dual_interp(const char *tag, uint16_t idx, uint32_t w0, uint32_t w1)
{
    uint32_t addrA = w0, statA = w1;
    uint32_t addrB = w1, statB = w0;

    uint32_t aA = addrA & ~0x3u;
    uint32_t aB = addrB & ~0x3u;

    unsigned usedA = !!(addrA & RX_USED);
    unsigned wrapA = !!(addrA & RX_WRAP);
    unsigned lenA  = (statA & RX_LEN_MASK);
    unsigned sofA  = !!(statA & RX_SOF);
    unsigned eofA  = !!(statA & RX_EOF);

    unsigned usedB = !!(addrB & RX_USED);
    unsigned wrapB = !!(addrB & RX_WRAP);
    unsigned lenB  = (statB & RX_LEN_MASK);
    unsigned sofB  = !!(statB & RX_SOF);
    unsigned eofB  = !!(statB & RX_EOF);

    RTE_LOG(INFO, PMD,
        "RX[dual:%s] d%03u raw w0=%08x w1=%08x | "
        "A(addr=w0 stat=w1): addr=%08x USED=%u WRAP=%u LEN=%u SOF=%u EOF=%u | "
        "B(addr=w1 stat=w0): addr=%08x USED=%u WRAP=%u LEN=%u SOF=%u EOF=%u\n",
        tag ? tag : "-", idx, w0, w1,
        aA, usedA, wrapA, lenA, sofA, eofA,
        aB, usedB, wrapB, lenB, sofB, eofB
    );
}

static inline void macb_tsr_decode(uint32_t tsr)
{
    if (!tsr) return;
    RTE_LOG(INFO, PMD, "TSR[post-kick]=0x%08x%s%s%s%s%s%s%s%s",
        tsr,
        (tsr & MACB_TSR_UBR)    ? " UBR"    : "",
        (tsr & MACB_TSR_TXGO)   ? " TXGO"   : "",
        (tsr & MACB_TSR_TXCOMP) ? " TXCOMP" : "",
        (tsr & MACB_TSR_COL)    ? " COL"    : "",
        (tsr & MACB_TSR_RLE)    ? " RLE"    : "",
        (tsr & MACB_TSR_UND)    ? " UND"    : "",
        (tsr & MACB_TSR_BEX)    ? " BEX"    : "",
        (tsr & MACB_TSR_HRESP)  ? " HRESP"  : ""
    );
}
static inline void macb_rsr_decode(uint32_t rsr)
{
    if (!rsr) return;
    RTE_LOG(INFO, PMD, "RSR=0x%08x (REC=%u BNA=%u OVR=%u%s%s) \n",
        rsr,
        !!(rsr & MACB_RSR_REC), !!(rsr & MACB_RSR_BNA), !!(rsr & MACB_RSR_OVR),
        (MACB_RSR_BEX && (rsr & MACB_RSR_BEX))     ? " BEX"   : "",
        (MACB_RSR_HRESP && (rsr & MACB_RSR_HRESP)) ? " HRESP" : "");
}

static inline uint32_t macb_rx_desc_wrap(uint16_t i, uint16_t n)
{ return (i == (n - 1)) ? RX_WRAP : 0; }

static inline void macb_read_desc_words_cap(volatile struct macb_desc *dv,
                                            uint32_t *w0, uint32_t *w1,
                                            int sync_fd,
                                            unsigned hw_dma_cap)
{
    const size_t stride = macb_desc_stride_bytes(hw_dma_cap);
    sync_desc_from_dev_v(sync_fd, dv, stride);
    rte_io_rmb();
    *w0 = ((volatile uint32_t *)dv)[0];
    *w1 = ((volatile uint32_t *)dv)[1];
}

static inline void
macb_rx_dump_ring_window(struct macb_rxq *rxq, const char *tag,
                         uint16_t center, uint16_t radius)
{
    const uint16_t nb = rxq->nb_desc;
    const uint16_t start = (uint16_t)((center + nb - radius) % nb);
    const uint16_t count = (uint16_t)(2 * radius + 1);

    const size_t stride = rxq->desc_stride ? rxq->desc_stride
                                          : (size_t)macb_desc_stride_bytes(rxq->hw_dma_cap);

    RTE_LOG(INFO, PMD, "RX[ringdump:%s] center=%u radius=%u (start=%u count=%u stride=%zu)\n",
            tag ? tag : "-", center, radius, start, count, stride);

    for (uint16_t k = 0; k < count; k++) {
        const uint16_t i = (uint16_t)((start + k) % nb);
        volatile struct macb_desc *dv =
            (volatile struct macb_desc *)macb_desc_at((void *)rxq->ring, rxq->hw_dma_cap, i);

        uint32_t w0, w1;
        macb_read_desc_words_cap(dv, &w0, &w1, rxq->sync_fd, rxq->hw_dma_cap);

        RTE_LOG(INFO, PMD, "  d%03u raw w0=%08x w1=%08x\n", i, w0, w1);

        sync_desc_from_dev_v(rxq->sync_fd, dv, stride);
        rte_io_mb();
        rte_hexdump(stderr, "    desc-bytes", (const void *)(uintptr_t)dv, stride);
    }
}

static inline void
macb_rx_probe_slot(struct macb_rxq *rxq, uint16_t i, const char *tag)
{
    volatile struct macb_desc *dv =
        (volatile struct macb_desc *)macb_desc_at((void *)rxq->ring, rxq->hw_dma_cap, i);

    uint32_t w0, w1;
    macb_read_desc_words_cap(dv, &w0, &w1, rxq->sync_fd, rxq->hw_dma_cap);

    uint32_t addr = RX_PICK_ADDR(w0, w1);
    uint32_t stat = RX_PICK_STAT(w0, w1);

    struct rte_mbuf *m = rxq->sw_ring[i];

    uint32_t expect_bus = 0;
    rte_iova_t expect_iova = 0;
    void *buf_va = NULL;

    if (m) {
        expect_iova = rte_mbuf_data_iova(m);         /* in PA mode, this is PA */
        expect_bus  = (uint32_t)BUS_IOVA(expect_iova);
        buf_va = m->buf_addr;
    }

    RTE_LOG(INFO, PMD,
        "RX[probe:%s] i=%u dv=%p w0=%08x w1=%08x | addr=%08x stat=%08x (USED=%u WRAP=%u LEN=%u SOF=%u EOF=%u)"
        " | sw_m=%p buf_va=%p expect_iova=%08x expect_bus=%08x\n",
        tag ? tag : "-",
        i, (void *)(uintptr_t)dv, w0, w1,
        addr, stat,
        !!(addr & RX_USED), !!(addr & RX_WRAP),
        (stat & RX_LEN_MASK), !!(stat & RX_SOF), !!(stat & RX_EOF),
        (void *)m, buf_va,
        (uint32_t)expect_iova, expect_bus
    );
}

static inline void macb_rx_read_rbqp(struct macb_rxq *rxq, uint32_t *hi, uint32_t *lo);
static inline uint16_t macb_rbqp_hw_idx(struct macb_rxq *rxq);


static inline void
macb_rx_dump_bna_snapshot(struct macb_rxq *rxq, uint32_t rsr, const char *why)
{
    const uint16_t nb = rxq->nb_desc;
    const unsigned cap = rxq->hw_dma_cap;
    const size_t stride = rxq->desc_stride ? rxq->desc_stride
                                           : macb_desc_stride_bytes(cap);

    /* HW pointer */
    uint32_t rbqph = 0, rbqp = 0;
    macb_rx_read_rbqp(rxq, &rbqph, &rbqp);
    const uint16_t hw = macb_rbqp_hw_idx(rxq);

    /* Quick ownership summary */
    uint16_t hw_owned = 0, sw_owned = 0, used_set = 0;
    for (uint16_t i = 0; i < nb; i++) {
        volatile struct macb_desc *dv = macb_desc_at(rxq->ring, cap, i);
        uint32_t w0, w1;
        macb_read_desc_words_cap(dv, &w0, &w1, rxq->sync_fd, cap);
        uint32_t addr = RX_PICK_ADDR(w0, w1);
        if (addr & RX_USED) {
            /* USED=1 => HW “done” / SW-owned in your convention */
            sw_owned++;
            used_set++;
        } else {
            /* USED=0 => still posted to HW */
            hw_owned++;
        }
    }

    RTE_LOG(ERR, PMD,
        "RX[BNA SNAP] %s: RSR=0x%08x (REC=%u BNA=%u OVR=%u) nb=%u stride=%zu cap=0x%x "
        "rbqp=%08x:%08x hw_idx=%u sw_head=%u sw_tail=%u hw_owned=%u sw_owned=%u\n",
        why ? why : "-",
        rsr, !!(rsr & MACB_RSR_REC), !!(rsr & MACB_RSR_BNA), !!(rsr & MACB_RSR_OVR),
        nb, stride, cap, rbqph, rbqp, hw, rxq->rx_head, rxq->rx_tail,
        hw_owned, sw_owned);

    /* Probe “who owns what” around where HW is pointing */
    for (int d = -4; d <= 4; d++) {
        uint16_t idx = (uint16_t)((hw + nb + d) % nb);
        macb_rx_probe_slot(rxq, idx, "BNA");
    }

    /* Dump raw bytes of the descriptor HW is pointing at (and around it) */
    macb_rx_dump_ring_window(rxq, "BNA", hw, 2);

    /* Also dump SW tail slot, since that’s where you’re trying to consume */
    macb_rx_probe_slot(rxq, rxq->rx_tail, "BNA-tail");
    macb_dump_rx_desc_raw(rxq, rxq->rx_tail, "BNA-tail-raw");

    /* If there is an mbuf at tail, dump its buffer windows too */
    if (rxq->sw_ring[rxq->rx_tail]) {
        struct rte_mbuf *m = rxq->sw_ring[rxq->rx_tail];
        macb_dump_mbuf_window(rxq, "BNA-tail-mbuf", m, 0 /* claimed_len */, RTE_PKTMBUF_HEADROOM);
    }
}

static inline void
macb_rx_publish_desc_stride24(struct macb_rxq *rxq,
                              volatile struct macb_desc *dv,
                              uint64_t bus, bool is_last)
{
    const uint32_t wrap = is_last ? RX_WRAP : 0;
    volatile uint32_t *w = (volatile uint32_t *)dv;

    /* Clear all 24B (w0..w5) so HW never sees stale ext words */
    w[0] = 0; w[1] = 0; w[2] = 0; w[3] = 0; w[4] = 0; w[5] = 0;

    /* Publish: STAT first, then ADDR last */
#if defined(MACB_PINNED_RX_LAYOUT_W1_ADDR_W0_STAT)
    w[0] = 0; /* STAT */
    rte_io_wmb();
    w[1] = (((uint32_t)bus & ~0x3u) | wrap); /* ADDR (RX_USED clear => HW owns) */
#else
    w[1] = 0; /* STAT */
    rte_io_wmb();
    w[0] = (((uint32_t)bus & ~0x3u) | wrap); /* ADDR */
#endif

    /* MUST sync full 24 bytes, not 8 */
    sync_desc_to_dev(rxq->sync_fd, (const volatile void *)dv, 24);
    rte_io_wmb();
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

    /* 3) Publish order: STAT first, then ADDR last */
#if defined(MACB_PINNED_RX_LAYOUT_W1_ADDR_W0_STAT)
    w[0] = 0;        /* STAT */
    rte_io_wmb();
    w[1] = addr_lo;  /* ADDR (USED=0 => HW owns) */
#else
    w[1] = 0;        /* STAT */
    rte_io_wmb();
    w[0] = addr_lo;  /* ADDR */
#endif

    /* 4) Sync FULL stride so device sees cleared w2..w5 */
    sync_desc_to_dev_v(rxq->sync_fd, dv, stride);
    rte_io_wmb();
}


/* Debug dump */
static inline void macb_rx_dump_d2(struct macb_rxq *rxq, const char *tag)
{
    uint32_t rbqp = macb_readl(&rxq->ad->hw, MACB_RBQP);
#ifdef MACB_RBQPH
    uint32_t rbqph = macb_readl(&rxq->ad->hw, MACB_RBQPH);
#else
    uint32_t rbqph = 0;
#endif
    RTE_LOG(INFO, PMD, "RX[%s]: RBQP now %08x:%08x", tag ? tag : "dump", rbqph, rbqp);

    for (int i = 0; i < 2 && i < rxq->nb_desc; ++i) {
        volatile struct macb_desc *dv =
    (volatile struct macb_desc *)macb_desc_at((void *)rxq->ring, rxq->hw_dma_cap, i);

	uint32_t w0, w1; macb_read_desc_words_cap(dv, &w0, &w1, rxq->sync_fd, rxq->hw_dma_cap);
        uint32_t daddr_raw = RX_PICK_ADDR(w0, w1);
        uint32_t dstat     = RX_PICK_STAT(w0, w1);
        const uint32_t daddr_iova = (daddr_raw & ~0x3u);
        const int used   = !!(daddr_raw & RX_USED);
        const int wrap   = !!(daddr_raw & RX_WRAP);
        const int own_hw = !used;
        const uint32_t dlen = (dstat & RX_LEN_MASK);
        const int sof = !!(dstat & RX_SOF);
        const int eof = !!(dstat & RX_EOF);

        RTE_LOG(INFO, PMD,
            "RX[%s] d%03d: w0=%08x w1=%08x | layout=%s | addr=%08x USED=%d OWN=%s WRAP=%d len=%u sof=%d eof=%d",
            tag ? tag : "-", i, w0, w1, RX_LAYOUT_STR,
            daddr_iova, used, own_hw ? "HW" : "SW", wrap, dlen, sof, eof);
    }

#ifdef MACB_RSR
    {
        uint32_t rsr = macb_readl(&rxq->ad->hw, MACB_RSR);
        if (rsr) { macb_writel(&rxq->ad->hw, MACB_RSR, rsr); rte_io_wmb(); }
        RTE_LOG(INFO, PMD, "RX[%s] RSR snap/ack=%08x", tag ? tag : "-", rsr);
    }
#endif
}

static inline void macb_rx_read_rbqp(struct macb_rxq *rxq, uint32_t *hi, uint32_t *lo)
{
#ifdef MACB_RBQPH
    *hi = macb_readl(&rxq->ad->hw, MACB_RBQPH);
#else
    *hi = 0;
#endif
    *lo = macb_readl(&rxq->ad->hw, MACB_RBQP);
}

/* ----------- sanity helpers requested: nb_desc, alignment, RBQP ∈ ring ---- */
static inline int is_pow2(unsigned x) { return x && !(x & (x - 1)); }

static inline int macb_rbqp_points_into_ring(struct macb_rxq *rxq)
{
    const uint16_t nb = rxq->nb_desc;

    const uint64_t base = rxq->ring_iova ? rxq->ring_iova
                                         : (rte_iova_t)(uintptr_t)rxq->ring;
    const uint64_t bus  = BUS_IOVA(base);
    const uint32_t lo   = (uint32_t)bus;

    const size_t stride = rxq->desc_stride ? rxq->desc_stride
                                          : (size_t)macb_desc_stride_bytes(rxq->hw_dma_cap);
    const size_t ring_bytes = (size_t)nb * stride;

    const uint32_t have = macb_readl(&rxq->ad->hw, MACB_RBQP);
    return ((have - lo) < ring_bytes);
}

/* Enable/disable RX buffer prep */
#ifndef MACB_RX_BUF_PREP
#define MACB_RX_BUF_PREP 1
#endif

/* How many bytes to prep (at least 1 cache line; 64 is a good start) */
#ifndef MACB_RX_BUF_PREP_BYTES
#define MACB_RX_BUF_PREP_BYTES 128
#endif

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

#if MACB_RX_BUF_PREP
        /*
         * Non-coherent DMA safety:
         * 1) (Optional debug) poison some bytes so we can tell if NIC wrote.
         * 2) Clean those cachelines to RAM BEFORE handing buffer to HW,
         *    otherwise dirty CPU lines may later write back and overwrite DMA.
         */
        void *data = rte_pktmbuf_mtod(m, void *);
        size_t room = rte_pktmbuf_data_room_size(rxq->mp) - RTE_PKTMBUF_HEADROOM;
        size_t prep = (size_t)MACB_RX_BUF_PREP_BYTES;
        if (prep > room) prep = room;

        /* Poison first bytes (debug aid). Comment out if you don’t want it. */
        //memset(data, 0xA5, prep);

        /* Clean to RAM so NIC DMA won't get clobbered by later writeback */
        int rc = sync_buf_to_dev_s(rxq->sync_fd, data, prep);
//	RTD("dma_sync TO_DEV refill i=%u data=%p prep=%zu rc=%d errno=%d\n",
//    i, data, prep, rc, errno);

        rte_io_wmb();
#else
        rte_io_wmb();
#endif

        const uint64_t bus  = (uint64_t)BUS_IOVA(bi);
        const uint32_t wrap = (i == (nb - 1)) ? RX_WRAP : 0;

        //RTE_LOG(INFO, PMD,
        //    "REARM refill i=%u m=%p data_off=%u buf_iova=%"PRIx64" data_iova=%"PRIx64" publish_bus=%"PRIx64"\n",
        //    i, (void*)m, m->data_off,
        //    (uint64_t)m->buf_iova,
        //    (uint64_t)rte_mbuf_data_iova(m),
        //    bus);

        volatile struct macb_desc *dv = macb_desc_at(rxq->ring, cap, i);
       macb_rx_publish_desc(rxq, dv, bus, wrap);


        rxq->sw_ring[i] = m;
        rxq->rx_head = macb_ring_next(rxq->rx_head, nb);
        space--;
    }
}


/* Post-init sanity with relaxed RBQP check (∈ window) */
static void macb_rx_post_init_sanity(struct macb_rxq *rxq, const char *tag)
{
    const uint16_t nb = rxq->nb_desc;
    const size_t stride = macb_desc_stride_bytes(rxq->hw_dma_cap);
    const size_t ring_bytes = (size_t)nb * stride;
    
    //RTE_LOG(INFO, PMD, "RX stride=%zu\n", stride);

    //RTE_LOG(INFO, PMD, "RX[%s] ring=%p (16B aligned=%d)",
    //    tag ? tag : "post-init", (void*)rxq->ring, ((uintptr_t)rxq->ring % 16) == 0);

    if (nb == 0 || !is_pow2(nb)) {
        //RTE_LOG(ERR, PMD, "RX sanity: nb_desc=%u (must be power-of-two > 0)\n", nb);
    }
    if (!rxq->ring) {
        //RTE_LOG(ERR, PMD, "RX sanity: ring pointer is NULL!\n");
    }

    /* What BUS address we programmed as the base */
    const uint64_t base_iova = rxq->ring_iova ? rxq->ring_iova
                                              : (rte_iova_t)(uintptr_t)rxq->ring;
    const uint64_t base_bus  = BUS_IOVA(base_iova);
    const uint32_t want_lo   = (uint32_t)base_bus;
#ifdef MACB_RBQPH
    const uint32_t want_hi   = (uint32_t)(base_bus >> 32);
#endif

    /* Read RBQP (it moves as HW consumes) */
    uint32_t have_hi = 0, have_lo = 0;
#ifdef MACB_RBQPH
    have_hi = macb_readl(&rxq->ad->hw, MACB_RBQPH);
#endif
    have_lo = macb_readl(&rxq->ad->hw, MACB_RBQP);

    const int in_window = ((have_lo - want_lo) < ring_bytes);
    const uint32_t ofs = have_lo - want_lo;
    const uint32_t idx = (ofs / stride) % nb;

    //RTE_LOG(INFO, PMD,
    //    "RX[%s] RBQP want=%08x:%08x have=%08x:%08x ring_size=%zu -> in_window=%d idx=%u\n",
#ifdef MACB_RBQPH
   //     tag ? tag : "post-init", want_hi, want_lo, have_hi, have_lo,
#else
   //     tag ? tag : "post-init", 0u, want_lo, 0u, have_lo,
#endif
   //     ring_bytes, in_window, idx);

    if (!in_window) {
        //RTE_LOG(ERR, PMD, "RX sanity: RBQP points OUTSIDE ring window (unexpected)\n");
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

static inline void macb_rx_check_reprime(struct macb_rxq *rxq)
{
#if !MACB_RX_REPRIME
    (void)rxq; return;
#else
    static uint64_t last_tsc;
    const uint64_t now = rte_get_timer_cycles();
    if (now - last_tsc < rte_get_timer_hz()) return;
    last_tsc = now;

#ifdef MACB_RSR
    const uint32_t rsr_now = macb_readl(&rxq->ad->hw, MACB_RSR);
    if (rsr_now & (MACB_RSR_OVR | MACB_RSR_HRESP | MACB_RSR_BEX))
        return;
#endif

    const uint64_t base = rxq->ring_iova ? rxq->ring_iova
                                         : (rte_iova_t)(uintptr_t)rxq->ring;
    const uint32_t want_lo  = (uint32_t)BUS_IOVA(base);

    const uint32_t have1 = macb_readl(&rxq->ad->hw, MACB_RBQP); rte_io_mb();
    const uint32_t have2 = macb_readl(&rxq->ad->hw, MACB_RBQP);
    if (have1 == want_lo && have2 == want_lo) return;

#ifdef MACB_RSR
    if (rsr_now) { macb_writel(&rxq->ad->hw, MACB_RSR, rsr_now); rte_io_wmb(); }
#endif

    macb_rx_program_rbqp(rxq);
    RTV("RX[reprime]: RBQP<=... (was %08x/%08x)\n", have1, have2);
#endif
}

/* ------------------------------ TX helpers -------------------------------- */
static inline void macb_tx_kick(struct macb_txq *txq)
{
    struct macb_adapter *ad = txq->ad;
    rte_wmb();
    uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
    ncr |= MACB_NCR_TXEN; macb_writel(&ad->hw, MACB_NCR, ncr);
    ncr |= MACB_NCR_TSTART; macb_writel(&ad->hw, MACB_NCR, ncr);
}

static inline uint32_t macb_tsr_snap_and_clear(struct macb_adapter *ad)
{
#ifdef MACB_TSR
    uint32_t tsr = macb_readl(&ad->hw, MACB_TSR);
    if (tsr) macb_writel(&ad->hw, MACB_TSR, tsr);
    return tsr;
#else
    return 0;
#endif
}

static inline void macb_tx_program_tbqp(struct macb_txq *txq) {
    uint64_t base = txq->ring_iova ? txq->ring_iova : (rte_iova_t)(uintptr_t)txq->ring;
    uint32_t tbqp_lo = (uint32_t)BUS_IOVA(base);
#ifdef MACB_TBQPH
    uint32_t tbqp_hi = (uint32_t)((BUS_IOVA(base) >> 32) & 0xFFFFFFFF);
    macb_writel(&txq->ad->hw, MACB_TBQPH, tbqp_hi);
#endif
    macb_writel(&txq->ad->hw, MACB_TBQP, tbqp_lo);
    rte_io_wmb();
}

static inline void macb_tx_check_wrap(struct macb_txq *txq, const char *tag)
{
    const uint16_t last = txq->nb_desc - 1;
    const unsigned cap = txq->hw_dma_cap;
    const size_t stride = txq->desc_stride ? txq->desc_stride
                                      : (size_t)macb_desc_stride_bytes(cap);

    volatile uint32_t *w = macb_tx_desc_words(txq->ring, cap, last);
    macb_tx_desc_sync_from(txq, w, cap);

    (void)tag;
}
static inline void macb_tx_recover_underrun(struct macb_txq *txq, const char *tag)
{
    struct macb_adapter *ad = txq->ad;

#ifdef MACB_TSR
    uint32_t tsr = macb_readl(&ad->hw, MACB_TSR);
    if (tsr & (MACB_TSR_UBR | MACB_TSR_TXCOMP | MACB_TSR_TXGO)) {
        macb_writel(&ad->hw, MACB_TSR, tsr); /* W1C */
        rte_io_wmb();
    }
#endif

    const size_t stride = macb_desc_stride_bytes(txq->hw_dma_cap);
    uint64_t bd_bus = BUS_IOVA(txq->ring_iova + (uint64_t)txq->cons * stride);
    macb_writel(&ad->hw, MACB_TBQP, (uint32_t)bd_bus);
#ifdef MACB_TBQPH
    macb_writel(&ad->hw, MACB_TBQPH, (uint32_t)(bd_bus >> 32));
#endif
    rte_io_wmb();

    uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
    ncr |= MACB_NCR_TXEN | MACB_NCR_TSTART;
    macb_writel(&ad->hw, MACB_NCR, ncr);
    rte_io_wmb();

    (void)tag;
}
void __attribute__((weak))
macb_log_regs_full(struct macb_adapter *ad, const char *tag)
{
    uint32_t ncr   = macb_readl(&ad->hw, MACB_NCR);
    uint32_t ncfgr = macb_readl(&ad->hw, MACB_NCFGR);
    uint32_t nsr   = macb_readl(&ad->hw, MACB_NSR);
#ifdef MACB_TSR
    uint32_t tsr   = macb_readl(&ad->hw, MACB_TSR);
#else
    uint32_t tsr   = 0;
#endif
    uint32_t rbqph = 0, tbqph = 0;
    uint32_t rbqp  = macb_readl(&ad->hw, MACB_RBQP);
    uint32_t tbqp  = macb_readl(&ad->hw, MACB_TBQP);
#ifdef MACB_RBQPH
    rbqph = macb_readl(&ad->hw, MACB_RBQPH);
#endif
#ifdef MACB_TBQPH
    tbqph = macb_readl(&ad->hw, MACB_TBQPH);
#endif
#ifdef GEM_DMACFG
    uint32_t dmacfg = macb_readl(&ad->hw, GEM_DMACFG);
#else
    uint32_t dmacfg = 0xFFFFFFFFu;
#endif

    //RTE_LOG(INFO, PMD,
    //    "macb: [%s] NCR=%08x NCFGR=%08x NSR=%08x TSR=%08x RBQP=%08x:%08x TBQP=%08x:%08x DMACFG=%08x",
    //    tag ? tag : "", ncr, ncfgr, nsr, tsr, rbqph, rbqp, tbqph, tbqp, dmacfg);
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

#ifdef MACB_TSR
    if (outstanding) {
        uint32_t tsr_chk = macb_readl(&txq->ad->hw, MACB_TSR);
        if ((tsr_chk & (MACB_TSR_TXGO | MACB_TSR_UBR | MACB_TSR_TXCOMP)) == 0) {
            macb_tx_recover_underrun(txq, "reclaim");
        }
    }
#endif

    const unsigned cap = txq->hw_dma_cap;
    const size_t stride = txq->desc_stride ? txq->desc_stride
                                      : (size_t)macb_desc_stride_bytes(cap);

    uint16_t reclaimed = 0;

    while (reclaimed < outstanding) {
        const uint16_t cons = txq->cons;

        volatile uint32_t *w = macb_tx_desc_words(txq->ring, cap, cons);
        macb_tx_desc_sync_from(txq, w, cap);
        const uint32_t ctrl = w[1];

        if ((ctrl & TX_USED) == 0) {
            RTI("TX reclaim: 0 (busy cons=%u prod=%u)", cons, txq->prod);
            break;
        }

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

    if (reclaimed) RTI("TX reclaim: %u (cons=%u)", reclaimed, txq->cons);
    else if (txq->cons == txq->prod) RTI("TX reclaim: 0 (idle: cons==prod==%u)", txq->cons);
    else RTI("TX reclaim: 0 (busy: cons=%u prod=%u)", txq->cons, txq->prod);

    return reclaimed;
}
/* ------------------------------- TX init ---------------------------------- */
static inline void macb_tx_init(struct macb_txq *txq)
{
    const unsigned cap = txq->hw_dma_cap;
    const size_t stride = txq->desc_stride ? txq->desc_stride
                                      : (size_t)macb_desc_stride_bytes(cap);

    for (uint16_t i = 0; i < txq->nb_desc; i++) {
        const uint32_t wrap = (i == (txq->nb_desc - 1)) ? TX_WRAP : 0;

        volatile uint32_t *w = macb_tx_desc_words(txq->ring, cap, i);

        memset((void *)(uintptr_t)w, 0, stride);
        w[0] = 0;
        w[1] = TX_USED | wrap;

        macb_tx_desc_sync_to(txq, w, cap);
    }

    sync_desc_to_dev_v(txq->sync_fd, txq->ring, (size_t)txq->nb_desc * stride);
    rte_io_wmb();

    macb_tx_program_tbqp(txq);

#ifdef MACB_TBQPH
    uint32_t tbqph = macb_readl(&txq->ad->hw, MACB_TBQPH);
#else
    uint32_t tbqph = 0;
#endif
    uint32_t tbqp = macb_readl(&txq->ad->hw, MACB_TBQP);

    RTI("TX TBQP programmed/readback: %08x:%08x (ring=%p iova=%08llx bytes=%zu stride=%zu)",
        tbqph, tbqp, txq->ring, (unsigned long long)txq->ring_iova,
        (size_t)txq->nb_desc * stride, stride);

    macb_log_regs_full(txq->ad, "after-tx-init");
    macb_tx_check_wrap(txq, "after-reset");
}
/* ------------------------------- RX cursor -------------------------------- */
static inline uint16_t macb_rbqp_hw_idx(struct macb_rxq *rxq)
{
    const uint64_t base = rxq->ring_iova ? rxq->ring_iova
                                         : (rte_iova_t)(uintptr_t)rxq->ring;
    const uint64_t base_bus = BUS_IOVA(base);
    const uint32_t base_lo  = (uint32_t)base_bus;

    uint32_t rbqp_lo = macb_readl(&rxq->ad->hw, MACB_RBQP);
    uint32_t ofs = rbqp_lo - base_lo;
    const size_t stride = macb_desc_stride_bytes(rxq->hw_dma_cap);

    return (uint16_t)((ofs / stride) % rxq->nb_desc);
}

static inline void macb_log_hw_rx_head(struct macb_adapter *ad, const char *tag)
{
    struct macb_rxq *rxq = &ad->rxq[0];

    uint64_t base = (uint64_t)BUS_IOVA(rxq->ring_iova);
#ifdef MACB_RBQPH
    uint64_t cur  = ((uint64_t)macb_readl(&ad->hw, MACB_RBQPH) << 32) |
                     (uint64_t)macb_readl(&ad->hw, MACB_RBQP);
#else
    uint64_t cur  = (uint64_t)macb_readl(&ad->hw, MACB_RBQP);
#endif
    uint32_t off  = (uint32_t)(cur - base);
    const size_t stride = macb_desc_stride_bytes(rxq->hw_dma_cap);

    uint32_t didx = off / stride;

    //RTE_LOG(INFO, PMD,
    //    "macb: RX[%s] head: RBQP=%08x:%08x base=%08x:%08x -> off=%u idx=%u\n",
    //    tag,
    //    (uint32_t)(cur >> 32), (uint32_t)cur,
    //    (uint32_t)(base >> 32), (uint32_t)base,
    //    off, didx);
}	

static void macb_rx_dump_at_hw(struct macb_rxq *rxq, const char *why)
{
    const uint16_t i = macb_rbqp_hw_idx(rxq);
    const unsigned cap = rxq->hw_dma_cap;

    volatile struct macb_desc *dv =
        (volatile struct macb_desc *)macb_desc_at((void *)rxq->ring, cap, i);

    uint32_t w0, w1;
    macb_read_desc_words_cap(dv, &w0, &w1, rxq->sync_fd, cap);

#if defined(MACB_PINNED_RX_LAYOUT_W1_ADDR_W0_STAT)
    const uint32_t addr = w1, stat = w0;
#else
    const uint32_t addr = w0, stat = w1;
#endif

    //RTE_LOG(INFO, PMD,
    //    "RX[%s] RBQP->idx=%u d%03u: w0=%08x w1=%08x | ADDR=%08x USED=%u WRAP=%u LEN=%u SOF=%u EOF=%u\n",
    //    why ? why : "at-hw",
    //    i, i, w0, w1, (addr & ~0x3u),
    //    !!(addr & RX_USED), !!(addr & RX_WRAP),
    //    (stat & RX_LEN_MASK), !!(stat & RX_SOF), !!(stat & RX_EOF));
}

static __rte_always_inline void
aarch64_invalidate_range(void *addr, size_t len)
{
#if defined(__aarch64__)
    uintptr_t p = (uintptr_t)addr;
    uintptr_t s = p & ~(uintptr_t)(MACB_CL_SIZE - 1);
    uintptr_t e = (p + len + (MACB_CL_SIZE - 1)) & ~(uintptr_t)(MACB_CL_SIZE - 1);

    for (uintptr_t x = s; x < e; x += MACB_CL_SIZE)
        __asm__ __volatile__("dc ivac, %0" :: "r"(x) : "memory");

    __asm__ __volatile__("dsb sy" ::: "memory");
    __asm__ __volatile__("isb" ::: "memory");
#else
    (void)addr; (void)len;
#endif
}


/* ------------------------------------------------------------------------- */
/* Sentinel test: prove CPU->ring writes + prove HW overwrites ring words     */
/* ------------------------------------------------------------------------- */

static inline void
macb_rx_write_sentinel(struct macb_rxq *rxq, uint16_t idx, uint32_t val)
{
    const unsigned cap = rxq->hw_dma_cap;
    volatile struct macb_desc *dv =
        (volatile struct macb_desc *)macb_desc_at((void *)rxq->ring, cap, idx);

    /* Layout pinned by compile-time define:
     *  - default: w0=addr, w1=stat
     *  - MACB_PINNED_RX_LAYOUT_W1_ADDR_W0_STAT: w1=addr, w0=stat
     *
     * We write the *STAT* word with a sentinel pattern.
     */
#if defined(MACB_PINNED_RX_LAYOUT_W1_ADDR_W0_STAT)
    ((volatile uint32_t *)dv)[0] = val; /* STAT */
#else
    ((volatile uint32_t *)dv)[1] = val; /* STAT */
#endif

    /* Ensure the store is visible before flushing/syncing */
    rte_io_wmb();

    /* Flush the whole descriptor stride (covers any ext words too) */
    sync_desc_to_dev_v(rxq->sync_fd, dv, macb_desc_stride_bytes(cap));
    rte_io_wmb();
}

static inline uint32_t
macb_rx_read_sentinel(struct macb_rxq *rxq, uint16_t idx)
{
    const unsigned cap = rxq->hw_dma_cap;
    volatile struct macb_desc *dv =
        (volatile struct macb_desc *)macb_desc_at((void *)rxq->ring, cap, idx);

    uint32_t w0 = 0, w1 = 0;
    macb_read_desc_words_cap(dv, &w0, &w1, rxq->sync_fd, cap);

#if defined(MACB_PINNED_RX_LAYOUT_W1_ADDR_W0_STAT)
    return w0; /* STAT */
#else
    return w1; /* STAT */
#endif
}

static void
macb_rx_sentinel_dump(struct macb_rxq *rxq, const char *tag)
{
    const unsigned cap = rxq->hw_dma_cap;

    RTE_LOG(INFO, PMD, "RX[sentinel:%s] cap=0x%x stride=%zu\n",
            tag ? tag : "-", cap, (size_t)macb_desc_stride_bytes(cap));

    for (uint16_t i = 0; i < 4 && i < rxq->nb_desc; i++) {
        volatile struct macb_desc *dv =
            (volatile struct macb_desc *)macb_desc_at((void *)rxq->ring, cap, i);

        uint32_t w0 = 0, w1 = 0;
        macb_read_desc_words_cap(dv, &w0, &w1, rxq->sync_fd, cap);

        RTE_LOG(INFO, PMD,
            "RX[sentinel:%s] d%03u dv=%p raw w0=%08x w1=%08x (ADDR=%08x STAT=%08x)\n",
            tag ? tag : "-",
            i, (void *)(uintptr_t)dv, w0, w1,
            RX_PICK_ADDR(w0, w1), RX_PICK_STAT(w0, w1));
    }
}

static inline void
macb_rx_log_buf_map(struct macb_rxq *rxq, uint16_t i,
                    uint32_t w0_addr_word, struct rte_mbuf *m, uint16_t len)
{
    uint32_t bus_lo = (w0_addr_word & ~0x3u);
    uint32_t exp_lo = (uint32_t)BUS_IOVA(rte_mbuf_data_iova(m));

    uint8_t *va_data = (uint8_t *)m->buf_addr + RTE_PKTMBUF_HEADROOM;

    RTE_LOG(INFO, PMD,
        "macb: RXMAP i=%u len=%u bus_lo=%08x exp_lo=%08x va_data=%p buf_addr=%p buf_iova=%"PRIx64" data_iova=%"PRIx64"\n",
        i, len,
        bus_lo, exp_lo,
        va_data, m->buf_addr,
        (uint64_t)m->buf_iova,
        (uint64_t)rte_mbuf_data_iova(m));
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

#if MACB_RX_BUF_PREP
    /* Clean (and optionally poison) the RX area before handing to HW */
    void *data = rte_pktmbuf_mtod(m_new, void *);
    size_t room = rte_pktmbuf_data_room_size(rxq->mp) - RTE_PKTMBUF_HEADROOM;
    if (room > (size_t)(m_new->buf_len - RTE_PKTMBUF_HEADROOM))
        room = (size_t)(m_new->buf_len - RTE_PKTMBUF_HEADROOM);

    size_t prep = (size_t)MACB_RX_BUF_PREP_BYTES;
    if (prep > room) prep = room;

    //memset(data, 0xA5, prep);
    (void)sync_buf_to_dev_s(rxq->sync_fd, data, prep);
    rte_io_wmb();
#endif

    const uint64_t bus = (uint64_t)BUS_IOVA(diova);
    const uint32_t wrap = (i == (rxq->nb_desc - 1)) ? 1u : 0u;

    /* publish new buffer into the SAME descriptor slot */
    macb_rx_publish_desc(rxq, dv, bus, wrap);

    /* Swap */
    rxq->sw_ring[i] = m_new;
    *out_old = m_old;
    return 0;
}


static void macb_dump_ring_bytes(struct macb_rxq *rxq, unsigned bytes)
{
    const volatile uint32_t *p = (const volatile uint32_t *)rxq->ring;
    unsigned words = bytes / 4;

    RTE_LOG(INFO, PMD, "RX[bytes] ring @%p iova=%"PRIx64" first %u bytes:\n",
            rxq->ring, (uint64_t)rxq->ring_iova, bytes);

    for (unsigned i = 0; i < words; i += 4) {
        RTE_LOG(INFO, PMD, "  +%02x: %08x %08x %08x %08x\n",
                i*4, p[i], p[i+1], p[i+2], p[i+3]);
    }
}

/* ------------------------------- RX init ---------------------------------- */
static inline uint32_t rx_arm_addr_word(uint32_t iova, uint32_t wrap) {
    uint32_t w = (iova & ~0x3u);
    if (wrap) w |= RX_WRAP;
    return w & ~RX_USED;
}

static inline void macb_rx_desc_clear_ext(struct macb_rxq *rxq,
                                         volatile struct macb_desc *dv)
{
    /* Clear extensions if present */
    if (rxq->hw_dma_cap & HW_DMA_CAP_64B) {
        volatile struct macb_desc_64 *d64 = macb_desc64_ptr(dv, rxq->hw_dma_cap);
        d64->addrh = 0;
        d64->resvd = 0;
    }
    if (rxq->hw_dma_cap & HW_DMA_CAP_PTP) {
        volatile struct macb_desc_ptp *ptp = macb_descptp_ptr(dv, rxq->hw_dma_cap);
        ptp->ts_1 = 0;
        ptp->ts_2 = 0;
    }
}

/* Fresh RX init that *always* clears any extension words present in the ring,
 * based on stride (not on cap flags), so w2..w5 can never contain poison.
 */
void
macb_rx_init(struct macb_rxq *rxq)
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

#if defined(MACB_PINNED_RX_LAYOUT_W1_ADDR_W0_STAT)
        w[0] = 0;              /* STAT */
        w[1] = RX_USED | wrap; /* ADDR (SW owns) */
#else
        w[0] = RX_USED | wrap; /* ADDR (SW owns) */
        w[1] = 0;              /* STAT */
#endif

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

    /* Optional: one-shot sanity (descriptor 0 should have w2..w5 = 0) */
    {
        volatile struct macb_desc *d0 =
            (volatile struct macb_desc *)macb_desc_at((void *)rxq->ring, cap, 0);
        volatile uint32_t *w = (volatile uint32_t *)d0;
        RTE_LOG(INFO, PMD,
                "macb: RX init d0 raw: w0=%08x w1=%08x w2=%08x w3=%08x w4=%08x w5=%08x (stride=%zu cap=0x%x)\n",
                w[0], w[1],
                (stride >= 16) ? w[2] : 0,
                (stride >= 16) ? w[3] : 0,
                (stride >= 24) ? w[4] : 0,
                (stride >= 24) ? w[5] : 0,
                stride, cap);
    }
}

uint16_t
macb_rx_burst(void *queue, struct rte_mbuf **rx_pkts, uint16_t nb_pkts)
{
    struct macb_rxq *rxq = (struct macb_rxq *)queue;
    uint16_t nb = 0;

    if (unlikely(nb_pkts == 0))
        return 0;

    /* 1s lightweight log (optional) */
    {
        static uint64_t last_tsc;
        const uint64_t now = rte_get_timer_cycles();
        if (now - last_tsc >= rte_get_timer_hz()) {
            last_tsc = now;
            uint32_t ncr   = macb_readl(&rxq->ad->hw, MACB_NCR);
            uint32_t ncfgr = macb_readl(&rxq->ad->hw, MACB_NCFGR);
            uint32_t nsr   = macb_readl(&rxq->ad->hw, MACB_NSR);
            RTI("RX[1s] NCR=%08x NCFGR=%08x NSR=%08x", ncr, ncfgr, nsr);
            macb_rx_check_rbqp_stride(rxq);
        }
    }

    while (nb < nb_pkts) {
        const uint16_t i = rxq->rx_tail;
        const unsigned cap = rxq->hw_dma_cap;
        volatile struct macb_desc *dv = macb_desc_at(rxq->ring, cap, i);

        /* Read descriptor */
        uint32_t w0, w1;
        macb_read_desc_words_cap(dv, &w0, &w1, rxq->sync_fd, cap);

        uint32_t addr = RX_PICK_ADDR(w0, w1);
        uint32_t stat = RX_PICK_STAT(w0, w1);

        /* Not ready until HW sets USED in ADDR */
        if ((addr & RX_USED) == 0) {
            uint32_t rsr = macb_readl(&rxq->ad->hw, MACB_RSR);
            if (rsr & MACB_RSR_BNA)
                macb_rx_dump_bna_snapshot(rxq, rsr, "RSR.BNA while idle");
            break;
        }

        /* Optional debug: raw descriptor */
        macb_dump_rx_desc_raw(rxq, i, "COMPLETE");

        /* Defensive re-read (keeps your original behavior) */
        rte_io_rmb();
        macb_read_desc_words_cap(dv, &w0, &w1, rxq->sync_fd, cap);
        addr = RX_PICK_ADDR(w0, w1);
        stat = RX_PICK_STAT(w0, w1);

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

        /* Compute VA/room from the OLD mbuf (the packet we are returning/dropping) */
        uint8_t *va   = (uint8_t *)m_old->buf_addr + RTE_PKTMBUF_HEADROOM;
        size_t room   = m_old->buf_len - RTE_PKTMBUF_HEADROOM;

        /* Clamp */
        if (unlikely(len > room))
            len = (uint16_t)room;

        /* Helpful sanity: does descriptor bus match OLD mbuf data iova? */
        {
            uint32_t d_bus_lo = (addr & ~0x3u);
            rte_iova_t diova  = rte_mbuf_data_iova(m_old);
            uint32_t m_bus_lo = ((uint32_t)BUS_IOVA(diova)) & ~0x3u;

            RTI("RX MAP i=%u addr_word=%08x stat=%08x d_bus=%08x m_bus=%08x m_old=%p buf=%p data_off=%u buf_iova=%#"PRIx64" data_iova=%#"PRIx64"\n",
                i, addr, stat, d_bus_lo, m_bus_lo,
                (void*)m_old, m_old->buf_addr, m_old->data_off,
                (uint64_t)m_old->buf_iova, (uint64_t)diova);

            if (unlikely(d_bus_lo != m_bus_lo)) {
                RTE_LOG(ERR, PMD,
                    "macb: RX bus mismatch i=%u desc=%08x mbuf=%08x (addr=%08x stat=%08x)\n",
                    i, d_bus_lo, m_bus_lo, addr, stat);
            }
        }

        /*
         * Sync payload (your existing approach).
         * Note: this may still be wrong on your platform, but it doesn't affect BNA anymore.
         */
        {
            const uint64_t desc_dma = (uint64_t)(addr & ~0x3u);
            const uint64_t cpu_pa_guess = (g_bus_ofs ? (desc_dma - (uint64_t)g_bus_ofs) : 0);

            RTD("RX PAYLOAD pre-sync i=%u va=%p desc_dma=0x%llx cpu_pa_guess=0x%llx: "
                "%02x %02x %02x %02x %02x %02x %02x %02x\n",
                i, va,
                (unsigned long long)desc_dma,
                (unsigned long long)cpu_pa_guess,
                va[0], va[1], va[2], va[3], va[4], va[5], va[6], va[7]);

            int rc_dma = sync_buf_from_dev_dma(rxq->sync_fd, desc_dma, room);
            int rc_va  = sync_buf_from_dev(rxq->sync_fd, (const volatile void *)va, room);

            rte_io_rmb();

            RTD("dma_sync FROM_DEV i=%u rc_dma=%d rc_va=%d errno=%d room=%zu desc_dma=0x%llx va=%p\n",
                i, rc_dma, rc_va, errno, room,
                (unsigned long long)desc_dma, va);

            RTD("RX PAYLOAD post-sync i=%u va=%p: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                i, va,
                va[0], va[1], va[2], va[3], va[4], va[5], va[6], va[7]);
        }

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

        /* Filter test packet */
        if (unlikely(!macb_is_my_test_pkt(m_old))) {
            macb_dump_mbuf(m_old, "RX mbuf (DROP)");
            rte_pktmbuf_free(m_old);
            rxq->rx_tail = macb_ring_next(rxq->rx_tail, rxq->nb_desc);
            continue;
        }

        macb_dump_mbuf(m_old, "RX mbuf (PASS)");
        rx_pkts[nb++] = m_old;

        rxq->rx_tail = macb_ring_next(rxq->rx_tail, rxq->nb_desc);
    }

    /*
     * With swap-rearm, refill is no longer critical to avoid BNA,
     * but keep it for extra buffering / recovery if other code paths NULL slots.
     */
    macb_rx_refill(rxq);

#ifdef MACB_RSR
    {
        uint32_t rsr = macb_readl(&rxq->ad->hw, MACB_RSR);
        if (rsr) {
            if (rsr & MACB_RSR_BNA)
                macb_rx_dump_bna_snapshot(rxq, rsr, "RSR.BNA");

            macb_rsr_decode(rsr);

            macb_log_hw_rx_head(rxq->ad, "rsr");
            macb_rx_audit_ring(rxq, 32, "rsr");

            macb_writel(&rxq->ad->hw, MACB_RSR, rsr); /* W1C */
            rte_io_wmb();
        }
    }
#endif

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

    RTE_LOG(INFO, PMD, "TX[entry]: nb_pkts=%u ring_size=%u cons=%u prod=%u used=%u free=%u \n",
        nb_pkts, nb_desc, cons, prod, used, free);

    if (free == 0) {
        RTE_LOG(WARNING, PMD, "TX[entry]: ring full, nothing sent \n");
        return 0;
    }
    if (nb_pkts > free) {
        RTE_LOG(INFO, PMD, "TX[throttle]: capping burst from %u to free=%u \n", nb_pkts, free);
        nb_pkts = free;
    }

    const uint16_t first_idx = prod;

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

        RTE_LOG(INFO, PMD, "TX[publish] idx=%u addr=%08x ctrl=%08x (WRAP=%u) \n",
                prod, w[0], w[1], !!wrap);

        if (++prod == nb_desc)
            prod = 0;
    }

    rte_io_wmb();
    macb_tx_kick(txq);

    uint32_t tsr = macb_tsr_snap_and_clear(ad);
    macb_tsr_decode(tsr);
    if (tsr & MACB_TSR_UBR)
        macb_tx_recover_underrun(txq, "post-publish");

    macb_tx_check_wrap(txq, "after-publish");

    RTE_LOG(INFO, PMD, "TX[summary]: published=%u first_idx=%u next_prod=%u cons=%u \n",
        sent, first_idx, prod, cons);

    txq->prod = prod;
    (void)macb_tx_reclaim(txq);
    ad->sw.tx_pkts += sent;
    return sent;
}
