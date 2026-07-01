/* macb_rx_audit.h — RX descriptor/audit helpers (layout-aware, non-conflicting)
 *
 * Include this after macb_hw.h. It prints clear RX descriptor state and a ring summary.
 * If sync_desc_from_dev() isn't defined by the translation unit, we provide a
 * harmless no-op fallback so this header can be included from macb_ethdev.c too.
 */

#ifndef MACB_RX_AUDIT_H
#define MACB_RX_AUDIT_H

#include <stdint.h>
#include <stddef.h>
#include <rte_log.h>
#include <rte_io.h>
#include <rte_common.h>   /* RTE_SET_USED */

/* Must be included before this header in the TU that uses it */
#include "macb_hw.h"      /* struct macb_desc, struct macb_rxq, RX_* bits, macb_readl(), regs */

/* Ensure logtype exists */
#ifndef RTE_LOGTYPE_PMD
#define RTE_LOGTYPE_PMD RTE_LOGTYPE_USER1
#endif

/* ---------- Fallback for cache sync if the TU didn't define it ---------- */
#ifndef sync_desc_from_dev
static inline void macb_audit_sync_from_dev_stub(int fd, const volatile void *addr, size_t len)
{
    RTE_SET_USED(fd);
    RTE_SET_USED(addr);
    RTE_SET_USED(len);
    /* Best effort ordering barrier; real sync provided by rxtx/ethdev TUs */
    rte_io_mb();
}
#define sync_desc_from_dev(fd, ptr, len) macb_audit_sync_from_dev_stub((fd), (ptr), (len))
#endif

/* ---------- RX layout helpers (shared with datapath) -------------------- */
/* If compile already defined a layout/pickers, reuse them. */
#ifndef RX_PICK_ADDR
# ifdef MACB_PINNED_RX_LAYOUT_W1_ADDR_W0_STAT
  /* W0 = STAT, W1 = ADDR(+USED/WRAP) */
#  define RX_PICK_ADDR(w0, w1)   (w1)
#  define RX_PICK_STAT(w0, w1)   (w0)
#  define RX_LAYOUT_STR          "w1=addr,w0=stat"
# else
  /* Legacy: W0 = ADDR(+USED/WRAP), W1 = STAT */
#  define RX_PICK_ADDR(w0, w1)   (w0)
#  define RX_PICK_STAT(w0, w1)   (w1)
#  define RX_LAYOUT_STR          "w0=addr,w1=stat"
# endif
#endif

static inline volatile struct macb_desc *
macb_rx_desc_v(const struct macb_rxq *rxq, uint16_t idx)
{
    return macb_desc_at((void *)rxq->ring, rxq->hw_dma_cap, idx);
}

/* ---------- Local (non-conflicting) tiny helpers ------------------------ */

static inline void
macb_audit_read_desc_words2(const struct macb_rxq *rxq,
                            volatile struct macb_desc *dv,
                            uint32_t *w0, uint32_t *w1)
{
    const size_t stride = rxq->desc_stride ? rxq->desc_stride
                                          : macb_desc_stride_bytes(rxq->hw_dma_cap);

    sync_desc_from_dev(rxq->sync_fd, (const volatile void *)dv, stride);
    rte_io_mb();

    const volatile uint32_t *p = (const volatile uint32_t *)dv;
    *w0 = p[0];
    *w1 = p[1];
}

static inline void
macb_audit_read_rbqp(const struct macb_rxq *rxq, uint32_t *hi, uint32_t *lo)
{
#ifdef MACB_RBQPH
    *hi = macb_readl(&rxq->ad->hw, MACB_RBQPH);
#else
    *hi = 0;
#endif
    *lo = macb_readl(&rxq->ad->hw, MACB_RBQP);
    rte_io_rmb();
}

/* ---------- Pretty-print one descriptor --------------------------------- */
static inline void
macb_rx_dbg_one(struct macb_rxq *rxq, uint16_t idx, const char *tag)
{
    volatile struct macb_desc *dv = macb_rx_desc_v(rxq, idx);

    uint32_t w0, w1;
    macb_audit_read_desc_words2(rxq, dv, &w0, &w1);

    const uint32_t addr_raw  = RX_PICK_ADDR(w0, w1);
    const uint32_t stat      = RX_PICK_STAT(w0, w1);
    const uint32_t addr_iova = (addr_raw & ~0x3u);

    const int used   = !!(addr_raw & RX_USED);  /* 1 -> SW owns, 0 -> HW owns */
    const int wrap   = !!(addr_raw & RX_WRAP);
    const int hw_own = !used;

    const int sof     = !!(stat & RX_SOF);
    const int eof     = !!(stat & RX_EOF);
    const uint16_t len = (uint16_t)(stat & RX_LEN_MASK);

    RTE_LOG(INFO, PMD,
        "macb: RX[%s] d%03u: w0=%08x w1=%08x | addr=%08x USED=%d OWN=%s WRAP=%d len=%u sof=%d eof=%d\n",
        tag ? tag : "-", idx, w0, w1, addr_iova, used, hw_own ? "HW" : "SW",
        wrap, len, sof, eof);
}

/* ---------- Audit N descriptors and print a compact summary -------------- */
static inline void
macb_rx_audit_ring(struct macb_rxq *rxq, uint16_t n, const char *why)
{
    if (!rxq || !rxq->ring || rxq->nb_desc == 0)
        return;

    if (n > rxq->nb_desc)
        n = rxq->nb_desc;

    uint32_t rbqph = 0, rbqp = 0;
    macb_audit_read_rbqp(rxq, &rbqph, &rbqp);

    unsigned c_hw = 0, c_sw = 0, c_sof = 0, c_eof = 0, c_len = 0;

    for (uint16_t i = 0; i < n; i++) {
        volatile struct macb_desc *dv = macb_rx_desc_v(rxq, i);

        uint32_t w0, w1;
        macb_audit_read_desc_words2(rxq, dv, &w0, &w1);

	uint32_t addrA = RX_PICK_ADDR(w0,w1);
	uint32_t statA = RX_PICK_STAT(w0,w1);
	//uint32_t addrB = RX_PICK_ADDR(w1,w0);
	//uint32_t statB = RX_PICK_STAT(w1,w0);
        
	const int used   = !!(addrA & RX_USED);
        const int sof    = !!(statA & RX_SOF);
        const int eof    = !!(statA & RX_EOF);
        const uint16_t l = (uint16_t)(statA & RX_LEN_MASK);

        if (used) c_sw++; else c_hw++;
        if (sof)  c_sof++;
        if (eof)  c_eof++;
        if (l)    c_len++;

        /* Dump first few descriptors verbosely for bearings */
        if (i < 16)
            macb_rx_dbg_one(rxq, i, why ? why : "audit");

    RTE_LOG(INFO, PMD,
  "macb: RX[rsr] d%03u A(addr=%08x stat=%08x used=%d len=%u sof=%d eof=%d)\n",
  i,
  addrA, statA, !!(addrA & RX_USED), (unsigned)(statA & RX_LEN_MASK),
  !!(statA & RX_SOF), !!(statA & RX_EOF));

   }
}

#endif /* MACB_RX_AUDIT_H */
