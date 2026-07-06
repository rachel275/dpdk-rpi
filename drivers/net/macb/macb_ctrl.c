/*
 * macb_ctrl.c - Hardware control sequences for Cadence GEM driver
 */

#include <string.h>
#include <rte_log.h>
#include <rte_io.h>
#include <rte_dev.h>

#include "macb_hw.h"
#include "macb_ctrl.h"
#include "macb_dma_sync.h"

/* TX descriptor bits */
#define TX_USED  (1u << 31)
#define TX_WRAP  (1u << 30)

/* NCR bits */
#define NCR_RXEN   (1u << 2)
#define NCR_TXEN   (1u << 0)
#define NCR_MPE    (1u << 4)
#define NCR_TSTART (1u << 3)

/* Loopback function (from macb_phy.c) */
extern void macb_apply_loopback(struct macb_adapter *ad);

/*
 * Forward reference to structs from macb_hw.h
 */
struct macb_adapter;
struct macb_rxq;
struct macb_txq;

int macb_map_mempool(struct rte_device *rdev, struct rte_mempool *mp)
{
    struct rte_mempool_memhdr *hdr;
    int mapped = 0;

    STAILQ_FOREACH(hdr, &mp->mem_list, next) {
        void *va = hdr->addr;
        size_t len = hdr->len;
        rte_iova_t iova = rte_mem_virt2iova(va);
        if (iova == RTE_BAD_IOVA) {
            RTE_LOG(ERR, PMD, "mempool seg has BAD IOVA va=%p len=%zu\n", va, len);
            return -EINVAL;
        }

        int rc = rte_dev_dma_map(rdev, va, BUS_IOVA_DATA(iova), len);
        if (rc == -ENOTSUP || rc == -EINVAL)
            rc = 0;
        if (rc) {
            RTE_LOG(ERR, PMD, "dma_map mempool seg failed rc=%d va=%p len=%zu iova=%#"PRIx64"\n",
                    rc, va, len, (uint64_t)iova);
            return rc;
        }
        mapped++;
    }

    RTE_LOG(INFO, PMD, "mempool DMA mapped: %d segments\n", mapped);
    return 0;
}

/* ======== Control Functions ======== */

void macb_hw_disable_rxtx(struct macb_adapter *ad)
{
    uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
    ncr &= ~(NCR_TXEN | NCR_RXEN);
    macb_writel(&ad->hw, MACB_NCR, ncr);
    rte_io_wmb();
    (void)macb_readl(&ad->hw, MACB_NCR);
}

void macb_hw_reset_status_regs(struct macb_adapter *ad)
{
#ifdef MACB_RSR
    macb_writel(&ad->hw, MACB_RSR, 0xffffffffu);
#endif
#ifdef MACB_TSR
    macb_writel(&ad->hw, MACB_TSR, 0xffffffffu);
#endif
    rte_io_wmb();
}

int macb_hw_reset_tx_ring(struct macb_txq *txq)
{
    const uint16_t nb = txq->nb_desc;
    
    /* Set all TX descriptors to FREE (USED=1) with WRAP on last */
    for (uint16_t i = 0; i < nb; i++) {
        const uint32_t wrap = (i == nb - 1) ? TX_WRAP : 0;
        volatile uint32_t *w = (volatile uint32_t *)macb_desc_at(txq->ring, txq->hw_dma_cap, i);
        memset((void *)(uintptr_t)w, 0, txq->desc_stride);
        w[1] = TX_USED | wrap;
    }

    /* Sync all descriptors to device */
    sync_desc_to_dev(txq->sync_fd, txq->ring,
                     (size_t)txq->nb_desc * (size_t)txq->desc_stride);
    rte_io_wmb();

    return 0;
}

int macb_hw_program_ring_ptrs(struct macb_adapter *ad,
                              struct macb_rxq *rxq,
                              struct macb_txq *txq)
{
    /* RP1 GEM: real hi pointer regs */
#define RP1_GEM_TBQPH   0x04C8u
#define RP1_GEM_RBQPH   0x04D4u

    const uint64_t rb = BUS_IOVA(rxq->ring_iova);
    const uint64_t tb = BUS_IOVA(txq->ring_iova);

    const uint32_t rb_lo = (uint32_t)(rb & 0xffffffffu);
    const uint32_t tb_lo = (uint32_t)(tb & 0xffffffffu);
    const uint32_t rb_hi = (uint32_t)(rb >> 32);
    const uint32_t tb_hi = (uint32_t)(tb >> 32);

    /* Decide whether HW is in 64-bit mode (ADDR64) */
    uint32_t dmacfg = 0;
#ifdef GEM_DMACFG
    dmacfg = macb_readl(&ad->hw, GEM_DMACFG);
#endif
    const int want_hi = (tb_hi | rb_hi) != 0;
    const int addr64  = !!(dmacfg & (1u << 30)); /* ADDR64 */

    /* 1) Program *real* RP1 hi regs (0x04C8/0x04D4) if we need them */
    if (want_hi || addr64) {
        macb_writel(&ad->hw, RP1_GEM_TBQPH, tb_hi);
        macb_writel(&ad->hw, RP1_GEM_RBQPH, rb_hi);
    }

    /* 2) Program legacy low regs (some GEM paths still use these) */
    macb_writel(&ad->hw, MACB_RBQP, rb_lo);
    macb_writel(&ad->hw, MACB_TBQP, tb_lo);

    /* Program RP1 queue0 low regs if we discovered them */
    if (ad->rp1_rbqb0_off != 0xffffffffu) {
        macb_writel(&ad->hw, ad->rp1_rbqb0_off, (uint32_t)(rb & 0xffffffffu));
    }
    if (ad->rp1_tbqb0_off != 0xffffffffu) {
        macb_writel(&ad->hw, ad->rp1_tbqb0_off, (uint32_t)(tb & 0xffffffffu));
    }

    /* Log readback */
    if (ad->rp1_rbqb0_off != 0xffffffffu || ad->rp1_tbqb0_off != 0xffffffffu) {
        uint32_t r = (ad->rp1_rbqb0_off != 0xffffffffu) ? macb_readl(&ad->hw, ad->rp1_rbqb0_off) : 0;
        uint32_t t = (ad->rp1_tbqb0_off != 0xffffffffu) ? macb_readl(&ad->hw, ad->rp1_tbqb0_off) : 0;
        RTE_LOG(INFO, PMD, "macb: q0 ptr readback: RBQB0@%04x=%08x TBQB0@%04x=%08x\n",
                (ad->rp1_rbqb0_off==0xffffffffu)?0:ad->rp1_rbqb0_off, r,
                (ad->rp1_tbqb0_off==0xffffffffu)?0:ad->rp1_tbqb0_off, t);
    }

    /* Readback: show *both* possible hi-reg locations + both low-reg paths */
    {
        const uint32_t rbqp = macb_readl(&ad->hw, MACB_RBQP);
        const uint32_t tbqp = macb_readl(&ad->hw, MACB_TBQP);

        const uint32_t rbqph_rp1 = macb_readl(&ad->hw, RP1_GEM_RBQPH);
        const uint32_t tbqph_rp1 = macb_readl(&ad->hw, RP1_GEM_TBQPH);

        /* legacy hi regs that your DPDK header currently points at */
        const uint32_t rbqph_legacy = macb_readl(&ad->hw, 0x00A4u);
        const uint32_t tbqph_legacy = macb_readl(&ad->hw, 0x00A8u);

        const uint32_t rbqb_q0 = macb_readl(&ad->hw, 0x0480u);
        const uint32_t tbqb_q0 = macb_readl(&ad->hw, 0x0484u);

        RTE_LOG(INFO, PMD,
            "macb: ring ptrs: RBQP=%08x (exp %08x) TBQP=%08x (exp %08x)\n",
            rbqp, rb_lo, tbqp, tb_lo);

        RTE_LOG(INFO, PMD,
            "macb: ring ptrs: RP1_HI RBQPH@04D4=%08x (exp %08x) TBQPH@04C8=%08x (exp %08x)\n",
            rbqph_rp1, rb_hi, tbqph_rp1, tb_hi);

        RTE_LOG(INFO, PMD,
            "macb: ring ptrs: LEGACY_HI RBQPH@00A4=%08x TBQPH@00A8=%08x (should be 0 on RP1)\n",
            rbqph_legacy, tbqph_legacy);

        RTE_LOG(INFO, PMD,
            "macb: ring ptrs: Q0 RBQB@0480=%08x (exp %08x) TBQB@0484=%08x (exp %08x)\n",
            rbqb_q0, rb_lo, tbqb_q0, tb_lo);
    }

    return 0;
}

int macb_hw_configure_dma(struct macb_adapter *ad, struct macb_rxq *rxq)
{
    /* DMA-map mempool segments if available */
    if (rxq->mp) {
        (void)macb_map_mempool(ad->device, rxq->mp);
        rte_io_wmb();
    }

    /* Program RX buffer size */
#if defined(GEM_DMACFG) && defined(GEM_DMACFG_RXBS_MASK) && defined(GEM_DMACFG_RXBS_SHIFT)
    {
        const uint16_t room = rxq->data_room_bytes;
        uint32_t units64 = room / 64u;
        if (units64 > 0xFFu) units64 = 0xFFu;

        uint32_t before = macb_readl(&ad->hw, GEM_DMACFG);
        uint32_t after  = (before & ~GEM_DMACFG_RXBS_MASK) |
                          (((uint32_t)units64 & 0xFFu) << GEM_DMACFG_RXBS_SHIFT);

        macb_writel(&ad->hw, GEM_DMACFG, after);
        rte_io_wmb();

        RTE_LOG(INFO, PMD,
                "macb: DMACFG RXBS program: room=%u bytes -> units64=%u (0x%08x -> 0x%08x)\n",
                (unsigned)room, (unsigned)units64, before, after);
    }
#endif

#ifdef GEM_DMACFG
    {
        uint32_t d = macb_readl(&ad->hw, GEM_DMACFG);
        RTE_LOG(INFO, PMD,
          "DMACFG=0x%08x RXEXT=%u TXEXT=%u ADDR64=%u ENDIA_DESC=%u ENDIA_PKT=%u RXBS=%u\n",
          d,
          !!(d & (1u<<28)),
          !!(d & (1u<<29)),
          !!(d & (1u<<30)),
          !!(d & (1u<<6)),
          !!(d & (1u<<7)),
          (d >> 16) & 0xff);
    }
#endif

    return 0;
}

void macb_hw_configure_mac(struct macb_adapter *ad)
{
    /* Set promiscuous mode (CAF) and disable broadcast (NBC) */
    uint32_t n = macb_readl(&ad->hw, MACB_NCFGR);
    n |= MACB_NCFGR_CAF;
    n &= ~MACB_NCFGR_NBC;
    macb_writel(&ad->hw, MACB_NCFGR, n);
    (void)macb_readl(&ad->hw, MACB_NCFGR);
}

uint8_t macb_hw_probe_dma_cap(struct macb_adapter *ad)
{
    uint32_t dmacfg = macb_readl(&ad->hw, GEM_DMACFG);

    uint8_t cap = HW_DMA_CAP_32B;
    if (dmacfg & (1u << 30))
        cap |= HW_DMA_CAP_64B;

    RTE_LOG(INFO, PMD, "macb: probed hw_dma_cap=0x%x (addr64=%d) dmacfg=%08x\n",
            cap, !!(cap & HW_DMA_CAP_64B), dmacfg);
    return cap;
}

void macb_hw_discover_rp1_q0_ptr_regs(struct macb_adapter *ad,
                                      uint32_t fw_rbqp_lo,
                                      uint32_t fw_tbqp_lo)
{
    const uint32_t lo = 0x0000;
    const uint32_t hi = 0x4000;

    uint32_t rb_hit = 0xffffffffu;
    uint32_t tb_hit = 0xffffffffu;

    for (uint32_t off = lo; off <= hi; off += 4) {
        uint32_t v = macb_readl(&ad->hw, off);

        if (v == fw_rbqp_lo && off != MACB_RBQP && rb_hit == 0xffffffffu)
            rb_hit = off;

        if (v == fw_tbqp_lo && off != MACB_TBQP && tb_hit == 0xffffffffu)
            tb_hit = off;

        if (rb_hit != 0xffffffffu && tb_hit != 0xffffffffu)
            break;
    }

    ad->rp1_rbqb0_off = rb_hit;
    ad->rp1_tbqb0_off = tb_hit;

    RTE_LOG(INFO, PMD,
        "macb: RP1 q0 discover: fw RBQP=0x%08x hit_off=%s fw TBQP=0x%08x hit_off=%s\n",
        fw_rbqp_lo, (rb_hit == 0xffffffffu) ? "NONE" : "FOUND",
        fw_tbqp_lo, (tb_hit == 0xffffffffu) ? "NONE" : "FOUND");

    if (rb_hit != 0xffffffffu || tb_hit != 0xffffffffu) {
        RTE_LOG(INFO, PMD,
            "macb: RP1 q0 discover offsets: RBQB0=0x%04x TBQB0=0x%04x\n",
            (rb_hit == 0xffffffffu) ? 0 : rb_hit,
            (tb_hit == 0xffffffffu) ? 0 : tb_hit);
    }
}

void macb_hw_enable_rxtx(struct macb_adapter *ad)
{
    uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
    ncr |= NCR_TXEN | NCR_MPE | NCR_RXEN;
    macb_writel(&ad->hw, MACB_NCR, ncr);
    rte_io_wmb();

    /* TSTART is often write-only/self-clearing; don't expect it to read back as 1 */
    macb_writel(&ad->hw, MACB_NCR, ncr | NCR_TSTART);
    rte_io_wmb();

    /* Log what NCR *does* read back as */
    {
        uint32_t ncr_rb = macb_readl(&ad->hw, MACB_NCR);
        RTE_LOG(INFO, PMD, "macb: NCR readback after enable/kick = 0x%08x\n", ncr_rb);
    }
}

