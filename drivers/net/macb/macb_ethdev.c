// macb_ethdev_rp1.c — DPDK vdev PMD for RP1/Cadence GEM (Raspberry Pi 5)
//
// Focus: stable external switch link under UIO userspace.
// - Default: NO loopback, PHY autoneg once, then leave it alone.
// - 64-bit ring base programming (write HIGH then LOW).
// - Proper TX kick (NCR_TSTART) after MMIO write barrier.
// - RX/TX descriptor ownership (USED bit31) consistent with macb_hw.h.
// - MDIO link reporting with real speed/duplex (Clause 22 generic).
// - Stats wired to macb_sw_stats (from your bursts), plus reset stub.
// - KV args: dev, phy_addr, phy_lb, mac_lb, bus_ofs, phy_mode.
//
// Example launch:
//   --vdev=net_macb0,dev=/dev/uio0,phy_addr=1,phy_lb=0,mac_lb=0,bus_ofs=0,phy_mode=auto
//
// NEW: phy_mode=auto|10hd|10fd|100hd|100fd

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <rte_cycles.h>
#include <rte_dev.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_interrupts.h>
#include <rte_io.h>
#include <rte_kvargs.h>
#include <rte_log.h>
#include <rte_malloc.h>
#include <rte_mbuf.h>
#include <rte_memzone.h>
#include <rte_mempool.h>
#include <rte_string_fns.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <ethdev_driver.h>
#include <ethdev_vdev.h>
#include <rte_bus_vdev.h>

#ifndef RTE_LOGTYPE_PMD
#define RTE_LOGTYPE_PMD RTE_LOGTYPE_USER1
#endif

#include "macb_hw.h"         /* NIC regs/desc + struct macb_adapter/rxq/txq */
#include "dma_sync_uapi.h"   /* userspace DMA sync helper ioctl API */

#define MACB_DBG(fmt, ...) RTE_LOG(INFO,  PMD, "macb: " fmt, ##__VA_ARGS__)
#define MACB_ERR(fmt, ...) RTE_LOG(ERR,   PMD, "macb: " fmt, ##__VA_ARGS__)

/* ---------------- Register shims (added only if missing) ---------------- */
#ifndef MACB_NSR
# define MACB_NSR    0x0008u
#endif
#ifndef MACB_RSR
# define MACB_RSR    0x0020u
#endif
#ifndef MACB_MAN
# define MACB_MAN    0x0034u
#endif
/* RP1/GEM_GXL high dwords */
#ifndef MACB_RBQPH
# define MACB_RBQPH  0x00A4u
#endif
#ifndef MACB_TBQPH
# define MACB_TBQPH  0x00A8u
#endif

/* RIGHT for this variant (what your dump shows) */
#define TX_USED 0x00004000u
#define TX_WRAP 0x00004000u

#ifndef TX_LEN_MASK
#define TX_LEN_MASK 0x00003FFFu
#endif

#define TX_LAST 0x00004000u  /* adjust if your dump shows a different bit */

#ifndef RX_OWN
#define RX_OWN (1u << 0)      /* ADDR bit0: 1=SW owns */
#endif
#ifndef RX_USED
# define RX_USED 0x00000001u
#endif
#ifndef RX_WRAP
# define RX_WRAP 0x00000002u
#endif

#ifndef RX_SOF
# define RX_SOF (1u << 14)
#endif
#ifndef RX_EOF
# define RX_EOF (1u << 15)
#endif
#ifndef RX_LEN_MASK
# define RX_LEN_MASK 0x1FFFu
#endif

/* -------- Bit fallbacks -------- */
#ifndef NCFGR_DRFCS
# define NCFGR_DRFCS (1u << 11)   /* Discard Rx FCS */
#endif
#ifndef NCR_LB
# define NCR_LB      (1u << 1)    /* MAC internal loopback */
#endif
#ifndef NCR_MPE
# define NCR_MPE     (1u << 4)    /* Management Port Enable (MDIO) */
#endif
#ifndef NCR_TSTART
# define NCR_TSTART  (1u << 9)    /* Start transmission from idle */
#endif

/* Fallbacks in case the PMD variant of the header doesn't define them */
#ifndef MACB_NCFGR_CAF
/* Copy All Frames (CAF) mask */
#define MACB_NCFGR_CAF (1u << 4)
#endif

#ifndef MACB_NCFGR_NBC
/* No BroadCast (NBC) mask */
#define MACB_NCFGR_NBC (1u << 5)
#endif

/* -------- Clause 22 MDIO (guarded) -------- */
#ifndef MACB_MAN_SOF_SHIFT
# define MACB_MAN_SOF_SHIFT  30
#endif
#ifndef MACB_MAN_OP_SHIFT
# define MACB_MAN_OP_SHIFT   28
#endif
#ifndef MACB_MAN_PHYA_SHIFT
# define MACB_MAN_PHYA_SHIFT 23
#endif
#ifndef MACB_MAN_REGA_SHIFT
# define MACB_MAN_REGA_SHIFT 18
#endif
#ifndef MACB_MAN_CODE_SHIFT
# define MACB_MAN_CODE_SHIFT 16
#endif
#ifndef MACB_MAN_OP_WRITE
# define MACB_MAN_OP_WRITE   (1u)
#endif
#ifndef MACB_MAN_OP_READ
# define MACB_MAN_OP_READ    (2u)
#endif
#ifndef MACB_MAN_CODE_C22
# define MACB_MAN_CODE_C22   (2u)
#endif

#ifndef MII_BMCR
# define MII_BMCR       0x00
#endif
#ifndef MII_BMSR
# define MII_BMSR       0x01
#endif
#ifndef MII_PHYSID1
# define MII_PHYSID1    0x02
#endif
#ifndef MII_PHYSID2
# define MII_PHYSID2    0x03
#endif
#ifndef MII_ADVERTISE
# define MII_ADVERTISE  0x04
#endif
#ifndef MII_LPA
# define MII_LPA        0x05
#endif
#ifndef MII_CTRL1000
# define MII_CTRL1000   0x09
#endif
#ifndef MII_STAT1000
# define MII_STAT1000   0x0A
#endif

#ifndef BMCR_RESET
# define BMCR_RESET     (1u << 15)
#endif
#ifndef BMCR_LOOPBACK
# define BMCR_LOOPBACK  (1u << 14)
#endif
#ifndef BMCR_SPEED100
# define BMCR_SPEED100  (1u << 13)
#endif
#ifndef BMCR_ANENABLE
# define BMCR_ANENABLE  (1u << 12)
#endif
#ifndef BMCR_PDOWN
# define BMCR_PDOWN     (1u << 11)
#endif
#ifndef BMCR_ISOLATE
# define BMCR_ISOLATE   (1u << 10)
#endif
#ifndef BMCR_RESTARTAN
# define BMCR_RESTARTAN (1u << 9)
#endif
#ifndef BMCR_FULLDPLX
# define BMCR_FULLDPLX  (1u << 8)
#endif

#ifndef BMSR_ANEGCOMPLETE
# define BMSR_ANEGCOMPLETE (1u << 5)
#endif
#ifndef BMSR_LSTATUS
# define BMSR_LSTATUS   (1u << 2)
#endif

/* AN abilities in ADVERTISE/LPA */
#ifndef ADVERTISE_10HALF
# define ADVERTISE_10HALF   0x0020
#endif
#ifndef ADVERTISE_10FULL
# define ADVERTISE_10FULL   0x0040
#endif
#ifndef ADVERTISE_100HALF
# define ADVERTISE_100HALF  0x0080
#endif
#ifndef ADVERTISE_100FULL
# define ADVERTISE_100FULL  0x0100
#endif
/* 1000Base-T partner abilities (STAT1000) */
#ifndef LPA_1000HALF
# define LPA_1000HALF       0x0400
#endif
#ifndef LPA_1000FULL
# define LPA_1000FULL       0x0800
#endif

/* -------- Runtime knobs / KV args -------- */
uint64_t g_bus_ofs         = 0;    /* CPU-phys → device bus offset (RP1: 0) */
static int      g_force_phy_lb    = 0;    /* default: external link (no PHY LB) */
static int      g_mac_lb          = 0;    /* default: MAC LB off */
static int      g_forced_phy_addr = -1;   /* -1 = auto-scan; else 0..31 */

/* NEW: simple PHY mode for 10/100 forcing (BMCR) */
enum macb_phy_mode { PHY_MODE_AUTO = 0, PHY_MODE_10HD, PHY_MODE_10FD, PHY_MODE_100HD, PHY_MODE_100FD };
static enum macb_phy_mode g_phy_mode = PHY_MODE_AUTO;

#ifndef RTE_VERSION_NUM
#define RTE_VERSION_NUM(a,b,c,d) (((a)<<24) | ((b)<<16) | ((c)<<8) | (d))
#endif

static inline rte_iova_t macb_mbuf_data_iova(const struct rte_mbuf *m)
{
#if RTE_VERSION >= RTE_VERSION_NUM(21,11,0,0)
    return rte_mbuf_data_iova_default(m);
#else
    return rte_pktmbuf_iova(m);
#endif
}

static uint64_t macb_calc_bus_ofs_from_tbqp(struct macb_adapter *ad, struct macb_txq *txq)
{
    uint64_t tbqp = macb_readl(&ad->hw, MACB_TBQP);
#ifdef MACB_TBQPH
    tbqp |= (uint64_t)macb_readl(&ad->hw, MACB_TBQPH) << 32;
#endif
    rte_iova_t ring_iova = rte_mem_virt2iova(txq->ring);
    return (tbqp - (uint64_t)ring_iova);
}

static inline void macb_set_bits(volatile void *io, uint32_t off, uint32_t set, uint32_t clr)
{
    uint32_t v = macb_readl(io, off);
    v |= set;
    v &= ~clr;
    macb_writel(io, off, v);
    /* read back to post the write, and log once */
    v = macb_readl(io, off);
    RTE_LOG(INFO, PMD, "macb: NCFGR now=0x%08x (CAF=%u NBC=%u)\n",
            v, !!(v & MACB_NCFGR_CAF), !!(v & MACB_NCFGR_NBC));
}

static void macb_force_promisc_allow_bcast(struct macb_adapter *ad)
{
    const uint32_t set = MACB_NCFGR_CAF;
    const uint32_t clr = MACB_NCFGR_NBC;
    macb_set_bits(&ad->hw, MACB_NCFGR, set, clr);
}

#define BUS_IOVA(x) ((rte_iova_t)((rte_iova_t)(x) + (rte_iova_t)g_bus_ofs))

/* -------- dma_sync_helper integration (per-adapter) --------------------- */
static __rte_always_inline void macb_sync_to_dev_fd(int fd, const void *addr, size_t len)
{
    if (fd < 0 || len == 0) return;
    struct dma_sync_range r = { .uaddr = (uintptr_t)addr, .len = len };
    (void)ioctl(fd, DMA_SYNC_TO_DEV, &r);
}
static __rte_always_inline void macb_sync_from_dev_fd(int fd, const void *addr, size_t len)
{
    if (fd < 0 || len == 0) return;
    struct dma_sync_range r = { .uaddr = (uintptr_t)addr, .len = len };
    (void)ioctl(fd, DMA_SYNC_FROM_DEV, &r);
}
#define sync_desc_to_dev(fd, ptr, len)    macb_sync_to_dev_fd((fd), (ptr), (len))
#define sync_desc_from_dev(fd, ptr, len)  macb_sync_from_dev_fd((fd), (ptr), (len))
#define sync_buf_to_dev(fd, ptr, len)     macb_sync_to_dev_fd((fd), (ptr), (len))
#define sync_buf_from_dev(fd, ptr, len)   macb_sync_from_dev_fd((fd), (ptr), (len))

static void macb_dma_sync_open_once(struct macb_adapter *ad)
{
    if (ad->sync_fd >= 0) return;
    const char *node = getenv("MACB_DMA_SYNC_NODE");  /* optional */
    if (!node || !*node) node = "/dev/dma_sync_eth0";
    ad->sync_fd = open(node, O_RDWR);
    if (ad->sync_fd < 0) {
        MACB_ERR("dma_sync_helper not active (open(%s) failed) — continuing without cache sync; TX/RX may stall on non-coherent SoCs\n", node);
    } else {
        MACB_DBG("dma_sync_helper active via %s\n", node);
    }
}

/* TX ring helpers */
static void macb_tx_force_free(struct macb_txq *txq)
{
    if (!txq || !txq->ring || !txq->nb_desc) return;
    const uint16_t nb = txq->nb_desc;
    for (uint16_t i = 0; i < nb; i++) {
        uint32_t wrap = (i == nb - 1) ? 0x40000000u : 0;
        txq->ring[i].addr = 0;
        txq->ring[i].ctrl = 0x80000000u | wrap;
        txq->sw_ring[i]   = NULL;
        sync_desc_to_dev(txq->sync_fd, &txq->ring[i], sizeof(txq->ring[i]));

        // Print the IOVA of the TX descriptor
        RTE_LOG(INFO, PMD, "TX[%u] descriptor[%u]: IOVA = 0x%" PRIx64 "\n", 0, i, txq->ring[i].addr);
    }
    sync_desc_to_dev(txq->sync_fd, txq->ring, nb * sizeof(txq->ring[0]));
    rte_io_wmb();
    txq->prod = 0;
    txq->cons = 0;
    RTE_LOG(INFO, PMD, "macb: TX ring reset: nb=%u (all FREE, WRAP at %u)\n", nb, nb - 1);
}

static void macb_tx_dump_head(struct macb_txq *txq, const char *tag, int n)
{
    int lim = (n > (int)txq->nb_desc) ? txq->nb_desc : n;
    for (int i = 0; i < lim; ++i) {
        struct macb_desc *d = &txq->ring[i];
        uint32_t a = d->addr, c = d->ctrl;
        RTE_LOG(INFO, PMD,
                "macb: TX[%s] d%03d: addr=0x%08x ctrl=0x%08x USED=%d WRAP=%d LEN=%u\n",
                tag, i, a, c, !!(c & 0x80000000u), !!(c & 0x40000000u), (c & 0x3FFFu));
    }
}

static void macb_dma_sync_close(struct macb_adapter *ad)
{
    if (ad->sync_fd >= 0) { close(ad->sync_fd); ad->sync_fd = -1; }
}

/* -------- MDIO helpers -------- */
static inline void macb_wait_mdio_idle(struct macb_adapter *ad)
{
    for (int i = 0; i < 1000; i++) {
        if (macb_readl(&ad->hw, MACB_NSR) & 0x4u) return;
        rte_delay_us_block(5);
    }
    MACB_ERR("MDIO idle wait timeout\n");
}
static uint16_t macb_mdio_read_c22(struct macb_adapter *ad, uint8_t phy, uint8_t reg)
{
    macb_wait_mdio_idle(ad);
    uint32_t man = ((1u << MACB_MAN_SOF_SHIFT) |
                    (MACB_MAN_OP_READ  << MACB_MAN_OP_SHIFT) |
                    ((uint32_t)phy << MACB_MAN_PHYA_SHIFT)   |
                    ((uint32_t)reg << MACB_MAN_REGA_SHIFT)   |
                    (MACB_MAN_CODE_C22 << MACB_MAN_CODE_SHIFT));
    macb_writel(&ad->hw, MACB_MAN, man);
    macb_wait_mdio_idle(ad);
    return (uint16_t)macb_readl(&ad->hw, MACB_MAN);
}
static void macb_mdio_write_c22(struct macb_adapter *ad, uint8_t phy, uint8_t reg, uint16_t val)
{
    macb_wait_mdio_idle(ad);
    uint32_t man = ((1u << MACB_MAN_SOF_SHIFT) |
                    (MACB_MAN_OP_WRITE << MACB_MAN_OP_SHIFT) |
                    ((uint32_t)phy << MACB_MAN_PHYA_SHIFT)   |
                    ((uint32_t)reg << MACB_MAN_REGA_SHIFT)   |
                    (MACB_MAN_CODE_C22 << MACB_MAN_CODE_SHIFT) |
                    (uint32_t)val);
    macb_writel(&ad->hw, MACB_MAN, man);
    macb_wait_mdio_idle(ad);
}

static int macb_map_mempool(struct rte_device *rdev, struct rte_mempool *mp)
{
    struct rte_mempool_memhdr *hdr;
    int mapped = 0;

    /* mem_list is STAILQ in newer DPDK */
    STAILQ_FOREACH(hdr, &mp->mem_list, next) {
        void *va = hdr->addr;
        size_t len = hdr->len;
        rte_iova_t iova = rte_mem_virt2iova(va);
        if (iova == RTE_BAD_IOVA) {
            RTE_LOG(ERR, PMD, "mempool seg has BAD IOVA va=%p len=%zu\n", va, len);
            return -EINVAL;
        }
        int rc = rte_dev_dma_map(rdev, va, BUS_IOVA(iova), len);
        if (rc == -ENOTSUP || rc == -EINVAL) {
            /* Platform doesn’t require mapping; treat as success. */
            rc = 0;
        }
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

static int macb_find_phy_addr(struct macb_adapter *ad, uint8_t *out_phy,
                              uint16_t *out_id1, uint16_t *out_id2)
{
    if (g_forced_phy_addr >= 0 && g_forced_phy_addr < 32) {
        uint8_t p = (uint8_t)g_forced_phy_addr;
        uint16_t id1 = macb_mdio_read_c22(ad, p, MII_PHYSID1);
        uint16_t id2 = macb_mdio_read_c22(ad, p, MII_PHYSID2);
        if (out_phy) *out_phy = p;
        if (out_id1) *out_id1 = id1;
        if (out_id2) *out_id2 = id2;
        return 0;
    }
    for (uint8_t phy = 0; phy < 32; phy++) {
        uint16_t id1 = macb_mdio_read_c22(ad, phy, MII_PHYSID1);
        uint16_t id2 = macb_mdio_read_c22(ad, phy, MII_PHYSID2);
        if (id1 != 0xffff && id1 != 0x0000) {
            if (out_phy) *out_phy = phy;
            if (out_id1) *out_id1 = id1;
            if (out_id2) *out_id2 = id2;
            return 0;
        }
    }
    return -1;
}

/* -------- PHY bring-up modes -------- */
static int macb_phy_normal_up(struct macb_adapter *ad)
{
    uint8_t phy; uint16_t id1=0, id2=0;
    if (macb_find_phy_addr(ad, &phy, &id1, &id2) != 0) return -1;

    MACB_DBG("Found PHY id1=0x%04x id2=0x%04x at addr=%u\n", id1, id2, (unsigned)phy);

    uint16_t bmcr = macb_mdio_read_c22(ad, phy, MII_BMCR);
    uint16_t adv  = macb_mdio_read_c22(ad, phy, MII_ADVERTISE);
    /* Advertise all 10/100 modes for best chance */
    adv |= (ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL);
    macb_mdio_write_c22(ad, phy, MII_ADVERTISE, adv);

    bmcr &= ~(BMCR_PDOWN | BMCR_ISOLATE | BMCR_LOOPBACK | BMCR_SPEED100 | BMCR_FULLDPLX);
    bmcr |=  (BMCR_ANENABLE | BMCR_RESTARTAN);
    macb_mdio_write_c22(ad, phy, MII_BMCR, bmcr);

    for (int i = 0; i < 200; i++) {
        uint16_t bmsr = macb_mdio_read_c22(ad, phy, MII_BMSR);
        if ((bmsr & BMSR_LSTATUS) && (bmsr & BMSR_ANEGCOMPLETE)) break;
        rte_delay_us_block(5000);
    }
    return 0;
}

/* NEW: force 10/100 speed/duplex by BMCR (no autoneg) */
static int macb_phy_force_10_100(struct macb_adapter *ad, enum macb_phy_mode mode)
{
    uint8_t phy; uint16_t id1=0, id2=0;
    if (macb_find_phy_addr(ad, &phy, &id1, &id2) != 0) return -1;

    uint16_t bmcr = macb_mdio_read_c22(ad, phy, MII_BMCR);
    uint16_t newb = bmcr;

    /* Disable autoneg, powerdown/iso/loopback off */
    newb &= ~(BMCR_ANENABLE | BMCR_PDOWN | BMCR_ISOLATE | BMCR_LOOPBACK | BMCR_RESTARTAN);
    /* Clear speed/duplex bits first */
    newb &= ~(BMCR_SPEED100 | BMCR_FULLDPLX);

    switch (mode) {
    case PHY_MODE_10HD:    break;
    case PHY_MODE_10FD:    newb |= BMCR_FULLDPLX; break;
    case PHY_MODE_100HD:   newb |= BMCR_SPEED100; break;
    case PHY_MODE_100FD:   newb |= (BMCR_SPEED100 | BMCR_FULLDPLX); break;
    default: return -EINVAL;
    }

    if (newb != bmcr) macb_mdio_write_c22(ad, phy, MII_BMCR, newb);

    MACB_DBG("Forced PHY mode %s (BMCR 0x%04x -> 0x%04x, phy=%u id=%04x:%04x)\n",
             (mode==PHY_MODE_10HD)?"10HD":(mode==PHY_MODE_10FD)?"10FD":
             (mode==PHY_MODE_100HD)?"100HD":"100FD",
             bmcr, newb, (unsigned)phy, id1, id2);

    /* Give it a moment to resolve link (partner must match exactly) */
    for (int i = 0; i < 200; i++) {
        uint16_t bmsr = macb_mdio_read_c22(ad, phy, MII_BMSR);
        if (bmsr & BMSR_LSTATUS) break;
        rte_delay_us_block(5000);
    }
    return 0;
}

/* PHY loopback toggle (unchanged) */
static int macb_phy_loopback_set(struct macb_adapter *ad, int enable)
{
    uint8_t phy; uint16_t id1=0,id2=0;
    if (macb_find_phy_addr(ad, &phy, &id1, &id2) != 0) return -1;
    uint16_t bmcr = macb_mdio_read_c22(ad, phy, MII_BMCR);
    uint16_t newbmcr = (bmcr & ~(BMCR_ANENABLE | BMCR_PDOWN | BMCR_ISOLATE)) |
                       BMCR_SPEED100 | BMCR_FULLDPLX;
    if (enable) newbmcr |=  BMCR_LOOPBACK; else newbmcr &= ~BMCR_LOOPBACK;
    if (newbmcr != bmcr) macb_mdio_write_c22(ad, phy, MII_BMCR, newbmcr);
    MACB_DBG("PHY loopback %s (BMCR 0x%04x -> 0x%04x, phy=%u id=%04x:%04x)\n",
             enable?"EN":"DIS", bmcr, newbmcr, (unsigned)phy, id1, id2);
    return 0;
}

/* ----- MAC loopback helper (off by default) ----- */
/* ----- MAC loopback helper (safer) ----- */
#ifndef NCFGR_LBL
/* Some GEM variants name it LBL/LLB; keep it optional */
#define NCFGR_LBL 0
#endif

static void macb_mac_loopback_set(struct macb_adapter *ad, int enable)
{
    uint32_t ncfgr0 = macb_readl(&ad->hw, MACB_NCFGR);
    uint32_t ncr0   = macb_readl(&ad->hw, MACB_NCR);

    uint32_t ncfgr  = ncfgr0;
    uint32_t ncr    = ncr0;

    /* Keep Copy-All for loopback; avoid forcing DRFCS. */
#ifdef NCFGR_CAF
    if (enable) ncfgr |= NCFGR_CAF;
    else        ncfgr &= ~NCFGR_CAF;
#endif

    /* Primary loopback enable: NCR.LB (MAC internal loopback) */
#ifdef NCR_LB
    if (enable) ncr |= NCR_LB; else ncr &= ~NCR_LB;
#endif

    /* Optional: some GEMs also have NCFGR.LBL */
#if NCFGR_LBL
    if (enable) ncfgr |= NCFGR_LBL; else ncfgr &= ~NCFGR_LBL;
#endif

    macb_writel(&ad->hw, MACB_NCFGR, ncfgr);
    /* Ensure RX/TX remain enabled */
    ncr |= (NCR_RXEN | NCR_TXEN);
    macb_writel(&ad->hw, MACB_NCR,   ncr);

    MACB_DBG("MAC loopback %s: NCFGR 0x%08x->0x%08x, NCR 0x%08x->0x%08x\n",
             enable ? "EN" : "DIS", ncfgr0, ncfgr, ncr0, ncr);
}

/* Apply PHY+MAC loopback selections */
static void macb_apply_loopback(struct macb_adapter *ad)
{
    (void)macb_phy_loopback_set(ad, g_force_phy_lb ? 1 : 0);
    macb_mac_loopback_set(ad, g_mac_lb ? 1 : 0);
}

/* -------- Ethdev ops decl -------- */
static int macb_dev_configure(struct rte_eth_dev *dev);
static int macb_dev_start(struct rte_eth_dev *dev);
static int macb_dev_stop(struct rte_eth_dev *dev);
static int macb_dev_close(struct rte_eth_dev *dev);
static int macb_promiscuous_enable(struct rte_eth_dev *dev);
static int macb_promiscuous_disable(struct rte_eth_dev *dev);
static int macb_info_get(struct rte_eth_dev *dev, struct rte_eth_dev_info *di);
static int macb_rx_queue_setup(struct rte_eth_dev *dev, uint16_t qid,
                               uint16_t nb_desc, unsigned int,
                               const struct rte_eth_rxconf *rx_conf,
                               struct rte_mempool *mp);
static int macb_tx_queue_setup(struct rte_eth_dev *dev, uint16_t qid,
                               uint16_t nb_desc, unsigned int,
                               const struct rte_eth_txconf *tx_conf);
static int macb_rx_queue_start(struct rte_eth_dev *dev, uint16_t qid);
static int macb_rx_queue_stop (struct rte_eth_dev *dev, uint16_t qid);
static int macb_tx_queue_start(struct rte_eth_dev *dev, uint16_t qid);
static int macb_tx_queue_stop (struct rte_eth_dev *dev, uint16_t qid);
static int macb_link_update    (struct rte_eth_dev *dev, int wait);
static int macb_mac_addr_set   (struct rte_eth_dev *dev, struct rte_ether_addr *ea);
static int macb_stats_get      (struct rte_eth_dev *dev, struct rte_eth_stats *stats);
static int macb_stats_reset    (struct rte_eth_dev *dev);

/* -------- dev ops table -------- */
static const struct eth_dev_ops macb_ops = {
    .dev_configure      = macb_dev_configure,
    .dev_start          = macb_dev_start,
    .dev_stop           = macb_dev_stop,
    .dev_close          = macb_dev_close,
    .promiscuous_enable = macb_promiscuous_enable,
    .promiscuous_disable= macb_promiscuous_disable,
    .dev_infos_get      = macb_info_get,
    .rx_queue_setup     = macb_rx_queue_setup,
    .tx_queue_setup     = macb_tx_queue_setup,
    .rx_queue_start     = macb_rx_queue_start,
    .rx_queue_stop      = macb_rx_queue_stop,
    .tx_queue_start     = macb_tx_queue_start,
    .tx_queue_stop      = macb_tx_queue_stop,
    .link_update        = macb_link_update,
    .mac_addr_set       = macb_mac_addr_set,
    .stats_get          = macb_stats_get,
    .stats_reset        = macb_stats_reset,
};

/* -------- dev_configure/info -------- */
static int macb_dev_configure(struct rte_eth_dev *dev)
{
    struct rte_eth_conf *c = &dev->data->dev_conf;
    const uint64_t rx_ok = 0;
    const uint64_t tx_ok = RTE_ETH_TX_OFFLOAD_MBUF_FAST_FREE;
    if (c->rxmode.offloads & ~rx_ok) c->rxmode.offloads &= rx_ok;
    if (c->txmode.offloads & ~tx_ok) c->txmode.offloads &= tx_ok;
    return 0;
}
static int macb_info_get(struct rte_eth_dev *dev, struct rte_eth_dev_info *di)
{
    RTE_SET_USED(dev);
    di->max_rx_queues  = 1; di->max_tx_queues = 1;
    di->max_rx_pktlen  = RTE_ETHER_MAX_LEN; di->min_mtu = RTE_ETHER_MIN_MTU; di->max_mtu = RTE_ETHER_MTU;
    di->min_rx_bufsize = 64;
    di->rx_desc_lim = (struct rte_eth_desc_lim){ .nb_min = 64, .nb_max = MACB_NRXD, .nb_align = 1 };
    di->tx_desc_lim = (struct rte_eth_desc_lim){ .nb_min = 64, .nb_max = MACB_NTXD, .nb_align = 1 };
    di->rx_offload_capa = 0;
    di->tx_offload_capa = RTE_ETH_TX_OFFLOAD_MBUF_FAST_FREE;
    di->speed_capa = RTE_ETH_LINK_SPEED_10M_HD  | RTE_ETH_LINK_SPEED_10M |
                     RTE_ETH_LINK_SPEED_100M_HD | RTE_ETH_LINK_SPEED_100M |
                     RTE_ETH_LINK_SPEED_1G;
#if defined(RTE_VERSION_MAJOR) && (RTE_VERSION_MAJOR >= 21)
    di->rss_algo_capa = 0;
#endif
    return 0;
}

/* -------- queue setup -------- */
static inline int dma_map_region_relaxed(struct rte_device *dev,
                                         const void *addr, rte_iova_t iova, size_t len)
{
    int rc = rte_dev_dma_map(dev, (void *)(uintptr_t)addr, iova, len);
    if (rc == -ENOTSUP || rc == -EINVAL) return 0;
    return rc;
}

static int macb_rx_queue_setup(struct rte_eth_dev *dev, uint16_t qid,
                               uint16_t nb_desc, unsigned int so,
                               const struct rte_eth_rxconf *rx_conf,
                               struct rte_mempool *mp)
{
    RTE_SET_USED(so); RTE_SET_USED(rx_conf);
    struct macb_adapter *ad = dev->data->dev_private;
    struct macb_rxq *rxq    = &ad->rxq[qid];

    const struct rte_memzone *mz = rte_eth_dma_zone_reserve(dev, "rx_ring", qid,
                                 nb_desc * sizeof(struct macb_desc),
                                 RTE_CACHE_LINE_SIZE, rte_socket_id());
    if (!mz) return -ENOMEM;

    // Print the RX ring memory addresses (virtual and IOVA)
    RTE_LOG(INFO, PMD, "RX ring reserved at virtual address: %p, IOVA: 0x%" PRIx64 "\n", mz->addr, mz->iova);

    int rc = dma_map_region_relaxed(dev->device, mz->addr, BUS_IOVA(mz->iova), mz->len);
    if (rc) { MACB_ERR("dma_map RX ring failed: %d\n", rc); return rc; }

    rxq->ring      = (struct macb_desc *)mz->addr;
    rxq->ring_iova = mz->iova;
    rxq->nb_desc   = nb_desc;
    rxq->mp        = mp;
    rxq->sync_fd   = -1;
    rxq->ad        = ad;
    rxq->port_id   = dev->data->port_id;
    rxq->sw_ring   = rte_zmalloc_socket("macb_rx_sw", nb_desc * sizeof(struct rte_mbuf *),
                                        RTE_CACHE_LINE_SIZE, rte_socket_id());
    if (!rxq->sw_ring) return -ENOMEM;
    memset(rxq->ring, 0, nb_desc * sizeof(struct macb_desc));
    macb_rx_init(rxq);
    dev->data->rx_queues[qid] = rxq;
    return 0;
}

static int macb_tx_queue_setup(struct rte_eth_dev *dev, uint16_t qid,
                               uint16_t nb_desc, unsigned int so,
                               const struct rte_eth_txconf *tx_conf)
{
    RTE_SET_USED(so);
    RTE_SET_USED(tx_conf);

    struct macb_adapter *ad = dev->data->dev_private;
    struct macb_txq *txq    = &ad->txq[qid];

    const struct rte_memzone *mz =
        rte_eth_dma_zone_reserve(dev, "tx_ring", qid,
                                 nb_desc * sizeof(struct macb_desc),
                                 RTE_CACHE_LINE_SIZE, rte_socket_id());
    if (!mz) return -ENOMEM;

    // Print the TX ring memory addresses (virtual and IOVA)
    RTE_LOG(INFO, PMD, "TX ring reserved at virtual address: %p, IOVA: 0x%" PRIx64 "\n", mz->addr, mz->iova);

    int rc = rte_dev_dma_map(dev->device, (void *)(uintptr_t)mz->addr, BUS_IOVA(mz->iova), mz->len);
    if (rc == -ENOTSUP || rc == -EINVAL) rc = 0;
    if (rc) { MACB_ERR("dma_map TX ring failed: %d\n", rc); return rc; }

    txq->ring      = (struct macb_desc *)mz->addr;
    txq->ring_iova = mz->iova;
    txq->nb_desc   = nb_desc;
    txq->sync_fd   = -1;
    txq->ad        = ad;

    txq->sw_ring = rte_zmalloc_socket("macb_tx_sw",
                                      nb_desc * sizeof(struct rte_mbuf *),
                                      RTE_CACHE_LINE_SIZE, rte_socket_id());
    if (!txq->sw_ring) return -ENOMEM;
    for (uint16_t i = 0; i < nb_desc; i++) {
        const uint32_t wrap = (i == nb_desc - 1) ? TX_WRAP : 0;
        txq->ring[i].addr = 0;
        txq->ring[i].ctrl = (TX_USED | wrap);
        txq->sw_ring[i]   = NULL;
    }
    rte_io_wmb();

    txq->prod = 0;
    txq->cons = 0;
    dev->data->tx_queues[qid] = txq;
    return 0;
}

/* -------- Queue state -------- */
static int macb_rx_queue_start(struct rte_eth_dev *dev, uint16_t qid){
    if (qid < RTE_ETHDEV_QUEUE_STAT_CNTRS)
        dev->data->rx_queue_state[qid] = RTE_ETH_QUEUE_STATE_STARTED;
    return 0;
}
static int macb_rx_queue_stop(struct rte_eth_dev *dev, uint16_t qid){
    if (qid < RTE_ETHDEV_QUEUE_STAT_CNTRS)
        dev->data->rx_queue_state[qid] = RTE_ETH_QUEUE_STATE_STOPPED;
    return 0;
}
static int macb_tx_queue_start(struct rte_eth_dev *dev, uint16_t qid){
    if (qid < RTE_ETHDEV_QUEUE_STAT_CNTRS)
        dev->data->tx_queue_state[qid] = RTE_ETH_QUEUE_STATE_STARTED;
    struct macb_adapter *ad = dev->data->dev_private;
    macb_tx_force_free(&ad->txq[qid]);
    macb_tx_dump_head(&ad->txq[qid], "txq_start", 4);
    return 0;
}
static int macb_tx_queue_stop(struct rte_eth_dev *dev, uint16_t qid){
    if (qid < RTE_ETHDEV_QUEUE_STAT_CNTRS)
        dev->data->tx_queue_state[qid] = RTE_ETH_QUEUE_STATE_STOPPED;
    return 0;
}

/* -------- Promisc / link -------- */
int macb_promiscuous_enable(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    uint32_t n = macb_readl(&ad->hw, MACB_NCFGR);
    n |= NCFGR_CAF;
#ifdef NCFGR_NBC
    n &= ~NCFGR_NBC;
#endif
    macb_writel(&ad->hw, MACB_NCFGR, n);
    (void)macb_readl(&ad->hw, MACB_NCFGR);
    return 0;
}

int macb_promiscuous_disable(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    uint32_t n = macb_readl(&ad->hw, MACB_NCFGR);
    n &= ~NCFGR_CAF;
#ifdef NCFGR_NBC
    n &= ~NCFGR_NBC;
#endif
    macb_writel(&ad->hw, MACB_NCFGR, n);
    (void)macb_readl(&ad->hw, MACB_NCFGR);
    return 0;
}

/* --- Fallbacks if some SoC headers don't define these --- */
#ifndef NCFGR_SPD      /* 100 Mbps select */
# define NCFGR_SPD    (1u << 0)
#endif
#ifndef NCFGR_FD       /* Full duplex */
# define NCFGR_FD     (1u << 1)
#endif
#ifndef GEM_NCFGR_GBE  /* 1G enable on GEM variants */
# define GEM_NCFGR_GBE (1u << 10)
#endif
#ifndef MACB_BIT
# define MACB_BIT(x)   (1u << (x))
#endif
/* CAF/NBC bit positions should come from your MACB headers.
 * If not available, define them to the proper positions for your IP.
#ifndef CAF
# define CAF  4   // example only: Copy All Frames
#endif
#ifndef NBC
# define NBC  5   // example only: No BroadCast
#endif
*/

/* Only the speed/duplex bits are managed here */
#define NCFGR_SPEED_MASK   (NCFGR_SPD | NCFGR_FD | GEM_NCFGR_GBE)

static int macb_link_update(struct rte_eth_dev *dev, int wait)
{
    RTE_SET_USED(wait);

    struct macb_adapter *ad = dev->data->dev_private;
    uint8_t  phy = 0;
    uint16_t id1 = 0, id2 = 0;

    struct rte_eth_link l;
    memset(&l, 0, sizeof(l));
    l.link_autoneg = RTE_ETH_LINK_AUTONEG;
    l.link_status  = RTE_ETH_LINK_DOWN;
    l.link_speed   = RTE_ETH_SPEED_NUM_NONE;
    l.link_duplex  = RTE_ETH_LINK_HALF_DUPLEX;

    if (macb_find_phy_addr(ad, &phy, &id1, &id2) != 0) {
        rte_eth_linkstatus_set(dev, &l);
        return 0;
    }

    /* Read BMSR twice to clear latched bits on some PHYs */
    (void)macb_mdio_read_c22(ad, phy, MII_BMSR);
    uint16_t bmsr = macb_mdio_read_c22(ad, phy, MII_BMSR);

    if (bmsr & BMSR_LSTATUS) {
        l.link_status = RTE_ETH_LINK_UP;

        if (bmsr & BMSR_ANEGCOMPLETE) {
            uint16_t lpa      = macb_mdio_read_c22(ad, phy, MII_LPA);
            uint16_t stat1000 = macb_mdio_read_c22(ad, phy, MII_STAT1000);
            if (stat1000 & LPA_1000FULL)      { l.link_speed = RTE_ETH_SPEED_NUM_1G;   l.link_duplex = RTE_ETH_LINK_FULL_DUPLEX; }
            else if (stat1000 & LPA_1000HALF) { l.link_speed = RTE_ETH_SPEED_NUM_1G;   l.link_duplex = RTE_ETH_LINK_HALF_DUPLEX; }
            else if (lpa & ADVERTISE_100FULL) { l.link_speed = RTE_ETH_SPEED_NUM_100M; l.link_duplex = RTE_ETH_LINK_FULL_DUPLEX; }
            else if (lpa & ADVERTISE_100HALF) { l.link_speed = RTE_ETH_SPEED_NUM_100M; l.link_duplex = RTE_ETH_LINK_HALF_DUPLEX; }
            else if (lpa & ADVERTISE_10FULL)  { l.link_speed = RTE_ETH_SPEED_NUM_10M;  l.link_duplex = RTE_ETH_LINK_FULL_DUPLEX; }
            else if (lpa & ADVERTISE_10HALF)  { l.link_speed = RTE_ETH_SPEED_NUM_10M;  l.link_duplex = RTE_ETH_LINK_HALF_DUPLEX; }
            else                               { l.link_speed = RTE_ETH_SPEED_NUM_100M; l.link_duplex = RTE_ETH_LINK_FULL_DUPLEX; }
        } else {
            /* Forced mode (AN disabled): infer from BMCR */
            uint16_t bmcr = macb_mdio_read_c22(ad, phy, MII_BMCR);
            l.link_speed  = (bmcr & BMCR_SPEED100) ? RTE_ETH_SPEED_NUM_100M : RTE_ETH_SPEED_NUM_10M;
            l.link_duplex = (bmcr & BMCR_FULLDPLX) ? RTE_ETH_LINK_FULL_DUPLEX : RTE_ETH_LINK_HALF_DUPLEX;
        }
    }

    /* Publish link to DPDK */
    rte_eth_linkstatus_set(dev, &l);

    /* === Program MAC speed/duplex; preserve RX filter and other bits === */
    uint32_t ncfgr_old = macb_readl(&ad->hw, MACB_NCFGR);
    uint32_t ncfgr_new = ncfgr_old & ~NCFGR_SPEED_MASK;

    if (l.link_status == RTE_ETH_LINK_UP) {
        if (l.link_duplex == RTE_ETH_LINK_FULL_DUPLEX)
            ncfgr_new |= NCFGR_FD;

        if (l.link_speed == RTE_ETH_SPEED_NUM_1G) {
#ifdef GEM_NCFGR_GBE
            ncfgr_new |= GEM_NCFGR_GBE;
#endif
        } else if (l.link_speed == RTE_ETH_SPEED_NUM_100M) {
            ncfgr_new |= NCFGR_SPD; /* 100M */
        }
    }

    /* Enforce receive filter policy:
     * - Always accept broadcast (clear NBC).
     * - If promiscuous, set CAF too.
     * - Leave other filters (e.g., all-multicast) untouched.
     */
    ncfgr_new &= ~MACB_NCFGR_NBC;
    if (dev->data->promiscuous)
        ncfgr_new |= MACB_NCFGR_CAF;

    if (ncfgr_new != ncfgr_old) {
        macb_writel(&ad->hw, MACB_NCFGR, ncfgr_new);
        (void)macb_readl(&ad->hw, MACB_NCFGR); /* post-write fence */
    }

    MACB_DBG("NCFGR=0x%08x after link update (FD=%d SPD100=%d GBE=%d CAF=%d NBC=%d)\n",
             ncfgr_new,
             !!(ncfgr_new & NCFGR_FD),
             !!(ncfgr_new & NCFGR_SPD),
             !!(ncfgr_new & GEM_NCFGR_GBE),
             !!(ncfgr_new & MACB_NCFGR_CAF),
             !!(ncfgr_new & MACB_NCFGR_NBC));

    MACB_DBG("Link: %s, speed=%u, duplex=%s (phy=%u id=%04x:%04x)\n",
             l.link_status ? "UP" : "DOWN",
             l.link_speed,
             (l.link_duplex == RTE_ETH_LINK_FULL_DUPLEX) ? "FD" : "HD",
             (unsigned)phy, id1, id2);

    return 0;
}

/* -------- RX ring ownership -------- */
#ifndef MACB_RXD_ADDR_OWN
#define MACB_RXD_ADDR_OWN  0x1u
#endif
#ifndef MACB_RXD_ADDR_WRAP
#define MACB_RXD_ADDR_WRAP 0x2u
#endif

static inline void macb_rx_fill_desc(struct macb_rxq *rxq, uint16_t i, rte_iova_t data_iova)
{
    volatile struct macb_desc *d = &rxq->ring[i];

    const uint32_t wrap = (i == (rxq->nb_desc - 1)) ? MACB_RXD_ADDR_WRAP : 0;

    /* Hand ownership to HW: set OWN (bit0) + optional WRAP (bit1) */
    d->addr = (uint32_t)(data_iova) | wrap;

    // Print the IOVA of the descriptor
    RTE_LOG(INFO, PMD, "RX[%u] descriptor[%u]: IOVA = 0x%" PRIx64 "\n", rxq->port_id, i, data_iova);

    /* Status/length word is cleared on arm (WRAP is in addr on this IP) */
    d->ctrl = 0;

    rte_wmb();
    sync_desc_to_dev(rxq->sync_fd, d, sizeof(*d));
    rte_io_wmb();
}

/* When (re)arming the ring, set OWN on every desc */
static void macb_rx_force_hw_own(struct macb_rxq *rxq)
{
    for (uint16_t i = 0; i < rxq->nb_desc; i++) {
        struct rte_mbuf *mb = rxq->sw_ring[i];
        macb_rx_fill_desc(rxq, i, macb_mbuf_data_iova(mb));
    }
}

/* -------- debug helpers -------- */
static void macb_dump_first_rx_descs(struct macb_rxq *rxq, const char *tag)
{
    for (int i = 0; i < 2 && i < rxq->nb_desc; ++i) {
        volatile struct macb_desc *dv = &rxq->ring[i];

        /* Fresh device->CPU view */
        sync_desc_from_dev(rxq->sync_fd, (const void *)dv, sizeof(*dv));
        rte_io_mb();

        uint32_t w0 = ((volatile uint32_t *)dv)[0];
        uint32_t w1 = ((volatile uint32_t *)dv)[1];

        int layout = 0; /* 0: w0=addr, w1=status (typical MACB) */
        uint32_t addr_raw = (layout == 1) ? w1 : w0;
        uint32_t stat     = (layout == 1) ? w0 : w1;

        uint32_t addr_iova = addr_raw & ~0x3u;
        int used  = !!(addr_raw & RX_USED);  /* 1 = SW owns */
        int wrap  = !!(addr_raw & RX_WRAP);
        int hw_own = !used;                  /* 1 = HW owns (armed) */

        MACB_DBG(
            "RX[%s] d%03d: w0=%08x w1=%08x | addr=%08x USED=%d OWN=%s WRAP=%d len=%u sof=%d eof=%d\n",
            tag, i, w0, w1, addr_iova, used, hw_own ? "HW" : "SW", wrap,
            (stat & RX_LEN_MASK), !!(stat & RX_SOF), !!(stat & RX_EOF));
    }
}

static void macb_dump_first_tx_descs(struct macb_txq *txq, const char *tag)
{
    for (int i = 0; i < 4 && i < txq->nb_desc; ++i++) {
        struct macb_desc *d = &txq->ring[i];
        uint32_t a = d->addr, c = d->ctrl;
        RTE_LOG(INFO, PMD,
                "macb: TX[%s] d%d: addr=0x%08x ctrl=0x%08x (USED=%d, WRAP=%d)\n",
                tag, i, a, c, !!(c & 0x80000000u), !!(c & 0x40000000u));
    }
}

/* After RBQP/TBQP are written, before enabling RX/TX */
static inline uint16_t macb_rxbuf_bytes(struct macb_adapter *ad)
{
    /* Use the actual data room, not full mbuf size, and clamp to hw granularity */
    const uint32_t room = rte_pktmbuf_data_room_size(ad->mb_pool) - RTE_PKTMBUF_HEADROOM;
    /* GEM expects (RBSZ = (bytes / 64) - 1). Round down to 64B, minimum 64B. */
    uint32_t sz = RTE_ALIGN_FLOOR(room, 64);
    if (sz < 64) sz = 64;
    return (uint16_t)sz;
}

static void macb_program_rxbufsz(struct macb_adapter *ad)
{
#ifdef GEM_DMACFG
    /* Only present on Cadence GEM variants */
    uint32_t dmacfg = macb_readl(&ad->hw, GEM_DMACFG);

# ifdef GEM_DMACFG_RXBUF_Msk
    const uint32_t rx_bytes = macb_rxbuf_bytes(ad);
    const uint32_t rbsz = (rx_bytes / 64) - 1;      /* HW encoding */
    dmacfg &= ~GEM_DMACFG_RXBUF_Msk;
#  ifdef GEM_DMACFG_RXBUF
    dmacfg |= GEM_DMACFG_RXBUF(rbsz);
#  else
    dmacfg |= (rbsz << GEM_DMACFG_RXBUF_Pos);
#  endif
    macb_writel(&ad->hw, GEM_DMACFG, dmacfg);
    MACB_DBG("DMACFG=0x%08x (RXBUF=%u bytes)\n", dmacfg, rx_bytes);
# else
    const uint32_t rx_bytes = macb_rxbuf_bytes(ad);
    const uint32_t rbsz = (rx_bytes / 64) - 1;
    dmacfg &= ~(0x7Fu << 16);              /* TODO: replace with correct mask */
    dmacfg |=  (rbsz & 0x7Fu) << 16;       /* TODO: replace with correct shift */
    macb_writel(&ad->hw, GEM_DMACFG, dmacfg);
    MACB_DBG("DMACFG(guess)=0x%08x (RXBUF=%u bytes)\n", dmacfg, rx_bytes);
# endif /* GEM_DMACFG_RXBUF_Msk */
#else
    /* Plain MACB has no RX buffer size field — nothing to do. */
    RTE_SET_USED(ad);
#endif
}

/* -------- dev_start -------- */
static int macb_dev_start(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    struct macb_rxq *rxq = &ad->rxq[0];
    struct macb_txq *txq = &ad->txq[0];
    uint32_t ncr;

    MACB_DBG("starting dev...\n");

    macb_dma_sync_open_once(ad);
    rxq->sync_fd = ad->sync_fd;
    txq->sync_fd = ad->sync_fd;

    macb_rx_force_hw_own(rxq);
    macb_tx_force_free(txq);
#ifdef MACB_RSR
    macb_writel(&ad->hw, MACB_RSR, 0xffffffffu);
#endif
#ifdef MACB_TSR
    macb_writel(&ad->hw, MACB_TSR, 0xffffffffu);
#endif

    //uint64_t probed = macb_calc_bus_ofs_from_tbqp(ad, txq);
    //if ((int64_t)probed != (int64_t)g_bus_ofs) {
    //    RTE_LOG(INFO, PMD, "macb: bus_ofs corrected: 0x%llx -> 0x%llx\n",
    //            (unsigned long long)g_bus_ofs, (unsigned long long)probed);
    //    g_bus_ofs = probed;
    //}
    /* Recompute ring base IOVAs *after* any correction */
    uint64_t rb = BUS_IOVA(rxq->ring_iova);
    uint64_t tb = BUS_IOVA(txq->ring_iova);

    /* Program ring bases, then fence */
    macb_writel(&ad->hw, MACB_RBQPH, (uint32_t)(rb >> 32));
    macb_writel(&ad->hw, MACB_RBQP,  (uint32_t)(rb & 0xffffffffu));
    macb_writel(&ad->hw, MACB_TBQPH, (uint32_t)(tb >> 32));
    macb_writel(&ad->hw, MACB_TBQP,  (uint32_t)(tb & 0xffffffffu));
#ifdef GEM_TBQB_Q0
    macb_writel(&ad->hw, GEM_TBQB_Q0, (uint32_t)(tb & 0xffffffffu));
#endif
#ifdef GEM_RBQB_Q0
    macb_writel(&ad->hw, GEM_RBQB_Q0, (uint32_t)(rb & 0xffffffffu));
#endif
    rte_io_wmb();

    /* Push all RX/TX descriptors to NIC */
    for (uint16_t i = 0; i < rxq->nb_desc; i++)
        sync_desc_to_dev(rxq->sync_fd, &rxq->ring[i], sizeof(rxq->ring[i]));
    for (uint16_t i = 0; i < txq->nb_desc; i++)
        sync_desc_to_dev(txq->sync_fd, &txq->ring[i], sizeof(txq->ring[i]));

    for (uint16_t i = 0; i < rxq->nb_desc; i++) {
        volatile struct macb_desc *d = &rxq->ring[i];
        sync_desc_from_dev(rxq->sync_fd, (const void *)d, sizeof(*d));
        uint32_t a = d->addr;
        if (a & RX_USED) {                  /* 1 = SW-owned; clear it before RXEN */
            d->addr = a & ~RX_USED;         /* make HW-owned/armed */
            d->ctrl = 0;
            rte_wmb();
            sync_desc_to_dev(rxq->sync_fd, (const void *)d, sizeof(*d));
            RTE_LOG(WARNING, PMD, "macb: RX[pre-enable-fix] d%03u USED was 1, forced to 0\n", i);
        }
    }

    if (ad->rxq[0].mp)
        macb_map_mempool(dev->device, ad->rxq[0].mp);

    macb_dump_first_rx_descs(rxq, "pre-enable");
    macb_tx_dump_head(txq, "pre-enable", 4);

    /* PHY config */
    if (g_phy_mode == PHY_MODE_AUTO)
        macb_phy_normal_up(ad);
    else
        macb_phy_force_10_100(ad, g_phy_mode);

    /* Promisc/CAF config */
    uint32_t n = macb_readl(&ad->hw, MACB_NCFGR);
#ifdef NCFGR_CAF
    n |= NCFGR_CAF;
#endif
#ifdef NCFGR_NBC
    n &= ~NCFGR_NBC;
#endif
    macb_writel(&ad->hw, MACB_NCFGR, n);

    macb_force_promisc_allow_bcast(ad);
    /* Program Tx ring base again before enable */
    ncr = macb_readl(&ad->hw, MACB_NCR);
#ifdef MACB_TBQPH
    macb_writel(&ad->hw, MACB_TBQPH, (uint32_t)(tb >> 32));
#endif
    macb_writel(&ad->hw, MACB_TBQP,  (uint32_t)tb);
    rte_io_wmb();

    macb_program_rxbufsz(ad);

    /* ******** FINAL FENCE BEFORE RXEN ******** */
    rte_io_wmb();

    ncr |= NCR_RXEN | NCR_TXEN | NCR_MPE;
    macb_writel(&ad->hw, MACB_NCR, ncr);

    macb_apply_loopback(ad);
    {
        uint32_t ncf = macb_readl(&ad->hw, MACB_NCFGR);
        uint32_t ncrv = macb_readl(&ad->hw, MACB_NCR);
        MACB_DBG("MAC LB check: NCR=0x%08x (LB=%d RXEN=%d TXEN=%d) NCFGR=0x%08x\n",
            ncrv, !!(ncrv & NCR_LB), !!(ncrv & NCR_RXEN), !!(ncrv & NCR_TXEN), ncf);
    }
    rte_io_wmb();

    /* Lightweight RX probe after enable */
    volatile struct macb_desc *d0 = &rxq->ring[0];
    volatile struct macb_desc *d1 = &rxq->ring[1];
    uint32_t rph = macb_readl(&ad->hw, MACB_RBQPH);
    uint32_t rpl = macb_readl(&ad->hw, MACB_RBQP);
    MACB_DBG("RBQP read=0x%08x, expected ring_iova=0x%08x\n",
         rpl, (uint32_t)rxq->ring_iova);
    uint32_t rsr = 0;
#ifdef MACB_RSR
    rsr = macb_readl(&ad->hw, MACB_RSR);
#endif

    MACB_DBG("RX[probe0] RBQP=%08x:%08x d0:%08x/%08x d1:%08x/%08x RSR=%08x\n",
             rph, rpl, d0->addr, d0->ctrl, d1->addr, d1->ctrl, rsr);

    rte_delay_us_block(5000);

#ifdef MACB_RSR
    rsr = macb_readl(&ad->hw, MACB_RSR);
#endif
    MACB_DBG("RX[probe1] d0:%08x/%08x d1:%08x/%08x RSR=%08x\n",
             d0->addr, d0->ctrl, d1->addr, d1->ctrl, rsr);

    macb_dump_first_tx_descs(txq, "post-kick");
    macb_dump_first_rx_descs(rxq, "post-kick");
    macb_rx_probe_once(rxq);
    rte_delay_us_block(5000);
    macb_rx_probe_once(rxq);

    return 0;
}

static int macb_dev_stop(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
    ncr &= ~(NCR_RXEN | NCR_TXEN);
    macb_writel(&ad->hw, MACB_NCR, ncr);
    dev->data->dev_link.link_status = RTE_ETH_LINK_DOWN;

    macb_dma_sync_close(ad);
    ad->rxq[0].sync_fd = -1;
    ad->txq[0].sync_fd = -1;
    return 0;
}
static int macb_dev_close(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    macb_dev_stop(dev);
    if (ad->rxq[0].sw_ring) { rte_free(ad->rxq[0].sw_ring); ad->rxq[0].sw_ring = NULL; }
    if (ad->txq[0].sw_ring) { rte_free(ad->txq[0].sw_ring); ad->txq[0].sw_ring = NULL; }
    macb_uio_unmap(&ad->hw);
    return 0;
}

/* -------- Program MAC address -------- */
#if defined(MACB_SA1B) && defined(MACB_SA1T)
static void macb_hw_write_mac(struct macb_adapter *ad, const struct rte_ether_addr *ea)
{
    uint32_t sa1b = ea->addr_bytes[3] << 24 | ea->addr_bytes[2] << 16 |
                    ea->addr_bytes[1] << 8  | ea->addr_bytes[0];
    uint32_t sa1t = ea->addr_bytes[5] << 8  | ea->addr_bytes[4];
    macb_writel(&ad->hw, MACB_SA1B, sa1b);
    macb_writel(&ad->hw, MACB_SA1T, sa1t);
}
#else
static void macb_hw_write_mac(struct macb_adapter *ad, const struct rte_ether_addr *ea)
{ RTE_SET_USED(ad); RTE_SET_USED(ea); }
#endif

static int macb_mac_addr_set(struct rte_eth_dev *dev, struct rte_ether_addr *ea)
{
    struct macb_adapter *ad = dev->data->dev_private;
    rte_ether_addr_copy(ea, &dev->data->mac_addrs[0]);
    macb_hw_write_mac(ad, ea);
    return 0;
}

/* -------- stats -------- */
static int macb_stats_get(struct rte_eth_dev *dev, struct rte_eth_stats *stats)
{
    if (!stats) return -EINVAL;
    struct macb_adapter *ad = dev->data->dev_private;
    memset(stats, 0, sizeof(*stats));
    stats->ipackets = ad->sw.rx_pkts;
    stats->ibytes   = ad->sw.rx_bytes;
    stats->ierrors  = ad->sw.rx_errs;
    stats->opackets = ad->sw.tx_pkts;
    stats->obytes   = ad->sw.tx_bytes;
    stats->oerrors  = ad->sw.tx_errs;
    return 0;
}
static int macb_stats_reset(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    memset(&ad->sw, 0, sizeof(ad->sw));
    return 0;
}

/* -------- vdev probe/remove -------- */
static int parse_dev_arg_cb(const char *key, const char *value, void *opaque)
{
    if (strcmp(key, "dev") != 0 || value == NULL || *value == '\0') return -EINVAL;
    char **out = (char **)opaque; if (*out) rte_free(*out);
    *out = (char *)rte_malloc("macb", strlen(value) + 1, 0);
    if (!*out) return -ENOMEM;
    strcpy(*out, value);
    return 0;
}
static int parse_bool_arg_cb(const char *key, const char *val, void *extra)
{
    RTE_SET_USED(key); int *out = (int *)extra; if (!val) return -EINVAL;
    if (!strcmp(val, "1") || !strcasecmp(val, "true") || !strcasecmp(val, "on")) *out = 1;
    else if (!strcmp(val, "0") || !strcasecmp(val, "false") || !strcasecmp(val, "off")) *out = 0;
    else return -EINVAL;
    return 0;
}
static int parse_int_arg_cb(const char *key, const char *val, void *extra)
{
    RTE_SET_USED(key); if (!val) return -EINVAL;
    char *end = NULL; long v = strtol(val, &end, 0);
    if (end == val || *end || v < -1 || v > 31) return -EINVAL;
    *(int *)extra = (int)v; return 0;
}
static int parse_u64_arg_cb(const char *key, const char *val, void *extra)
{
    RTE_SET_USED(key); if (!val) return -EINVAL; char *end = NULL;
    uint64_t v = strtoull(val, &end, 0); if (end == val || *end) return -EINVAL; *(uint64_t *)extra = v; return 0;
}

/* NEW: parse phy_mode value */
static int parse_phy_mode_cb(const char *key, const char *val, void *extra)
{
    RTE_SET_USED(key); RTE_SET_USED(extra);
    if (!val) return -EINVAL;
    if (!strcasecmp(val, "auto"))   { g_phy_mode = PHY_MODE_AUTO; }
    else if (!strcasecmp(val, "10hd"))  { g_phy_mode = PHY_MODE_10HD; }
    else if (!strcasecmp(val, "10fd"))  { g_phy_mode = PHY_MODE_10FD; }
    else if (!strcasecmp(val, "100hd")) { g_phy_mode = PHY_MODE_100HD; }
    else if (!strcasecmp(val, "100fd")) { g_phy_mode = PHY_MODE_100FD; }
    else return -EINVAL;
    MACB_DBG("phy_mode set to %s\n", val);
    return 0;
}

static int macb_probe(struct rte_vdev_device *vdev)
{
    const char *args = rte_vdev_device_args(vdev);
    struct rte_kvargs *kv = (args && *args) ? rte_kvargs_parse(args, NULL) : NULL;
    char *uio = NULL;
    if (kv) {
        (void)rte_kvargs_process(kv, "dev",      parse_dev_arg_cb,   &uio);
        (void)rte_kvargs_process(kv, "phy_lb",   parse_bool_arg_cb,  &g_force_phy_lb);
        (void)rte_kvargs_process(kv, "mac_lb",   parse_bool_arg_cb,  &g_mac_lb);
        (void)rte_kvargs_process(kv, "phy_addr", parse_int_arg_cb,   &g_forced_phy_addr);
        (void)rte_kvargs_process(kv, "bus_ofs",  parse_u64_arg_cb,   &g_bus_ofs);
        /* NEW: */
        (void)rte_kvargs_process(kv, "phy_mode", parse_phy_mode_cb,  NULL);
        rte_kvargs_free(kv);
    }
    if (!uio) { uio = (char *)rte_malloc("macb", strlen("/dev/uio0")+1, 0); if (!uio) return -ENOMEM; strcpy(uio, "/dev/uio0"); }

    struct rte_eth_dev *eth_dev = rte_eth_vdev_allocate(vdev, sizeof(struct macb_adapter));
    if (!eth_dev) { rte_free(uio); return -ENOMEM; }

    struct macb_adapter *ad = eth_dev->data->dev_private;
    memset(ad, 0, sizeof(*ad));
    ad->hw.uio_fd = -1;
    ad->sync_fd   = -1;
    ad->port_id   = eth_dev->data->port_id;
    MACB_DBG("probe args='%s'\n", args ? args : "(none)");

    int rc = macb_uio_map(&ad->hw, uio);
    rte_free(uio);
    if (rc) { rte_eth_dev_release_port(eth_dev); return rc; }

    eth_dev->dev_ops      = &macb_ops;
    eth_dev->rx_pkt_burst = macb_rx_burst;
    eth_dev->tx_pkt_burst = macb_tx_burst;
#ifdef RTE_ETH_DEV_CLOSE_REMOVE
    eth_dev->data->dev_flags |= RTE_ETH_DEV_CLOSE_REMOVE;
#endif
    eth_dev->data->mac_addrs = rte_zmalloc("macb_mac", sizeof(struct rte_ether_addr), 0);
    if (!eth_dev->data->mac_addrs) { macb_uio_unmap(&ad->hw); rte_eth_dev_release_port(eth_dev); return -ENOMEM; }
    struct rte_ether_addr mac = { .addr_bytes = {0x02,0,0,0,0,1} };
    rte_ether_addr_copy(&mac, &eth_dev->data->mac_addrs[0]);
    macb_hw_write_mac(ad, &mac);

    eth_dev->data->nb_rx_queues = 1; eth_dev->data->nb_tx_queues = 1;
    rte_eth_dev_probing_finish(eth_dev);
    RTE_LOG(INFO, PMD, "macb: probe done; initial bus_ofs=0x%llx\n",
            (unsigned long long)g_bus_ofs);
    return 0;
}

static int macb_remove(struct rte_vdev_device *vdev)
{
    struct rte_eth_dev *eth_dev = rte_eth_dev_allocated(rte_vdev_device_name(vdev));
    if (!eth_dev) return 0;
    struct macb_adapter *ad = eth_dev->data->dev_private;
    if (ad) macb_uio_unmap(&ad->hw);
    rte_eth_dev_release_port(eth_dev);
    return 0;
}

static struct rte_vdev_driver macb_drv = { .probe = macb_probe, .remove = macb_remove };
RTE_PMD_REGISTER_VDEV(net_macb, macb_drv);
RTE_PMD_REGISTER_ALIAS(net_macb, net_macb0);
RTE_PMD_REGISTER_PARAM_STRING(net_macb, "dev=<path> phy_addr=<int> phy_lb=<0|1> mac_lb=<0|1> bus_ofs=<u64> phy_mode=<auto|10hd|10fd|100hd|100fd>");

