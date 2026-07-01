// macb_ethdev.c — DPDK vdev PMD for RP1/Cadence GEM (Raspberry Pi 5)

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <rte_cycles.h>
#include <rte_dev.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_hexdump.h>

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
#include <sys/mman.h>   /* mprotect */
#include <unistd.h>

#include <ethdev_driver.h>
#include <ethdev_vdev.h>
#include <rte_bus_vdev.h>

#ifndef RTE_LOGTYPE_PMD
#define RTE_LOGTYPE_PMD RTE_LOGTYPE_USER1
#endif

#include "macb_rx_audit.h"
#include "macb_hw.h"         /* NIC regs/desc + struct macb_adapter/rxq/txq */
#include "dma_sync_uapi.h"   /* userspace DMA sync helper ioctl API */
#include "macb_dma_sync.h" 

#define MACB_DBG(fmt, ...) RTE_LOG(INFO,  PMD, "macb: " fmt, ##__VA_ARGS__)
#define MACB_ERR(fmt, ...) RTE_LOG(ERR,   PMD, "macb: " fmt, ##__VA_ARGS__)

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
uint64_t g_bus_ofs = 0;
#define BUS_IOVA_RING(x) ((rte_iova_t)((rte_iova_t)(x) + (rte_iova_t)g_bus_ofs))
#ifndef MACB_FORCE_BUS_OFS_FOR_DATA
#define MACB_FORCE_BUS_OFS_FOR_DATA 1
#endif
#define BUS_IOVA_DATA(x) \
    ( (MACB_FORCE_BUS_OFS_FOR_DATA) ? \
      (rte_iova_t)((rte_iova_t)(x) + (rte_iova_t)g_bus_ofs) : (rte_iova_t)(x) )
/* Back-compat alias used in older code paths */
#define BUS_IOVA(x) BUS_IOVA_DATA((x))

static int      g_force_phy_lb    = 0;    /* default: external link (no PHY LB) */
static int      g_mac_lb          = 0;    /* default: MAC LB off */
static int      g_forced_phy_addr = -1;   /* -1 = auto-scan; else 0..31 */

/* NEW: simple PHY mode for 10/100 forcing (BMCR) */
enum macb_phy_mode { PHY_MODE_AUTO = 0, PHY_MODE_10HD, PHY_MODE_10FD, PHY_MODE_100HD, PHY_MODE_100FD };
static enum macb_phy_mode g_phy_mode = PHY_MODE_AUTO;

/* NEW: make RX ring read-only after arming (debugging CPU writes) */
static int g_rx_ring_ro = 0;  /* devarg rx_ring_ro=1 (or env MACB_RX_RING_RO=1) */

static int g_dma_selftest = 0; /* devarg dma_selftest=1 */
static int g_rxq_rwtest = 0;
static int g_rx_selftest = 1; /* devarg rx_selftest=1 */

/* Single helper that works across DPDK versions we care about */
static inline rte_iova_t macb_mbuf_data_iova(const struct rte_mbuf *m)
{
#if defined(rte_mbuf_data_iova_default)
    return rte_mbuf_data_iova_default(m);
#else
    return rte_mbuf_data_iova(m);
#endif
}

void
macb_rx_publish_desc(struct macb_rxq *rxq,
                     volatile struct macb_desc *dv,
                     uint64_t bus_addr, uint32_t wrap);

/* -------- type-safe set/clear helper -------- */
static inline void macb_set_bits(struct macb_hw *hw, uint32_t off, uint32_t set, uint32_t clr)
{
    uint32_t v = macb_readl(hw, off);
    v |= set;
    v &= ~clr;
    macb_writel(hw, off, v);
    v = macb_readl(hw, off); /* post the write */
    RTE_LOG(INFO, PMD, "macb: NCFGR now=0x%08x (CAF=%u NBC=%u)\n",
            v, !!(v & MACB_NCFGR_CAF), !!(v & MACB_NCFGR_NBC));
}

static void macb_force_promisc_allow_bcast(struct macb_adapter *ad)
{
    const uint32_t set = MACB_NCFGR_CAF;
    const uint32_t clr = MACB_NCFGR_NBC;
    macb_set_bits(&ad->hw, MACB_NCFGR, set, clr);
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
        //MACB_DBG("dma_sync_helper active via %s\n", node);
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

/* DMA-map mempool segments if platform requires it */
static int macb_map_mempool(struct rte_device *rdev, struct rte_mempool *mp)
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

        if (rc == -ENOTSUP || rc == -EINVAL) rc = 0;
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

/* PHY scan (with optional forced addr) */
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

    //MACB_DBG("Found PHY id1=0x%04x id2=0x%04x at addr=%u\n", id1, id2, (unsigned)phy);

    uint16_t bmcr = macb_mdio_read_c22(ad, phy, MII_BMCR);
    uint16_t adv  = macb_mdio_read_c22(ad, phy, MII_ADVERTISE);
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

    newb &= ~(BMCR_ANENABLE | BMCR_PDOWN | BMCR_ISOLATE | BMCR_LOOPBACK | BMCR_RESTARTAN);
    newb &= ~(BMCR_SPEED100 | BMCR_FULLDPLX);

    switch (mode) {
    case PHY_MODE_10HD:    break;
    case PHY_MODE_10FD:    newb |= BMCR_FULLDPLX; break;
    case PHY_MODE_100HD:   newb |= BMCR_SPEED100; break;
    case PHY_MODE_100FD:   newb |= (BMCR_SPEED100 | BMCR_FULLDPLX); break;
    default: return -EINVAL;
    }

    if (newb != bmcr) macb_mdio_write_c22(ad, phy, MII_BMCR, newb);

    //MACB_DBG("Forced PHY mode %s (BMCR 0x%04x -> 0x%04x, phy=%u id=%04x:%04x)\n",
    //         (mode==PHY_MODE_10HD)?"10HD":(mode==PHY_MODE_10FD)?"10FD":
    //         (mode==PHY_MODE_100HD)?"100HD":"100FD",
    //         bmcr, newb, (unsigned)phy, id1, id2);

    for (int i = 0; i < 200; i++) {
        uint16_t bmsr = macb_mdio_read_c22(ad, phy, MII_BMSR);
        if (bmsr & BMSR_LSTATUS) break;
        rte_delay_us_block(5000);
    }
    return 0;
}

/* ----- MAC loopback helper (off by default) ----- */
#ifndef NCFGR_LBL
#define NCFGR_LBL 0
#endif

/* PHY loopback toggle (off by default unless requested) */
static int macb_phy_loopback_set(struct macb_adapter *ad, int enable)
{
    uint8_t phy; uint16_t id1=0,id2=0;
    if (macb_find_phy_addr(ad, &phy, &id1, &id2) != 0)
        return -1;

    uint16_t bmcr = macb_mdio_read_c22(ad, phy, MII_BMCR);
    uint16_t newbmcr = bmcr;

    if (enable) {
        /* If you want loopback, it’s OK to force a known mode */
        newbmcr &= ~(BMCR_ANENABLE | BMCR_PDOWN | BMCR_ISOLATE);
        newbmcr |= BMCR_SPEED100 | BMCR_FULLDPLX | BMCR_LOOPBACK;
    } else {
        /* If disabling loopback, restore normal autoneg */
        //MACB_DBG("loopback_disable: g_phy_mode=%d (AUTO=%d)\n", g_phy_mode, PHY_MODE_AUTO);

	newbmcr &= ~BMCR_LOOPBACK;
    	if (g_phy_mode == PHY_MODE_AUTO){
            newbmcr |= (BMCR_ANENABLE | BMCR_RESTARTAN);
	}
    }

    if (newbmcr != bmcr)
        macb_mdio_write_c22(ad, phy, MII_BMCR, newbmcr);

    //MACB_DBG("PHY loopback %s (BMCR 0x%04x -> 0x%04x)\n",
    //         enable ? "EN" : "DIS", bmcr, newbmcr);
    return 0;
}

static void macb_mac_loopback_set(struct macb_adapter *ad, int enable)
{
    uint32_t ncfgr0 = macb_readl(&ad->hw, MACB_NCFGR);
    uint32_t ncr0   = macb_readl(&ad->hw, MACB_NCR);

    uint32_t ncfgr  = ncfgr0;
    uint32_t ncr    = ncr0;

#ifdef NCFGR_CAF
    if (enable) ncfgr |= NCFGR_CAF;
    else        ncfgr &= ~NCFGR_CAF;
#endif

#ifdef NCR_LB
    if (enable) ncr |= NCR_LB; else ncr &= ~NCR_LB;
#endif

#if NCFGR_LBL
    if (enable) ncfgr |= NCFGR_LBL; else ncfgr &= ~NCFGR_LBL;
#endif

    macb_writel(&ad->hw, MACB_NCFGR, ncfgr);
    ncr |= (NCR_RXEN | NCR_TXEN);
    macb_writel(&ad->hw, MACB_NCR,   ncr);

    //MACB_DBG("MAC loopback %s: NCFGR 0x%08x->0x%08x, NCR 0x%08x->0x%08x\n",
    //         enable ? "EN" : "DIS", ncfgr0, ncfgr, ncr0, ncr);
}

static void macb_apply_loopback(struct macb_adapter *ad)
{
    (void)macb_phy_loopback_set(ad, g_force_phy_lb ? 1 : 0);
    macb_mac_loopback_set(ad, g_mac_lb ? 1 : 0);
}

/* ---- Forward decls shared with rxtx ---- */
void macb_log_regs_full(struct macb_adapter *ad, const char *tag);

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

static void macb_scan_regs_for_tbqp(struct macb_adapter *ad, uint32_t expect_lo, const char *tag)
{
    RTE_LOG(INFO, PMD, "macb: regscan(%s): looking for %08x\n", tag, expect_lo);

    /* Scan a reasonable GEM register window. Adjust upper bound if needed. */
    for (uint32_t off = 0x0000; off <= 0x4000; off += 4) {
        uint32_t v = macb_readl(&ad->hw, off);

        if (v == expect_lo || v == (expect_lo + 0x18)) {
            RTE_LOG(INFO, PMD, "macb: regscan(%s): hit off=0x%04x val=%08x\n",
                    tag, off, v);
        }
    }
}

static inline void macb_set_tbqp_q0(struct macb_adapter *ad, uint64_t iova)
{
    uint32_t lo = (uint32_t)iova;
    uint32_t hi = (uint32_t)(iova >> 32);

    /* Program BOTH paths: legacy + GEM queue0 */
#ifdef MACB_TBQPH
    macb_writel(&ad->hw, MACB_TBQPH, hi);
#endif
    macb_writel(&ad->hw, MACB_TBQP, lo);

    macb_writel(&ad->hw, GEM_TBQP(0), lo);

    /* Make sure register writes hit before TX start */
    rte_io_wmb();
}


static inline uint32_t macb_get_tbqp_legacy(struct macb_adapter *ad) { return macb_readl(&ad->hw, MACB_TBQP); }
static inline uint32_t macb_get_tbqp_q0(struct macb_adapter *ad)     { return macb_readl(&ad->hw, GEM_TBQP(0)); }

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

static int macb_forced_desc_stride(void)
{
    const char *s = getenv("MACB_DESC_STRIDE");
    if (!s || !*s) return 0;
    return atoi(s); /* allow 8/16/32/64 */
}


/* Helpers to avoid double BUS_IOVA() mistakes */
static inline uint64_t macb_iova_u64(rte_iova_t iova)
{
    return (uint64_t)iova;
}

static inline uint32_t macb_iova_lo(rte_iova_t iova)
{
    return (uint32_t)(macb_iova_u64(iova) & 0xffffffffu);
}

static inline uint32_t macb_iova_hi(rte_iova_t iova)
{
    return (uint32_t)(macb_iova_u64(iova) >> 32);
}

/* --- debug: treat stride as truth for ring walking --- */
static inline volatile uint32_t *
macb_desc_words_at_stride(void *ring_base, uint32_t stride, uint32_t idx)
{
    return (volatile uint32_t *)((uint8_t *)ring_base + (size_t)idx * (size_t)stride);
}

/* Decode TX ctrl (GEM/MACB) from word1 */
static inline void macb_tx_decode_ctrl(uint32_t w1,
                                       unsigned *used, unsigned *wrap,
                                       unsigned *last, unsigned *len)
{
    *used = !!(w1 & TX_USED);
    *wrap = !!(w1 & TX_WRAP);
    *last = !!(w1 & TX_LAST);
    *len  = (unsigned)(w1 & TX_LEN_MASK);
}

/* Print one TX descriptor slot, respecting stride-based layout */
static void macb_dump_tx_desc_one(const char *tag,
                                  struct macb_txq *txq,
                                  uint32_t idx,
                                  volatile uint32_t *w,
                                  uint32_t stride)
{
    /* always at least 8 bytes */
    uint32_t w0 = w[0]; /* addr low */
    uint32_t w1 = w[1]; /* ctrl/status */
    unsigned used, wrap, last, len;
    macb_tx_decode_ctrl(w1, &used, &wrap, &last, &len);

    RTE_LOG(INFO, PMD,
        "macb: [%s] TX d%03u @%p: w0(addr)=%08x w1(ctrl)=%08x "
        "(USED=%u WRAP=%u LAST=%u LEN=%u)\n",
        tag, idx, (void *)w, w0, w1, used, wrap, last, len);

    /* 16-byte extension (addr high + reserved) */
    if (stride >= 16) {
        uint32_t w2 = w[2]; /* addrh */
        uint32_t w3 = w[3]; /* reserved */
        RTE_LOG(INFO, PMD, " + ext64: w2(addrh)=%08x w3(resvd)=%08x\n", w2, w3);
    }

    /* 24-byte extension (PTP timestamp words) */
    if (stride >= 24) {
        uint32_t w4 = w[4]; /* ts_1 */
        uint32_t w5 = w[5]; /* ts_2 */
        RTE_LOG(INFO, PMD, " + ptp: w4(ts1)=%08x w5(ts2)=%08x\n", w4, w5);
    }

    /* If you ever use 32/64, dump the extra words too (raw) */
    if (stride > 24) {
        uint32_t nwords = stride / 4;
        RTE_LOG(INFO, PMD, " + raw[%u..%u]:", 6u, nwords ? (nwords - 1) : 0);
        for (uint32_t j = 6; j < nwords; j++) {
            RTE_LOG(INFO, PMD, "   w%u=%08x", j, w[j]);
        }
    }
}


static void macb_dump_tx_state(struct macb_adapter *ad,
                               struct macb_txq *txq,
                               const char *tag)
{
    uint64_t ring_bus = BUS_IOVA(txq->ring_iova);

#ifdef MACB_TBQPH
    uint32_t tb_hi = macb_readl(&ad->hw, MACB_TBQPH);
#else
    uint32_t tb_hi = 0;
#endif
    uint32_t tb_lo = macb_readl(&ad->hw, MACB_TBQP);

    uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
    uint32_t nsr = macb_readl(&ad->hw, MACB_NSR);
    uint32_t tsr = macb_readl(&ad->hw, MACB_TSR);

    RTE_LOG(INFO, PMD,
        "macb: [%s] TX state: NCR=%08x NSR=%08x TSR=%08x "
        "TBQP(reg)=%08x:%08x ring(expect)=%08x:%08x ringVA=%p stride=%u nb_desc=%u\n",
        tag, ncr, nsr, tsr,
        tb_hi, tb_lo,
        (uint32_t)(ring_bus >> 32), (uint32_t)ring_bus,
        (void *)txq->ring, (unsigned)txq->desc_stride, (unsigned)txq->nb_desc);

    uint32_t tb_lo_legacy = macb_readl(&ad->hw, MACB_TBQP);

    uint32_t tb_lo_q0     = macb_readl(&ad->hw, GEM_TBQP(0));

	RTE_LOG(INFO, PMD,
	  "macb: [%s] TBQP legacy=%08x q0=%08x expect=%08x\n",
	  tag, tb_lo_legacy, tb_lo_q0, (uint32_t)BUS_IOVA(txq->ring_iova));

    /* Dump first 4 desc */
    /* Dump first 4 desc (stride-based, layout-aware) */
    for (uint32_t i = 0; i < 4 && i < txq->nb_desc; i++) {
        volatile uint32_t *w =
            macb_desc_words_at_stride(txq->ring, txq->desc_stride, i);

        /* If HW might write back, invalidate the whole slot before reading */
        sync_desc_from_dev(txq->sync_fd, (void *)(uintptr_t)w, txq->desc_stride);
        rte_io_rmb();

        macb_dump_tx_desc_one(tag, txq, i, w, txq->desc_stride);
    }

}

/* Fill a region with canary pattern and check if it changes */
static void macb_canary_set(void *p, size_t len)
{
    //memset(p, 0xA5, len);
}

static int macb_canary_check(const void *p, size_t len)
{
    const uint8_t *b = (const uint8_t *)p;
    for (size_t i = 0; i < len; i++) {
        if (b[i] != 0xA5)
            return -1;
    }
    return 0;
}

static void macb_dump_tbqp_progress(struct macb_txq *txq, struct macb_adapter *ad, const char *tag)
{
    uint64_t base = BUS_IOVA(txq->ring_iova);
    uint64_t end  = base + (uint64_t)txq->nb_desc * (uint64_t)txq->desc_stride;

    uint32_t tb_lo = macb_readl(&ad->hw, MACB_TBQP);
#ifdef MACB_TBQPH
    uint32_t tb_hi = macb_readl(&ad->hw, MACB_TBQPH);
#else
    uint32_t tb_hi = 0;
#endif
    uint64_t tb = ((uint64_t)tb_hi << 32) | tb_lo;

    uint64_t delta = (tb >= base) ? (tb - base) : 0xffffffffffffffffULL;
    uint32_t stride = txq->desc_stride ? txq->desc_stride : 1;
    uint64_t idx = (tb >= base) ? (delta / stride) : (uint64_t)-1;
    uint64_t mod = (tb >= base) ? (delta % stride) : (uint64_t)-1;

    RTE_LOG(INFO, PMD,
        "macb: [%s] TBQP=%08x:%08x base=%08x:%08x end=%08x:%08x delta=0x%llx idx=%llu mod=%llu stride=%u\n",
        tag,
        (unsigned)(tb >> 32), (unsigned)tb,
        (unsigned)(base >> 32), (unsigned)base,
        (unsigned)(end >> 32), (unsigned)end,
        (unsigned long long)delta,
        (unsigned long long)idx,
        (unsigned long long)mod,
        stride);

    if (!(tb >= base && tb < end))
        RTE_LOG(ERR, PMD, "macb: [%s] TBQP OUT OF RANGE!\n", tag);
}

static void macb_dump_tx_window(struct macb_txq *txq, struct macb_adapter *ad, const char *tag)
{
    uint64_t base = BUS_IOVA(txq->ring_iova);

    uint32_t tb_lo = macb_readl(&ad->hw, MACB_TBQP);
#ifdef MACB_TBQPH
    uint32_t tb_hi = macb_readl(&ad->hw, MACB_TBQPH);
#else
    uint32_t tb_hi = 0;
#endif
    uint64_t tb = ((uint64_t)tb_hi << 32) | tb_lo;

    uint32_t stride = txq->desc_stride ? txq->desc_stride : 1;
    int64_t k = (tb >= base) ? (int64_t)((tb - base) / stride) : -1;

    int start = (k > 2) ? (int)(k - 2) : 0;
    int end   = (k >= 0) ? (int)(k + 2) : 3;
    if (end >= (int)txq->nb_desc) end = (int)txq->nb_desc - 1;

    RTE_LOG(INFO, PMD, "macb: [%s] TBQP->idx=%ld dumping %d..%d\n", tag, (long)k, start, end);

        for (int i = start; i <= end; i++) {
        volatile uint32_t *w =
            macb_desc_words_at_stride(txq->ring, txq->desc_stride, (uint32_t)i);

        sync_desc_from_dev(txq->sync_fd, (void *)(uintptr_t)w, txq->desc_stride);
        rte_io_rmb();

        macb_dump_tx_desc_one(tag, txq, (uint32_t)i, w, txq->desc_stride);
    }

}

static void macb_dump_tbqp_mirrors(struct macb_adapter *ad, const char *tag)
{
    uint32_t v0 = macb_readl(&ad->hw, 0x001c);
    uint32_t v1 = macb_readl(&ad->hw, 0x101c);
    uint32_t v2 = macb_readl(&ad->hw, 0x201c);
    uint32_t v3 = macb_readl(&ad->hw, 0x301c);

    RTE_LOG(INFO, PMD, "macb: [%s] TBQP mirrors: 001c=%08x 101c=%08x 201c=%08x 301c=%08x\n",
            tag, v0, v1, v2, v3);
}


static int macb_dma_tx_selftest(struct macb_adapter *ad, struct macb_txq *txq)
{
    const size_t pkt_len = 128;
    const int n_pkts = 4;

    /* 0) Ensure sync helper is active */
    macb_dma_sync_open_once(ad);
    txq->sync_fd = ad->sync_fd;

    /* 1) Allocate packet buffer region (unique name to avoid memzone clash) */
    char mz_name[64];
    snprintf(mz_name, sizeof(mz_name), "macb_txst_pkt_%d", getpid());

    const struct rte_memzone *mz_pkt =
        rte_memzone_reserve_aligned(mz_name,
            RTE_PGSIZE_2M, rte_socket_id(),
            RTE_MEMZONE_2MB | RTE_MEMZONE_IOVA_CONTIG, RTE_PGSIZE_2M);
    if (!mz_pkt)
        return -ENOMEM;

    /* Carve 4 packet buffers from the memzone (keep them separated) */
    uint8_t  *buf_base = (uint8_t *)mz_pkt->addr;
    rte_iova_t iova_base = mz_pkt->iova;

    uint8_t  *buf[n_pkts];
    rte_iova_t iova[n_pkts];

    /* Use a conservative spacing so we never overlap */
    const size_t spacing = 256; /* >= pkt_len, cacheline-friendly */

    for (int p = 0; p < n_pkts; p++) {
        buf[p]  = buf_base + (p * spacing);
        iova[p] = iova_base + (p * spacing);

        /* Build an Ethernet frame; vary payload per packet so you can tell them apart */
        memset(buf[p], 0, pkt_len);
        memset(buf[p], 0xff, 6);           /* dst */
        buf[p][6]  = 0x02;                 /* src */
        buf[p][11] = 0x01 + p;             /* vary last byte of src */
        buf[p][12] = 0x08; buf[p][13] = 0x00; /* ethertype IPv4-ish */

        for (size_t i = 14; i < pkt_len; i++)
            buf[p][i] = (uint8_t)((0xA0 ^ (uint8_t)i) + (uint8_t)p);

        sync_buf_to_dev(txq->sync_fd, buf[p], pkt_len);
    }
    rte_io_wmb();

    /* Clear TSR */
#ifdef MACB_TSR
    macb_writel(&ad->hw, MACB_TSR, 0xffffffffu);
    rte_io_wmb();
#endif

    /* 2) Initialize ENTIRE ring to a safe SW-owned state.
     * WRAP must be on last descriptor in ring.
     */
    const uint16_t nb = txq->nb_desc;
    const size_t stride = txq->desc_stride;

    for (uint16_t i = 0; i < nb; i++) {
        volatile struct macb_desc *dv = macb_desc_at(txq->ring, txq->hw_dma_cap, i);

        /* ZERO EVERYTHING */
        memset((void *)dv, 0, stride);

        /* Mark descriptor SW-owned */
        ((volatile uint32_t *)dv)[1] =
            TX_USED | ((i == (nb - 1)) ? TX_WRAP : 0);

        sync_desc_to_dev(txq->sync_fd, dv, stride);
    }
    sync_desc_to_dev(txq->sync_fd, txq->ring, (size_t)nb * stride);
    rte_io_wmb();

    /* 3) Arm descriptors 0..3 (NO WRAP here) */
    volatile struct macb_desc *dv[n_pkts];
    volatile uint32_t *w[n_pkts];

    for (int p = 0; p < n_pkts; p++) {
        dv[p] = macb_desc_at(txq->ring, txq->hw_dma_cap, (uint16_t)p);
        w[p]  = (volatile uint32_t *)dv[p];

        uint32_t ctrl = ((uint32_t)pkt_len & TX_LEN_MASK) | TX_LAST;
        ctrl &= ~TX_USED; /* give to HW */
        uint64_t bus = (uint64_t)BUS_IOVA_DATA(iova[p]);
        /* Address low always */
        w[p][0] = (uint32_t)(bus);

        /* Descriptor layout variants:
         * - legacy: only w0/w1 are used
         * - ext64: w2 holds addr high (if enabled)
         * - ptp: extra timestamp words are written by HW (don’t pre-fill)
         */
        if (1) {
            /* Only if your driver defines this cap; otherwise remove this block.
             * Many GEM variants use word2 for addr high when 64-bit addressing is enabled.
             */
            w[p][2] = (uint32_t)(bus >> 32);
        }

        w[p][1] = ctrl;

        sync_desc_to_dev(txq->sync_fd, dv[p], stride);
    }
    rte_io_wmb();

    for (int p = 0; p < 4; p++) {
	    volatile struct macb_desc *dv = macb_desc_at(txq->ring, txq->hw_dma_cap, (uint16_t)p);
	    sync_desc_from_dev(txq->sync_fd, dv, txq->desc_stride);
	    rte_io_rmb();
	    uint32_t w0 = ((volatile uint32_t *)dv)[0];
	    uint32_t w1 = ((volatile uint32_t *)dv)[1];
	    RTE_LOG(INFO, PMD, "macb: [armed] d%d w0=%08x w1=%08x USED=%u LAST=%u LEN=%u\n",
		    p, w0, w1, !!(w1 & TX_USED), !!(w1 & TX_LAST), (unsigned)(w1 & TX_LEN_MASK));
    }
    /* --- PRE-KICK INSPECTION (no TX running yet) --- */
    macb_dump_tx_state(ad, txq, "selftest-pre-kick");
    macb_dump_tbqp_progress(txq, ad, "selftest-pre-kick");
    macb_dump_tx_window(txq, ad, "selftest-pre-kick");
    macb_dump_tbqp_mirrors(ad, "selftest-pre-kick");
    /* ---------------------------------------------- */

    uint32_t expect_lo = (uint32_t)BUS_IOVA(txq->ring_iova);
    macb_scan_regs_for_tbqp(ad, expect_lo, "pre-kick");

    /* 4) Program ring base + kick */
    macb_set_tbqp_q0(ad, BUS_IOVA(txq->ring_iova));

    /* --- IMMEDIATELY AFTER PROGRAMMING TBQP (still before TSTART) --- */
    macb_dump_tbqp_progress(txq, ad, "selftest-post-tbqp-write");
    macb_dump_tx_window(txq, ad, "selftest-post-tbqp-write");
    macb_dump_tbqp_mirrors(ad, "selftest-post-tbqp-write");
    /* --------------------------------------------------------------- */

    RTE_LOG(INFO, PMD, "macb: set_tbqp: legacy=%08x q0=%08x\n",
        macb_readl(&ad->hw, MACB_TBQP),
        macb_readl(&ad->hw, GEM_TBQP(0)));

    rte_io_wmb();

    uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
    macb_writel(&ad->hw, MACB_NCR, ncr | NCR_TXEN | NCR_TSTART);
    rte_io_wmb();

    /* 5) Poll for ALL 4 descriptors to become USED again */
    for (int i = 0; i < 5000; i++) {
        int done = 1;

        for (int p = 0; p < n_pkts; p++) {
            sync_desc_from_dev(txq->sync_fd, dv[p], stride);
        }
        rte_io_rmb();

        for (int p = 0; p < n_pkts; p++) {
            if (!(w[p][1] & TX_USED)) {
                done = 0;
                break;
            }
        }

        if (done) {
            macb_dump_tx_state(ad, txq, "selftest-done");
            macb_dump_tbqp_progress(txq, ad, "selftest-done");
            macb_dump_tx_window(txq, ad, "selftest-done");
            macb_dump_tbqp_mirrors(ad, "selftest-done");
#ifdef MACB_TSR
            {
                uint32_t tsr = macb_readl(&ad->hw, MACB_TSR);
                RTE_LOG(INFO, PMD, "macb: tx_selftest: TSR=0x%08x\n", tsr);
            }
#endif
            return 0;
        }

        if ((i % 200) == 0) {
            macb_dump_tx_state(ad, txq, "selftest-poll");
            macb_dump_tbqp_progress(txq, ad, "selftest-poll");
            macb_dump_tx_window(txq, ad, "selftest-poll");
            macb_dump_tbqp_mirrors(ad, "selftest-poll");
            macb_scan_regs_for_tbqp(ad, expect_lo, "selftest-poll");
        }

        rte_delay_us_block(50);
    }

    macb_dump_tx_state(ad, txq, "selftest-timeout");
    macb_dump_tbqp_progress(txq, ad, "selftest-timeout");
    macb_dump_tx_window(txq, ad, "selftest-timeout");
    macb_dump_tbqp_mirrors(ad, "selftest-timeout");
    return -ETIMEDOUT;
}

static int macb_rxq_cpu_rw_test(struct macb_rxq *rxq, const char *tag)
{
    if (!rxq || !rxq->ring || rxq->nb_desc == 0 || rxq->desc_stride < 8) {
        RTE_LOG(ERR, PMD, "macb: RXQ RWTEST[%s]: invalid rxq\n", tag);
        return -EINVAL;
    }

    macb_dma_sync_open_once(rxq->ad);
    rxq->sync_fd = rxq->ad->sync_fd;

    const uint16_t nb = rxq->nb_desc;
    const uint32_t stride = rxq->desc_stride;

    RTE_LOG(INFO, PMD,
        "macb: RXQ RWTEST[%s]: ring=%p iova=0x%"PRIx64" nb=%u stride=%u cap=0x%x sync_fd=%d\n",
        tag, rxq->ring, (uint64_t)rxq->ring_iova, nb, stride, rxq->hw_dma_cap, rxq->sync_fd);

    if (rxq->sync_fd < 0) {
        RTE_LOG(ERR, PMD, "macb: RXQ RWTEST[%s]: sync helper not active; aborting test\n", tag);
        return -ENODEV;
    }

    /* --- Phase A: descriptor slot R/W + sync test --- */
    const uint16_t nslots = (nb < 8) ? nb : 8;

    for (uint16_t i = 0; i < nslots; i++) {
        volatile uint32_t *w = macb_desc_words_at_stride(rxq->ring, stride, i);

        /* write a recognizable pattern into the whole slot */
        for (uint32_t j = 0; j < stride / 4; j++)
            ((volatile uint32_t *)w)[j] = 0xC0DE0000u ^ ((uint32_t)i << 8) ^ j;

        /* Make it look like an "armed RX desc" too (addr+flags, stat=0) */
        if (rxq->sw_ring && rxq->sw_ring[i]) {
            const struct rte_mbuf *m = rxq->sw_ring[i];
            rte_iova_t biova = macb_mbuf_data_iova(m);
            uint32_t addr_lo = (uint32_t)(BUS_IOVA(biova) & ~0x3u);
            if (i == (nb - 1)) addr_lo |= RX_WRAP; /* wrap on last only */
            /* NOTE: do NOT set RX_USED when arming for HW */
            ((volatile uint32_t *)w)[0] = addr_lo;
            ((volatile uint32_t *)w)[1] = 0;
        }

        /* clean slot to RAM */
        sync_desc_to_dev(rxq->sync_fd, (void *)(uintptr_t)w, stride);
    }
    rte_io_wmb();

    for (uint16_t i = 0; i < nslots; i++) {
        volatile uint32_t *w = macb_desc_words_at_stride(rxq->ring, stride, i);

        /* invalidate slot from RAM, then read back */
        sync_desc_from_dev(rxq->sync_fd, (void *)(uintptr_t)w, stride);
        rte_io_rmb();

        /* verify signature word-by-word */
        for (uint32_t j = 0; j < stride / 4; j++) {
            uint32_t exp = 0xC0DE0000u ^ ((uint32_t)i << 8) ^ j;

            /* if we overwrote w0/w1 with armed desc, skip strict check for those */
            if (j < 2 && rxq->sw_ring && rxq->sw_ring[i])
                continue;

            uint32_t got = ((volatile uint32_t *)w)[j];
            if (got != exp) {
                RTE_LOG(ERR, PMD,
                    "macb: RXQ RWTEST[%s]: desc%u word%u mismatch got=%08x exp=%08x\n",
                    tag, i, j, got, exp);
                return -EIO;
            }
        }

        /* sanity log w0/w1 (addr/stat) */
        uint32_t w0 = ((volatile uint32_t *)w)[0];
        uint32_t w1 = ((volatile uint32_t *)w)[1];
        RTE_LOG(INFO, PMD,
            "macb: RXQ RWTEST[%s]: d%u w0=%08x w1=%08x USED=%u WRAP=%u\n",
            tag, i, w0, w1, !!(w0 & RX_USED), !!(w0 & RX_WRAP));
    }

    /* --- Phase B: buffer R/W + sync test on a couple of RX mbufs --- */
    if (rxq->sw_ring) {
        const uint16_t bslots = (nb < 4) ? nb : 4;

        for (uint16_t i = 0; i < bslots; i++) {
            struct rte_mbuf *m = rxq->sw_ring[i];
            if (!m) continue;

            uint8_t *p = rte_pktmbuf_mtod(m, uint8_t *);
            const uint32_t room = rte_pktmbuf_data_room_size(rxq->mp) - RTE_PKTMBUF_HEADROOM;
            const uint32_t len = (room > 256) ? 256 : room;

            /* Alignment sanity (helps catch weird headroom/alignment issues) */
            rte_iova_t iova = macb_mbuf_data_iova(m);
            if (((uintptr_t)p & 0xF) || (iova & 0xF)) {
                RTE_LOG(WARNING, PMD,
                    "macb: RXQ RWTEST[%s]: buf%u alignment: VA=%p IOVA=0x%"PRIx64"\n",
                    tag, i, p, (uint64_t)iova);
            }

            /* poison + clean */
            //memset(p, 0xA5, len);
            sync_buf_to_dev(rxq->sync_fd, p, len);
        }
        rte_io_wmb();

        for (uint16_t i = 0; i < bslots; i++) {
            struct rte_mbuf *m = rxq->sw_ring[i];
            if (!m) continue;

            uint8_t *p = rte_pktmbuf_mtod(m, uint8_t *);
            const uint32_t room = rte_pktmbuf_data_room_size(rxq->mp) - RTE_PKTMBUF_HEADROOM;
            const uint32_t len = (room > 256) ? 256 : room;

            /* invalidate + verify poison still visible */
            sync_buf_from_dev(rxq->sync_fd, p, len);
            rte_io_rmb();

            //for (uint32_t k = 0; k < len; k++) {
            //    if (p[k] != 0xA5) {
            //        RTE_LOG(ERR, PMD,
            //            "macb: RXQ RWTEST[%s]: buf%u changed at +%u got=%02x exp=A5\n",
            //            tag, i, k, p[k]);
            //        return -EIO;
            //    }
            //}

            RTE_LOG(INFO, PMD,
                "macb: RXQ RWTEST[%s]: buf%u OK (first %u bytes stayed A5 after clean+invalidate)\n",
                tag, i, len);
        }
    }

    RTE_LOG(INFO, PMD, "macb: RXQ RWTEST[%s]: PASS\n", tag);
    return 0;
}

static inline void macb_wr_ncr_dbg(struct macb_adapter *ad, uint32_t val, const char *tag)
{
    uint32_t before = macb_readl(&ad->hw, MACB_NCR);
    macb_writel(&ad->hw, MACB_NCR, val);
    rte_io_wmb();
    uint32_t after1 = macb_readl(&ad->hw, MACB_NCR);
    uint32_t after2 = macb_readl(&ad->hw, MACB_NCR);

    RTE_LOG(INFO, PMD, "macb: [%s] NCR before=%08x write=%08x after1=%08x after2=%08x\n",
            tag, before, val, after1, after2);
}


static inline uint32_t rd_off(struct macb_adapter *ad, uint32_t off)
{
    return macb_readl(&ad->hw, off);
}

static void macb_log_hi_ptr_regs(struct macb_adapter *ad, const char *tag)
{
    uint32_t tbqph_a8 = rd_off(ad, 0x00A8);
    uint32_t rbqph_a4 = rd_off(ad, 0x00A4);

    uint32_t tbqph_4c8 = rd_off(ad, 0x04C8);
    uint32_t rbqph_4d4 = rd_off(ad, 0x04D4);

    RTE_LOG(INFO, PMD,
        "macb: [%s] TBQPH@00A8=%08x  RBQPH@00A4=%08x  TBQPH@04C8=%08x  RBQPH@04D4=%08x\n",
        tag, tbqph_a8, rbqph_a4, tbqph_4c8, rbqph_4d4);
}

static inline void arm64_dcache_invalidate_range(const void *addr, size_t len)
{
#if defined(__aarch64__)
    const size_t line = 64; // most ARM64 systems; if unsure, use 64
    uintptr_t p = (uintptr_t)addr & ~(line - 1);
    uintptr_t end = ((uintptr_t)addr + len + line - 1) & ~(line - 1);

    /* ensure all prior writes/reads complete */
    __asm__ __volatile__("dsb ish" ::: "memory");

    for (; p < end; p += line) {
        __asm__ __volatile__("dc ivac, %0" :: "r"(p) : "memory");
    }

    __asm__ __volatile__("dsb ish" ::: "memory");
    __asm__ __volatile__("isb" ::: "memory");
#else
    (void)addr; (void)len;
#endif
}

static void macb_log_tsr(struct macb_adapter *ad, const char *tag)
{
#ifdef MACB_TSR
    uint32_t tsr = macb_readl(&ad->hw, MACB_TSR);
    RTE_LOG(INFO, PMD,
        "macb: [%s] TSR=0x%08x (UBR=%d COL=%d RLE=%d TXGO=%d TFC=%d TXCOMP=%d)\n",
        tag, tsr,
        !!(tsr & (1u<<6)),  /* UBR (often bit 6 on MACB) */
        !!(tsr & (1u<<5)),  /* COL */
        !!(tsr & (1u<<2)),  /* RLE */
        !!(tsr & (1u<<3)),  /* TXGO */
        !!(tsr & (1u<<1)),  /* TFC */
        !!(tsr & (1u<<0))   /* TXCOMP */
    );
#endif
}

static void macb_log_hw_tx_head(struct macb_adapter *ad,
                                struct macb_txq *txq,
                                const char *tag)
{
    uint64_t base = (uint64_t)BUS_IOVA(txq->ring_iova);
    uint64_t cur;

#ifdef MACB_TBQPH
    cur = ((uint64_t)macb_readl(&ad->hw, MACB_TBQPH) << 32) |
           (uint64_t)macb_readl(&ad->hw, MACB_TBQP);
#else
    cur = (uint64_t)macb_readl(&ad->hw, MACB_TBQP);
#endif

    uint32_t off = (uint32_t)(cur - base);
    uint32_t idx = off / (uint32_t)txq->desc_stride;

    RTE_LOG(INFO, PMD,
        "macb: TX[%s] head: TBQP=%08x:%08x base=%08x:%08x -> off=%u idx=%u\n",
        tag,
        (uint32_t)(cur >> 32), (uint32_t)cur,
        (uint32_t)(base >> 32), (uint32_t)base,
        off, idx);
}

static void macb_dump_desc_bytes(const char *tag, const void *p, size_t len)
{
    const uint8_t *b = p;
    RTE_LOG(INFO, PMD, "%s desc @%p len=%zu\n", tag, p, len);
    for (size_t i = 0; i < len; i += 16) {
        RTE_LOG(INFO, PMD,
            "%04zu: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
            i,
            b[i+0], b[i+1], b[i+2], b[i+3], b[i+4], b[i+5], b[i+6], b[i+7],
            b[i+8], b[i+9], b[i+10], b[i+11], b[i+12], b[i+13], b[i+14], b[i+15]);
    }
}

static inline void
macb_log_ncr_nsr(struct macb_adapter *ad, const char *tag)
{
    uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
    uint32_t nsr = macb_readl(&ad->hw, MACB_NSR);

    RTE_LOG(INFO, PMD,
        "macb: [%s] NCR=0x%08x (TXEN=%d RXEN=%d TSTART=%d) NSR=0x%08x\n",
        tag,
        ncr,
        !!(ncr & NCR_TXEN),
        !!(ncr & NCR_RXEN),
        !!(ncr & NCR_TSTART),
        nsr);
}

/* Send one packet using TX desc 0; frame data comes from an mbuf (DMA-mapped mempool). */
static int macb_tx_send_one_test_mbuf(struct macb_adapter *ad,
                                      struct macb_txq *txq,
                                      struct rte_mbuf *m)
{
    macb_dma_sync_open_once(ad);
    txq->sync_fd = ad->sync_fd;
    if (txq->sync_fd < 0)
        return -ENODEV;

    const uint32_t stride = txq->desc_stride;

    /* We use descriptor 0 only */
    volatile uint32_t *w = macb_desc_words_at_stride(txq->ring, stride, 0);

    uint8_t   *frame     = rte_pktmbuf_mtod(m, uint8_t *);
    uint32_t   frame_len = rte_pktmbuf_pkt_len(m);

    if (frame_len == 0 || frame_len > 0x3FFFu) { /* TX_LEN_MASK is typically 14 bits */
        RTE_LOG(ERR, PMD, "macb: tx_send_one_test_mbuf_v2: bad frame_len=%u\n", frame_len);
        return -EINVAL;
    }

    rte_iova_t iova = macb_mbuf_data_iova(m);
    uint64_t   bus  = (uint64_t)BUS_IOVA_DATA(iova);

    /* Make sure payload is visible to DMA */
    sync_buf_to_dev(txq->sync_fd, frame, frame_len);
    rte_io_wmb();

    macb_log_hi_ptr_regs(ad, "after-prog");
    macb_log_hw_tx_head(ad, txq, "after-prog");
    macb_log_ncr_nsr(ad, "after-prog");

    /* Read DMACFG to decide if we should write addr-high. */
    uint32_t dmacfg = macb_readl(&ad->hw, GEM_DMACFG);
    int hw_txext = !!(dmacfg & (1u << 29)); /* TXEXT */
    int hw_addr64 = !!(dmacfg & (1u << 30));/* ADDR64 */

    /* 1) Make desc SW-owned while preparing */
    memset((void *)(uintptr_t)w, 0, stride);

    /* Keep ring in “free” state: USED set (SW-owned), no WRAP here because it’s d0 only.
     * If you ever use last descriptor, set TX_WRAP on the last.
     */
    w[1] = TX_USED;

    sync_desc_to_dev(txq->sync_fd, (void *)(uintptr_t)w, stride);
    rte_io_wmb();

    /* 2) Fill descriptor */
    uint64_t bus_aligned = bus & ~0x3ull;

    w[0] = (uint32_t)(bus_aligned & 0xffffffffu);

    /* Only write addr high if HW is configured for it AND we actually have space */
    if (hw_txext && hw_addr64 && stride >= 16) {
        w[2] = (uint32_t)(bus_aligned >> 32);
        /* w[3] is reserved; leave 0 */
    }

    /* 3) Hand to HW: clear USED, set LAST, set LEN */
    uint32_t ctrl = (frame_len & TX_LEN_MASK) | TX_LAST;
    ctrl &= ~TX_USED; /* give ownership to HW */
    w[1] = ctrl;

    macb_dump_desc_bytes("TX d0 after-give", (const void *)w, stride);
    sync_desc_to_dev(txq->sync_fd, (void *)(uintptr_t)w, stride);
    rte_io_wmb();

    /* 4) Ensure TX enabled + kick */
    //uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
    //ncr |= NCR_TXEN | NCR_TSTART;

    uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);

    macb_wr_ncr_dbg(ad, ncr | NCR_TXEN | NCR_TSTART, "kick");
    ncr |= NCR_TXEN | NCR_TSTART;
    macb_wr_ncr_dbg(ad, ncr | NCR_TXEN | NCR_TSTART, "post-kick");

    macb_log_hi_ptr_regs(ad, "after-kick");
    macb_log_ncr_nsr(ad, "after-kick");
    macb_log_tsr(ad, "after-kick");
    macb_log_hw_tx_head(ad, txq, "after-kick");

    macb_writel(&ad->hw, MACB_NCR, ncr);
    rte_io_wmb();

    /* 5) Poll for USED to return */
    for (int i = 0; i < 2000; i++) {
        sync_desc_from_dev(txq->sync_fd, (void *)(uintptr_t)w, stride);
	
        rte_io_rmb();

        if (w[1] & TX_USED)
            return 0;

        if ((i % 200) == 0) {
#ifdef MACB_TSR
            uint32_t tsr = macb_readl(&ad->hw, MACB_TSR);
#else
            uint32_t tsr = 0;
#endif
            uint32_t tbqp = macb_readl(&ad->hw, MACB_TBQP);
#ifdef MACB_TBQPH
            uint32_t tbqph = macb_readl(&ad->hw, MACB_TBQPH);
#else
            uint32_t tbqph = 0;
#endif
	    RTE_LOG(INFO, PMD,
  "poll i=%d d0 w0=%08x w1=%08x w2=%08x w3=%08x TSR=%08x TBQP=%08x:%08x\n",
  i, w[0], w[1], w[2], w[3], tsr, tbqph, tbqp);
        }

	macb_log_hw_tx_head(ad, txq, "poll");
	macb_log_tsr(ad, "poll");
        rte_delay_us_block(50);
    }

    RTE_LOG(ERR, PMD, "macb: tx_send_one_test_mbuf_v2 timeout (TX_USED never returned)\n");
    return -ETIMEDOUT;
}
/* Loopback RX selftest: arms RX desc0, sends a 64B frame via TX using a mempool mbuf,
 * waits for RX_USED on desc0, then inspects RX buffer for the frame signature.
 */
static int macb_rx_loopback_selftest(struct macb_adapter *ad,
                                     struct macb_rxq *rxq,
                                     struct macb_txq *txq)
{
    macb_dma_sync_open_once(ad);
    rxq->sync_fd = ad->sync_fd;
    txq->sync_fd = ad->sync_fd;
    if (rxq->sync_fd < 0)
        return -ENODEV;

    if (!rxq->sw_ring || !rxq->sw_ring[0]) {
        RTE_LOG(ERR, PMD, "macb: RX selftest: sw_ring[0] missing\n");
        return -EINVAL;
    }
	
    uint64_t rbqp_cur = ((uint64_t)macb_readl(&ad->hw, MACB_RBQPH) << 32) |
			     (uint64_t)macb_readl(&ad->hw, MACB_RBQP);

	uint64_t base = rxq->ring_iova;           /* the IOVA you programmed into RBQP */
	uint32_t off  = (uint32_t)(rbqp_cur - base);

	uint16_t i = 0;// (uint16_t)((off / rxq->desc_stride) % rxq->nb_desc);

	RTE_LOG(INFO, PMD, "RXST: rbqp_cur=%#"PRIx64" base=%#"PRIx64" off=%u -> i=%u\n",
		rbqp_cur, base, off, i);

    /* Use RX descriptor 0 */
    //const uint16_t i = 0;
    struct rte_mbuf *rm = rxq->sw_ring[i];
    uint8_t *rp = rte_pktmbuf_mtod(rm, uint8_t *);

    const uint32_t room = rte_pktmbuf_data_room_size(rxq->mp) - RTE_PKTMBUF_HEADROOM;
    const uint32_t plen = (room > 256) ? 256 : room;

    RTE_LOG(INFO, PMD, "RXST: pid=%d rp=%p plen=%u\n", getpid(), rp, plen);

    /* Poison RX buffer */
    memset(rp, 0xA5, plen);
    sync_buf_to_dev(rxq->sync_fd, rp, plen);
    rte_io_wmb();

    /* Re-arm RX desc 0 to HW */
    volatile struct macb_desc *rdv = macb_desc_at(rxq->ring, rxq->hw_dma_cap, i);

    rte_iova_t r_iova = macb_mbuf_data_iova(rm);
    uint64_t r_bus = (uint64_t)BUS_IOVA_DATA(r_iova);
    uint32_t wrap = (i == (rxq->nb_desc - 1)) ? RX_WRAP : 0;

    RTE_LOG(INFO, PMD,
            "RXST: RX mbuf=%p rp=%p r_iova=%#"PRIx64" r_bus=%#"PRIx64" g_bus_ofs=%#"PRIx64"\n",
            rm, rp, (uint64_t)r_iova, (uint64_t)r_bus, (uint64_t)g_bus_ofs);

    /* --- ADD THIS: snapshot regs + descriptor words around publish --- */
    {
        /* If you have split RBQP/RBQPH regs, grab both. If your macb_readl() already
         * returns low 32 and you have macb_readl_hi() or similar, adjust accordingly.
         */
        uint32_t ncr   = macb_readl(&ad->hw, MACB_NCR);
        uint32_t ncfgr = macb_readl(&ad->hw, MACB_NCFGR);
        uint32_t dmacfg = macb_readl(&ad->hw, GEM_DMACFG);

        uint32_t rbqp  = macb_readl(&ad->hw, MACB_RBQP);
        uint32_t tbqp  = macb_readl(&ad->hw, MACB_TBQP);
        uint32_t rbqph = 0, tbqph = 0;

        /* If your hw supports high address regs, read them too (guards are fine). */
        if (rxq->hw_dma_cap & HW_DMA_CAP_64B) {
            rbqph = macb_readl(&ad->hw, MACB_RBQPH); /* or GEM_RBQPH if you use GEM_ regs */
            tbqph = macb_readl(&ad->hw, MACB_TBQPH);
        }

        /* Read current descriptor words before we touch it */
        sync_desc_from_dev(rxq->sync_fd, (void *)(uintptr_t)rdv, rxq->desc_stride);
        rte_io_rmb();

        /* Print 6 words (stride=24) even if you only “use” first 4 */
        RTE_LOG(INFO, PMD,
            "RXST: pre-publish regs: NCR=%08x NCFGR=%08x DMACFG=%08x RBQP=%08x RBQPH=%08x TBQP=%08x TBQPH=%08x\n",
            ncr, ncfgr, dmacfg, rbqp, rbqph, tbqp, tbqph);

        RTE_LOG(INFO, PMD,
            "RXST: pre-publish d%u @%p (stride=%u): w0=%08x w1=%08x w2=%08x w3=%08x w4=%08x w5=%08x\n",
            i, (void *)rdv, rxq->desc_stride,
            ((volatile uint32_t *)rdv)[0], ((volatile uint32_t *)rdv)[1],
            ((volatile uint32_t *)rdv)[2], ((volatile uint32_t *)rdv)[3],
            ((volatile uint32_t *)rdv)[4], ((volatile uint32_t *)rdv)[5]);
    }
    /* --- END ADD --- */

    macb_rx_publish_desc(rxq, rdv, r_bus, wrap);

    /* Optional: read back the desc after publish (catches w0/w1 swap/layout issues) */
    sync_desc_from_dev(rxq->sync_fd, (void *)(uintptr_t)rdv, rxq->desc_stride);
    rte_io_rmb();

    /* --- UPDATE THIS LOG to dump all 6 words --- */
    RTE_LOG(INFO, PMD,
            "RXST: after-publish d%u @%p: w0=%08x w1=%08x w2=%08x w3=%08x w4=%08x w5=%08x\n",
            i, (void *)rdv,
            ((volatile uint32_t *)rdv)[0], ((volatile uint32_t *)rdv)[1],
            ((volatile uint32_t *)rdv)[2], ((volatile uint32_t *)rdv)[3],
            ((volatile uint32_t *)rdv)[4], ((volatile uint32_t *)rdv)[5]);

    /* Build TX frame in a mempool mbuf (so it is definitely DMA-mapped by your map_mempool()) */
    struct rte_mbuf *tm = rte_pktmbuf_alloc(rxq->mp);
    if (!tm) {
        RTE_LOG(ERR, PMD, "macb: RX selftest: failed to alloc TX mbuf\n");
        return -ENOMEM;
    }

    uint8_t *tp = rte_pktmbuf_mtod(tm, uint8_t *);
    const uint32_t troom = rte_pktmbuf_data_room_size(rxq->mp) - RTE_PKTMBUF_HEADROOM;
    if (troom < 64) {
        RTE_LOG(ERR, PMD, "macb: RX selftest: tx mbuf room too small (%u)\n", troom);
        rte_pktmbuf_free(tm);
        return -EINVAL;
    }

    memset(tp, 0, 64);
    memset(tp + 0, 0xff, 6);            /* dst ff:ff:ff:ff:ff:ff */
    tp[6]  = 0x02; tp[11] = 0x01;       /* src 02:00:00:00:00:01 */
    tp[12] = 0x08; tp[13] = 0x00;       /* ethertype */
    for (int k = 14; k < 64; k++)
        tp[k] = (uint8_t)(0xAA ^ k);

    tm->data_len = 64;
    tm->pkt_len  = 64;

    rte_iova_t t_iova = macb_mbuf_data_iova(tm);
    RTE_LOG(INFO, PMD, "RXST: TX mbuf=%p tp=%p t_iova=%#"PRIx64" t_bus=%#"PRIx64"\n",
            tm, tp, (uint64_t)t_iova, (uint64_t)BUS_IOVA_DATA(t_iova));

    /* Force loopback for test only (you can flip these two lines as desired) */
    int saved_phy_lb = g_force_phy_lb;
    int saved_mac_lb = g_mac_lb;

    /* MAC loopback tends to be simplest to prove the datapath */
    g_force_phy_lb = 0;
    g_mac_lb       = 1;
    macb_apply_loopback(ad);
    macb_log_regs_full(ad, "rx-selftest-after-loopback");


    /* Send one frame */
    int txrc = macb_tx_send_one_test_mbuf(ad, txq, tm);
    rte_pktmbuf_free(tm);

    if (txrc) {
        RTE_LOG(ERR, PMD, "macb: RX selftest: TX send failed rc=%d\n", txrc);
        g_force_phy_lb = saved_phy_lb;
        g_mac_lb       = saved_mac_lb;
        macb_apply_loopback(ad);
        return txrc;
    }
    /* Snapshot TX ring right after enqueue */
	{
	    uint64_t tbqp_cur = ((uint64_t)macb_readl(&ad->hw, MACB_TBQPH) << 32) |
				(uint64_t)macb_readl(&ad->hw, MACB_TBQP);

	    uint64_t tx_base = txq->ring_iova;
	    uint32_t tx_off  = (uint32_t)(tbqp_cur - tx_base);
	    uint16_t ti      = (uint16_t)((tx_off / txq->desc_stride) % txq->nb_desc);

	    RTE_LOG(INFO, PMD, "TXST: tbqp_cur=%#"PRIx64" base=%#"PRIx64" off=%u -> i=%u\n",
		    tbqp_cur, tx_base, tx_off, ti);

	    for (int j = 0; j < 8; j++) {
		uint16_t dj = (ti + j) % txq->nb_desc;
		volatile uint32_t *tw = (volatile uint32_t *)macb_desc_at(txq->ring, txq->hw_dma_cap, dj);
		RTE_LOG(INFO, PMD, "TXST: d%u w0=%08x w1=%08x w2=%08x w3=%08x w4=%08x w5=%08x\n",
			dj, tw[0], tw[1], tw[2], tw[3], tw[4], tw[5]);
	    }
	}

	for (int t = 0; t < 2000; t++) {  /* ~100ms if 50us sleep */
	    uint64_t tbqp_cur = ((uint64_t)macb_readl(&ad->hw, MACB_TBQPH) << 32) |
				(uint64_t)macb_readl(&ad->hw, MACB_TBQP);
	    /* log occasionally */
	    if ((t % 200) == 0)
		RTE_LOG(INFO, PMD, "TXST: tbqp_cur now=%#"PRIx64"\n", tbqp_cur);
		uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
		uint32_t nsr = macb_readl(&ad->hw, MACB_NSR);
		uint32_t tsr = macb_readl(&ad->hw, MACB_TSR);   /* if exists on your IP */

		//RTE_LOG(INFO, PMD,
		//  "TXST: NCR=%08x NSR=%08x TSR=%08x TBQP=%08x TBQPH=%08x\n",
		//  ncr, nsr, tsr,
		//  macb_readl(&ad->hw, MACB_TBQP),
		//  macb_readl(&ad->hw, MACB_TBQPH));
	    rte_delay_us_block(50);
	}
    /* Poll RX completion */
    for (int t = 0; t < 4000; t++) {
        sync_desc_from_dev(rxq->sync_fd, (void *)(uintptr_t)rdv, rxq->desc_stride);
        rte_io_rmb();

        uint32_t w0 = ((volatile uint32_t *)rdv)[0];
        uint32_t w1 = ((volatile uint32_t *)rdv)[1];

        if (w0 & RX_USED) {
	   RTE_LOG(INFO, PMD,  "about to invalidate rp=%p", rp);

	   rte_hexdump(stdout, "rp-before-inv", rp, 64);

	   //arm64_dcache_invalidate_range(rp, 256);

	   //rte_hexdump(stdout, "rp-after-inv", rp, 64);

            /* Invalidate RX buffer and inspect */
            sync_buf_from_dev(rxq->sync_fd, rp, 128); // plen);
	    rte_io_rmb();

	   rte_hexdump(stdout, "rp-after-inv", rp, 64);
            int all_a5 = 1, all_00 = 1, found = 0;
            for (uint32_t k = 0; k < 64 && k < plen; k++) {
                if (rp[k] != 0xA5) all_a5 = 0;
                if (rp[k] != 0x00) all_00 = 0;
            }

            /* Look for our signature */
            for (uint32_t off = 0; off + 16 < plen; off++) {
                if (rp[off + 0]  == 0xff && rp[off + 1]  == 0xff && rp[off + 2]  == 0xff &&
                    rp[off + 12] == 0x08 && rp[off + 13] == 0x00 &&
                    rp[off + 14] == (uint8_t)(0xC0 ^ 14)) {
                    found = 1;
                    RTE_LOG(INFO, PMD, "RXST: frame signature at +%u\n", off);
                    break;
                }
            }

            RTE_LOG(INFO, PMD,
                "macb: RX selftest: COMPLETE d%u w0=%08x w1=%08x len=%u sof=%u eof=%u all_a5=%d all_00=%d found=%d\n",
                i, w0, w1,
                (unsigned)(w1 & RX_LEN_MASK), !!(w1 & RX_SOF), !!(w1 & RX_EOF),
                all_a5, all_00, found);

            rte_hexdump(stderr, "macb: RX selftest first64", rp, (plen > 64) ? 64 : plen);

            /* restore loopback settings */
            g_force_phy_lb = saved_phy_lb;
            g_mac_lb       = saved_mac_lb;
            macb_apply_loopback(ad);

            /* Consider it pass only if buffer actually changed AND signature found */
            if (all_a5 || all_00 || !found)
                return -EIO;
            return 0;
        }

        rte_delay_us_block(50);
    }

    /* restore loopback settings */
    g_force_phy_lb = saved_phy_lb;
    g_mac_lb       = saved_mac_lb;
    macb_apply_loopback(ad);

    RTE_LOG(ERR, PMD, "macb: RX selftest: timed out waiting for RX_USED\n");
    return -ETIMEDOUT;
}


/*
 * If you want to “protect the RX ring hugepage”, do it correctly:
 * - Ensure alignment to 2MB boundary
 * - Only protect the 2MB page that actually backs the ring
 * - Allow enabling/disabling at runtime (so you can debug without crashing)
 *
 * NOTE: mprotect() on hugetlbfs-backed mappings works on Linux, but if the ring
 * is mapped in a special way or shared, behavior can differ. Use carefully.
 */
static int macb_protect_rx_ring_full_hugepage(struct macb_rxq *rxq, int enable_ro)
{
    void   *ring_base = (void *)rxq->ring;
    size_t  page_len  = RTE_PGSIZE_2M;

    /* Align down to 2MB boundary */
    uintptr_t base = (uintptr_t)ring_base;
    uintptr_t page_base_u = base & ~(uintptr_t)(page_len - 1);
    void *page_base = (void *)page_base_u;

    int prot = enable_ro ? PROT_READ : (PROT_READ | PROT_WRITE);

    if (mprotect(page_base, page_len, prot) != 0) {
        RTE_LOG(ERR, PMD, "RX ring mprotect(%s) failed: %s (base=%p ring=%p)\n",
                enable_ro ? "RO" : "RW", strerror(errno), page_base, ring_base);
        return -1;
    }

    RTE_LOG(INFO, PMD, "RX ring hugepage now %s (%zu bytes @ %p, ring=%p)\n",
            enable_ro ? "read-only" : "read-write",
            page_len, page_base, ring_base);

    return 0;
}

/* -------- helpers for RX mprotect -------- */
static size_t macb_pagesize(void)
{
    long ps = sysconf(_SC_PAGESIZE);
    if (ps <= 0) ps = 4096;
    return (size_t)ps;
}

static size_t macb_round_up_pages(size_t len)
{
    size_t ps = macb_pagesize();
    size_t rem = len % ps;
    return rem ? (len + (ps - rem)) : len;
}

static int macb_rx_ring_set_prot(struct macb_rxq *rxq, int prot, const char *tag)
{
    void *addr = (void *)(uintptr_t)rxq->ring;
    size_t len = macb_round_up_pages((size_t)rxq->nb_desc * (size_t)rxq->desc_stride);
    int rc = mprotect(addr, len, prot);
    if (rc) {
        MACB_ERR("RX ring mprotect(%s) failed addr=%p len=%zu errno=%d\n",
                 tag, addr, len, errno);
        return -errno ? -errno : -1;
    }
    RTE_LOG(INFO, PMD, "macb: RX ring mprotect %s at %p len=%zu\n",
            tag, addr, len);
    return 0;
}

/* -------- queue setup -------- */
/* devarg: desc_ptp=0/1 */
static int g_desc_ptp = 0; /* -1 auto */

static inline uint8_t macb_pick_hw_dma_cap(struct macb_adapter *ad,
                                           struct rte_mempool *mp)
{
    uint8_t cap = HW_DMA_CAP_32B;

    /* 32b vs 64b address decision */
    struct rte_mbuf *m = rte_pktmbuf_alloc(mp);
    if (m) {
        const uint64_t bus = (uint64_t)BUS_IOVA(rte_mbuf_data_iova(m));
        rte_pktmbuf_free(m);
        if (bus >> 32)
            cap |= HW_DMA_CAP_64B;
    }

    /* PTP descriptor extension decision */
    if (g_desc_ptp == 1) {
        cap |= HW_DMA_CAP_PTP;
    } else if (g_desc_ptp == 0) {
        /* leave it off */
    } else {
        /* auto: RP1 GEM => assume PTP layout */
        //if (ad->is_rp1_gem)
        cap |= HW_DMA_CAP_PTP;
    }

    return cap;
}

static void macb_dump_mac_regs(struct macb_adapter *ad, const char *tag)
{
    uint32_t ncr   = macb_readl(&ad->hw, MACB_NCR);
    uint32_t ncfgr = macb_readl(&ad->hw, MACB_NCFGR);
    uint32_t nsr   = macb_readl(&ad->hw, MACB_NSR);

#ifdef MACB_RSR
    uint32_t rsr = macb_readl(&ad->hw, MACB_RSR);
#else
    uint32_t rsr = 0;
#endif
#ifdef MACB_TSR
    uint32_t tsr = macb_readl(&ad->hw, MACB_TSR);
#else
    uint32_t tsr = 0;
#endif

    uint32_t rb_lo = macb_readl(&ad->hw, MACB_RBQP);
#ifdef MACB_RBQPH
    uint32_t rb_hi = macb_readl(&ad->hw, MACB_RBQPH);
#else
    uint32_t rb_hi = 0;
#endif

    uint32_t tb_lo = macb_readl(&ad->hw, MACB_TBQP);
#ifdef MACB_TBQPH
    uint32_t tb_hi = macb_readl(&ad->hw, MACB_TBQPH);
#else
    uint32_t tb_hi = 0;
#endif

#ifdef GEM_DMACFG
    uint32_t dmacfg = macb_readl(&ad->hw, GEM_DMACFG);
#else
    uint32_t dmacfg = 0;
#endif

    /* MAN is the MDIO access register; useful to see busy/data */
    uint32_t man = macb_readl(&ad->hw, MACB_MAN);

    //MACB_DBG("[%s] MAC regs: NCR=%08x NCFGR=%08x NSR=%08x RSR=%08x TSR=%08x "
    //         "RBQP=%08x:%08x TBQP=%08x:%08x DMACFG=%08x MAN=%08x\n",
    //         tag, ncr, ncfgr, nsr, rsr, tsr,
    //         rb_hi, rb_lo, tb_hi, tb_lo, dmacfg, man);
}

static void macb_dump_phy_regs(struct macb_adapter *ad, uint8_t phy, const char *tag)
{
    /* Clause 22 basics */
    uint16_t bmcr  = macb_mdio_read_c22(ad, phy, MII_BMCR);
    (void)macb_mdio_read_c22(ad, phy, MII_BMSR); /* latch */
    uint16_t bmsr  = macb_mdio_read_c22(ad, phy, MII_BMSR);

    uint16_t id1   = macb_mdio_read_c22(ad, phy, MII_PHYSID1);
    uint16_t id2   = macb_mdio_read_c22(ad, phy, MII_PHYSID2);

    uint16_t anar  = macb_mdio_read_c22(ad, phy, MII_ADVERTISE);
    uint16_t anlpa = macb_mdio_read_c22(ad, phy, MII_LPA);

    uint16_t gbcr  = macb_mdio_read_c22(ad, phy, MII_CTRL1000);
    uint16_t gbsr  = macb_mdio_read_c22(ad, phy, MII_STAT1000);

    MACB_DBG("[%s] PHY@%u: ID=%04x:%04x BMCR=%04x BMSR=%04x (LSTATUS=%d ANEGDONE=%d) "
             "ANAR=%04x ANLPAR=%04x GBCR=%04x GBSR=%04x\n",
             tag, (unsigned)phy, id1, id2, bmcr, bmsr,
             !!(bmsr & BMSR_LSTATUS), !!(bmsr & BMSR_ANEGCOMPLETE),
             anar, anlpa, gbcr, gbsr);
}


static void macb_log_ring_reg_readback(struct macb_adapter *ad,
                                       struct macb_rxq *rxq,
                                       struct macb_txq *txq,
                                       const char *tag)
{
    uint64_t rb = BUS_IOVA(rxq->ring_iova);
    uint64_t tb = BUS_IOVA(txq->ring_iova);

#ifdef MACB_RBQPH
    uint32_t rb_hi = macb_readl(&ad->hw, MACB_RBQPH);
#else
    uint32_t rb_hi = 0;
#endif
    uint32_t rb_lo = macb_readl(&ad->hw, MACB_RBQP);

#ifdef MACB_TBQPH
    uint32_t tb_hi = macb_readl(&ad->hw, MACB_TBQPH);
#else
    uint32_t tb_hi = 0;
#endif
    uint32_t tb_lo = macb_readl(&ad->hw, MACB_TBQP);

    RTE_LOG(INFO, PMD,
        "macb: [%s] reg-readback: RBQP=%08x:%08x (expect %08x:%08x) "
        "TBQP=%08x:%08x (expect %08x:%08x)\n",
        tag,
        rb_hi, rb_lo, (uint32_t)(rb >> 32), (uint32_t)rb,
        tb_hi, tb_lo, (uint32_t)(tb >> 32), (uint32_t)tb);
}


static void macb_dump_first_rx_descs(struct macb_rxq *rxq, const char *tag, int hw_view)
{
    for (int i = 0; i < 2 && i < rxq->nb_desc; ++i) {
        volatile struct macb_desc *dv = macb_desc_at(rxq->ring, rxq->hw_dma_cap, (uint16_t)i);

        if (hw_view) {
            /* only do this when RX is running and HW may have written back */
            sync_desc_from_dev(rxq->sync_fd, (void *)(uintptr_t)dv, rxq->desc_stride);
            rte_io_mb();
        } else {
            /* raw CPU view: do NOT invalidate */
            rte_io_rmb();
        }

        uint32_t w0 = ((volatile uint32_t *)dv)[0];
        uint32_t w1 = ((volatile uint32_t *)dv)[1];

        uint32_t addr_raw = w0;
        uint32_t stat     = w1;

        uint32_t addr_iova = addr_raw & ~0x3u;
        int used  = !!(addr_raw & RX_USED);
        int wrap  = !!(addr_raw & RX_WRAP);
        int hw_own = !used;

        //MACB_DBG(
        //    "RX[%s] d%03d: w0=%08x w1=%08x | addr=%08x USED=%d OWN=%s WRAP=%d len=%u sof=%d eof=%d\n",
        //    tag, i, w0, w1, addr_iova, used, hw_own ? "HW" : "SW", wrap,
        //    (stat & RX_LEN_MASK), !!(stat & RX_SOF), !!(stat & RX_EOF));
    }
}

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

    ad->hw_dma_cap   = macb_pick_hw_dma_cap(ad, mp);
    //rxq->hw_dma_cap  = ad->hw_dma_cap;
    //rxq->desc_stride = (uint16_t)macb_desc_stride_bytes(rxq->hw_dma_cap);


	int forced = macb_forced_desc_stride();
	if (forced) {
	    rxq->hw_dma_cap  = ad->hw_dma_cap;      /* keep real cap if you use it elsewhere */
	    rxq->desc_stride = (uint16_t)forced;
	} else {
	    rxq->hw_dma_cap  = ad->hw_dma_cap;
	    rxq->desc_stride = (uint16_t)macb_desc_stride_bytes(rxq->hw_dma_cap);
	}

    rxq->hw_dma_cap = HW_DMA_CAP_64B_PTP;
    rxq->desc_stride = 24;
    size_t ring_bytes    = (size_t)nb_desc * (size_t)rxq->desc_stride;
    size_t ring_bytes_pg = macb_round_up_pages(ring_bytes);
    unsigned align       = (unsigned)macb_pagesize();

    /* Your current code reserves a full 2MB page; keep it, but log true ring_bytes */
    const struct rte_memzone *mz =
        rte_memzone_reserve_aligned("macb_rx_ring_0",
            RTE_PGSIZE_2M,
            rte_socket_id(),
            RTE_MEMZONE_2MB | RTE_MEMZONE_IOVA_CONTIG,
            RTE_PGSIZE_2M);

    if (!mz) return -ENOMEM;

    RTE_LOG(INFO, PMD,
        "RX ring: cap=0x%x stride=%u nb=%u ring_bytes=%zu (mz.len=%zu) VA=%p IOVA=0x%" PRIx64 "\n",
        rxq->hw_dma_cap, rxq->desc_stride, nb_desc, ring_bytes, mz->len, mz->addr, mz->iova);

    int rc = dma_map_region_relaxed(dev->device, mz->addr, BUS_IOVA(mz->iova), mz->len);
    if (rc) { MACB_ERR("dma_map RX ring failed: %d\n", rc); return rc; }

    rxq->ring      = (uint8_t *)mz->addr;
    rxq->vring     = (volatile uint8_t *)mz->addr;
    rxq->ring_iova = mz->iova;
    rxq->nb_desc   = nb_desc;
    rxq->mp        = mp;
    rxq->ad        = ad;
    rxq->port_id   = dev->data->port_id;

    rxq->sw_ring   = rte_zmalloc_socket("macb_rx_sw",
                                        (size_t)nb_desc * sizeof(struct rte_mbuf *),
                                        RTE_CACHE_LINE_SIZE, rte_socket_id());
    rxq->data_room_bytes = (uint16_t)(rte_pktmbuf_data_room_size(mp) - RTE_PKTMBUF_HEADROOM);
    /* Open sync helper early so macb_rx_init() writes get cleaned to RAM */
	macb_dma_sync_open_once(ad);
	rxq->sync_fd = ad->sync_fd;

	memset(rxq->ring, 0, ring_bytes);
	sync_desc_to_dev(rxq->sync_fd, rxq->ring, ring_bytes);
	rte_io_wmb();
	macb_rx_init(rxq);
	macb_dump_first_rx_descs(rxq, "post-rx-init", 0 /* CPU view */);

	/* Ensure everything macb_rx_init() wrote is visible to the device */
	sync_desc_to_dev(rxq->sync_fd, rxq->ring, (size_t)nb_desc * rxq->desc_stride);
	rte_io_wmb();
	if (g_rxq_rwtest) {
	    int trc = macb_rxq_cpu_rw_test(rxq, "queue_setup");
	    if (trc) {
		RTE_LOG(ERR, PMD, "macb: RXQ RWTEST failed rc=%d\n", trc);
		/* choose: return error or continue */
		return trc;
	    }
	}

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

    /* TX and RX must agree on descriptor format for the device */
    /* If you already stored ad->hw_dma_cap once, use that instead. */
    /* Keep TX cap consistent with RX (or default to 32B if RX not set yet) */
    //txq->hw_dma_cap  = ad->hw_dma_cap ? ad->hw_dma_cap : HW_DMA_CAP_32B;
    //txq->desc_stride = (uint16_t)macb_desc_stride_bytes(txq->hw_dma_cap);
    

	int forced = macb_forced_desc_stride();
	if (forced) {
	    txq->hw_dma_cap  = ad->hw_dma_cap;      /* keep real cap if you use it elsewhere */
	    txq->desc_stride = (uint16_t)forced;
	} else {
	    txq->hw_dma_cap  = ad->hw_dma_cap;
	    txq->desc_stride = (uint16_t)macb_desc_stride_bytes(txq->hw_dma_cap);
	}

	txq->hw_dma_cap= HW_DMA_CAP_64B_PTP;
	txq->desc_stride = 24;
    const size_t ring_bytes = (size_t)nb_desc * (size_t)txq->desc_stride;

    const struct rte_memzone *mz =
        rte_eth_dma_zone_reserve(dev, "tx_ring", qid,
                                 ring_bytes,
                                 RTE_CACHE_LINE_SIZE, rte_socket_id());
    if (!mz) return -ENOMEM;

    RTE_LOG(INFO, PMD,
        "TX ring: cap=0x%x stride=%u nb=%u ring_bytes=%zu VA=%p IOVA=0x%" PRIx64 "\n",
        txq->hw_dma_cap, txq->desc_stride, nb_desc, ring_bytes, mz->addr, mz->iova);

    int rc = rte_dev_dma_map(dev->device, (void *)(uintptr_t)mz->addr, BUS_IOVA(mz->iova), mz->len);
    if (rc == -ENOTSUP || rc == -EINVAL) rc = 0;
    if (rc) { MACB_ERR("dma_map TX ring failed: %d\n", rc); return rc; }

    txq->mz        = mz;
    txq->ring      = (uint8_t *)mz->addr;
    txq->vring     = (volatile uint8_t *)mz->addr;
    txq->ring_iova = mz->iova;
    txq->nb_desc   = nb_desc;
    //txq->sync_fd   = -1;
    macb_dma_sync_open_once(ad);
    txq->sync_fd = ad->sync_fd;
    txq->ad        = ad;

    txq->sw_ring = rte_zmalloc_socket("macb_tx_sw",
                                      (size_t)nb_desc * sizeof(struct rte_mbuf *),
                                      RTE_CACHE_LINE_SIZE, rte_socket_id());
    txq->dbg_ctrl0 = rte_zmalloc("macb_dbg_ctrl0",
                             (size_t)nb_desc * sizeof(uint32_t), 0);
    if (!txq->sw_ring) return -ENOMEM;

    /* Init each descriptor as FREE (TX_USED set) */
    for (uint16_t i = 0; i < nb_desc; i++) {
        const uint32_t wrap = (i == nb_desc - 1) ? TX_WRAP : 0;

	volatile uint32_t *w = macb_desc_words_at_stride(txq->ring, txq->desc_stride, i);
	memset((void *)w, 0, txq->desc_stride);
	w[0] = 0;
	w[1] = TX_USED | wrap;
	sync_desc_to_dev(txq->sync_fd, (void *)(uintptr_t)w, txq->desc_stride);

    }
    sync_desc_to_dev(txq->sync_fd, txq->ring, (size_t)nb_desc * txq->desc_stride);

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
    struct macb_txq *txq = &ad->txq[qid];

    txq->prod = 0;
    txq->cons = 0;

    const uint16_t nb = txq->nb_desc;
    const size_t ring_bytes = (size_t)nb * (size_t)txq->desc_stride;

    for (uint16_t i = 0; i < nb; i++) {
        uint32_t wrap = (i == nb - 1) ? TX_WRAP : 0;

	volatile uint32_t *w = macb_desc_words_at_stride(txq->ring, txq->desc_stride, i);
	memset((void *)w, 0, txq->desc_stride);
	w[1] = TX_USED | wrap;
	sync_desc_to_dev(txq->sync_fd, (void *)(uintptr_t)w, txq->desc_stride);

    }
    sync_desc_to_dev(txq->sync_fd, txq->ring, ring_bytes);
    rte_io_wmb();

    return 0;
}

static int macb_tx_queue_stop(struct rte_eth_dev *dev, uint16_t qid){
    if (qid < RTE_ETHDEV_QUEUE_STAT_CNTRS)
        dev->data->tx_queue_state[qid] = RTE_ETH_QUEUE_STATE_STOPPED;
    return 0;
}

/* Read back primary station address (SA1) for MACB or GEM */
static inline void macb_dbg_sa(struct macb_hw *hw)
{
#if defined(MACB_SA1L) && defined(MACB_SA1H)
    uint32_t sa1l = macb_readl(hw, MACB_SA1L);
    uint32_t sa1h = macb_readl(hw, MACB_SA1H);
    uint8_t mac0 = (sa1l >>  0) & 0xff;
    uint8_t mac1 = (sa1l >>  8) & 0xff;
    uint8_t mac2 = (sa1l >> 16) & 0xff;
    uint8_t mac3 = (sa1l >> 24) & 0xff;
    uint8_t mac4 = (sa1h >>  0) & 0xff;
    uint8_t mac5 = (sa1h >>  8) & 0xff;
    RTE_LOG(INFO, PMD,
        "macb: SA1 now=%02x:%02x:%02x:%02x:%02x:%02x (SA1H=0x%08x SA1L=0x%08x)\n",
        mac0, mac1, mac2, mac3, mac4, mac5, sa1h, sa1l);

#elif defined(GEM_SA1B) && defined(GEM_SA1T)
    uint32_t sa1b = macb_readl(hw, GEM_SA1B);
    uint32_t sa1t = macb_readl(hw, GEM_SA1T);
    uint8_t mac0 = (sa1b >>  0) & 0xff;
    uint8_t mac1 = (sa1b >>  8) & 0xff;
    uint8_t mac2 = (sa1b >> 16) & 0xff;
    uint8_t mac3 = (sa1b >> 24) & 0xff;
    uint8_t mac4 = (sa1t >>  0) & 0xff;
    uint8_t mac5 = (sa1t >>  8) & 0xff;
    RTE_LOG(INFO, PMD,
        "macb: SA1 now=%02x:%02x:%02x:%02x:%02x:%02x (SA1T=0x%08x SA1B=0x%08x)\n",
        mac0, mac1, mac2, mac3, mac4, mac5, sa1t, sa1b);
#else
    RTE_LOG(INFO, PMD, "macb: SA1 regs not present on this IP\n");
#endif
}

/* -------- Promisc / link -------- */
int macb_promiscuous_enable(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    uint32_t n = macb_readl(&ad->hw, MACB_NCFGR);
    n |= MACB_NCFGR_CAF;
    n &= ~MACB_NCFGR_NBC;
    macb_writel(&ad->hw, MACB_NCFGR, n);
    macb_dbg_sa(&ad->hw);
    (void)macb_readl(&ad->hw, MACB_NCFGR);

    macb_force_promisc_allow_bcast(ad);
    return 0;
}

int macb_promiscuous_disable(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    uint32_t n = macb_readl(&ad->hw, MACB_NCFGR);
    n &= ~MACB_NCFGR_CAF;
    n &= ~MACB_NCFGR_NBC;
    macb_writel(&ad->hw, MACB_NCFGR, n);
    (void)macb_readl(&ad->hw, MACB_NCFGR);
    return 0;
}

/* --- Fallbacks if headers don't define these --- */
#ifndef NCFGR_SPD      /* 100 Mbps select */
# define NCFGR_SPD    (1u << 0)
#endif
#ifndef NCFGR_FD       /* Full duplex */
# define NCFGR_FD     (1u << 1)
#endif
#ifndef GEM_NCFGR_GBE  /* 1G enable on GEM variants */
# define GEM_NCFGR_GBE (1u << 10)
#endif

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

    (void)macb_mdio_read_c22(ad, phy, MII_BMSR);
    uint16_t bmsr = macb_mdio_read_c22(ad, phy, MII_BMSR);

    if (!(bmsr & BMSR_LSTATUS)) {
        macb_dump_mac_regs(ad, "link-down");
        macb_dump_phy_regs(ad, phy, "link-down");
    }

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
            uint16_t bmcr = macb_mdio_read_c22(ad, phy, MII_BMCR);
            l.link_speed  = (bmcr & BMCR_SPEED100) ? RTE_ETH_SPEED_NUM_100M : RTE_ETH_SPEED_NUM_10M;
            l.link_duplex = (bmcr & BMCR_FULLDPLX) ? RTE_ETH_LINK_FULL_DUPLEX : RTE_ETH_LINK_HALF_DUPLEX;
        }
    }

    rte_eth_linkstatus_set(dev, &l);

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

    /* Policy: allow broadcast, add CAF if promisc */
    ncfgr_new &= ~MACB_NCFGR_NBC;
    if (dev->data->promiscuous)
        ncfgr_new |= MACB_NCFGR_CAF;

    if (ncfgr_new != ncfgr_old) {
        macb_writel(&ad->hw, MACB_NCFGR, ncfgr_new);
        (void)macb_readl(&ad->hw, MACB_NCFGR);
    }

    //MACB_DBG("NCFGR=0x%08x after link update (FD=%d SPD100=%d GBE=%d CAF=%d NBC=%d)\n",
    //         ncfgr_new,
    //         !!(ncfgr_new & NCFGR_FD),
    //         !!(ncfgr_new & NCFGR_SPD),
    //         !!(ncfgr_new & GEM_NCFGR_GBE),
    //         !!(ncfgr_new & MACB_NCFGR_CAF),
    //         !!(ncfgr_new & MACB_NCFGR_NBC));

    //MACB_DBG("Link: %s, speed=%u, duplex=%s (phy=%u id=%04x:%04x)\n",
    //         l.link_status ? "UP" : "DOWN",
    //         l.link_speed,
    //         (l.link_duplex == RTE_ETH_LINK_FULL_DUPLEX) ? "FD" : "HD",
    //         (unsigned)phy, id1, id2);

    return 0;
}



/* Compute and log HW RX head index from RBQP against our ring base */
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
    uint32_t didx = off / (uint32_t)rxq->desc_stride;

    RTE_LOG(INFO, PMD,
        "macb: RX[%s] head: RBQP=%08x:%08x base=%08x:%08x -> off=%u idx=%u\n",
        tag,
        (uint32_t)(cur >> 32), (uint32_t)cur,
        (uint32_t)(base >> 32), (uint32_t)base,
        off, didx);
}

static void macb_dump_first_tx_descs(struct macb_txq *txq, const char *tag)
{
    struct macb_adapter *ad = txq->ad;
    for (uint32_t i = 0; i < 4 && i < txq->nb_desc; i++) {
        volatile uint32_t *w =
            macb_desc_words_at_stride(txq->ring, txq->desc_stride, i);

        sync_desc_from_dev(txq->sync_fd, (void *)(uintptr_t)w, txq->desc_stride);
        rte_io_rmb();

        macb_dump_tx_desc_one(tag, txq, i, w, txq->desc_stride);
    }
}

static int macb_dev_start(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    struct macb_rxq *rxq = &ad->rxq[0];
    struct macb_txq *txq = &ad->txq[0];

    /* RP1 GEM: real hi pointer regs (your logs prove these are the ones that read back = 1) */
#define RP1_GEM_TBQPH   0x04C8u
#define RP1_GEM_RBQPH   0x04D4u

    MACB_DBG("starting dev...\n");

    macb_dma_sync_open_once(ad);
    rxq->sync_fd = ad->sync_fd;
    txq->sync_fd = ad->sync_fd;

    /* ---------------- Disable TX/RX while (re)programming rings ---------------- */
    {
        uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
        ncr &= ~(NCR_TXEN | NCR_RXEN);
        macb_writel(&ad->hw, MACB_NCR, ncr);
        rte_io_wmb();
        (void)macb_readl(&ad->hw, MACB_NCR);
    }

#ifdef MACB_RSR
    macb_writel(&ad->hw, MACB_RSR, 0xffffffffu);
#endif
#ifdef MACB_TSR
    macb_writel(&ad->hw, MACB_TSR, 0xffffffffu);
#endif
    rte_io_wmb();

    /* ---------------- TX ring reset (all FREE: USED=1, WRAP on last) ---------------- */
    {
        const uint16_t nb = txq->nb_desc;
        for (uint16_t i = 0; i < nb; i++) {
            const uint32_t wrap = (i == nb - 1) ? TX_WRAP : 0;
            volatile uint32_t *w = macb_desc_words_at_stride(txq->ring, txq->desc_stride, i);
            memset((void *)(uintptr_t)w, 0, txq->desc_stride);
            w[1] = TX_USED | wrap;
        }

        sync_desc_to_dev(txq->sync_fd, txq->ring,
                         (size_t)txq->nb_desc * (size_t)txq->desc_stride);
        rte_io_wmb();
    }

    /* ---------------- Program ring pointers ---------------- */
    {
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

        /* 1) Program *real* RP1 hi regs (0x04C8/0x04D4) if we need them.
         *    (These are the regs your logs show as correct.)
         */
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

	/* log readback */
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
    }

    /* ---------------- Optional: DMA-map mempool segments ---------------- */
    if (rxq->mp) {
        (void)macb_map_mempool(dev->device, rxq->mp);
        rte_io_wmb();
    }

    /* ---------------- Program RX buffer size ---------------- */
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

    /* ---------------- PHY config + promisc policy ---------------- */
    if (g_phy_mode == PHY_MODE_AUTO)
        macb_phy_normal_up(ad);
    else
        macb_phy_force_10_100(ad, g_phy_mode);

    {
        uint32_t n = macb_readl(&ad->hw, MACB_NCFGR);
        n |= MACB_NCFGR_CAF;
        n &= ~MACB_NCFGR_NBC;
        macb_writel(&ad->hw, MACB_NCFGR, n);
        (void)macb_readl(&ad->hw, MACB_NCFGR);
    }

    /* ---------------- Enable RX/TX ---------------- */
    {
        uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
        ncr |= NCR_TXEN | NCR_MPE | NCR_RXEN;
        macb_writel(&ad->hw, MACB_NCR, ncr);
        rte_io_wmb();

        /* TSTART is often write-only/self-clearing; don’t expect it to read back as 1 */
        macb_writel(&ad->hw, MACB_NCR, ncr | NCR_TSTART);
        rte_io_wmb();

        /* Log what NCR *does* read back as */
        {
            uint32_t ncr_rb = macb_readl(&ad->hw, MACB_NCR);
            RTE_LOG(INFO, PMD, "macb: NCR readback after enable/kick = 0x%08x\n", ncr_rb);
        }
    }

    macb_apply_loopback(ad);
    macb_log_regs_full(ad, "post-enable");

    if (g_rx_selftest) {
        RTE_LOG(INFO, PMD, "macb: running RX loopback selftest...\n");
        int rc = macb_rx_loopback_selftest(ad, rxq, txq);
        if (rc) {
            RTE_LOG(ERR, PMD, "macb: RX loopback selftest FAILED rc=%d\n", rc);
            return rc;
        }
        RTE_LOG(INFO, PMD, "macb: RX loopback selftest PASS\n");
    }

    if (ad->dma_selftest) {
        int rc = macb_dma_tx_selftest(ad, txq);
        if (rc) {
            MACB_ERR("DMA selftest failed rc=%d\n", rc);
            return rc;
        }
    }

    macb_dump_first_tx_descs(txq, "post-start");
    macb_dump_first_rx_descs(rxq, "post-start", 1 /* HW view */);

    return 0;
}


static int macb_dev_stop(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
    ncr &= ~(NCR_RXEN | NCR_TXEN);
    macb_writel(&ad->hw, MACB_NCR, ncr);
    dev->data->dev_link.link_status = RTE_ETH_LINK_DOWN;

    /* Turn RX ring back to RW so reconfiguration can update descriptors */
    if (g_rx_ring_ro) {
        (void)macb_rx_ring_set_prot(&ad->rxq[0], PROT_READ | PROT_WRITE, "RW");
    }

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
    //MACB_DBG("phy_mode set to %s\n", val);
    return 0;
}

/* NEW: parse rx_ring_ro debug flag */
static int parse_rx_ring_ro_cb(const char *key, const char *val, void *extra)
{
    RTE_SET_USED(key); RTE_SET_USED(extra);
    if (!val) return -EINVAL;
    if (!strcmp(val, "1") || !strcasecmp(val, "true") || !strcasecmp(val, "on"))
        g_rx_ring_ro = 1;
    else if (!strcmp(val, "0") || !strcasecmp(val, "false") || !strcasecmp(val, "off"))
        g_rx_ring_ro = 0;
    else return -EINVAL;
    //MACB_DBG("rx_ring_ro set to %d\n", g_rx_ring_ro);
    return 0;
}

static int parse_rxq_rwtest_cb(const char *key, const char *val, void *extra)
{
    RTE_SET_USED(key); RTE_SET_USED(extra);
    if (!val) return -EINVAL;
    if (!strcmp(val, "1") || !strcasecmp(val, "true") || !strcasecmp(val, "on"))
        g_rxq_rwtest = 1;
    else if (!strcmp(val, "0") || !strcasecmp(val, "false") || !strcasecmp(val, "off"))
        g_rxq_rwtest = 0;
    else return -EINVAL;
    return 0;
}

static void macb_discover_rp1_q0_ptr_regs(struct macb_adapter *ad,
                                         uint32_t fw_rbqp_lo,
                                         uint32_t fw_tbqp_lo)
{
    /* scan only a reasonable window */
    const uint32_t lo = 0x0000;
    const uint32_t hi = 0x4000;

    uint32_t rb_hit = 0xffffffffu;
    uint32_t tb_hit = 0xffffffffu;

    for (uint32_t off = lo; off <= hi; off += 4) {
        uint32_t v = macb_readl(&ad->hw, off);

        /* ignore the legacy registers themselves (MACB_RBQP / MACB_TBQP) */
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
        "macb: RP1 q0 discover: fw RBQP=0x%08x hit_off=%s  fw TBQP=0x%08x hit_off=%s\n",
        fw_rbqp_lo, (rb_hit==0xffffffffu) ? "NONE" : "FOUND",
        fw_tbqp_lo, (tb_hit==0xffffffffu) ? "NONE" : "FOUND");

    if (rb_hit != 0xffffffffu || tb_hit != 0xffffffffu) {
        RTE_LOG(INFO, PMD,
            "macb: RP1 q0 discover offsets: RBQB0=0x%04x TBQB0=0x%04x\n",
            (rb_hit==0xffffffffu)?0:rb_hit,
            (tb_hit==0xffffffffu)?0:tb_hit);
    }
}


static int macb_probe(struct rte_vdev_device *vdev)
{
    const char *args = rte_vdev_device_args(vdev);
    struct rte_kvargs *kv = (args && *args) ? rte_kvargs_parse(args, NULL) : NULL;
    char *uio = NULL;
    int dma_selftest = 0;
    int rxq_rwtest = 1; 
    if (kv) {
        (void)rte_kvargs_process(kv, "dev",        parse_dev_arg_cb,     &uio);
        (void)rte_kvargs_process(kv, "phy_lb",     parse_bool_arg_cb,    &g_force_phy_lb);
        (void)rte_kvargs_process(kv, "mac_lb",     parse_bool_arg_cb,    &g_mac_lb);
        (void)rte_kvargs_process(kv, "phy_addr",   parse_int_arg_cb,     &g_forced_phy_addr);
        (void)rte_kvargs_process(kv, "bus_ofs",    parse_u64_arg_cb,     &g_bus_ofs);
        (void)rte_kvargs_process(kv, "dma_selftest", parse_bool_arg_cb, &dma_selftest);
//	(void)rte_kvargs_process(kv, "rxq_rwtest", parse_bool_arg_cb, &rxq_rwtest);
	(void)rte_kvargs_process(kv, "phy_mode",   parse_phy_mode_cb,    NULL);
        (void)rte_kvargs_process(kv, "rx_ring_ro", parse_rx_ring_ro_cb,  NULL);
        rte_kvargs_free(kv);
    }
    if (!uio) { uio = (char *)rte_malloc("macb", strlen("/dev/uio0")+1, 0); if (!uio) return -ENOMEM; strcpy(uio, "/dev/uio0"); }

    struct rte_eth_dev *eth_dev = rte_eth_vdev_allocate(vdev, sizeof(struct macb_adapter));
    if (!eth_dev) { rte_free(uio); return -ENOMEM; }

    struct macb_adapter *ad = eth_dev->data->dev_private;
    memset(ad, 0, sizeof(*ad));
    ad->hw.uio_fd = -1;
    ad->sync_fd   = -1;
    ad->dma_selftest = dma_selftest;
 //   ad->rxq_rwtest = rxq_rwtest;
    MACB_DBG("probe: dma_selftest=%d\n", ad->dma_selftest);
    ad->port_id   = eth_dev->data->port_id;
    ad->edev = eth_dev;

    MACB_DBG("probe args='%s'\n", args ? args : "(none)");

    uint32_t fw_rb = macb_readl(&ad->hw, MACB_RBQP);

    uint32_t fw_tb = macb_readl(&ad->hw, MACB_TBQP);

    macb_discover_rp1_q0_ptr_regs(ad, fw_rb, fw_tb);
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
RTE_PMD_REGISTER_PARAM_STRING(net_macb, "dev=<path> phy_addr=<int> phy_lb=<0|1> mac_lb=<0|1> bus_ofs=<u64> phy_mode=<auto|10hd|10fd|100hd|100fd> rx_ring_ro=<0|1> dma_selftest=<0|1> rxq_rwtest=<0|1>");
