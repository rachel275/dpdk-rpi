// SPDX-License-Identifier: BSD-3-Clause
// macb_ethdev_rp1.c — DPDK vdev PMD for RP1/Cadence GEM (Raspberry Pi 5)
//
// This file replaces your previous macb_ethdev.c with fixes for:
//  - 64-bit ring base registers (RBQP/TBQP high dwords)
//  - Proper TX kick (NCR_TSTART) from idle
//  - Consistent RX/TX descriptor ownership handling (USED bit31)
//  - Force 100/FD for bring-up to match PHY-loopback default
//  - Cleaned up loopback control (PHY vs MAC)
//
// Build: compile within your DPDK tree as a vdev PMD, e.g. net/macb.

#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>

#include <rte_string_fns.h>
#include <rte_malloc.h>
#include <rte_log.h>
#include <rte_kvargs.h>
#include <rte_bus_vdev.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <ethdev_driver.h>
#include <ethdev_vdev.h>
#include <rte_memzone.h>
#include <rte_interrupts.h>
#include <rte_cycles.h>
#include <rte_mbuf.h>
#include <rte_io.h>

/* Fallback for builds where RTE_LOGTYPE_PMD is not defined */
#ifndef RTE_LOGTYPE_PMD
#define RTE_LOGTYPE_PMD RTE_LOGTYPE_USER1
#endif


#include "dma_sync_user.h"
#include "macb_hw.h"

#define MACB_DBG(fmt, ...) RTE_LOG(INFO, PMD, "macb: " fmt, ##__VA_ARGS__)
#define MACB_ERR(fmt, ...) RTE_LOG(ERR,  PMD, "macb: " fmt, ##__VA_ARGS__)

/* ---------------- Register compatibility shims ---------------- */
#ifndef MACB_NCR
# define MACB_NCR    0x0000u
#endif
#ifndef MACB_NCFGR
# define MACB_NCFGR  0x0004u
#endif
#ifndef MACB_NSR
# define MACB_NSR    0x0008u
#endif
#ifndef MACB_TSR
# define MACB_TSR    0x0014u
#endif
#ifndef MACB_RBQP
# define MACB_RBQP   0x0018u
#endif
#ifndef MACB_TBQP
# define MACB_TBQP   0x001Cu
#endif
#ifndef MACB_RSR
# define MACB_RSR    0x0020u
#endif
#ifndef MACB_MAN
# define MACB_MAN    0x0034u
#endif
/* RP1/GEM_GXL: high 32-bit ring base registers (adjust if your SoC differs) */
#ifndef MACB_RBQPH
# define MACB_RBQPH  0x00A4u
#endif
#ifndef MACB_TBQPH
# define MACB_TBQPH  0x00A8u
#endif

/* --------- Bitfields (guarded so macb_hw.h can override) --------- */
#ifndef NCR_RXEN
# define NCR_RXEN    (1u << 2)
#endif
#ifndef NCR_TXEN
# define NCR_TXEN    (1u << 3)
#endif
#ifndef NCR_MPE
# define NCR_MPE     (1u << 4)
#endif
#ifndef NCR_TSTART
# define NCR_TSTART  (1u << 9)
#endif
#ifndef NCR_LB
# define NCR_LB      (1u << 1)
#endif

#ifndef NCFGR_CAF
# define NCFGR_CAF   (1u << 4)
#endif
#ifndef NCFGR_DRFCS
# define NCFGR_DRFCS (1u << 11)
#endif
#ifndef NCFGR_LLB
# define NCFGR_LLB   (1u << 1)
#endif
#ifndef NCFGR_SPD
# define NCFGR_SPD   (1u << 0)  /* 100M for classic 10/100 MACB; keep as bring-up */
#endif
#ifndef NCFGR_FD
# define NCFGR_FD    (1u << 1)
#endif

#ifndef NSR_IDLE
# define NSR_IDLE    (1u << 2)
#endif

/* Descriptor flags — classic GEM: bit31 USED=1:CPU owns; 0:HW owns */
#ifndef RX_USED
# define RX_USED     0x80000000u   /* GEM: bit31 USED=1 → CPU owns; 0 → HW owns */
#endif
#ifndef RX_WRAP
# define RX_WRAP     0x40000000u   /* GEM: bit30 WRAP in addr word */
#endif
#ifndef TX_USED
# define TX_USED     0x80000000u
#endif
#ifndef TX_WRAP
# define TX_WRAP     0x40000000u
#endif

/* ---------------- External symbols from your other units -------- */
extern void     macb_rx_init(struct macb_rxq *rxq);
extern uint16_t macb_rx_burst(void *queue, struct rte_mbuf **rx_pkts, uint16_t nb_pkts);
extern uint16_t macb_tx_burst(void *queue, struct rte_mbuf **tx_pkts, uint16_t nb_pkts);
extern int      macb_uio_map(struct macb_hw *hw, const char *uio_path);
extern void     macb_uio_unmap(struct macb_hw *hw);

/* ---------------- MDIO helpers (Clause 22) ---------------------- */
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
# define MII_BMCR   0x00
#endif
#ifndef MII_BMSR
# define MII_BMSR   0x01
#endif
#ifndef MII_PHYSID1
# define MII_PHYSID1 0x02
#endif
#ifndef MII_PHYSID2
# define MII_PHYSID2 0x03
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

/* ---------------- Runtime knobs --------------------------------- */
uint64_t g_bus_ofs      = 0;     /* CPU-phys -> device-DMA bus offset (RP1: keep 0) */
int      g_force_phy_lb = 1;     /* default = PHY loopback on */

#define BUS_IOVA(x) ((rte_iova_t)((rte_iova_t)(x) + (rte_iova_t)g_bus_ofs))

/* ---------------- DMA sync wrappers ------------------------------ */
static inline void macb_sync_desc_to_dev(int fd, const volatile void *desc, size_t len)
{ if (fd >= 0) dma_sync_to_dev(fd, (void *)(uintptr_t)desc, len); }
static inline void macb_sync_desc_from_dev(int fd, const volatile void *desc, size_t len)
{ if (fd >= 0) dma_sync_from_dev(fd, (void *)(uintptr_t)desc, len); }

/* ---------------- MDIO helpers ---------------------------------- */
static inline void macb_wait_mdio_idle(struct macb_adapter *ad)
{
    for (int i = 0; i < 1000; i++) {
        if (macb_readl(&ad->hw, MACB_NSR) & NSR_IDLE)
            return;
        rte_delay_us_block(5);
    }
    MACB_ERR("MDIO idle wait timeout");
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
static int macb_find_phy_addr(struct macb_adapter *ad, uint8_t *out_phy)
{
    for (uint8_t phy = 0; phy < 32; phy++) {
        uint16_t id1 = macb_mdio_read_c22(ad, phy, MII_PHYSID1);
        uint16_t id2 = macb_mdio_read_c22(ad, phy, MII_PHYSID2);
        (void)id2; /* silence unused if not checked */
        if (id1 != 0xffff && id1 != 0x0000) {
            if (out_phy) *out_phy = phy;
            return 0;
        }
    }
    return -1;
}
static int macb_phy_loopback_set(struct macb_adapter *ad, int enable)
{
    uint8_t phy;
    if (macb_find_phy_addr(ad, &phy) != 0)
        return -1;
    uint16_t bmcr = macb_mdio_read_c22(ad, phy, MII_BMCR);
    uint16_t newbmcr = (bmcr & ~BMCR_ANENABLE) | BMCR_SPEED100 | BMCR_FULLDPLX;
    if (enable) newbmcr |=  BMCR_LOOPBACK; else newbmcr &= ~BMCR_LOOPBACK;
    if (newbmcr != bmcr) macb_mdio_write_c22(ad, phy, MII_BMCR, newbmcr);
    MACB_DBG("PHY loopback %s (BMCR 0x%04x -> 0x%04x, phy=%u)\n",
             enable?"EN":"DIS", bmcr, newbmcr, (unsigned)phy);
    return 0;
}

/* ---------------- Loopback helpers ------------------------------- */
static void macb_mac_loopback_set(struct macb_adapter *ad, int enable)
{
    uint32_t ncfgr = macb_readl(&ad->hw, MACB_NCFGR);
    uint32_t ncr   = macb_readl(&ad->hw, MACB_NCR);
    if (enable) {
        ncfgr |= (NCFGR_CAF | NCFGR_DRFCS | NCFGR_LLB);
        ncr   |= NCR_LB;
    } else {
        ncfgr &= ~NCFGR_LLB;
        ncr   &= ~NCR_LB;
    }
    macb_writel(&ad->hw, MACB_NCFGR, ncfgr);
    macb_writel(&ad->hw, MACB_NCR,   (ncr | NCR_RXEN | NCR_TXEN));
}
static void macb_apply_loopback(struct macb_adapter *ad)
{
    if (g_force_phy_lb) { macb_mac_loopback_set(ad, 0); (void)macb_phy_loopback_set(ad, 1); }
    else                { (void)macb_phy_loopback_set(ad, 0); macb_mac_loopback_set(ad, 1); }
}
static void macb_force_mac_100fd(struct macb_adapter *ad)
{
    uint32_t ncf0 = macb_readl(&ad->hw, MACB_NCFGR);
    uint32_t ncf1 = ncf0 | NCFGR_SPD | NCFGR_FD;
    if (ncf1 != ncf0) macb_writel(&ad->hw, MACB_NCFGR, ncf1);
}

/* ---------------- Ethdev ops declarations ------------------------ */
static int macb_dev_configure(struct rte_eth_dev *dev);
static int macb_dev_start(struct rte_eth_dev *dev);
static int macb_dev_stop(struct rte_eth_dev *dev);
static int macb_dev_close(struct rte_eth_dev *dev);
static int macb_promisc_enable(struct rte_eth_dev *dev);
static int macb_promisc_disable(struct rte_eth_dev *dev);
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

static const struct eth_dev_ops macb_ops = {
    .dev_configure      = macb_dev_configure,
    .dev_start          = macb_dev_start,
    .dev_stop           = macb_dev_stop,
    .dev_close          = macb_dev_close,
    .promiscuous_enable = macb_promisc_enable,
    .promiscuous_disable= macb_promisc_disable,
    .dev_infos_get      = macb_info_get,
    .rx_queue_setup     = macb_rx_queue_setup,
    .tx_queue_setup     = macb_tx_queue_setup,
    .rx_queue_start     = macb_rx_queue_start,
    .rx_queue_stop      = macb_rx_queue_stop,
    .tx_queue_start     = macb_tx_queue_start,
    .tx_queue_stop      = macb_tx_queue_stop,
    .link_update        = macb_link_update,
};

/* ---------------- dev_configure/info ----------------------------- */
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

/* ---------------- queue setup ----------------------------------- */
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
    int rc = dma_map_region_relaxed(dev->device, mz->addr, mz->iova, mz->len);
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
    RTE_SET_USED(so); RTE_SET_USED(tx_conf);
    struct macb_adapter *ad = dev->data->dev_private;
    struct macb_txq *txq    = &ad->txq[qid];
    const struct rte_memzone *mz = rte_eth_dma_zone_reserve(dev, "tx_ring", qid,
                                 nb_desc * sizeof(struct macb_desc),
                                 RTE_CACHE_LINE_SIZE, rte_socket_id());
    if (!mz) return -ENOMEM;
    int rc = dma_map_region_relaxed(dev->device, mz->addr, mz->iova, mz->len);
    if (rc) { MACB_ERR("dma_map TX ring failed: %d\n", rc); return rc; }
    txq->ring      = (struct macb_desc *)mz->addr;
    txq->ring_iova = mz->iova;
    txq->nb_desc   = nb_desc;
    txq->sync_fd   = -1;
    txq->ad        = ad;
    txq->sw_ring   = rte_zmalloc_socket("macb_tx_sw", nb_desc * sizeof(struct rte_mbuf *),
                                        RTE_CACHE_LINE_SIZE, rte_socket_id());
    if (!txq->sw_ring) return -ENOMEM;
    for (uint16_t i = 0; i < nb_desc; i++) {
        txq->ring[i].addr = TX_USED;                /* CPU owns/idle */
        txq->ring[i].ctrl = (i == nb_desc - 1) ? TX_WRAP : 0;
        txq->sw_ring[i]   = NULL;
    }
    rte_io_wmb();
    txq->prod = 0; txq->cons = 0;
    dev->data->tx_queues[qid] = txq;
    return 0;
}

/* ---------------- Queue state ----------------------------------- */
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
    return 0;
}
static int macb_tx_queue_stop(struct rte_eth_dev *dev, uint16_t qid){
    if (qid < RTE_ETHDEV_QUEUE_STAT_CNTRS)
        dev->data->tx_queue_state[qid] = RTE_ETH_QUEUE_STATE_STOPPED;
    return 0;
}

/* ---------------- Promisc / link stubs -------------------------- */
static int macb_promisc_enable(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private; uint32_t n = macb_readl(&ad->hw, MACB_NCFGR);
    n |= NCFGR_CAF; macb_writel(&ad->hw, MACB_NCFGR, n); return 0;
}
static int macb_promisc_disable(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private; uint32_t n = macb_readl(&ad->hw, MACB_NCFGR);
    n &= ~NCFGR_CAF; macb_writel(&ad->hw, MACB_NCFGR, n); return 0;
}
static int macb_link_update(struct rte_eth_dev *dev, int wait)
{ RTE_SET_USED(dev); RTE_SET_USED(wait); return 0; }

/* ---------------- RX ring ownership ------------------------------ */
static void macb_rx_force_hw_own(struct macb_rxq *rxq)
{
    if (!rxq || !rxq->ring || !rxq->nb_desc) return;
    const uint16_t nb = rxq->nb_desc;
    for (uint16_t i = 0; i < nb; i++) {
        struct macb_desc *d = &rxq->ring[i];
        /* Clear USED (bit31) to hand to HW; set WRAP on last only. */
        uint32_t a = d->addr;
        a &= ~RX_USED;                         /* 0 → HW owns */
        if (i == nb - 1) a = (a & ~RX_WRAP) | RX_WRAP; else a &= ~RX_WRAP;
        d->addr = a; d->ctrl = 0;
        macb_sync_desc_to_dev(rxq->sync_fd, d, sizeof(*d));
    }
    if (rxq->sync_fd >= 0)
        dma_sync_to_dev(rxq->sync_fd, rxq->ring, nb * sizeof(struct macb_desc));
}



/* ---------------- debug helpers ----------------------------------- */
static void macb_dump_first_rx_descs(struct macb_rxq *rxq, const char *tag)
{
    if (!rxq || !rxq->ring) return;
    for (int i = 0; i < 2 && i < rxq->nb_desc; ++i) {
        struct macb_desc *d = &rxq->ring[i];
        uint32_t a = d->addr;
        uint32_t c = d->ctrl;
        int used = (a & RX_USED) ? 1 : 0;
        int wrap = (a & RX_WRAP) ? 1 : 0;
        MACB_DBG("RX[%s] d%d: addr=0x%08x ctrl=0x%08x (USED=%d %s, WRAP=%d)\n",
                 tag, i, a, c, used, used?"CPU":"HW", wrap);
    }
}

/* ---------------- dev_start/stop/close --------------------------- */
static int macb_dev_start(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    struct macb_rxq *rxq = &ad->rxq[0];
    struct macb_txq *txq = &ad->txq[0];

    MACB_DBG("starting dev...\n");

    macb_rx_force_hw_own(rxq);

    macb_writel(&ad->hw, MACB_RSR, 0xffffffffu);
    macb_writel(&ad->hw, MACB_TSR, 0xffffffffu);

    uint64_t rb = BUS_IOVA(rxq->ring_iova);
    uint64_t tb = BUS_IOVA(txq->ring_iova);
    macb_writel(&ad->hw, MACB_RBQP,  (uint32_t)(rb & 0xffffffffu));
    macb_writel(&ad->hw, MACB_TBQP,  (uint32_t)(tb & 0xffffffffu));
    macb_writel(&ad->hw, MACB_RBQPH, (uint32_t)(rb >> 32));
    macb_writel(&ad->hw, MACB_TBQPH, (uint32_t)(tb >> 32));

    rte_wmb(); /* ensure HW sees rings/pointers */

    /* Debug: show first two RX descs before enabling MAC */
    macb_dump_first_rx_descs(rxq, "pre-enable");

    macb_force_mac_100fd(ad);      /* bring-up: force 100/FD */

    uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
    ncr |= (NCR_RXEN | NCR_TXEN | NCR_MPE);
    macb_writel(&ad->hw, MACB_NCR, ncr);

    macb_apply_loopback(ad);       /* PHY or MAC loopback */

    /* Prime TX in case HW needs an edge to leave idle */
    uint32_t ncr2 = macb_readl(&ad->hw, MACB_NCR);
    macb_writel(&ad->hw, MACB_NCR, ncr2 | NCR_TSTART);

    /* Debug: show first two RX descs after initial TX kick */
    macb_dump_first_rx_descs(rxq, "post-kick");

    MACB_DBG("dev_start complete (RBQP=%08x:%08x TBQP=%08x:%08x)\n",
             macb_readl(&ad->hw, MACB_RBQPH), macb_readl(&ad->hw, MACB_RBQP),
             macb_readl(&ad->hw, MACB_TBQPH), macb_readl(&ad->hw, MACB_TBQP));
    return 0;
}
static int macb_dev_stop(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    uint32_t ncr = macb_readl(&ad->hw, MACB_NCR);
    ncr &= ~(NCR_RXEN | NCR_TXEN);
    macb_writel(&ad->hw, MACB_NCR, ncr);
    dev->data->dev_link.link_status = RTE_ETH_LINK_DOWN;
    if (ad->rxq[0].sync_fd >= 0) { close(ad->rxq[0].sync_fd); ad->rxq[0].sync_fd = -1; }
    ad->txq[0].sync_fd = -1;
    return 0;
}
static int macb_dev_close(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    macb_dev_stop(dev);
    macb_uio_unmap(&ad->hw);
    return 0;
}

/* ---------------- vdev probe/remove ------------------------------ */
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
static int parse_u64_arg_cb(const char *key, const char *val, void *extra)
{
    RTE_SET_USED(key); if (!val) return -EINVAL; char *end = NULL;
    uint64_t v = strtoull(val, &end, 0); if (end == val || *end) return -EINVAL; *(uint64_t *)extra = v; return 0;
}

static int macb_probe(struct rte_vdev_device *vdev)
{
    const char *args = rte_vdev_device_args(vdev);
    struct rte_kvargs *kv = (args && *args) ? rte_kvargs_parse(args, NULL) : NULL;
    char *uio = NULL;
    if (kv) {
        (void)rte_kvargs_process(kv, "dev",     parse_dev_arg_cb,  &uio);
        (void)rte_kvargs_process(kv, "phy_lb",  parse_bool_arg_cb, &g_force_phy_lb);
        (void)rte_kvargs_process(kv, "bus_ofs", parse_u64_arg_cb,  &g_bus_ofs);
        rte_kvargs_free(kv);
    }
    if (!uio) { uio = (char *)rte_malloc("macb", strlen("/dev/uio0")+1, 0); if (!uio) return -ENOMEM; strcpy(uio, "/dev/uio0"); }

    struct rte_eth_dev *eth_dev = rte_eth_vdev_allocate(vdev, sizeof(struct macb_adapter));
    if (!eth_dev) { rte_free(uio); return -ENOMEM; }

    struct macb_adapter *ad = eth_dev->data->dev_private;
    memset(ad, 0, sizeof(*ad)); ad->hw.uio_fd = -1; ad->port_id = eth_dev->data->port_id;

    MACB_DBG("probe args='%s'\n", args ? args : "(none)");
    int rc = macb_uio_map(&ad->hw, uio); rte_free(uio); if (rc) { rte_eth_dev_release_port(eth_dev); return rc; }

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
    eth_dev->data->nb_rx_queues = 1; eth_dev->data->nb_tx_queues = 1;
    rte_eth_dev_probing_finish(eth_dev);
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
RTE_PMD_REGISTER_PARAM_STRING(net_macb, "dev=<path> phy_lb=<0|1> bus_ofs=<u64>");

