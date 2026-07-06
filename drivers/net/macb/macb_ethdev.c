// macb_ethdev.c — DPDK vdev PMD for RP1/Cadence GEM (Raspberry Pi 5)

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
#include "macb_dma_sync.h"
#include "macb_phy.h"        /* PHY/MDIO functions */
#include "macb_dma.h"        /* DMA sync functions */
#include "macb_ring.h"       /* Ring allocation/init functions */
#include "macb_ctrl.h"       /* Hardware control/configuration functions */

#define MACB_DBG(fmt, ...) RTE_LOG(INFO,  PMD, "macb: " fmt, ##__VA_ARGS__)
#define MACB_ERR(fmt, ...) RTE_LOG(ERR,   PMD, "macb: " fmt, ##__VA_ARGS__)

/* -------- Runtime knobs / KV args -------- */
uint64_t g_bus_ofs = 0;

static int      g_force_phy_lb    = 0;    /* default: external link (no PHY LB) */
static int      g_mac_lb          = 0;    /* default: MAC LB off */
static int      g_forced_phy_addr = -1;   /* -1 = auto-scan; else 0..31 */

static enum macb_phy_mode g_phy_mode = PHY_MODE_AUTO;




#define sync_desc_to_dev(fd, ptr, len)    macb_sync_to_dev_fd((fd), (ptr), (len))
#define sync_desc_from_dev(fd, ptr, len)  macb_sync_from_dev_fd((fd), (ptr), (len))
#define sync_buf_to_dev(fd, ptr, len)     macb_sync_to_dev_fd((fd), (ptr), (len))
#define sync_buf_from_dev(fd, ptr, len)   macb_sync_from_dev_fd((fd), (ptr), (len))

/* DMA sync functions moved to macb_dma.c */

/* MDIO and PHY functions moved to macb_phy.c */

/* -------- Ethdev ops decl -------- */
static int macb_dev_configure(struct rte_eth_dev *dev);
static int macb_dev_start(struct rte_eth_dev *dev);
static int macb_dev_stop(struct rte_eth_dev *dev);



static int macb_rx_queue_setup(struct rte_eth_dev *dev, uint16_t qid,
                               uint16_t nb_desc, unsigned int so,
                               const struct rte_eth_rxconf *rx_conf,
                               struct rte_mempool *mp)
{
    RTE_SET_USED(so); RTE_SET_USED(rx_conf);
    struct macb_adapter *ad = dev->data->dev_private;
    struct macb_rxq *rxq    = &ad->rxq[qid];

    /* Allocate ring DMA buffer and software structures */
    int rc = macb_ring_alloc_rx(dev, qid, nb_desc, mp);
    if (rc) return rc;

    /* Initialize ring descriptors */
    rc = macb_ring_init_rx(rxq);
    if (rc) return rc;

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

    /* Allocate ring DMA buffer and software structures */
    int rc = macb_ring_alloc_tx(dev, qid, nb_desc);
    if (rc) return rc;

    /* Initialize ring descriptors */
    rc = macb_ring_init_tx(txq);
    if (rc) return rc;

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

	volatile uint32_t *w = (volatile uint32_t *)macb_desc_at(txq->ring, txq->hw_dma_cap, i);
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

/* -------- Promisc / link -------- */
int macb_promiscuous_enable(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    uint32_t n = macb_readl(&ad->hw, MACB_NCFGR);
    n |= MACB_NCFGR_CAF;
    n &= ~MACB_NCFGR_NBC;
    macb_writel(&ad->hw, MACB_NCFGR, n);
    (void)macb_readl(&ad->hw, MACB_NCFGR);

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

    if (!(bmsr & BMSR_LSTATUS))
        RTE_LOG(INFO, PMD, "macb: link down\n");

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




static int macb_dev_start(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    struct macb_rxq *rxq = &ad->rxq[0];
    struct macb_txq *txq = &ad->txq[0];

    MACB_DBG("starting dev...\n");

    /* Open DMA sync helper device */
    macb_dma_sync_open_once(ad);
    rxq->sync_fd = ad->sync_fd;
    txq->sync_fd = ad->sync_fd;

    /* Hardware initialization sequence */
    macb_hw_disable_rxtx(ad);
    macb_hw_reset_status_regs(ad);
    macb_hw_reset_tx_ring(txq);
    macb_hw_program_ring_ptrs(ad, rxq, txq);
    macb_hw_configure_dma(ad, rxq);
    macb_hw_configure_mac(ad);

    /* PHY initialization */
    if (g_phy_mode == PHY_MODE_AUTO)
        macb_phy_normal_up(ad);
    else
        macb_phy_force_10_100(ad, g_phy_mode);

    /* Enable DMA */
    macb_hw_enable_rxtx(ad);

    return 0;
}


static int macb_dev_stop(struct rte_eth_dev *dev)
{
    struct macb_adapter *ad = dev->data->dev_private;
    
    macb_hw_disable_rxtx(ad);
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
    //MACB_DBG("phy_mode set to %s\n", val);
    return 0;
}

static int macb_parse_devargs(const char *args, char **uio_out)
{
    struct rte_kvargs *kv = (args && *args) ? rte_kvargs_parse(args, NULL) : NULL;
    char *uio = NULL;

    if (kv) {
        (void)rte_kvargs_process(kv, "dev",      parse_dev_arg_cb,  &uio);
        (void)rte_kvargs_process(kv, "phy_lb",   parse_bool_arg_cb, &g_force_phy_lb);
        (void)rte_kvargs_process(kv, "mac_lb",   parse_bool_arg_cb, &g_mac_lb);
        (void)rte_kvargs_process(kv, "phy_addr", parse_int_arg_cb,  &g_forced_phy_addr);
        (void)rte_kvargs_process(kv, "bus_ofs",  parse_u64_arg_cb,  &g_bus_ofs);
        (void)rte_kvargs_process(kv, "phy_mode", parse_phy_mode_cb, NULL);
        rte_kvargs_free(kv);
    }

    if (!uio) {
        uio = (char *)rte_malloc("macb", strlen("/dev/uio0") + 1, 0);
        if (!uio)
            return -ENOMEM;
        strcpy(uio, "/dev/uio0");
    }

    *uio_out = uio;
    return 0;
}

static int macb_probe(struct rte_vdev_device *vdev)
{
    const char *args = rte_vdev_device_args(vdev);
    char *uio = NULL;
    int rc = macb_parse_devargs(args, &uio);
    if (rc)
        return rc;

    struct rte_eth_dev *eth_dev = rte_eth_vdev_allocate(vdev, sizeof(struct macb_adapter));
    if (!eth_dev) { rte_free(uio); return -ENOMEM; }

    struct macb_adapter *ad = eth_dev->data->dev_private;
    memset(ad, 0, sizeof(*ad));
    ad->hw.uio_fd = -1;
    ad->sync_fd   = -1;
    ad->hw_dma_cap = macb_hw_probe_dma_cap(ad);
    ad->port_id   = eth_dev->data->port_id;
    ad->edev = eth_dev;

    rc = macb_uio_map(&ad->hw, uio);
    rte_free(uio);
    if (rc) {
        rte_eth_dev_release_port(eth_dev);
        return rc;
    }

    uint32_t fw_rb = macb_readl(&ad->hw, MACB_RBQP);
    uint32_t fw_tb = macb_readl(&ad->hw, MACB_TBQP);

	macb_hw_discover_rp1_q0_ptr_regs(ad, fw_rb, fw_tb);
	
    MACB_DBG("probe args='%s'\n", args ? args : "(none)");

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
