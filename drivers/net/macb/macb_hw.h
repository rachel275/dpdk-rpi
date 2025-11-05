#pragma once

#include <stdint.h>
#include <stddef.h>
#include <rte_common.h>
#include <rte_memory.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_ethdev.h>
#include <rte_io.h>

/* Tunables */
#define MACB_NRXD   256
#define MACB_NTXD   256
#define MACB_ALIGN  64

/* ===== Registers (from kernel headers) ===== */
#define MACB_NCR        0x0000u   /* Network Control */
#define MACB_NCFGR      0x0004u   /* Network Config  */
#define MACB_TSR        0x0014u   /* Transmit Status */


/* Classic MACB single-queue base pointers */
#define MACB_RBQP      0x0018u  /* Receive Buffer Queue Pointer base */
#define MACB_TBQP      0x001Cu  /* Transmit Buffer Queue Pointer base */

/* GEM queue-0 base registers (present on most MACB/GEM variants) */
#ifndef GEM_RBQB_Q0
#define GEM_RBQB_Q0 0x0480u
#endif
#ifndef GEM_TBQB_Q0
#define GEM_TBQB_Q0 0x0484u
#endif

/* also provide the indexed form used by macb_ethdev.c */
#define GEM_TBQP(q)     (GEM_TBQB_Q0 + ((q) << 2))
#define GEM_RBQP(q)     (GEM_RBQB_Q0 + ((q) << 2))

/* ===== NCR/NCFGR bits ===== */
#define NCR_RXEN        (1u << 2)
#define NCR_TXEN        (1u << 3)
#define NCR_CLRSTAT     (1u << 5)

#define NCFGR_CAF       (1u << 4)  /* Copy All Frames (promisc) */
#define NCFGR_NBC       (1u << 1)  /* No BroadCast (0 = allow) */

/* ---- Descriptor ownership & fields for modern MACB/GEM (USED in ADDR MSB) ---- */

/* RX descriptor */
#ifndef RX_USED
# define RX_USED 0x00000001u
#endif
#ifndef RX_WRAP
# define RX_WRAP 0x00000002u
#endif
#ifndef RX_ADDR_MASK
# define RX_ADDR_MASK   0x3fffffffu   /* lower 30 bits hold buffer address */
#endif
#ifndef RX_LEN_MASK
# define RX_LEN_MASK    0x00000fffu   /* adjust if your SoC uses 11/12 bits */
#endif

/* TX descriptor */
#ifndef TX_LEN_MASK
# define TX_LEN_MASK    0x000007ffu   /* adjust if your SoC uses different width */
#endif

/* --- Fallback register offsets used by the debug dump --- */
#ifndef MACB_NCR
# define MACB_NCR   0x0000
#endif
#ifndef MACB_NCFGR
# define MACB_NCFGR 0x0004
#endif
#ifndef MACB_NSR
# define MACB_NSR   0x0008
#endif
#ifndef MACB_TSR
# define MACB_TSR   0x0014
#endif
#ifndef MACB_RBQP
# define MACB_RBQP  0x0018
#endif
#ifndef MACB_TBQP
# define MACB_TBQP  0x001c
#endif
#ifndef MACB_RSR
# define MACB_RSR   0x0020
#endif


/* Helpful accessor */
static inline uint16_t macb_rx_len(uint32_t ctrl)
{
    return (uint16_t)(ctrl & RX_LEN_MASK);
}

/* Descriptor */
struct macb_desc {
    volatile uint32_t addr;
    volatile uint32_t ctrl;
} __attribute__((__packed__));

/* HW mapping */
struct macb_hw {
    volatile uint8_t *regs;  /* MMIO base (UIO) */
    int uio_fd;
    size_t regs_len;
};

/* optional SW stats */
struct macb_sw_stats {
    uint64_t rx_pkts, rx_bytes, rx_errs;
    uint64_t tx_pkts, tx_bytes, tx_errs;
};

/* Queues */
struct macb_rxq {
    struct macb_desc *ring;
    struct rte_mbuf **sw_ring;
    uint16_t prod, cons, nb_desc;
    struct rte_mempool *mp;
    const struct rte_memzone *mz;
    rte_iova_t ring_iova;
    int sync_fd;
    struct macb_adapter *ad;
    uint16_t port_id;
};

struct macb_txq {
    struct macb_desc *ring;
    struct rte_mbuf **sw_ring;
    uint16_t prod, cons, nb_desc;
    const struct rte_memzone *mz;
    rte_iova_t ring_iova;
    int sync_fd;
    struct macb_adapter *ad;
};

/* Per-port adapter */
struct rte_eth_dev;
struct macb_adapter {
    struct rte_eth_dev *edev;
    struct macb_hw hw;
    struct macb_rxq rxq[1];
    struct macb_txq txq[1];
    struct macb_sw_stats sw;
    uint8_t  mac_addr[6];
    uint32_t link_speed;
    int      link_up;
    uint32_t ncfgr_shadow;
    int      promisc;
    int      force_phy_lb;
    uint16_t port_id;
    int sync_fd;
};

/* UIO helpers */
int  macb_uio_map(struct macb_hw *hw, const char *uio_path);
void macb_uio_unmap(struct macb_hw *hw);

/* bursts */
uint16_t macb_rx_burst(void *queue, struct rte_mbuf **rx_pkts, uint16_t nb_pkts);
uint16_t macb_tx_burst(void *queue, struct rte_mbuf **tx_pkts, uint16_t nb_pkts);

/* MMIO helpers used by ethdev */
static inline void macb_writel(struct macb_hw *hw, uint32_t off, uint32_t v) {
    rte_write32(v, (volatile void *)(hw->regs + off));
}
static inline uint32_t macb_readl(struct macb_hw *hw, uint32_t off) {
    return rte_read32((volatile void *)(hw->regs + off));
}

void macb_rx_init(struct macb_rxq *rxq);

void macb_rx_probe_once(struct macb_rxq *rxq);

