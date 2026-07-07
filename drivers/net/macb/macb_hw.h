#pragma once

#include <stdint.h>
#include <stddef.h>
#include <rte_common.h>
#include <rte_memory.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_ethdev.h>
#include <rte_io.h>

extern uint64_t g_bus_ofs;

#define BUS_IOVA_DATA(x) \
    ((rte_iova_t)((rte_iova_t)(x) + (rte_iova_t)g_bus_ofs))
#define BUS_IOVA(x) BUS_IOVA_DATA((x))

/* ===================== Tunables ===================== */
#define MACB_NRXD   256
#define MACB_NTXD   256
#define MACB_ALIGN  64

/* ===================== Registers ===================== */
/* Core regs (MACB/GEM family) */
#ifndef MACB_NCR
# define MACB_NCR    0x0000u   /* Network Control */
#endif
#ifndef MACB_NCFGR
# define MACB_NCFGR  0x0004u   /* Network Configuration */
#endif
#ifndef MACB_NSR
# define MACB_NSR    0x0008u   /* Network Status */
#endif
#ifndef MACB_TSR
# define MACB_TSR    0x0014u   /* Transmit Status */
#endif
#ifndef MACB_RBQP
# define MACB_RBQP   0x0018u   /* RX Buffer Queue Pointer (low) */
#endif
#ifndef MACB_TBQP
# define MACB_TBQP   0x001Cu   /* TX Buffer Queue Pointer (low) */
#endif
#ifndef MACB_RSR
# define MACB_RSR    0x0020u   /* Receive Status */
#endif
#ifndef MACB_MAN
# define MACB_MAN    0x0034u   /* MDIO Access */
#endif

/* 64-bit ring base high dwords (present on many GEM variants incl. RP1) */
//#ifndef MACB_RBQPH
//# define MACB_RBQPH  0x00A4u
//#endif
//#ifndef MACB_TBQPH
//# define MACB_TBQPH  0x00A8u
//#endif

/* Match Linux cadence/macb.h for RBQPH/TBQPH */
#ifndef MACB_TBQPH
# define MACB_TBQPH  0x04C8u
#endif
#ifndef MACB_RBQPH
# define MACB_RBQPH  0x04D4u
#endif

#ifndef GEM_TBQPH
# define GEM_TBQPH(hw_q)  (0x04C8u)
#endif
#ifndef GEM_RBQPH
# define GEM_RBQPH(hw_q)  (0x04D4u)
#endif

/* GEM queue-0 base regs (common aliasing) */
#ifndef GEM_RBQB_Q0
# define GEM_RBQB_Q0 0x0480u
#endif
#ifndef GEM_TBQB_Q0
# define GEM_TBQB_Q0 0x0484u
#endif
#define GEM_RBQP(q)  (GEM_RBQB_Q0 + ((q) << 2))
#define GEM_TBQP(q)  (GEM_TBQB_Q0 + ((q) << 2))

/* GEM DMA Configuration (for RX buffer size programming) */
#ifndef GEM_DMACFG
# define GEM_DMACFG               0x0010u
#endif
#ifndef GEM_DMACFG_RXBS_SHIFT
# define GEM_DMACFG_RXBS_SHIFT    16         /* DRBS field shift (x64B units) */
#endif
#ifndef GEM_DMACFG_RXBS_MASK
# define GEM_DMACFG_RXBS_MASK     (0xFFu << GEM_DMACFG_RXBS_SHIFT)
#endif
#ifndef GEM_DMACFG_RXBS
# define GEM_DMACFG_RXBS(units64) (((uint32_t)(units64) & 0xFFu) << GEM_DMACFG_RXBS_SHIFT)
#endif
#ifndef GEM_DCFG5
#define GEM_DCFG5   0x0290u   /* has TSU bit, ADDR64 bit live in DCFG regs on macb-family */
#endif
#ifndef GEM_DCFG10
#define GEM_DCFG10  0x0298u
#endif

/* ================= NCR / NCFGR bits ================= */
#ifndef NCR_RXEN
# define NCR_RXEN   (1u << 2)
#endif
#ifndef NCR_TXEN
# define NCR_TXEN   (1u << 3)
#endif
#ifndef NCR_CLRSTAT
# define NCR_CLRSTAT (1u << 5)
#endif
#ifndef NCR_LB
# define NCR_LB     (1u << 1)   /* MAC internal loopback */
#endif
#ifndef NCR_MPE
# define NCR_MPE    (1u << 4)   /* MDIO management port enable */
#endif
#ifndef NCR_TSTART
# define NCR_TSTART (1u << 9)   /* Start transmission from idle */
#endif

/* Speed/duplex controls (names vary across SoCs; keep fallbacks) */
#ifndef NCFGR_SPD
# define NCFGR_SPD  (1u << 0)   /* 100 Mb/s select (GEM: SPD) */
#endif
#ifndef NCFGR_FD
# define NCFGR_FD   (1u << 1)   /* Full duplex */
#endif
#ifndef GEM_NCFGR_GBE
# define GEM_NCFGR_GBE (1u << 10) /* 1 Gb/s enable on GEM variants */
#endif

/* Receive filter controls: keep both MACB_ and legacy aliases consistent */
#ifndef MACB_NCFGR_CAF
# define MACB_NCFGR_CAF (1u << 4)  /* Copy-All-Frames (promisc) */
#endif
#ifndef MACB_NCFGR_NBC
# define MACB_NCFGR_NBC (1u << 5)  /* No BroadCast (1 = drop broadcast) */
#endif
#ifndef NCFGR_CAF
# define NCFGR_CAF MACB_NCFGR_CAF
#endif
#ifndef NCFGR_NBC
# define NCFGR_NBC MACB_NCFGR_NBC
#endif
#ifndef NCFGR_DRFCS
# define NCFGR_DRFCS (1u << 11)    /* Discard Rx FCS */
#endif
/* ========= Logging helpers (maps to DPDK logging) ========= */
#include <rte_log.h>

/* Dynamic logtype registered once (in macb_uio.c) via RTE_LOG_REGISTER_DEFAULT().
 * This extern + alias makes RTE_LOG(level, MACB, ...) resolve to that same
 * registered id from every translation unit in the driver.
 *
 * NOTE: the old fallback here used to be
 *   #ifndef RTE_LOGTYPE_PMD
 *   #define RTE_LOGTYPE_PMD RTE_LOGTYPE_USER1
 *   #endif
 * Modern DPDK no longer defines a static RTE_LOGTYPE_PMD, so that #ifndef was
 * always true and silently rerouted every "PMD"-tagged log call in this
 * driver to RTE_LOGTYPE_USER1 -- which is why raising the "pmd" log level
 * had no effect on this driver's debug output. Don't reintroduce it.
 */
extern int macb_logtype;
#define RTE_LOGTYPE_MACB macb_logtype

#ifndef MACB_LOG
#define MACB_LOG(level, fmt, ...) \
    RTE_LOG(level, MACB, "macb: " fmt "\n", ##__VA_ARGS__)
#endif

#ifndef DEBUG
#define DEBUG RTE_LOG_DEBUG
#endif

/* ================== Generic bit helper ================== */
#ifndef MACB_BIT
#define MACB_BIT(n) (1u << (n))
#endif

/* ================== RSR bit positions =================== */
/* Receive Status Register (MACB/GEM):
 *  REC: frame received
 *  OVR: overrun
 *  BNA: buffer not available
 * These are bit positions, used with MACB_BIT().
 */
#ifndef REC
#define REC 0
#endif
#ifndef OVR
#define OVR 1
#endif
#ifndef BNA
#define BNA 2
#endif

/* Mask of speed/duplex bits we overwrite during link update */
#define NCFGR_SPEED_MASK (NCFGR_SPD | NCFGR_FD | GEM_NCFGR_GBE)

/* Specific Address 1 (perfect match unicast) */
#ifndef GEM_SA1B
# define GEM_SA1B 0x0088u  /* Specific Address 1 Bottom (bytes 0..3) */
#endif
#ifndef GEM_SA1T
# define GEM_SA1T 0x008Cu  /* Specific Address 1 Top    (bytes 4..5) */
#endif

/* Some code paths use MACB_* names; alias them to GEM_* */
#ifndef MACB_SA1B
# define MACB_SA1B GEM_SA1B
#endif
#ifndef MACB_SA1T
# define MACB_SA1T GEM_SA1T
#endif

/* ================== Descriptors (GEM layout) ================== */
/* RX descriptor address word (bit0..1 special) */
#ifndef RX_USED
# define RX_USED      0x00000001u   /* ADDR bit0: 1 = SW owns (not armed) */
#endif
#ifndef RX_WRAP
# define RX_WRAP      0x00000002u   /* ADDR bit1: wrap at end of ring */
#endif
#ifndef RX_ADDR_MASK
# define RX_ADDR_MASK 0x3FFFFFFCu   /* ADDR[31:2] carry buffer address */
#endif

/* RX status/length word */
#ifndef RX_SOF
# define RX_SOF       (1u << 14)
#endif
#ifndef RX_EOF
# define RX_EOF       (1u << 15)
#endif
#ifndef RX_LEN_MASK
# define RX_LEN_MASK  0x00001FFFu   /* 13-bit length on GEM */
#endif

/* TX descriptor control word (GEM) */
#ifndef TX_USED
# define TX_USED      (1u << 31)    /* 1 = FREE (SW owns); 0 = in use by HW */
#endif
#ifndef TX_WRAP
# define TX_WRAP      (1u << 30)    /* marks last descriptor in ring */
#endif
#ifndef TX_LAST
# define TX_LAST      (1u << 15)
#endif
#ifndef TX_LEN_MASK
# define TX_LEN_MASK  0x00003FFFu   /* 14-bit length on GEM */
#endif

/* ================== Helper accessors ================== */
static inline uint16_t macb_rx_len(uint32_t ctrl)
{
    return (uint16_t)(ctrl & RX_LEN_MASK);
}

/* ================== Descriptor / HW structs ================== */

struct macb_desc {
    volatile uint32_t addr;
    volatile uint32_t ctrl;
};
_Static_assert(sizeof(struct macb_desc) == 8, "macb_desc must be 8 bytes");

struct macb_desc_64 {
    volatile uint32_t addrh;
    volatile uint32_t resvd;
};
_Static_assert(sizeof(struct macb_desc_64) == 8, "macb_desc_64 must be 8 bytes");

struct macb_desc_ptp {
    volatile uint32_t ts_1;
    volatile uint32_t ts_2;
};
_Static_assert(sizeof(struct macb_desc_ptp) == 8, "macb_desc_ptp must be 8 bytes");

enum macb_hw_dma_cap {
    HW_DMA_CAP_32B     = 0,
    HW_DMA_CAP_64B     = (1u << 0),
    HW_DMA_CAP_PTP     = (1u << 1),
    HW_DMA_CAP_64B_PTP = (HW_DMA_CAP_64B | HW_DMA_CAP_PTP),
};

static inline uint32_t macb_desc_stride_bytes(unsigned cap)
{
    switch (cap) {
    case HW_DMA_CAP_64B:
        return sizeof(struct macb_desc) + sizeof(struct macb_desc_64);
    case HW_DMA_CAP_PTP:
        return sizeof(struct macb_desc) + sizeof(struct macb_desc_ptp);
    case HW_DMA_CAP_64B_PTP:
        return sizeof(struct macb_desc) + sizeof(struct macb_desc_64) + sizeof(struct macb_desc_ptp);
    default:
        return sizeof(struct macb_desc);
    }
}

static inline uint32_t macb_desc_size_bytes(unsigned caps)
{
    return macb_desc_stride_bytes(caps);

}

static inline volatile struct macb_desc *
macb_desc_at_stride(void *ring_base, uint32_t desc_stride, uint32_t idx)
{
    return (volatile struct macb_desc *)((uint8_t *)ring_base + (size_t)idx * (size_t)desc_stride);
}


static inline volatile struct macb_desc *
macb_desc_at(void *ring_base, unsigned cap, uint32_t idx)
{
    const size_t stride = (size_t)macb_desc_stride_bytes(cap);
    return (volatile struct macb_desc *)((uint8_t *)ring_base + (size_t)idx * stride);
}

static inline volatile struct macb_desc_64 *
macb_desc64_ptr(volatile struct macb_desc *d, unsigned cap)
{
    if (cap & HW_DMA_CAP_64B)
        return (volatile struct macb_desc_64 *)((volatile uint8_t *)d + sizeof(*d));
    return NULL;
}

static inline volatile struct macb_desc_ptp *
macb_descptp_ptr(volatile struct macb_desc *d, unsigned cap)
{
    if (!(cap & HW_DMA_CAP_PTP))
        return NULL;

    size_t off = sizeof(*d);
    if (cap & HW_DMA_CAP_64B)
        off += sizeof(struct macb_desc_64);

    return (volatile struct macb_desc_ptp *)((volatile uint8_t *)d + off);
}


struct macb_hw {
    volatile uint8_t *regs;  /* MMIO base (UIO) */
    int               uio_fd;
    size_t            regs_len;
};

struct macb_sw_stats {
    uint64_t rx_pkts, rx_bytes, rx_errs;
    uint64_t tx_pkts, tx_bytes, tx_errs;
};

struct macb_rxq {
    struct macb_adapter *ad;
    struct rte_mempool  *mp;

    uint8_t            *ring;
    volatile uint8_t   *vring;
    rte_iova_t          ring_iova;

    uint8_t             hw_dma_cap;
    uint8_t             _pad0;
    uint16_t            desc_stride;

    struct rte_mbuf   **sw_ring;
    uint16_t            nb_desc;

    /* NEW: head/tail like Linux */
    uint16_t            rx_tail;          /* next desc to consume (Linux: rx_tail) */
    uint16_t            rx_head;          /* next desc to (re)fill/arm (Linux: rx_prepared_head) */

    uint16_t            port_id;
    int                 sync_fd;
    uint16_t            data_room_bytes;

    uint8_t             use_ctrl_used;
    uint8_t             used_autod;

    uint16_t            free_thresh;

    uint8_t             layout_status_is_w1;
    uint8_t             layout_autod;

    uint16_t            _pad1;
};

struct macb_txq {
    /* DMA descriptor ring (variable-length entries: 8/16/24 bytes) */
    uint8_t            *ring;        /* base VA */
    volatile uint8_t   *vring;       /* optional volatile view */
    rte_iova_t          ring_iova;    /* base IOVA */

    /* Cached descriptor layout */
    uint8_t             hw_dma_cap;   /* enum macb_hw_dma_cap */
    uint8_t             _pad0;
    uint16_t            desc_stride;  /* bytes per descriptor entry */

    struct rte_mbuf   **sw_ring;
    uint16_t            prod, cons, nb_desc;

    const struct rte_memzone *mz;
    int                 sync_fd;
    struct macb_adapter *ad;

    uint32_t           *dbg_ctrl0;
};

struct rte_eth_dev;
struct macb_adapter {
    struct rte_eth_dev *edev;
    struct macb_hw      hw;
    struct macb_rxq     rxq[1];
    struct macb_txq     txq[1];
    struct macb_sw_stats sw;
    uint8_t   mac_addr[6];
    uint32_t  link_speed;
    int       link_up;
    uint32_t  ncfgr_shadow;
    int       promisc;
    int       force_phy_lb;
    uint16_t  port_id;
    int       sync_fd;
    uint8_t   hw_dma_cap;
    uint32_t rp1_rbqb0_off; /* offset of queue0 RX base low */
    uint32_t rp1_tbqb0_off; /* offset of queue0 TX base low */
};

/* ================== UIO & burst prototypes ================== */
int  macb_uio_map(struct macb_hw *hw, const char *uio_path);
void macb_uio_unmap(struct macb_hw *hw);

uint16_t macb_rx_burst(void *queue, struct rte_mbuf **rx_pkts, uint16_t nb_pkts);
uint16_t macb_tx_burst(void *queue, struct rte_mbuf **tx_pkts, uint16_t nb_pkts);

/* MMIO helpers used by ethdev */
#include <rte_log.h>

static inline void macb_writel(struct macb_hw *hw, uint32_t off, uint32_t v)
{
    rte_write32(v, (volatile void *)((uintptr_t)hw->regs + off));
}

static inline uint32_t macb_readl(struct macb_hw *hw, uint32_t off) {
    return rte_read32((volatile void *)(hw->regs + off));
}

/* RX helpers exposed across TUs */
void macb_rx_init(struct macb_rxq *rxq);
void macb_rx_publish_desc(struct macb_rxq *rxq, volatile struct macb_desc *dv,
                          uint64_t bus_addr, uint32_t wrap);