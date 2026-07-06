// macb_ring.c — Ring allocation and initialization for Cadence GEM

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <rte_common.h>
#include <rte_dev.h>
#include <rte_ethdev.h>
#include <ethdev_driver.h>
#include <rte_io.h>
#include <rte_log.h>
#include <rte_malloc.h>
#include <rte_memory.h>
#include <rte_memzone.h>

#include "macb_hw.h"
#include "macb_dma.h"
#include "macb_ring.h"
#include "dma_sync_uapi.h"
#include "macb_dma_sync.h"

#ifndef RTE_LOGTYPE_PMD
#define RTE_LOGTYPE_PMD RTE_LOGTYPE_USER1
#endif

#define MACB_DBG(fmt, ...) RTE_LOG(INFO,  PMD, "macb: " fmt, ##__VA_ARGS__)
#define MACB_ERR(fmt, ...) RTE_LOG(ERR,   PMD, "macb: " fmt, ##__VA_ARGS__)

/* Sync macros (must match ethdev.c) */
#define sync_desc_to_dev(fd, ptr, len)    macb_sync_to_dev_fd((fd), (ptr), (len))
#define sync_desc_from_dev(fd, ptr, len)  macb_sync_from_dev_fd((fd), (ptr), (len))

/* Import TX wrap flag */
#ifndef TX_WRAP
#define TX_WRAP (1u << 30)
#endif
#ifndef TX_USED
#define TX_USED (1u << 31)
#endif

/* macb_rx_init is declared in macb_hw.h and implemented in macb_rxtx.c */

/* ============== Ring Allocation ============== */

/**
 * macb_ring_alloc_rx - Allocate RX ring buffer and descriptor table
 *
 * Allocates:
 * - DMA-coherent descriptor ring from memzone
 * - Software ring buffer pointers array
 * - Stores IOVA and configures hw_dma_cap
 */
int macb_ring_alloc_rx(struct rte_eth_dev *dev, uint16_t qid,
                       uint16_t nb_desc, struct rte_mempool *mp)
{
    struct macb_adapter *ad = dev->data->dev_private;
    struct macb_rxq *rxq = &ad->rxq[qid];

    /* Set DMA capability and descriptor stride */
    rxq->hw_dma_cap = ad->hw_dma_cap;
    rxq->desc_stride = (uint16_t)macb_desc_stride_bytes(rxq->hw_dma_cap);

    size_t ring_bytes = (size_t)nb_desc * (size_t)rxq->desc_stride;

    /* Reserve DMA-coherent memzone for descriptors */
    const struct rte_memzone *mz =
        rte_eth_dma_zone_reserve(dev, "rx_ring", qid,
                                 ring_bytes,
                                 RTE_CACHE_LINE_SIZE, rte_socket_id());

    if (!mz) {
        MACB_ERR("RX ring memzone reserve failed: qid=%u ring_bytes=%zu socket=%d\n",
                 qid, ring_bytes, rte_socket_id());
        return -ENOMEM;
    }

    RTE_LOG(INFO, PMD,
        "RX ring alloc: cap=0x%x stride=%u nb=%u ring_bytes=%zu (mz.len=%zu) "
        "VA=%p IOVA=0x%" PRIx64 "\n",
        rxq->hw_dma_cap, rxq->desc_stride, nb_desc, ring_bytes, mz->len, 
        mz->addr, mz->iova);

    /* DMA-map the descriptor ring if needed */
    int rc = rte_dev_dma_map(dev->device, (void *)(uintptr_t)mz->addr, 
                             (rte_iova_t)mz->iova, mz->len);
    if (rc == -ENOTSUP || rc == -EINVAL) rc = 0;  /* Some platforms don't require explicit map */
    if (rc) {
        MACB_ERR("dma_map RX ring failed: %d (mz.addr=%p mz.iova=%" PRIx64 " mz.len=%zu)\n",
                 rc, mz->addr, mz->iova, mz->len);
        return rc;
    }

    /* Store ring pointers and configuration */
    rxq->ring = (uint8_t *)mz->addr;
    rxq->vring = (volatile uint8_t *)mz->addr;
    rxq->ring_iova = mz->iova;
    rxq->nb_desc = nb_desc;
    rxq->mp = mp;
    rxq->ad = ad;
    rxq->port_id = dev->data->port_id;

    /* Allocate software ring (mbuf pointers) */
    rxq->sw_ring = rte_zmalloc_socket("macb_rx_sw",
                                      (size_t)nb_desc * sizeof(struct rte_mbuf *),
                                      RTE_CACHE_LINE_SIZE, rte_socket_id());
    if (!rxq->sw_ring) {
        MACB_ERR("RX sw_ring allocation failed: qid=%u\n", qid);
        return -ENOMEM;
    }

    rxq->data_room_bytes = (uint16_t)(rte_pktmbuf_data_room_size(mp) - RTE_PKTMBUF_HEADROOM);

    /* Open DMA sync helper (once per adapter, safe to call multiple times) */
    macb_dma_sync_open_once(ad);
    rxq->sync_fd = ad->sync_fd;

    RTE_LOG(INFO, PMD, "RX ring allocation complete: nb=%u stride=%u bytes=%zu\n",
            nb_desc, rxq->desc_stride, ring_bytes);

    return 0;
}

/**
 * macb_ring_alloc_tx - Allocate TX ring buffer and descriptor table
 */
int macb_ring_alloc_tx(struct rte_eth_dev *dev, uint16_t qid, uint16_t nb_desc)
{
    struct macb_adapter *ad = dev->data->dev_private;
    struct macb_txq *txq = &ad->txq[qid];

    /* Set DMA capability and descriptor stride */
    txq->hw_dma_cap = ad->hw_dma_cap;
    txq->desc_stride = (uint16_t)macb_desc_stride_bytes(txq->hw_dma_cap);

    size_t ring_bytes = (size_t)nb_desc * (size_t)txq->desc_stride;

    /* Reserve DMA-coherent memzone for descriptors */
    const struct rte_memzone *mz =
        rte_eth_dma_zone_reserve(dev, "tx_ring", qid,
                                 ring_bytes,
                                 RTE_CACHE_LINE_SIZE, rte_socket_id());
    if (!mz) {
        MACB_ERR("TX ring memzone reserve failed: qid=%u ring_bytes=%zu\n", 
                 qid, ring_bytes);
        return -ENOMEM;
    }

    RTE_LOG(INFO, PMD,
        "TX ring alloc: cap=0x%x stride=%u nb=%u ring_bytes=%zu VA=%p IOVA=0x%" PRIx64 "\n",
        txq->hw_dma_cap, txq->desc_stride, nb_desc, ring_bytes, mz->addr, mz->iova);

    /* DMA-map the descriptor ring if needed */
    int rc = rte_dev_dma_map(dev->device, (void *)(uintptr_t)mz->addr,
                             (rte_iova_t)mz->iova, mz->len);
    if (rc == -ENOTSUP || rc == -EINVAL) rc = 0;
    if (rc) {
        MACB_ERR("dma_map TX ring failed: %d\n", rc);
        return rc;
    }

    /* Store ring pointers and configuration */
    txq->mz = mz;
    txq->ring = (uint8_t *)mz->addr;
    txq->vring = (volatile uint8_t *)mz->addr;
    txq->ring_iova = mz->iova;
    txq->nb_desc = nb_desc;
    txq->ad = ad;

    /* Open DMA sync helper (once per adapter, safe to call multiple times) */
    macb_dma_sync_open_once(ad);
    txq->sync_fd = ad->sync_fd;

    /* Allocate software ring (mbuf pointers) */
    txq->sw_ring = rte_zmalloc_socket("macb_tx_sw",
                                      (size_t)nb_desc * sizeof(struct rte_mbuf *),
                                      RTE_CACHE_LINE_SIZE, rte_socket_id());
    if (!txq->sw_ring) {
        MACB_ERR("TX sw_ring allocation failed: qid=%u\n", qid);
        return -ENOMEM;
    }

    /* Allocate optional debug control array */
    txq->dbg_ctrl0 = rte_zmalloc("macb_dbg_ctrl0",
                                 (size_t)nb_desc * sizeof(uint32_t), 0);

    RTE_LOG(INFO, PMD, "TX ring allocation complete: nb=%u stride=%u bytes=%zu\n",
            nb_desc, txq->desc_stride, ring_bytes);

    return 0;
}

/* ============== Ring Initialization ============== */

/**
 * macb_ring_init_rx - Initialize RX ring descriptors
 *
 * Clears descriptor memory and calls macb_rx_init() to populate
 * each descriptor with the appropriate flags and cached values.
 */
int macb_ring_init_rx(struct macb_rxq *rxq)
{
    size_t ring_bytes = (size_t)rxq->nb_desc * (size_t)rxq->desc_stride;

    /* Clear descriptor ring */
    memset(rxq->ring, 0, ring_bytes);
    sync_desc_to_dev(rxq->sync_fd, rxq->ring, ring_bytes);
    rte_io_wmb();

    /* Initialize descriptors with RX-specific setup */
    macb_rx_init(rxq);

    /* Ensure HW sees all writes */
    sync_desc_to_dev(rxq->sync_fd, rxq->ring, ring_bytes);
    rte_io_wmb();

    RTE_LOG(INFO, PMD, "RX ring initialization complete: nb=%u\n", rxq->nb_desc);

    return 0;
}

/**
 * macb_ring_init_tx - Initialize TX ring descriptors
 *
 * Each descriptor is set with TX_USED (free) and the WRAP flag
 * on the last descriptor. HW will transmit only descriptors
 * with TX_USED cleared.
 */
int macb_ring_init_tx(struct macb_txq *txq)
{
    /* Initialize each descriptor as FREE (TX_USED set) */
    for (uint16_t i = 0; i < txq->nb_desc; i++) {
        const uint32_t wrap = (i == txq->nb_desc - 1) ? TX_WRAP : 0;

        volatile uint32_t *w = 
            (volatile uint32_t *)macb_desc_at(txq->ring, txq->hw_dma_cap, i);
        
        /* Clear entire descriptor */
        memset((void *)(uintptr_t)w, 0, txq->desc_stride);
        
        /* Set control word: TX_USED indicates buffer owned by SW */
        w[0] = 0;
        w[1] = TX_USED | wrap;
        
        /* Sync this descriptor to device */
        sync_desc_to_dev(txq->sync_fd, (void *)(uintptr_t)w, txq->desc_stride);
    }

    /* Sync all descriptors to ensure device sees them */
    sync_desc_to_dev(txq->sync_fd, txq->ring, 
                     (size_t)txq->nb_desc * txq->desc_stride);
    rte_io_wmb();

    /* Initialize software state */
    txq->prod = 0;
    txq->cons = 0;

    RTE_LOG(INFO, PMD, "TX ring initialization complete: nb=%u\n", txq->nb_desc);

    return 0;
}

/* ============== Ring Cleanup ============== */

/**
 * macb_ring_free_rx - Free RX ring resources
 */
void macb_ring_free_rx(struct macb_rxq *rxq)
{
    if (rxq->sw_ring) {
        rte_free(rxq->sw_ring);
        rxq->sw_ring = NULL;
    }
    if (rxq->ring) {
        rxq->ring = NULL;
        rxq->vring = NULL;
    }
}

/**
 * macb_ring_free_tx - Free TX ring resources
 */
void macb_ring_free_tx(struct macb_txq *txq)
{
    if (txq->sw_ring) {
        rte_free(txq->sw_ring);
        txq->sw_ring = NULL;
    }
    if (txq->dbg_ctrl0) {
        rte_free(txq->dbg_ctrl0);
        txq->dbg_ctrl0 = NULL;
    }
    if (txq->ring) {
        txq->ring = NULL;
        txq->vring = NULL;
    }
}
