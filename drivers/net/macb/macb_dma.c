// macb_dma.c — DMA cache synchronization for Cadence GEM

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <rte_log.h>

#include "macb_hw.h"
#include "macb_dma.h"

#ifndef RTE_LOGTYPE_PMD
#define RTE_LOGTYPE_PMD RTE_LOGTYPE_USER1
#endif

#define MACB_ERR(fmt, ...) RTE_LOG(ERR, PMD, "macb: " fmt, ##__VA_ARGS__)

/**
 * macb_dma_sync_open_once - Open DMA sync helper device (once per adapter)
 *
 * On non-coherent SoCs, userspace must explicitly sync caches between
 * CPU and DMA master. This opens the dma_sync_eth0 character device.
 *
 * If the device is not available, continues anyway (may stall on non-coherent).
 */
void macb_dma_sync_open_once(struct macb_adapter *ad)
{
    if (ad->sync_fd >= 0) return;
    const char *node = getenv("MACB_DMA_SYNC_NODE");
    if (!node || !*node) node = "/dev/dma_sync_eth0";
    ad->sync_fd = open(node, O_RDWR);
    if (ad->sync_fd < 0) {
        MACB_ERR("dma_sync_helper not active (open(%s) failed) — continuing without cache sync; "
                 "TX/RX may stall on non-coherent SoCs\n", node);
    }
}

/**
 * macb_dma_sync_close - Close DMA sync helper device
 */
void macb_dma_sync_close(struct macb_adapter *ad)
{
    if (ad->sync_fd >= 0) {
        close(ad->sync_fd);
        ad->sync_fd = -1;
    }
}
