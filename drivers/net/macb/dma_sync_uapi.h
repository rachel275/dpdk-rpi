/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/* drivers/net/macb/dma_sync_uapi.h
 * Userspace UAPI mirror for dma_sync_helper.
 *
 * This header is included by the DPDK PMD and must stay libc-friendly.
 */
#ifndef DMA_SYNC_UAPI_H
#define DMA_SYNC_UAPI_H

#include <stdint.h>
#include <sys/ioctl.h>

#define DMA_SYNC_IOC_MAGIC  'd'

struct dma_sync_range {
    uint64_t uaddr; /* user virtual address */
    uint64_t len;   /* bytes */
};

struct dma_sync_vec {
    uint64_t nr;
    uint64_t user_ptr_entries; /* userspace ptr to array of dma_sync_range */
};

/* Optional DMA-address ioctls (only if your kernel module supports them) */
struct dma_sync_dma {
    uint64_t dma_addr; /* IOVA/PA as seen by device */
    uint64_t len;
};

#define DMA_SYNC_TO_DEV          _IOW(DMA_SYNC_IOC_MAGIC, 1, struct dma_sync_range)
#define DMA_SYNC_FROM_DEV        _IOW(DMA_SYNC_IOC_MAGIC, 2, struct dma_sync_range)
#define DMA_SYNCV_TO_DEV         _IOW(DMA_SYNC_IOC_MAGIC, 3, struct dma_sync_vec)
#define DMA_SYNCV_FROM_DEV       _IOW(DMA_SYNC_IOC_MAGIC, 4, struct dma_sync_vec)

/* If your module implements these, keep them enabled. */
#define DMA_SYNC_DMA_TO_DEV      _IOW(DMA_SYNC_IOC_MAGIC, 5, struct dma_sync_dma)
#define DMA_SYNC_DMA_FROM_DEV    _IOW(DMA_SYNC_IOC_MAGIC, 6, struct dma_sync_dma)

/* Feature flag used by macb_dma_sync.h */
#define DMA_SYNC_HAVE_DMA_IOCTLS 1

#endif /* DMA_SYNC_UAPI_H */
