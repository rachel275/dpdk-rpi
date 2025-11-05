/* drivers/net/macb/dma_sync_uapi.h
 * Minimal UAPI mirror for the dma_sync_helper char device.
 */
#ifndef DMA_SYNC_UAPI_H
#define DMA_SYNC_UAPI_H

#include <stdint.h>
#include <sys/ioctl.h>

struct dma_sync_range {
    uint64_t uaddr;   /* user virtual address (start of buffer) */
    uint64_t len;     /* length in bytes */
};

#ifndef DMA_SYNC_IOC_MAGIC
#define DMA_SYNC_IOC_MAGIC  'd'
#endif

#ifndef DMA_SYNC_TO_DEV
#define DMA_SYNC_TO_DEV     _IOW(DMA_SYNC_IOC_MAGIC, 1, struct dma_sync_range)
#endif

#ifndef DMA_SYNC_FROM_DEV
#define DMA_SYNC_FROM_DEV   _IOW(DMA_SYNC_IOC_MAGIC, 2, struct dma_sync_range)
#endif

#endif /* DMA_SYNC_UAPI_H */

