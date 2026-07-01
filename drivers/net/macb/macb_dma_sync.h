/* SPDX-License-Identifier: BSD-3-Clause */
/* drivers/net/macb/macb_dma_sync.h */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <inttypes.h>
#include <sys/ioctl.h>

#include <rte_common.h>
#include <rte_log.h>
#include <rte_io.h>

#include "dma_sync_uapi.h"

/*
 * VA-based syncing aligns to cache lines to avoid partial-line hazards.
 * DMA-address syncing does not need VA alignment (kernel does DMA sync by addr+len).
 */

#ifndef MACB_CL_SIZE
#define MACB_CL_SIZE RTE_CACHE_LINE_SIZE
#endif

static __rte_always_inline void
macb_align_range(const void *addr, size_t len, uintptr_t *start, size_t *alen)
{
    const uintptr_t p = (uintptr_t)addr;
    const uintptr_t s = p & ~(uintptr_t)(MACB_CL_SIZE - 1);
    const uintptr_t e = (p + len + (MACB_CL_SIZE - 1)) & ~(uintptr_t)(MACB_CL_SIZE - 1);
    *start = s;
    *alen  = (size_t)(e - s);
}

static __rte_always_inline int
macb_ioctl_ret(int rc)
{
    if (likely(rc == 0))
        return 0;
    /* ioctl returns -1 on error */
    return (errno != 0) ? -errno : -EIO;
}

/* -------- VA-based sync -------- */

static __rte_always_inline int
macb_sync_to_dev_fd(int fd, const volatile void *addr, size_t len)
{
    if (fd < 0 || len == 0)
        return 0;

    uintptr_t start; size_t alen;
    macb_align_range((const void *)(uintptr_t)addr, len, &start, &alen);

    struct dma_sync_range r = { .uaddr = start, .len = alen };
    int rc = ioctl(fd, DMA_SYNC_TO_DEV, &r);
    int ret = macb_ioctl_ret(rc);

    if (unlikely(ret)) {
        RTE_LOG(ERR, PMD,
            "macb: DMA_SYNC_TO_DEV failed ret=%d addr=%p len=%zu aligned=%p+%zu fd=%d\n",
            ret, (const void *)(uintptr_t)addr, len, (void *)start, alen, fd);
    }

    /* If caller uses this for descriptors, a wmb after “publish” is sensible */
    rte_io_wmb();
    return ret;
}

static __rte_always_inline int
macb_sync_from_dev_fd(int fd, const volatile void *addr, size_t len)
{
    if (fd < 0 || len == 0)
        return 0;

    uintptr_t start; size_t alen;
    macb_align_range((const void *)(uintptr_t)addr, len, &start, &alen);

    struct dma_sync_range r = { .uaddr = start, .len = alen };
    int rc = ioctl(fd, DMA_SYNC_FROM_DEV, &r);
    int ret = macb_ioctl_ret(rc);

    if (unlikely(ret)) {
        RTE_LOG(ERR, PMD,
            "macb: DMA_SYNC_FROM_DEV failed ret=%d addr=%p len=%zu aligned=%p+%zu fd=%d\n",
            ret, (const void *)(uintptr_t)addr, len, (void *)start, alen, fd);
    }

    /* If caller uses this before reading descriptor/status, an rmb is sensible */
    rte_io_rmb();
    return ret;
}

/* -------- DMA-address sync (preferred for DPDK hugepages) -------- */

static __rte_always_inline int
macb_sync_dma_to_dev_fd(int fd, uint64_t dma_addr, size_t len)
{
#if defined(DMA_SYNC_HAVE_DMA_IOCTLS) && DMA_SYNC_HAVE_DMA_IOCTLS
    if (fd < 0 || len == 0)
        return 0;

    struct dma_sync_dma d = { .dma_addr = dma_addr, .len = (uint64_t)len };
    int rc = ioctl(fd, DMA_SYNC_DMA_TO_DEV, &d);
    int ret = macb_ioctl_ret(rc);

    if (unlikely(ret)) {
        RTE_LOG(ERR, PMD,
            "macb: DMA_SYNC_DMA_TO_DEV failed ret=%d dma=%"PRIx64" len=%zu fd=%d\n",
            ret, dma_addr, len, fd);
    }

    rte_io_wmb();
    return ret;
#else
    (void)fd; (void)dma_addr; (void)len;
    return 0;
#endif
}

static __rte_always_inline int
macb_sync_dma_from_dev_fd(int fd, uint64_t dma_addr, size_t len)
{
#if defined(DMA_SYNC_HAVE_DMA_IOCTLS) && DMA_SYNC_HAVE_DMA_IOCTLS
    if (fd < 0 || len == 0)
        return 0;

    struct dma_sync_dma d = { .dma_addr = dma_addr, .len = (uint64_t)len };
    int rc = ioctl(fd, DMA_SYNC_DMA_FROM_DEV, &d);
    int ret = macb_ioctl_ret(rc);

    if (unlikely(ret)) {
        RTE_LOG(ERR, PMD,
            "macb: DMA_SYNC_DMA_FROM_DEV failed ret=%d dma=%"PRIx64" len=%zu fd=%d\n",
            ret, dma_addr, len, fd);
    }

    rte_io_rmb();
    return ret;
#else
    (void)fd; (void)dma_addr; (void)len;
    return 0;
#endif
}

/* -------- Convenience wrappers -------- */

#define sync_desc_to_dev(fd, ptr, len)      macb_sync_to_dev_fd((fd), (ptr), (len))
#define sync_desc_from_dev(fd, ptr, len)    macb_sync_from_dev_fd((fd), (ptr), (len))
#define sync_buf_to_dev(fd, ptr, len)       macb_sync_to_dev_fd((fd), (ptr), (len))
#define sync_buf_from_dev(fd, ptr, len)     macb_sync_from_dev_fd((fd), (ptr), (len))

#define sync_buf_to_dev_dma(fd, dma, len)   macb_sync_dma_to_dev_fd((fd), (dma), (len))
#define sync_buf_from_dev_dma(fd, dma, len) macb_sync_dma_from_dev_fd((fd), (dma), (len))

static __rte_always_inline int sync_desc_to_dev_v(int fd, volatile void *p, size_t len)
{ return sync_desc_to_dev(fd, (const volatile void *)p, len); }

static __rte_always_inline int sync_desc_from_dev_v(int fd, volatile void *p, size_t len)
{ return sync_desc_from_dev(fd, (const volatile void *)p, len); }

static __rte_always_inline int sync_buf_to_dev_s(int fd, const void *p, size_t len)
{ return sync_buf_to_dev(fd, (const volatile void *)p, len); }

static __rte_always_inline int sync_buf_from_dev_s(int fd, const void *p, size_t len)
{ return sync_buf_from_dev(fd, (const volatile void *)p, len); }
