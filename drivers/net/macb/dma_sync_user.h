// drivers/net/macb/dma_sync_user.h
#pragma once
#include <stddef.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>

/*
 * Minimal userland “DMA sync” shim for the PMD build.
 * - dma_sync_open("eth0") tries /dev/dma_sync_eth0 and returns fd or -1.
 * - dma_sync_to_dev / dma_sync_from_dev are NO-OPs here (coherent platforms).
 *
 * If you have a real helper device/driver, you can replace these
 * implementations with your ioctl/mmap logic. The PMD only needs a
 * best-effort hook; returning -1 is fine (it’ll keep running).
 */

extern int macb_dma_sync_fd;

static inline int dma_sync_open(const char *ifname)
{
    char path[128];
    if (!ifname || !*ifname) ifname = "eth0";
    snprintf(path, sizeof(path), "/dev/dma_sync_%s", ifname);
    int fd = open(path, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        // Not fatal: caller will warn & continue without explicit cache ops.
        return -1;
    }
    return fd;
}

static inline void dma_sync_to_dev(int fd, const void *addr, size_t len)
{
    (void)fd; (void)addr; (void)len;  /* no-op on coherent systems */
}

static inline void dma_sync_from_dev(int fd, const void *addr, size_t len)
{
    (void)fd; (void)addr; (void)len;  /* no-op on coherent systems */
}

