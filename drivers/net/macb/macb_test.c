// drivers/net/macb/macb_test.c
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <rte_eal.h>
#include <rte_log.h>
#include <rte_memzone.h>
#include <rte_io.h>
#include <rte_errno.h>   // <-- for rte_errno
#include <rte_string_fns.h> // optional; safe in-tree

#include "dma_sync_uapi.h"   // your DMA_SYNC_TO_DEV/FROM_DEV + dma_sync_range

#ifndef RTE_LOGTYPE_APP
#define RTE_LOGTYPE_APP RTE_LOGTYPE_USER1
#endif

#define APP_LOG(fmt, ...) RTE_LOG(INFO, APP, "mztest: " fmt, ##__VA_ARGS__)
#define APP_ERR(fmt, ...) RTE_LOG(ERR,  APP, "mztest: " fmt, ##__VA_ARGS__)

static inline void sync_to_dev(int fd, const void *addr, size_t len)
{
    if (fd < 0 || len == 0) return;
    struct dma_sync_range r = { .uaddr = (uintptr_t)addr, .len = len };
    (void)ioctl(fd, DMA_SYNC_TO_DEV, &r);
}

static inline void sync_from_dev(int fd, const void *addr, size_t len)
{
    if (fd < 0 || len == 0) return;
    struct dma_sync_range r = { .uaddr = (uintptr_t)addr, .len = len };
    (void)ioctl(fd, DMA_SYNC_FROM_DEV, &r);
}

static uint32_t xor32(const uint8_t *p, size_t n)
{
    uint32_t x = 0;
    for (size_t i = 0; i < n; i++) x ^= p[i];
    return x;
}

static int verify(const uint8_t *p, size_t n, uint8_t pat)
{
    for (size_t i = 0; i < n; i++) {
        if (p[i] != pat) {
            APP_ERR("mismatch at +%zu: got 0x%02x expected 0x%02x\n", i, p[i], pat);
            return -1;
        }
    }
    return 0;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s <EAL args> -- "
        "--name <mzname> --bytes <n> --pattern <hexbyte> [--sync-node <path>]\n"
        "\nExample:\n"
        "  sudo %s -l 0 -n 4 -- --bytes 65536 --pattern 0xa5 --sync-node /dev/dma_sync_eth0\n",
        argv0, argv0);
}

int main(int argc, char **argv)
{
    int eal = rte_eal_init(argc, argv);
    if (eal < 0) rte_panic("EAL init failed\n");
    argc -= eal;
    argv += eal;

    const char *mzname = "macb_mz_test0";
    size_t bytes = 65536;
    uint8_t pat = 0xA5;
    const char *sync_node = NULL;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--name") && i + 1 < argc) mzname = argv[++i];
        else if (!strcmp(argv[i], "--bytes") && i + 1 < argc) bytes = strtoull(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--pattern") && i + 1 < argc) pat = (uint8_t)strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--sync-node") && i + 1 < argc) sync_node = argv[++i];
        else {
            usage(argv[-eal]);
            return 1;
        }
    }

    if (bytes == 0 || bytes > RTE_PGSIZE_2M) {
        APP_ERR("--bytes must be 1..%u\n", (unsigned)RTE_PGSIZE_2M);
        return 1;
    }

    const struct rte_memzone *mz =
        rte_memzone_reserve_aligned(
            mzname,
            RTE_PGSIZE_2M,
            rte_socket_id(),
            RTE_MEMZONE_2MB | RTE_MEMZONE_IOVA_CONTIG,
            RTE_PGSIZE_2M);

    if (!mz) {
        APP_ERR("memzone reserve failed: %s (rte_errno=%d)\n",
                rte_strerror(rte_errno), rte_errno);
        return 1;
    }

    APP_LOG("memzone '%s': len=%zu VA=%p IOVA=0x%" PRIx64 "\n",
            mz->name, mz->len, mz->addr, (uint64_t)mz->iova);

    int sync_fd = -1;
    if (sync_node) {
        sync_fd = open(sync_node, O_RDWR);
        if (sync_fd < 0)
            APP_ERR("open(%s) failed: %s (continuing without cache sync)\n",
                    sync_node, strerror(errno));
        else
            APP_LOG("cache sync node opened: %s\n", sync_node);
    }

    uint8_t *p = (uint8_t *)mz->addr;

    APP_LOG("CPU write/read test: bytes=%zu pattern=0x%02x\n", bytes, pat);
    memset(p, pat, bytes);
    rte_io_wmb();
    sync_to_dev(sync_fd, p, bytes);
    rte_io_wmb();

    rte_io_rmb();
    if (verify(p, bytes, pat) != 0) {
        APP_ERR("CPU readback FAILED\n");
        return 2;
    }

    APP_LOG("CPU readback OK xor32=0x%08x\n", xor32(p, bytes));

    /* Demonstrate invalidate path */
    sync_from_dev(sync_fd, p, bytes);
    rte_io_rmb();
    if (verify(p, bytes, pat) != 0) {
        APP_ERR("post-invalidate verify FAILED\n");
        return 3;
    }
    APP_LOG("post-invalidate verify OK\n");

    if (sync_fd >= 0) close(sync_fd);
    APP_LOG("DONE\n");
    return 0;
}
