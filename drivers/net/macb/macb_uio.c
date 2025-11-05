// drivers/net/macb/macb_uio.c
#include <stdlib.h>
#include <rte_log.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include "macb_hw.h"

RTE_LOG_REGISTER_DEFAULT(macb_logtype, NOTICE);

/* --------- Guarded register offsets so we can sanity-read after mmap --------- */
#ifndef MACB_NCR
# define MACB_NCR   0x0000
#endif
#ifndef MACB_NCFGR
# define MACB_NCFGR 0x0004
#endif
#ifndef MACB_NSR
# define MACB_NSR   0x0008
#endif
#ifndef MACB_RBQP
# define MACB_RBQP  0x0018
#endif
#ifndef MACB_TBQP
# define MACB_TBQP  0x001c
#endif

/* --- small sysfs helpers ---------------------------------------------------- */

static int read_sysfs_str(const char *path, char *out, size_t out_sz)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        RTE_LOG(ERR, EAL, "UIO: cannot open %s: %s\n", path, strerror(errno));
        return -1;
    }
    ssize_t n = read(fd, out, (out_sz > 0 ? (ssize_t)out_sz - 1 : 0));
    close(fd);
    if (n <= 0) {
        RTE_LOG(ERR, EAL, "UIO: cannot read %s: %s\n", path, strerror(errno));
        return -1;
    }
    out[n] = '\0';
    /* strip trailing newline if present */
    char *nl = strchr(out, '\n');
    if (nl) *nl = '\0';
    return 0;
}

static int read_sysfs_hex_ull(const char *path, unsigned long long *val)
{
    char buf[64] = {0};
    if (read_sysfs_str(path, buf, sizeof(buf)) < 0) return -1;
    errno = 0;
    unsigned long long x = strtoull(buf, NULL, 0);
    if (errno || x == 0ULL) {
        RTE_LOG(ERR, EAL, "UIO: bad hex in %s: '%s'\n", path, buf);
        return -1;
    }
    *val = x;
    return 0;
}

static int read_uio_meta(const char *uio_basename,
                         unsigned long long *map0_size,
                         unsigned long long *map0_addr,
                         char *uio_name, size_t name_sz)
{
    char path[256];

    if (map0_size) {
        snprintf(path, sizeof(path), "/sys/class/uio/%s/maps/map0/size", uio_basename);
        if (read_sysfs_hex_ull(path, map0_size) < 0) return -1;
    }
    if (map0_addr) {
        snprintf(path, sizeof(path), "/sys/class/uio/%s/maps/map0/addr", uio_basename);
        if (read_sysfs_hex_ull(path, map0_addr) < 0) return -1;
    }
    if (uio_name && name_sz) {
        snprintf(path, sizeof(path), "/sys/class/uio/%s/name", uio_basename);
        (void)read_sysfs_str(path, uio_name, name_sz); /* name is best-effort */
    }
    return 0;
}

/* --- public API ------------------------------------------------------------- */

int macb_uio_map(struct macb_hw *hw, const char *uio_path)
{
    if (!uio_path || !*uio_path) {
        RTE_LOG(ERR, EAL, "UIO: null path\n");
        return -EINVAL;
    }

    /* Accept "uio0" or "/dev/uio0" */
    const char *bn = uio_path;
    const char *slash = strrchr(uio_path, '/');
    if (slash) bn = slash + 1;

    unsigned long long map_sz = 0, map_addr = 0;
    char uio_name[128] = {0};

    if (read_uio_meta(bn, &map_sz, &map_addr, uio_name, sizeof(uio_name)) < 0) {
        RTE_LOG(ERR, EAL, "UIO: failed reading sysfs metadata for %s\n", bn);
        return -ENODEV;
    }

    /* Resolve device path to open */
    char devpath[64];
    const char *openpath = uio_path;
    if (uio_path[0] != '/') {
        snprintf(devpath, sizeof(devpath), "/dev/%s", bn);
        openpath = devpath;
    }

    int fd = open(openpath, O_RDWR | O_SYNC);
    if (fd < 0) {
        RTE_LOG(ERR, EAL, "UIO: open %s failed: %s\n", openpath, strerror(errno));
        return -errno;
    }

    void *map = mmap(NULL, (size_t)map_sz, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED) {
        int err = errno;
        RTE_LOG(ERR, EAL, "UIO: mmap(%s, 0x%llx) failed: %s\n", bn, map_sz, strerror(err));
        close(fd);
        return -err;
    }

    hw->regs     = (volatile uint8_t *)map;
    hw->regs_len = (size_t)map_sz;
    hw->uio_fd   = fd; /* keep open for the lifetime of the driver */

    RTE_LOG(INFO, EAL,
            "UIO: mapped %s (%s) phys=0x%llx size=0x%llx at %p\n",
            bn, (uio_name[0] ? uio_name : "?"),
            map_addr, map_sz, (void*)hw->regs);

    /* --------- quick sanity snapshot of a few top regs --------- */
    /* If macb_readl/ writel are macros, these will compile fine. */
    uint32_t ncr   = 0, ncfgr = 0, nsr = 0, rbqp = 0, tbqp = 0;
    int ok = 1;

    /* Best-effort: try/assume the device is present; if this faults on some
     * platforms it will simply read garbage (UIO mapping is userspace-safe). */
    ncr   = macb_readl(hw, MACB_NCR);
    ncfgr = macb_readl(hw, MACB_NCFGR);
    nsr   = macb_readl(hw, MACB_NSR);

    /* Optional ring base readbacks (helpful when debugging why RX is idle) */
    rbqp  = macb_readl(hw, MACB_RBQP);
    tbqp  = macb_readl(hw, MACB_TBQP);

    if (ncfgr == 0xFFFFFFFFu || ncr == 0xFFFFFFFFu) ok = 0;   /* unmapped/bad bus */
    if (ncfgr == 0x00000000u && ncr == 0x00000000u)  ok = 0;   /* suspicious too */

    if (!ok) {
        RTE_LOG(WARNING, EAL,
                "UIO: suspicious register reads (NCR=0x%08x NCFGR=0x%08x NSR=0x%08x). "
                "Check UIO mapping and device power/reset.\n", ncr, ncfgr, nsr);
    } else {
        RTE_LOG(INFO, EAL,
                "UIO: initial regs NCR=0x%08x NCFGR=0x%08x NSR=0x%08x RBQP=0x%08x TBQP=0x%08x\n",
                ncr, ncfgr, nsr, rbqp, tbqp);
    }

    return 0;
}

void macb_uio_unmap(struct macb_hw *hw)
{
    if (hw->regs && hw->regs_len)
        munmap((void *)(uintptr_t)hw->regs, hw->regs_len);
    if (hw->uio_fd >= 0)
        close(hw->uio_fd);
    hw->regs = NULL;
    hw->regs_len = 0;
    hw->uio_fd = -1;
}

