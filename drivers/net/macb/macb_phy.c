// macb_phy.c — PHY/MDIO control for Cadence GEM

#include <errno.h>
#include <stdint.h>
#include <rte_cycles.h>
#include <rte_io.h>
#include <rte_log.h>

#include "macb_hw.h"
#include "macb_phy.h"

#ifndef RTE_LOGTYPE_PMD
#define RTE_LOGTYPE_PMD RTE_LOGTYPE_USER1
#endif

#define MACB_DBG(fmt, ...) RTE_LOG(INFO,  PMD, "macb: " fmt, ##__VA_ARGS__)
#define MACB_ERR(fmt, ...) RTE_LOG(ERR,   PMD, "macb: " fmt, ##__VA_ARGS__)

/* Clause 22 MDIO register address fields (from macb_hw.h patterns) */
#ifndef MACB_MAN_SOF_SHIFT
# define MACB_MAN_SOF_SHIFT  30
#endif
#ifndef MACB_MAN_OP_SHIFT
# define MACB_MAN_OP_SHIFT   28
#endif
#ifndef MACB_MAN_PHYA_SHIFT
# define MACB_MAN_PHYA_SHIFT 23
#endif
#ifndef MACB_MAN_REGA_SHIFT
# define MACB_MAN_REGA_SHIFT 18
#endif
#ifndef MACB_MAN_CODE_SHIFT
# define MACB_MAN_CODE_SHIFT 16
#endif
#ifndef MACB_MAN_OP_WRITE
# define MACB_MAN_OP_WRITE   (1u)
#endif
#ifndef MACB_MAN_OP_READ
# define MACB_MAN_OP_READ    (2u)
#endif
#ifndef MACB_MAN_CODE_C22
# define MACB_MAN_CODE_C22   (2u)
#endif

/* MII register addresses */
#ifndef MII_BMCR
# define MII_BMCR       0x00
#endif
#ifndef MII_BMSR
# define MII_BMSR       0x01
#endif
#ifndef MII_PHYSID1
# define MII_PHYSID1    0x02
#endif
#ifndef MII_PHYSID2
# define MII_PHYSID2    0x03
#endif
#ifndef MII_ADVERTISE
# define MII_ADVERTISE  0x04
#endif
#ifndef MII_STAT1000
# define MII_STAT1000   0x0A
#endif

/* BMCR bits */
#ifndef BMCR_RESET
# define BMCR_RESET     (1u << 15)
#endif
#ifndef BMCR_LOOPBACK
# define BMCR_LOOPBACK  (1u << 14)
#endif
#ifndef BMCR_SPEED100
# define BMCR_SPEED100  (1u << 13)
#endif
#ifndef BMCR_ANENABLE
# define BMCR_ANENABLE  (1u << 12)
#endif
#ifndef BMCR_PDOWN
# define BMCR_PDOWN     (1u << 11)
#endif
#ifndef BMCR_ISOLATE
# define BMCR_ISOLATE   (1u << 10)
#endif
#ifndef BMCR_RESTARTAN
# define BMCR_RESTARTAN (1u << 9)
#endif
#ifndef BMCR_FULLDPLX
# define BMCR_FULLDPLX  (1u << 8)
#endif

/* BMSR bits */
#ifndef BMSR_LSTATUS
# define BMSR_LSTATUS   (1u << 2)
#endif
#ifndef BMSR_ANEGCOMPLETE
# define BMSR_ANEGCOMPLETE (1u << 5)
#endif

/* ADVERTISE bits */
#ifndef ADVERTISE_10HALF
# define ADVERTISE_10HALF    (1u << 5)
#endif
#ifndef ADVERTISE_10FULL
# define ADVERTISE_10FULL    (1u << 6)
#endif
#ifndef ADVERTISE_100HALF
# define ADVERTISE_100HALF   (1u << 7)
#endif
#ifndef ADVERTISE_100FULL
# define ADVERTISE_100FULL   (1u << 8)
#endif

/* Global PHY mode and loopback settings (TODO: move to adapter struct) */
static enum macb_phy_mode g_phy_mode = PHY_MODE_AUTO;
static int g_force_phy_lb = 0;
static int g_mac_lb = 0;
static int g_forced_phy_addr = -1;

/* ============== MDIO helpers ============== */

static inline void macb_wait_mdio_idle(struct macb_adapter *ad)
{
    for (int i = 0; i < 1000; i++) {
        if (macb_readl(&ad->hw, MACB_NSR) & 0x4u) return;
        rte_delay_us_block(5);
    }
    MACB_ERR("MDIO idle wait timeout\n");
}

uint16_t macb_mdio_read_c22(struct macb_adapter *ad, uint8_t phy, uint8_t reg)
{
    macb_wait_mdio_idle(ad);
    uint32_t man = ((1u << MACB_MAN_SOF_SHIFT) |
                    (MACB_MAN_OP_READ  << MACB_MAN_OP_SHIFT) |
                    ((uint32_t)phy << MACB_MAN_PHYA_SHIFT)   |
                    ((uint32_t)reg << MACB_MAN_REGA_SHIFT)   |
                    (MACB_MAN_CODE_C22 << MACB_MAN_CODE_SHIFT));
    macb_writel(&ad->hw, MACB_MAN, man);
    macb_wait_mdio_idle(ad);
    return (uint16_t)macb_readl(&ad->hw, MACB_MAN);
}

void macb_mdio_write_c22(struct macb_adapter *ad, uint8_t phy, uint8_t reg, uint16_t val)
{
    macb_wait_mdio_idle(ad);
    uint32_t man = ((1u << MACB_MAN_SOF_SHIFT) |
                    (MACB_MAN_OP_WRITE << MACB_MAN_OP_SHIFT) |
                    ((uint32_t)phy << MACB_MAN_PHYA_SHIFT)   |
                    ((uint32_t)reg << MACB_MAN_REGA_SHIFT)   |
                    (MACB_MAN_CODE_C22 << MACB_MAN_CODE_SHIFT) |
                    (uint32_t)val);
    macb_writel(&ad->hw, MACB_MAN, man);
    macb_wait_mdio_idle(ad);
}

/* ============== PHY scanning ============== */

int macb_find_phy_addr(struct macb_adapter *ad, uint8_t *out_phy,
                       uint16_t *out_id1, uint16_t *out_id2)
{
    if (g_forced_phy_addr >= 0 && g_forced_phy_addr < 32) {
        uint8_t p = (uint8_t)g_forced_phy_addr;
        uint16_t id1 = macb_mdio_read_c22(ad, p, MII_PHYSID1);
        uint16_t id2 = macb_mdio_read_c22(ad, p, MII_PHYSID2);
        if (out_phy) *out_phy = p;
        if (out_id1) *out_id1 = id1;
        if (out_id2) *out_id2 = id2;
        return 0;
    }
    for (uint8_t phy = 0; phy < 32; phy++) {
        uint16_t id1 = macb_mdio_read_c22(ad, phy, MII_PHYSID1);
        uint16_t id2 = macb_mdio_read_c22(ad, phy, MII_PHYSID2);
        if (id1 != 0xffff && id1 != 0x0000) {
            if (out_phy) *out_phy = phy;
            if (out_id1) *out_id1 = id1;
            if (out_id2) *out_id2 = id2;
            return 0;
        }
    }
    return -1;
}

/* ============== PHY bring-up modes ============== */

int macb_phy_normal_up(struct macb_adapter *ad)
{
    uint8_t phy; uint16_t id1=0, id2=0;
    if (macb_find_phy_addr(ad, &phy, &id1, &id2) != 0) return -1;

    uint16_t bmcr = macb_mdio_read_c22(ad, phy, MII_BMCR);
    uint16_t adv  = macb_mdio_read_c22(ad, phy, MII_ADVERTISE);
    adv |= (ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL);
    macb_mdio_write_c22(ad, phy, MII_ADVERTISE, adv);

    bmcr &= ~(BMCR_PDOWN | BMCR_ISOLATE | BMCR_LOOPBACK | BMCR_SPEED100 | BMCR_FULLDPLX);
    bmcr |=  (BMCR_ANENABLE | BMCR_RESTARTAN);
    macb_mdio_write_c22(ad, phy, MII_BMCR, bmcr);

    for (int i = 0; i < 200; i++) {
        uint16_t bmsr = macb_mdio_read_c22(ad, phy, MII_BMSR);
        if ((bmsr & BMSR_LSTATUS) && (bmsr & BMSR_ANEGCOMPLETE)) break;
        rte_delay_us_block(5000);
    }
    return 0;
}

int macb_phy_force_10_100(struct macb_adapter *ad, enum macb_phy_mode mode)
{
    uint8_t phy; uint16_t id1=0, id2=0;
    if (macb_find_phy_addr(ad, &phy, &id1, &id2) != 0) return -1;

    uint16_t bmcr = macb_mdio_read_c22(ad, phy, MII_BMCR);
    uint16_t newb = bmcr;

    newb &= ~(BMCR_ANENABLE | BMCR_PDOWN | BMCR_ISOLATE | BMCR_LOOPBACK | BMCR_RESTARTAN);
    newb &= ~(BMCR_SPEED100 | BMCR_FULLDPLX);

    switch (mode) {
    case PHY_MODE_10HD:    break;
    case PHY_MODE_10FD:    newb |= BMCR_FULLDPLX; break;
    case PHY_MODE_100HD:   newb |= BMCR_SPEED100; break;
    case PHY_MODE_100FD:   newb |= (BMCR_SPEED100 | BMCR_FULLDPLX); break;
    default: return -EINVAL;
    }

    if (newb != bmcr) macb_mdio_write_c22(ad, phy, MII_BMCR, newb);

    for (int i = 0; i < 200; i++) {
        uint16_t bmsr = macb_mdio_read_c22(ad, phy, MII_BMSR);
        if (bmsr & BMSR_LSTATUS) break;
        rte_delay_us_block(5000);
    }
    return 0;
}

/* ============== PHY loopback control ============== */

int macb_phy_loopback_set(struct macb_adapter *ad, int enable)
{
    uint8_t phy; uint16_t id1=0,id2=0;
    if (macb_find_phy_addr(ad, &phy, &id1, &id2) != 0)
        return -1;

    uint16_t bmcr = macb_mdio_read_c22(ad, phy, MII_BMCR);
    uint16_t newbmcr = bmcr;

    if (enable) {
        newbmcr &= ~(BMCR_ANENABLE | BMCR_PDOWN | BMCR_ISOLATE);
        newbmcr |= BMCR_SPEED100 | BMCR_FULLDPLX | BMCR_LOOPBACK;
    } else {
        newbmcr &= ~BMCR_LOOPBACK;
        if (g_phy_mode == PHY_MODE_AUTO) {
            newbmcr |= (BMCR_ANENABLE | BMCR_RESTARTAN);
        }
    }

    if (newbmcr != bmcr)
        macb_mdio_write_c22(ad, phy, MII_BMCR, newbmcr);

    return 0;
}

#ifndef NCFGR_LBL
#define NCFGR_LBL 0
#endif

void macb_mac_loopback_set(struct macb_adapter *ad, int enable)
{
    uint32_t ncfgr0 = macb_readl(&ad->hw, MACB_NCFGR);
    uint32_t ncr0   = macb_readl(&ad->hw, MACB_NCR);

    uint32_t ncfgr  = ncfgr0;
    uint32_t ncr    = ncr0;

#ifdef NCFGR_CAF
    if (enable) ncfgr |= NCFGR_CAF;
    else        ncfgr &= ~NCFGR_CAF;
#endif

#ifdef NCR_LB
    if (enable) ncr |= NCR_LB; else ncr &= ~NCR_LB;
#endif

#if NCFGR_LBL
    if (enable) ncfgr |= NCFGR_LBL; else ncfgr &= ~NCFGR_LBL;
#endif

    macb_writel(&ad->hw, MACB_NCFGR, ncfgr);
    ncr |= (NCR_RXEN | NCR_TXEN);
    macb_writel(&ad->hw, MACB_NCR,   ncr);
}

void macb_apply_loopback(struct macb_adapter *ad)
{
    (void)macb_phy_loopback_set(ad, g_force_phy_lb ? 1 : 0);
    macb_mac_loopback_set(ad, g_mac_lb ? 1 : 0);
}
