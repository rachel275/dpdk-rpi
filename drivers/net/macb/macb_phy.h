#pragma once

#include "macb_hw.h"

/* -------- Clause 22 MDIO frame-format constants -------- */
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

/* -------- Standard MII register addresses -------- */
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
#ifndef MII_LPA
# define MII_LPA        0x05
#endif
#ifndef MII_CTRL1000
# define MII_CTRL1000   0x09
#endif
#ifndef MII_STAT1000
# define MII_STAT1000   0x0A
#endif

/* -------- BMCR bits -------- */
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

/* -------- BMSR bits -------- */
#ifndef BMSR_ANEGCOMPLETE
# define BMSR_ANEGCOMPLETE (1u << 5)
#endif
#ifndef BMSR_LSTATUS
# define BMSR_LSTATUS   (1u << 2)
#endif

/* -------- Autoneg advertise / link-partner ability bits -------- */
#ifndef ADVERTISE_10HALF
# define ADVERTISE_10HALF   0x0020
#endif
#ifndef ADVERTISE_10FULL
# define ADVERTISE_10FULL   0x0040
#endif
#ifndef ADVERTISE_100HALF
# define ADVERTISE_100HALF  0x0080
#endif
#ifndef ADVERTISE_100FULL
# define ADVERTISE_100FULL  0x0100
#endif
#ifndef LPA_1000HALF
# define LPA_1000HALF       0x0400
#endif
#ifndef LPA_1000FULL
# define LPA_1000FULL       0x0800
#endif

/* -------- PHY operating mode (used by devargs and phy bring-up) -------- */
enum macb_phy_mode {
    PHY_MODE_AUTO  = 0,
    PHY_MODE_10HD,
    PHY_MODE_10FD,
    PHY_MODE_100HD,
    PHY_MODE_100FD,
};

/* MDIO operations */
uint16_t macb_mdio_read_c22(struct macb_adapter *ad, uint8_t phy, uint8_t reg);
void macb_mdio_write_c22(struct macb_adapter *ad, uint8_t phy, uint8_t reg, uint16_t val);

/* PHY scanning */
int macb_find_phy_addr(struct macb_adapter *ad, uint8_t *out_phy,
                       uint16_t *out_id1, uint16_t *out_id2);

/* PHY bring-up modes */
int macb_phy_normal_up(struct macb_adapter *ad);
int macb_phy_force_10_100(struct macb_adapter *ad, enum macb_phy_mode mode);

/* PHY loopback control */
int macb_phy_loopback_set(struct macb_adapter *ad, int enable);
void macb_mac_loopback_set(struct macb_adapter *ad, int enable);
void macb_apply_loopback(struct macb_adapter *ad);
