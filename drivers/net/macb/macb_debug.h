#ifndef _RTE_MACB_DEBUG_H_
#define _RTE_MACB_DEBUG_H_

#include <rte_log.h>
#include "macb_hw.h"

/* ---------- Build-time sanity: make sure owner bits match what rxtx expects ---------- */

static inline uint32_t _macb_rd(void *hw, uint32_t reg_off)
{
    return macb_readl(hw, reg_off);
}

static inline void macb_dump_top_regs(struct macb_adapter *ad, const char *tag)
{
    if (!ad) return;
    uint32_t ncr   = _macb_rd(&ad->hw, MACB_NCR);
    uint32_t ncfgr = _macb_rd(&ad->hw, MACB_NCFGR);
    uint32_t nsr   = _macb_rd(&ad->hw, MACB_NSR);
    uint32_t rsr   = _macb_rd(&ad->hw, MACB_RSR);
    uint32_t tsr   = _macb_rd(&ad->hw, MACB_TSR);

    /* Ring bases (classic) */
    uint32_t rbqp = 0, tbqp = 0;
#ifdef MACB_RBQP
    rbqp = _macb_rd(&ad->hw, MACB_RBQP);
#endif
#ifdef MACB_TBQP
    tbqp = _macb_rd(&ad->hw, MACB_TBQP);
#endif

    RTE_LOG(INFO, EAL,
        "macb[%s]: NCR=0x%08x NCFGR=0x%08x NSR=0x%08x RSR=0x%08x TSR=0x%08x RBQP=0x%08x TBQP=0x%08x\n",
        tag ? tag : "regs", ncr, ncfgr, nsr, rsr, tsr, rbqp, tbqp);

#ifdef MACB_RBQB
    uint32_t rbqb_q0 = _macb_rd(&ad->hw, MACB_RBQB);
#else
    uint32_t rbqb_q0 = 0;
#endif
#ifdef MACB_TBQB
    uint32_t tbqb_q0 = _macb_rd(&ad->hw, MACB_TBQB);
#else
    uint32_t tbqb_q0 = 0;
#endif
    if (rbqb_q0 || tbqb_q0) {
        RTE_LOG(INFO, EAL, "macb[%s]: RBQB_Q0=0x%08x TBQB_Q0=0x%08x\n",
            tag ? tag : "regs", rbqb_q0, tbqb_q0);
    }
}

static inline void macb_dump_addr_filters(struct macb_adapter *ad)
{
    if (!ad) return;

    /* Hash filter (optional) */
#ifdef MACB_HRB
    uint32_t hrb = _macb_rd(&ad->hw, MACB_HRB);
    uint32_t hru = _macb_rd(&ad->hw, MACB_HRU);
    RTE_LOG(INFO, EAL, "macb: HRB=0x%08x HRU=0x%08x (hash filter)\n", hrb, hru);
#endif

    /* Specific address 1 (optional naming varies across SoCs) */
#if defined(MACB_SA1B) && defined(MACB_SA1T)
    uint32_t sa1b = _macb_rd(&ad->hw, MACB_SA1B);
    uint32_t sa1t = _macb_rd(&ad->hw, MACB_SA1T);
    RTE_LOG(INFO, EAL, "macb: SA1B=0x%08x SA1T=0x%08x (MAC addr low/high)\n", sa1b, sa1t);
#elif defined(MACB_SA1L) && defined(MACB_SA1H)
    uint32_t sa1l = _macb_rd(&ad->hw, MACB_SA1L);
    uint32_t sa1h = _macb_rd(&ad->hw, MACB_SA1H);
    RTE_LOG(INFO, EAL, "macb: SA1L=0x%08x SA1H=0x%08x (MAC addr low/high)\n", sa1l, sa1h);
#endif

#ifdef MACB_NCFGR
    uint32_t ncfgr = _macb_rd(&ad->hw, MACB_NCFGR);
    RTE_LOG(INFO, EAL, "macb: NCFGR=0x%08x (RX config: speed/duplex/bcast/promisc/etc.)\n", ncfgr);
#endif
}

static inline void macb_dump_status_edges(struct macb_adapter *ad, const char *why)
{
    if (!ad) return;
    uint32_t rsr = _macb_rd(&ad->hw, MACB_RSR);
    uint32_t tsr = _macb_rd(&ad->hw, MACB_TSR);
    uint32_t nsr = _macb_rd(&ad->hw, MACB_NSR);
    RTE_LOG(INFO, EAL, "macb[edge:%s]: RSR=0x%08x TSR=0x%08x NSR=0x%08x\n",
            why ? why : "-", rsr, tsr, nsr);
}

/* Optional: dump a condensed PHY page (uses the adapter's existing MDIO helpers if present) */
static inline void macb_phy_dump(struct macb_adapter *ad)
{
    if (!ad) return;
}
#endif /* _RTE_MACB_DEBUG_H_ */

