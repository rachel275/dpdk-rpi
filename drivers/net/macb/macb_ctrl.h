/*
 * macb_ctrl.h - Hardware control sequences for Cadence GEM driver
 *
 * Encapsulates register programming, status register management,
 * and hardware initialization routines separate from ethdev callbacks.
 */

#ifndef MACB_CTRL_H
#define MACB_CTRL_H

#include <stdint.h>
#include <rte_dev.h>
#include <rte_mempool.h>

struct macb_adapter;
struct macb_rxq;
struct macb_txq;
struct rte_eth_dev;

/*
 * Hardware State Control
 */

/**
 * macb_hw_disable_rxtx - Disable RX and TX in NCR register
 * @ad: Adapter context
 *
 * Clears NCR_RXEN and NCR_TXEN bits. Used before ring reprogramming
 * to ensure DMA is idle.
 */
void macb_hw_disable_rxtx(struct macb_adapter *ad);

/**
 * macb_hw_reset_status_regs - Clear RSR/TSR status registers
 * @ad: Adapter context
 *
 * Writes 0xffffffff to RSR (RX Status) and TSR (TX Status) registers
 * to clear any latched status bits before starting DMA.
 */
void macb_hw_reset_status_regs(struct macb_adapter *ad);

/**
 * macb_hw_reset_tx_ring - Reset all TX descriptors to FREE state
 * @txq: TX queue context
 *
 * Sets all descriptors to TX_USED flag with WRAP on the last descriptor.
 * Syncs changes to device cache. Called during dev_start to ensure
 * TX ring is clean before enabling transmit.
 */
int macb_hw_reset_tx_ring(struct macb_txq *txq);

/**
 * macb_hw_program_ring_ptrs - Program RBQP/TBQP ring address registers
 * @ad: Adapter context
 * @rxq: RX queue context
 * @txq: TX queue context
 *
 * Programs RX and TX ring base pointers into hardware registers,
 * handling both 32-bit and 64-bit address modes. Supports RP1 GEM
 * variants with multiple register locations. Logs readback verification.
 */
int macb_hw_program_ring_ptrs(struct macb_adapter *ad,
                              struct macb_rxq *rxq,
                              struct macb_txq *txq);

/**
 * macb_hw_configure_dma - Program DMA configuration (DMACFG)
 * @ad: Adapter context
 * @rxq: RX queue context
 *
 * Sets RX buffer size in DMACFG register based on mempool data room.
 * Logs configuration details for debugging.
 */
int macb_hw_configure_dma(struct macb_adapter *ad, struct macb_rxq *rxq);

/**
 * macb_hw_configure_mac - Program MAC configuration (NCFGR)
 * @ad: Adapter context
 *
 * Sets promiscuous mode (CAF) and disables broadcast (NBC) handling
 * as per DPDK driver policy.
 */
void macb_hw_configure_mac(struct macb_adapter *ad);

/**
 * macb_hw_probe_dma_cap - Probe DMA capability bits from hardware
 * @ad: Adapter context
 *
 * Detects descriptor/addressing capabilities and returns a compact bitmask
 * used by the ring allocator.
 */
uint8_t macb_hw_probe_dma_cap(struct macb_adapter *ad);

/**
 * macb_hw_discover_rp1_q0_ptr_regs - Find RP1 queue0 pointer registers
 * @ad: Adapter context
 * @fw_rbqp_lo: Current RBQP low register value
 * @fw_tbqp_lo: Current TBQP low register value
 *
 * Scans the MMIO window for the queue0 pointer registers used by RP1.
 */
void macb_hw_discover_rp1_q0_ptr_regs(struct macb_adapter *ad,
                                      uint32_t fw_rbqp_lo,
                                      uint32_t fw_tbqp_lo);

/**
 * macb_hw_enable_rxtx - Enable RX and TX in NCR register
 * @ad: Adapter context
 *
 * Sets NCR_RXEN, NCR_TXEN, and NCR_MPE (Management Port Enable).
 * Issues TSTART kick. Logs NCR readback for verification.
 */
void macb_hw_enable_rxtx(struct macb_adapter *ad);

/**
 * macb_map_mempool - DMA-map all mempool memory segments
 * @rdev: DPDK device (for rte_dev_dma_map)
 * @mp: mempool whose segments to map
 */
int macb_map_mempool(struct rte_device *rdev, struct rte_mempool *mp);

#endif /* MACB_CTRL_H */
