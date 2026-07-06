#pragma once

#include <stdint.h>
#include <rte_mempool.h>
#include "macb_hw.h"

/* Full struct rte_eth_dev definition (needed for ->data->dev_private etc.) */
#include <ethdev_driver.h>

/* Ring allocation - allocates DMA memory and software ring buffers */
int macb_ring_alloc_rx(struct rte_eth_dev *dev, uint16_t qid,
                       uint16_t nb_desc, struct rte_mempool *mp);

int macb_ring_alloc_tx(struct rte_eth_dev *dev, uint16_t qid, uint16_t nb_desc);

/* Ring initialization - sets up descriptor layout */
int macb_ring_init_rx(struct macb_rxq *rxq);
int macb_ring_init_tx(struct macb_txq *txq);

/* Ring cleanup */
void macb_ring_free_rx(struct macb_rxq *rxq);
void macb_ring_free_tx(struct macb_txq *txq);
