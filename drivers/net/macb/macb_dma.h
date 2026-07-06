#pragma once

#include "macb_hw.h"

/* DMA cache synchronization */
void macb_dma_sync_open_once(struct macb_adapter *ad);
void macb_dma_sync_close(struct macb_adapter *ad);
