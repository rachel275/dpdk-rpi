# MACB Driver Refactoring - Phase 1 Complete

## Summary of Changes

### Phase 1: Module Structure Extraction ✓ COMPLETE

**New modules created:**

1. **macb_phy.c/h** (176 lines total)
   - MDIO Clause 22 read/write operations
   - PHY auto-negotiation and forced mode control
   - PHY discovery/scanning
   - MAC and PHY loopback control
   - Functions: `macb_mdio_read_c22`, `macb_mdio_write_c22`, `macb_phy_normal_up`, `macb_phy_force_10_100`, `macb_phy_loopback_set`, `macb_mac_loopback_set`, `macb_apply_loopback`

2. **macb_dma.c/h** (46 lines total)
   - DMA cache synchronization helper management
   - Opens/closes `/dev/dma_sync_eth0` for non-coherent memory sync
   - Functions: `macb_dma_sync_open_once`, `macb_dma_sync_close`

**Code reduction:**
- macb_ethdev.c: 2806 lines → 2602 lines (-204 lines, 7% reduction)
- Removed 186 lines of extracted code
- Restored 29 lines (macb_map_mempool was accidentally deleted, re-added)

**Updated files:**
- macb_ethdev.c: Added includes for new modules, removed duplicate implementations
- meson.build: Added new source files to build

### Current State

**ethdev.c now focuses on:**
- DPDK ethdev interface implementation
- Queue setup/teardown
- Link state management
- Statistics
- Device configuration

**Still needs work in ethdev.c:**
- `macb_dev_start()` at line 2089: 228+ lines of register programming logic
  - Current: DMA init, ring setup, PHY config, register dumps, selftests
  - Target: ~15 lines calling helper functions
- `macb_dev_stop()`: similar complexity
- PHY-related global variables (could move to adapter struct)
- Extensive debug/audit code (scheduled for Phase 6)

### Next Steps

**Phase 2: Extract Ring Management (Recommended next)**
- Create `macb_ring.c/h`
- Move: Ring allocation, descriptor initialization, RBQP/TBQP programming
- Target functions: Extract from `macb_rx_queue_setup`, `macb_tx_queue_setup`
- Functions to create:
  - `macb_ring_alloc_rx(rxq, nb_desc)`
  - `macb_ring_alloc_tx(txq, nb_desc)`
  - `macb_ring_init_rx(rxq)`
  - `macb_ring_init_tx(txq)`
  - `macb_ring_free(rxq/txq)`

**Phase 3: Extract Hardware Control (macb_hw.c)**
- Move register access sequences
- Move NCR/DMA control logic
- Create: `macb_hw_reset()`, `macb_hw_program_dma()`, `macb_hw_enable()`

**Phase 4: Simplify dev_start/dev_stop**
- Use new helpers to condense to ~30 lines each

**Phase 5: Remove Debug/Audit Code**
- Delete `macb_rx_audit.h` and all audit helpers
- Remove `macb_dump_*` functions
- Remove `macb_canary_*` functions
- Remove `macb_rxq_cpu_rw_test`, DMA selftest code

**Phase 6: Consolidate Logging**
- Keep only state-transition logs
- Remove descriptor-level debugging

### Testing Recommendations

1. Verify compilation with meson:
   ```bash
   cd /Users/adityachoubey/Projects/dpdk-rpi
   meson builddir
   meson compile -C builddir
   ```

2. Test on Raspberry Pi 5:
   - Verify PHY detection still works
   - Verify basic RX/TX still functions
   - Check link state changes

3. Smoke test: Run testpmd if available

### Files to Review

- [macb_phy.c](macb_phy.c) - New PHY module (review MDIO timing)
- [macb_dma.c](macb_dma.c) - New DMA module (check sync fd handling)
- [macb_ethdev.c](macb_ethdev.c) - Updated main file (check includes)

### Technical Notes

- Global variables `g_phy_mode`, `g_force_phy_lb`, `g_mac_lb`, `g_forced_phy_addr` should eventually move to `struct macb_adapter` to support multiple devices
- MDIO register definitions now duplicated between ethdev.c and phy.c (for safety; can consolidate later)
- macb_map_mempool is device-specific and remains in ethdev.c (correct placement)
