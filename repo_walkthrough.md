# DPDK MACB/GEM PMD for Raspberry Pi 5 (RP1) – Developer Documentation

## Overview

This repository implements a custom DPDK Poll Mode Driver (PMD) for the Raspberry Pi 5 RP1 Ethernet controller, which is based on the Cadence MACB/GEM Ethernet MAC.

The driver operates in userspace using DPDK and UIO, bypassing the Linux networking stack and directly controlling:

- MACB/GEM registers
- DMA descriptor rings
- RX/TX packet processing
- PHY management via MDIO
- Link configuration and loopback modes
- Optional DMA cache synchronization hooks

The PMD is exposed as a DPDK Virtual Device (vdev).

Example launch:

```bash
--vdev=net_macb0,dev=/dev/uio0,phy_addr=1,phy_lb=0,mac_lb=0,bus_ofs=0,phy_mode=auto
```

---

## Acronyms and Terminology

| Acronym | Meaning | Description |
|---|---|---|
| API | Application Programming Interface | Interface exposed between software components. |
| BMCR | Basic Mode Control Register | PHY register used to control speed, duplex, loopback, and autonegotiation. |
| BMSR | Basic Mode Status Register | PHY register reporting link and negotiation status. |
| CAF | Copy All Frames | MACB/GEM configuration bit commonly used for promiscuous receive behaviour. |
| DMA | Direct Memory Access | Hardware transfer of data directly between memory and a peripheral. |
| DPDK | Data Plane Development Kit | High-performance packet processing framework that bypasses the kernel networking stack. |
| EOF | End Of Frame | Descriptor flag marking the final buffer of a received frame. |
| FD | Full Duplex | Ethernet mode where transmit and receive can happen simultaneously. |
| GEM | Gigabit Ethernet MAC | Cadence Ethernet MAC variant used in many SoCs, including the RP1 Ethernet block. |
| GMII | Gigabit Media Independent Interface | Interface between an Ethernet MAC and PHY for gigabit-capable links. |
| GPIO | General Purpose Input/Output | Generic hardware control pin. |
| HD | Half Duplex | Ethernet mode where transmit and receive share the medium. |
| IOVA | I/O Virtual Address | Address presented to hardware for DMA. |
| IRQ | Interrupt Request | Hardware event notification. DPDK PMDs usually poll rather than rely heavily on interrupts. |
| KVARGS | Key-Value Arguments | DPDK mechanism for passing arguments to virtual devices. |
| LB | Loopback | Test mode where traffic is routed back internally. |
| MAC | Media Access Control | Ethernet controller responsible for frame transmission and reception. |
| MACB | Media Access Controller B | Cadence Ethernet MAC family used by many ARM SoCs. |
| MBUF | Memory Buffer | DPDK packet buffer structure, usually `struct rte_mbuf`. |
| MDIO | Management Data Input/Output | Serial management bus used to communicate with Ethernet PHYs. |
| MII | Media Independent Interface | Ethernet MAC/PHY interface family and PHY register convention. |
| MMIO | Memory-Mapped I/O | Hardware registers accessed through memory addresses. |
| NCR | Network Control Register | MACB/GEM register controlling core MAC behaviour such as RX/TX enable. |
| NCFGR | Network Configuration Register | MACB/GEM register controlling receive filtering, speed, duplex, and related options. |
| NIC | Network Interface Controller | Ethernet network device. |
| NSR | Network Status Register | MACB/GEM status register. |
| PHY | Physical Layer Device | Ethernet transceiver connected to the MAC. |
| PMD | Poll Mode Driver | DPDK userspace driver model. |
| RBQP | Receive Buffer Queue Pointer | Register containing the RX descriptor ring base address. |
| RBQPH | Receive Buffer Queue Pointer High | Upper 32 bits of the RX descriptor ring base address. |
| RSR | Receive Status Register | MACB/GEM receive status register. |
| RP1 | Raspberry Pi I/O Controller | Southbridge-style I/O controller used on Raspberry Pi 5. |
| RX | Receive | Packet ingress path. |
| SOF | Start Of Frame | Descriptor flag marking the first buffer of a received frame. |
| TBQP | Transmit Buffer Queue Pointer | Register containing the TX descriptor ring base address. |
| TBQPH | Transmit Buffer Queue Pointer High | Upper 32 bits of the TX descriptor ring base address. |
| TSR | Transmit Status Register | MACB/GEM transmit status register. |
| TX | Transmit | Packet egress path. |
| UAPI | Userspace API | Header/API boundary shared between kernel-facing and userspace code. |
| UIO | Userspace I/O | Linux framework allowing userspace drivers to map and access device registers. |
| VDEV | Virtual Device | DPDK software-created device instance. |
| WRAP | Ring Wrap Bit | Descriptor flag indicating the final descriptor in a circular ring. |

---

## Descriptor Ownership Terminology

The MACB/GEM hardware and software exchange ownership of descriptors using ownership bits. This is one of the most important concepts in the driver.

### RX Descriptor Ownership

| State | RX ownership bit | Owner | Meaning |
|---|---:|---|---|
| Armed | `0` | Hardware | Hardware may DMA a received frame into the buffer. |
| Packet received | `1` | Software | Hardware has completed the descriptor and software may consume it. |

RX lifecycle:

```text
Software allocates mbuf
        ↓
Software arms RX descriptor
        ↓
Hardware owns descriptor
        ↓
Frame arrives
        ↓
Hardware writes packet data
        ↓
Hardware marks descriptor as software-owned
        ↓
Driver consumes packet
        ↓
Driver rearms descriptor
```

### TX Descriptor Ownership

| State | TX ownership bit | Owner | Meaning |
|---|---:|---|---|
| Free | `1` | Software | Descriptor is available for a new packet. |
| Busy | `0` | Hardware | Hardware owns the descriptor and may transmit it. |

TX lifecycle:

```text
Software owns TX descriptor
        ↓
Application submits packet
        ↓
Driver writes descriptor address and length
        ↓
Driver clears TX_USED
        ↓
Hardware transmits packet
        ↓
Hardware sets TX_USED
        ↓
Driver frees mbuf and reclaims descriptor
```

> Note: Several files contain compatibility shims and comments around ownership-bit placement because MACB/GEM descriptor layouts vary by hardware generation. This is an area that needs particular care when porting the driver.

---

## Repository Architecture

```text
DPDK Application
        │
        ▼
+-------------------+
|  macb_ethdev.c    |
|  DPDK PMD Layer   |
+-------------------+
        │
        ├─────────────┐
        ▼             ▼
+-------------+  +------------+
| macb_rxtx.c |  | PHY / MDIO |
+-------------+  +------------+
        │
        ▼
+-------------------+
| Descriptor Rings  |
+-------------------+
        │
        ▼
+-------------------+
| RP1 GEM Hardware  |
+-------------------+
        │
        ▼
   Ethernet PHY
```

---

# File Breakdown

---

## `dma_sync_uapi.h`

### Purpose

Defines the userspace-facing API for DMA cache synchronization.

This file mirrors the small UAPI used by an optional `dma_sync_helper` character device. It allows userspace code to request cache maintenance for a region of memory before or after DMA.

### Key Structure

```c
struct dma_sync_range {
    uint64_t uaddr;
    uint64_t len;
};
```

Fields:

- `uaddr`: userspace virtual address of the buffer to synchronize
- `len`: length of the region in bytes

### IOCTL Commands

```c
DMA_SYNC_TO_DEV
DMA_SYNC_FROM_DEV
```

Meaning:

- `DMA_SYNC_TO_DEV`: prepare memory before hardware reads it, usually used before TX
- `DMA_SYNC_FROM_DEV`: prepare memory after hardware writes it, usually used after RX

### Role in the Driver

The PMD uses these definitions when calling into the optional DMA sync helper. If the helper is unavailable, the driver can continue, but may warn that TX/RX could stall on non-coherent systems.

---

## `dma_sync_user.h`

### Purpose

Provides a lightweight userspace shim for DMA synchronization.

This lets the rest of the PMD call a consistent set of functions without needing to know whether a real cache-maintenance helper exists.

### Main Functions

```c
int dma_sync_open(const char *ifname);
void dma_sync_to_dev(int fd, const void *addr, size_t len);
void dma_sync_from_dev(int fd, const void *addr, size_t len);
```

### Behaviour

`dma_sync_open()` attempts to open a helper node such as:

```text
/dev/dma_sync_eth0
```

The sync functions are currently no-ops:

```c
dma_sync_to_dev(...)
dma_sync_from_dev(...)
```

This is acceptable on coherent platforms. On non-coherent platforms, these should be replaced with real ioctl or mmap-based cache maintenance logic.

### Design Significance

This file keeps cache coherency concerns isolated from the RX/TX fast path. The PMD can be built and tested even when the helper device is not present.

---

## `macb_hw.h`

### Purpose

Primary hardware abstraction header for the MACB/GEM PMD.

It defines:

- register offsets
- bit masks
- descriptor layout
- RX/TX queue structures
- hardware mapping structures
- software statistics structures

### Important Register Offsets

Examples include:

```c
MACB_NCR
MACB_NCFGR
MACB_TSR
MACB_RBQP
MACB_TBQP
GEM_RBQB_Q0
GEM_TBQB_Q0
GEM_DMACFG
```

These map to MACB/GEM hardware registers.

### Register Groups

#### Control and Configuration

- `MACB_NCR`: Network Control Register
- `MACB_NCFGR`: Network Configuration Register
- `MACB_NSR`: Network Status Register

#### Ring Base Registers

- `MACB_RBQP`: RX descriptor ring base pointer
- `MACB_TBQP`: TX descriptor ring base pointer
- `GEM_RBQP(q)`: queue-indexed RX ring base
- `GEM_TBQP(q)`: queue-indexed TX ring base

#### DMA Configuration

- `GEM_DMACFG`
- `GEM_DMACFG_RXBS_SHIFT`
- `GEM_DMACFG_RXBS_MASK`

These are used when configuring RX buffer sizing and DMA behaviour.

### Descriptor Definition

```c
struct macb_desc {
    volatile uint32_t addr;
    volatile uint32_t ctrl;
} __attribute__((__packed__));
```

Each descriptor contains:

- `addr`: DMA address plus some ownership/control bits
- `ctrl`: length and status/control flags

### RX Queue Structure

```c
struct macb_rxq {
    struct macb_desc *ring;
    struct rte_mbuf **sw_ring;
    uint16_t prod, cons, nb_desc;
    struct rte_mempool *mp;
    const struct rte_memzone *mz;
    rte_iova_t ring_iova;
    int sync_fd;
    struct macb_adapter *ad;
    uint16_t port_id;
    uint16_t data_room_bytes;
};
```

This tracks both the hardware descriptor ring and the software mbuf array used by DPDK.

### TX Queue Structure

The TX queue mirrors the RX queue conceptually:

- descriptor ring
- software mbuf tracking
- producer and consumer indexes
- descriptor count
- adapter pointer
- DMA synchronization file descriptor

### Hardware Mapping

```c
struct macb_hw {
    volatile uint8_t *regs;
    int uio_fd;
    size_t regs_len;
};
```

This stores the MMIO region mapped through UIO.

### Software Statistics

```c
struct macb_sw_stats {
    uint64_t rx_pkts, rx_bytes, rx_errs;
    uint64_t tx_pkts, tx_bytes, tx_errs;
};
```

These counters are updated by the RX/TX burst paths and exposed through DPDK ethdev statistics.

---

## `macb_uio.c`

### Purpose

Handles userspace mapping of the MACB/GEM hardware registers through Linux UIO.

The PMD needs direct access to device registers. UIO provides a controlled way to expose those registers to userspace.

### Startup Sequence

1. Accept a UIO path such as `uio0` or `/dev/uio0`.
2. Read sysfs metadata.
3. Open the UIO device node.
4. `mmap()` the register region.
5. Store the mapping in `struct macb_hw`.
6. Keep the UIO file descriptor open for the lifetime of the driver.

### Sysfs Metadata

The code reads paths like:

```text
/sys/class/uio/uio0/maps/map0/size
/sys/class/uio/uio0/maps/map0/addr
/sys/class/uio/uio0/name
```

These provide:

- MMIO size
- physical address
- UIO device name

### Key Function

```c
int macb_uio_map(struct macb_hw *hw, const char *uio_path);
```

This is the main mapping function called by the PMD.

### Error Handling

The file contains detailed logging for:

- missing UIO path
- failed sysfs reads
- failed `/dev/uioX` open
- failed `mmap()`

### Why This Matters

Without this file, the PMD could not read or write hardware registers from userspace. It is the bridge between DPDK and the RP1 Ethernet hardware.

---

## `macb_ethdev.c`

### Purpose

Main DPDK Ethernet device implementation.

This file integrates the MACB/GEM hardware with DPDK's ethdev API.

It handles:

- virtual device probing
- runtime argument parsing
- register programming
- RX/TX queue setup
- device start and stop
- PHY configuration
- MDIO access
- link reporting
- statistics
- promiscuous mode
- loopback modes
- DMA synchronization setup

### Launch Arguments

The file supports vdev-style arguments such as:

```bash
--vdev=net_macb0,dev=/dev/uio0,phy_addr=1,phy_lb=0,mac_lb=0,bus_ofs=0,phy_mode=auto
```

Important parameters:

| Parameter | Meaning |
|---|---|
| `dev` | UIO device path, for example `/dev/uio0`. |
| `phy_addr` | Fixed PHY address, or auto-scan if unset. |
| `phy_lb` | Enable or disable PHY loopback. |
| `mac_lb` | Enable or disable MAC loopback. |
| `bus_ofs` | CPU physical to device bus address offset. |
| `phy_mode` | PHY mode: `auto`, `10hd`, `10fd`, `100hd`, `100fd`. |

### Register Access Helpers

The driver uses helpers such as:

```c
macb_readl(...)
macb_writel(...)
```

These access the MMIO mapping created by `macb_uio.c`.

### PHY and MDIO Management

The driver implements Clause 22 MDIO transactions.

Important functions:

```c
macb_wait_mdio_idle()
macb_mdio_read_c22()
macb_mdio_write_c22()
macb_find_phy_addr()
```

#### PHY Discovery

`macb_find_phy_addr()` scans PHY addresses `0..31`, reading `MII_PHYSID1` and `MII_PHYSID2` to identify a valid PHY.

#### Autonegotiation

`macb_phy_normal_up()` advertises common 10/100 modes and restarts autonegotiation.

Advertised modes include:

- 10 half-duplex
- 10 full-duplex
- 100 half-duplex
- 100 full-duplex

#### Forced PHY Modes

`macb_phy_force_10_100()` disables autonegotiation and configures BMCR for a specific speed/duplex mode.

Supported forced modes:

- `10hd`
- `10fd`
- `100hd`
- `100fd`

### Loopback Support

The driver supports both PHY and MAC loopback.

#### PHY Loopback

PHY loopback is configured through BMCR and loops traffic inside the PHY.

Useful for validating:

- TX descriptor programming
- RX descriptor completion
- DMA buffer movement
- basic packet path without requiring external traffic

#### MAC Loopback

MAC loopback uses MACB/GEM register bits such as `NCR_LB` and optional NCFGR loopback bits.

Useful for testing MAC-level packet movement independent of the external PHY.

### DMA Ring Programming

The driver programs descriptor ring base registers:

```c
MACB_RBQP
MACB_TBQP
MACB_RBQPH
MACB_TBQPH
```

The high 32-bit registers are important for 64-bit DMA addressing on RP1/GEM variants.

### TX Kick

The driver uses:

```c
NCR_TSTART
```

After preparing TX descriptors, this tells the hardware to start transmission when idle.

### DMA Synchronization Integration

The file attempts to open a helper such as:

```text
/dev/dma_sync_eth0
```

or the node provided by:

```text
MACB_DMA_SYNC_NODE
```

It then calls ioctl-based synchronization helpers around descriptors and buffers when the helper exists.

### Statistics

The file connects DPDK stats callbacks to software counters maintained in the adapter.

Tracked values include:

- RX packets
- RX bytes
- RX errors
- TX packets
- TX bytes
- TX errors

### Development Notes

This file is the largest and most complex part of the project. It combines DPDK ethdev integration with low-level hardware bring-up logic.

When debugging, start here for:

- vdev argument issues
- link not coming up
- PHY not detected
- UIO mapping failures
- rings not being programmed
- TX not starting

---

## `macb_ethdev.bak.2.c`

### Purpose

Backup or earlier version of the main ethdev implementation.

This file appears to represent an earlier driver revision focused on bring-up fixes including:

- 64-bit ring base registers
- proper TX kick using `NCR_TSTART`
- consistent RX/TX descriptor ownership handling
- default PHY loopback behaviour
- loopback cleanup

### Differences from Current `macb_ethdev.c`

The backup version appears more bring-up oriented and contains comments indicating it replaced a previous `macb_ethdev.c`.

Notable differences include:

- `g_force_phy_lb` defaults to enabled
- PHY loopback is central to the default test path
- forced 100/full-duplex configuration is emphasized
- DMA sync uses `dma_sync_user.h` rather than the ioctl UAPI header directly

### Why Keep It

This file is useful for understanding how the driver evolved and which hardware issues were being debugged:

- descriptor ownership polarity
- 64-bit DMA ring addresses
- PHY versus MAC loopback
- TX-start behaviour

For current development, treat it as a reference rather than the authoritative implementation.

---

## `macb_rxtx.c`

### Purpose

Packet fast path for the PMD.

This file contains the RX and TX burst functions used by DPDK applications:

```c
macb_rx_burst(...)
macb_tx_burst(...)
```

It is responsible for actual packet movement between DPDK mbufs and MACB/GEM descriptor rings.

### Runtime Tracing

The file supports runtime tracing through:

```bash
MACB_RXTX_TRACE=0..3
```

Typical meaning:

| Level | Meaning |
|---:|---|
| `0` | Minimal tracing. |
| `1` | Basic RX/TX state. |
| `2` | More verbose descriptor tracing. |
| `3` | Detailed debugging output. |

### RX Path

The RX path checks descriptor ownership, extracts frame length and status flags, and returns completed packets to the DPDK application.

Important RX concepts:

- `RX_USED`
- `RX_WRAP`
- `RX_SOF`
- `RX_EOF`
- `RX_LEN_MASK`

### RX Descriptor Arming

The helper:

```c
rx_arm_addr_word(...)
```

prepares an RX descriptor address word by:

- inserting the DMA address
- preserving the wrap bit if needed
- clearing the ownership bit so hardware owns the descriptor

### RX Completion

When hardware receives a packet, it updates descriptor state so software can process it.

The driver then:

1. synchronizes descriptor memory from device if needed
2. checks ownership
3. reads length and flags
4. synchronizes packet buffer if needed
5. returns mbuf to application
6. allocates/reuses a replacement mbuf
7. rearms the descriptor

### RX Probe and Debugging

```c
macb_rx_probe_once(...)
```

Dumps selected RX descriptors and receive status register state. This is useful for diagnosing whether hardware is seeing packets but not handing them to software.

### TX Path

The TX path takes packets from DPDK, maps their data IOVA, writes TX descriptors, and kicks hardware.

Important TX concepts:

- `TX_USED`
- `TX_WRAP`
- `TX_LAST`
- `TX_LEN_MASK`
- `NCR_TSTART`

### TX Descriptor Flow

```text
Application mbuf
        ↓
macb_tx_burst()
        ↓
Descriptor address = mbuf data IOVA
        ↓
Descriptor length/status programmed
        ↓
TX_USED cleared
        ↓
Memory barrier
        ↓
NCR_TSTART written
        ↓
Hardware transmits
```

### TX Completion

The driver checks whether hardware has returned ownership of descriptors. Completed mbufs are freed so the ring can be reused.

### Software Statistics

The file updates counters such as:

- `ad->sw.tx_pkts`
- `ad->sw.tx_bytes`
- `ad->sw.rx_pkts`
- `ad->sw.rx_bytes`

### Development Notes

Most packet-loss, stall, and corruption bugs will be found here.

Key things to inspect:

- ownership-bit polarity
- wrap-bit preservation
- descriptor synchronization
- IOVA correctness
- mbuf lifetime
- TX completion cleanup

---

## `macb_rxtx.c.bak`

### Purpose

Backup copy of the RX/TX fast path.

Use this file as historical reference if behaviour changed between revisions.

### Likely Use Cases

- Compare descriptor ownership handling
- Compare TX flag definitions
- Recover older debug logic
- Understand bring-up attempts

### Development Warning

Do not assume this file is current. It may contain stale constants or logic that were corrected in the active `macb_rxtx.c`.

---

## `macb_debug.h`

### Purpose

Debug helper header for register and descriptor inspection.

This file provides small inline functions that can be called from the PMD to dump important hardware state.

### Register Dumping

```c
macb_dump_top_regs(...)
```

Dumps key registers such as:

- `NCR`
- `NCFGR`
- `NSR`
- `RSR`
- `TSR`
- `RBQP`
- `TBQP`

### Address Filter Diagnostics

```c
macb_dump_addr_filters(...)
```

Dumps MAC address and filtering-related registers where available.

Useful for debugging:

- promiscuous mode
- broadcast filtering
- unicast MAC address setup
- hash filtering

### Status Edge Diagnostics

```c
macb_dump_status_edges(...)
```

Captures RX/TX status changes and helps diagnose stalls.

### Why This File Matters

Low-level drivers often fail silently when hardware is misconfigured. This header provides visibility into hardware state without requiring external register-dump tools.

---

## `meson.build`

### Purpose

Build definition for integrating the PMD into a DPDK source tree.

Although the exact content may vary, this file normally defines:

- source files for the `net/macb` PMD
- dependencies
- include paths
- build conditions

### Expected Role

It should include source files such as:

```text
macb_ethdev.c
macb_rxtx.c
macb_uio.c
```

and register them as part of a DPDK network driver build.

### Development Notes

When adding or removing source files, update this file so DPDK's Meson build system compiles the correct driver components.

---

# Packet Flow Walkthrough

## Transmit Path

```text
DPDK Application
    ↓
rte_eth_tx_burst()
    ↓
macb_tx_burst()
    ↓
TX descriptor filled
    ↓
Descriptor synced to device if needed
    ↓
TX_USED cleared
    ↓
NCR_TSTART written
    ↓
GEM DMA reads packet buffer
    ↓
MAC transmits frame
    ↓
PHY sends frame to wire
```

## Receive Path

```text
Wire
    ↓
PHY
    ↓
MAC receives frame
    ↓
GEM DMA writes packet buffer
    ↓
RX descriptor marked complete
    ↓
macb_rx_burst()
    ↓
Buffer synced from device if needed
    ↓
mbuf returned to DPDK app
```

---

# Common Debugging Scenarios

## Link Does Not Come Up

Check:

- PHY address
- MDIO reads
- BMCR/BMSR values
- autonegotiation status
- `phy_mode` argument
- external switch/cable

Relevant files:

- `macb_ethdev.c`
- `macb_debug.h`

## TX Packets Are Queued but Not Sent

Check:

- TX descriptor ownership
- `NCR_TSTART`
- TX ring base address
- `TBQP`/`TBQPH`
- IOVA correctness
- cache synchronization

Relevant files:

- `macb_rxtx.c`
- `macb_ethdev.c`
- `macb_hw.h`

## RX Packets Never Arrive

Check:

- RX descriptor ownership
- `RBQP`/`RBQPH`
- RX buffer size
- promiscuous mode
- broadcast filtering
- DMA sync behaviour
- RSR status bits

Relevant files:

- `macb_rxtx.c`
- `macb_ethdev.c`
- `macb_debug.h`

## Driver Fails to Map Hardware

Check:

- `/dev/uioX` exists
- sysfs UIO metadata exists
- permissions
- kernel driver binding
- UIO device name

Relevant file:

- `macb_uio.c`

---

# Key Challenges Solved by this Driver

1. Running the Raspberry Pi 5 RP1 Ethernet MAC from userspace.
2. Integrating MACB/GEM hardware with DPDK ethdev.
3. Mapping MMIO registers through UIO.
4. Managing RX/TX DMA descriptor rings.
5. Handling 64-bit ring base programming.
6. Managing descriptor ownership transitions correctly.
7. Discovering and configuring the Ethernet PHY through MDIO.
8. Supporting PHY and MAC loopback for testing.
9. Supporting optional DMA cache synchronization.
10. Providing enough debug instrumentation for hardware bring-up.

---

# Summary

This repository is a custom DPDK userspace Ethernet driver for the Raspberry Pi 5 RP1 Cadence MACB/GEM controller.

The driver bypasses the Linux networking stack and directly manages:

- UIO register mapping
- MACB/GEM register configuration
- DMA descriptor rings
- DPDK mbufs
- RX/TX packet bursts
- PHY configuration over MDIO
- link and loopback state
- optional DMA cache synchronization

The most important files are:

| File | Role |
|---|---|
| `macb_ethdev.c` | Main DPDK PMD and device control layer. |
| `macb_rxtx.c` | RX/TX packet fast path. |
| `macb_hw.h` | Hardware definitions and shared structures. |
| `macb_uio.c` | UIO-based MMIO mapping. |
| `dma_sync_uapi.h` | IOCTL API for optional DMA cache synchronization. |
| `dma_sync_user.h` | Userspace DMA sync shim. |
| `macb_debug.h` | Register and descriptor debugging helpers. |
| `meson.build` | DPDK build integration. |

The project is best understood as hardware bring-up work for a DPDK PMD on non-standard embedded Ethernet hardware, with substantial attention paid to descriptor ownership, DMA addressing, link setup, and debug visibility.
