# Matrix RX Test Application

## Overview

`matrix_rx.c` is a DPDK userspace application designed to test and validate the Raspberry Pi 5 MACB/GEM Poll Mode Driver (PMD).

The application:

* Initializes DPDK EAL
* Locates the MACB PMD device
* Creates RX/TX queues
* Receives custom Ethernet frames
* Parses matrix payloads
* Sends acknowledgement packets
* Reports receive statistics

Unlike normal network applications, this program bypasses:

* Linux networking
* TCP/IP
* UDP
* Sockets

and communicates directly using raw Ethernet frames.

---

# Building and Running

## Prerequisites

The application requires:

* DPDK installed and built
* Hugepages configured
* MACB PMD compiled into DPDK
* UIO access to the RP1 Ethernet controller

Verify DPDK:

```bash
dpdk-testpmd --version
```

---

## Hugepages

Check current hugepage allocation:

```bash
grep Huge /proc/meminfo
```

Allocate hugepages:

```bash
sudo sysctl -w vm.nr_hugepages=1024
```

Verify:

```bash
grep HugePages_Total /proc/meminfo
```

---

## UIO Setup

Verify UIO device:

```bash
ls /dev/uio*
```

Expected:

```text
/dev/uio0
```

Check the mapped device:

```bash
cat /sys/class/uio/uio0/name
```

---

## Running with the MACB PMD

Example launch:

```bash
sudo ./matrix-rx \
  -l 2 \
  -n 4 \
  --log-level=pmd,8 \
  --vdev="net_macb0,dev=/dev/uio0,phy_addr=1,phy_lb=0,mac_lb=0" \
  --
```

The application will:

1. Initialise DPDK
2. Create mbuf pools
3. Configure RX/TX queues
4. Start the MACB PMD
5. Enable promiscuous mode
6. Begin receiving matrix packets

Expected output:

```text
APP:start
APP:after_eal

ethdevs avail: 1
port 0: net_macb0

Using port 0
```

---

## Running with AF_PACKET

If MACB hardware is unavailable:

```bash
sudo ./matrix-rx \
  -l 2 \
  -n 4 \
  --vdev=net_af_packet0,iface=eth0
```

The application automatically attempts:

```text
net_macb0
```

first and falls back to:

```text
net_af_packet0
```

if unavailable.

---

# Purpose

The primary goals are:

1. Validate RX functionality of the MACB PMD
2. Validate TX functionality of the MACB PMD
3. Verify descriptor ownership transitions
4. Verify DMA operation
5. Measure packet throughput
6. Transport matrix data with minimal overhead

---

# Protocol Overview

The application uses a custom Ethernet protocol.

## Ethertype

```c
#define ETHERTYPE_MATRIX 0x88BD
```

Frames with this Ethertype are interpreted as matrix packets.

---

# Packet Format

## Ethernet Header

```text
+------------------+
| Destination MAC  |
+------------------+
| Source MAC       |
+------------------+
| Ethertype 88BD   |
+------------------+
```

## Matrix Header

```text
+----------------+
| "MX01"         | 4 bytes
+----------------+
| Rows           | 2 bytes
+----------------+
| Columns        | 2 bytes
+----------------+
| Element Type   | 1 byte
+----------------+
| Padding        | 3 bytes
+----------------+
```

Header size:

```text
12 bytes
```

---

## Matrix Data

Supported types:

| Type | Meaning |
| ---- | ------- |
| 1    | Float32 |

Example:

```text
1024 × 1024 float matrix

1,048,576 elements
×
4 bytes

=
4 MB payload
```

---

# Application Startup

## EAL Initialisation

```c
rte_eal_init()
```

Initialises:

* Hugepages
* Memory subsystem
* CPU cores
* PMDs

---

## Device Discovery

The application enumerates all available DPDK ports:

```text
port 0: net_macb0
```

or

```text
port 0: net_af_packet0
```

---

## Port Selection

Priority order:

1. net_macb0
2. net_af_packet0

If neither exists:

```text
No usable ethdev found
```

and the application exits.

---

# Queue Configuration

## RX Queue

```c
RX_DESC = 256
```

256 receive descriptors.

---

## TX Queue

```c
TX_DESC = 256
```

256 transmit descriptors.

---

## Burst Size

```c
BURST = 32
```

The application processes up to 32 packets per poll cycle.

Benefits:

* Better cache utilisation
* Reduced overhead
* Improved throughput

---

# Mempool

## Creation

```c
rte_pktmbuf_pool_create()
```

Configuration:

```text
8192 mbufs
2048-byte buffers
512 cache entries
```

---

## Warmup

Function:

```c
warmup_mempool()
```

Touches allocated memory before packet processing.

Purpose:

* Prevent first-use page faults
* Improve latency consistency
* Benchmark stability

---

# Receive Processing

Main loop:

```c
rte_eth_rx_burst()
```

Receives up to:

```text
32 packets
```

per iteration.

---

# Packet Validation

The application checks:

## Ethertype

```text
0x88BD
```

## Signature

```text
MX01
```

## Dimensions

```text
Rows
Columns
```

## Element Type

```text
1 = float32
```

---

# Matrix Parsing

Payload size calculation:

```text
rows × cols × element_size
```

Example:

```text
8 × 8 × 4

=
256 bytes
```

The application validates packet size before processing.

---

# Matrix Inspection

For debugging:

```text
Matrix 1024x1024 (type=1):
first two elems = 1.000, 2.000
```

Printed periodically to confirm:

* Parsing correctness
* Endianness
* DMA transfer validity

---

# ACK Mechanism

Every valid matrix packet generates an acknowledgement.

---

## ACK Packet Format

Payload:

```text
MXOK
```

followed by:

```text
32-bit packet counter
```

Structure:

```text
+------------+
| MXOK       |
+------------+
| Counter    |
+------------+
```

---

## ACK Transmission

Function:

```c
send_ack()
```

Process:

1. Allocate mbuf
2. Build Ethernet frame
3. Copy sender MAC address
4. Insert MXOK payload
5. Send using:

```c
rte_eth_tx_burst()
```

---

# Prefetch Optimisation

The receive path uses:

```c
rte_prefetch0()
```

Configuration:

```c
PREFETCH_OFFSET = 4
```

Meaning:

While processing packet N:

```text
Packet N+4 is prefetched
```

into cache.

Benefits:

* Lower memory latency
* Higher packet throughput

---

# Statistics

Printed approximately once per second.

Example:

```text
[stats]
RX=100000
TX=100000
RX-err=0
miss=0
no_mbuf=0
seen=100000
```

---

## Statistics Fields

| Field   | Description                    |
| ------- | ------------------------------ |
| RX      | Received packets               |
| TX      | Transmitted packets            |
| RX-err  | Receive errors                 |
| miss    | Missed packets                 |
| no_mbuf | Mbuf allocation failures       |
| seen    | Valid matrix packets processed |

---

# Packet Flow

## Receive Path

```text
Sender
   ↓
Ethernet Frame
   ↓
MACB PMD
   ↓
RX Descriptor
   ↓
rte_eth_rx_burst()
   ↓
handle_pkt()
   ↓
Matrix Parsing
```

---

## ACK Path

```text
Valid Matrix
      ↓
send_ack()
      ↓
Build Ethernet Frame
      ↓
TX Descriptor
      ↓
rte_eth_tx_burst()
      ↓
Sender Receives ACK
```

---

# Acronyms

| Acronym | Meaning                       |
| ------- | ----------------------------- |
| ACK     | Acknowledgement               |
| DMA     | Direct Memory Access          |
| DPDK    | Data Plane Development Kit    |
| EAL     | Environment Abstraction Layer |
| MAC     | Media Access Control          |
| MBUF    | DPDK Packet Buffer            |
| PMD     | Poll Mode Driver              |
| RX      | Receive                       |
| TX      | Transmit                      |
| UIO     | Userspace I/O                 |

---

# Summary

`matrix_rx.c` is a lightweight DPDK validation and benchmarking application that receives custom matrix packets over raw Ethernet, validates and parses the matrix payload, and sends acknowledgement frames back to the sender.

It is primarily used to validate:

* MACB PMD functionality
* DMA operation
* RX/TX descriptor handling
* Ethernet transport of matrix payloads
* End-to-end packet flow on Raspberry Pi 5

