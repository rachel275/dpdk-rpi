DPDK is a set of libraries and drivers for fast packet processing.
It supports many processor architectures and both FreeBSD and Linux.

The DPDK uses the Open Source BSD-3-Clause license for the core libraries
and drivers. The kernel components are GPL-2.0 licensed.

Please check the doc directory for release notes,
API documentation, and sample application information.

For questions and usage discussions, subscribe to: users@dpdk.org
Report bugs and issues to the development mailing list: dev@dpdk.org

System Setup (Raspberry Pi 5 + RP1 MACB)

This fork enables the Raspberry Pi 5 RP1 Ethernet controller (Cadence MACB/GEM) to run as a DPDK Poll Mode Driver (PMD) using a UIO-backed userspace driver.

Unlike conventional DPDK NIC drivers, this PMD does **not** use PCI device discovery. Instead, the RP1 Ethernet platform device is detached from the Linux networking stack, rebound to `uio_pdrv_genirq`, and then accessed through `/dev/uio0` by the DPDK MACB vdev.

The DMA sync helper is also required on Raspberry Pi 5 to ensure cache coherency between DMA operations and CPU access.

---

# Architecture

```text
Linux Ethernet Driver (eth0)
            │
            ▼
    Unbind platform device
            │
            ▼
     uio_pdrv_genirq
            │
            ▼
         /dev/uio0
            │
            ▼
      DPDK net_macb PMD
            │
            ▼
   RP1 MACB/GEM Hardware
```

The PMD is instantiated as a DPDK virtual device:

```bash
--vdev=net_macb0,dev=/dev/uio0
```

# 0. Build DPDK

Install dependencies:

```bash
sudo apt update

sudo apt install -y \
    build-essential \
    meson \
    ninja-build \
    pkg-config \
    libnuma-dev \
    python3-pyelftools \
    libpcap-dev \
    libbpf-dev \
    clang
```

Configure and build:

```bash
meson setup build -Dexamples=matrix-rx
ninja -C build
sudo ninja -C build install
sudo ldconfig
```

Reconfigure:

```bash
meson configure build -Dexamples=matrix-rx
ninja -C build
```

Clean build:

```bash
rm -rf build
meson setup build -Dexamples=matrix-rx
ninja -C build
```

---

# 1. Configure Hugepages

DPDK requires hugepage-backed memory for packet buffers and DMA allocations.

Allocate hugepages:

```bash
echo 1024 | sudo tee /proc/sys/vm/nr_hugepages
```

Persist across reboots:

```bash
echo "vm.nr_hugepages=1024" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

Verify:

```bash
grep Huge /proc/meminfo
```

Example:

```text
HugePages_Total: 205
```

The exact number may be lower than requested depending on available memory.

---

# 2. Mount Hugepages

After every reboot:

```bash
sudo mkdir -p /dev/hugepages
sudo mount -t hugetlbfs nodev /dev/hugepages
```

Optional `/etc/fstab` entry:

```text
nodev /dev/hugepages hugetlbfs defaults 0 0
```

Verify:

```bash
mount | grep huge
```

---

# 3. Determine the Ethernet Platform Device

Different Pi firmware and kernel versions may expose different names.
Determine the correct device dynamically:

```bash
NAME=$(basename "$(readlink /sys/class/net/eth0/device)")
echo $NAME
```

Example:

```text
1f00100000.ethernet
```

---

# 4. Detach eth0 from Linux

If connected via SSH over eth0, you will lose connectivity.

Bring the interface down:

```bash
sudo ip link set eth0 down
```

Check the current driver:

```bash
readlink /sys/bus/platform/devices/$NAME/driver
```

Unbind the Linux driver:

```bash
echo $NAME | sudo tee \
    /sys/bus/platform/devices/$NAME/driver/unbind
```

After this step, eth0 will no longer appear as a normal Linux network interface.

---

# 5. Bind the Device to UIO

Load UIO support:

```bash
sudo modprobe uio
sudo modprobe uio_pdrv_genirq
```

Override the driver:

```bash
echo uio_pdrv_genirq | sudo tee \
    /sys/bus/platform/devices/$NAME/driver_override
```

Bind:

```bash
echo $NAME | sudo tee \
    /sys/bus/platform/drivers/uio_pdrv_genirq/bind
```

Expected behaviour:

```text
1f00100000.ethernet
```

The command should return immediately.

Verify:

```bash
ls -l /dev/uio*
```

Expected:

```text
/dev/uio0
```

Confirm ownership:

```bash
readlink /sys/bus/platform/devices/$NAME/driver
```

Expected:

```text
/sys/bus/platform/drivers/uio_pdrv_genirq
```

If the bind hangs indefinitely, collect:

```bash
dmesg -w
```

in another terminal before attempting the bind.

---

# 6. DMA Sync Helper (Required)

RP1 DMA is not cache coherent with CPU caches.

The DMA sync helper exposes Linux DMA cache-synchronisation primitives to userspace and is required for correct operation.

Clone:

```bash
git clone https://github.com/rachel275/dma-sync-helper.git
cd dma-sync-helper
make
```

Load:

```bash
sudo insmod dma_sync_helper.ko pdev_name=1f00100000.ethernet alias=eth0
```

Check dmesg for the allocated device major number:

```bash
dmesg | tail
```

Create device node if required:

```bash
sudo mknod /dev/dma_sync_eth0 c <major> 0
sudo chmod 666 /dev/dma_sync_eth0
```

Optional:

```bash
export MACB_DMA_SYNC_NODE=/dev/dma_sync_eth0
```

---

# 7. Run testpmd

```bash
sudo env MACB_RXTX_TRACE=2 \
./build/app/dpdk-testpmd \
  -l 1-2 \
  -n 1 \
  --log-level=pmd:debug \
  --iova=pa \
  --huge-dir=/dev/hugepages \
  --socket-mem=64 \
  --file-prefix=macb \
  --no-telemetry \
  --huge-unlink \
  --vdev='net_macb0,dev=/dev/uio0,phy_lb=0,mac_lb=0,phy_mode=100fd' \
  -- \
  --interactive \
  --nb-cores=1 \
  --rxd=128 \
  --txd=128 \
  --total-num-mbufs=2048
```

---

# 8. Run Matrix RX

```bash
sudo ./build/examples/dpdk-matrix-rx \
  --vdev=net_macb0,dev=/dev/uio0,phy_mode=auto \
  -l 0-1 \
  -n 1 \
  --no-pci
```

---

# Troubleshooting

## No `/dev/uio0`

Verify:

```bash
lsmod | grep uio
```

and:

```bash
readlink /sys/bus/platform/devices/$NAME/driver
```

The device must be bound to:

```text
uio_pdrv_genirq
```

---

## Bind Hangs

Before binding:

```bash
sudo ip link set eth0 down

echo $NAME | sudo tee \
    /sys/bus/platform/devices/$NAME/driver/unbind
```

Then monitor:

```bash
dmesg -w
```

and attempt the bind again.

---

## Why Not Use dpdk-devbind?

This PMD is implemented as a DPDK vdev and accesses the RP1 Ethernet hardware through UIO.

The RP1 Ethernet MAC is exposed as a platform device, not as a standalone PCI NIC that can be managed through the normal DPDK PCI binding workflow.

Therefore:

```bash
dpdk-devbind.py
```

is not used for the MACB PMD.

