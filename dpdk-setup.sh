#!/usr/bin/env bash
#
# rp1-dpdk-setup.sh
#
# Post-reboot setup for running the DPDK net_macb PMD against the
# Raspberry Pi 5 RP1 Ethernet controller (Cadence MACB/GEM) via UIO.
# Covers README sections 1-6: hugepages, detaching the interface from
# the Linux stack, binding to uio_pdrv_genirq, and loading the DMA
# sync helper module. Safe to re-run (idempotent).
#
# Usage:
#   sudo ./rp1-dpdk-setup.sh [options]
#
# Options:
#   --iface=NAME        Linux interface to detach (default: eth0)
#   --hugepages=N        Number of 2M hugepages to request (default: 1024)
#   --huge-dir=PATH       Hugepage mountpoint (default: /dev/hugepages)
#   --dpdk-dir=PATH       Path to DPDK checkout, for --testpmd/--matrix-rx
#                         (default: $HOME/dpdk)
#   --testpmd             After setup, exec testpmd with the README's example args
#   --matrix-rx            After setup, exec dpdk-matrix-rx
#   -h, --help             Show this help
#
# Environment overrides (same effect as the matching flag):
#   IFACE, HUGEPAGES, HUGE_MOUNT, DPDK_DIR, NAME (force platform device name)
#
# After a successful run, source the generated env file to pick up
# MACB_DMA_SYNC_NODE in your interactive shell:
#   source /run/rp1-dpdk-env
#
set -euo pipefail

# ---------- defaults (overridable via env or flags) ----------
IFACE="${IFACE:-eth0}"
HUGEPAGES="${HUGEPAGES:-1024}"
HUGE_MOUNT="${HUGE_MOUNT:-/dev/hugepages}"
DPDK_DIR="${DPDK_DIR:-$HOME/dpdk}"
DMA_HELPER_DIR="${DMA_HELPER_DIR:-$HOME/dma-sync-helper}"
DMA_HELPER_REPO="https://github.com/rachel275/dma-sync-helper.git"
DMA_NODE="${DMA_NODE:-/dev/dma_sync_eth0}"
ENV_FILE="/run/rp1-dpdk-env"
DO_TESTPMD=0
DO_MATRIX_RX=0
# ---------------------------------------------------------------

log()  { echo -e "\033[1;32m[+]\033[0m $*"; }
warn() { echo -e "\033[1;33m[!]\033[0m $*" >&2; }
err()  { echo -e "\033[1;31m[x]\033[0m $*" >&2; }

usage() { sed -n '2,26p' "$0"; }

for arg in "$@"; do
    case "$arg" in
        --iface=*)      IFACE="${arg#*=}" ;;
        --hugepages=*)  HUGEPAGES="${arg#*=}" ;;
        --huge-dir=*)   HUGE_MOUNT="${arg#*=}" ;;
        --dpdk-dir=*)   DPDK_DIR="${arg#*=}" ;;
        --testpmd)      DO_TESTPMD=1 ;;
        --matrix-rx)    DO_MATRIX_RX=1 ;;
        -h|--help)      usage; exit 0 ;;
        *) err "Unknown option: $arg"; usage; exit 1 ;;
    esac
done

require_root() {
    if [[ $EUID -ne 0 ]]; then
        err "Run as root, e.g.: sudo $0 $*"
        exit 1
    fi
}

# ---------- 1 & 2. Hugepages ----------
setup_hugepages() {
    log "Configuring hugepages"
    local current
    current=$(cat /proc/sys/vm/nr_hugepages)
    if (( current < HUGEPAGES )); then
        echo "$HUGEPAGES" > /proc/sys/vm/nr_hugepages
    fi
    current=$(cat /proc/sys/vm/nr_hugepages)
    log "nr_hugepages = $current (requested $HUGEPAGES; actual count depends on free memory)"

    mkdir -p "$HUGE_MOUNT"
    if ! mountpoint -q "$HUGE_MOUNT"; then
        mount -t hugetlbfs nodev "$HUGE_MOUNT"
        log "Mounted hugetlbfs at $HUGE_MOUNT"
    else
        log "$HUGE_MOUNT already mounted"
    fi
}

# ---------- 3. Determine the ethernet platform device ----------
detect_device_name() {
    if [[ -n "${NAME:-}" ]]; then
        log "Using NAME from environment: $NAME"
        return
    fi

    if [[ -e "/sys/class/net/$IFACE/device" ]]; then
        NAME=$(basename "$(readlink "/sys/class/net/$IFACE/device")")
        log "Platform device: $NAME"
        return
    fi

    # $IFACE is already gone from the netdev list, most likely because a
    # prior run this boot already unbound it. Fall back to whatever is
    # already sitting under uio_pdrv_genirq.
    local uio_dev
    uio_dev=$(find /sys/bus/platform/drivers/uio_pdrv_genirq -maxdepth 1 -name '*.ethernet' -printf '%f\n' 2>/dev/null | head -1) || true
    if [[ -n "$uio_dev" ]]; then
        NAME="$uio_dev"
        log "$IFACE already detached; found $NAME bound to uio_pdrv_genirq"
        return
    fi

    err "Could not determine the platform device name."
    err "Set it explicitly, e.g.: sudo NAME=1f00100000.ethernet $0"
    exit 1
}

# ---------- 4. Detach interface from Linux ----------
detach_from_linux() {
    if [[ -e "/sys/class/net/$IFACE" ]]; then
        log "Bringing $IFACE down"
        ip link set "$IFACE" down || warn "could not bring $IFACE down (continuing)"
    fi

    local driver_link="/sys/bus/platform/devices/$NAME/driver"
    if [[ -e "$driver_link" ]]; then
        local current_driver
        current_driver=$(basename "$(readlink -f "$driver_link")")
        if [[ "$current_driver" != "uio_pdrv_genirq" ]]; then
            log "Unbinding $NAME from $current_driver"
            echo "$NAME" > "$driver_link/unbind"
        else
            log "$NAME already bound to uio_pdrv_genirq"
        fi
    else
        log "$NAME currently has no driver bound"
    fi
}

# ---------- 5. Bind to UIO ----------
bind_to_uio() {
    log "Loading uio / uio_pdrv_genirq"
    modprobe uio
    modprobe uio_pdrv_genirq

    local driver_link="/sys/bus/platform/devices/$NAME/driver"
    if [[ -e "$driver_link" ]] && [[ "$(basename "$(readlink -f "$driver_link")")" == "uio_pdrv_genirq" ]]; then
        log "$NAME already bound to uio_pdrv_genirq"
    else
        echo uio_pdrv_genirq > "/sys/bus/platform/devices/$NAME/driver_override"
        log "Binding $NAME to uio_pdrv_genirq"
        echo "$NAME" > /sys/bus/platform/drivers/uio_pdrv_genirq/bind
    fi

    sleep 1
    if [[ ! -e /dev/uio0 ]]; then
        err "/dev/uio0 did not appear after bind."
        err "If this hangs, run 'dmesg -w' in another terminal and retry — see README Troubleshooting."
        exit 1
    fi
    log "/dev/uio0 ready"
}

# ---------- 6. DMA sync helper ----------
load_dma_helper() {
    if lsmod | grep -q '^dma_sync_helper'; then
        log "dma_sync_helper already loaded"
    else
        if [[ ! -d "$DMA_HELPER_DIR" ]]; then
            log "Cloning dma-sync-helper into $DMA_HELPER_DIR"
            git clone "$DMA_HELPER_REPO" "$DMA_HELPER_DIR"
        fi
        log "Building dma-sync-helper"
        make -C "$DMA_HELPER_DIR"
        log "Loading dma_sync_helper.ko (pdev_name=$NAME alias=$IFACE)"
        insmod "$DMA_HELPER_DIR/dma_sync_helper.ko" pdev_name="$NAME" alias="$IFACE"
    fi

    # Best-effort scrape of the major number from dmesg. The exact log
    # line format depends on the module; adjust the grep pattern below,
    # or just set DMA_MAJOR=<n> yourself, if this doesn't find it.
    local major="${DMA_MAJOR:-}"
    if [[ -z "$major" ]]; then
        major=$(dmesg | grep -i 'dma_sync' | grep -oE '[Mm]ajor[[:space:]]*[:=][[:space:]]*[0-9]+' | tail -1 | grep -oE '[0-9]+$') || true
    fi

    if [[ -z "$major" ]]; then
        warn "Could not auto-detect the major number. Check: dmesg | tail"
        warn "Then create the node manually: sudo mknod $DMA_NODE c <major> 0 && sudo chmod 666 $DMA_NODE"
        return
    fi

    if [[ ! -e "$DMA_NODE" ]]; then
        mknod "$DMA_NODE" c "$major" 0
        chmod 666 "$DMA_NODE"
        log "Created $DMA_NODE (major $major)"
    else
        log "$DMA_NODE already exists"
    fi
}

write_env_file() {
    cat > "$ENV_FILE" <<EOF
export NAME="$NAME"
export MACB_DMA_SYNC_NODE="$DMA_NODE"
EOF
    chmod 644 "$ENV_FILE"
    log "Wrote $ENV_FILE — run 'source $ENV_FILE' to get these in your shell"
}

run_testpmd() {
    log "Launching testpmd"
    exec env MACB_RXTX_TRACE=2 MACB_DMA_SYNC_NODE="$DMA_NODE" \
        "$DPDK_DIR/build/app/dpdk-testpmd" \
        -l 1-2 \
        -n 1 \
        --log-level=pmd:debug \
        --iova=pa \
        --huge-dir="$HUGE_MOUNT" \
        --socket-mem=64 \
        --file-prefix=macb \
        --no-telemetry \
        --huge-unlink \
        --vdev="net_macb0,dev=/dev/uio0,phy_lb=0,mac_lb=0,phy_mode=100fd" \
        -- \
        --interactive \
        --nb-cores=1 \
        --rxd=128 \
        --txd=128 \
        --total-num-mbufs=2048
}

run_matrix_rx() {
    log "Launching dpdk-matrix-rx"
    exec env MACB_DMA_SYNC_NODE="$DMA_NODE" \
        "$DPDK_DIR/build/examples/dpdk-matrix-rx" \
        --vdev=net_macb0,dev=/dev/uio0,phy_mode=auto \
        -l 0-1 \
        -n 1 \
        --no-pci
}

main() {
    require_root
    setup_hugepages
    detect_device_name
    detach_from_linux
    bind_to_uio
    load_dma_helper
    write_env_file
    log "Setup complete."

    if [[ "$DO_TESTPMD" -eq 1 ]]; then
        run_testpmd
    elif [[ "$DO_MATRIX_RX" -eq 1 ]]; then
        run_matrix_rx
    else
        log "Ready. Launch testpmd or dpdk-matrix-rx yourself, or re-run with --testpmd / --matrix-rx."
    fi
}

main