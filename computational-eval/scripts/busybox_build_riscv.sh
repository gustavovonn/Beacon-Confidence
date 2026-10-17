#!/bin/sh
# ==============================================================================
# Isolated Static BusyBox Installer / Builder
# Targets: Static Userland for RISC-V 64-bit
# ==============================================================================

set -e

ARCH="${ARCH:-riscv}"
CROSS_COMPILE="${CROSS_COMPILE:-riscv64-linux-gnu-}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BASE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUSYBOX_DIR="$BASE_DIR/busybox"
INSTALL_DIR="$BUSYBOX_DIR/_install"

echo "=============================================================================="
echo "                   STATIC BUSYBOX INSTALLER & BUILDER"
echo "=============================================================================="
echo " Target Architecture : $ARCH (64-bit)"
echo " Cross Compiler      : ${CROSS_COMPILE}gcc"
echo " Source Directory    : $BUSYBOX_DIR"
echo " Install Staging     : $INSTALL_DIR"
echo "=============================================================================="

# 1. Verify Cross-Compiler
if ! command -v "${CROSS_COMPILE}gcc" >/dev/null 2>&1; then
    echo "[!] ERROR: Cross compiler '${CROSS_COMPILE}gcc' not found in PATH."
    echo "    Install on Arch Linux: sudo pacman -S riscv64-linux-gnu-gcc"
    echo "    Install on Ubuntu:     sudo apt install gcc-riscv64-linux-gnu g++-riscv64-linux-gnu"
    exit 1
fi

# 2. Clone or Update BusyBox
if [ ! -d "$BUSYBOX_DIR" ]; then
    echo "[*] Cloning BusyBox source repository..."
    git clone https://github.com/mirror/busybox.git "$BUSYBOX_DIR"
else
    echo "[*] Using existing BusyBox source at $BUSYBOX_DIR"
fi

# 3. Configure and Compile Static BusyBox
cd "$BUSYBOX_DIR"

if [ ! -f "$INSTALL_DIR/bin/busybox" ] || [ "${REBUILD_BUSYBOX:-0}" = "1" ]; then
    echo "[*] Configuring BusyBox for $ARCH (CONFIG_STATIC=y)..."
    make ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE" defconfig

    # Force static build
    sed -i 's/# CONFIG_STATIC is not set/CONFIG_STATIC=y/g' .config
    if ! grep -q "^CONFIG_STATIC=y" .config; then
        echo "CONFIG_STATIC=y" >> .config
    fi

    # Disable Linux kernel header dependent or problematic applets for cross-build
    sed -i 's/CONFIG_TC=y/# CONFIG_TC is not set/' .config
    sed -i 's/CONFIG_FEATURE_TC_INGRESS=y/# CONFIG_FEATURE_TC_INGRESS is not set/' .config
    sed -i 's/CONFIG_SHA1_HWACCEL=y/# CONFIG_SHA1_HWACCEL is not set/' .config
    sed -i 's/CONFIG_SHA256_HWACCEL=y/# CONFIG_SHA256_HWACCEL is not set/' .config

    echo "[*] Compiling and installing static BusyBox..."
    make ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE" -j"$(nproc)" install

    echo "=============================================================================="
    echo "[+] SUCCESS! Static BusyBox binary installed at:"
    ls -lh "$INSTALL_DIR/bin/busybox"
    echo "=============================================================================="
else
    echo "[+] Static BusyBox binary is already built at:"
    ls -lh "$INSTALL_DIR/bin/busybox"
    echo "    (Set REBUILD_BUSYBOX=1 to force clean rebuild)"
fi
