#!/bin/sh
# ==============================================================================
# Automated RISC-V Bare-Metal Initramfs Packaging Pipeline
# Targets: StarFive VisionFive 2 / RISC-V 64-bit Bare-Metal Remote Boot (TFTP)
# ==============================================================================

set -e

ARCH="${ARCH:-riscv}"
CROSS_COMPILE="${CROSS_COMPILE:-riscv64-linux-gnu-}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BASE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUSYBOX_DIR="$BASE_DIR/busybox"
BUILD_RISCV_DIR="$BASE_DIR/build-riscv"
VEREMI_DIR="$BASE_DIR/veremi"
INSTALL_DIR="$BUSYBOX_DIR/_install"
INITRAMFS_CPIO="$BASE_DIR/initramfs.cpio"
INITRAMFS_GZ="$BASE_DIR/initramfs.cpio.gz"
TFTP_DIR="/srv/tftp"

echo "=============================================================================="
echo "      AUTOMATED RISC-V BARE-METAL BENCHMARK INITRAMFS GENERATOR"
echo "=============================================================================="
echo " Target Architecture : $ARCH (64-bit)"
echo " Cross Compiler      : ${CROSS_COMPILE}gcc / ${CROSS_COMPILE}g++"
echo " Workspace Root      : $BASE_DIR"
echo " Initramfs Output    : $INITRAMFS_GZ"
echo " TFTP Remote Target  : $TFTP_DIR"
echo "=============================================================================="

# 1. Verify Cross-Compiler
if ! command -v "${CROSS_COMPILE}gcc" >/dev/null 2>&1; then
    echo "[!] ERROR: Cross compiler '${CROSS_COMPILE}gcc' not found in PATH."
    echo "    Install on Arch Linux: sudo pacman -S riscv64-linux-gnu-gcc"
    exit 1
fi
if ! command -v "${CROSS_COMPILE}g++" >/dev/null 2>&1; then
    echo "[!] ERROR: Cross compiler '${CROSS_COMPILE}g++' not found in PATH."
    exit 1
fi

# 2. Build or Reuse Static BusyBox
if [ ! -f "$INSTALL_DIR/bin/busybox" ] || [ "${REBUILD_BUSYBOX:-0}" = "1" ]; then
    echo "[*] Configuring and building BusyBox for $ARCH..."
    if [ ! -d "$BUSYBOX_DIR" ]; then
        git clone https://github.com/mirror/busybox.git "$BUSYBOX_DIR"
    fi
    cd "$BUSYBOX_DIR"
    make ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE" defconfig
    sed -i 's/# CONFIG_STATIC is not set/CONFIG_STATIC=y/g' .config
    if ! grep -q "^CONFIG_STATIC=y" .config; then
        echo "CONFIG_STATIC=y" >> .config
    fi
    sed -i 's/CONFIG_TC=y/# CONFIG_TC is not set/' .config
    sed -i 's/CONFIG_FEATURE_TC_INGRESS=y/# CONFIG_FEATURE_TC_INGRESS is not set/' .config
    sed -i 's/CONFIG_SHA1_HWACCEL=y/# CONFIG_SHA1_HWACCEL is not set/' .config
    sed -i 's/CONFIG_SHA256_HWACCEL=y/# CONFIG_SHA256_HWACCEL is not set/' .config
    make ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE" -j"$(nproc)" install
    echo "[+] BusyBox static build complete."
else
    echo "[+] Using existing static BusyBox binary at $INSTALL_DIR/bin/busybox"
fi

# 3. Cross-Compile offline_evaluator for RISC-V (Static, Zero-Dependency)
echo "[*] Cross-compiling offline_evaluator for RISC-V 64-bit..."
cd "$BASE_DIR"
cmake -B "$BUILD_RISCV_DIR" -S . \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=riscv64 \
    -DCMAKE_C_COMPILER="${CROSS_COMPILE}gcc" \
    -DCMAKE_CXX_COMPILER="${CROSS_COMPILE}g++" \
    -DUSE_ARROW=OFF \
    -DSTATIC_BUILD=ON >/dev/null

cmake --build "$BUILD_RISCV_DIR" -j"$(nproc)"

# Strip debug symbols
echo "[*] Stripping debug symbols from RISC-V binary..."
"${CROSS_COMPILE}strip" -s "$BUILD_RISCV_DIR/offline_evaluator" -o "$BUILD_RISCV_DIR/offline_evaluator_stripped"
ls -lh "$BUILD_RISCV_DIR/offline_evaluator_stripped"

# 4. Verify Datasets (Convert any missing .parquet to .bin)
echo "[*] Verifying binary datasets in $VEREMI_DIR..."
PYTHON_BIN="/home/gust/CP-Artery/.venv/bin/python"
if [ ! -x "$PYTHON_BIN" ]; then
    PYTHON_BIN="python3"
fi

$PYTHON_BIN "$SCRIPT_DIR/export_parquet_to_bin.py" -i "$VEREMI_DIR" -o "$VEREMI_DIR"

BIN_COUNT=$(ls -1 "$VEREMI_DIR"/*.bin 2>/dev/null | wc -l)
echo "[+] Total binary datasets available: $BIN_COUNT files."

# 5. Staging Filesystem Tree
echo "[*] Staging initramfs root filesystem in $INSTALL_DIR..."
cd "$INSTALL_DIR"
rm -f linuxrc
mkdir -p proc sys dev etc tmp root bench bench/data bench/results

# Copy benchmark binary, runner, and datasets
cp -f "$BUILD_RISCV_DIR/offline_evaluator_stripped" bench/offline_evaluator
chmod +x bench/offline_evaluator

cp -f "$SCRIPT_DIR/run_benchmarks.sh" bench/run_benchmarks.sh
chmod +x bench/run_benchmarks.sh

echo "[*] Installing 8 standardized binary datasets into /bench/data/..."
rm -rf bench/data/*
cp -f "$VEREMI_DIR"/*_0709.bin bench/data/

# 6. Generate Init Script (/init)
cat << 'EOF' > init
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs devtmpfs /dev 2>/dev/null || true
mkdir -p /tmp /bench/results
mount -t tmpfs tmpfs /tmp
mount -t tmpfs tmpfs /bench/results

clear
echo "=================================================================="
echo "    STARFIVE VISIONFIVE 2 / RISC-V BARE-METAL BENCHMARK SUITE     "
echo "=================================================================="
echo " Architecture: $(uname -m) (Kernel: $(uname -r))"
echo " Bench Dir   : /bench"
echo " Datasets    : $(ls /bench/data/*.bin 2>/dev/null | wc -l) fault profiles (50,000 msgs each)"
echo "------------------------------------------------------------------"
echo " QUICK COMMANDS:"
echo "   1. Run full 3-Tier suite on all profiles:"
echo "      cd /bench && ./run_benchmarks.sh --all-profiles"
echo ""
echo "   2. Run single profile (e.g. ConstPos):"
echo "      cd /bench && ./run_benchmarks.sh --file data/ConstPos_0709.bin"
echo ""
echo "   3. Direct low-level execution:"
echo "      /bench/offline_evaluator --tier 3 --stage1 ols-ctra --stage2 beta-reputation --dir /bench/data/ConstPos_0709.bin --profile"
echo "=================================================================="

exec /bin/sh
EOF

chmod +x init

# 7. Package CPIO and Compress with Gzip
echo "[*] Packaging root filesystem into CPIO archive..."
cd "$INSTALL_DIR"
find . | cpio -o -H newc > "$INITRAMFS_CPIO" 2>/dev/null

echo "[*] Compressing with gzip -9..."
gzip -9 -c "$INITRAMFS_CPIO" > "$INITRAMFS_GZ"
rm -f "$INITRAMFS_CPIO"

echo "=============================================================================="
echo "[+] SUCCESS! Generated bare-metal initramfs archive:"
ls -lh "$INITRAMFS_GZ"
echo "=============================================================================="

# 8. Deploy to /srv/tftp/ if permitted
if [ -d "$TFTP_DIR" ] && [ -w "$TFTP_DIR" ]; then
    echo "[*] Deploying directly to $TFTP_DIR/initramfs.cpio.gz..."
    cp -f "$INITRAMFS_GZ" "$TFTP_DIR/initramfs.cpio.gz"
    echo "[+] Deployed successfully to TFTP boot directory!"
else
    echo "[!] Notice: $TFTP_DIR requires elevated permissions to overwrite."
    echo "    Run the following command to update your TFTP boot payload:"
    echo ""
    echo "    sudo cp $INITRAMFS_GZ $TFTP_DIR/initramfs.cpio.gz"
    echo ""
fi
echo "=============================================================================="
