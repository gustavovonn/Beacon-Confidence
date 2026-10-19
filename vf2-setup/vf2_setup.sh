#!/bin/sh
# ==============================================================================
# StarFive VisionFive 2 (VF2) Modular Environment Setup & Packaging Pipeline
# ==============================================================================
# Orchestrates:
#   1. Toolchain: Verifies cross-compilation tools
#   2. BusyBox: Fetches/builds static via busybox_build_riscv.sh
#   3. Project Code: Cross-compiles static C++ offline_evaluator for RISC-V 64-bit
#   4. Project Data: Verifies/converts binary traces into veremi/bin/
#   5. Rootfs Staging & Packaging: Stages /bench and creates initramfs.cpio.gz
#   6. TFTP Deployment: Syncs boot payload to TFTP directory
# ==============================================================================

set -e

ARCH="${ARCH:-riscv}"
CROSS_COMPILE="${CROSS_COMPILE:-riscv64-linux-gnu-}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"      # vf2-setup/
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BASE_DIR="$REPO_ROOT/computational-eval"          # where the evaluator and its data live
CONFIG_DIR="$SCRIPT_DIR/config"                   # prebuilt kernel Image + device tree
ETL_DIR="$BASE_DIR/scripts"                       # parquet/bin conversion scripts
# Packaging artefacts stay under vf2-setup/; only the cross-build and the trace
# data live beside the evaluator they belong to.
BUSYBOX_DIR="$SCRIPT_DIR/busybox"
INSTALL_DIR="$BUSYBOX_DIR/_install"
BUILD_RISCV_DIR="$BASE_DIR/build-riscv"
VEREMI_BIN_DIR="$BASE_DIR/veremi/bin"
VEREMI_PARQUET_DIR="$BASE_DIR/veremi/parquet"
INITRAMFS_CPIO="$SCRIPT_DIR/initramfs.cpio"
INITRAMFS_GZ="$SCRIPT_DIR/initramfs.cpio.gz"
TFTP_DIR="${TFTP_DIR:-/srv/tftp}"
KERNEL_IMAGE="$CONFIG_DIR/Image"
DEVICE_TREE="$CONFIG_DIR/jh7110-starfive-visionfive-2-v1.3b.dtb"
TFTP_IFACE="${TFTP_IFACE:-enp2s0}"
TFTP_HOST_IP="${TFTP_HOST_IP:-192.168.1.100/24}"
TFTP_NM_PROFILE="${TFTP_NM_PROFILE:-vf2-tftp}"

REBUILD_BUSYBOX=0
REBUILD_PROJECT=0
SKIP_TFTP=0
SKIP_TFTP_NETWORK=0

# Parse CLI options
while [ $# -gt 0 ]; do
    case "$1" in
        --rebuild-all)
            REBUILD_BUSYBOX=1
            REBUILD_PROJECT=1
            shift
            ;;
        --rebuild-busybox)
            REBUILD_BUSYBOX=1
            shift
            ;;
        --rebuild-project)
            REBUILD_PROJECT=1
            shift
            ;;
        --skip-tftp)
            SKIP_TFTP=1
            shift
            ;;
        --skip-tftp-network)
            SKIP_TFTP_NETWORK=1
            shift
            ;;
        --tftp-dir)
            TFTP_DIR="$2"
            shift 2
            ;;
        --tftp-iface)
            TFTP_IFACE="$2"
            shift 2
            ;;
        --tftp-ip)
            TFTP_HOST_IP="$2"
            shift 2
            ;;
        --tftp-profile)
            TFTP_NM_PROFILE="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --rebuild-all        Clean and rebuild both BusyBox and C++ project"
            echo "  --rebuild-busybox    Force recompilation of static BusyBox"
            echo "  --rebuild-project    Force clean cross-compilation of C++ evaluator"
            echo "  --skip-tftp          Skip TFTP deployment step entirely"
            echo "  --skip-tftp-network  Deploy TFTP payload without reconfiguring host network"
            echo "  --tftp-dir <path>    Set custom TFTP deploy target (default: $TFTP_DIR)"
            echo "  --tftp-iface <dev>   Set host Ethernet interface for TFTP (default: $TFTP_IFACE)"
            echo "  --tftp-ip <cidr>     Set host IP address for TFTP (default: $TFTP_HOST_IP)"
            echo "  --tftp-profile <name> NetworkManager profile name (default: $TFTP_NM_PROFILE)"
            echo "  -h, --help           Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

echo "=============================================================================="
echo "         STARFIVE VISIONFIVE 2 (VF2) MODULAR SETUP & PACKAGER"
echo "=============================================================================="
echo " Target Architecture : $ARCH (64-bit)"
echo " Cross Compiler      : ${CROSS_COMPILE}gcc / ${CROSS_COMPILE}g++"
echo " Workspace Root      : $BASE_DIR"
echo " Staging Rootfs      : $INSTALL_DIR"
echo " Initramfs Payload   : $INITRAMFS_GZ"
echo " Evaluator Source    : $BASE_DIR"
echo " Boot Config         : $CONFIG_DIR"
echo " TFTP Destination    : $TFTP_DIR"
echo " TFTP Host Interface : $TFTP_IFACE ($TFTP_HOST_IP)"
echo "=============================================================================="

# ------------------------------------------------------------------------------
# Module 1: Cross-Compiler Validation
# ------------------------------------------------------------------------------
check_toolchain() {
    echo ""
    echo "[*] [Module 1/6] Verifying RISC-V 64-bit cross-compilation toolchain..."
    if ! command -v "${CROSS_COMPILE}gcc" >/dev/null 2>&1; then
        echo "[!] ERROR: Cross compiler '${CROSS_COMPILE}gcc' not found in PATH."
        exit 1
    fi
    if ! command -v "${CROSS_COMPILE}g++" >/dev/null 2>&1; then
        echo "[!] ERROR: Cross compiler '${CROSS_COMPILE}g++' not found in PATH."
        exit 1
    fi
    echo "[+] Toolchain verified: $(${CROSS_COMPILE}gcc --version | head -n 1)"
}

# ------------------------------------------------------------------------------
# Module 2: BusyBox Userland Setup
# ------------------------------------------------------------------------------
setup_busybox_userland() {
    echo ""
    echo "[*] [Module 2/6] Setting up static BusyBox userland..."
    if [ ! -f "$INSTALL_DIR/bin/busybox" ] || [ "$REBUILD_BUSYBOX" = "1" ]; then
        REBUILD_BUSYBOX="$REBUILD_BUSYBOX" "$SCRIPT_DIR/busybox_build_riscv.sh"
    else
        echo "[+] Static BusyBox already present at $INSTALL_DIR/bin/busybox"
    fi
}

# ------------------------------------------------------------------------------
# Module 3: Project Source Code Compilation (RISC-V Evaluator)
# ------------------------------------------------------------------------------
build_project_code() {
    echo ""
    echo "[*] [Module 3/6] Compiling project C++ evaluator for RISC-V..."
    cd "$BASE_DIR"
    if [ "$REBUILD_PROJECT" = "1" ]; then
        rm -rf "$BUILD_RISCV_DIR"
    fi

    cmake -B "$BUILD_RISCV_DIR" -S "$BASE_DIR" \
        -DCMAKE_SYSTEM_NAME=Linux \
        -DCMAKE_SYSTEM_PROCESSOR=riscv64 \
        -DCMAKE_C_COMPILER="${CROSS_COMPILE}gcc" \
        -DCMAKE_CXX_COMPILER="${CROSS_COMPILE}g++" \
        -DSTATIC_BUILD=ON >/dev/null

    cmake --build "$BUILD_RISCV_DIR" -j"$(nproc)"

    echo "[*] Stripping symbols for compact embedded rootfs payload..."
    "${CROSS_COMPILE}strip" -s "$BUILD_RISCV_DIR/offline_evaluator" -o "$BUILD_RISCV_DIR/offline_evaluator_stripped"
    ls -lh "$BUILD_RISCV_DIR/offline_evaluator_stripped"
}

# ------------------------------------------------------------------------------
# Module 4: Project Benchmark Trace Data Verification
# ------------------------------------------------------------------------------
prepare_project_data() {
    echo ""
    echo "[*] [Module 4/6] Verifying benchmark binary trace datasets (veremi/bin)..."
    mkdir -p "$VEREMI_BIN_DIR"

    # Check python environment
    REPO_ROOT="$(cd "$BASE_DIR/../.." && pwd)"
    if [ -n "$VIRTUAL_ENV" ] && [ -x "$VIRTUAL_ENV/bin/python" ]; then
        PYTHON_BIN="$VIRTUAL_ENV/bin/python"
    elif [ -x "$BASE_DIR/.venv/bin/python" ]; then
        PYTHON_BIN="$BASE_DIR/.venv/bin/python"
    elif [ -x "$REPO_ROOT/.venv/bin/python" ]; then
        PYTHON_BIN="$REPO_ROOT/.venv/bin/python"
    elif [ -x "$REPO_ROOT/veremi-extension/.venv/bin/python" ]; then
        PYTHON_BIN="$REPO_ROOT/veremi-extension/.venv/bin/python"
    else
        PYTHON_BIN="python3"
    fi

    # Verify python dependencies
    if ! "$PYTHON_BIN" -c "import pyarrow, scipy, pandas" >/dev/null 2>&1; then
        echo "[*] Bootstrapping virtual environment in $BASE_DIR/.venv..."
        python3 -m venv "$BASE_DIR/.venv"
        "$BASE_DIR/.venv/bin/pip" install --quiet -r "$BASE_DIR/requirements.txt"
        PYTHON_BIN="$BASE_DIR/.venv/bin/python"
    fi

    # Check if parquet files exist; if not, generate them
    PARQUET_COUNT=0
    [ -d "$VEREMI_PARQUET_DIR" ] && PARQUET_COUNT=$(ls -1 "$VEREMI_PARQUET_DIR"/*.parquet 2>/dev/null | wc -l)

    if [ "$PARQUET_COUNT" -eq 0 ]; then
        echo "[!] No parquet traces found. Invoking interpolate_veremi.py --all-profiles..."
        "$PYTHON_BIN" "$ETL_DIR/interpolate_veremi.py" --all-profiles
    fi

    # Convert Parquet to binary
    echo "[*] Converting Parquet traces to zero-dependency binary format (.bin)..."
    "$PYTHON_BIN" "$ETL_DIR/export_parquet_to_bin.py" -i "$VEREMI_PARQUET_DIR" -o "$VEREMI_BIN_DIR"

    BIN_COUNT=$(ls -1 "$VEREMI_BIN_DIR"/*.bin 2>/dev/null | wc -l)
    if [ "$BIN_COUNT" -eq 0 ]; then
        echo "[!] Error: No .bin files generated in $VEREMI_BIN_DIR."
        echo "    Please verify raw simulation traces exist in veremi-extension/data/"
        exit 1
    else
        echo "[+] $BIN_COUNT binary dataset(s) ready in $VEREMI_BIN_DIR"
    fi
}

# ------------------------------------------------------------------------------
# Module 5: Rootfs Staging & Packaging
# ------------------------------------------------------------------------------
stage_and_package() {
    echo ""
    echo "[*] [Module 5/6] Staging rootfs, benchmark runner, and packaging initramfs..."
    cd "$INSTALL_DIR"
    rm -f linuxrc
    mkdir -p proc sys dev etc tmp root bench bench/data bench/results

    # 1. Install project binary
    cp -f "$BUILD_RISCV_DIR/offline_evaluator_stripped" bench/offline_evaluator
    chmod +x bench/offline_evaluator

    # 2. Install dedicated RISC-V runner
    cp -f "$SCRIPT_DIR/run_benchmarks_riscv.sh" bench/run_benchmarks_riscv.sh
    chmod +x bench/run_benchmarks_riscv.sh

    # 3. Install project trace data
    rm -rf bench/data/*
    if ls "$VEREMI_BIN_DIR"/*_0709.bin >/dev/null 2>&1; then
        cp -f "$VEREMI_BIN_DIR"/*_0709.bin bench/data/
    elif ls "$VEREMI_BIN_DIR"/*.bin >/dev/null 2>&1; then
        cp -f "$VEREMI_BIN_DIR"/*.bin bench/data/
    fi
    echo "[+] Staged $(ls -1 bench/data/*.bin 2>/dev/null | wc -l) trace dataset(s) into /bench/data/"

    # 4. Generate Init Script (/init)
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
echo " Datasets    : $(ls /bench/data/*.bin 2>/dev/null | wc -l) fault profiles"
echo "------------------------------------------------------------------"
echo " QUICK COMMANDS:"
echo "   1. Run full 3-Tier suite on all profiles (default):"
echo "      cd /bench && ./run_benchmarks_riscv.sh"
echo ""
echo "   2. Run single profile (e.g. ConstPos):"
echo "      cd /bench && ./run_benchmarks_riscv.sh --profile ConstPos_0709"
echo ""
echo "   3. Direct low-level execution:"
echo "      /bench/offline_evaluator --tier 3 --stage1 ols-ctra --stage2 beta-reputation --dir /bench/data/ConstPos_0709.bin --profile"
echo "=================================================================="

exec /bin/sh
EOF
    chmod +x init

    # 5. Pack CPIO and compress with gzip
    echo "[*] Compressing rootfs into $INITRAMFS_GZ..."
    find . | cpio -o -H newc > "$INITRAMFS_CPIO" 2>/dev/null
    gzip -9 -c "$INITRAMFS_CPIO" > "$INITRAMFS_GZ"
    rm -f "$INITRAMFS_CPIO"

    echo "=============================================================================="
    echo "[+] SUCCESS! Generated VF2 bare-metal initramfs:"
    ls -lh "$INITRAMFS_GZ"
    echo "=============================================================================="
}

# ------------------------------------------------------------------------------
# Module 6: TFTP Boot Deployment & Connection Setup
# ------------------------------------------------------------------------------

# Host Network Interface & Persistent Static IP Configuration
configure_tftp_network() {
    echo ""
    echo "[*] [6.1] Configuring host network interface for TFTP ($TFTP_IFACE -> $TFTP_HOST_IP)..."

    # Verify physical interface exists
    if ! ip link show "$TFTP_IFACE" >/dev/null 2>&1; then
        echo "[!] Warning: Network interface '$TFTP_IFACE' not found on host. Skipping network configuration."
        return 1
    fi

    # Approach A: NetworkManager (persistent static connection, survives cable/board resets)
    if command -v nmcli >/dev/null 2>&1 && systemctl is-active --quiet NetworkManager 2>/dev/null; then
        echo "[*] Using NetworkManager for persistent static IP management..."
        PROFILE_NAME="${TFTP_NM_PROFILE:-vf2-tftp}"

        if ! nmcli connection show "$PROFILE_NAME" >/dev/null 2>&1; then
            echo "[*] Creating persistent NetworkManager connection '$PROFILE_NAME' on $TFTP_IFACE..."
            sudo nmcli connection add type ethernet \
                con-name "$PROFILE_NAME" \
                ifname "$TFTP_IFACE" \
                ip4 "$TFTP_HOST_IP" \
                connection.autoconnect yes \
                connection.autoconnect-priority 10
        else
            echo "[*] Updating existing NetworkManager profile '$PROFILE_NAME'..."
            sudo nmcli connection modify "$PROFILE_NAME" \
                connection.interface-name "$TFTP_IFACE" \
                ipv4.addresses "$TFTP_HOST_IP" \
                ipv4.method manual \
                connection.autoconnect yes \
                connection.autoconnect-priority 10
        fi

        sudo nmcli connection up "$PROFILE_NAME"
        echo "[+] NetworkManager profile '$PROFILE_NAME' active with static IP $TFTP_HOST_IP."
    else
        # Approach B: iproute2 fallback (for minimal/containerized host environments)
        echo "[*] Using iproute2 fallback for interface configuration..."
        CLEAN_IP="${TFTP_HOST_IP%%/*}"
        if ! ip -br addr show dev "$TFTP_IFACE" 2>/dev/null | grep -q "$CLEAN_IP"; then
            sudo ip addr add dev "$TFTP_IFACE" "$TFTP_HOST_IP" 2>/dev/null || true
        fi
        sudo ip link set dev "$TFTP_IFACE" up
        echo "[+] Interface $TFTP_IFACE configured with $TFTP_HOST_IP and link is UP."
    fi
}

# TFTP Daemon / Socket Verification & Activation
ensure_tftp_daemon() {
    echo ""
    echo "[*] [6.2] Verifying TFTP server daemon (tftpd.socket)..."
    if systemctl is-active --quiet tftpd.socket 2>/dev/null; then
        echo "[+] tftpd.socket daemon is active and listening on port 69."
    else
        echo "[*] Enabling and activating tftpd.socket daemon..."
        sudo systemctl enable --now tftpd.socket
        echo "[+] tftpd.socket successfully enabled and active."
    fi
}

# TFTP Payload Deployment
deploy_tftp_payload() {
    echo ""
    echo "[*] [6.3] Deploying boot payload to $TFTP_DIR..."
    sudo mkdir -p "$TFTP_DIR"

    # Kernel and device tree ship from vf2-setup/config/, so the board can be
    # netbooted without any artefact built outside this repository.
    if [ -f "$KERNEL_IMAGE" ]; then
        sudo cp -f "$KERNEL_IMAGE" "$TFTP_DIR/Image"
        echo "[+] Kernel      : $TFTP_DIR/Image"
    else
        echo "[!] Warning: kernel image not found at $KERNEL_IMAGE"
    fi

    if [ -f "$DEVICE_TREE" ]; then
        sudo cp -f "$DEVICE_TREE" "$TFTP_DIR/$(basename "$DEVICE_TREE")"
        echo "[+] Device tree : $TFTP_DIR/$(basename "$DEVICE_TREE")"
    else
        echo "[!] Warning: device tree not found at $DEVICE_TREE"
    fi

    sudo cp -f "$INITRAMFS_GZ" "$TFTP_DIR/initramfs.cpio.gz"
    echo "[+] Initramfs   : $TFTP_DIR/initramfs.cpio.gz"
}

# Module 6 Orchestrator
deploy_tftp() {
    if [ "$SKIP_TFTP" = "1" ]; then
        echo ""
        echo "[*] [Module 6/6] Skipping TFTP deployment (--skip-tftp specified)."
        return 0
    fi

    echo ""
    echo "[*] [Module 6/6] TFTP connection & deployment verification:"
    echo "    - Target Directory : $TFTP_DIR"
    echo "    - Host Interface   : $TFTP_IFACE"
    echo "    - Required Host IP : $TFTP_HOST_IP"
    echo "    - TFTP Daemon      : tftpd.socket"
    echo "    - NM Profile       : $TFTP_NM_PROFILE"

    USER_CHOICE=""
    if [ -t 0 ] || [ -c /dev/tty ]; then
        printf "[-] Configure TFTP connection and deploy payload with sudo now? [y/N]: "
        read -r USER_CHOICE </dev/tty 2>/dev/null || read -r USER_CHOICE || true
    fi

    case "$USER_CHOICE" in
        [yY]|[yY][eE][sS])
            if [ "$SKIP_TFTP_NETWORK" != "1" ]; then
                configure_tftp_network
                ensure_tftp_daemon
            else
                echo "[*] Skipping network/daemon configuration (--skip-tftp-network specified)."
            fi
            deploy_tftp_payload
            echo ""
            echo "=============================================================================="
            echo "[+] TFTP setup and payload deployment completed successfully!"
            echo "=============================================================================="
            ;;
        *)
            echo "[-] TFTP configuration & deployment skipped (or non-interactive)."
            echo "    To manually configure your host and deploy, run:"
            echo ""
            echo "    sudo nmcli connection add type ethernet con-name $TFTP_NM_PROFILE ifname $TFTP_IFACE ip4 $TFTP_HOST_IP"
            echo "    sudo nmcli connection up $TFTP_NM_PROFILE"
            echo "    sudo systemctl enable --now tftpd.socket"
            echo "    sudo mkdir -p $TFTP_DIR"
            echo "    sudo cp $KERNEL_IMAGE $TFTP_DIR/Image"
            echo "    sudo cp $DEVICE_TREE $TFTP_DIR/"
            echo "    sudo cp $INITRAMFS_GZ $TFTP_DIR/initramfs.cpio.gz"
            echo ""
            ;;
    esac
}

# Execute all modules
check_toolchain
setup_busybox_userland
build_project_code
prepare_project_data
stage_and_package
deploy_tftp
