#!/bin/sh
# ==============================================================================
# StarFive VisionFive 2 (VF2) Setup & Packaging Pipeline - object-trajectory
# ==============================================================================
# Orchestrates:
#   1. Toolchain: Verifies cross-compilation tools
#   2. BusyBox Userland: Fetches/builds static userland via busybox_build_riscv.sh
#   3. Project Code: Cross-compiles the static RISC-V engine, tests and evaluators
#   4. Project Data: Trims an optional dataset slice for the real-trace replay
#   5. Rootfs Staging & Packaging: Stages /bench and creates initramfs.cpio.gz
#   6. TFTP Deployment: Syncs boot payload to TFTP directory
# ==============================================================================
#
# PROVENANCE
# The structure, the module layout and the TFTP handling follow
# cp-trustfulness/f2md/scripts/vf2_setup.sh, so that both experiments are driven the same way.
# What differs is this experiment's own build and payload, described below.
#
# HOW THIS DIFFERS FROM THE f2md PIPELINE
#
#   Build system.
#   The engine builds with a plain Makefile, not CMake.
#   Cross-compiling is therefore `make CXX=riscv64-linux-gnu-g++`, with no toolchain file.
#
#   Dependencies.
#   There are none to install for the target.
#   Eigen is vendored, header-only, and cross-compiles with the rest; see shared/eigen/.
#
#   Data.
#   The headline benchmark needs none.
#   cost_evaluator generates its own synthetic load and prints the host CPU, precisely so that a
#   board result can never be confused with a workstation one.
#   A dataset slice is shipped only if you ask for it, for the real-trace replay.
#
#   Tests.
#   The test suites go on the board too.
#   A cost number from a binary whose own tests have not passed on that silicon is worth little.
# ==============================================================================

set -e

ARCH="${ARCH:-riscv}"
CROSS_COMPILE="${CROSS_COMPILE:-riscv64-linux-gnu-}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BASE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VF2_DIR="$BASE_DIR/vf2"
BUSYBOX_DIR="$VF2_DIR/busybox"
INSTALL_DIR="$BUSYBOX_DIR/_install"
BUILD_RISCV_DIR="$BASE_DIR/build-riscv"
DATASET_DIR="$BASE_DIR/dataset/groundtruth"
STAGED_DATA_DIR="$VF2_DIR/data"
INITRAMFS_CPIO="$VF2_DIR/initramfs.cpio"
INITRAMFS_GZ="$VF2_DIR/initramfs.cpio.gz"
TFTP_DIR="${TFTP_DIR:-/srv/tftp}"
TFTP_IFACE="${TFTP_IFACE:-enp2s0}"
TFTP_HOST_IP="${TFTP_HOST_IP:-192.168.1.100/24}"
TFTP_NM_PROFILE="${TFTP_NM_PROFILE:-vf2-tftp}"

REBUILD_BUSYBOX=0
REBUILD_PROJECT=0
SKIP_TFTP=0
SKIP_TFTP_NETWORK=0
WITH_DATA=0
DATA_SECONDS=20

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
        --with-data)
            WITH_DATA=1
            shift
            ;;
        --data-seconds)
            WITH_DATA=1
            DATA_SECONDS="$2"
            shift 2
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
            echo "  --rebuild-all         Clean and rebuild both BusyBox and the C++ project"
            echo "  --rebuild-busybox     Force recompilation of static BusyBox"
            echo "  --rebuild-project     Force clean cross-compilation of the engine"
            echo "  --with-data           Also ship a dataset slice for the real-trace replay"
            echo "  --data-seconds <s>    Seconds of trace to ship (implies --with-data, default: $DATA_SECONDS)"
            echo "  --skip-tftp           Skip TFTP deployment step entirely"
            echo "  --skip-tftp-network   Deploy TFTP payload without reconfiguring host network"
            echo "  --tftp-dir <path>     Set custom TFTP deploy target (default: $TFTP_DIR)"
            echo "  --tftp-iface <dev>    Set host Ethernet interface for TFTP (default: $TFTP_IFACE)"
            echo "  --tftp-ip <cidr>      Set host IP address for TFTP (default: $TFTP_HOST_IP)"
            echo "  --tftp-profile <name> NetworkManager profile name (default: $TFTP_NM_PROFILE)"
            echo "  -h, --help            Show this help message"
            echo ""
            echo "The headline cost benchmark needs no dataset: cost_evaluator makes its own load."
            echo "Use --with-data only when you also want the real-trace replay on the board."
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

echo "=============================================================================="
echo "      STARFIVE VISIONFIVE 2 (VF2) SETUP - OBJECT-TRAJECTORY EXPERIMENT"
echo "=============================================================================="
echo " Target Architecture : $ARCH (64-bit)"
echo " Cross Compiler      : ${CROSS_COMPILE}gcc / ${CROSS_COMPILE}g++"
echo " Workspace Root      : $BASE_DIR"
echo " Staging Rootfs      : $INSTALL_DIR"
echo " Initramfs Payload   : $INITRAMFS_GZ"
echo " Dataset Slice       : $([ "$WITH_DATA" = "1" ] && echo "${DATA_SECONDS}s" || echo "none (synthetic load only)")"
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
# Module 3: Project Source Code Compilation (static RISC-V)
# ------------------------------------------------------------------------------
# Everything is linked statically because the initramfs has no shared libraries and no loader.
# Eigen needs nothing here: it is headers, and they cross-compile with the rest.
build_project_code() {
    echo ""
    echo "[*] [Module 3/6] Cross-compiling the engine, its tests and the evaluators..."
    cd "$BASE_DIR"

    if [ "$REBUILD_PROJECT" = "1" ]; then
        rm -rf "$BUILD_RISCV_DIR"
    fi

    make BUILD="$BUILD_RISCV_DIR" \
         CXX="${CROSS_COMPILE}g++" \
         CXXFLAGS="-std=c++17 -O2 -Wall -Wextra -isystem shared/eigen -static" \
         all

    echo "[*] Stripping symbols for a compact embedded rootfs payload..."
    for b in cost_evaluator experiment_runner test_core test_pipeline test_dataset test_lab_ctra; do
        if [ -f "$BUILD_RISCV_DIR/$b" ]; then
            "${CROSS_COMPILE}strip" -s "$BUILD_RISCV_DIR/$b"
        fi
    done

    echo "[+] Built for $(file -b "$BUILD_RISCV_DIR/cost_evaluator" 2>/dev/null | cut -d, -f1-2)"
    ls -lh "$BUILD_RISCV_DIR"/cost_evaluator "$BUILD_RISCV_DIR"/experiment_runner 2>/dev/null || true
}

# ------------------------------------------------------------------------------
# Module 4: Optional dataset slice
# ------------------------------------------------------------------------------
# The full trace is about 60 MB, which is more than an initramfs wants to carry.
# Trimming by TIME rather than by line keeps the three files consistent with each other.
# The parser tolerates a reception whose content was cut and content never received, so even a
# ragged cut degrades gracefully, but a time cut leaves nothing ragged.
prepare_project_data() {
    echo ""
    if [ "$WITH_DATA" != "1" ]; then
        echo "[*] [Module 4/6] No dataset slice requested; the board will run the synthetic load only."
        rm -rf "$STAGED_DATA_DIR"
        return 0
    fi

    echo "[*] [Module 4/6] Trimming a ${DATA_SECONDS}s dataset slice..."
    if [ ! -f "$DATASET_DIR/cam.csv" ]; then
        echo "[!] ERROR: no dataset at $DATASET_DIR."
        echo "    Generate one first, or drop --with-data."
        exit 1
    fi

    rm -rf "$STAGED_DATA_DIR"
    mkdir -p "$STAGED_DATA_DIR"

    FIRST_T=$(awk -F, 'NR==2 { print $2; exit }' "$DATASET_DIR/cam.csv")
    CUTOFF=$(awk -v a="$FIRST_T" -v b="$DATA_SECONDS" 'BEGIN { print a + b }')
    echo "    trace starts at ${FIRST_T}s, cutting at ${CUTOFF}s"

    # cam.csv and cpm.csv carry the content time in column 2.
    awk -F, -v c="$CUTOFF" 'NR==1 || $2 <= c' "$DATASET_DIR/cam.csv" > "$STAGED_DATA_DIR/cam.csv"
    awk -F, -v c="$CUTOFF" 'NR==1 || $2 <= c' "$DATASET_DIR/cpm.csv" > "$STAGED_DATA_DIR/cpm.csv"
    # reception.csv carries the arrival time in column 3.
    awk -F, -v c="$CUTOFF" 'NR==1 || $3 <= c' "$DATASET_DIR/reception.csv" > "$STAGED_DATA_DIR/reception.csv"
    cp -f "$DATASET_DIR/metadata.json" "$STAGED_DATA_DIR/metadata.json" 2>/dev/null || true

    echo "[+] Slice ready:"
    du -sh "$STAGED_DATA_DIR"
    wc -l "$STAGED_DATA_DIR"/*.csv | sed 's/^/    /'
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

    # 1. Install the binaries
    rm -f bench/cost_evaluator bench/experiment_runner bench/test_*
    for b in cost_evaluator experiment_runner test_core test_pipeline test_dataset test_lab_ctra; do
        if [ -f "$BUILD_RISCV_DIR/$b" ]; then
            cp -f "$BUILD_RISCV_DIR/$b" "bench/$b"
            chmod +x "bench/$b"
        fi
    done

    # 2. Install the dedicated RISC-V runner
    cp -f "$SCRIPT_DIR/run_benchmarks_riscv.sh" bench/run_benchmarks_riscv.sh
    chmod +x bench/run_benchmarks_riscv.sh

    # 3. Install the dataset slice, if one was asked for
    rm -rf bench/data
    mkdir -p bench/data
    if [ -d "$STAGED_DATA_DIR" ]; then
        cp -f "$STAGED_DATA_DIR"/* bench/data/ 2>/dev/null || true
        echo "[+] Staged $(ls -1 bench/data 2>/dev/null | wc -l) dataset file(s) into /bench/data/"
    else
        echo "[+] No dataset staged; the synthetic benchmark needs none."
    fi

    # 4. Generate Init Script (/init)
    cat << 'INITEOF' > init
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs devtmpfs /dev 2>/dev/null || true
mkdir -p /tmp /bench/results
mount -t tmpfs tmpfs /tmp
mount -t tmpfs tmpfs /bench/results

clear
echo "=================================================================="
echo "  VISIONFIVE 2 / RISC-V - OBJECT-TRAJECTORY CONFIDENCE BENCHMARK  "
echo "=================================================================="
echo " Architecture: $(uname -m) (Kernel: $(uname -r))"
echo " Bench Dir   : /bench"
echo " Dataset     : $(ls /bench/data/*.csv 2>/dev/null | wc -l) file(s) staged"
echo "------------------------------------------------------------------"
echo " QUICK COMMANDS:"
echo "   1. Full benchmark (tests, then the cost suite):"
echo "      cd /bench && ./run_benchmarks_riscv.sh"
echo ""
echo "   2. Cost only, skipping the test suites:"
echo "      cd /bench && ./run_benchmarks_riscv.sh --skip-tests"
echo ""
echo "   3. The one number that matters, by hand:"
echo "      /bench/cost_evaluator --tracks 61 --vectors 5 --messages 20000"
echo ""
echo "   4. The scale sweep:"
echo "      /bench/cost_evaluator --sweep"
echo "=================================================================="

exec /bin/sh
INITEOF
    chmod +x init

    # 5. Pack CPIO and compress with gzip
    echo "[*] Compressing rootfs into $INITRAMFS_GZ..."
    mkdir -p "$VF2_DIR"
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

# Sub-module 6.1: Host Network Interface & Persistent Static IP Configuration
configure_tftp_network() {
    echo ""
    echo "[*] [6.1] Configuring host network interface for TFTP ($TFTP_IFACE -> $TFTP_HOST_IP)..."

    if ! ip link show "$TFTP_IFACE" >/dev/null 2>&1; then
        echo "[!] Warning: Network interface '$TFTP_IFACE' not found on host. Skipping network configuration."
        return 1
    fi

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
        echo "[*] Using iproute2 fallback for interface configuration..."
        CLEAN_IP="${TFTP_HOST_IP%%/*}"
        if ! ip -br addr show dev "$TFTP_IFACE" 2>/dev/null | grep -q "$CLEAN_IP"; then
            sudo ip addr add dev "$TFTP_IFACE" "$TFTP_HOST_IP" 2>/dev/null || true
        fi
        sudo ip link set dev "$TFTP_IFACE" up
        echo "[+] Interface $TFTP_IFACE configured with $TFTP_HOST_IP and link is UP."
    fi
}

# Sub-module 6.2: TFTP Daemon / Socket Verification & Activation
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

# Sub-module 6.3: TFTP Payload Deployment
deploy_tftp_payload() {
    echo ""
    echo "[*] [6.3] Deploying initramfs to $TFTP_DIR/initramfs.cpio.gz..."
    sudo mkdir -p "$TFTP_DIR"
    sudo cp -f "$INITRAMFS_GZ" "$TFTP_DIR/initramfs.cpio.gz"
    echo "[+] Deployed successfully: $TFTP_DIR/initramfs.cpio.gz"
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
            echo "    sudo mkdir -p $TFTP_DIR && sudo cp $INITRAMFS_GZ $TFTP_DIR/initramfs.cpio.gz"
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
