#!/bin/sh
# ==============================================================================
# Three-Tier Computational Cost Benchmark Runner - RISC-V SBC Platform
# Targets: StarFive VisionFive 2 (JH7110 RV64GC), QEMU riscv64, BusyBox Ramdisk
# Strict POSIX /bin/sh syntax
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
F2MD_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Resolve binary with dynamic fallback hierarchy (or override with -b)
DEFAULT_BIN=""
if [ -x "$SCRIPT_DIR/offline_evaluator" ]; then
    DEFAULT_BIN="$SCRIPT_DIR/offline_evaluator"
elif [ -x "./offline_evaluator" ]; then
    DEFAULT_BIN="./offline_evaluator"
elif [ -x "$F2MD_DIR/build-riscv/offline_evaluator_stripped" ]; then
    DEFAULT_BIN="$F2MD_DIR/build-riscv/offline_evaluator_stripped"
elif [ -x "$F2MD_DIR/build-riscv/offline_evaluator" ]; then
    DEFAULT_BIN="$F2MD_DIR/build-riscv/offline_evaluator"
elif [ -x "$F2MD_DIR/build/offline_evaluator" ]; then
    DEFAULT_BIN="$F2MD_DIR/build/offline_evaluator"
fi

# Resolve default binary trace dataset (.bin only for RISC-V zero-dependency)
DEFAULT_DATA=""
if [ -f "$SCRIPT_DIR/data/ConstPos_0709.bin" ]; then
    DEFAULT_DATA="$SCRIPT_DIR/data/ConstPos_0709.bin"
elif [ -f "./data/ConstPos_0709.bin" ]; then
    DEFAULT_DATA="./data/ConstPos_0709.bin"
elif [ -f "$F2MD_DIR/veremi/bin/ConstPos_0709.bin" ]; then
    DEFAULT_DATA="$F2MD_DIR/veremi/bin/ConstPos_0709.bin"
fi

# Results directory
if [ -d "$F2MD_DIR/results" ]; then
    RESULTS_DIR="$F2MD_DIR/results"
elif [ -d "$SCRIPT_DIR/results" ]; then
    RESULTS_DIR="$SCRIPT_DIR/results"
else
    RESULTS_DIR="./results"
fi

BIN=""
DATA_PATH=""
MAX_MSGS=50000
REPEAT=1
TIER_MODE="all"
CORE_ID=""
ALL_PROFILES=1
CUSTOM_OUT=""

# Parse CLI arguments
while [ $# -gt 0 ]; do
    case "$1" in
        --bin|-b)
            BIN="$2"
            shift 2
            ;;
        --dir|-d)
            DATA_PATH="$2"
            ALL_PROFILES=1
            shift 2
            ;;
        --file|-f)
            DATA_PATH="$2"
            ALL_PROFILES=0
            shift 2
            ;;
        --profile|-p)
            DATA_PATH="$2"
            ALL_PROFILES=0
            shift 2
            ;;
        --single)
            ALL_PROFILES=0
            shift
            ;;
        --max-msgs|-n)
            MAX_MSGS="$2"
            shift 2
            ;;
        --repeat|-r)
            REPEAT="$2"
            shift 2
            ;;
        --tier|-t)
            TIER_MODE="$2"
            shift 2
            ;;
        --core|-c)
            CORE_ID="$2"
            shift 2
            ;;
        --all-profiles|-a)
            ALL_PROFILES=1
            shift
            ;;
        --out|-o)
            CUSTOM_OUT="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -a, --all-profiles     Run across all 8 rush-hour fault profiles (DEFAULT)"
            echo "  -p, --profile <name>   Run single specific profile (e.g. ConstPos_0709)"
            echo "  -f, --file <path>      Path to single binary trace (.bin)"
            echo "  -d, --dir <path>       Directory containing binary traces (e.g. veremi/bin or data/)"
            echo "  -b, --bin <path>       Path to offline_evaluator RISC-V binary (default: $DEFAULT_BIN)"
            echo "  -t, --tier <1|2|3|all> Evaluation tier (default: all)"
            echo "  -n, --max-msgs <N>     Maximum messages to evaluate (default: 50000)"
            echo "  -r, --repeat <N>       Repetitions for stability (default: 1)"
            echo "  -c, --core <id>        Pin process to specific CPU core (e.g. 1-3 on Quad-Core SBC)"
            echo "  -o, --out <csv_path>   Custom path for CSV output"
            echo "  -h, --help             Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

# Resolve binary
[ -z "$BIN" ] && BIN="$DEFAULT_BIN"

if [ ! -f "$BIN" ]; then
    echo "[!] Error: RISC-V evaluator binary not found at '$BIN'."
    echo "    Specify via --bin <path> or compile using cmake -B build-riscv -S . -DUSE_ARROW=OFF -DSTATIC_BUILD=ON"
    exit 1
fi

if [ ! -x "$BIN" ]; then
    echo "[!] Error: Binary is not executable: $BIN"
    exit 1
fi

# Resolve dataset
if [ "$ALL_PROFILES" = "0" ]; then
    [ -z "$DATA_PATH" ] && DATA_PATH="$DEFAULT_DATA"
    if [ ! -e "$DATA_PATH" ]; then
        echo "[!] Error: Dataset not found at '$DATA_PATH'."
        echo "    Specify via --file <path.bin> or prepare datasets in veremi/bin/"
        exit 1
    fi
fi

mkdir -p "$RESULTS_DIR"

TIMESTAMP=$(date +"%Y%m%d_%H%M%S" 2>/dev/null || date +"%s")
if [ -n "$CUSTOM_OUT" ]; then
    CSV_OUT="$CUSTOM_OUT"
else
    CSV_OUT="$RESULTS_DIR/benchmark_riscv_${TIMESTAMP}.csv"
fi

# Core pinning (taskset)
CMD_PREFIX=""
if [ -n "$CORE_ID" ]; then
    if command -v taskset >/dev/null 2>&1; then
        CMD_PREFIX="taskset -c $CORE_ID"
        echo "[*] Pinning benchmark process to CPU Core $CORE_ID via taskset"
    else
        echo "[!] Warning: taskset not available, running without core affinity."
    fi
fi

# Elevate priority if root
if [ "$(id -u)" = "0" ]; then
    if command -v nice >/dev/null 2>&1; then
        CMD_PREFIX="nice -n -20 $CMD_PREFIX"
        echo "[*] Running with maximum process priority (nice -20)"
    fi
    if [ -w /proc/sys/vm/drop_caches ]; then
        sync
        echo 3 > /proc/sys/vm/drop_caches
        echo "[*] Page caches flushed."
    fi
fi

echo "=============================================================================="
echo "    THREE-TIER COMPUTATIONAL COST BENCHMARK SUITE (STARFIVE RISC-V SBC)"
echo "=============================================================================="
echo " Architecture     : $(uname -m) (Kernel: $(uname -r))"
echo " Evaluator Binary : $BIN"
if [ "$ALL_PROFILES" = "1" ]; then
    echo " Target Dataset   : All 8 Rush-Hour Profiles (.bin)"
else
    echo " Target Dataset   : $DATA_PATH"
fi
echo " Max Messages     : $MAX_MSGS"
echo " Repetitions      : $REPEAT"
echo " Tier Mode        : $TIER_MODE"
echo " Output CSV       : $CSV_OUT"
echo "=============================================================================="

run_bench() {
    _tier="$1"
    _s1="$2"
    _s2="$3"
    _cur_data="$4"
    if [ ! -f "$_cur_data" ]; then
        echo "[!] Error: Target data file does not exist: $_cur_data"
        exit 1
    fi
    printf "[-] [%-16s] Tier %s: [%-10s + %-15s] ... " "$(basename "$_cur_data" | cut -d. -f1)" "$_tier" "$_s1" "$_s2"
    _out=$($CMD_PREFIX "$BIN" --tier "$_tier" --stage1 "$_s1" --stage2 "$_s2" \
        --dir "$_cur_data" --max-msgs "$MAX_MSGS" --repeat "$REPEAT" \
        --csv-out "$CSV_OUT" --profile 2>&1)
    _status=$?
    if [ $_status -ne 0 ]; then
        echo "FAILED (exit code $_status)!"
        echo "$_out" | sed 's/^/    /'
        exit 1
    fi
    echo "Done."
}

run_suite_for_dataset() {
    _cur="$1"
    # Warm-up pass (discarded)
    $CMD_PREFIX "$BIN" --tier 1 --stage1 ols-ctra --dir "$_cur" --max-msgs 1000 >/dev/null 2>&1 || true

    if [ "$TIER_MODE" = "1" ] || [ "$TIER_MODE" = "all" ]; then
        run_bench "1" "ols-ctra" "beta-reputation" "$_cur"
        run_bench "1" "catch" "threshold" "$_cur"
        run_bench "1" "experi" "threshold" "$_cur"
        run_bench "1" "legacy" "threshold" "$_cur"
    fi

    if [ "$TIER_MODE" = "2" ] || [ "$TIER_MODE" = "all" ]; then
        run_bench "2" "ols-ctra" "beta-reputation" "$_cur"
        run_bench "2" "legacy" "threshold" "$_cur"
        run_bench "2" "legacy" "behavioral" "$_cur"
        run_bench "2" "legacy" "aggregation" "$_cur"
    fi

    if [ "$TIER_MODE" = "3" ] || [ "$TIER_MODE" = "all" ]; then
        run_bench "3" "ols-ctra" "beta-reputation" "$_cur"
        run_bench "3" "legacy" "threshold" "$_cur"
        run_bench "3" "legacy" "behavioral" "$_cur"
        run_bench "3" "catch" "behavioral" "$_cur"
        run_bench "3" "experi" "threshold" "$_cur"
    fi
}

if [ "$ALL_PROFILES" = "1" ]; then
    if [ -n "$DATA_PATH" ] && [ -d "$DATA_PATH" ]; then
        DATA_DIR="$DATA_PATH"
    elif [ -n "$DATA_PATH" ] && [ -f "$DATA_PATH" ]; then
        DATA_DIR="$(dirname "$DATA_PATH")"
    elif [ -d "$SCRIPT_DIR/data" ]; then
        DATA_DIR="$SCRIPT_DIR/data"
    elif [ -d "./data" ]; then
        DATA_DIR="./data"
    elif [ -d "$F2MD_DIR/veremi/bin" ]; then
        DATA_DIR="$F2MD_DIR/veremi/bin"
    else
        DATA_DIR="./data"
    fi
    echo "[*] Running comprehensive benchmark suite across all 8 rush-hour fault profiles..."
    PROFILES="ConstPos_0709 ConstPosOffset_0709 ConstSpeed_0709 ConstSpeedOffset_0709 RandomPos_0709 RandomPosOffset_0709 RandomSpeed_0709 RandomSpeedOffset_0709"
    for prof in $PROFILES; do
        TARGET_FILE=""
        if [ -f "$DATA_DIR/bin/${prof}.bin" ]; then
            TARGET_FILE="$DATA_DIR/bin/${prof}.bin"
        elif [ -f "$DATA_DIR/${prof}.bin" ]; then
            TARGET_FILE="$DATA_DIR/${prof}.bin"
        elif [ -f "$SCRIPT_DIR/data/${prof}.bin" ]; then
            TARGET_FILE="$SCRIPT_DIR/data/${prof}.bin"
        elif [ -f "./data/${prof}.bin" ]; then
            TARGET_FILE="./data/${prof}.bin"
        elif [ -f "$F2MD_DIR/veremi/bin/${prof}.bin" ]; then
            TARGET_FILE="$F2MD_DIR/veremi/bin/${prof}.bin"
        fi

        if [ -n "$TARGET_FILE" ]; then
            echo ""
            echo ">>> [Profile: $prof] Ingesting $TARGET_FILE..."
            run_suite_for_dataset "$TARGET_FILE"
        else
            echo "[!] Warning: Profile $prof not found in $DATA_DIR, $SCRIPT_DIR/data, or $F2MD_DIR/veremi/bin"
        fi
    done
else
    run_suite_for_dataset "$DATA_PATH"
fi

printf "\n==============================================================================\n"
echo "                            BENCHMARK RESULTS SUMMARY"
echo "=============================================================================="

awk -F, '
BEGIN {
    printf "%-18s | %-4s | %-10s | %-15s | %-10s | %-10s | %-10s | %-12s\n", "Dataset", "Tier", "Stage 1", "Stage 2", "S1 Lat(us)", "S2 Lat(us)", "Tot Lat(us)", "Msgs/sec"
    print "-------------------+------+------------+-----------------+------------+------------+------------+-------------"
}
NR > 1 {
    printf "%-18s | %-4s | %-10s | %-15s | %10.4f | %10.4f | %10.4f | %12.0f\n", $1, $2, $3, $4, $6, $7, $8, $9
}
' "$CSV_OUT"

echo "=============================================================================="
echo "[+] Full raw dataset saved to: $CSV_OUT"

awk -F, '
NR > 1 {
    cfg = sprintf("T%s: %-10s + %-15s", $2, $3, $4)
    count[cfg]++
    sum_tot[cfg] += $8
    sumsq_tot[cfg] += ($8 * $8)
    sum_fps[cfg] += $9
    sumsq_fps[cfg] += ($9 * $9)
}
END {
    if (NR > 1) {
        printf "\n==============================================================================\n"
        printf "             CROSS-PROFILE STATISTICAL SUMMARY (MEAN +- STD DEV)\n"
        printf "==============================================================================\n"
        printf "%-32s | %-6s | %-20s | %-20s\n", "Pipeline Configuration", "Runs", "Total Latency (us)", "Throughput (msgs/sec)"
        print "---------------------------------+--------+----------------------+---------------------"
        for (cfg in count) {
            n = count[cfg]
            mean_lat = sum_tot[cfg] / n
            var_lat = (n > 1) ? (sumsq_tot[cfg] - (sum_tot[cfg] * sum_tot[cfg] / n)) / (n - 1) : 0
            std_lat = (var_lat > 0) ? sqrt(var_lat) : 0

            mean_fps = sum_fps[cfg] / n
            var_fps = (n > 1) ? (sumsq_fps[cfg] - (sum_fps[cfg] * sum_fps[cfg] / n)) / (n - 1) : 0
            std_fps = (var_fps > 0) ? sqrt(var_fps) : 0

            printf "%-32s | %6d | %8.4f +- %-8.4f | %10.0f +- %-8.0f\n", cfg, n, mean_lat, std_lat, mean_fps, std_fps
        }
        printf "==============================================================================\n"
    }
}
' "$CSV_OUT"

MD_OUT="${CSV_OUT%.csv}.md"
awk -F, -v dataset="$DATA_PATH" -v timestamp="$TIMESTAMP" -v maxmsgs="$MAX_MSGS" -v repeat="$REPEAT" -v csvfile="$(basename "$CSV_OUT")" '
BEGIN {
    print "# Benchmark Results (StarFive VisionFive 2 RISC-V SBC)"
    print ""
    print "- **Run Timestamp**: " timestamp
    print "- **Dataset**: `" dataset "`"
    print "- **Messages per Profile**: " maxmsgs
    print "- **Repetitions**: " repeat
    print "- **CSV**: [`" csvfile "`](" csvfile ")"
    print ""
    print "## Complete Results Table"
    print ""
    print "| Dataset | Tier | Stage 1 | Stage 2 | Messages | S1 Latency (us) | S2 Latency (us) | Total Latency (us) | Throughput (msgs/sec) | Total Time (ms) |"
    print "| :--- | :---: | :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |"
}
NR > 1 {
    printf "| %s | %s | %s | %s | %s | %10.4f | %10.4f | %10.4f | %12.1f | %10.2f |\n", $1, $2, $3, $4, $5, $6, $7, $8, $9, $10
}
' "$CSV_OUT" > "$MD_OUT"

echo "[+] Markdown report saved to: $MD_OUT"
echo "=============================================================================="
