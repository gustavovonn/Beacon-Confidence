#!/bin/sh
# ==============================================================================
# Three-Tier Computational Cost Benchmark Runner
# Designed for Bare-Metal Linux & BusyBox (Strict POSIX /bin/sh syntax)
# ==============================================================================

set -e

# Determine repository and directory paths
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [ -d "$SCRIPT_DIR/../f2md" ]; then
    ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
elif [ -d "$SCRIPT_DIR/../../f2md" ]; then
    ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
else
    ROOT_DIR="$(pwd)"
fi

# Auto-detect binary and results directory
if [ -x "/bench/offline_evaluator" ]; then
    BIN="/bench/offline_evaluator"
    RESULTS_DIR="/bench/results"
    DATA_PATH="/bench/data/ConstPos_0709.bin"
elif [ -x "$ROOT_DIR/f2md/build-riscv/offline_evaluator_stripped" ] && [ "$(uname -m)" = "riscv64" ]; then
    BIN="$ROOT_DIR/f2md/build-riscv/offline_evaluator_stripped"
    RESULTS_DIR="$ROOT_DIR/f2md/results"
    DATA_PATH="$ROOT_DIR/f2md/veremi/ConstPos_0709.bin"
elif [ -x "$ROOT_DIR/f2md/build/offline_evaluator" ]; then
    BIN="$ROOT_DIR/f2md/build/offline_evaluator"
    RESULTS_DIR="$ROOT_DIR/f2md/results"
    DATA_PATH="$ROOT_DIR/f2md/veremi/ConstPos_0709.bin"
    [ ! -f "$DATA_PATH" ] && DATA_PATH="$ROOT_DIR/f2md/veremi/ConstPos_0709.parquet"
    [ ! -f "$DATA_PATH" ] && DATA_PATH="$ROOT_DIR/f2md/veremi/ConstPos_0709_interpolated.parquet"
else
    BIN="offline_evaluator"
    RESULTS_DIR="./results"
    DATA_PATH="./ConstPos_0709.bin"
fi

MAX_MSGS=50000
REPEAT=1
TIER_MODE="all"
CORE_ID=""
ALL_PROFILES=0

# Parse CLI arguments
while [ $# -gt 0 ]; do
    case "$1" in
        --dir|-d)
            DATA_PATH="$2"
            shift 2
            ;;
        --file|-f)
            DATA_PATH="$2"
            shift 2
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
            echo "Options:"
            echo "  -d, --dir <path>       Directory or file with Parquet dataset"
            echo "  -n, --max-msgs <N>     Maximum messages to evaluate (default: 50000)"
            echo "  -r, --repeat <N>       Benchmark repetitions for stability (default: 1)"
            echo "  -t, --tier <1|2|3|all> Evaluation tier (default: all)"
            echo "  -c, --core <id>        Pin process to specific CPU core (via taskset)"
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

# Ensure binary exists
if [ ! -f "$BIN" ]; then
    echo "[!] Binary not found at $BIN. Building now..."
    cmake -B "$ROOT_DIR/f2md/build" -S "$ROOT_DIR/f2md" && cmake --build "$ROOT_DIR/f2md/build" -j$(nproc)
fi

mkdir -p "$RESULTS_DIR"

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
if [ -n "$CUSTOM_OUT" ]; then
    CSV_OUT="$CUSTOM_OUT"
else
    CSV_OUT="$RESULTS_DIR/benchmark_results_${TIMESTAMP}.csv"
fi

# Detect core isolation tool (taskset)
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
    # Flush OS page cache
    if [ -w /proc/sys/vm/drop_caches ]; then
        sync
        echo 3 > /proc/sys/vm/drop_caches
        echo "[*] Page caches flushed."
    fi
fi

echo "=============================================================================="
echo "          THREE-TIER COMPUTATIONAL COST BENCHMARK SUITE"
echo "=============================================================================="
echo " Dataset Path : $DATA_PATH"
echo " Max Messages : $MAX_MSGS"
echo " Repetitions  : $REPEAT"
echo " Tier Mode    : $TIER_MODE"
echo " Output CSV   : $CSV_OUT"
echo "=============================================================================="

# Cache Warmup run (discarded)
echo "[*] Performing cache warm-up pass (1,000 messages)..."
run_bench() {
    _tier="$1"
    _s1="$2"
    _s2="$3"
    _cur_data="$4"
    printf "[-] [%-16s] Tier %s: [%-10s + %-15s] ... " "$(basename "$_cur_data" | cut -d. -f1)" "$_tier" "$_s1" "$_s2"
    $CMD_PREFIX $BIN --tier "$_tier" --stage1 "$_s1" --stage2 "$_s2" \
        --dir "$_cur_data" --max-msgs "$MAX_MSGS" --repeat "$REPEAT" \
        --csv-out "$CSV_OUT" --profile >/dev/null 2>&1
    echo "Done."
}

run_suite_for_dataset() {
    _cur="$1"
    # Cache Warmup pass (discarded)
    $CMD_PREFIX $BIN --tier 1 --stage1 ols-ctra --dir "$_cur" --max-msgs 1000 >/dev/null 2>&1 || true

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
    DATA_DIR="$(dirname "$DATA_PATH")"
    echo "[*] Running comprehensive benchmark suite across all 8 rush-hour fault profiles in: $DATA_DIR"
    PROFILES="ConstPos_0709 ConstPosOffset_0709 ConstSpeed_0709 ConstSpeedOffset_0709 RandomPos_0709 RandomPosOffset_0709 RandomSpeed_0709 RandomSpeedOffset_0709"
    for prof in $PROFILES; do
        TARGET_FILE=""
        if [ -f "$DATA_DIR/${prof}.bin" ]; then
            TARGET_FILE="$DATA_DIR/${prof}.bin"
        elif [ -f "$DATA_DIR/${prof}.parquet" ]; then
            TARGET_FILE="$DATA_DIR/${prof}.parquet"
        fi
        if [ -n "$TARGET_FILE" ]; then
            echo ""
            echo ">>> [Profile: $prof] Ingesting $TARGET_FILE..."
            run_suite_for_dataset "$TARGET_FILE"
        else
            echo "[!] Warning: Profile $prof not found in $DATA_DIR"
        fi
    done
else
    run_suite_for_dataset "$DATA_PATH"
fi

printf "\n==============================================================================\n"
echo "                            BENCHMARK RESULTS SUMMARY"
echo "=============================================================================="

# Print summary table using awk (POSIX compatible)
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

# Compute and display Cross-Profile Aggregates (Mean +/- StdDev) if multi-profile
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

# Generate pure Markdown tables report
MD_OUT="${CSV_OUT%.csv}.md"

awk -F, -v dataset="$DATA_PATH" -v timestamp="$TIMESTAMP" -v maxmsgs="$MAX_MSGS" -v repeat="$REPEAT" -v csvfile="$(basename "$CSV_OUT")" '
BEGIN {
    print "# Benchmark Results"
    print ""
    print "- **Run Timestamp**: " timestamp
    print "- **Dataset**: `" dataset "`"
    print "- **Messages per Profile**: " maxmsgs
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
