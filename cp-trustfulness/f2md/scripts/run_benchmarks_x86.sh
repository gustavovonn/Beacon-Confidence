#!/bin/sh
# ==============================================================================
# Three-Tier Computational Cost Benchmark Runner - x86_64 Host Platform
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
F2MD_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Default paths
DEFAULT_BIN="$F2MD_DIR/build/offline_evaluator"
[ ! -x "$DEFAULT_BIN" ] && [ -x "$F2MD_DIR/build-standalone/offline_evaluator" ] && DEFAULT_BIN="$F2MD_DIR/build-standalone/offline_evaluator"

DEFAULT_DATA="$F2MD_DIR/veremi/bin/ConstPos_0709.bin"
[ ! -f "$DEFAULT_DATA" ] && DEFAULT_DATA="$F2MD_DIR/veremi/parquet/ConstPos_0709.parquet"

RESULTS_DIR="$F2MD_DIR/results"

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
            echo "  -f, --file <path>      Path to single dataset file (.bin or .parquet)"
            echo "  -d, --dir <path>       Directory containing datasets (e.g. veremi/bin or veremi/parquet)"
            echo "  -b, --bin <path>       Path to offline_evaluator binary (default: $DEFAULT_BIN)"
            echo "  -t, --tier <1|2|3|all> Evaluation tier (default: all)"
            echo "  -n, --max-msgs <N>     Maximum messages to evaluate (default: 50000)"
            echo "  -r, --repeat <N>       Repetitions for stability (default: 1)"
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

# Resolve binary
[ -z "$BIN" ] && BIN="$DEFAULT_BIN"

if [ ! -f "$BIN" ]; then
    echo "[!] Binary not found at $BIN. Attempting to build host target..."
    cmake -B "$F2MD_DIR/build" -S "$F2MD_DIR" && cmake --build "$F2MD_DIR/build" -j"$(nproc)"
fi

if [ ! -x "$BIN" ]; then
    echo "[!] Error: Binary is not executable: $BIN"
    exit 1
fi

# ------------------------------------------------------------------------------
# Python & Dataset Environment Resolution
# ------------------------------------------------------------------------------
resolve_python() {
    REPO_ROOT="$(cd "$F2MD_DIR/../.." && pwd)"
    if [ -n "$VIRTUAL_ENV" ] && [ -x "$VIRTUAL_ENV/bin/python" ]; then
        PYTHON_BIN="$VIRTUAL_ENV/bin/python"
    elif [ -x "$F2MD_DIR/.venv/bin/python" ]; then
        PYTHON_BIN="$F2MD_DIR/.venv/bin/python"
    elif [ -x "$REPO_ROOT/.venv/bin/python" ]; then
        PYTHON_BIN="$REPO_ROOT/.venv/bin/python"
    elif [ -x "$REPO_ROOT/veremi-extension/.venv/bin/python" ]; then
        PYTHON_BIN="$REPO_ROOT/veremi-extension/.venv/bin/python"
    else
        PYTHON_BIN="python3"
    fi

    # Check if Python environment has required modules
    if ! "$PYTHON_BIN" -c "import pyarrow, scipy, pandas" >/dev/null 2>&1; then
        echo "[*] Python dependencies (pyarrow, scipy, pandas) not detected in $PYTHON_BIN."
        echo "[*] Bootstrapping virtual environment in $F2MD_DIR/.venv..."
        python3 -m venv "$F2MD_DIR/.venv"
        "$F2MD_DIR/.venv/bin/pip" install --quiet -r "$F2MD_DIR/requirements.txt"
        PYTHON_BIN="$F2MD_DIR/.venv/bin/python"
    fi
}

ensure_datasets() {
    PROFILES="ConstPos_0709 ConstPosOffset_0709 ConstSpeed_0709 ConstSpeedOffset_0709 RandomPos_0709 RandomPosOffset_0709 RandomSpeed_0709 RandomSpeedOffset_0709"

    if [ "$ALL_PROFILES" = "1" ]; then
        MISSING=0
        for p in $PROFILES; do
            if [ ! -f "$F2MD_DIR/veremi/parquet/${p}.parquet" ] && [ ! -f "$F2MD_DIR/veremi/bin/${p}.bin" ]; then
                MISSING=1
                break
            fi
        done
        if [ "$MISSING" = "1" ]; then
            echo "[!] Benchmark datasets missing for all profiles. Automatically invoking preparation pipeline..."
            resolve_python
            "$PYTHON_BIN" "$SCRIPT_DIR/interpolate_veremi.py" --all-profiles
            "$PYTHON_BIN" "$SCRIPT_DIR/export_parquet_to_bin.py"
        fi
    else
        TARGET_FILE="$DATA_PATH"
        [ -z "$TARGET_FILE" ] && TARGET_FILE="$DEFAULT_DATA"

        if [ ! -e "$TARGET_FILE" ]; then
            PROF_NAME="$(basename "$TARGET_FILE" | cut -d. -f1)"
            [ -z "$PROF_NAME" ] && PROF_NAME="ConstPos_0709"

            echo "[!] Dataset not found at: $TARGET_FILE"
            echo "[*] Automatically invoking ETL pipeline for profile '$PROF_NAME'..."
            resolve_python
            "$PYTHON_BIN" "$SCRIPT_DIR/interpolate_veremi.py" --profile "$PROF_NAME"
            "$PYTHON_BIN" "$SCRIPT_DIR/export_parquet_to_bin.py"

            # Re-check target
            if [ -f "$F2MD_DIR/veremi/parquet/${PROF_NAME}.parquet" ]; then
                DATA_PATH="$F2MD_DIR/veremi/parquet/${PROF_NAME}.parquet"
            elif [ -f "$F2MD_DIR/veremi/bin/${PROF_NAME}.bin" ]; then
                DATA_PATH="$F2MD_DIR/veremi/bin/${PROF_NAME}.bin"
            else
                echo "[!] Error: Failed to generate dataset '$PROF_NAME'."
                echo "    Please verify raw simulation traces exist in veremi-extension/data/"
                exit 1
            fi
        fi
    fi
}

ensure_datasets

# Resolve final dataset path
[ -z "$DATA_PATH" ] && DATA_PATH="$DEFAULT_DATA"

if [ "$ALL_PROFILES" = "0" ] && [ ! -e "$DATA_PATH" ]; then
    echo "[!] Error: Dataset not found at '$DATA_PATH'."
    exit 1
fi

mkdir -p "$RESULTS_DIR"

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
if [ -n "$CUSTOM_OUT" ]; then
    CSV_OUT="$CUSTOM_OUT"
else
    CSV_OUT="$RESULTS_DIR/benchmark_x86_${TIMESTAMP}.csv"
fi

# Core pinning
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
echo "      THREE-TIER COMPUTATIONAL COST BENCHMARK SUITE (x86_64 HOST)"
echo "=============================================================================="
echo " Evaluator Binary : $BIN"
if [ "$ALL_PROFILES" = "1" ]; then
    echo " Target Dataset   : All 8 Rush-Hour Profiles (veremi/bin or veremi/parquet)"
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
    if [ -d "$DATA_PATH" ]; then
        DATA_DIR="$DATA_PATH"
    else
        DATA_DIR="$(dirname "$DATA_PATH")"
    fi
    echo "[*] Running comprehensive benchmark suite across all 8 rush-hour fault profiles..."
    PROFILES="ConstPos_0709 ConstPosOffset_0709 ConstSpeed_0709 ConstSpeedOffset_0709 RandomPos_0709 RandomPosOffset_0709 RandomSpeed_0709 RandomSpeedOffset_0709"
    for prof in $PROFILES; do
        TARGET_FILE=""
        if [ -f "$DATA_DIR/bin/${prof}.bin" ]; then
            TARGET_FILE="$DATA_DIR/bin/${prof}.bin"
        elif [ -f "$DATA_DIR/parquet/${prof}.parquet" ]; then
            TARGET_FILE="$DATA_DIR/parquet/${prof}.parquet"
        elif [ -f "$DATA_DIR/${prof}.bin" ]; then
            TARGET_FILE="$DATA_DIR/${prof}.bin"
        elif [ -f "$DATA_DIR/${prof}.parquet" ]; then
            TARGET_FILE="$DATA_DIR/${prof}.parquet"
        elif [ -f "$F2MD_DIR/veremi/bin/${prof}.bin" ]; then
            TARGET_FILE="$F2MD_DIR/veremi/bin/${prof}.bin"
        elif [ -f "$F2MD_DIR/veremi/parquet/${prof}.parquet" ]; then
            TARGET_FILE="$F2MD_DIR/veremi/parquet/${prof}.parquet"
        fi

        if [ -n "$TARGET_FILE" ]; then
            echo ""
            echo ">>> [Profile: $prof] Ingesting $TARGET_FILE..."
            run_suite_for_dataset "$TARGET_FILE"
        else
            echo "[!] Warning: Profile $prof not found in $DATA_DIR or $F2MD_DIR/veremi/(bin|parquet)"
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
    print "# Benchmark Results (x86_64 Host)"
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
