#!/usr/bin/env bash
# ==============================================================================
# Object-trajectory confidence - cost benchmark runner, x86-64 host
# ==============================================================================
#
# The reference run. What the board produces only means something next to this.
#
# THE BENCHMARK POINTS BELOW MUST STAY IN STEP WITH run_benchmarks_riscv.sh.
# The whole purpose is comparing one architecture against the other, and that comparison is void
# the moment the two scripts measure different things.
# If you add a point there, add it here in the same order and with the same label.
#
# This one builds first, because on the host the sources are right next to it.
# The RISC-V runner cannot: it ships inside an initramfs with no compiler.
# ==============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BASE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$BASE_DIR"

BUILD_DIR="${BUILD_DIR:-$BASE_DIR/build}"
BIN="${BIN:-$BUILD_DIR/cost_evaluator}"
RUNNER="${RUNNER:-$BUILD_DIR/experiment_runner}"
DATA_DIR="${DATA_DIR:-$BASE_DIR/dataset/groundtruth}"
RESULTS_DIR="${RESULTS_DIR:-$BASE_DIR/results/vf2_compare}"

MESSAGES=20000
REPEAT=1
BUDGET_US=100000
SKIP_TESTS=0
SKIP_SWEEP=0
SKIP_BUILD=0
CORE_ID=""
LABEL="x86"

while [ $# -gt 0 ]; do
    case "$1" in
        --messages|-n) MESSAGES="$2"; shift 2 ;;
        --repeat|-r)   REPEAT="$2"; shift 2 ;;
        --budget-us)   BUDGET_US="$2"; shift 2 ;;
        --core|-c)     CORE_ID="$2"; shift 2 ;;
        --label|-l)    LABEL="$2"; shift 2 ;;
        --results|-o)  RESULTS_DIR="$2"; shift 2 ;;
        --skip-tests)  SKIP_TESTS=1; shift ;;
        --skip-sweep)  SKIP_SWEEP=1; shift ;;
        --skip-build)  SKIP_BUILD=1; shift ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "  -n, --messages <N>   Messages per timed point (default: $MESSAGES)"
            echo "  -r, --repeat <N>     Repeat each point N times (default: $REPEAT)"
            echo "      --budget-us <US> Per-message deadline to judge against (default: $BUDGET_US)"
            echo "  -c, --core <ID>      Pin to one CPU core"
            echo "  -l, --label <NAME>   Tag for the result files (default: $LABEL)"
            echo "  -o, --results <DIR>  Where to write results (default: $RESULTS_DIR)"
            echo "      --skip-tests     Do not run the test suites first"
            echo "      --skip-sweep     Do not run the N x m sweep"
            echo "      --skip-build     Use whatever is already in $BUILD_DIR"
            echo "  -h, --help           Show this help"
            exit 0
            ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

if [ "$SKIP_BUILD" != "1" ]; then
    echo "[*] Building for the host..."
    make -s all || { echo "[!] build failed"; exit 1; }
fi

if [ ! -x "$BIN" ]; then
    echo "[!] ERROR: cost_evaluator not found or not executable at $BIN"
    exit 1
fi

mkdir -p "$RESULTS_DIR"
SUMMARY="$RESULTS_DIR/summary_${LABEL}.txt"
: > "$SUMMARY"

PIN=""
if [ -n "$CORE_ID" ] && command -v taskset >/dev/null 2>&1; then
    PIN="taskset -c $CORE_ID"
fi

echo "=============================================================================="
echo "   OBJECT-TRAJECTORY CONFIDENCE - COST BENCHMARK ($LABEL)"
echo "=============================================================================="
echo " Machine      : $(uname -m)  kernel $(uname -r)"
echo " CPU          : $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | sed 's/^ *//')"
echo " Cores        : $(grep -c '^processor' /proc/cpuinfo)"
echo " Messages     : $MESSAGES per point, repeated $REPEAT time(s)"
echo " Budget       : $BUDGET_US us per message"
echo " Pinning      : ${PIN:-none}"
echo " Results      : $RESULTS_DIR"
echo "=============================================================================="

run_tests() {
    if [ "$SKIP_TESTS" = "1" ]; then
        echo ""
        echo "[*] [1/3] Skipping test suites (--skip-tests)."
        return 0
    fi
    echo ""
    echo "[*] [1/3] Running the test suites on this machine..."
    local failed=0
    for t in test_lab_ctra test_core test_pipeline test_dataset; do
        if [ ! -x "$BUILD_DIR/$t" ]; then
            echo "    $t: not built, skipped"
            continue
        fi
        local out="$RESULTS_DIR/${LABEL}_${t}.log"
        if $PIN "$BUILD_DIR/$t" > "$out" 2>&1; then
            echo "    $t: $(grep 'checks,' "$out" | tail -1)"
        else
            echo "    $t: FAILED  (see $out)"
            failed=$((failed + 1))
        fi
    done
    if [ "$failed" -gt 0 ]; then
        echo "[!] $failed suite(s) failed. The cost numbers below are suspect."
        echo "TESTS FAILED on $LABEL: $failed suite(s)" >> "$SUMMARY"
    else
        echo "[+] All suites passed."
        echo "TESTS PASSED on $LABEL" >> "$SUMMARY"
    fi
}

# KEEP THIS LIST IN STEP WITH run_benchmarks_riscv.sh.
POINTS="sparse:25:1 operating:61:5 dense:150:11"

one_point() {
    local label="$1" tracks="$2" vectors="$3" iter="$4"
    local out="$RESULTS_DIR/${LABEL}_${label}_r${iter}.log"

    $PIN "$BIN" --tracks "$tracks" --vectors "$vectors" \
                --messages "$MESSAGES" --budget-us "$BUDGET_US" > "$out" 2>&1

    awk -v lab="$label" -v n="$tracks" -v m="$vectors" -v it="$iter" '
        $1 == "onMotionVectors" {
            printf "POINT %-10s N=%-4s m=%-3s run=%s mean=%8.2f p50=%8.2f p99=%8.2f max=%9.2f\n",
                   lab, n, m, it, $3, $4, $5, $6
        }' "$out"
}

run_points() {
    echo ""
    echo "[*] [2/3] Timed points (microseconds per message)..."
    echo ""
    for spec in $POINTS; do
        local label tracks vectors
        label=$(echo "$spec" | cut -d: -f1)
        tracks=$(echo "$spec" | cut -d: -f2)
        vectors=$(echo "$spec" | cut -d: -f3)
        local i=1
        while [ "$i" -le "$REPEAT" ]; do
            local line
            line=$(one_point "$label" "$tracks" "$vectors" "$i")
            echo "    $line"
            echo "$line" >> "$SUMMARY"
            i=$((i + 1))
        done
    done

    local oplog="$RESULTS_DIR/${LABEL}_operating_r1.log"
    if [ -f "$oplog" ]; then
        echo ""
        grep -E "budget|headroom" "$oplog" | sed 's/^/    /'
        grep -E "headroom" "$oplog" | sed "s/^/VERDICT $LABEL /" >> "$SUMMARY"
    fi
}

run_sweep() {
    if [ "$SKIP_SWEEP" = "1" ]; then
        echo ""
        echo "[*] [3/3] Skipping the sweep (--skip-sweep)."
        return 0
    fi
    echo ""
    echo "[*] [3/3] Scale sweep (whole-pipeline p99 against N and m)..."
    local sweeplog="$RESULTS_DIR/${LABEL}_sweep.log"
    $PIN "$BIN" --sweep --budget-us "$BUDGET_US" > "$sweeplog" 2>&1
    sed -n '/SWEEP/,/^$/p' "$sweeplog" | sed 's/^/    /'

    if [ -x "$RUNNER" ] && [ -f "$DATA_DIR/cam.csv" ]; then
        echo ""
        echo "[*] Real-trace replay over $DATA_DIR..."
        local replaylog="$RESULTS_DIR/${LABEL}_replay.log"
        $PIN "$RUNNER" --dataset "$DATA_DIR" --no-csv --label "$LABEL" > "$replaylog" 2>&1
        grep -E "^BATCH|motion vectors|accuracy" "$replaylog" | sed 's/^/    /'
        grep "^BATCH" "$replaylog" >> "$SUMMARY"
    fi
}

run_tests
run_points
run_sweep

echo ""
echo "=============================================================================="
echo "[+] Done. Summary written to $SUMMARY"
echo "    Once the board has run, compare with:"
echo "    scripts/compare_benchmarks.sh $SUMMARY <summary_riscv.txt>"
echo "=============================================================================="
