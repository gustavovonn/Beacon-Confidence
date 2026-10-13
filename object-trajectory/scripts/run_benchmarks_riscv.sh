#!/bin/sh
# ==============================================================================
# Object-trajectory confidence - cost benchmark runner, RISC-V SBC platform
# Targets: StarFive VisionFive 2 (JH7110 RV64GC), QEMU riscv64, BusyBox ramdisk
# Strict POSIX /bin/sh syntax
# ==============================================================================
#
# PROVENANCE
# Follows cp-trustfulness/f2md/scripts/run_benchmarks_riscv.sh in shape and invocation, so that
# both experiments are driven the same way on the same board.
#
# THE BENCHMARK POINTS BELOW MUST STAY IN STEP WITH run_benchmarks_x86.sh.
# The whole purpose is comparing one architecture against the other, and that comparison is void
# the moment the two scripts measure different things.
# If you add a point here, add it there in the same order and with the same label.
# ==============================================================================

set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

BIN="${BIN:-$SCRIPT_DIR/cost_evaluator}"
RUNNER="${RUNNER:-$SCRIPT_DIR/experiment_runner}"
DATA_DIR="${DATA_DIR:-$SCRIPT_DIR/data}"
RESULTS_DIR="${RESULTS_DIR:-$SCRIPT_DIR/results}"

MESSAGES=20000
REPEAT=1
BUDGET_US=100000
SKIP_TESTS=0
SKIP_SWEEP=0
CORE_ID=""
LABEL="riscv"

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
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "  -n, --messages <N>   Messages per timed point (default: $MESSAGES)"
            echo "  -r, --repeat <N>     Repeat each point N times (default: $REPEAT)"
            echo "      --budget-us <US> Per-message deadline to judge against (default: $BUDGET_US)"
            echo "  -c, --core <ID>      Pin to one CPU core, if taskset exists"
            echo "  -l, --label <NAME>   Tag for the result files (default: $LABEL)"
            echo "  -o, --results <DIR>  Where to write results (default: $RESULTS_DIR)"
            echo "      --skip-tests     Do not run the test suites first"
            echo "      --skip-sweep     Do not run the N x m sweep"
            echo "  -h, --help           Show this help"
            exit 0
            ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

if [ ! -x "$BIN" ]; then
    echo "[!] ERROR: cost_evaluator not found or not executable at $BIN"
    exit 1
fi

mkdir -p "$RESULTS_DIR"
SUMMARY="$RESULTS_DIR/summary_${LABEL}.txt"
: > "$SUMMARY"

# Pin to one core when possible.
# A benchmark that migrates between cores measures the scheduler as much as the code.
PIN=""
if [ -n "$CORE_ID" ] && command -v taskset >/dev/null 2>&1; then
    PIN="taskset -c $CORE_ID"
fi

echo "=============================================================================="
echo "   OBJECT-TRAJECTORY CONFIDENCE - COST BENCHMARK ($LABEL)"
echo "=============================================================================="
echo " Machine      : $(uname -m)  kernel $(uname -r)"
echo " CPU          : $(grep -m1 -E 'uarch|model name' /proc/cpuinfo 2>/dev/null | cut -d: -f2- | sed 's/^ *//')"
echo " ISA          : $(grep -m1 'isa' /proc/cpuinfo 2>/dev/null | cut -d: -f2- | sed 's/^ *//')"
echo " Cores        : $(grep -c '^processor' /proc/cpuinfo 2>/dev/null)"
echo " Messages     : $MESSAGES per point, repeated $REPEAT time(s)"
echo " Budget       : $BUDGET_US us per message"
echo " Pinning      : ${PIN:-none}"
echo " Results      : $RESULTS_DIR"
echo "=============================================================================="

# ------------------------------------------------------------------------------
# Stage 1: the test suites
# ------------------------------------------------------------------------------
# A cost number from a binary whose own tests have not passed on this silicon is worth little.
# Cross-compilation can change floating-point behaviour, and these suites would catch that.
run_tests() {
    if [ "$SKIP_TESTS" = "1" ]; then
        echo ""
        echo "[*] [1/3] Skipping test suites (--skip-tests)."
        return 0
    fi

    echo ""
    echo "[*] [1/3] Running the test suites on this machine..."
    FAILED=0
    for t in test_lab_ctra test_core test_pipeline test_dataset; do
        if [ ! -x "$SCRIPT_DIR/$t" ]; then
            echo "    $t: not staged, skipped"
            continue
        fi
        OUT="$RESULTS_DIR/${LABEL}_${t}.log"
        if $PIN "$SCRIPT_DIR/$t" > "$OUT" 2>&1; then
            echo "    $t: $(grep 'checks,' "$OUT" | tail -1)"
        else
            echo "    $t: FAILED  (see $OUT)"
            FAILED=$((FAILED + 1))
        fi
    done
    if [ "$FAILED" -gt 0 ]; then
        echo "[!] $FAILED suite(s) failed on this machine. The cost numbers below are suspect."
        echo "TESTS FAILED on $LABEL: $FAILED suite(s)" >> "$SUMMARY"
    else
        echo "[+] All staged suites passed."
        echo "TESTS PASSED on $LABEL" >> "$SUMMARY"
    fi
}

# ------------------------------------------------------------------------------
# Stage 2: the timed points
# ------------------------------------------------------------------------------
# Each point is one (N, m) pair: N trajectories believed, m motion vectors per message.
# The three fixed points are chosen from what measure_scale.py found on the LuST dataset, so they
# are the real operating range and not round numbers.
#
# KEEP THIS LIST IN STEP WITH run_benchmarks_x86.sh.
POINTS="sparse:25:1 operating:61:5 dense:150:11"

one_point() {
    label="$1"; tracks="$2"; vectors="$3"; iter="$4"
    out="$RESULTS_DIR/${LABEL}_${label}_r${iter}.log"

    $PIN "$BIN" --tracks "$tracks" --vectors "$vectors" \
                --messages "$MESSAGES" --budget-us "$BUDGET_US" > "$out" 2>&1

    # The whole-pipeline row is the verdict; the stage rows beneath it are the breakdown.
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
        label=$(echo "$spec" | cut -d: -f1)
        tracks=$(echo "$spec" | cut -d: -f2)
        vectors=$(echo "$spec" | cut -d: -f3)
        i=1
        while [ "$i" -le "$REPEAT" ]; do
            line=$(one_point "$label" "$tracks" "$vectors" "$i")
            echo "    $line"
            echo "$line" >> "$SUMMARY"
            i=$((i + 1))
        done
    done

    # The headroom verdict, taken from the operating point, which is the one that decides.
    OPLOG="$RESULTS_DIR/${LABEL}_operating_r1.log"
    if [ -f "$OPLOG" ]; then
        echo ""
        grep -E "budget|headroom" "$OPLOG" | sed 's/^/    /'
        grep -E "headroom" "$OPLOG" | sed "s/^/VERDICT $LABEL /" >> "$SUMMARY"
    fi
}

# ------------------------------------------------------------------------------
# Stage 3: the scale sweep and the optional real-trace replay
# ------------------------------------------------------------------------------
run_sweep() {
    if [ "$SKIP_SWEEP" = "1" ]; then
        echo ""
        echo "[*] [3/3] Skipping the sweep (--skip-sweep)."
        return 0
    fi

    echo ""
    echo "[*] [3/3] Scale sweep (whole-pipeline p99 against N and m)..."
    SWEEPLOG="$RESULTS_DIR/${LABEL}_sweep.log"
    $PIN "$BIN" --sweep --budget-us "$BUDGET_US" > "$SWEEPLOG" 2>&1
    sed -n '/SWEEP/,/^$/p' "$SWEEPLOG" | sed 's/^/    /'

    # The real-trace replay only runs if a dataset slice was shipped.
    if [ -x "$RUNNER" ] && [ -f "$DATA_DIR/cam.csv" ]; then
        echo ""
        echo "[*] Real-trace replay over the shipped dataset slice..."
        REPLAYLOG="$RESULTS_DIR/${LABEL}_replay.log"
        $PIN "$RUNNER" --dataset "$DATA_DIR" --no-csv --label "$LABEL" > "$REPLAYLOG" 2>&1
        grep -E "^BATCH|motion vectors|accuracy" "$REPLAYLOG" | sed 's/^/    /'
        grep "^BATCH" "$REPLAYLOG" >> "$SUMMARY"
    else
        echo ""
        echo "[*] No dataset slice staged; skipping the real-trace replay."
        echo "    (Rebuild the image with vf2_setup.sh --with-data to include one.)"
    fi
}

run_tests
run_points
run_sweep

echo ""
echo "=============================================================================="
echo "[+] Done. Summary written to $SUMMARY"
echo "    Copy it off the board and compare against the x86 run with:"
echo "    scripts/compare_benchmarks.sh summary_x86.txt summary_${LABEL}.txt"
echo "=============================================================================="
