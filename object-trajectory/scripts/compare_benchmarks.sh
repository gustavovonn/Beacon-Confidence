#!/usr/bin/env bash
# ==============================================================================
# Object-trajectory confidence - compare two benchmark summaries
# ==============================================================================
#
# Takes the summary a host run wrote and the summary the board wrote, and puts them side by side.
#
#   scripts/compare_benchmarks.sh results/vf2_compare/summary_x86.txt /path/summary_riscv.txt
#
# The number this exists to produce is the slowdown factor.
# Everything said about running on the VisionFive 2 before the board had actually run was a
# projection; this replaces it with a measurement.
# ==============================================================================

set -uo pipefail

if [ $# -lt 2 ]; then
    echo "Usage: $0 <summary_reference.txt> <summary_target.txt>"
    echo ""
    echo "Both files are produced by run_benchmarks_x86.sh / run_benchmarks_riscv.sh."
    exit 1
fi

REF="$1"
TGT="$2"

for f in "$REF" "$TGT"; do
    if [ ! -f "$f" ]; then
        echo "[!] ERROR: no such summary: $f"
        exit 1
    fi
done

REF_LABEL=$(basename "$REF" .txt | sed 's/^summary_//')
TGT_LABEL=$(basename "$TGT" .txt | sed 's/^summary_//')

echo "=============================================================================="
echo "   COST COMPARISON:  $TGT_LABEL  against  $REF_LABEL"
echo "=============================================================================="

# Tests first. A slowdown factor computed from a binary that failed its own suites is noise.
echo ""
echo "test suites"
grep -h "^TESTS" "$REF" "$TGT" | sed 's/^/  /' || echo "  (no test lines in either summary)"

echo ""
echo "whole-pipeline p99, microseconds per message"
printf "  %-10s %-6s %-5s %12s %12s %10s\n" "point" "N" "m" "$REF_LABEL" "$TGT_LABEL" "slowdown"
printf "  %s\n" "------------------------------------------------------------------------"

# Join on the point label. Only run 1 is compared; repeats are in the summaries for spread.
awk '
    FILENAME == ARGV[1] && $1 == "POINT" {
        split($2, a, ""); label = $2
        n = $3; m = $4; run = $5
        if (run != "run=1") next
        split($8, p, "=")                      # p99=<value>
        ref[label] = p[2]; refn[label] = n; refm[label] = m
        order[++k] = label
        next
    }
    FILENAME == ARGV[2] && $1 == "POINT" {
        label = $2; run = $5
        if (run != "run=1") next
        split($8, p, "=")
        tgt[label] = p[2]
    }
    END {
        for (i = 1; i <= k; i++) {
            l = order[i]
            if (!(l in tgt)) {
                printf "  %-10s %-6s %-5s %12.2f %12s %10s\n",
                       l, substr(refn[l],3), substr(refm[l],3), ref[l], "absent", "-"
                continue
            }
            factor = (ref[l] > 0) ? tgt[l] / ref[l] : 0
            printf "  %-10s %-6s %-5s %12.2f %12.2f %9.1fx\n",
                   l, substr(refn[l],3), substr(refm[l],3), ref[l], tgt[l], factor
        }
    }
' "$REF" "$TGT"

echo ""
echo "verdict against the per-message budget"
grep -h "^VERDICT" "$REF" "$TGT" | sed 's/^VERDICT /  /' || echo "  (no verdict lines)"

# The real-trace replay, when both ran one.
if grep -q "^BATCH" "$REF" 2>/dev/null && grep -q "^BATCH" "$TGT" 2>/dev/null; then
    echo ""
    echo "real-trace replay: the outputs must be IDENTICAL across architectures"
    echo "  a difference here is a portability bug, not a performance result"
    REF_BATCH=$(grep -m1 "^BATCH" "$REF" | sed 's/^BATCH [^ ]* //')
    TGT_BATCH=$(grep -m1 "^BATCH" "$TGT" | sed 's/^BATCH [^ ]* //')
    if [ "$REF_BATCH" = "$TGT_BATCH" ]; then
        echo "  IDENTICAL"
        echo "    $REF_BATCH"
    else
        echo "  THEY DIFFER:"
        echo "    $REF_LABEL: $REF_BATCH"
        echo "    $TGT_LABEL: $TGT_BATCH"
    fi
fi

echo ""
echo "=============================================================================="
