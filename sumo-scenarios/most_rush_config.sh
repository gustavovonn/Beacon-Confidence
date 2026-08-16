#!/usr/bin/env bash
# =============================================================================
# most_rush_config.sh
# Prepares the MoSTScenario submodule for use with the cp-trustfulness
# simulation. This script derives the fast start rush hour configuration
# from the original MoST files.
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
MOST_DIR="$SCRIPT_DIR/MoSTScenario/scenario"
SOURCE_CFG="$MOST_DIR/most.sumocfg"
TARGET_CFG="$MOST_DIR/most.rush.sumocfg"

echo "=================================================="
echo " Preparing MoST Scenario for cp-trustfulness"
echo "=================================================="

# --- Step 1: Validate the submodule is initialized ---
if [ ! -f "$SOURCE_CFG" ]; then
    echo "[ERROR] MoSTScenario submodule not found or empty."
    echo "        Run: git submodule update --init --recursive"
    exit 1
fi

# --- Step 2: Derive most.rush.sumocfg from the original most.sumocfg ---
# The original starts at 14400s (04:00 AM).
# We set <begin value="25200"/> (07:00 AM) to skip the low traffic period.
echo "  [1/2] Deriving rush-hour config from original MoST files..."

sed 's|<begin value="14400"/>|<begin value="25200"/>|g' "$SOURCE_CFG" > "$TARGET_CFG"

# --- Step 3: Verify the result ---
if grep -q 'begin value="25200"' "$TARGET_CFG"; then
    echo "  [2/2] Verified: begin time set to 25200s (07:00 AM rush hour)."
else
    echo "[ERROR] Patch failed. Check $SOURCE_CFG format."
    rm -f "$TARGET_CFG"
    exit 1
fi

echo ""
echo "  Generated: $TARGET_CFG"
echo "  MoST is ready. Run python3 cp-trustfulness/scripts/experiment.py --map most ..."
echo "=================================================="
