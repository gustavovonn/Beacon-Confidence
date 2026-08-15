#!/usr/bin/env bash
# =============================================================================
# lust_rush_config.sh
# Prepares the LuSTScenario submodule for use with the cp-trustfulness
# simulation. This script derives the fast start rush hour configuration
# from the original LuST files.
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
LUST_DIR="$SCRIPT_DIR/LuSTScenario/scenario"
SOURCE_CFG="$LUST_DIR/dua.actuated.sumocfg"
TARGET_CFG="$LUST_DIR/dua.rush.sumocfg"

echo "=================================================="
echo " Preparing LuST Scenario for cp-trustfulness"
echo "=================================================="

# --- Step 1: Validate the submodule is initialized ---
if [ ! -f "$SOURCE_CFG" ]; then
    echo "[ERROR] LuSTScenario submodule not found or empty."
    echo "        Run: git submodule update --init --recursive"
    exit 1
fi

# --- Step 2: Derive dua.rush.sumocfg from the original dua.actuated.sumocfg ---
# The only modification is setting <begin value="25200"/> (07:00 AM)
# to skip the simulation warm-up period and start directly at peak traffic.
echo "  [1/2] Deriving rush-hour config from original LuST files..."

sed 's|<begin value="0"/>|<begin value="25200"/>|g' "$SOURCE_CFG" > "$TARGET_CFG"

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
echo "  LuST is ready. Run ./scripts/run_simulation.sh --map lust"
echo "=================================================="
