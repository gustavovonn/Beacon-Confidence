#!/usr/bin/env bash
# =============================================================================
# setup_lust.sh
# Preparatory script to inject the CP-Artery fast-start configuration into a
# fresh LuSTScenario submodule.
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
LUST_SCENARIO_DIR="$SCRIPT_DIR/../../artery/scenarios/LuSTScenario"
TARGET_FILE="$LUST_SCENARIO_DIR/scenario/dua.rush.sumocfg"

echo "=================================================="
echo " Preparing LuST Scenario for CP-Artery"
echo "=================================================="

if [ ! -d "$LUST_SCENARIO_DIR/scenario" ]; then
    echo "[ERROR] LuSTScenario not found at $LUST_SCENARIO_DIR"
    echo "        Did you clone the submodule? (git submodule update --init)"
    exit 1
fi

echo "  [1/1] Injecting Rush Hour Fast-Start Configuration..."

cat << 'EOF' > "$TARGET_FILE"
<?xml version="1.0" encoding="UTF-8"?>

<!--
Luxembourg SUMO Traffic (LuST) Scenario - Rush Hour Fast-Start Config (Starts directly at 07:00 AM / 25200s)
-->

<configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo-sim.org/xsd/sumoConfiguration.xsd">

    <input>
        <net-file value="lust.net.xml"/>
        <route-files value="buslines.rou.xml,DUARoutes/local.0.rou.xml,DUARoutes/local.1.rou.xml,DUARoutes/local.2.rou.xml,transit.rou.xml"/>
        <additional-files value="vtypes.add.xml,busstops.add.xml,e1detectors.add.xml,lust.poly.xml"/>
    </input>

    <output>
        <summary-output value="dua.actuated.summary.xml"/>
        <tripinfo-output value="dua.actuated.tripinfo.xml"/>
    </output>

    <time>
        <begin value="25200"/>
        <step-length value="1"/>
    </time>

    <processing>
        <ignore-junction-blocker value="20"/>
        <time-to-teleport value="600"/>
        <max-depart-delay value="600"/>
        <routing-algorithm value="dijkstra"/>
    </processing>

    <routing>
        <device.rerouting.probability value="0.70"/>
        <device.rerouting.period value="300"/>
        <device.rerouting.pre-period value="300"/>
    </routing>

    <report>
        <verbose value="true"/>
        <log value="dua.actuated.log"/>
    </report>

</configuration>
EOF

echo "  [OK] Generated: $TARGET_FILE"
echo "  LuST is ready for execution!"
echo "=================================================="
