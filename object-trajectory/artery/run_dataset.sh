#!/bin/bash
# Run the honest dataset generator.
#
# Must run from cp-trustfulness/artery: the BASE omnetpp.ini's paths (config/lust/...,
# config/sensors.xml) are relative to it, and this experiment's ini only extends that one.
# object-trajectory now lives at the repo root, so it is reached as ../../object-trajectory.
set -e
NED=/workspace/artery/src/artery:/workspace/artery/src/traci:/workspace/artery/extern/veins/examples/veins:/workspace/artery/extern/veins/src/veins:/workspace/artery/extern/inet/src:/workspace/artery/extern/inet/examples:/workspace/artery/extern/inet/tutorials:/workspace/artery/extern/inet/showcases:/workspace/artery/src/artery/envmod:/workspace/cp-trustfulness/artery:/workspace/object-trajectory/artery
B=/workspace/build-opt
exec /omnetpp/bin/opp_run_release -n "$NED" \
  -l $B/artery/src/artery/envmod/libartery_envmod.so \
  -l $B/cp-trustfulness/artery/libartery_cp_trustfulness.so \
  -l $B/object-trajectory/artery/libartery_object_trajectory.so \
  -l $B/artery/extern/libINET.so \
  -l $B/artery/extern/libveins.so \
  -l $B/artery/src/traci/libtraci.so \
  -l $B/artery/libartery_core.so \
  -f ../../object-trajectory/artery/omnetpp.ini -c dataset-lust -u Cmdenv "$@"
