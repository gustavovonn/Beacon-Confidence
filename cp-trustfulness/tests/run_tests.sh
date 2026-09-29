#!/usr/bin/env bash
# Builds and runs the OMNeT-free unit tests of the confidence core.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build
${CXX:-g++} -std=c++17 -O1 -Wall -Wextra -Wno-unused-parameter \
    test_trust_core.cc ../src/TrustEngine.cc ../src/AssociationEngines.cc ../src/DeadReckoningModels.cc ../src/TrustEvaluators.cc \
    -o build/test_trust_core
./build/test_trust_core
