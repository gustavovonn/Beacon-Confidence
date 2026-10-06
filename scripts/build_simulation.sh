#!/usr/bin/env bash
# =============================================================================
# build_simulation.sh
# Configures and builds a scenario inside the container.
# Requires the container to be running (run setup_docker.sh first).
# =============================================================================

set -e

CONTAINER_NAME="artery"
BUILD_DIR="/workspace/build"

SCENARIO="cp-trustfulness"

if [[ "$1" == "--scenario" ]]; then
    SCENARIO="$2"
    shift 2
fi

SCENARIO_TARGET="${SCENARIO//-/_}"

echo "========================================"
echo " Artery Build — $SCENARIO scenario"
echo "========================================"

# --- Check submodules are checked out (a clone without --recursive leaves artery/ empty) ---
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
if [ ! -f "$REPO_DIR/artery/CMakeLists.txt" ] || [ ! -f "$REPO_DIR/artery/extern/vanetza/CMakeLists.txt" ]; then
    echo "[ERROR] The artery submodule (or its extern/ submodules) is not checked out."
    echo "        Run: git -C $REPO_DIR submodule update --init --recursive"
    echo "        If a configure already failed, also run: docker exec $CONTAINER_NAME rm -rf $BUILD_DIR"
    exit 1
fi
echo "[OK] Submodules are checked out."

# --- Check container is running ---
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "[ERROR] Container '$CONTAINER_NAME' is not running."
    echo "        Run ./setup_docker.sh first."
    exit 1
fi
echo "[OK] Container '$CONTAINER_NAME' is running."

# --- Step 1: CMake configure (only if build dir does not exist) ---
if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "[CMAKE] Configuring build (first time)..."
    docker exec "$CONTAINER_NAME" bash -c "
        cmake -B $BUILD_DIR \
              -DCMAKE_BUILD_TYPE=Debug \
              -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
              -DWITH_STORYBOARD=OFF \
              /workspace
    "
    echo "[OK] CMake configured."
else
    echo "[SKIP] Build already configured. Using existing $BUILD_DIR."
    echo "       To force reconfigure: docker exec -it $CONTAINER_NAME bash"
    echo "       then: rm -rf $BUILD_DIR && exit, then re-run this script."
fi

# --- Step 2: Build the scenario target ---
echo "[BUILD] Building $SCENARIO_TARGET target..."
docker exec "$CONTAINER_NAME" bash -c "
    cmake --build $BUILD_DIR --target $SCENARIO_TARGET -j\$(nproc)
"

echo ""
echo "========================================"
echo " Build complete."
echo " Try to run a batch or single an experiment now!"
echo "========================================"
