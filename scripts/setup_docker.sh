#!/usr/bin/env bash
# =============================================================================
# setup_docker.sh
# Builds the Artery Docker image and starts the persistent container.
# Run this ONCE (or whenever you need to rebuild the image).
# The first run will take a long time.
# Subsequent runs skip the build if image exists.
# =============================================================================

set -e  # exit on any error

CONTAINER_NAME="artery"
IMAGE_NAME="artery-dev"
WORKSPACE="$(cd "$(dirname "$0")/.." && pwd)"   # absolute path to this repo
DOCKERFILE="artery/.devcontainer/debian/Dockerfile"

echo "========================================"
echo " Artery Docker Setup"
echo " Workspace: $WORKSPACE"
echo "========================================"

# --- Step 1: Check Docker is running ---
if ! docker info > /dev/null 2>&1; then
    echo "[ERROR] Docker is not running. Start Docker and try again."
    exit 1
fi
echo "[OK] Docker is running."

# --- Step 2: Build image (skip if already built) ---
if docker image inspect "$IMAGE_NAME" > /dev/null 2>&1; then
    echo "[SKIP] Image '$IMAGE_NAME' already exists. To force rebuild, run:"
    echo "       docker rmi $IMAGE_NAME && ./setup_docker.sh"
else
    echo "[BUILD] Building Docker image '$IMAGE_NAME'..."
    echo "        This will take a long time on first run."
    docker build \
        -t "$IMAGE_NAME" \
        -f "$WORKSPACE/$DOCKERFILE" \
        "$WORKSPACE"
    echo "[OK] Image built successfully."
fi

# --- Step 3: Allow container to access the display ---
echo "[DISPLAY] Granting display access to Docker containers..."
xhost +local:docker || echo "[WARN] xhost failed — GUI may not work. Is XWayland running?"

# --- Step 4: Stop and remove any existing container with the same name ---
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "[CLEAN] Removing existing container '$CONTAINER_NAME'..."
    docker rm -f "$CONTAINER_NAME"
fi

# --- Step 5: Start the persistent container (running as host UID:GID to prevent permission conflicts) ---
echo "[START] Starting container '$CONTAINER_NAME'..."
docker run -d \
    --name "$CONTAINER_NAME" \
    --shm-size=4g \
    --user $(id -u):$(id -g) \
    --ipc=host \
    -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device /dev/dri:/dev/dri \
    -v "$WORKSPACE":/workspace \
    -w /workspace \
    "$IMAGE_NAME" \
    sleep infinity

echo ""
echo "========================================"
echo " Container '$CONTAINER_NAME' is running."
echo ""
echo " To open a shell inside it:"
echo "   docker exec -it $CONTAINER_NAME bash"
echo ""
echo " Next step: run ./scripts/build_simulation.sh"
echo "========================================"
