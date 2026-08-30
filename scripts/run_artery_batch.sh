#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# Run the batch sweep specifically for cp-trustfulness using its dedicated python orchestrator
cd "$SCRIPT_DIR/../cp-trustfulness/artery/" || exit 1

if [ ! -d ".venv" ]; then
    echo "Creating virtual environment in $(pwd)..."
    python3 -m venv .venv
    .venv/bin/pip install -r requirements.txt
fi

.venv/bin/python3 scripts/run_batch_sweep.py "$@"
