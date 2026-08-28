#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# Run the batch sweep specifically for the VeReMi extension experiment
cd "$SCRIPT_DIR/../cp-trustfulness/veremi-extension/" || exit 1

if [ ! -d ".venv" ]; then
    echo "Creating virtual environment in $(pwd)..."
    python3 -m venv .venv
    .venv/bin/pip install -r requirements.txt
fi

.venv/bin/python3 scripts/run_veremi_batch.py "$@"
