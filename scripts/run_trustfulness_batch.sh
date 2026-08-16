#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# Run the batch sweep specifically for cp-trustfulness using its dedicated python orchestrator
cd "$SCRIPT_DIR/../cp-trustfulness" || exit 1
python3 scripts/run_batch_sweep.py "$@"
