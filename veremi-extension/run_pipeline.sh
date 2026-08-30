#!/usr/bin/env bash

# Wrapper script for the VeReMi extension data pipeline
# It downloads the dataset, extracts the nested zips, and converts JSON to Parquet chunks.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
cd "$SCRIPT_DIR" || exit 1

VENV_PYTHON=".venv/bin/python3"

# Ensure virtual environment exists
if [ ! -d ".venv" ]; then
    echo "Creating virtual environment in $(pwd)..."
    python3 -m venv .venv
    .venv/bin/pip install -r requirements.txt
fi

DATA_DIR="data"
PARQUET_DIR="parquet"

echo "========================================="
echo " Starting VeReMi Extension Pipeline"
echo "========================================="

echo "[1/3] Downloading dataset from Zenodo..."
"$VENV_PYTHON" scripts/download.py --dest "$DATA_DIR"

echo ""
echo "[2/3] Extracting nested zip files..."
"$VENV_PYTHON" scripts/extract_zips.py --dir "$DATA_DIR"

echo ""
echo "[3/3] Converting JSON records to Parquet datalake..."
"$VENV_PYTHON" scripts/parquet_chunking.py --input "$DATA_DIR" --output "$PARQUET_DIR"

echo ""
echo "========================================="
echo " Pipeline Complete!"
echo " Parquet datalake is available at: veremi-extension/$PARQUET_DIR"
echo "========================================="
