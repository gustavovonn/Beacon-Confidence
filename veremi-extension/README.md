# VeReMi Extension

This directory contains the scripts necessary to download, extract, and format the VeReMi dataset into Parquet for efficient querying and analysis.

## Data Workflow

Follow these steps sequentially from the `veremi-extension` directory to prepare your dataset.

### 1. Download the Dataset
The `download.py` script concurrently fetches the dataset zip files directly from Zenodo.

```bash
python3 scripts/download.py 20090854
```

- Files are downloaded to `veremi-extension/data/`.
- By default, the script filters for specific attack types (e.g., ConstPos, RandomPos, etc.). You can override this using the `--filter` argument.
- You can control the number of concurrent download threads using the `--workers` argument.

### 2. Extract the Archives
The VeReMi dataset is heavily nested within multiple layers of zip archives. The `extract_zips.py` script handles this recursive extraction.

```bash
python3 scripts/extract_zips.py
```

- Extracted contents are placed inside `veremi-extension/data/`.
- The original zip files are preserved by default.
- If you wish to save disk space, you can append the `--delete-zips` flag to permanently remove the zip archives once their contents are successfully extracted.

### 3. Chunk into Parquet Format
Once the raw JSON files are extracted, the `parquet_chunking.py` script converts them into a structured Parquet datalake.

```bash
python3 scripts/parquet_chunking.py
```

- This script searches the `veremi-extension/data/` directory for `traceJSON` and `traceGroundTruthJSON` files.
- It parses the relevant fields (filtering out noise) and writes the results to `veremi-extension/parquet/`.
- The exact semantic folder structure (e.g., attack subtypes like `ConstPosOffset/ConstPosOffset_1416/...`) is mirrored inside the `parquet/` directory so no metadata is lost.
