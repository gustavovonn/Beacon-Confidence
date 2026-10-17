# F2MD Offline Evaluator & Benchmarks

Decoupled reference implementation of F2MD and offline evaluator comparing OLS-CTRA + Beta-Reputation against baseline algorithms.

---

## 1. Directory Structure

```
f2md/
├── CMakeLists.txt                 # Build configuration
├── offline_evaluator.cc           # Benchmark app
├── requirements.txt               # Python ETL dependencies (pandas, pyarrow, scipy)
├── include/                       # Engine, BSM, and binary trace headers
├── scripts/
│   ├── interpolate_veremi.py      # Raw JSON -> synchronized Parquet
│   ├── export_parquet_to_bin.py   # Parquet -> zero-dependency .bin
│   ├── run_benchmarks_riscv.sh    # RISC-V SBC runner
│   ├── busybox_build_riscv.sh     # Isolated static BusyBox builder
│   └── vf2_setup.sh               # StarFive VisionFive 2 modular setup & packager
├── veremi/                        # Benchmark datasets
│   ├── parquet/                   # Columnar traces (.parquet, ETL intermediate)
│   └── bin/                       # Binary traces (.bin)
├── results/                       # CSV and Markdown reports
└── veins/                         # Decoupled F2MD checks & applications
```

---

## 2. Data Preparation Pipeline

Raw simulation traces live in `<repo_root>/veremi-extension/data/`. All scripts use dynamic relative paths.

### Python Virtual Environment Setup
Install the required ETL packages (`pandas`, `pyarrow`, `scipy`):
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Trace Ingestion & Binary Export
```bash
# 1. Parse raw JSON traces into synchronized Parquet (all 8 profiles by default):
python3 scripts/interpolate_veremi.py

# 2. Convert Parquet to zero-dependency binary traces (veremi/bin/):
python3 scripts/export_parquet_to_bin.py
```

*Options for `interpolate_veremi.py`: `--profile <name>` (single profile), `--max-msgs 50000`, `-j <workers>`.*

---

## 3. Build

### Host Build (Zero Dependencies)

The evaluator ingests only the zero-dependency binary traces (`.bin`), so no Apache Arrow
or Parquet C++ libraries are required. Produce the traces with the ETL step above.

```bash
cmake -B build -S .
cmake --build build -j$(nproc)
```

### RISC-V SBC Cross-Compilation & Packaging
```bash
# Option A: Direct CMake build
cmake -B build-riscv -S . \
    -DCMAKE_C_COMPILER=riscv64-linux-gnu-gcc \
    -DCMAKE_CXX_COMPILER=riscv64-linux-gnu-g++ \
    -DSTATIC_BUILD=ON
cmake --build build-riscv -j$(nproc)
riscv64-linux-gnu-strip build-riscv/offline_evaluator -o build-riscv/offline_evaluator_stripped

# Option B: Modular StarFive VisionFive 2 setup & bare-metal initramfs packaging
./scripts/vf2_setup.sh           # Automated setup: builds BusyBox, compiles code, prepares data, packages initramfs
                                 # (Interactively prompts before deploying to /srv/tftp; use --skip-tftp to bypass)
```

---

## 4. Running Benchmarks

### Automated Runners (Evaluates All 8 Profiles by Default)

```bash
# StarFive VisionFive 2 RISC-V SBC:
./scripts/run_benchmarks_riscv.sh --max-msgs 50000

# (Optional) Evaluate a single profile instead:
./scripts/run_benchmarks_riscv.sh --profile ConstPos_0709 --max-msgs 50000
```

### Direct CLI Execution

```bash
./build/offline_evaluator --tier <1|2|3|all> --stage1 <checker> --stage2 <app> --dir <dataset> --profile
```

Results and statistical summaries are saved to `results/`.
