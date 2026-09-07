# F2MD Offline Evaluator & Benchmarks

Decoupled, standalone reference implementation of the Framework for Misbehavior Detection (F2MD) and the offline computational cost evaluator for comparing OLS-fitted CTRA + Beta-Reputation against F2MD baseline algorithms.

---

## 1. Directory Structure

```
f2md/
├── CMakeLists.txt                 # CMake build configuration
├── offline_evaluator.cc           # Offline evaluation harness CLI
├── README.md                      # Documentation
│
├── include/                       # Standalone headers
│   ├── BasicSafetyMessage.h       # BSM message struct (with observer kinematics)
│   ├── BinaryMessageFormat.h      # Compact binary trace format (.bin)
│   ├── F2MDEngine.h               # F2MD execution engine (checks & apps)
│   ├── VeremiEngine.h             # Proposed model engine (OLS-CTRA & Beta-Reputation)
│   └── omnetpp.h                  # Minimal OMNeT++ type definitions (Coord, simtime_t)
│
├── scripts/                       # Benchmark automation & ETL scripts
│   ├── run_benchmarks.sh          # Automated 3-tier benchmark execution script
│   ├── prepare_benchmark_dataset.py # Multi-process ETL tool for all rush-hour profiles
│   ├── interpolate_veremi_traces.py # VeReMi JSON trace interpolator -> Parquet
│   ├── export_parquet_to_bin.py   # Parquet -> Compact binary (.bin) converter
│   └── busybox.sh                 # Minimal BusyBox / RISC-V SBC packager
│
├── veremi/                        # Benchmark trace datasets (Parquet & Binary)
│   ├── ConstPos_0709.parquet / .bin
│   ├── ConstPosOffset_0709.parquet / .bin
│   ├── ConstSpeed_0709.parquet / .bin
│   ├── ConstSpeedOffset_0709.parquet / .bin
│   ├── RandomPos_0709.parquet / .bin
│   ├── RandomPosOffset_0709.parquet / .bin
│   ├── RandomSpeed_0709.parquet / .bin
│   └── RandomSpeedOffset_0709.parquet / .bin
│
├── results/                       # Benchmark output CSV and Markdown reports
│
└── veins/                         # Standalone F2MD reference code (decoupled from OMNeT++)
    ├── base/                      # Coordinate utilities (Coord)
    └── modules/application/f2md/  # mdChecks, mdApplications, mdBase, mdSupport
```

---

## 2. Building the Evaluator (CMake)

### Prerequisites
- C++20 compiler (`g++` >= 10 or `clang++` >= 11)
- `cmake` >= 3.16
- *(Optional)* `libarrow-dev`, `libparquet-dev` (only required when reading `.parquet` files)

### Standard Host Build (with Apache Arrow)
```bash
cmake -B build -S .
cmake --build build -j$(nproc)
```

### Standalone Build (Binary Format, Zero External Dependencies)
Disables Apache Arrow and Parquet to compile without external libraries. Evaluates compact `.bin` traces directly:
```bash
cmake -B build-standalone -S . -DUSE_ARROW=OFF
cmake --build build-standalone -j$(nproc)
```

### RISC-V SBC / Bare-Metal Static Cross-Compilation
```bash
cmake -B build-riscv -S . \
    -DCMAKE_C_COMPILER=riscv64-linux-gnu-gcc \
    -DCMAKE_CXX_COMPILER=riscv64-linux-gnu-g++ \
    -DUSE_ARROW=OFF \
    -DSTATIC_BUILD=ON

cmake --build build-riscv -j$(nproc)
riscv64-linux-gnu-strip build-riscv/offline_evaluator
```

Or generate an automated bootable BusyBox `initramfs.cpio.gz` bundle for SBCs:
```bash
./scripts/busybox.sh
```

---

## 3. Running Benchmarks

### Automated Full Suite (`scripts/run_benchmarks.sh`)

The automated benchmark script profiles all checker and application combinations across Tiers 1, 2, and 3:

```bash
# Run the full benchmark suite on the default trace:
./scripts/run_benchmarks.sh --tier all --max-msgs 50000 --repeat 3

# Run across all 8 rush-hour fault profiles:
./scripts/run_benchmarks.sh --all-profiles --max-msgs 50000 --repeat 1

# Pin execution to a specific CPU core for deterministic timing:
./scripts/run_benchmarks.sh --tier all --core 3
```

Results and formatted Markdown tables are automatically saved to [`results/`](results/).

---

### Direct CLI Usage (`offline_evaluator`)

```bash
./build/offline_evaluator --tier <1|2|3|all> --stage1 <checker> --stage2 <app> --dir <dataset_path> [OPTIONS]
```

#### CLI Options
| Flag | Options / Format | Description |
| :--- | :--- | :--- |
| `--tier` | `1`, `2`, `3`, `all` | Benchmark tier (1: Checks, 2: Scorers, 3: End-to-End). Default: `3`. |
| `--stage1` | `ols-ctra`, `legacy`, `catch`, `experi` | Stage 1 plausibility checker algorithm. |
| `--stage2` | `beta-reputation`, `threshold`, `behavioral`, `aggregation`, `experi` | Stage 2 trust scoring / decision application. |
| `--model` | `ols-ctra`, `f2md-legacy`, `f2md-catch`, `f2md-experi` | Preset pipeline shortcut. |
| `--dir` / `--file` | `<path>` | Input trace (`.parquet` or `.bin` file or directory). |
| `--max-msgs` | `<N>` | Maximum messages to process (e.g. `50000`). |
| `--repeat` | `<N>` | Number of repetitions to average. |
| `--profile` | *(flag)* | Print human-readable latency and throughput table to stdout. |
| `--csv-out` | `<path>` | Append results directly to a CSV file. |

#### Examples

```bash
DATA="veremi/ConstPos_0709.parquet"
# (Or use DATA="veremi/ConstPos_0709.bin" for zero-dependency binary format)

# 1. Profile Tier 1 (Stage 1) - OLS-CTRA Checker
./build/offline_evaluator --tier 1 --stage1 ols-ctra --dir "$DATA" --max-msgs 50000 --profile

# 2. Profile Tier 1 (Stage 1) - F2MD CaTCh Kalman Filter Checks
./build/offline_evaluator --tier 1 --stage1 catch --dir "$DATA" --max-msgs 50000 --profile

# 3. Profile Tier 2 (Stage 2) - Beta-Reputation Scorer
./build/offline_evaluator --tier 2 --stage2 beta-reputation --dir "$DATA" --max-msgs 50000 --profile

# 4. Profile Tier 2 (Stage 2) - F2MD ThresholdApp Scorer
./build/offline_evaluator --tier 2 --stage2 threshold --dir "$DATA" --max-msgs 50000 --profile

# 5. Profile Tier 3 (End-to-End) - Full Proposed Pipeline
./build/offline_evaluator --tier 3 --stage1 ols-ctra --stage2 beta-reputation --dir "$DATA" --max-msgs 50000 --profile
```

---

### Benchmark Output Schema

Results written to [`results/`](results/) adhere to the following schema:

| Column | Description |
| :--- | :--- |
| `Dataset` | Trace name evaluated (e.g., `ConstPos_0709`). |
| `Tier` | Evaluation tier (`1`, `2`, or `3`). |
| `Stage1` | Plausibility checker algorithm evaluated. |
| `Stage2` | Trust scorer / decision application evaluated. |
| `Messages` | Number of BSM messages processed. |
| `Stage1_Latency_us` | Average latency of Stage 1 per message (in microseconds $\mu s$). |
| `Stage2_Latency_us` | Average latency of Stage 2 per message (in microseconds $\mu s$). |
| `Total_Latency_us` | Total processing latency per message including pipeline overhead ($\mu s$). |
| `Throughput_msgs_sec` | Overall processing throughput in messages per second. |
| `Total_Time_ms` | Total execution duration (in milliseconds). |
