# VeReMi Extension: High-Performance Trust & Dead-Reckoning Benchmark Suite

High-throughput native C++ Parquet evaluation runner and Python analysis suite for vehicular misbehavior detection. Evaluates history-aware Dead Reckoning (`HistoryAwareCTRA`) and trust mechanisms (`BetaReputation`) across the extended VeReMi dataset under varied traffic densities (rush hour `0709` vs. afternoon `1416`) and attack models.

---

## 1. Directory Structure

```
cp-trustfulness/veremi-extension/
├── CMakeLists.txt                       # CMake build definition (links Arrow & Parquet)
├── veremi_runner.cc                     # Native C++ streaming Parquet processor
├── requirements.txt                     # Python evaluation dependencies
├── README.md                            # Documentation
├── build/                               # Compiled C++ binaries (veremi_runner)
├── scripts/
│   ├── run_veremi_batch.py              # Batch scenario runner and multi-scenario sweep
│   ├── plot_window_sweep.py             # Single-scenario history window sweep & 4-panel plot
│   ├── plot_multi_scenario_comparison.py# Multi-scenario sensitivity heatmap, Pareto dashboard, & decision matrix
│   └── optimize_weights.py              # OLS regression to calibrate kinematic thresholds
└── results/                             # Evaluation outputs
    ├── plots/                           # Publication figures (.png and .pdf)
    ├── run_<timestamp>/                 # Raw metrics (.json) per scenario run
    └── multi_scenario_*                 # Compressed decision summaries (.csv, .md)
```

---

## 2. Architecture & Key Features

### High-Speed Native C++ Engine (`veremi_runner`)
- **Direct Parquet Ingestion**: Leverages Apache Arrow C++ and Parquet APIs to read columnar BSM traces with zero intermediate conversion overhead.
- **Dead Reckoning Models**:
  - `CartesianHistoryAwareCTRA`: Fits historical position, speed, acceleration, and yaw rate across a parametric window size ($W$) via linear regression, integrating Cartesian CTRA equations to predict next-state kinematics.
  - `CartesianCTRA`: Instantaneous Constant Turn Rate and Acceleration dead-reckoning fallback.
- **Trust Evaluation**:
  - `BetaReputation`: Continuously updates $\alpha$ and $\beta$ reputation parameters based on dead-reckoning deviation error and dynamic context bounds (speed and yaw weights).
- **Extreme Throughput**: Sustains over **200,000–350,000 messages/second** (average message latency between **3 µs and 6 µs**), making it well-suited for processing millions of vehicular records in seconds.

### Automated Evaluation & Decision Pipeline
- **Automatic Attacker Identification**: Direct extraction of malicious node IDs from VeReMi trace nomenclature (`traceJSON-[nodeID]-[pseudoID]-A[type]-*`).
- **Comprehensive Detection Metrics**: Evaluates Recall (Detection Rate), Precision, F1-Score, False Positive Rate (FPR), Accuracy, Latency (µs/msg), and Throughput (msg/s).
- **Multi-Scenario Decision Matrix**: Identifies the globally optimal history window size ($W^*$) by synthesizing macro-average F1, worst-case attack robustness, and computational latency.

---

## 3. Setup & Build

### Python Environment Setup
Activate the workspace virtual environment and ensure all dependencies are installed:
```bash
cd cp-trustfulness/veremi-extension
source ../../.venv/bin/activate
pip install -r requirements.txt
```

*Required packages: `numpy`, `pandas`, `pyarrow`, `matplotlib`, `seaborn`, `scipy`.*

### C++ Compilation (`veremi_runner`)
Ensure Apache Arrow and Parquet libraries are available (`libarrow-dev`, `libparquet-dev`):
```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
cd ..
```
This produces the high-speed executable: `build/veremi_runner`.

---

## 4. Dataset Integration

The runner automatically resolves Parquet datalakes converted by `veremi-extension/scripts/parquet_chunking.py`.

Default search locations:
1. `<repo_root>/veremi-extension/parquet/<ScenarioName>/VeReMi_*`
2. `<repo_root>/cp-trustfulness/veremi-extension/parquet/<ScenarioName>/VeReMi_*`
3. `<repo_root>/parquet/<ScenarioName>/VeReMi_*`

Supported scenarios include both rush hour (`0709`) and non-rush hour (`1416`) traces:
- `ConstPosOffset_0709`, `ConstPosOffset_1416`
- `ConstSpeedOffset_0709`, `ConstSpeed_1416`
- `ConstPos_1416`
- `RandomSpeed_1416`, `RandomSpeedOffset_1416`

---

## 5. Usage & Workflows

### A. Window Sweep Runner (`plot_window_sweep.py`)
Sweeps tracking history window sizes ($W = 4, 6, 8, \dots, 30$), saves JSON raw metrics, and outputs a 4-panel publication figure per scenario. Supports single-scenario evaluation, automated multi-scenario execution (`--all`), variant filtering (`--variant 0709|1416`), and parallel processing (`-j`):

```bash
# 1. Evaluate a single scenario (default: W = 4 to 30, step = 2):
python scripts/plot_window_sweep.py --scenario RandomSpeed_1416

# 2. Evaluate ALL scenarios across both variants (0709 & 1416):
python scripts/plot_window_sweep.py --all

# 3. Evaluate ONLY rush-hour scenarios (0709):
python scripts/plot_window_sweep.py --all --variant 0709

# 4. Evaluate ONLY afternoon scenarios (1416):
python scripts/plot_window_sweep.py --all --variant 1416

# 5. Fast parallel sweep across scenarios using 4 CPU workers:
python scripts/plot_window_sweep.py --all --variant 0709 -j 4

# 6. Custom window sizes & limits:
python scripts/plot_window_sweep.py --scenario ConstPosOffset_1416 --window-sizes 4 6 8 10 12 15 20 25 30
python scripts/plot_window_sweep.py --all --variant 0709 --limit 1

# 7. Re-generate plot from existing JSON run without re-executing C++ binary:
python scripts/plot_window_sweep.py --from-json results/run_20260908_152326/window_sweep_RandomSpeed_1416_20260908_152326.json
```

**Output Figure Panels**:
1. Detection Performance (F1 Score, Recall, Precision) vs. Window Size
2. False Positive Rate (FPR %) vs. Window Size
3. Computational Cost (Latency in µs & Throughput in msg/s) vs. Window Size
4. Pareto Frontier (Latency vs. F1 Score with sweet spot highlighted)

*Note: When running with `--all`, the script automatically triggers `plot_multi_scenario_comparison.py` upon completion to generate the aggregated Sensitivity Heatmap, Decision Dashboard, and Decision Matrix report.*

---

### B. Multi-Scenario Decision & Sensitivity Matrix (`plot_multi_scenario_comparison.py`)
Aggregates the latest sweep runs across all scenarios in `results/`, calculates Borda ranks and composite utility scores, and determines the optimal general history window size ($W^*$):

```bash
# Automatically discover and aggregate all latest scenario runs
python scripts/plot_multi_scenario_comparison.py

# Compare specific scenarios only
python scripts/plot_multi_scenario_comparison.py --scenarios ConstPosOffset_0709 ConstSpeedOffset_0709 RandomSpeed_1416

# Specify explicit JSON files
python scripts/plot_multi_scenario_comparison.py --files results/run_*/window_sweep_*.json
```

**Generated Visualizations & Reports**:
- **Sensitivity Heatmap (`results/plots/*_heatmap.png`, `.pdf`)**: Scenarios $\times$ Window Sizes matrix with aggregate summary rows (Macro Mean F1, Worst-case F1, and Latency).
- **Optimization Dashboard (`results/plots/*_dashboard.png`, `.pdf`)**: 4-panel dashboard with macro F1 variance bands, worst-case curves, latency scaling, and composite utility score.
- **Decision Matrix Report (`results/*_decision_summary.md`, `.csv`)**: Full statistical table with average ranks and recommended $W^*$.

---

### C. Full Batch Multi-Scenario Sweep (`run_veremi_batch.py`)
Automates sweeps across all discovered parquet scenarios or evaluates a single scenario:

```bash
# Run across ALL discovered scenarios in parquet/ folder:
python scripts/run_veremi_batch.py --sweep --history-sweep 4 8 12 16 20

# Single scenario run with specific dead-reckoning and trust threshold:
python scripts/run_veremi_batch.py --scenario ConstPosOffset_1416 --history-size 18 --threshold 0.5
```

---

### D. Kinematic Parameter Optimization (`optimize_weights.py`)
Fits Ordinary Least Squares (OLS) regression over honest baseline traces (`A0`) to calibrate context-aware error bounds:
```bash
python scripts/optimize_weights.py
```
Outputs calibrated `maxPosDev` ($\beta_0$), `speedWeight` ($w_v$), and `yawWeight` ($w_{\text{yaw}}$).

---

### E. Direct C++ Execution (`veremi_runner`)
Run the native binary directly on a simulation directory:
```bash
./build/veremi_runner \
    --dir /path/to/parquet/scenario/simulation_dir \
    --dr HistoryAwareCTRA \
    --trust BetaReputation \
    --historySize 18 \
    --maxPosDev 4.3506 \
    --speedWeight 5.2264 \
    --yawWeight 0.6536

# Run with high-resolution latency profiling:
./build/veremi_runner --dir /path/to/parquet/dir --profile --historySize 18
```

---

## 6. Output Artifacts

| Directory / File | Description |
|:---|:---|
| `results/plots/window_sweep_*.png` (.pdf) | Single-scenario 4-panel analysis figures |
| `results/plots/multi_scenario_*_heatmap.png` (.pdf) | Multi-scenario $W \times \text{Scenario}$ sensitivity heatmap |
| `results/plots/multi_scenario_*_dashboard.png` (.pdf) | Multi-objective trade-off dashboard and decision curve |
| `results/run_<timestamp>/window_sweep_*.json` | Per-window raw metrics, confusion matrices, and execution timings |
| `results/multi_scenario_*_decision_summary.md` | Formatted Markdown decision matrix with recommended $W^*$ |
| `results/multi_scenario_*_decision_summary.csv` | Machine-readable tabular summary for downstream analysis |
