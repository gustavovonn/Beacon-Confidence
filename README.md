# A Lightweight Mechanism to Attribute Confidence to V2X Messages based on Physical Consistence

This repository hosts two offline evaluation experiments that share a common C++ core for
motion prediction and confidence scoring of V2X kinematic beacons. Both are driven by the
[VeReMi Extension](https://zenodo.org/records/20090854) dataset and run outside of OMNeT++,
so they can be profiled on desktop and embedded hardware.

## Repository Layout

- `include/`, `src/` — The confidence mechanism evaluated in this work, shared by both
  experiments: `BeaconConfidenceEngine` (Predictor + Scorer), the consistency and predictive
  checks, the confidence evaluators (Beta reputation, EWMA, majority vote), `EngineConfig`
  and the paper's calibrated constants (`MechanismConstants.h`).
- `libs/v2x-misc/` — Reusable V2X motion primitives, header-only and free of any detection
  policy: WGS-84 geodesy, coordinate policies, the CTRA/CA integrator, CTRA motion models and
  OLS kinematic regression.
- `veremi-extension/` — Dataset pipeline: downloads the VeReMi Extension traces from Zenodo,
  extracts the nested archives, and converts the JSON records into Parquet format.
- `detection-eval/` — Detection-quality experiment: the `veremi_runner` binary (reads Parquet)
  plus the Python analysis suite (window sweeps, scorer comparison, checker ablation, ROC curves).
- `computational-eval/` — Computational-cost experiment: `offline_evaluator`, which runs the
  mechanism and a decoupled F2MD reference implementation side by side on zero-dependency `.bin`
  traces.
- `vf2-setup/` — Cross-compilation and netboot packaging of `offline_evaluator` for the StarFive
  VisionFive 2 (RISC-V), including the kernel image and device tree served over TFTP.
- `run_veremi_batch.sh` — Entry point for the detection experiment (builds the runner, sets up its
  Python environment and launches the batch sweep).

## Prerequisites

- Git
- CMake (3.16 or newer) and a C++20 compiler
- Python 3
- Apache Arrow and Parquet for C++ (`libarrow-dev`, `libparquet-dev`) — only for
  `detection-eval`; `computational-eval` has no C++ dependencies beyond the standard library
- For `vf2-setup` only: a RISC-V cross toolchain (`riscv64-linux-gnu-gcc`/`g++`), `cpio`, `gzip`,
  and a TFTP server

*Note: The primary tested setup for this framework was on Ubuntu 24.04.4 LTS.*

## Setup

1. Clone the repository:
```bash
git clone https://github.com/gustavovonn/CP-Artery.git
cd CP-Artery
```

2. Install Python dependencies:

The project uses self-contained experiments and modules. Create a virtual environment inside the
module you are working with (`veremi-extension`, `detection-eval`, or
`computational-eval`). For example, for the dataset pipeline:

```bash
cd veremi-extension
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
cd ..
```

The `run_pipeline.sh` and `run_veremi_batch.sh` wrappers create their module's virtual
environment automatically on first run.

## Build and Run

### VeReMi Extension Simulation

1. **VeReMi Extension Data Pipeline:** Downloads the VeReMi dataset, extracts nested zips, and
converts the JSON records into a Parquet datalake for faster processing.
```bash
./veremi-extension/run_pipeline.sh
```

2. **Build Simulation:** Compiles the runner needed for the simulation, in
`detection-eval/`.

```bash
cd detection-eval/
mkdir -p build
cd build
cmake .. && make
```

3. **Run VeReMi Batch Sweep:** Executes the detection algorithm across the VeReMi Extension
dataset scenarios (e.g., ConstPos, RandomSpeed), generating F1-score evaluation metrics.
```bash
./run_veremi_batch.sh --sweep
```

*VeReMi results and reports are exported to `detection-eval/results/`.*

4. **Standalone C++ Engine Profiling:** To evaluate computational cost and latency outside of the
OMNeT++ environment (e.g., for embedded hardware benchmarking), you can run the core
`veremi_runner` directly with the `--profile` flag on any parquet data folder.
```bash
./detection-eval/build/veremi_runner --dir <path-to-scenario> --profile
```

See [detection-eval/README.md](detection-eval/README.md) for the full
analysis workflow (window sweeps, decision matrices, scorer comparison, and checker ablation).

### F2MD Baseline Comparisons

1. **Raw Trace Ingestion:** The F2MD evaluator reads the *raw* VeReMi JSON traces rather than the
shared datalake, so it only needs the download and extraction stages of the pipeline. Run the
pipeline once, then convert the traces with F2MD's own ETL scripts.
```bash
./veremi-extension/run_pipeline.sh

cd computational-eval/
python3 -m venv .venv && source .venv/bin/activate
python3 -m pip install -r requirements.txt
python3 scripts/interpolate_veremi.py      # raw JSON -> synchronized Parquet (veremi/parquet/)
python3 scripts/export_parquet_to_bin.py   # Parquet -> zero-dependency .bin (veremi/bin/)
```

2. **Build Simulation:** Compiles the runner needed for the simulation, in `computational-eval/`.

```bash
cd computational-eval/
cmake -B build -S .
cmake --build build -j$(nproc)
```

3. **Standalone C++ Engine Profiling:** To evaluate computational cost and latency outside of the
OMNeT++ environment, run `offline_evaluator` directly with the `--profile` flag on a `.bin` trace
produced by the step above.
```bash
./computational-eval/build/offline_evaluator --tier 3 --stage1 ols-ctra --stage2 beta-reputation --dir computational-eval/veremi/bin/ConstPos_0709.bin --profile
```

See [computational-eval/README.md](computational-eval/README.md) for the tiers, stages and data
preparation details.

### RISC-V Benchmark on the VisionFive 2

`vf2-setup/vf2_setup.sh` cross-compiles a static `offline_evaluator`, builds a BusyBox userland,
prepares the `.bin` traces, packs everything into an initramfs and deploys it, together with
`vf2-setup/config/Image` and the JH7110 device tree, to `/srv/tftp` for netboot:

```bash
./vf2-setup/vf2_setup.sh               # full pipeline, prompts before touching the network/TFTP
./vf2-setup/vf2_setup.sh --skip-tftp   # build and package only
```

On the board, run `cd /bench && ./run_benchmarks_riscv.sh`.

## References

- [OMNeT++](https://omnetpp.org/)
- [F2MD (Framework For Misbehavior Detection)](https://github.com/josephkamel/F2MD)
- [VeReMi Extension Dataset](https://zenodo.org/records/20090854)
