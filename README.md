# V2X Trust Framework

This repository hosts a V2X simulation framework based on an overlay architecture, separating the core network engine from the research scenarios.

## Architecture

- `artery/`: The core Artery framework engine (submodule).
- `cp-trustfulness/`: V2X Trust Models and evaluation logic.
- `sumo-scenarios/`: Reproducible SUMO mobility scenarios (LuST, MoST).
- `scripts/`: Docker orchestration and simulation build/run automation.

## Prerequisites

- Git
- Docker
- Python3 (3.14.x)

### VeReMi Extension Simulations Prerequesites

- Apache Arrow for C++

### F2MD Simulations Prerequesites

- Apache Arrow for C++
- Boost library for C++

*Note: The primary tested setup for this framework was on Ubuntu 24.04.4 LTS.*

## Setup

1. Clone the repository:
```bash
git clone https://github.com/gustavovonn/cp-artery.git
cd cp-artery
```

2. Initialize submodules (Downloads the core engine and its dependencies):
```bash
git submodule update --init --recursive
```

3. Install Python dependencies:

The project uses self-contained experiments and modules. You should create a virtual environment inside the module you are working with (e.g., `cp-trustfulness/artery`, `cp-trustfulness/veremi-extension`, or `veremi-extension`). For example, to set up the Artery simulation scripts:

```bash
cd cp-trustfulness/artery
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
cd ../../
```

## Build and Run

### Artery Simulation

1. **Setup Environment:** Builds the `artery-dev` Docker image and starts the persistent container. Only needs to be run once.
```bash
./scripts/setup_docker.sh
```

2. **Build Simulation:** Compiles the Artery framework and the out-of-tree `cp-trustfulness` scenario inside the container. Run this whenever you modify C++ source files.
```bash
./scripts/build_simulation.sh
```

3. **Run a Single Simulation:** Executes a specific scenario with manual parameter overrides. Ideal for testing and debugging. Ensure you have activated the virtual environment in `cp-trustfulness/artery/`.
```bash
cp-trustfulness/artery/.venv/bin/python3 cp-trustfulness/artery/scripts/experiment.py --map lust --dr HistoryAwareCTRA --trust BetaReputation
```

4. **Automated Context Tuning:** Uses machine learning to scientifically discover the optimal physical thresholds (`maxPositionDeviation`, `speedWeight`, `yawWeight`) by running a clean baseline. Currently, this script just prints out the recommended weights (*TODO: Add persistent statistics and training parameters to allow reproducibility*).
```bash
cp-trustfulness/artery/.venv/bin/python3 cp-trustfulness/artery/scripts/scorer_context_tuning.py --map lust --training-time 100s --validation-time 45s --fault-profile random_noise
```

5. **Run a Batch Sweep:** Executes multiple automated simulations sweeping across various parameters. Ideal for data collection, graphing, and result generation. Alternatively, use `./scripts/run_artery_batch.sh`.
```bash
cp-trustfulness/artery/.venv/bin/python3 cp-trustfulness/artery/scripts/run_batch_sweep.py
```

*Results, SQLite databases, and efficacy reports are exported to `cp-trustfulness/artery/results/`.*

### VeRemi Extension Simulation

1. **VeReMi Extension Data Pipeline:** Downloads the VeReMi dataset, extracts nested zips, and converts the JSON records into a Parquet datalake for faster processing.
```bash
./veremi-extension/run_pipeline.sh
```

2. **Build Simulation:** Compiles the runner needed for the simulation, in `cp-trustfulness/veremi-extension/`.

```bash
cd cp-trustfulness/veremi-extension/
mkdir build
cd build
cmake .. && make
```

3. **Run VeReMi Batch Sweep:** Executes the detection algorithm across the VeReMi Extension dataset scenarios (e.g., ConstPos, RandomSpeed), generating F1-score evaluation metrics.
```bash
./scripts/run_veremi_batch.sh --sweep
```

*VeReMi results and reports are exported to `cp-trustfulness/veremi-extension/results/`.*

4. **Standalone C++ Engine Profiling:** To evaluate computational cost and latency outside of the OMNeT++ environment (e.g., for embedded hardware benchmarking), you can run the core `veremi_runner` directly with the `--profile` flag on any parquet data folder.
```bash
./cp-trustfulness/veremi-extension/build/veremi_runner --dir <path-to-scenario> --profile
```

### F2MD Simulation

1. **VeReMi Extension Data Pipeline:** Downloads the VeReMi dataset, extracts nested zips, and converts the JSON records into a Parquet datalake for faster processing.
```bash
./veremi-extension/run_pipeline.sh
```

2. **Build Simulation:** Compiles the runner needed for the simulation, in `cp-trustfulness/f2md/`.

```bash
cd cp-trustfulness/f2md/
mkdir build
cd build
cmake .. && make
```

3. **Standalone C++ Engine Profiling:** To evaluate computational cost and latency outside of the OMNeT++ environment (e.g., for embedded hardware benchmarking), you can run the core `f2md_runner` directly with the `--profile` flag on any parquet data folder.
```bash
./cp-trustfulness/f2md/build/f2md_runner --dir <path-to-scenario> --profile
```

## References

- [Artery Framework](https://github.com/riebl/artery)
- [SUMO (Simulation of Urban MObility)](https://sumo.dlr.de/docs/)
- [OMNeT++](https://omnetpp.org/)
- [LuST Scenario](https://github.com/lcodeca/LuSTScenario)
- [MoST Scenario](https://github.com/lcodeca/MoSTScenario)
- [VeReMi Extension Dataset](https://zenodo.org/records/20090854)