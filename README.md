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

3. Prepare the Mobility Scenarios (LuST and MoST):
```bash
./sumo-scenarios/lust_rush_config.sh
./sumo-scenarios/most_rush_config.sh
```

4. Install the Python3 dependencies. We recommend the creation of a virtual environment for this project, as follows:
```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

## Build and Run

1. **Setup Environment:** Builds the `artery-dev` Docker image and starts the persistent container. Only needs to be run once.
```bash
./scripts/setup_docker.sh
```

2. **Build Simulation:** Compiles the Artery framework and the out-of-tree `cp-trustfulness` scenario inside the container. Run this whenever you modify C++ source files.
```bash
./scripts/build_simulation.sh
```

3. **Run a Single Simulation:** Executes a specific scenario with manual parameter overrides. Ideal for testing and debugging.
```bash
python3 cp-trustfulness/scripts/experiment.py --map lust --dr HistoryAwareCTRA --trust BetaReputation
```

4. **Automated Context Tuning:** Uses machine learning to scientifically discover the optimal physical thresholds (`maxPositionDeviation`, `speedWeight`, `yawWeight`) by running a clean baseline. Currently, this script just prints out the recommended weights (*TODO: Add persistent statistics and training parameters to allow reproducibility*).
```bash
python3 cp-trustfulness/scripts/scorer_context_tuning.py --map lust --training-time 100s --validation-time 45s --fault-profile random_noise
```

5. **Run a Batch Sweep:** Executes multiple automated simulations sweeping across various parameters. Ideal for data collection, graphing, and result generation.
```bash
python3 cp-trustfulness/scripts/run_batch_sweep.py
```

*Results, SQLite databases, and efficacy reports are exported to `cp-trustfulness/results/`.*

## References

- [Artery Framework](https://github.com/riebl/artery)
- [SUMO (Simulation of Urban MObility)](https://sumo.dlr.de/docs/)
- [OMNeT++](https://omnetpp.org/)
- [LuST Scenario](https://github.com/lcodeca/LuSTScenario)
- [MoST Scenario](https://github.com/lcodeca/MoSTScenario)