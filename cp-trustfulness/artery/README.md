# Running the CAM/CPM confidence experiments

Everything runs inside the Docker container `artery` (`/workspace` = this repository). Run the commands below from
`cp-trustfulness/artery` on the host.

## 1. Build (after any C++ change)

```bash
docker exec artery bash -c 'cd /workspace/build && cmake --build . --target cp_trustfulness -j4'
bash ../tests/run_tests.sh                      # unit tests of the core (no simulator needed)
.venv/bin/python3 scripts/test_cpm_pipeline.py  # unit tests + config checks + a 10 s smoke simulation
```

## 2. Run experiments

```bash
.venv/bin/python3 scripts/run_cpm_experiment.py --jobs 5                          # every block, seeds from the YAML
.venv/bin/python3 scripts/run_cpm_experiment.py --block honest --seed-list 1       # one block, one seed
.venv/bin/python3 scripts/run_cpm_experiment.py --block position_faults --block speed_faults --seed-list 1 --jobs 5
.venv/bin/python3 scripts/run_cpm_experiment.py --id const_pos_offset --seed-list 1  # one configuration
.venv/bin/python3 scripts/run_cpm_experiment.py --block honest --dry-run          # print the commands only
```

Useful options: `--seed-list 1,2,3`, `--time-limit 30s`, `--scenario lust`, `--tag my_name`, `--jobs N`
(at most 5 on the grid, 3 on LuST).

Results go to `results/cpm_experiments/exp_<date>_<tag>/`: `REPORT.md` (all configurations), `runs/<id>_s<seed>/RUN.md`
(one run), `cpm_results.csv` (all metrics), `claims.csv` per run (sampled per-claim scores).

To merge several experiments into one report:

```bash
.venv/bin/python3 scripts/combine_experiments.py results/cpm_experiments/exp_A results/cpm_experiments/exp_B \
    --out results/cpm_experiments/combined_name
```

To rebuild the reports of an interrupted experiment: `.venv/bin/python3 scripts/report_experiment.py <exp folder>`.

## 3. Switch parameters and batches

All experiments are defined in `cpm_batch_config.yaml`:

* `defaults:` applies to every run (scenario, time, seeds, faulty share, error models, fault start times).
* Each other top-level key is a **block** (`honest`, `position_faults`, `speed_faults`, `cpm_faults`, `ablations`,
  `coverage`); each item in a block is one configuration with an `id` and the keys it changes.

To add a configuration, add an item to a block (or a new block):

```yaml
my_block:
  - id: offset_without_peers
    gnss_fault_profile: "const_pos_offset"
    enable_peers: false
```

and run it with `--block my_block`. The keys you can use are listed in `PARAMS` at the top of
`scripts/run_cpm_experiment.py` (each maps to one NED parameter); an unknown key stops the run. The main ones:

| Key | Values |
|---|---|
| `scenario` | `grid`, `lust` (configs of `omnetpp.ini`) |
| `faulty_rate` | share of faulty vehicles, e.g. `0.10` |
| `gnss_fault_profile` | `none`, `const_pos`, `const_pos_offset`, `random_pos`, `random_pos_offset` |
| `fault_profile` | `none`, `const_speed`, `const_speed_offset`, `random_speed`, `random_speed_offset` |
| `cpm_fault_profile` | `none`, `ghost`, `object_const_offset`, `object_random_offset` |
| `sensor_setup` | `front`, `front_rear` (default), `surround` |
| `enable_radar`, `enable_peers`, `enable_admissibility`, `enable_object_kinematics`, `enable_pose_check` | `true` / `false` (component ablations) |
| `gnss_error_model`, `radar_error_model` | `veremi` / `none`, `jacobs` / `none` |

Default values of every parameter, with their sources, are in `config/neds/CollectiveTrustService.ned` (the receiver)
and `config/neds/SensorNoiseService.ned` (the error models).

## 4. Stop a run

```bash
docker exec artery pkill -f opp_run   # stops the simulations; then stop the runner by its PID on the host
```
