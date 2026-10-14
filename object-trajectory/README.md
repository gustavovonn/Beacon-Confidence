# object-trajectory

Receiver-local confidence attached to a **fused object trajectory**, derived from V2X motion
vectors that may arrive in a CAM or in a CPM's perceived-object container. Past the parser the
source format stops mattering: both become a `MotionVector`, and the engine never learns which wire
carried one.

This is a self-contained experiment. Nothing here is on the LKPM paper's code path, and the only
code reused from `cp-trustfulness` is the simulation scenario — the base `omnetpp.ini`, the LuST
configuration, and the `ArteryAdapters` parsers — all confined to `artery/`.

## Architecture: two build targets, kept separate on purpose

| | What | Built by | Depends on |
|---|---|---|---|
| **Engine** | `include/`, `src/`, `tools/` | `Makefile` | a C++17 compiler, nothing else |
| **Recorder** | `artery/` | Artery's CMake | OMNeT++, Artery, Vanetza, SUMO |

The split is the point. The engine must run on a **VisionFive 2** — a RISC-V board running bare
Linux with no simulator — so it must build with `make` alone. The recorder runs once on a
workstation to produce the dataset.

## Quick start

```sh
make              # build the engine and tools
make sweep        # per-message cost table (the feasibility numbers)
make run          # replay dataset/groundtruth through the engine
make validate     # cross-check the UKF step-by-step against FilterPy
```

## Pipeline

`onMotionVectors(vectors, receiver, now)` takes a **whole message at once**. A CPM carries many
objects and the assignment is a joint decision over all of them against all tracks. Feeding vectors
one at a time silently degrades joint assignment into greedy first-fit, which is exactly the
configuration that swaps identities when two objects cross.

```
1  plausibilityChecks     drop impossible vectors against absolute bounds          O(m)
2  projectKnownObjects    UKF predict every track, ONCE per message                O(N)
3  associate              coarse gate → ellipsoidal gate → joint assignment        O(k³)
4  kalmanUpdate           fold the gated reports into their matched tracks
   computeTrajectoryConfidence
5  initiateTrack          start a new trajectory for every unmatched vector
6  retireStaleTracks      expire tracks nobody has fed recently
```

The predict runs once per message, not once per vector — otherwise the dominant term becomes O(N·m)
and the stage name becomes misleading.

## Motion model

State `[px, py, v, θ, ω, a]` — position, speed, heading, yaw rate, and acceleration. CTRA is
nonlinear (it contains `sin(θ)` and `v/ω`), so an EKF would need thirty-six hand-written Jacobian
entries that no library validates. The UKF avoids all of that: it places thirteen sigma points
around the current belief, pushes each one through the real nonlinear motion model, and reads the
mean and spread of the result.

Only the prediction is unscented. The measurement update is not, and that is not a shortcut: a
motion vector reports the first four states directly, making the observation a plain row-selection.
For a linear observation the unscented transform is exact and collapses to the ordinary Kalman
update — running sigma points through it costs more and computes the same numbers.

**Two things that bite and are handled:**

- **ω → 0.** CTRA divides by the turn rate. Below `kStraight` the model uses the second-order
  Taylor expansion of the arc, *not* constant velocity — dropping the `ω·dt²` term makes the state
  independent of ω while the Jacobian still has a `∂/∂ω` entry, and the two then disagree.
  This is caught by a finite-difference check in the CTRA test suite.

- **Angle wrapping.** `θ` is in the state, so every heading residual and every sigma-point average
  must go through `wrapPi`. Averaging angles naively breaks at the seam.

The UKF is cross-checked numerically against FilterPy (`scripts/validate_against_filterpy.py`):
six state values and thirty-six covariance entries compared after every predict and every update,
over two hundred deterministic steps.

## Confidence

`computeTrajectoryConfidence` is a **seam**: `setConfidencePolicy()` replaces it without touching
anything above. The default maintains Beta counters `α`/`β` on each track and publishes
`α / (α + β)`.

Every fused report splits one unit of evidence between "this trajectory is real and moving as
believed" and "it is not", in proportion to how well the report agreed. A report landing at the
gate's centre is strong evidence *for*; one that barely cleared the edge is mostly evidence
*against*, even though it was accepted. Treating weak agreement as neutral evidence is what freezes
a score — confidence then only ever rises, and a track being fed bad reports looks exactly like one
being fed good ones.

The counters live on the track rather than being derived from the published scalar because a single
number cannot be un-updated: `1.0` is a fixed point of any "add evidence to both sides" rule.

## Dataset

`dataset/groundtruth/` is honest by construction: what was transmitted and who received it, without
errors or faults.

| File | Contents |
|---|---|
| `cam.csv` | one row per CAM: sender pose and kinematics |
| `cpm.csv` | one row per perceived object: author pose, then **author-relative** `dx, dy`, plus the `true_station` oracle |
| `reception.csv` | `(message, receiver, arrival_time)` |
| `metadata.json` | every frozen parameter and the row counts |

**Perceived objects are stored relative to their author, never absolute.** Radar noise acts in the
author's sensor frame; the author's GNSS error then displaces that whole frame. Flattening them to
absolute at recording time makes the two inseparable — and no later injector can pull them apart
again. `scripts/subset_builder.py` applies error and attack models at ingestion, so one expensive
simulation serves every experimental subset.

A CAM heard by forty neighbours is one content row and forty reception rows. Content is
deduplicated by message id.

### Regenerating the dataset

Must run from `cp-trustfulness/artery/` — the base `omnetpp.ini`'s paths are relative to it.

```sh
docker exec -w /workspace/cp-trustfulness/artery artery \
    $ARTERY_BUILD_DIR/run_artery -f ../../object-trajectory/artery/omnetpp.ini -c dataset-lust -u Cmdenv
```

`artery/omnetpp.ini` *extends* `cp-trustfulness/artery/omnetpp.ini` rather than restating it, so
LuST, its region of interest, GEMV² propagation, and the ITU-R M.2057-1 radar field of view stay
exactly as configured there. `SensorNoiseService` is loaded error-free: it is what populates the
`objectStation` oracle sender-side, and without the oracle there is no association ground truth.

## Measurement and analysis

```sh
# Feasibility: does this fit in a 100 ms V2X message interval?
./build/cost_evaluator --sweep

# What N, m, and k actually look like on this dataset
./scripts/measure_scale.py

# Full replay, scored against the oracle
./build/experiment_runner --out results/run.csv

# AUC, error-by-k, CAM vs CPM split, confidence threshold sweep
./scripts/report.py results/run.csv

# Parameter sweeps (gate, noise, retirement threshold, confirmation count)
./scripts/run_experiment_batch.py
```

`cost_evaluator` prints the host CPU alongside every result — a table from the VisionFive 2 and
the same table from a workstation are otherwise indistinguishable. Run `make sweep` on the board to
replace the projected numbers with measurements.

## Cost on the VisionFive 2

The engine is cross-compiled static for RISC-V, packed into a BusyBox initramfs, and booted over
TFTP. Same pipeline as `cp-trustfulness/f2md`, so both experiments are driven the same way.

```sh
./scripts/vf2_setup.sh                    # build, pack, offer TFTP deploy
./scripts/vf2_setup.sh --with-data        # also ship a 20 s dataset slice for real-trace replay
./scripts/vf2_setup.sh --rebuild-all      # rebuild BusyBox and the project from scratch
```

On the board after boot:

```sh
cd /bench && ./run_benchmarks_riscv.sh
```

On the host, for the reference numbers and the side-by-side comparison:

```sh
./scripts/run_benchmarks_x86.sh
./scripts/compare_benchmarks.sh results/vf2_compare/summary_x86.txt <summary_riscv.txt>
```

Both runners measure the same three load points — `sparse` (N=25, m=1), `operating` (N=61, m=5),
and `dense` (N=150, m=11) — taken from what `measure_scale.py` found on LuST rather than from
round numbers.

Everything links statically because the initramfs has no shared libraries. Eigen is header-only,
vendored in `shared/eigen/`, and cross-compiles with the rest.

## Repository layout

```
include/     MotionVector, UnscentedCTRA, ObjectTrack, ObjectConfidenceEngine,
             DatasetParser, MeasurementSpace, Hungarian, StageTimer
src/         UnscentedCTRA.cc, ObjectConfidenceEngine.cc, DatasetParser.cc
tools/       cost_evaluator, experiment_runner, ukf_trace
scripts/     measure_scale, subset_builder, run_experiment_batch, report,
             validate_against_filterpy, run_benchmarks_{x86,riscv}, compare_benchmarks, vf2_setup
artery/      the recorder: simulator-coupled, built by Artery's CMake, never by this Makefile
shared/      vendored copies — Eigen 3.4.0, scipy's rectangular LSAP solver, LabCtraPredict, Geodesy
             each file carries a provenance header; copies, not submodule includes
dataset/     groundtruth/ (produced by the recorder) and subsets/ (produced by subset_builder)
results/     experiment_runner and benchmark output (gitignored)
```

`shared/` holds copies rather than includes so the engine builds on the board with nothing
available. If an upstream file changes, the copy does not follow automatically — update it and
update the provenance header.