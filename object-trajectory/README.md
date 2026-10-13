# Object-trajectory confidence

Confidence attached to a **fused object trajectory**, built from motion vectors that may arrive in a
CAM or in a CPM's perceived-object container. Past the parser the container stops mattering: both
become a `MotionVector`, and the engine never learns which wire format carried one.

This is a self-contained experiment. Nothing here is on Paper 1's code path, and the only code reused
from it is CPM/CAM parsing and the evaluation oracle, both confined to the recorder.

## Two build targets, on purpose

| | What | Built by | Depends on |
|---|---|---|---|
| **Engine** | `include/`, `src/`, `tests/`, `tools/` | `Makefile` | a C++17 compiler, nothing else |
| **Recorder** | `artery/` | Artery's CMake | OMNeT++, Artery, Vanetza, SUMO |

The split is the point. The engine has to run on a **VisionFive 2** — a RISC-V board with a bare
Linux and no simulator — so it must build with a compiler and `make` alone. The recorder runs once on
a workstation and leaves a dataset behind.

```
make            build engine, tests and tools
make test       run every suite
make sweep      the feasibility table (this is the number the advisor asked for)
make run        replay dataset/groundtruth through the engine
```

## Pipeline

```
onMotionVectors(vectors, receiver, now)
  1  plausibilityChecks      drop the impossible                      O(m)
  2  projectKnownObjects     EKF predict, every track, ONCE           O(N)
  3  associate               coarse gate -> ellipsoidal gate -> joint assignment
  4  kalmanUpdate            fuse what the gate admitted
  5  initiateTrack           whatever matched nothing
  6  retireStaleTracks       whatever nobody feeds
```

Three decisions worth knowing before reading the code:

**Batched entry point.** A CPM carries many objects and the assignment is a *joint* decision over all
of them. One call per object silently degrades it into greedy first-fit and makes track swaps worse.

**Predict once per message, not per vector.** Otherwise the dominant linear term becomes `O(N*m)`.

**`kalmanUpdate`, not `kalmanFusion`.** Predict already happened in stage 2. Calling it "fusion"
invites predicting twice, which inflates the covariance, widens every gate, and fails silently.

## Motion model: CTRA with an EKF

State `[px, py, v, psi, omega]`. CTRA is nonlinear — it contains `sin(psi)` and `v/omega` — so there
is no fixed matrix `F`. The EKF moves the **state** with the true nonlinear equations and the
**covariance** with the Jacobian evaluated at the current state.

Two things bite and both are handled:

- **`omega -> 0`.** CTRA divides by the turn rate, and a vehicle driving straight has none. Below
  `kStraight` the model uses the second-order Taylor expansion of the arc, *not* plain constant
  velocity — dropping the `w*dt^2` term makes the state independent of `omega` while the Jacobian
  still has a `d/d omega` entry, and the two then disagree. `tests/test_core.cc` catches exactly this
  by finite differences, and it did.
- **Angle wrapping.** `psi` is in the state, so a heading residual of 179° against −179° is +2°, not
  +358°. Every heading difference goes through `wrapPi`.

## Dataset

`dataset/groundtruth/` is honest by construction: what was transmitted and who received it, no error,
no fault, no judgement.

| File | Holds |
|---|---|
| `cam.csv` | one row per CAM: sender pose and kinematics |
| `cpm.csv` | one row per perceived object: author pose, then **author-relative** `dx, dy`, plus the `true_station` oracle |
| `reception.csv` | `(message, receiver, arrival time)` |
| `metadata.json` | every frozen parameter and the row counts |

**Perceived objects are stored relative to their author, never absolute.** Radar noise acts in the
author's sensor frame; the author's GNSS error then displaces that whole frame. Flatten them together
at recording time and no later injector can separate them again — the one irreversible mistake.
`scripts/subset_builder.py` is where error and attacks are applied, at ingestion, so one expensive
simulation serves every subset.

Content is written once and deduplicated by message id; `reception.csv` carries the per-receiver
arrivals. A CAM heard by forty neighbours is one content row and forty reception rows.

### Regenerating it

Must run from `cp-trustfulness/artery/` — the base `omnetpp.ini`'s paths are relative to it.

```sh
docker exec -w /workspace/cp-trustfulness/artery artery \
    $ARTERY_BUILD_DIR/run_artery -f ../../object-trajectory/artery/omnetpp.ini -c dataset-lust -u Cmdenv
```

`artery/omnetpp.ini` **extends** `cp-trustfulness/artery/omnetpp.ini` rather than restating it, so LuST, its region of interest,
GEMV² propagation and the ITU-R M.2057-1 radar field of view stay exactly as configured there. Only
the service set and the error models are overridden. `SensorNoiseService` is loaded but set to
`gnssErrorModel = "none"` and `radarErrorModel = "none"`: it is what populates the `objectStation`
oracle sender-side, and without the oracle there is no association ground truth and no metric worth
reporting.

## Measuring

```sh
./build/cost_evaluator --sweep                      # cost against a per-message budget
./scripts/measure_scale.py                          # what N, m and k actually are on this dataset
./build/experiment_runner --out results/run.csv     # replay, scored against the oracle
./scripts/report.py results/run.csv                 # AUC, error-by-k, CAM vs CPM, thresholds
./scripts/run_experiment_batch.py                   # parameter sweeps
```

`cost_evaluator` generates its own synthetic load and prints the host CPU, so a result from the board
can never be mistaken for one from a workstation. Run `make sweep` on the VisionFive 2 to replace the
projection with a measurement.

## Cost on the VisionFive 2

The engine is cross-compiled static for RISC-V, packed into a BusyBox initramfs and booted on the
board over TFTP. Same pipeline as `cp-trustfulness/f2md`, so both experiments are driven the same
way.

```sh
./scripts/vf2_setup.sh                    # build, pack, offer to deploy over TFTP
./scripts/vf2_setup.sh --with-data        # also ship a 20 s dataset slice for the real replay
./scripts/vf2_setup.sh --rebuild-all      # rebuild BusyBox and the project from scratch
```

On the board, after it boots:

```sh
cd /bench && ./run_benchmarks_riscv.sh
```

On the host, for the reference numbers, then the comparison:

```sh
./scripts/run_benchmarks_x86.sh
./scripts/compare_benchmarks.sh results/vf2_compare/summary_x86.txt <summary_riscv.txt>
```

Three things are deliberate here:

**The headline benchmark needs no data.** `cost_evaluator` generates its own synthetic load and
prints the host CPU, so a board result can never be mistaken for a workstation one. The dataset
slice is optional and only exists for the real-trace replay.

**The test suites go on the board too.** A cost number from a binary whose own tests have not
passed on that silicon is worth little, and cross-compilation can change floating-point
behaviour.

**Both runners measure the same three points** — `sparse` (N=25, m=1), `operating` (N=61, m=5) and
`dense` (N=150, m=11) — taken from what `measure_scale.py` found on LuST rather than from round
numbers. The comparison is void if the two scripts ever drift apart, which is why both say so at
the top.

Everything links statically because the initramfs has no shared libraries and no loader. Eigen
needs nothing extra: it is headers, vendored in `shared/eigen/`, and cross-compiles with the rest.

## Layout

```
include/     MotionVector, KalmanCTRA_EKF, ObjectTrack, ObjectConfidenceEngine, DatasetParser, StageTimer
src/         the three translation units
shared/      code copied in from elsewhere, each file carrying a provenance header
artery/      the recorder: the only simulator-coupled code
tools/       cost_evaluator, experiment_runner
scripts/     measure_scale, subset_builder, run_experiment_batch, report
tests/       test_core, test_pipeline, test_dataset -- plain binaries, no framework to install
dataset/     groundtruth/ and subsets/
```

`shared/` holds copies, not includes, so the engine builds on the board with nothing available. Each
file names where it came from; if the original changes, the copy does not follow automatically.

## Confidence

`computeTrajectoryConfidence` is a **seam**: `setConfidencePolicy()` replaces it without touching
anything above. The default keeps Beta counters `alpha`/`beta` on the track and publishes
`alpha / (alpha + beta)`.

Every fused report splits one unit of evidence between "this trajectory is real and moving as
believed" and "it is not", by how well the report agreed. The split is the part that matters: a
report landing mid-gate is strong evidence *for*, while one that barely squeezed inside is mostly
evidence *against*, even though it was accepted. Treating a weak agreement as merely "no evidence" is
what freezes a score — confidence then only ever rises, and a trajectory being fed bad reports looks
exactly like one being fed good ones.

The counters live on the track rather than being derived from the published scalar because a single
number cannot be un-updated: 1.0 is a fixed point of any "add evidence to both sides" rule.
