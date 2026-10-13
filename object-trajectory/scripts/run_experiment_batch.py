#!/usr/bin/env python3
"""
run_experiment_batch.py -- sweep engine parameters over one dataset and tabulate the outcomes.

Qualitative, not a tuner. The MVP's claim is that a lightweight pipeline produces a confidence that
tracks association correctness; the point of a sweep is to show the behaviour is not an artefact of
one lucky parameter set, and to expose which knobs actually move anything. It is NOT an attempt to
find the best configuration -- fitting parameters to the one dataset we have would just be
overfitting with extra steps.

Every point reuses the SAME dataset. The dataset is expensive and the engine is cheap, so a sweep is
seconds of CPU; regenerating the trace per point would also change the traffic underneath and make
the comparison meaningless.

    ./scripts/run_experiment_batch.py --dataset dataset/groundtruth
    ./scripts/run_experiment_batch.py --sweep gate
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RUNNER = os.path.join(ROOT, "build", "experiment_runner")

# Each sweep is one knob moved across a defensible range, everything else at its default.
# The baseline is always included so every table has something to be read against.
SWEEPS = {
    # chi-square critical values for 4 degrees of freedom: 90 %, 95 %, 99 %, 99.9 %, plus a
    # deliberately loose one to show what over-wide gating costs
    "gate": ("--gate", [7.78, 9.49, 13.28, 18.47, 30.0]),
    # R for position. The gate is a threshold on gap-over-doubt, so this is the knob that decides
    # how wide the gate is in metres -- arguably more important than the gate value itself.
    "pos-var": ("--pos-var", [0.25, 1.0, 4.0, 9.0]),
    # how long a track survives without being fed; shorter means more fragmentation, longer means
    # more stale tracks competing for new reports
    "retire-after": ("--retire-after", [0.5, 1.0, 2.0, 5.0]),
    # M-of-N confirmation
    "confirm-after": ("--confirm-after", [1, 2, 3, 5]),
    # doubt a new track starts with: too small and its gate is shut before it has any evidence
    "initial-pos-var": ("--initial-pos-var", [4.0, 25.0, 100.0]),
}

LINE = re.compile(r"^BATCH (?P<label>\S+) (?P<rest>.*)$", re.M)


def run_point(dataset, label, extra):
    cmd = [RUNNER, "--dataset", dataset, "--no-csv", "--label", label] + extra
    t0 = time.time()
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=3600)
    except subprocess.TimeoutExpired:
        print(f"  {label}: TIMED OUT", file=sys.stderr)
        return None
    if res.returncode != 0:
        print(f"  {label}: runner failed ({res.returncode})\n{res.stderr}", file=sys.stderr)
        return None
    m = LINE.search(res.stdout)
    if not m:
        print(f"  {label}: no BATCH line in output", file=sys.stderr)
        return None
    point = {"label": label, "seconds": time.time() - t0}
    for kv in m.group("rest").split():
        k, _, v = kv.partition("=")
        point[k] = float(v) if _ else v
    return point


def table(points, knob):
    if not points:
        print("  nothing to show")
        return
    cols = [(knob, 12), ("accuracy", 11), ("wrong", 9), ("switches", 10), ("started", 10),
            ("mean_k", 9), ("conf_correct", 14), ("conf_wrong", 12), ("sep", 8)]
    print("  " + "".join(f"{name:>{w}}" for name, w in cols))
    print("  " + "-" * sum(w for _, w in cols))
    for p in points:
        sep = p["conf_correct"] - p["conf_wrong"]
        vals = [p["label"].split("=")[-1], f"{p['accuracy']:.4f}", f"{int(p['wrong'])}",
                f"{int(p['switches'])}", f"{int(p['started'])}", f"{p['mean_k']:.3f}",
                f"{p['conf_correct']:.4f}", f"{p['conf_wrong']:.4f}", f"{sep:+.3f}"]
        print("  " + "".join(f"{v:>{w}}" for v, (_, w) in zip(vals, cols)))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", default=os.path.join(ROOT, "dataset", "groundtruth"))
    ap.add_argument("--sweep", action="append",
                    help=f"one of {', '.join(SWEEPS)}; repeatable; default is all")
    ap.add_argument("--out", default=os.path.join(ROOT, "results"))
    args = ap.parse_args()

    if not os.path.exists(RUNNER):
        sys.exit(f"missing {RUNNER} -- run: make all")
    if not os.path.isdir(args.dataset):
        sys.exit(f"missing dataset {args.dataset}")

    chosen = args.sweep or list(SWEEPS)
    unknown = [s for s in chosen if s not in SWEEPS]
    if unknown:
        sys.exit(f"unknown sweep(s): {', '.join(unknown)}; known: {', '.join(SWEEPS)}")

    os.makedirs(args.out, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(args.out, f"batch_{stamp}")
    os.makedirs(run_dir, exist_ok=True)

    # The dataset's manifest travels with the results: a table of accuracies means nothing without
    # knowing which trace produced it.
    meta = os.path.join(args.dataset, "metadata.json")
    if os.path.exists(meta):
        shutil.copy2(meta, os.path.join(run_dir, "dataset_metadata.json"))

    all_points = []
    for name in chosen:
        flag, values = SWEEPS[name]
        print(f"\nsweep: {name}")
        points = []
        for v in values:
            label = f"{name}={v}"
            p = run_point(args.dataset, label, [flag, str(v)])
            if p:
                points.append(p)
                all_points.append(p)
        table(points, name)

    # one extra point: the whole class discrimination turned off, which the plan calls out as free
    # discrimination the motion vector already carries
    print("\nablation: ignore objectClass when associating")
    base = run_point(args.dataset, "class=on", [])
    off = run_point(args.dataset, "class=off", ["--ignore-class"])
    pts = [p for p in (base, off) if p]
    all_points.extend(pts)
    table(pts, "class")

    csv_path = os.path.join(run_dir, "batch.csv")
    if all_points:
        keys = ["label"] + [k for k in all_points[0] if k != "label"]
        with open(csv_path, "w") as f:
            f.write(",".join(keys) + "\n")
            for p in all_points:
                f.write(",".join(str(p.get(k, "")) for k in keys) + "\n")
        print(f"\n{len(all_points)} points written to {csv_path}")


if __name__ == "__main__":
    main()
