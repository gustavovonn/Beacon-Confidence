#!/usr/bin/env python3
"""Fit the context-aware manoeuvre threshold d_max over CAM *and* CPM object streams.

    d_max = maxPosDev + contextSpeedWeight * (v * dt) + contextYawWeight * (|w| * v)

Paper 1 fitted this on CAM streams alone (`scorer_context_tuning.py`, from `TrustService`'s
`trust_context_dataset.csv`), giving 30.85 / 0.9874 / 0.0387. `CpTrustEngine::check()` then applies the
same three numbers unchanged to perceived-object streams, which differ in every input to the fit:

  * dt distribution     -- CPM generation rules, not CAM's
  * measurement noise   -- the author's radar at range r, not the receiver's view of one GNSS fix
  * history             -- one author's reports of one object, not a sender's own trajectory
  * reported yaw rate   -- absent from PerceivedObject, so it arrives as 0

So the threshold is being extrapolated outside the data it was fitted on. This script refits it on both
kinds and reports them side by side, so the choice between one shared d_max and one per kind is made on
evidence.

Method follows Paper 1's so the numbers stay comparable: least squares of distError on (v*dt) and
(|w|*v) over HONEST vectors only, then a safety margin at a high percentile of the residuals added to
the intercept.

Input is `context.csv`, written when `contextRecordFile` is set on CollectiveTrustService.

    scripts/context_tuning.py --run --time-limit 60s
    scripts/context_tuning.py results/cpm_experiments/<exp>/runs/<run>/context.csv
"""

import argparse
import csv
import math
import os
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
PAPER1 = {"maxPosDev": 30.85, "contextSpeedWeight": 0.9874, "contextYawWeight": 0.0387}


def load(paths, honest_only: bool = True):
    """Rows grouped by claim kind, merged over every seed. Only honest vectors define an honest budget."""
    by_kind = defaultdict(list)
    for path in paths:
        _load_one(path, by_kind, honest_only)
    return by_kind


def _load_one(path: Path, by_kind, honest_only: bool):
    with path.open(newline="") as fh:
        for row in csv.DictReader(fh):
            if honest_only and row.get("falsified", "0") not in ("0", "", "false"):
                continue
            try:
                r = {k: float(row[k]) for k in ("dt", "speed", "yaw_rate", "dist_error", "threshold")}
            except (KeyError, ValueError):
                continue
            if not all(math.isfinite(v) for v in r.values()) or r["dt"] <= 0.0:
                continue
            by_kind[row["kind"]].append(r)


def ols2(rows):
    """Least squares of y on [x1, x2] with an intercept, by the normal equations.

    Kept dependency-free on purpose: the venv has no scikit-learn, and a 3x3 solve does not need one.
    """
    n = len(rows)
    xs1 = [r["speed"] * r["dt"] for r in rows]
    xs2 = [abs(r["yaw_rate"]) * r["speed"] for r in rows]
    ys = [r["dist_error"] for r in rows]

    # normal equations A.b = c for b = [intercept, b1, b2]
    s1, s2, sy = sum(xs1), sum(xs2), sum(ys)
    s11 = sum(x * x for x in xs1)
    s22 = sum(x * x for x in xs2)
    s12 = sum(a * b for a, b in zip(xs1, xs2))
    s1y = sum(a * b for a, b in zip(xs1, ys))
    s2y = sum(a * b for a, b in zip(xs2, ys))
    A = [[n, s1, s2], [s1, s11, s12], [s2, s12, s22]]
    c = [sy, s1y, s2y]

    # Gauss-Jordan with partial pivoting
    M = [A[i][:] + [c[i]] for i in range(3)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-12:
            return None  # degenerate: a regressor is constant (e.g. yaw_rate is always 0)
        M[col], M[piv] = M[piv], M[col]
        d = M[col][col]
        M[col] = [v / d for v in M[col]]
        for r in range(3):
            if r != col and M[r][col] != 0.0:
                f = M[r][col]
                M[r] = [a - f * b for a, b in zip(M[r], M[col])]
    return M[0][3], M[1][3], M[2][3]


def percentile(values, q):
    if not values:
        return 0.0
    v = sorted(values)
    k = (len(v) - 1) * q / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return v[lo] if lo == hi else v[lo] + (v[hi] - v[lo]) * (k - lo)


def fit(kind, rows, margin_q):
    print(f"\n── {kind}  ({len(rows):,} honest vectors) " + "─" * 28)
    if len(rows) < 100:
        print("   too few samples to fit")
        return None
    yaw = [abs(r["yaw_rate"]) for r in rows]
    if max(yaw) <= 0.0:
        print("   NOTE: yaw_rate is identically 0 — PerceivedObject carries no yaw rate, so the yaw term")
        print("         cannot be fitted for this kind and is reported as 0.")

    res = ols2(rows)
    if res is None:
        # refit without the yaw regressor
        n = len(rows)
        xs = [r["speed"] * r["dt"] for r in rows]
        ys = [r["dist_error"] for r in rows]
        sx, sy = sum(xs), sum(ys)
        sxx = sum(x * x for x in xs)
        sxy = sum(a * b for a, b in zip(xs, ys))
        den = n * sxx - sx * sx
        if abs(den) < 1e-12:
            print("   degenerate: speed*dt is constant")
            return None
        b1 = (n * sxy - sx * sy) / den
        a = (sy - b1 * sx) / n
        b2 = 0.0
    else:
        a, b1, b2 = res

    resid = [r["dist_error"] - (a + b1 * r["speed"] * r["dt"] + b2 * abs(r["yaw_rate"]) * r["speed"])
             for r in rows]
    margin = percentile(resid, margin_q)
    intercept = a + margin

    errs = [r["dist_error"] for r in rows]
    over = sum(1 for r in rows if r["dist_error"] > r["threshold"])
    print(f"   distError  median {percentile(errs, 50):7.3f} m   p99 {percentile(errs, 99):8.3f} m   "
          f"max {max(errs):8.3f} m")
    print(f"   fit        distError ~ {a:.3f} + {b1:.4f}*(v*dt) + {b2:.4f}*(|w|*v)")
    print(f"   margin     p{margin_q} of residuals = {margin:+.3f} m")
    print(f"   exceeds the CURRENT d_max: {over:,} / {len(rows):,}  ({100.0 * over / len(rows):.2f} %)")
    print(f"   -> maxPosDev {intercept:8.2f}   contextSpeedWeight {b1:.4f}   contextYawWeight {b2:.4f}")
    return {"maxPosDev": intercept, "contextSpeedWeight": b1, "contextYawWeight": b2, "n": len(rows)}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="*", type=Path, help="context.csv files, or an experiment directory")
    ap.add_argument("--run", action="store_true", help="run an honest simulation first")
    ap.add_argument("--time-limit", default="60s")
    ap.add_argument("--seeds", type=int, default=1, help="number of seeds (1..N)")
    ap.add_argument("--jobs", type=int, default=3, help="parallel runs; 3 is the LuST cap")
    ap.add_argument("--tag", default="context_tuning")
    ap.add_argument("--margin", type=float, default=99.5,
                    help="residual percentile added to the intercept (Paper 1 used 99.5)")
    args = ap.parse_args()

    path = args.csv
    if args.run:
        cfg = HERE.parent / "cpm_context_tuning.yaml"
        cfg.write_text(
            "defaults:\n  scenario: \"lust\"\n  time_limit: \"%s\"\n  seeds: %s\n  faulty_rate: 0.0\n"
            "  gnss_error_model: \"veremi\"\n  radar_error_model: \"jacobs\"\n"
            "  context_log: true\n\n"
            "context:\n  - id: honest_context\n    description: \"Honest run logging phase 0/1 inputs\"\n"
            % (args.time_limit, list(range(1, args.seeds + 1))))
        cmd = [sys.executable, str(HERE / "run_cpm_experiment.py"), "--config", str(cfg),
               "--jobs", str(args.jobs), "--tag", args.tag]
        env = dict(os.environ)
        env.setdefault("ARTERY_BUILD_DIR", "/workspace/build-opt")
        print("+ " + " ".join(cmd))
        if subprocess.run(cmd, cwd=HERE.parent, env=env).returncode != 0:
            print("[FAIL] the run failed", file=sys.stderr)
            return 2
        latest = (HERE.parent / "results" / "cpm_experiments" / "latest").resolve()
        found = sorted(latest.glob("runs/*/context.csv"))
        if not found:
            print(f"[FAIL] no context.csv under {latest}", file=sys.stderr)
            return 2
        paths = found
    else:
        paths = []
        for p in (args.csv or []):
            paths.extend(sorted(p.glob("runs/*/context.csv")) if p.is_dir() else [p])
    if not paths:
        ap.error("give one or more context.csv files (or a directory), or pass --run")

    by_kind = load(paths)
    if not by_kind:
        print("[FAIL] no usable honest rows", file=sys.stderr)
        return 2

    for p in paths:
        print(f"\n{p}")
    print(f"Paper 1 (CAM only): maxPosDev {PAPER1['maxPosDev']}  "
          f"speedWeight {PAPER1['contextSpeedWeight']}  yawWeight {PAPER1['contextYawWeight']}")

    out = {k: fit(k, rows, args.margin) for k, rows in sorted(by_kind.items())}
    joint = [r for rows in by_kind.values() for r in rows]
    out["joint"] = fit("joint (CAM + CPM)", joint, args.margin)

    print("\n" + "=" * 72)
    print("A single d_max is defensible only if the per-kind fits agree. Where they do not, the shared")
    print("threshold is an extrapolation onto whichever kind it was not fitted on.")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    sys.exit(main())
