#!/usr/bin/env python3
"""
subset_builder.py -- materialise a subset of the honest dataset, applying an injector chain.

Why this is a separate tool, and separate from day one even though the MVP's only chain is empty:

The recording in dataset/groundtruth/ is honest by construction. Every error model, every fault and
every attack is a transformation applied to it afterwards. Keeping that transformation in its own
tool means it can never leak into the engine, and it means one expensive simulation serves every
subset -- honest, noisy, attacked -- instead of one run per condition.

The schema is what makes this possible. Perceived objects are stored as offsets from their author,
with the author's pose in separate columns, so the two error sources act independently:

    author pose error (GNSS)  ->  displaces the whole frame, moving every object the author reports
    sensor error (radar)      ->  perturbs one offset, leaving the author's other objects alone

Flatten those to absolute positions at recording time and no injector can tell them apart again.
That is the one irreversible mistake, and it is why this tool reads dx/dy rather than lat/lon.

    ./scripts/subset_builder.py --out honest
    ./scripts/subset_builder.py --out noisy --inject gnss:5.0 --inject radar:0.4
"""

import argparse
import csv
import json
import math
import os
import random
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


# --- injectors ---------------------------------------------------------------------------------
# Each takes the parsed row and the shared RNG and mutates it in place. They are deliberately
# trivial: the MVP does not use them, and a placeholder that pretends to be a referenced error model
# would be worse than none. When error models return, they come from the references, not from here.

def inject_gnss(bound_m):
    """
    Displace an author's whole frame: its own CAM position and, with it, every object it reports.

    A constant per-author offset, drawn once, because that is the part of a GNSS error that does not
    average away -- and the part the engine cannot see, since a constant offset shared by all of an
    author's reports is indistinguishable from the author really being there.
    """
    offsets = {}

    def apply(kind, row, rng):
        author = row["sender"] if kind == "cam" else row["author"]
        if author not in offsets:
            offsets[author] = (rng.uniform(-bound_m, bound_m), rng.uniform(-bound_m, bound_m))
        de, dn = offsets[author]
        if kind == "cam":
            _shift_latlon(row, "lat", "lon", de, dn)
        else:
            _shift_latlon(row, "author_lat", "author_lon", de, dn)

    return apply


def inject_radar(sigma_m):
    """Perturb ONE perceived object's offset, leaving the author's pose and its other objects alone."""

    def apply(kind, row, rng):
        if kind != "cpm":
            return
        row["dx"] = f"{float(row['dx']) + rng.gauss(0.0, sigma_m):.4f}"
        row["dy"] = f"{float(row['dy']) + rng.gauss(0.0, sigma_m):.4f}"

    return apply


def _shift_latlon(row, lat_key, lon_key, east_m, north_m):
    lat = float(row[lat_key])
    lat_rad = math.radians(lat)
    m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * lat_rad)
    m_per_deg_lon = 111412.84 * math.cos(lat_rad)
    row[lat_key] = f"{lat + north_m / m_per_deg_lat:.9f}"
    row[lon_key] = f"{float(row[lon_key]) + east_m / m_per_deg_lon:.9f}"


INJECTORS = {"gnss": inject_gnss, "radar": inject_radar}


def parse_injector(spec):
    name, _, arg = spec.partition(":")
    if name not in INJECTORS:
        sys.exit(f"unknown injector '{name}'; known: {', '.join(INJECTORS)}")
    try:
        value = float(arg)
    except ValueError:
        sys.exit(f"injector '{name}' needs a numeric argument, got '{arg}'")
    return name, value, INJECTORS[name](value)


def copy_with(src, dst, kind, chain, rng):
    with open(src, newline="") as fin:
        reader = csv.DictReader(fin)
        fields = reader.fieldnames
        if fields is None:
            sys.exit(f"{src} has no header")
        with open(dst, "w", newline="") as fout:
            writer = csv.DictWriter(fout, fieldnames=fields)
            writer.writeheader()
            n = 0
            for row in reader:
                for apply in chain:
                    apply(kind, row, rng)
                writer.writerow(row)
                n += 1
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--groundtruth", default=os.path.join(ROOT, "dataset", "groundtruth"))
    ap.add_argument("--out", required=True, help="subset name, created under dataset/subsets/")
    ap.add_argument("--inject", action="append", default=[],
                    help="name:value, e.g. gnss:5.0 or radar:0.4; repeatable, applied in order")
    ap.add_argument("--seed", type=int, default=20261009)
    args = ap.parse_args()

    gt = args.groundtruth
    for name in ("cam.csv", "cpm.csv", "reception.csv"):
        if not os.path.exists(os.path.join(gt, name)):
            sys.exit(f"missing {os.path.join(gt, name)}")

    out = os.path.join(ROOT, "dataset", "subsets", args.out)
    os.makedirs(out, exist_ok=True)

    chain_specs = []
    chain = []
    for spec in args.inject:
        name, value, fn = parse_injector(spec)
        chain_specs.append({"injector": name, "value": value})
        chain.append(fn)

    rng = random.Random(args.seed)
    n_cam = copy_with(os.path.join(gt, "cam.csv"), os.path.join(out, "cam.csv"), "cam", chain, rng)
    n_cpm = copy_with(os.path.join(gt, "cpm.csv"), os.path.join(out, "cpm.csv"), "cpm", chain, rng)
    # receptions describe who heard what and when: no injector touches them, because propagation is
    # the simulator's business and faking it here would contradict the recorded radio model
    shutil.copy2(os.path.join(gt, "reception.csv"), os.path.join(out, "reception.csv"))

    # the subset's own manifest, carrying the parent's and the chain that produced it
    parent = {}
    parent_path = os.path.join(gt, "metadata.json")
    if os.path.exists(parent_path):
        with open(parent_path) as f:
            parent = json.load(f)
    with open(os.path.join(out, "metadata.json"), "w") as f:
        json.dump({
            "subset": args.out,
            "parent": os.path.relpath(gt, ROOT),
            "parent_manifest": parent,
            "injectors": chain_specs,
            "seed": args.seed,
            "counts": {"cam_rows": n_cam, "cpm_rows": n_cpm},
        }, f, indent=2)
        f.write("\n")

    label = ", ".join(f"{c['injector']}={c['value']}" for c in chain_specs) or "none (honest copy)"
    print(f"wrote {out}")
    print(f"  injectors: {label}")
    print(f"  {n_cam} cam rows, {n_cpm} cpm rows, reception.csv copied unchanged")


if __name__ == "__main__":
    main()
