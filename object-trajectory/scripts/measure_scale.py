#!/usr/bin/env python3
"""
measure_scale.py -- how big the association problem actually is, measured on the dataset.

The cost of the pipeline is set by three numbers, and guessing them is how a feasibility claim goes
wrong:

    N   trajectories plausibly alive in one receiver's neighbourhood   -> the predict stage is O(N)
    m   perceived objects carried by one CPM                           -> gating is O(N*m)
    k   candidates falling inside the gate for one motion vector       -> the assignment is cubic
                                                                          in what is contested

`k` properly depends on the filter's covariance, which does not exist before the engine runs. This
script therefore uses a FIXED-RADIUS PROXY: how many other objects sit within 10, 20 and 30 m of a
given one at the same instant. That brackets `k` without needing a filter, which is all this step
has to do -- the engine's own run reports the true `k` afterwards.

Reads only the dataset. No engine, no simulator.

    ./scripts/measure_scale.py --dataset dataset/groundtruth
"""

import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict

# Metres per degree near the dataset's latitude. Good to a fraction of a percent over a city, and
# this script only needs neighbour counts, not survey-grade positions.
def _scale(lat_deg):
    lat = math.radians(lat_deg)
    return (111132.92 - 559.82 * math.cos(2 * lat) + 1.175 * math.cos(4 * lat),
            111412.84 * math.cos(lat) - 93.5 * math.cos(3 * lat))


def percentiles(values, qs=(50, 90, 99, 100)):
    if not values:
        return {q: 0.0 for q in qs}
    s = sorted(values)
    out = {}
    for q in qs:
        i = min(len(s) - 1, int(round(q / 100.0 * (len(s) - 1))))
        out[q] = s[i]
    return out


def describe(name, values, unit=""):
    if not values:
        print(f"  {name:<34} no samples")
        return
    p = percentiles(values)
    mean = sum(values) / len(values)
    print(f"  {name:<34} n={len(values):<8} mean={mean:7.2f}  "
          f"p50={p[50]:7.2f}  p90={p[90]:7.2f}  p99={p[99]:7.2f}  max={p[100]:7.2f} {unit}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", default="dataset/groundtruth")
    ap.add_argument("--radii", default="10,20,30",
                    help="fixed-radius proxies for k, in metres")
    ap.add_argument("--bucket", type=float, default=0.1,
                    help="time quantisation [s] for 'the same instant'")
    args = ap.parse_args()

    cam_path = os.path.join(args.dataset, "cam.csv")
    cpm_path = os.path.join(args.dataset, "cpm.csv")
    rx_path = os.path.join(args.dataset, "reception.csv")
    for p in (cam_path, cpm_path, rx_path):
        if not os.path.exists(p):
            sys.exit(f"missing {p}")

    radii = [float(r) for r in args.radii.split(",")]

    # ---- m: objects per CPM -----------------------------------------------------------------
    objects_per_cpm = defaultdict(int)
    # ---- true positions per instant, for the k proxy ----------------------------------------
    # Keyed by quantised time; one entry per (station) so an object reported by several authors is
    # counted once. The oracle is what makes that deduplication possible at all.
    positions = defaultdict(dict)

    origin = None
    with open(cpm_path, newline="") as f:
        for row in csv.DictReader(f):
            objects_per_cpm[row["message_id"]] += 1
            truth = row.get("true_station") or ""
            if not truth:
                continue
            lat, lon = float(row["author_lat"]), float(row["author_lon"])
            if origin is None:
                origin = (lat, lon)
            mlat, mlon = _scale(origin[0])
            x = (lon - origin[1]) * mlon + float(row["dx"])
            y = (lat - origin[0]) * mlat + float(row["dy"])
            t = round(float(row["t"]) / args.bucket) * args.bucket
            positions[round(t, 3)][truth] = (x, y)

    # ---- N: how many distinct objects a receiver could plausibly be tracking -----------------
    # Proxy: distinct authors plus distinct true objects seen by that receiver inside a sliding
    # second. A track that has not been fed for the engine's retireAfter is gone, so a one-second
    # window is the right order of magnitude for "alive".
    seen = defaultdict(lambda: defaultdict(set))   # receiver -> time bucket -> ids
    content_author = {}
    content_objects = defaultdict(set)
    with open(cam_path, newline="") as f:
        for row in csv.DictReader(f):
            content_author[row["message_id"]] = row["sender"]
    with open(cpm_path, newline="") as f:
        for row in csv.DictReader(f):
            truth = row.get("true_station") or ""
            if truth:
                content_objects[row["message_id"]].add(truth)

    with open(rx_path, newline="") as f:
        for row in csv.DictReader(f):
            mid, rx = row["message_id"], row["receiver"]
            t = round(float(row["t_rx"]))
            if mid in content_author:
                seen[rx][t].add(content_author[mid])
            for o in content_objects.get(mid, ()):
                seen[rx][t].add(o)

    n_samples = [len(ids) for buckets in seen.values() for ids in buckets.values()]
    m_samples = list(objects_per_cpm.values())

    # ---- k proxy: neighbours within each radius ----------------------------------------------
    k_samples = {r: [] for r in radii}
    for _, by_station in positions.items():
        pts = list(by_station.values())
        if len(pts) < 2:
            for r in radii:
                k_samples[r].append(1)      # itself only
            continue
        for i, (xi, yi) in enumerate(pts):
            d = [math.hypot(xi - xj, yi - yj) for j, (xj, yj) in enumerate(pts) if j != i]
            for r in radii:
                k_samples[r].append(1 + sum(1 for v in d if v <= r))

    print(f"dataset {args.dataset}")
    meta_path = os.path.join(args.dataset, "metadata.json")
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)
        print(f"  manifest counts: {meta.get('counts')}")

    print("\nN -- distinct objects one receiver saw within a one-second window")
    describe("N", n_samples)
    print("\nm -- perceived objects per CPM")
    describe("m", m_samples)
    print("\nk -- fixed-radius proxy: objects within R of one object, same instant")
    for r in radii:
        describe(f"k (R = {r:.0f} m)", k_samples[r])

    # The sentence this step exists to produce.
    worst_n = percentiles(n_samples)[99] if n_samples else 0
    worst_m = percentiles(m_samples)[99] if m_samples else 0
    worst_k = percentiles(k_samples[max(radii)])[99] if k_samples[max(radii)] else 0
    print(f"\nAt the 99th percentile: N = {worst_n:.0f}, m = {worst_m:.0f}, "
          f"k = {worst_k:.0f} (R = {max(radii):.0f} m).")
    print(f"One assignment therefore costs at most about {worst_k ** 3:.0f} operations, and the "
          f"predict stage\nruns over {worst_n:.0f} trajectories. Feed these to cost_evaluator with "
          f"--tracks {worst_n:.0f} --vectors {worst_m:.0f}.")

    # sanity bounds the plan asks for
    print("\nsanity")
    if m_samples:
        print(f"  largest CPM carried {max(m_samples)} objects")
    if n_samples:
        print(f"  largest neighbourhood seen by one receiver: {max(n_samples)} distinct objects")


if __name__ == "__main__":
    main()
