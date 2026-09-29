#!/usr/bin/env python3
"""
combine_experiments.py
Merges several experiment folders (results/cpm_experiments/exp_*) into one CSV and one report,
e.g. to put a grid and a LuST campaign, or separate blocks, side by side.

    python3 scripts/combine_experiments.py results/cpm_experiments/exp_A results/cpm_experiments/exp_B \
        --out results/cpm_experiments/combined_<name>
"""

import os
import sys

# --- VENV AUTO-BOOTSTRAP ---
if sys.prefix == sys.base_prefix:
    venv_python = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".venv", "bin", "python3"))
    if os.path.exists(venv_python):
        os.execv(venv_python, [venv_python] + sys.argv)
# ---------------------------

import argparse
import csv

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_cpm_experiment as runner  # noqa: E402


def load_rows(exp_dir: str):
    rows = []
    with open(os.path.join(exp_dir, "cpm_results.csv")) as f:
        for row in csv.DictReader(f):
            for k, v in list(row.items()):
                try:
                    row[k] = float(v)
                except (TypeError, ValueError):
                    pass
            row["experiment"] = os.path.basename(os.path.normpath(exp_dir))
            row["block"] = f"{row['experiment']} / {row['block']}"
            if not row.get("run_dir"):
                row["run_dir"] = os.path.join(exp_dir, "runs", f"{row['id']}_s{int(row['seed'])}")
            rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser(description="Combine experiment folders")
    parser.add_argument("experiments", nargs="+")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    rows = []
    for exp in args.experiments:
        rows += load_rows(exp)
    os.makedirs(args.out, exist_ok=True)
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(os.path.join(args.out, "combined_results.csv"), "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    runner.write_report(rows, os.path.join(args.out, "REPORT.md"),
                        {"combined": ", ".join(os.path.basename(os.path.normpath(e)) for e in args.experiments)})
    print(f"{len(rows)} runs -> {args.out}/REPORT.md")


if __name__ == "__main__":
    main()
