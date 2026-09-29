#!/usr/bin/env python3
"""
report_experiment.py
(Re)builds RUN.md for every finished run and REPORT.md for the experiment, from the files on disk.
Works on experiments still running (finished runs only) and on older experiments.

    python3 scripts/report_experiment.py results/cpm_experiments/<exp>      # or: latest
"""

import os
import sys

# --- VENV AUTO-BOOTSTRAP ---
if sys.prefix == sys.base_prefix:
    venv_python = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".venv", "bin", "python3"))
    if os.path.exists(venv_python):
        os.execv(venv_python, [venv_python] + sys.argv)
# ---------------------------

import json
import re

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_cpm_experiment as runner  # noqa: E402


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    exp_dir = sys.argv[1]
    if not os.path.isabs(exp_dir) and not os.path.exists(exp_dir):
        exp_dir = os.path.join(runner.RESULTS_DIR, exp_dir)
    exp_dir = os.path.realpath(exp_dir)

    configs, blocks = {}, {}
    meta_path = os.path.join(exp_dir, "metadata.json")
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            for run in json.load(f).get("runs", []):
                configs[run["id"]] = {k: v for k, v in run.items() if k != "block"}
                blocks[run["id"]] = run.get("block", "")

    rows, running = [], []
    runs_dir = os.path.join(exp_dir, "runs")
    for name in sorted(os.listdir(runs_dir)):
        match = re.match(r"(.+)_s(\d+)$", name)
        if not match:
            continue
        run_id, seed = match.group(1), int(match.group(2))
        run_dir = os.path.join(runs_dir, name)
        sca = os.path.join(run_dir, "results.sca")
        finished = os.path.exists(os.path.join(run_dir, "run.log"))
        if not finished:
            running.append(name)
            continue
        cfg = configs.get(run_id, {"id": run_id})
        row = {"block": blocks.get(run_id, ""), "id": run_id, "seed": seed, "run_dir": run_dir,
               **{k: v for k, v in cfg.items() if k != "id"}}
        if os.path.exists(sca):
            row.update(status="ok", **runner.compute_metrics(runner.read_scalars(sca)))
        else:
            row["status"] = "failed"
        runner.write_run_report(run_dir, row, cfg)
        rows.append(row)

    order = list(configs)
    rows.sort(key=lambda r: (order.index(r["id"]) if r["id"] in order else len(order), r["seed"]))
    total = len(configs) * max(1, len({r["seed"] for r in rows})) if configs else len(rows)
    header = {"experiment": os.path.basename(exp_dir), "progress": f"{len(rows)} runs finished"
              + (f", running: {', '.join(running)}" if running else "")}
    runner.write_report(rows, os.path.join(exp_dir, "REPORT.md"), header)
    runner.write_csv(rows, os.path.join(exp_dir, "cpm_results.csv"))
    print(f"{len(rows)} finished runs, {len(running)} running -> {exp_dir}/REPORT.md")


if __name__ == "__main__":
    main()
