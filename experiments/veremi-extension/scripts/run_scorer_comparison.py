#!/usr/bin/env python3
"""
run_scorer_comparison.py
Comparative Study Wrapper for Reputation Scorers across VeReMi 0709 subsets.

Scorers Evaluated:
1. BetaReputation (Proposed: Conjugate Bayesian updating of alpha/beta parameters)
2. EWMA (Baseline: Exponentially Weighted Moving Average with lambda mapped from priorWeight)
3. MajorityVote (Baseline: Sliding window majority vote of binary pass/fail verdicts)

Generates structured comparison tables in terminal, Markdown, and CSV.
"""

import os
import sys

# --- VENV AUTO-BOOTSTRAP ---
if sys.prefix == sys.base_prefix:
    venv_python = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".venv", "bin", "python3"))
    if os.path.exists(venv_python):
        os.execv(venv_python, [venv_python] + sys.argv)
# ---------------------------

import glob
import re
import io
import time
import argparse
import subprocess
import pandas as pd
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
WORKSPACE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../../.."))
RUNNER_BIN = os.path.join(PROJECT_ROOT, "build", "veremi_runner")
RESULTS_DIR = os.path.join(PROJECT_ROOT, "results", "scorers")

ALL_0709_SCENARIOS = [
    "ConstPos_0709",
    "ConstPosOffset_0709",
    "ConstSpeed_0709",
    "ConstSpeedOffset_0709",
    "RandomPos_0709",
    "RandomPosOffset_0709",
    "RandomSpeed_0709",
    "RandomSpeedOffset_0709",
]

SCORER_CONFIGS = [
    {
        "id": "beta_reputation",
        "name": "BetaReputation",
        "display": "Beta-Reputation (Proposed)",
        "type": "Bayesian Conjugate",
        "flags": ["--trust", "BetaReputation"]
    },
    {
        "id": "ewma",
        "name": "EWMA",
        "display": "EWMA Filter",
        "type": "Exponential Smoothing",
        "flags": ["--trust", "EWMA"]
    },
    {
        "id": "majority_vote",
        "name": "MajorityVote",
        "display": "Sliding Window Majority Vote",
        "type": "Sliding Window Count",
        "flags": ["--trust", "MajorityVote"]
    }
]

def find_parquet_dirs(scenario):
    candidates = [
        os.path.join(WORKSPACE_ROOT, "veremi-extension", "parquet", scenario, "VeReMi_*"),
        os.path.join(WORKSPACE_ROOT, "experiments", "veremi-extension", "parquet", scenario, "VeReMi_*"),
        os.path.join(PROJECT_ROOT, "parquet", scenario, "VeReMi_*"),
        os.path.join(WORKSPACE_ROOT, "parquet", scenario, "VeReMi_*"),
    ]
    dirs = []
    for pattern in candidates:
        matched = glob.glob(pattern)
        dirs.extend([d for d in matched if os.path.isdir(d)])
    seen = set()
    return [d for d in dirs if not (d in seen or seen.add(d))]

def extract_attackers(sim_dir):
    attackers = set()
    for f in glob.glob(os.path.join(sim_dir, "traceJSON-*.parquet")):
        m = re.match(r"traceJSON-(\d+)-\d+-A([0-9]+)-", os.path.basename(f))
        if m and int(m.group(2)) > 0:
            attackers.add(int(m.group(1)))
    return attackers

def evaluate_simulation(sim_dir, scorer_flags, history_size=4, threshold=0.5):
    attackers = extract_attackers(sim_dir)
    cmd = [
        RUNNER_BIN,
        "--dir", sim_dir,
        "--historySize", str(history_size),
        "--min-scores"
    ] + scorer_flags

    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        print(f"Runner error on {sim_dir}: {proc.stderr}", file=sys.stderr)
        return None

    out_df = pd.read_csv(io.StringIO(proc.stdout))
    if out_df.empty or "sender" not in out_df.columns:
        return None

    out_df["is_attacker"] = out_df["sender"].isin(attackers)
    out_df["predicted_attacker"] = out_df["score"] < threshold

    tp = int(((out_df["predicted_attacker"] == True) & (out_df["is_attacker"] == True)).sum())
    fp = int(((out_df["predicted_attacker"] == True) & (out_df["is_attacker"] == False)).sum())
    tn = int(((out_df["predicted_attacker"] == False) & (out_df["is_attacker"] == False)).sum())
    fn = int(((out_df["predicted_attacker"] == False) & (out_df["is_attacker"] == True)).sum())

    return {"tp": tp, "fp": fp, "tn": tn, "fn": fn}

def run_scenario_scorer(scenario, scorer, history_size=4, threshold=0.5):
    sim_dirs = find_parquet_dirs(scenario)
    if not sim_dirs:
        print(f"Warning: No parquet directories found for scenario {scenario}", file=sys.stderr)
        return None

    total_tp = 0
    total_fp = 0
    total_tn = 0
    total_fn = 0

    for sdir in sim_dirs:
        res = evaluate_simulation(sdir, scorer["flags"], history_size, threshold)
        if res:
            total_tp += res["tp"]
            total_fp += res["fp"]
            total_tn += res["tn"]
            total_fn += res["fn"]

    precision = (total_tp / (total_tp + total_fp) * 100.0) if (total_tp + total_fp) > 0 else 0.0
    recall = (total_tp / (total_tp + total_fn) * 100.0) if (total_tp + total_fn) > 0 else 0.0
    fpr = (total_fp / (total_fp + total_tn) * 100.0) if (total_fp + total_tn) > 0 else 0.0
    total_nodes = total_tp + total_tn + total_fp + total_fn
    accuracy = ((total_tp + total_tn) / total_nodes * 100.0) if total_nodes > 0 else 0.0
    f1 = (2 * (recall * precision) / (recall + precision)) / 100.0 if (recall + precision) > 0 else 0.0

    return {
        "scenario": scenario,
        "scorer_id": scorer["id"],
        "scorer_name": scorer["name"],
        "scorer_display": scorer["display"],
        "scorer_type": scorer["type"],
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "fpr": fpr,
        "accuracy": accuracy,
        "tp": total_tp,
        "fp": total_fp,
        "tn": total_tn,
        "fn": total_fn
    }

from concurrent.futures import ThreadPoolExecutor, as_completed

def main():
    parser = argparse.ArgumentParser(description="Run Scorer Replacement Comparison Study on 0709 Subsets")
    parser.add_argument("--scenarios", nargs="+", default=ALL_0709_SCENARIOS,
                        help="Specific 0709 scenarios to run (default: all 8)")
    parser.add_argument("--history-size", type=int, default=4,
                        help="History window size (default: 4)")
    parser.add_argument("--threshold", type=float, default=0.5,
                        help="Trust threshold (default: 0.5)")
    parser.add_argument("--workers", type=int, default=6,
                        help="Parallel workers (default: 6)")
    parser.add_argument("--output-dir", default=RESULTS_DIR,
                        help="Output directory for results and markdown tables")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    print("==================================================================================")
    print("               SCORER REPLACEMENT STUDY (VeReMi 0709)")
    print(f" Scenarios:    {len(args.scenarios)} | History Window: {args.history_size} | Threshold: {args.threshold}")
    print(f" Scorers:      {[s['name'] for s in SCORER_CONFIGS]}")
    print(f" Parallel:     {args.workers} workers")
    print("==================================================================================\n")

    tasks = []
    for sc in args.scenarios:
        for scorer in SCORER_CONFIGS:
            tasks.append((sc, scorer))

    all_results = []
    start_total = time.time()

    print(f"Launching {len(tasks)} benchmark evaluations in parallel...")
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        future_to_task = {
            executor.submit(run_scenario_scorer, sc, scorer, args.history_size, args.threshold): (sc, scorer)
            for (sc, scorer) in tasks
        }
        for future in as_completed(future_to_task):
            sc, scorer = future_to_task[future]
            try:
                res = future.result()
                if res:
                    all_results.append(res)
                    print(f"  ✓ [{sc:<22}] {scorer['name']:<16} -> Rec: {res['recall']:5.2f}% | Prec: {res['precision']:5.2f}% | F1: {res['f1']:0.4f} | FPR: {res['fpr']:5.2f}%")
            except Exception as exc:
                print(f"  ✗ [{sc}] {scorer['name']} generated exception: {exc}", file=sys.stderr)

    if not all_results:
        print("No results generated. Check if parquet dataset directories exist.")
        return

    df = pd.DataFrame(all_results)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.output_dir, f"scorer_comparison_{timestamp}.csv")
    df.to_csv(csv_path, index=False)
    print(f"\nSaved raw results to: {csv_path}\n")

    # =========================================================================
    # Table 1: Per-Fault Detailed Breakdown Table
    # =========================================================================
    print("=" * 90)
    print(" TABLE 1: PER-FAULT SCORER COMPARISON RESULTS (VeReMi 07:00-09:00 Peak Traffic)")
    print("=" * 90)
    header_fmt = "{:<18} | {:<16} | {:<10} | {:<10} | {:<8} | {:<8}"
    row_fmt    = "{:<18} | {:<16} | {:<9.2f}% | {:<9.2f}% | {:<8.4f} | {:<7.2f}%"
    print(header_fmt.format("Fault Scenario", "Scorer", "Recall", "Precision", "F1", "FPR"))
    print("-" * 90)

    md_lines = [
        "# Scorer Replacement Comparison Results\n",
        f"Generated on {time.strftime('%Y-%m-%d %H:%M:%S')}\n",
        "## Table 1: Per-Fault Scorer Comparison Results (07:00–09:00 Peak Traffic)\n",
        "| Fault Scenario | Scorer | Recall (%) | Precision (%) | F1 Score | FPR (%) |",
        "| :--- | :--- | :---: | :---: | :---: | :---: |"
    ]

    for sc in args.scenarios:
        fault_name = sc.replace("_0709", "")
        sc_df = df[df["scenario"] == sc]
        for _, r in sc_df.iterrows():
            print(row_fmt.format(fault_name, r["scorer_name"],
                                 r["recall"], r["precision"], r["f1"], r["fpr"]))
            md_lines.append(f"| {fault_name} | {r['scorer_name']} | {r['recall']:.2f}% | {r['precision']:.2f}% | {r['f1']:.4f} | {r['fpr']:.2f}% |")
        print("-" * 90)

    # =========================================================================
    # Table 2: Side-by-Side Matrix per Fault (F1 Score & FPR across Scorers)
    # =========================================================================
    print("\n" + "=" * 110)
    print(" TABLE 2: SCORER COMPARISON SIDE-BY-SIDE MATRIX (07:00-09:00 Peak Traffic)")
    print("=" * 110)
    pivot_f1 = df.pivot(index="scenario", columns="scorer_name", values="f1")
    pivot_fpr = df.pivot(index="scenario", columns="scorer_name", values="fpr")
    cols_order = [s["name"] for s in SCORER_CONFIGS]
    pivot_f1 = pivot_f1.reindex(columns=cols_order)
    pivot_fpr = pivot_fpr.reindex(columns=cols_order)

    mat_fmt = "{:<18} | {:^16} | {:^10} | {:^14} || {:^16} | {:^10} | {:^14}"
    print(mat_fmt.format("Fault Scenario", "BetaReputation", "EWMA", "MajorityVote",
                         "BetaReputation", "EWMA", "MajorityVote"))
    print("-" * 110)

    md_lines.extend([
        "\n## Table 2: Scorer Side-by-Side Matrix (07:00–09:00 Peak Traffic)\n",
        "| Fault Scenario | BetaReputation F1 | EWMA F1 | MajorityVote F1 || BetaReputation FPR | EWMA FPR | MajorityVote FPR |",
        "| :--- | :---: | :---: | :---: | :--- | :---: | :---: | :---: |"
    ])

    for sc in ALL_0709_SCENARIOS:
        fault_name = sc.replace("_0709", "")
        if sc in pivot_f1.index:
            rf1 = pivot_f1.loc[sc]
            rfpr = pivot_fpr.loc[sc]
            print(mat_fmt.format(
                fault_name,
                f"{rf1.get('BetaReputation', 0):.4f}",
                f"{rf1.get('EWMA', 0):.4f}",
                f"{rf1.get('MajorityVote', 0):.4f}",
                f"{rfpr.get('BetaReputation', 0):.2f}%",
                f"{rfpr.get('EWMA', 0):.2f}%",
                f"{rfpr.get('MajorityVote', 0):.2f}%"
            ))
            md_lines.append(f"| {fault_name} | {rf1.get('BetaReputation', 0):.4f} | {rf1.get('EWMA', 0):.4f} | {rf1.get('MajorityVote', 0):.4f} || {rfpr.get('BetaReputation', 0):.2f}% | {rfpr.get('EWMA', 0):.2f}% | {rfpr.get('MajorityVote', 0):.2f}% |")

    # =========================================================================
    # Table 3: Macro-Average Summary across all 0709 Subsets
    # =========================================================================
    print("\n" + "=" * 92)
    print(" TABLE 3: MACRO-AVERAGE SUMMARY (07:00-09:00 Peak Traffic)")
    print("=" * 92)
    sum_header_fmt = "{:<22} | {:<22} | {:<10} | {:<10} | {:<8} | {:<8}"
    sum_row_fmt    = "{:<22} | {:<22} | {:<9.2f}% | {:<9.2f}% | {:<8.4f} | {:<7.2f}%"
    print(sum_header_fmt.format("Scorer Model", "Algorithm Type", "Recall", "Precision", "F1", "FPR"))
    print("-" * 92)

    md_lines.extend([
        "\n## Table 3: Macro-Average Summary (07:00–09:00 Peak Traffic)\n",
        "| Scorer Model | Algorithm Type | Recall (%) | Precision (%) | F1 Score | FPR (%) |",
        "| :--- | :--- | :---: | :---: | :---: | :---: |"
    ])

    for scorer in SCORER_CONFIGS:
        sc_df = df[df["scorer_id"] == scorer["id"]]
        if not sc_df.empty:
            avg_rec = sc_df["recall"].mean()
            avg_prec = sc_df["precision"].mean()
            avg_f1 = sc_df["f1"].mean()
            avg_fpr = sc_df["fpr"].mean()
            print(sum_row_fmt.format(scorer["display"], scorer["type"],
                                     avg_rec, avg_prec, avg_f1, avg_fpr))
            md_lines.append(f"| {scorer['display']} | {scorer['type']} | {avg_rec:.2f}% | {avg_prec:.2f}% | {avg_f1:.4f} | {avg_fpr:.2f}% |")

    print("=" * 92)
    total_sec = time.time() - start_total
    print(f"\nScorer comparison study completed in {total_sec:.2f} seconds.")

    md_path = os.path.join(args.output_dir, f"scorer_comparison_{timestamp}.md")
    latest_md = os.path.join(args.output_dir, "scorer_comparison_latest.md")
    with open(md_path, "w") as f:
        f.write("\n".join(md_lines))
    with open(latest_md, "w") as f:
        f.write("\n".join(md_lines))
    print(f"Saved Markdown report to: {md_path}")
    print(f"Symlinked/copied latest report to: {latest_md}")

if __name__ == "__main__":
    main()
