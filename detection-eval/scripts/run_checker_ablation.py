#!/usr/bin/env python3
"""
run_checker_ablation.py
Comprehensive Ablation Study Wrapper for Checkers Pipeline across VeReMi 0709 subsets.

Evaluates where each component succeeds, fails, or is absent:
1. Full Model (P0: Yes, DYN: Yes, DR: Yes, OLS: Yes)
2. w/o Consistency Check (CC: No, DYN: Yes, DR: Yes, OLS: Yes) [--no-consistency]
3. w/o Dynamic Thresh (P0: Yes, DYN: No, DR: Yes, OLS: Yes) [--no-dyn-thresh]
4. w/o Dead-Reckoning (P0: Yes, DYN: Yes, DR: No, OLS: No) [--no-dr]
5. w/o OLS Smoothing (Bare CTRA) (P0: Yes, DYN: Yes, DR: Yes, OLS: No) [--no-ols]
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
from concurrent.futures import ThreadPoolExecutor, as_completed
import pandas as pd
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
WORKSPACE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../.."))
RUNNER_BIN = os.path.join(PROJECT_ROOT, "build", "veremi_runner")
RESULTS_DIR = os.path.join(PROJECT_ROOT, "results", "ablations")

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

CHECKER_CONFIGS = [
    {
        "id": "full",
        "name": "Full Model",
        "short": "Full",
        "p0": "Yes",
        "dyn": "Yes",
        "dr": "Yes",
        "ols": "Yes",
        "flags": []
    },
    {
        "id": "no_p0",
        "name": "w/o Consistency Check",
        "short": "w/o CC",
        "p0": "No",
        "dyn": "Yes",
        "dr": "Yes",
        "ols": "Yes",
        "flags": ["--no-consistency"]
    },
    {
        "id": "no_dyn",
        "name": "w/o Dyn-Thresh",
        "short": "w/o Dyn",
        "p0": "Yes",
        "dyn": "No",
        "dr": "Yes",
        "ols": "Yes",
        "flags": ["--no-dyn-thresh"]
    },
    {
        "id": "no_ols",
        "name": "w/o OLS (Bare CTRA)",
        "short": "w/o OLS",
        "p0": "Yes",
        "dyn": "Yes",
        "dr": "Yes",
        "ols": "No",
        "flags": ["--no-ols"]
    },
    {
        "id": "no_dr",
        "name": "w/o Dead-Reckoning",
        "short": "w/o DR",
        "p0": "Yes",
        "dyn": "Yes",
        "dr": "No",
        "ols": "No",
        "flags": ["--no-dr"]
    }
]

def find_parquet_dirs(scenario):
    candidates = [
        os.path.join(WORKSPACE_ROOT, "veremi-extension", "parquet", scenario, "VeReMi_*"),
        os.path.join(WORKSPACE_ROOT, "detection-eval", "parquet", scenario, "VeReMi_*"),
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

def evaluate_simulation(sim_dir, flags, history_size=4, threshold=0.5):
    attackers = extract_attackers(sim_dir)
    cmd = [
        RUNNER_BIN,
        "--dir", sim_dir,
        "--historySize", str(history_size),
        "--min-scores"
    ] + flags

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

def run_scenario_config(scenario, cfg, history_size=4, threshold=0.5):
    sim_dirs = find_parquet_dirs(scenario)
    if not sim_dirs:
        print(f"Warning: No parquet directories found for scenario {scenario}", file=sys.stderr)
        return None

    total_tp = 0
    total_fp = 0
    total_tn = 0
    total_fn = 0

    for sdir in sim_dirs:
        res = evaluate_simulation(sdir, cfg["flags"], history_size, threshold)
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
        "config_id": cfg["id"],
        "config_name": cfg["name"],
        "config_short": cfg["short"],
        "p0": cfg["p0"],
        "dyn": cfg["dyn"],
        "dr": cfg["dr"],
        "ols": cfg["ols"],
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

def main():
    parser = argparse.ArgumentParser(description="Run Checker Pipeline Ablation Study on 0709 Subsets")
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
    print("               CHECKER PIPELINE ABLATION STUDY (VeReMi 0709)")
    print(f" Scenarios:      {len(args.scenarios)} | History Window: {args.history_size} | Threshold: {args.threshold}")
    print(f" Configurations: {[c['name'] for c in CHECKER_CONFIGS]}")
    print(f" Parallel Jobs:  {args.workers} workers")
    print("==================================================================================\n")

    tasks = []
    for sc in args.scenarios:
        for cfg in CHECKER_CONFIGS:
            tasks.append((sc, cfg))

    all_results = []
    start_total = time.time()

    print(f"Launching {len(tasks)} benchmark evaluations across 0709 subsets in parallel...")
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        future_to_task = {
            executor.submit(run_scenario_config, sc, cfg, args.history_size, args.threshold): (sc, cfg)
            for (sc, cfg) in tasks
        }
        for future in as_completed(future_to_task):
            sc, cfg = future_to_task[future]
            try:
                res = future.result()
                if res:
                    all_results.append(res)
                    print(f"  ✓ [{sc:<22}] {cfg['name']:<20} -> Rec: {res['recall']:5.2f}% | Prec: {res['precision']:5.2f}% | F1: {res['f1']:0.4f} | FPR: {res['fpr']:5.2f}%")
            except Exception as exc:
                print(f"  ✗ [{sc}] {cfg['name']} generated exception: {exc}", file=sys.stderr)

    if not all_results:
        print("No results generated. Check if parquet dataset directories exist.")
        return

    df = pd.DataFrame(all_results)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.output_dir, f"checker_ablation_{timestamp}.csv")
    df.to_csv(csv_path, index=False)

    md_lines = [
        "# Checker Pipeline Ablation Results\n",
        f"Generated on {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
    ]

    # =========================================================================
    # Table 1: Per-Fault Detailed Breakdown Table (All Metrics for all Subsets)
    # =========================================================================
    print("\n" + "=" * 96)
    print(" TABLE 1: PER-FAULT CHECKER ABLATION RESULTS (VeReMi 07:00-09:00 Peak Traffic)")
    print("=" * 96)
    header_fmt = "{:<18} | {:<4} | {:<4} | {:<4} | {:<4} | {:<10} | {:<10} | {:<8} | {:<8}"
    row_fmt    = "{:<18} | {:<4} | {:<4} | {:<4} | {:<4} | {:<9.2f}% | {:<9.2f}% | {:<8.4f} | {:<7.2f}%"
    print(header_fmt.format("Fault Scenario", "P0", "DYN", "DR", "OLS", "Recall", "Precision", "F1", "FPR"))
    print("-" * 96)

    md_lines.extend([
        "## Table 1: Per-Fault Checker Ablation Results (07:00–09:00 Peak Traffic)\n",
        "| Fault Scenario | P0 | DYN | DR | OLS | Recall (%) | Precision (%) | F1 Score | FPR (%) |",
        "| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |"
    ])

    for sc in ALL_0709_SCENARIOS:
        fault_name = sc.replace("_0709", "")
        sc_df = df[df["scenario"] == sc]
        if not sc_df.empty:
            for cfg in CHECKER_CONFIGS:
                row_match = sc_df[sc_df["config_id"] == cfg["id"]]
                if not row_match.empty:
                    r = row_match.iloc[0]
                    print(row_fmt.format(
                        fault_name, cfg["p0"], cfg["dyn"], cfg["dr"], cfg["ols"],
                        r["recall"], r["precision"], r["f1"], r["fpr"]
                    ))
                    md_lines.append(f"| {fault_name} | {cfg['p0']} | {cfg['dyn']} | {cfg['dr']} | {cfg['ols']} | {r['recall']:.2f}% | {r['precision']:.2f}% | {r['f1']:.4f} | {r['fpr']:.2f}% |")
            print("-" * 96)

    # =========================================================================
    # Table 2: Side-by-Side Comparison Matrix per Fault (F1 Score & FPR)
    # =========================================================================
    pivot_f1 = df.pivot(index="scenario", columns="config_short", values="f1")
    pivot_fpr = df.pivot(index="scenario", columns="config_short", values="fpr")
    cols_order = [c["short"] for c in CHECKER_CONFIGS]
    pivot_f1 = pivot_f1.reindex(columns=cols_order)
    pivot_fpr = pivot_fpr.reindex(columns=cols_order)

    f1_headers = [f"{c} F1" for c in cols_order]
    fpr_headers = [f"{c} FPR" for c in cols_order]
    
    mat_col_w = 9
    header_str = "{:<18} | " + " | ".join(["{:^" + str(mat_col_w) + "}"] * len(cols_order)) + " || " + " | ".join(["{:^" + str(mat_col_w) + "}"] * len(cols_order))
    row_str    = "{:<18} | " + " | ".join(["{:^" + str(mat_col_w) + "}"] * len(cols_order)) + " || " + " | ".join(["{:^" + str(mat_col_w) + "}"] * len(cols_order))
    sep_len = 18 + 3 + (mat_col_w + 3) * len(cols_order) + 1 + (mat_col_w + 3) * len(cols_order)

    print("\n" + "=" * sep_len)
    print(" TABLE 2: CHECKER ABLATION SIDE-BY-SIDE MATRIX (07:00-09:00 Peak Traffic)")
    print("=" * sep_len)
    print(header_str.format("Fault Scenario", *f1_headers, *fpr_headers))
    print("-" * sep_len)

    md_lines.extend([
        "\n## Table 2: Checker Ablation Side-by-Side Matrix (07:00–09:00 Peak Traffic)\n",
        "| Fault Scenario | " + " | ".join(f1_headers) + " || " + " | ".join(fpr_headers) + " |",
        "| :--- | " + " | ".join([":---:"] * len(cols_order)) + " || " + " | ".join([":---:"] * len(cols_order)) + " |"
    ])

    for sc in ALL_0709_SCENARIOS:
        fault_name = sc.replace("_0709", "")
        if sc in pivot_f1.index:
            row_f1 = pivot_f1.loc[sc]
            row_fpr = pivot_fpr.loc[sc]
            f1_vals = [f"{row_f1.get(c, 0):.4f}" for c in cols_order]
            fpr_vals = [f"{row_fpr.get(c, 0):.2f}%" for c in cols_order]
            print(row_str.format(fault_name, *f1_vals, *fpr_vals))
            md_lines.append(f"| {fault_name} | " + " | ".join(f1_vals) + " || " + " | ".join(fpr_vals) + " |")

    # =========================================================================
    # Table 3: Macro-Average across all 0709 Subsets
    # =========================================================================
    print("\n" + "=" * 96)
    print(" TABLE 3: MACRO-AVERAGE SUMMARY (07:00-09:00 Peak Traffic)")
    print("=" * 96)
    sum_header_fmt = "{:<20} | {:<4} | {:<4} | {:<4} | {:<4} | {:<10} | {:<10} | {:<8} | {:<8}"
    sum_row_fmt    = "{:<20} | {:<4} | {:<4} | {:<4} | {:<4} | {:<9.2f}% | {:<9.2f}% | {:<8.4f} | {:<7.2f}%"
    print(sum_header_fmt.format("Configuration", "P0", "DYN", "DR", "OLS", "Recall", "Precision", "F1", "FPR"))
    print("-" * 96)

    md_lines.extend([
        "\n## Table 3: Macro-Average Summary (07:00–09:00 Peak Traffic)\n",
        "| Configuration | P0 | DYN | DR | OLS | Recall (%) | Precision (%) | F1 Score | FPR (%) |",
        "| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |"
    ])

    for cfg in CHECKER_CONFIGS:
        cfg_df = df[df["config_id"] == cfg["id"]]
        if not cfg_df.empty:
            avg_rec = cfg_df["recall"].mean()
            avg_prec = cfg_df["precision"].mean()
            avg_f1 = cfg_df["f1"].mean()
            avg_fpr = cfg_df["fpr"].mean()
            print(sum_row_fmt.format(cfg["name"], cfg["p0"], cfg["dyn"], cfg["dr"], cfg["ols"],
                                     avg_rec, avg_prec, avg_f1, avg_fpr))
            md_lines.append(f"| {cfg['name']} | {cfg['p0']} | {cfg['dyn']} | {cfg['dr']} | {cfg['ols']} | {avg_rec:.2f}% | {avg_prec:.2f}% | {avg_f1:.4f} | {avg_fpr:.2f}% |")

    print("=" * 96)
    total_sec = time.time() - start_total
    print(f"\nAblation study completed in {total_sec:.2f} seconds.")

    md_path = os.path.join(args.output_dir, f"checker_ablation_{timestamp}.md")
    latest_md = os.path.join(args.output_dir, "checker_ablation_latest.md")
    with open(md_path, "w") as f:
        f.write("\n".join(md_lines))
    with open(latest_md, "w") as f:
        f.write("\n".join(md_lines))
    print(f"Saved Markdown report to: {md_path}")
    print(f"Saved CSV results to: {csv_path}")

if __name__ == "__main__":
    main()
