#!/usr/bin/env python3
"""
plot_scorer_roc.py
Generates a fused ROC Comparison Plot comparing BetaReputation, EWMA, and MajorityVote
across all VeReMi 0709 scenarios.
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
import shutil
import subprocess
from concurrent.futures import ThreadPoolExecutor
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import roc_curve, auc

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
WORKSPACE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../.."))
RUNNER_BIN = os.path.join(PROJECT_ROOT, "build", "veremi_runner")
RESULTS_DIR = os.path.join(PROJECT_ROOT, "results", "scorers")
ARTIFACTS_DIR = "/home/gust/.gemini/antigravity-ide/brain/32dd96fc-654f-4703-a7b9-044fdad23e23"

ALL_0709_SCENARIOS = [
    "ConstPos_0709",
    "ConstSpeed_0709",
    "ConstSpeedOffset_0709",
    "RandomPos_0709",
    "RandomPosOffset_0709",
    "RandomSpeed_0709",
    "RandomSpeedOffset_0709",
]

SCORERS = [
    {
        "id": "BetaReputation",
        "name": "Beta-Reputation (Proposed)",
        "color": "#1f77b4",  # Bold Blue
        "style": "-",
        "width": 2.5
    },
    {
        "id": "MajorityVote",
        "name": "Sliding Window Majority Vote",
        "color": "#2ca02c",  # Green
        "style": "--",
        "width": 2.0
    },
    {
        "id": "EWMA",
        "name": "EWMA Filter",
        "color": "#ff7f0e",  # Orange
        "style": ":",
        "width": 2.0
    }
]

def find_parquet_dirs(scenario):
    candidates = [
        os.path.join(WORKSPACE_ROOT, "veremi-extension", "parquet", scenario, "VeReMi_*"),
        os.path.join(WORKSPACE_ROOT, "detection-eval", "parquet", scenario, "VeReMi_*"),
        os.path.join(PROJECT_ROOT, "parquet", scenario, "VeReMi_*"),
    ]
    dirs = []
    for pattern in candidates:
        dirs.extend([d for d in glob.glob(pattern) if os.path.isdir(d)])
    seen = set()
    return [d for d in dirs if not (d in seen or seen.add(d))]

def extract_attackers(sim_dir):
    attackers = set()
    for f in glob.glob(os.path.join(sim_dir, "traceJSON-*.parquet")):
        m = re.match(r"traceJSON-(\d+)-\d+-A([0-9]+)-", os.path.basename(f))
        if m and int(m.group(2)) > 0:
            attackers.add(int(m.group(1)))
    return attackers

def run_simulation(sim_dir, scorer_id):
    attackers = extract_attackers(sim_dir)
    cmd = [
        RUNNER_BIN,
        "--dir", sim_dir,
        "--historySize", "4",
        "--min-scores",
        "--trust", scorer_id
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        return None
    df = pd.read_csv(io.StringIO(proc.stdout))
    if df.empty or "sender" not in df.columns:
        return None
    df["is_attacker"] = df["sender"].isin(attackers).astype(int)
    # Anomaly score is 1.0 - trust score (lower trust -> higher anomaly)
    df["anomaly_score"] = 1.0 - df["score"]
    return df[["is_attacker", "anomaly_score"]]

def main():
    print("Collecting simulation predictions across all 07:00-09:00 subsets for ROC analysis...")
    os.makedirs(RESULTS_DIR, exist_ok=True)
    
    sim_dirs = []
    for sc in ALL_0709_SCENARIOS:
        sim_dirs.extend(find_parquet_dirs(sc))
    
    print(f"Discovered {len(sim_dirs)} trace directories across {len(ALL_0709_SCENARIOS)} scenarios.")
    
    scorer_data = {s["id"]: [] for s in SCORERS}
    
    for s in SCORERS:
        sid = s["id"]
        print(f"--> Evaluating {s['name']}...")
        with ThreadPoolExecutor(max_workers=6) as executor:
            futures = [executor.submit(run_simulation, sdir, sid) for sdir in sim_dirs]
            for f in futures:
                res = f.result()
                if res is not None:
                    scorer_data[sid].append(res)
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.2), dpi=300)
    
    # -------------------------------------------------------------
    # Panel 1: Full ROC Spectrum [0.0, 1.0]
    # -------------------------------------------------------------
    ax1.plot([0, 1], [0, 1], color="grey", linestyle="--", linewidth=1.2, label="Random Guess (AUC = 0.500)")
    
    # -------------------------------------------------------------
    # Panel 2: Magnified Operational Window [0.0, 0.05] (FPR <= 5%)
    # -------------------------------------------------------------
    
    for s in SCORERS:
        sid = s["id"]
        dfs = scorer_data[sid]
        if not dfs:
            continue
        all_df = pd.concat(dfs, ignore_index=True)
        y_true = all_df["is_attacker"].values
        y_scores = all_df["anomaly_score"].values
        
        fpr, tpr, _ = roc_curve(y_true, y_scores)
        roc_auc = auc(fpr, tpr)
        
        label_full = f"{s['name']} (AUC = {roc_auc:.4f})"
        
        # Plot full spectrum
        ax1.plot(fpr, tpr, color=s["color"], linestyle=s["style"], linewidth=s["width"], label=label_full)
        
        # Plot operational zoom
        ax2.plot(fpr * 100.0, tpr * 100.0, color=s["color"], linestyle=s["style"], linewidth=s["width"] + 0.5, label=f"{s['name']}")
        
        print(f"  ✓ {s['name']:<30} -> Macro AUC: {roc_auc:.4f} (Samples: {len(y_true)})")
    
    # Format Left Panel (Full Spectrum)
    ax1.set_xlim([-0.02, 1.02])
    ax1.set_ylim([0.0, 1.02])
    ax1.set_xlabel("False Positive Rate (FPR)", fontsize=11, fontweight="bold")
    ax1.set_ylabel("True Positive Rate (Recall / TPR)", fontsize=11, fontweight="bold")
    ax1.set_title("(a) Full ROC Spectrum", fontsize=12, fontweight="bold", pad=10)
    ax1.legend(loc="lower right", fontsize=9, frameon=True, framealpha=0.92, edgecolor="lightgray")
    ax1.grid(True, linestyle=":", alpha=0.6)
    
    # Highlight the zoom area on Panel 1
    rect = plt.Rectangle((0, 0.4), 0.05, 0.62, linewidth=1.5, edgecolor="red", facecolor="none", linestyle="--")
    ax1.add_patch(rect)
    ax1.text(0.06, 0.42, "Operational Region (Zoomed →)", color="red", fontsize=9, fontweight="bold")
    
    # Format Right Panel (Operational Region)
    ax2.set_xlim([0.0, 5.0])  # 0% to 5% FPR
    ax2.set_ylim([40.0, 101.5])  # 40% to 100% Recall
    ax2.set_xlabel("False Positive Rate (FPR %)", fontsize=11, fontweight="bold")
    ax2.set_ylabel("True Positive Rate (Recall %)", fontsize=11, fontweight="bold")
    ax2.set_title("(b) Operational Region (FPR ≤ 5%)", fontsize=12, fontweight="bold", pad=10)
    ax2.legend(loc="lower right", fontsize=9, frameon=True, framealpha=0.92, edgecolor="lightgray")
    ax2.grid(True, linestyle=":", alpha=0.6)
    
    fig.suptitle("Reputation Scorer ROC Analysis (VeReMi 07:00–09:00 Peak Traffic)", fontsize=13, fontweight="bold", y=0.98)
    plt.tight_layout()
    
    png_out = os.path.join(RESULTS_DIR, "scorer_roc_comparison.png")
    pdf_out = os.path.join(RESULTS_DIR, "scorer_roc_comparison.pdf")
    plt.savefig(png_out, dpi=300)
    plt.savefig(pdf_out)
    print(f"\nSaved publication-ready PNG plot to: {png_out}")
    print(f"Saved publication-ready PDF vector to: {pdf_out}")
    
    # Copy to artifacts for IDE viewing
    if os.path.exists(ARTIFACTS_DIR):
        artifact_png = os.path.join(ARTIFACTS_DIR, "scorer_roc_comparison.png")
        shutil.copyfile(png_out, artifact_png)
        print(f"Copied plot to artifacts directory: {artifact_png}")

if __name__ == "__main__":
    main()
