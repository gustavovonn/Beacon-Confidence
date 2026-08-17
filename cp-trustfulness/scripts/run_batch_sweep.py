#!/usr/bin/env python3
"""
run_batch_sweep.py
Batch Simulation Wrapper for V2X Trust Evaluation.

This script runs the defined baseline and ablation studies.
It consolidates all results into a single CSV file for easy plotting.
"""

import os
import sys
import subprocess
import yaml
import csv
import json
import time
from datetime import datetime
import argparse
from experiment import run_single_experiment, create_experiment_config

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_BASE = os.path.join(SCRIPT_DIR, "..", "results", "batch_sweeps")


def run_experiment(config, seed, batch_dir, global_args):
    # Add a custom name so experiment.py puts it in a nice folder
    exp_name = f"{config['id']}_seed{seed}"

    print(f"\n[{datetime.now().strftime('%H:%M:%S')}] RUNNING: {exp_name}")
    
    start_time = time.time()
    
    # Run in memory via imported module
    try:
        config = create_experiment_config(
            map_config=global_args.map,
            dr_alg=global_args.dr,
            trust_alg=global_args.trust,
            time_limit=global_args.time,
            name=exp_name,
            mode="headless",
            cam_freq=config.get("cam_freq"),
            fault_profile=config["fault_profile"],
            faulty_rate=config["faulty_rate"],
            faulty_cam_freq=config.get("faulty_cam_freq"),
            seed=seed
        )
        metrics = run_single_experiment(config)
    except Exception as e:
        print(f"\n[FATAL] Simulation {exp_name} crashed during execution: {e}")
        sys.exit(1)
        
    end_time = time.time()
    wall_time = end_time - start_time
    
    if not metrics:
        print(f"\n[FATAL] Simulation {exp_name} failed or returned no metrics.")
        print("        Aborting batch sweep to prevent generating garbage CSV data.")
        sys.exit(1)
        
    metrics["wall_time_sec"] = wall_time
    return metrics

def main():
    parser = argparse.ArgumentParser(description="Batch Runner for V2X Trust A1 Simulations")
    parser.add_argument("--seeds", type=int, default=1, help="Number of seeds to run per configuration (preliminarily 1, high quality would be 5.)")
    parser.add_argument("--config", type=str, default=os.path.join(SCRIPT_DIR, "..", "batch_config.yaml"), help="Path to batch YAML config file")
    
    # Global overrides
    parser.add_argument("--map", type=str, default="lust", help="Map configuration")
    parser.add_argument("--dr", type=str, default="HistoryAwareCTRA", help="Dead Reckoning algorithm")
    parser.add_argument("--trust", type=str, default="BetaReputation", help="Trust Evaluation algorithm")
    parser.add_argument("--time", type=str, default="45s", help="Simulation time limit")
    
    args = parser.parse_args()

    try:
        with open(args.config, 'r') as f:
            CONFIGS_RAW = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"[FATAL] Config file not found at {args.config}")
        sys.exit(1)

    if isinstance(CONFIGS_RAW, dict):
        ablations = CONFIGS_RAW.get('ablations', [])
    else:
        ablations = CONFIGS_RAW

    os.makedirs(RESULTS_BASE, exist_ok=True)
    batch_name = f"batch_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    batch_dir = os.path.join(RESULTS_BASE, batch_name)
    os.makedirs(batch_dir, exist_ok=True)
    
    csv_path = os.path.join(batch_dir, "batch_summary.csv")
    csv_headers = [
        "ConfigID", "Seed", "FaultyRate", "FaultProfile", "CamFreq", "FaultyCamFreq", 
        "F1_Score", "Precision", "Recall", "FPR", "Accuracy", "WallTime_sec"
    ]
    
    with open(csv_path, "w", newline='') as f:
        writer = csv.DictWriter(f, fieldnames=csv_headers)
        writer.writeheader()

    print(f"==================================================")
    print(f" STARTING BATCH SWEEP")
    print(f" Total Configs: {len(ablations)}")
    print(f" Seeds per config: {args.seeds} (Total runs: {len(ablations) * args.seeds})")
    print(f" Output directory: {batch_dir}")
    print(f"==================================================")

    for config in ablations:
        for seed in range(args.seeds):
            metrics = run_experiment(config, seed, batch_dir, args)
            
            # Append to CSV
            row = {
                "ConfigID": config["id"],
                "Seed": seed,
                "FaultyRate": config["faulty_rate"],
                "FaultProfile": config["fault_profile"],
                "CamFreq": config.get("cam_freq", "ETSI"),
                "FaultyCamFreq": config.get("faulty_cam_freq", "ETSI"),
                "F1_Score": metrics.get("f1_score", 0),
                "Precision": metrics.get("precision", 0),
                "Recall": metrics.get("recall", 0),
                "FPR": metrics.get("fpr", 0),
                "Accuracy": metrics.get("accuracy", 0),
                "WallTime_sec": round(metrics.get("wall_time_sec", 0), 2)
            }
            
            with open(csv_path, "a", newline='') as f:
                writer = csv.DictWriter(f, fieldnames=csv_headers)
                writer.writerow(row)
                
    print(f"\n==================================================")
    print(f" BATCH SWEEP COMPLETE")
    print(f" Consolidated CSV: {csv_path}")
    print(f"==================================================")
    
    # Generate polished visual report
    generate_report(csv_path, batch_dir)

def generate_report(csv_path, batch_dir):
    try:
        import pandas as pd
        import matplotlib.pyplot as plt
        import seaborn as sns
        
        df = pd.read_csv(csv_path)
        
        if df.empty:
            return
            
        sns.set_theme(style="whitegrid")
        
        # Plot 1: F1-Score vs Config ID
        plt.figure(figsize=(12, 6))
        ax = sns.barplot(x="ConfigID", y="F1_Score", hue="ConfigID", data=df, palette="viridis")
        plt.title("Faulty Vehicle Detection Efficacy (F1-Score) across Configurations", fontsize=14)
        plt.ylabel("F1-Score", fontsize=12)
        plt.xlabel("Ablation Configuration", fontsize=12)
        plt.xticks(rotation=45, ha="right")
        plt.ylim(0, 1.05)
        plt.tight_layout()
        plot1_path = os.path.join(batch_dir, "f1_score_plot.png")
        plt.savefig(plot1_path, dpi=300)
        plt.close()
        
        # Plot 2: FPR and Precision vs Config ID
        plt.figure(figsize=(12, 6))
        df_melt = df.melt(id_vars=["ConfigID"], value_vars=["Precision", "FPR"], var_name="Metric", value_name="Value")
        sns.barplot(x="ConfigID", y="Value", hue="Metric", data=df_melt, palette=["#2ecc71", "#e74c3c"])
        plt.title("Precision vs False Positive Rate (FPR)", fontsize=14)
        plt.ylabel("Score (%)", fontsize=12)
        plt.xlabel("Ablation Configuration", fontsize=12)
        plt.xticks(rotation=45, ha="right")
        plt.ylim(0, 105)
        plt.tight_layout()
        plot2_path = os.path.join(batch_dir, "precision_fpr_plot.png")
        plt.savefig(plot2_path, dpi=300)
        plt.close()

        # Generate Markdown Report
        md_path = os.path.join(batch_dir, "REPORT.md")
        with open(md_path, "w") as f:
            f.write("# V2X Trust Evaluation - Batch Report\n\n")
            f.write("## 1. Aggregated Metrics Table\n\n")
            
            # Manual Markdown Table Generation
            headers = df.columns.tolist()
            f.write("| " + " | ".join(headers) + " |\n")
            f.write("|" + "|".join(["---"] * len(headers)) + "|\n")
            for _, row in df.iterrows():
                f.write("| " + " | ".join(str(x) for x in row.values) + " |\n")
                
            f.write("\n\n## 2. Visualizations\n\n")
            f.write("### F1-Score (Overall Reliability)\n")
            f.write(f"![F1 Score](f1_score_plot.png)\n\n")
            f.write("### Precision vs False Positives\n")
            f.write(f"![Precision FPR](precision_fpr_plot.png)\n\n")
            
        print(f"\n==================================================")
        print(f" [REPORT] Polished Markdown Report generated at:")
        print(f" {md_path}")
        print(f"==================================================")
        
    except Exception as e:
        print(f"  [REPORT WARNING] Could not generate graphical report: {e}")
        pass

if __name__ == "__main__":
    main()
