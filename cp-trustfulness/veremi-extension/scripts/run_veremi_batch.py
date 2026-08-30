#!/usr/bin/env python3
import os
import glob
import sys
import argparse
import subprocess
import json
import pandas as pd
import numpy as np
from io import StringIO
from datetime import datetime

def find_parquet_dirs(base_dir, scenario):
    pattern = os.path.join(base_dir, "parquet", scenario, "VeReMi_*")
    dirs = glob.glob(pattern)
    return [d for d in dirs if os.path.isdir(d)]

import re

def extract_attackers_from_filenames(d):
    """
    Extract attacker node IDs directly from the VeReMi traceJSON filenames.
    Format: traceJSON-[nodeID]-[pseudoID]-[AttackerType]-...
    A0 = Honest, A1+ = Attacker.
    """
    attackers = set()
    files = glob.glob(os.path.join(d, "traceJSON-*.parquet"))
    for f in files:
        basename = os.path.basename(f)
        match = re.match(r"traceJSON-(\d+)-\d+-A([0-9]+)-", basename)
        if match:
            node_id = int(match.group(1))
            attacker_type = int(match.group(2))
            if attacker_type > 0:
                attackers.add(node_id)
    return attackers

def process_scenario(dirs, runner_path):
    all_metrics = []
    
    for d in dirs:
        print(f"Processing simulation directory {d}...")
        
        attackers = extract_attackers_from_filenames(d)
        print(f"  Identified {len(attackers)} attackers in this simulation.")
        
        # Run C++ Native Parquet Processor
        cmd = [runner_path, "--dir", d, "--dr", "HistoryAwareCTRA", "--trust", "BetaReputation"]
        proc = subprocess.run(cmd, text=True, capture_output=True)
        
        if proc.returncode != 0:
            print(f"Runner failed: {proc.stderr}")
            continue
            
        # Parse output
        out_str = proc.stdout
        out_df = pd.read_csv(StringIO(out_str))
        
        if out_df.empty:
            continue
            
        # Group by sender to get min trust score
        sender_scores = out_df.groupby('sender')['score'].min().reset_index()
        
        # Evaluate
        threshold = 0.5
        sender_scores['is_attacker'] = sender_scores['sender'].isin(attackers)
        sender_scores['predicted_attacker'] = sender_scores['score'] < threshold
        
        tp = ((sender_scores['predicted_attacker'] == True) & (sender_scores['is_attacker'] == True)).sum()
        fp = ((sender_scores['predicted_attacker'] == True) & (sender_scores['is_attacker'] == False)).sum()
        tn = ((sender_scores['predicted_attacker'] == False) & (sender_scores['is_attacker'] == False)).sum()
        fn = ((sender_scores['predicted_attacker'] == False) & (sender_scores['is_attacker'] == True)).sum()
        
        all_metrics.append({'tp': tp, 'fp': fp, 'tn': tn, 'fn': fn})

    if not all_metrics:
        return None
        
    total_tp = sum(m['tp'] for m in all_metrics)
    total_fp = sum(m['fp'] for m in all_metrics)
    total_tn = sum(m['tn'] for m in all_metrics)
    total_fn = sum(m['fn'] for m in all_metrics)
    
    precision = total_tp / (total_tp + total_fp) if (total_tp + total_fp) > 0 else 0
    recall = total_tp / (total_tp + total_fn) if (total_tp + total_fn) > 0 else 0
    fpr = total_fp / (total_fp + total_tn) if (total_fp + total_tn) > 0 else 0
    accuracy = (total_tp + total_tn) / (total_tp + total_tn + total_fp + total_fn) if (total_tp + total_tn + total_fp + total_fn) > 0 else 0
    
    f1_score = 2 * (recall * precision) / (recall + precision) if (recall + precision) > 0 else 0
    
    return {
        'precision': float(precision * 100),
        'recall': float(recall * 100),
        'fpr': float(fpr * 100),
        'accuracy': float(accuracy * 100),
        'f1_score': float(f1_score),
        'tp': int(total_tp),
        'fp': int(total_fp),
        'tn': int(total_tn),
        'fn': int(total_fn)
    }

def main():
    parser = argparse.ArgumentParser(description="Run standalone VeReMi evaluation")
    parser.add_argument("--scenario", required=False, help="e.g. ConstPos_0709 (Required unless --sweep is used)")
    parser.add_argument("--limit", type=int, default=None, help="Limit number of chunks to process")
    parser.add_argument("--sweep", action="store_true", help="Run all scenarios")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.abspath(os.path.join(script_dir, "../../.."))
    runner_path = os.path.join(workspace_dir, "cp-trustfulness", "veremi-extension", "build", "veremi_runner")
    results_dir = os.path.join(os.path.dirname(script_dir), "results")
    
    if not os.path.exists(runner_path):
        print(f"Error: {runner_path} not found. Please compile it first.")
        sys.exit(1)

    os.makedirs(results_dir, exist_ok=True)

    if args.sweep:
        scenarios = [d for d in os.listdir(os.path.join(workspace_dir, "veremi-extension", "parquet")) if os.path.isdir(os.path.join(workspace_dir, "veremi-extension", "parquet", d))]
        scenarios.sort()
        
        all_results = []
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = os.path.join(results_dir, f"run_{timestamp}")
        os.makedirs(run_dir, exist_ok=True)
        
        md_file = os.path.join(run_dir, f"sweep_summary_{timestamp}.md")
        
        def update_markdown(results):
            with open(md_file, "w") as f:
                f.write(f"# VeReMi Sweep Results (Run: {timestamp})\n\n")
                f.write("| Scenario | Recall | Precision | F1 Score | Accuracy | FPR |\n")
                f.write("|----------|--------|-----------|----------|----------|-----|\n")
                for r in results:
                    f.write(f"| {r['scenario']} | {r['recall']:.2f}% | {r['precision']:.2f}% | "
                            f"{r['f1_score']:.4f} | {r['accuracy']:.2f}% | {r['fpr']:.2f}% |\n")

        for sc in scenarios:
            dirs = find_parquet_dirs(os.path.join(workspace_dir, "veremi-extension"), sc)
            if not dirs:
                continue
            if args.limit:
                dirs = dirs[:args.limit]
            
            print(f"\n--- Running Sweep: Scenario={sc} ---")
            metrics = process_scenario(dirs, runner_path)
            if metrics:
                metrics['scenario'] = sc
                all_results.append(metrics)
                update_markdown(all_results)
                        
        # Save sweep summary JSON
        sweep_file = os.path.join(run_dir, f"sweep_results_{timestamp}.json")
        with open(sweep_file, 'w') as f:
            json.dump(all_results, f, indent=4)
        print(f"\nSweep complete! Saved JSON to {sweep_file} and Markdown to {md_file}")
        
        # Print summary table
        print("\n" + "="*80)
        print(f"{'Scenario':<25} | {'Recall':<10} | {'Precision':<10} | {'F1':<8}")
        print("-" * 80)
        for r in all_results:
            print(f"{r['scenario']:<25} | {r['recall']:<8.2f}% | {r['precision']:<8.2f}% | {r['f1_score']:<8.4f}")
        print("="*80)
        return

    if not args.sweep and not args.scenario:
        print("Error: --scenario is required unless --sweep is used.")
        parser.print_help()
        sys.exit(1)

    dirs = find_parquet_dirs(os.path.join(workspace_dir, "veremi-extension"), args.scenario)
    if not dirs:
        print(f"No parquet simulation directories found for scenario {args.scenario}")
        sys.exit(1)
        
    if args.limit:
        dirs = dirs[:args.limit]
        
    print(f"Found {len(dirs)} simulation directories for scenario {args.scenario}.")
    
    metrics = process_scenario(dirs, runner_path)
    
    if metrics:
        metrics['scenario'] = args.scenario
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_file = os.path.join(results_dir, f"{args.scenario}_{timestamp}.json")
        with open(out_file, 'w') as f:
            json.dump(metrics, f, indent=4)
            
        print("\n" + "="*50)
        print(f"RESULTS FOR {args.scenario}")
        print("="*50)
        print(f"Detection Rate (Recall) : {metrics['recall']:.2f}%")
        print(f"Precision               : {metrics['precision']:.2f}%")
        print(f"F1 Score                : {metrics['f1_score']:.4f}")
        print(f"False Positive Rate     : {metrics['fpr']:.2f}%")
        print(f"Accuracy                : {metrics['accuracy']:.2f}%")
        print(f"TP: {metrics['tp']} | FP: {metrics['fp']} | TN: {metrics['tn']} | FN: {metrics['fn']}")
        print(f"Saved results to: {out_file}")
        print("="*50)
    else:
        print("No metrics collected.")

if __name__ == "__main__":
    main()
