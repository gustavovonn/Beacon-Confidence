#!/usr/bin/env python3
import os
import glob
import sys
import argparse
import subprocess
import json
import time
import re
import pandas as pd
import numpy as np
from io import StringIO
from datetime import datetime

def find_parquet_dirs(workspace_dir, scenario):
    candidates = [
        os.path.join(workspace_dir, "veremi-extension", "parquet", scenario, "VeReMi_*"),
        os.path.join(workspace_dir, "experiments", "veremi-extension", "parquet", scenario, "VeReMi_*"),
        os.path.join(workspace_dir, "parquet", scenario, "VeReMi_*"),
    ]
    dirs = []
    for pattern in candidates:
        matched = glob.glob(pattern)
        dirs.extend([d for d in matched if os.path.isdir(d)])
    seen = set()
    return [d for d in dirs if not (d in seen or seen.add(d))]

def find_all_scenarios(workspace_dir):
    search_dirs = [
        os.path.join(workspace_dir, "veremi-extension", "parquet"),
        os.path.join(workspace_dir, "experiments", "veremi-extension", "parquet"),
        os.path.join(workspace_dir, "parquet"),
    ]
    scenarios = set()
    for sdir in search_dirs:
        if os.path.isdir(sdir):
            for item in os.listdir(sdir):
                if os.path.isdir(os.path.join(sdir, item)):
                    scenarios.add(item)
    return sorted(list(scenarios))

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

def process_scenario(dirs, runner_path, history_size=4, dr_model="HistoryAwareCTRA", trust_model="BetaReputation", threshold=0.5):
    all_metrics = []
    
    for d in dirs:
        print(f"Processing simulation directory {d} (history_size={history_size})...")
        
        attackers = extract_attackers_from_filenames(d)
        print(f"  Identified {len(attackers)} attackers in this simulation.")
        
        # Run C++ Native Parquet Processor with parametrized history size
        cmd = [
            runner_path,
            "--dir", d,
            "--dr", dr_model,
            "--trust", trust_model,
            "--historySize", str(history_size)
        ]
        
        start_time = time.perf_counter()
        proc = subprocess.run(cmd, text=True, capture_output=True)
        duration_sec = time.perf_counter() - start_time
        
        if proc.returncode != 0:
            print(f"Runner failed: {proc.stderr}")
            continue
            
        # Parse output
        out_str = proc.stdout
        out_df = pd.read_csv(StringIO(out_str))
        
        if out_df.empty:
            continue
            
        num_msgs = len(out_df)

        # Group by sender to get min trust score
        sender_scores = out_df.groupby('sender')['score'].min().reset_index()
        
        # Evaluate
        sender_scores['is_attacker'] = sender_scores['sender'].isin(attackers)
        sender_scores['predicted_attacker'] = sender_scores['score'] < threshold
        
        tp = int(((sender_scores['predicted_attacker'] == True) & (sender_scores['is_attacker'] == True)).sum())
        fp = int(((sender_scores['predicted_attacker'] == True) & (sender_scores['is_attacker'] == False)).sum())
        tn = int(((sender_scores['predicted_attacker'] == False) & (sender_scores['is_attacker'] == False)).sum())
        fn = int(((sender_scores['predicted_attacker'] == False) & (sender_scores['is_attacker'] == True)).sum())
        
        all_metrics.append({
            'tp': tp,
            'fp': fp,
            'tn': tn,
            'fn': fn,
            'messages': num_msgs,
            'duration_sec': duration_sec
        })

    if not all_metrics:
        return None
        
    total_tp = sum(m['tp'] for m in all_metrics)
    total_fp = sum(m['fp'] for m in all_metrics)
    total_tn = sum(m['tn'] for m in all_metrics)
    total_fn = sum(m['fn'] for m in all_metrics)
    total_messages = sum(m['messages'] for m in all_metrics)
    total_duration_sec = sum(m['duration_sec'] for m in all_metrics)
    
    precision = total_tp / (total_tp + total_fp) if (total_tp + total_fp) > 0 else 0
    recall = total_tp / (total_tp + total_fn) if (total_tp + total_fn) > 0 else 0
    fpr = total_fp / (total_fp + total_tn) if (total_fp + total_tn) > 0 else 0
    accuracy = (total_tp + total_tn) / (total_tp + total_tn + total_fp + total_fn) if (total_tp + total_tn + total_fp + total_fn) > 0 else 0
    
    f1_score = 2 * (recall * precision) / (recall + precision) if (recall + precision) > 0 else 0
    avg_latency_us = (total_duration_sec * 1e6 / total_messages) if total_messages > 0 else 0.0
    throughput = (total_messages / total_duration_sec) if total_duration_sec > 0 else 0.0
    
    return {
        'history_size': int(history_size),
        'precision': float(precision * 100),
        'recall': float(recall * 100),
        'fpr': float(fpr * 100),
        'accuracy': float(accuracy * 100),
        'f1_score': float(f1_score),
        'tp': int(total_tp),
        'fp': int(total_fp),
        'tn': int(total_tn),
        'fn': int(total_fn),
        'total_time_sec': float(total_duration_sec),
        'total_messages': int(total_messages),
        'avg_latency_us': float(avg_latency_us),
        'throughput_msg_s': float(throughput)
    }

def parse_history_sizes(args):
    if args.history_sweep:
        sizes = []
        for item in args.history_sweep:
            for part in str(item).replace(',', ' ').split():
                try:
                    val = int(part.strip())
                    if val > 0:
                        sizes.append(val)
                except ValueError:
                    pass
        seen = set()
        unique_sizes = [x for x in sizes if not (x in seen or seen.add(x))]
        if unique_sizes:
            return unique_sizes
    return [args.history_size]

def print_comparison_table(results, title="History Size Comparison"):
    print("\n" + "=" * 112)
    print(f"{title:^112}")
    print("=" * 112)
    header = f"{'History':<8} | {'Recall':<10} | {'Precision':<10} | {'F1 Score':<9} | {'Accuracy':<10} | {'FPR':<8} | {'Latency (us)':<13} | {'Throughput (msg/s)':<18} | {'Time (s)':<8}"
    print(header)
    print("-" * 112)
    for r in results:
        row = (f"{r['history_size']:<8} | "
               f"{r['recall']:<8.2f}% | "
               f"{r['precision']:<8.2f}% | "
               f"{r['f1_score']:<9.4f} | "
               f"{r['accuracy']:<8.2f}% | "
               f"{r['fpr']:<6.2f}% | "
               f"{r['avg_latency_us']:<13.2f} | "
               f"{r['throughput_msg_s']:<18.1f} | "
               f"{r['total_time_sec']:<8.2f}")
        print(row)
    print("=" * 112)

def generate_comparison_markdown(results, title="VeReMi History Size Sweep Comparison", subtitle=""):
    md = [f"# {title}\n"]
    if subtitle:
        md.append(f"{subtitle}\n")
    md.append("| History Size | Recall | Precision | F1 Score | Accuracy | FPR | Avg Latency (µs) | Throughput (msg/s) | Total Time (s) |")
    md.append("|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|")
    for r in results:
        md.append(f"| {r['history_size']} | {r['recall']:.2f}% | {r['precision']:.2f}% | "
                  f"{r['f1_score']:.4f} | {r['accuracy']:.2f}% | {r['fpr']:.2f}% | "
                  f"{r['avg_latency_us']:.2f} µs | {r['throughput_msg_s']:.1f} msg/s | {r['total_time_sec']:.2f} s |")
    md.append("")
    return "\n".join(md)

def main():
    parser = argparse.ArgumentParser(description="Run standalone VeReMi evaluation with history size parametrization and sweep")
    parser.add_argument("--scenario", required=False, help="e.g. ConstPos_0709 (Required unless --sweep is used)")
    parser.add_argument("--limit", type=int, default=None, help="Limit number of chunks to process")
    parser.add_argument("--sweep", action="store_true", help="Run across all scenarios")
    parser.add_argument("--history-size", type=int, default=4, help="History size for veremi_runner (default: 4)")
    parser.add_argument("--history-sweep", nargs="+", default=None, help="List of history sizes to sweep and compare (e.g. --history-sweep 5 10 20 or 5,10,20)")
    parser.add_argument("--dr", default="HistoryAwareCTRA", help="Dead reckoning model (default: HistoryAwareCTRA)")
    parser.add_argument("--trust", default="BetaReputation", help="Trust evaluator model (default: BetaReputation)")
    parser.add_argument("--threshold", type=float, default=0.5, help="Attacker trust threshold (default: 0.5)")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.abspath(os.path.join(script_dir, "../../.."))
    runner_path = os.path.join(os.path.dirname(script_dir), "build", "veremi_runner")
    results_dir = os.path.join(os.path.dirname(script_dir), "results")
    
    if not os.path.exists(runner_path):
        print(f"Error: {runner_path} not found. Please compile it first.")
        sys.exit(1)

    os.makedirs(results_dir, exist_ok=True)
    history_sizes = parse_history_sizes(args)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(results_dir, f"run_{timestamp}")
    os.makedirs(run_dir, exist_ok=True)

    # -------------------------------------------------------------------------
    # Multi-Scenario Sweep Mode (--sweep)
    # -------------------------------------------------------------------------
    if args.sweep:
        scenarios = find_all_scenarios(workspace_dir)
        if not scenarios:
            print("No scenario directories found in parquet/ folder.")
            sys.exit(1)
            
        print(f"Discovered {len(scenarios)} scenarios: {', '.join(scenarios)}")
        print(f"Testing history size(s): {history_sizes}")
        
        all_results = []
        md_file = os.path.join(run_dir, f"sweep_summary_{timestamp}.md")
        
        def save_sweep_markdown():
            with open(md_file, "w") as f:
                f.write(f"# VeReMi Sweep Results (Run: {timestamp})\n\n")
                f.write(f"Evaluated History Size(s): {', '.join(str(h) for h in history_sizes)}\n\n")
                f.write("| Scenario | History Size | Recall | Precision | F1 Score | Accuracy | FPR | Avg Latency (µs) | Throughput (msg/s) | Total Time (s) |\n")
                f.write("|----------|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|\n")
                for r in all_results:
                    f.write(f"| {r['scenario']} | {r['history_size']} | {r['recall']:.2f}% | {r['precision']:.2f}% | "
                            f"{r['f1_score']:.4f} | {r['accuracy']:.2f}% | {r['fpr']:.2f}% | "
                            f"{r['avg_latency_us']:.2f} µs | {r['throughput_msg_s']:.1f} msg/s | {r['total_time_sec']:.2f} s |\n")

        for h in history_sizes:
            print(f"\n==================================================")
            print(f"Evaluating History Size = {h}")
            print(f"==================================================")
            for sc in scenarios:
                dirs = find_parquet_dirs(workspace_dir, sc)
                if not dirs:
                    continue
                if args.limit:
                    dirs = dirs[:args.limit]
                
                print(f"\n--- Scenario: {sc} (History Size: {h}) ---")
                metrics = process_scenario(dirs, runner_path, history_size=h, dr_model=args.dr, trust_model=args.trust, threshold=args.threshold)
                if metrics:
                    metrics['scenario'] = sc
                    all_results.append(metrics)
                    save_sweep_markdown()
                    
        # Save JSON
        sweep_file = os.path.join(run_dir, f"sweep_results_{timestamp}.json")
        with open(sweep_file, 'w') as f:
            json.dump(all_results, f, indent=4)

        # Print per-scenario summary table
        print("\n" + "=" * 125)
        print(f"{'Scenario':<25} | {'History':<8} | {'Recall':<9} | {'Precision':<10} | {'F1':<8} | {'Latency (us)':<13} | {'Throughput (msg/s)':<18} | {'Time (s)':<8}")
        print("-" * 125)
        for r in all_results:
            print(f"{r['scenario']:<25} | {r['history_size']:<8} | {r['recall']:<7.2f}% | {r['precision']:<8.2f}% | {r['f1_score']:<8.4f} | {r['avg_latency_us']:<13.2f} | {r['throughput_msg_s']:<18.1f} | {r['total_time_sec']:<8.2f}")
        print("=" * 125)

        # If multiple history sizes were tested, compute and print macro-comparison across history sizes
        if len(history_sizes) > 1 and all_results:
            df = pd.DataFrame(all_results)
            macro_comparison = []
            for h in history_sizes:
                sub = df[df['history_size'] == h]
                if sub.empty:
                    continue
                macro_comparison.append({
                    'history_size': h,
                    'recall': sub['recall'].mean(),
                    'precision': sub['precision'].mean(),
                    'f1_score': sub['f1_score'].mean(),
                    'accuracy': sub['accuracy'].mean(),
                    'fpr': sub['fpr'].mean(),
                    'avg_latency_us': sub['avg_latency_us'].mean(),
                    'throughput_msg_s': sub['throughput_msg_s'].mean(),
                    'total_time_sec': sub['total_time_sec'].sum()
                })
            
            print_comparison_table(macro_comparison, title="Overall Macro-Average Comparison Across History Sizes")
            
            macro_md_file = os.path.join(run_dir, f"history_sweep_macro_summary_{timestamp}.md")
            with open(macro_md_file, "w") as f:
                f.write(generate_comparison_markdown(macro_comparison, title="VeReMi History Size Sweep (Macro-Averages)", subtitle=f"Averaged across {len(scenarios)} scenarios"))
            print(f"Saved Macro-Comparison Markdown to {macro_md_file}")

        print(f"\nSweep complete! Results saved in {run_dir}")
        return

    # -------------------------------------------------------------------------
    # Single Scenario Mode
    # -------------------------------------------------------------------------
    if not args.scenario:
        print("Error: --scenario is required unless --sweep is used.")
        parser.print_help()
        sys.exit(1)

    dirs = find_parquet_dirs(workspace_dir, args.scenario)
    if not dirs:
        print(f"No parquet simulation directories found for scenario '{args.scenario}'")
        sys.exit(1)
        
    if args.limit:
        dirs = dirs[:args.limit]
        
    print(f"Found {len(dirs)} simulation directories for scenario '{args.scenario}'.")
    print(f"Testing history size(s): {history_sizes}")

    scenario_results = []
    for h in history_sizes:
        print(f"\n--- Running evaluation with history_size={h} ---")
        metrics = process_scenario(dirs, runner_path, history_size=h, dr_model=args.dr, trust_model=args.trust, threshold=args.threshold)
        if metrics:
            metrics['scenario'] = args.scenario
            scenario_results.append(metrics)

    if not scenario_results:
        print("No metrics collected.")
        return

    # If multiple history sizes: print comparison and save comparison report
    if len(history_sizes) > 1:
        print_comparison_table(scenario_results, title=f"History Size Comparison for {args.scenario}")
        
        comp_md_file = os.path.join(run_dir, f"history_sweep_{args.scenario}_{timestamp}.md")
        with open(comp_md_file, "w") as f:
            f.write(generate_comparison_markdown(scenario_results, title=f"VeReMi History Size Sweep: {args.scenario}"))
            
        comp_json_file = os.path.join(run_dir, f"history_sweep_{args.scenario}_{timestamp}.json")
        with open(comp_json_file, "w") as f:
            json.dump(scenario_results, f, indent=4)
            
        print(f"\nSaved Comparison Markdown to: {comp_md_file}")
        print(f"Saved Comparison JSON to:     {comp_json_file}")
    else:
        # Single history size report
        metrics = scenario_results[0]
        out_file = os.path.join(run_dir, f"{args.scenario}_h{metrics['history_size']}_{timestamp}.json")
        with open(out_file, 'w') as f:
            json.dump(metrics, f, indent=4)
            
        print("\n" + "=" * 50)
        print(f"RESULTS FOR {args.scenario} (History Size: {metrics['history_size']})")
        print("=" * 50)
        print(f"Detection Rate (Recall) : {metrics['recall']:.2f}%")
        print(f"Precision               : {metrics['precision']:.2f}%")
        print(f"F1 Score                : {metrics['f1_score']:.4f}")
        print(f"False Positive Rate     : {metrics['fpr']:.2f}%")
        print(f"Accuracy                : {metrics['accuracy']:.2f}%")
        print(f"Avg Latency per Msg     : {metrics['avg_latency_us']:.2f} µs")
        print(f"Throughput              : {metrics['throughput_msg_s']:.1f} msg/s")
        print(f"Total Messages          : {metrics['total_messages']}")
        print(f"Total Execution Time    : {metrics['total_time_sec']:.2f} s")
        print(f"TP: {metrics['tp']} | FP: {metrics['fp']} | TN: {metrics['tn']} | FN: {metrics['fn']}")
        print(f"Saved results to: {out_file}")
        print("=" * 50)

if __name__ == "__main__":
    main()

