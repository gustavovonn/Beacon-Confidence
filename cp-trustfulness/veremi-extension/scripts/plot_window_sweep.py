#!/usr/bin/env python3
"""
plot_window_sweep.py

Evaluates and plots detection performance (Recall, Precision, F1, FPR) and
computational latency across different tracking history window sizes (starting from 4).
"""

import os
import sys
import argparse
import json
from datetime import datetime

# Optional plotting setup (headless backend friendly)
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# Ensure script dir is in sys.path to import from run_veremi_batch
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import run_veremi_batch

DEFAULT_WINDOW_SIZES = [4, 6, 8, 10, 12, 15, 20, 25, 30]

def parse_window_sizes(args):
    if args.window_sizes:
        sizes = []
        for item in args.window_sizes:
            for part in str(item).replace(',', ' ').split():
                try:
                    val = int(part.strip())
                    if val >= 4:
                        sizes.append(val)
                except ValueError:
                    pass
        sizes = sorted(list(set(sizes)))
        if sizes:
            return sizes
    
    # Generate range from min_size to max_size
    min_s = max(4, args.min_size)
    max_s = max(min_s, args.max_size)
    step = max(1, args.step)
    sizes = list(range(min_s, max_s + 1, step))
    if max_s not in sizes:
        sizes.append(max_s)
    return sizes

def generate_demo_data(window_sizes, scenario_name="Demo_Scenario"):
    """
    Generates physically representative metrics demonstrating characteristic
    VANET behavior across window sizes (4 to 30+):
    - At W=4: High variance / noise, moderate F1 (~0.91).
    - At W=8-15: Peak detection accuracy (~0.97 F1) with balanced smoothing.
    - At W>20-30: Degradation in turning maneuvers due to CTRA lag, latency increases linearly.
    """
    results = []
    base_time_per_msg_us = 8.5
    
    for w in window_sizes:
        # Latency scales approximately linearly with history window length W
        avg_latency_us = base_time_per_msg_us + 0.45 * w + np.random.normal(0, 0.15)
        avg_latency_us = max(5.0, avg_latency_us)
        throughput = 1e6 / avg_latency_us
        
        # Detection performance: inverted U-curve peaking around W=12-14
        # Small W has noise; overly large W suffers from maneuver lag during turns
        peak_w = 12.0
        dist_from_peak = abs(w - peak_w)
        penalty = 0.0035 * (dist_from_peak ** 1.3)
        
        recall = max(88.0, min(99.5, 98.8 - penalty * 100.0 + np.random.normal(0, 0.2)))
        precision = max(86.0, min(99.0, 97.5 - (penalty * 1.2) * 100.0 + np.random.normal(0, 0.2)))
        fpr = max(0.5, min(8.0, 1.2 + penalty * 80.0 + np.random.normal(0, 0.1)))
        accuracy = (recall + (100.0 - fpr)) / 2.0
        f1_score = 2 * (recall * precision) / (recall + precision) / 100.0
        
        results.append({
            'scenario': scenario_name,
            'history_size': w,
            'recall': float(recall),
            'precision': float(precision),
            'f1_score': float(f1_score),
            'fpr': float(fpr),
            'accuracy': float(accuracy),
            'avg_latency_us': float(avg_latency_us),
            'throughput_msg_s': float(throughput),
            'total_time_sec': float((avg_latency_us * 35000) / 1e6),
            'total_messages': 35000,
            'tp': int(350 * (recall / 100.0)),
            'fp': int(350 * (fpr / 100.0)),
            'tn': int(3500 - 350 * (fpr / 100.0)),
            'fn': int(350 - 350 * (recall / 100.0))
        })
    return results

def plot_results(results, scenario_title, output_filepath):
    """
    Creates a 4-panel publication-grade figure:
    1. F1 Score, Recall, and Precision vs Window Size
    2. False Positive Rate (FPR) vs Window Size
    3. Latency (us) & Throughput vs Window Size
    4. Pareto Curve: Latency vs F1 Score (Sweet spot analysis)
    """
    history_sizes = [r['history_size'] for r in results]
    f1_scores = [r['f1_score'] for r in results]
    recalls = [r['recall'] for r in results]
    precisions = [r['precision'] for r in results]
    fprs = [r['fpr'] for r in results]
    latencies = [r['avg_latency_us'] for r in results]
    throughputs = [r['throughput_msg_s'] for r in results]

    best_f1_idx = int(np.argmax(f1_scores))
    best_w = history_sizes[best_f1_idx]

    plt.rcParams.update({
        'font.size': 11,
        'axes.labelsize': 12,
        'axes.titlesize': 13,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'legend.fontsize': 10,
        'figure.titlesize': 15
    })

    fig, axs = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f"VeReMi Tracking History Window Analysis: {scenario_title}", fontweight='bold', y=0.98)

    # -------------------------------------------------------------------------
    # Panel 1: Detection Performance (F1, Recall, Precision)
    # -------------------------------------------------------------------------
    ax1 = axs[0, 0]
    ax1.plot(history_sizes, [f * 100 for f in f1_scores], marker='o', color='#1f77b4', linewidth=2.2, label='F1 Score (%)')
    ax1.plot(history_sizes, recalls, marker='s', color='#2ca02c', linewidth=2.0, linestyle='--', label='Recall (Detection Rate %)')
    ax1.plot(history_sizes, precisions, marker='^', color='#ff7f0e', linewidth=2.0, linestyle=':', label='Precision (%)')
    
    # Highlight optimal window
    ax1.axvline(best_w, color='#d62728', linestyle='-.', alpha=0.7, label=f'Optimal F1 (W={best_w})')
    ax1.scatter([best_w], [f1_scores[best_f1_idx] * 100], color='#d62728', s=100, zorder=5)

    ax1.set_title("Detection Performance vs. Window Size", fontweight='bold')
    ax1.set_xlabel("Tracking History Window Size (W)")
    ax1.set_ylabel("Score (%)")
    ax1.grid(True, linestyle='--', alpha=0.5)
    ax1.legend(loc='lower right', frameon=True)

    # -------------------------------------------------------------------------
    # Panel 2: False Positive Rate (FPR)
    # -------------------------------------------------------------------------
    ax2 = axs[0, 1]
    ax2.plot(history_sizes, fprs, marker='d', color='#d62728', linewidth=2.2, label='False Positive Rate (FPR %)')
    ax2.axvline(best_w, color='#1f77b4', linestyle='-.', alpha=0.7, label=f'Optimal F1 (W={best_w})')
    
    ax2.set_title("False Positive Rate vs. Window Size", fontweight='bold')
    ax2.set_xlabel("Tracking History Window Size (W)")
    ax2.set_ylabel("FPR (%)")
    ax2.grid(True, linestyle='--', alpha=0.5)
    ax2.legend(loc='upper right', frameon=True)

    # -------------------------------------------------------------------------
    # Panel 3: Computational Latency & Throughput
    # -------------------------------------------------------------------------
    ax3 = axs[1, 0]
    ax3_twin = ax3.twinx()
    
    line1 = ax3.plot(history_sizes, latencies, marker='o', color='#9467bd', linewidth=2.2, label='Avg Latency (µs/msg)')
    line2 = ax3_twin.plot(history_sizes, throughputs, marker='x', color='#8c564b', linewidth=2.0, linestyle='--', label='Throughput (msg/s)')

    ax3.set_title("Computational Cost vs. Window Size", fontweight='bold')
    ax3.set_xlabel("Tracking History Window Size (W)")
    ax3.set_ylabel("Avg Latency per Message (µs)", color='#9467bd')
    ax3_twin.set_ylabel("Throughput (messages / sec)", color='#8c564b')
    ax3.grid(True, linestyle='--', alpha=0.5)

    # Combined legend
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax3.legend(lines, labels, loc='center right', frameon=True)

    # -------------------------------------------------------------------------
    # Panel 4: Pareto Efficiency Frontier (Latency vs F1 Score)
    # -------------------------------------------------------------------------
    ax4 = axs[1, 1]
    scatter = ax4.scatter(latencies, [f * 100 for f in f1_scores], c=history_sizes, cmap='viridis', s=90, edgecolors='black', zorder=4)
    cbar = plt.colorbar(scatter, ax=ax4)
    cbar.set_label('Window Size (W)')

    for i, w in enumerate(history_sizes):
        ax4.annotate(f"W={w}", (latencies[i], f1_scores[i] * 100),
                     textcoords="offset points", xytext=(6, 4), fontsize=9)

    ax4.plot(latencies, [f * 100 for f in f1_scores], color='gray', linestyle=':', alpha=0.6, zorder=2)
    ax4.scatter([latencies[best_f1_idx]], [f1_scores[best_f1_idx] * 100], color='#d62728', s=130, marker='*', zorder=5, label=f'Best F1 (W={best_w})')

    ax4.set_title("Pareto Frontier: Latency vs. F1 Score", fontweight='bold')
    ax4.set_xlabel("Avg Latency per Message (µs) [Lower is better]")
    ax4.set_ylabel("F1 Score (%) [Higher is better]")
    ax4.grid(True, linestyle='--', alpha=0.5)
    ax4.legend(loc='lower left', frameon=True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    os.makedirs(os.path.dirname(os.path.abspath(output_filepath)), exist_ok=True)
    plt.savefig(output_filepath, dpi=300)
    print(f"Plot saved successfully to: {output_filepath}")
    
    pdf_filepath = os.path.splitext(output_filepath)[0] + ".pdf"
    plt.savefig(pdf_filepath)
    print(f"Vector PDF plot saved to:   {pdf_filepath}")
    plt.close()

def filter_scenarios_by_variant(scenarios, variant):
    if not variant or str(variant).lower() in ('all', 'both'):
        return scenarios
    v = str(variant).strip()
    return [s for s in scenarios if v in s]

def evaluate_scenario(scenario_name, window_sizes, workspace_dir, results_dir, plots_dir, runner_path, args, timestamp):
    dirs = run_veremi_batch.find_parquet_dirs(workspace_dir, scenario_name)
    if not dirs:
        print(f"[{scenario_name}] No parquet simulation directories found.")
        return None

    if args.limit:
        dirs = dirs[:args.limit]

    print(f"\n=======================================================")
    print(f" Running live window size sweep on: {scenario_name}")
    print(f" Found {len(dirs)} simulation directory(ies)")
    print(f" Window sizes: {window_sizes}")
    print(f"=======================================================")

    sweep_results = []
    for w in window_sizes:
        print(f"[{scenario_name}] Testing Window Size = {w}...")
        metrics = run_veremi_batch.process_scenario(
            dirs, runner_path, history_size=w, 
            dr_model=args.dr, trust_model=args.trust, threshold=args.threshold
        )
        if metrics:
            metrics['scenario'] = scenario_name
            sweep_results.append(metrics)

    if not sweep_results:
        print(f"[{scenario_name}] Error: No metrics collected.")
        return None

    # Save JSON results
    run_dir = os.path.join(results_dir, f"run_{timestamp}")
    os.makedirs(run_dir, exist_ok=True)
    json_path = os.path.join(run_dir, f"window_sweep_{scenario_name}_{timestamp}.json")
    with open(json_path, 'w') as f:
        json.dump(sweep_results, f, indent=4)
    print(f"[{scenario_name}] Saved sweep JSON to: {json_path}")

    # Generate plot
    output_file = args.output if (args.output and not args.all) else os.path.join(plots_dir, f"window_sweep_{scenario_name}_{timestamp}.png")
    plot_results(sweep_results, scenario_name, output_file)
    run_veremi_batch.print_comparison_table(sweep_results, title=f"Window Size Sweep: {scenario_name}")
    return json_path

def main():
    parser = argparse.ArgumentParser(description="Sweep history window sizes and generate comparison plots")
    parser.add_argument("--scenario", default=None, help="Scenario to evaluate (e.g. ConstPos_0709)")
    parser.add_argument("--all", action="store_true", help="Execute sweep across all available parquet scenarios")
    parser.add_argument("--variant", default=None, choices=["0709", "1416", "all", "both"], 
                        help="Filter scenarios by time/density variant: '0709' (rush hour), '1416' (afternoon), or 'all'/'both'")
    parser.add_argument("-j", "--jobs", type=int, default=1, help="Number of scenarios to process in parallel (default: 1)")
    parser.add_argument("--compare", action="store_true", help="Automatically generate multi-scenario comparison report after sweep")
    parser.add_argument("--min-size", type=int, default=4, help="Starting window size (default: 4)")
    parser.add_argument("--max-size", type=int, default=30, help="Maximum window size to test (default: 30)")
    parser.add_argument("--step", type=int, default=2, help="Step between window sizes (default: 2)")
    parser.add_argument("--window-sizes", nargs="+", default=None, help="Explicit list of window sizes (e.g. 4 6 8 10 15 20 25 30)")
    parser.add_argument("--from-json", default=None, help="Plot directly from an existing sweep_results.json or history_sweep.json")
    parser.add_argument("--output", default=None, help="Path to save output plot (.png)")
    parser.add_argument("--demo", action="store_true", help="Generate an illustrative benchmark plot without running runner")
    parser.add_argument("--limit", type=int, default=None, help="Limit number of chunks to process")
    parser.add_argument("--dr", default="HistoryAwareCTRA", help="Dead reckoning model (default: HistoryAwareCTRA)")
    parser.add_argument("--trust", default="BetaReputation", help="Trust evaluator model (default: BetaReputation)")
    parser.add_argument("--threshold", type=float, default=0.5, help="Trust score threshold (default: 0.5)")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.abspath(os.path.join(script_dir, "../../.."))
    results_dir = os.path.join(os.path.dirname(script_dir), "results")
    plots_dir = os.path.join(results_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    window_sizes = parse_window_sizes(args)

    # -------------------------------------------------------------------------
    # Mode 1: Load from existing JSON file
    # -------------------------------------------------------------------------
    if args.from_json:
        if not os.path.exists(args.from_json):
            print(f"Error: Specified JSON file not found: {args.from_json}")
            sys.exit(1)
        with open(args.from_json, 'r') as f:
            data = json.load(f)
        scenario_name = data[0].get('scenario', 'Sweep') if data else 'Sweep'
        output_file = args.output or os.path.join(plots_dir, f"window_sweep_{scenario_name}_{timestamp}.png")
        plot_results(data, scenario_name, output_file)
        return

    # -------------------------------------------------------------------------
    # Mode 2: Demo Mode (Synthetic Benchmark)
    # -------------------------------------------------------------------------
    if args.demo:
        print(f"Generating illustrative benchmark plot for window sizes: {window_sizes}...")
        scenario_name = args.scenario or "VeReMi_Benchmark_Simulation"
        demo_results = generate_demo_data(window_sizes, scenario_name)
        output_file = args.output or os.path.join(plots_dir, f"window_sweep_{scenario_name}_demo_{timestamp}.png")
        plot_results(demo_results, f"{scenario_name} (Benchmark Model)", output_file)
        run_veremi_batch.print_comparison_table(demo_results, title=f"Window Size Analysis: {scenario_name}")
        return

    # -------------------------------------------------------------------------
    # Mode 3: Live Evaluation
    # -------------------------------------------------------------------------
    runner_path = os.path.join(workspace_dir, "cp-trustfulness", "veremi-extension", "build", "veremi_runner")
    if not os.path.exists(runner_path):
        print(f"Error: {runner_path} not found. Please compile it first.")
        sys.exit(1)

    # Resolve target scenarios
    run_all = args.all or (args.variant is not None and not args.scenario)

    if run_all:
        all_scenarios = run_veremi_batch.find_all_scenarios(workspace_dir)
        target_scenarios = filter_scenarios_by_variant(all_scenarios, args.variant)
        
        if not target_scenarios:
            print(f"No scenarios found matching variant '{args.variant}'. Discovered: {all_scenarios}")
            sys.exit(1)

        print(f"=======================================================")
        print(f" Multi-Scenario Window Sweep")
        print(f" Target Scenarios ({len(target_scenarios)}): {', '.join(target_scenarios)}")
        print(f" Variant Filter: {args.variant or 'All'}")
        print(f" Window Sizes: {window_sizes}")
        print(f" Parallel Jobs: {args.jobs}")
        print(f"=======================================================")

        json_paths = []
        if args.jobs > 1 and len(target_scenarios) > 1:
            from concurrent.futures import ThreadPoolExecutor
            def _worker(sc):
                return evaluate_scenario(sc, window_sizes, workspace_dir, results_dir, plots_dir, runner_path, args, timestamp)
            
            with ThreadPoolExecutor(max_workers=args.jobs) as executor:
                results = list(executor.map(_worker, target_scenarios))
            json_paths = [r for r in results if r is not None]
        else:
            for sc in target_scenarios:
                res = evaluate_scenario(sc, window_sizes, workspace_dir, results_dir, plots_dir, runner_path, args, timestamp)
                if res:
                    json_paths.append(res)

        print(f"\nCompleted sweep for {len(json_paths)} scenarios.")

        if args.compare or len(json_paths) > 1:
            print("\nTriggering multi-scenario comparison analysis...")
            import plot_multi_scenario_comparison
            df = plot_multi_scenario_comparison.load_sweep_data({os.path.basename(f): f for f in json_paths})
            summary_df = plot_multi_scenario_comparison.compute_decision_metrics(df)
            
            prefix = f"variant_{args.variant}_{timestamp}" if args.variant else f"all_scenarios_{timestamp}"
            heatmap_file = os.path.join(plots_dir, f"{prefix}_heatmap.png")
            dashboard_file = os.path.join(plots_dir, f"{prefix}_dashboard.png")
            decision_md = os.path.join(results_dir, f"{prefix}_decision_summary.md")
            decision_csv = os.path.join(results_dir, f"{prefix}_decision_summary.csv")

            plot_multi_scenario_comparison.plot_heatmap(df, summary_df, heatmap_file)
            plot_multi_scenario_comparison.plot_dashboard(df, summary_df, dashboard_file)
            plot_multi_scenario_comparison.print_and_save_decision_table(summary_df, target_scenarios, decision_md, decision_csv)
        return

    # Single scenario mode
    if not args.scenario:
        print("Notice: No scenario specified and --all not used. Running in demo mode.")
        print("To run on all scenarios, use:   python3 plot_window_sweep.py --all [--variant 0709|1416]")
        print("To run on single scenario, use: python3 plot_window_sweep.py --scenario <ScenarioName>")
        scenario_name = "VeReMi_Simulation"
        demo_results = generate_demo_data(window_sizes, scenario_name)
        output_file = args.output or os.path.join(plots_dir, f"window_sweep_{scenario_name}_{timestamp}.png")
        plot_results(demo_results, f"{scenario_name} (Benchmark Model)", output_file)
        run_veremi_batch.print_comparison_table(demo_results, title=f"Window Size Analysis: {scenario_name}")
        return

    evaluate_scenario(args.scenario, window_sizes, workspace_dir, results_dir, plots_dir, runner_path, args, timestamp)

if __name__ == "__main__":
    main()
