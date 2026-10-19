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
import glob
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


def _build_panel_detection(ax, history_sizes, f1_scores, recalls, precisions, best_w, best_f1_idx, show_title=True):
    ax.plot(history_sizes, [f * 100 for f in f1_scores], marker='o', color='#1f77b4', linewidth=2.2, label='F1 Score (%)')
    ax.plot(history_sizes, recalls, marker='s', color='#2ca02c', linewidth=2.0, linestyle='--', label='Recall (Detection Rate %)')
    ax.plot(history_sizes, precisions, marker='^', color='#ff7f0e', linewidth=2.0, linestyle=':', label='Precision (%)')
    ax.axvline(best_w, color='#d62728', linestyle='-.', alpha=0.7, label=f'Optimal F1 (W={best_w})')
    ax.scatter([best_w], [f1_scores[best_f1_idx] * 100], color='#d62728', s=100, zorder=5)
    if show_title:
        ax.set_title("Detection Performance vs. Window Size", fontweight='bold')
    ax.set_xlabel("Tracking History Window Size (W)")
    ax.set_ylabel("Score (%)")
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(loc='lower right', frameon=True)


def _build_panel_fpr(ax, history_sizes, fprs, best_w, show_title=True):
    ax.plot(history_sizes, fprs, marker='d', color='#d62728', linewidth=2.2, label='False Positive Rate (FPR %)')
    ax.axvline(best_w, color='#1f77b4', linestyle='-.', alpha=0.7, label=f'Optimal F1 (W={best_w})')
    if show_title:
        ax.set_title("False Positive Rate vs. Window Size", fontweight='bold')
    ax.set_xlabel("Tracking History Window Size (W)")
    ax.set_ylabel("FPR (%)")
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(loc='upper right', frameon=True)


def _build_panel_latency(ax, history_sizes, latencies, throughputs, show_title=True):
    ax_twin = ax.twinx()
    line1 = ax.plot(history_sizes, latencies, marker='o', color='#9467bd', linewidth=2.2, label='Avg Latency (µs/msg)')
    line2 = ax_twin.plot(history_sizes, throughputs, marker='x', color='#8c564b', linewidth=2.0, linestyle='--', label='Throughput (msg/s)')
    if show_title:
        ax.set_title("Computational Cost vs. Window Size", fontweight='bold')
    ax.set_xlabel("Tracking History Window Size (W)")
    ax.set_ylabel("Avg Latency per Message (µs)", color='#9467bd')
    ax_twin.set_ylabel("Throughput (messages / sec)", color='#8c564b')
    ax.grid(True, linestyle='--', alpha=0.5)
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc='center right', frameon=True)
    return ax_twin   # caller may need it for tight_layout


def _build_panel_pareto(ax, fig, history_sizes, latencies, f1_scores, best_w, best_f1_idx, show_title=True):
    scatter = ax.scatter(latencies, [f * 100 for f in f1_scores],
                         c=history_sizes, cmap='viridis', s=90, edgecolors='black', zorder=4)
    cbar = fig.colorbar(scatter, ax=ax)
    cbar.set_label('Window Size (W)')
    for i, w in enumerate(history_sizes):
        ax.annotate(f"W={w}", (latencies[i], f1_scores[i] * 100),
                    textcoords="offset points", xytext=(6, 4), fontsize=9)
    ax.plot(latencies, [f * 100 for f in f1_scores], color='gray', linestyle=':', alpha=0.6, zorder=2)
    ax.scatter([latencies[best_f1_idx]], [f1_scores[best_f1_idx] * 100],
               color='#d62728', s=130, marker='*', zorder=5, label=f'Best F1 (W={best_w})')
    if show_title:
        ax.set_title("Pareto Frontier: Latency vs. F1 Score", fontweight='bold')
    ax.set_xlabel("Avg Latency per Message (µs) [Lower is better]")
    ax.set_ylabel("F1 Score (%) [Higher is better]")
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(loc='lower left', frameon=True)


RC = {
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 10,
    'figure.titlesize': 15,
}

INDIVIDUAL_PANELS = [
    ("detection",  "Detection Performance vs. Tracking History Window Size"),
    ("fpr",        "False Positive Rate vs. Window Size"),
    ("latency",    "Computational Cost vs. Window Size"),
    ("pareto",     "Pareto Frontier: Latency vs. F1 Score"),
]


def plot_results(results, scenario_title, output_filepath):
    """
    Creates a 4-panel publication-grade figure and also saves each panel
    as an individual PNG/PDF alongside the combined figure.

    Individual files are placed next to output_filepath with suffixes:
        _panel_detection.png / .pdf
        _panel_fpr.png / .pdf
        _panel_latency.png / .pdf
        _panel_pareto.png / .pdf
    """
    history_sizes = [r['history_size'] for r in results]
    f1_scores     = [r['f1_score']     for r in results]
    recalls       = [r['recall']        for r in results]
    precisions    = [r['precision']     for r in results]
    fprs          = [r['fpr']           for r in results]
    latencies     = [r['avg_latency_us']     for r in results]
    throughputs   = [r['throughput_msg_s']   for r in results]

    best_f1_idx = int(np.argmax(f1_scores))
    best_w      = history_sizes[best_f1_idx]

    out_dir  = os.path.dirname(os.path.abspath(output_filepath))
    out_stem = os.path.splitext(os.path.basename(output_filepath))[0]
    os.makedirs(out_dir, exist_ok=True)

    # ------------------------------------------------------------------
    # 1.  Combined 2×2 figure (original behaviour, inner titles shown)
    # ------------------------------------------------------------------
    with plt.rc_context(RC):
        fig, axs = plt.subplots(2, 2, figsize=(14, 10))

    fig.suptitle(f"VeReMi Tracking History Window Analysis: {scenario_title}",
                 fontweight='bold', y=0.98)

    _build_panel_detection(axs[0, 0], history_sizes, f1_scores, recalls, precisions, best_w, best_f1_idx, show_title=True)
    _build_panel_fpr      (axs[0, 1], history_sizes, fprs, best_w, show_title=True)
    _build_panel_latency  (axs[1, 0], history_sizes, latencies, throughputs, show_title=True)
    _build_panel_pareto   (axs[1, 1], fig, history_sizes, latencies, f1_scores, best_w, best_f1_idx, show_title=True)

    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    fig.savefig(output_filepath, dpi=300)
    print(f"Plot saved successfully to: {output_filepath}")

    pdf_filepath = os.path.splitext(output_filepath)[0] + ".pdf"
    fig.savefig(pdf_filepath)
    print(f"Vector PDF plot saved to:   {pdf_filepath}")
    plt.close(fig)

    # ------------------------------------------------------------------
    # 2.  Individual panel figures — single title only (suptitle), no
    #     inner ax.set_title so there is no duplicate heading.
    # ------------------------------------------------------------------
    panel_builders = {
        "detection": lambda ax, fig: _build_panel_detection(
            ax, history_sizes, f1_scores, recalls, precisions, best_w, best_f1_idx, show_title=False),
        "fpr": lambda ax, fig: _build_panel_fpr(
            ax, history_sizes, fprs, best_w, show_title=False),
        "latency": lambda ax, fig: _build_panel_latency(
            ax, history_sizes, latencies, throughputs, show_title=False),
        "pareto": lambda ax, fig: _build_panel_pareto(
            ax, fig, history_sizes, latencies, f1_scores, best_w, best_f1_idx, show_title=False),
    }

    panel_suptitles = {
        "detection": f"{scenario_title} \u2014 Detection Performance vs. Tracking History Window Size",
        "fpr":       f"{scenario_title} \u2014 False Positive Rate vs. Window Size",
        "latency":   f"{scenario_title} \u2014 Computational Cost vs. Window Size",
        "pareto":    f"{scenario_title} \u2014 Pareto Frontier: Latency vs. F1 Score",
    }

    for key, suptitle_text in panel_suptitles.items():
        with plt.rc_context(RC):
            fig_ind, ax_ind = plt.subplots(figsize=(7, 5))

        fig_ind.suptitle(suptitle_text, fontweight='bold')
        panel_builders[key](ax_ind, fig_ind)
        fig_ind.tight_layout()

        png_path = os.path.join(out_dir, f"{out_stem}_panel_{key}.png")
        pdf_path = os.path.join(out_dir, f"{out_stem}_panel_{key}.pdf")
        fig_ind.savefig(png_path, dpi=300, bbox_inches='tight')
        fig_ind.savefig(pdf_path,           bbox_inches='tight')
        print(f"  Individual panel saved: {png_path}")
        print(f"  Individual panel saved: {pdf_path}")
        plt.close(fig_ind)


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
    parser.add_argument("--from-json-dir", default=None, metavar="DIR",
                        help="Regenerate all plots from every window_sweep_*.json found in DIR (e.g. results/run_20260908_212755)")
    parser.add_argument("--output", default=None, help="Path to save output plot (.png)")
    parser.add_argument("--demo", action="store_true", help="Generate an illustrative benchmark plot without running runner")
    parser.add_argument("--limit", type=int, default=None, help="Limit number of chunks to process")
    parser.add_argument("--dr", default="HistoryAwareCTRA", help="Dead reckoning model (default: HistoryAwareCTRA)")
    parser.add_argument("--trust", default="BetaReputation", help="Trust evaluator model (default: BetaReputation)")
    parser.add_argument("--threshold", type=float, default=0.5, help="Trust score threshold (default: 0.5)")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.abspath(os.path.join(script_dir, "../.."))
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
    # Mode 1b: Regenerate plots from all JSONs in a directory
    # -------------------------------------------------------------------------
    if args.from_json_dir:
        json_dir = args.from_json_dir
        if not os.path.isdir(json_dir):
            print(f"Error: Directory not found: {json_dir}")
            sys.exit(1)
        json_files = sorted(glob.glob(os.path.join(json_dir, "window_sweep_*.json")))
        if not json_files:
            print(f"Error: No window_sweep_*.json files found in {json_dir}")
            sys.exit(1)
        print(f"Found {len(json_files)} JSON files in {json_dir}")

        def _plot_from_json(json_path):
            with open(json_path, 'r') as f:
                data = json.load(f)
            scenario_name = data[0].get('scenario', os.path.basename(json_path)) if data else 'Sweep'
            output_file = os.path.join(plots_dir, f"window_sweep_{scenario_name}_{timestamp}.png")
            plot_results(data, scenario_name, output_file)
            return json_path

        if args.jobs > 1 and len(json_files) > 1:
            from concurrent.futures import ThreadPoolExecutor
            with ThreadPoolExecutor(max_workers=args.jobs) as executor:
                list(executor.map(_plot_from_json, json_files))
        else:
            for jf in json_files:
                _plot_from_json(jf)

        print(f"\nAll plots saved to: {plots_dir}")
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
    runner_path = os.path.join(os.path.dirname(script_dir), "build", "veremi_runner")
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