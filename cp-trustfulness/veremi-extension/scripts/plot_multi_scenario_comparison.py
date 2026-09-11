#!/usr/bin/env python3
"""
plot_multi_scenario_comparison.py

Aggregates window sweep benchmark results across multiple VeReMi scenarios.
Generates:
1. Scenario x Window Size Performance Heatmap (F1 Score & Detection Rate)
2. Multi-Objective Analysis Dashboard (Macro F1 with std bands, Latency, Pareto Frontier)
3. Decision Table with Average Ranking and Multi-Objective Utility Scoring
"""

import os
import sys
import glob
import json
import argparse
from datetime import datetime
import pandas as pd
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import seaborn as sns


def find_latest_scenario_jsons(results_dir):
    """
    Scans the results directory for window_sweep_*.json files
    and picks the most recent run for each unique scenario.
    """
    pattern = os.path.join(results_dir, "run_*", "window_sweep_*.json")
    all_files = glob.glob(pattern)
    
    scenario_files = {}
    for f in all_files:
        basename = os.path.basename(f)
        # Expected format: window_sweep_<ScenarioName>_<timestamp>.json
        parts = basename.replace("window_sweep_", "").split("_")
        if len(parts) >= 2:
            # Reconstruct scenario name (handle scenarios containing underscores like ConstPosOffset_1416)
            timestamp_part = parts[-1].replace(".json", "")
            if len(timestamp_part) == 6 and len(parts) >= 3 and len(parts[-2]) == 8:
                # Timestamp is YYYYMMDD_HHMMSS
                scenario = "_".join(parts[:-2])
            else:
                scenario = "_".join(parts[:-1])
        else:
            scenario = basename.replace(".json", "")

        mtime = os.path.getmtime(f)
        if scenario not in scenario_files or mtime > scenario_files[scenario]['mtime']:
            scenario_files[scenario] = {'path': f, 'mtime': mtime}

    return {sc: info['path'] for sc, info in scenario_files.items()}


def load_sweep_data(file_paths):
    """
    Loads JSON sweep files into a unified DataFrame.
    """
    records = []
    for sc_name, path in file_paths.items():
        with open(path, 'r') as f:
            data = json.load(f)
            for row in data:
                row['scenario'] = row.get('scenario', sc_name)
                records.append(row)
    return pd.DataFrame(records)


def compute_decision_metrics(df, w_f1=0.45, w_worst=0.25, w_fpr=0.15, w_lat=0.15):
    """
    Computes macro averages, standard deviations, worst-case scores,
    Borda count ranks, and multi-objective composite utility.
    """
    scenarios = df['scenario'].unique()
    window_sizes = sorted(df['history_size'].unique())

    # 1. Compute per-scenario rank for each window size
    df['scenario_f1_rank'] = df.groupby('scenario')['f1_score'].rank(ascending=False, method='min')

    # 2. Macro aggregation per window size
    summary_rows = []
    for w in window_sizes:
        sub = df[df['history_size'] == w]
        if sub.empty:
            continue

        mean_f1 = sub['f1_score'].mean()
        std_f1 = sub['f1_score'].std() if len(sub) > 1 else 0.0
        min_f1 = sub['f1_score'].min()
        max_f1 = sub['f1_score'].max()

        mean_recall = sub['recall'].mean()
        mean_prec = sub['precision'].mean()
        mean_acc = sub['accuracy'].mean()
        mean_fpr = sub['fpr'].mean()

        mean_lat = sub['avg_latency_us'].mean()
        mean_tput = sub['throughput_msg_s'].mean()
        avg_rank = sub['scenario_f1_rank'].mean()

        summary_rows.append({
            'history_size': int(w),
            'mean_f1': float(mean_f1),
            'std_f1': float(std_f1),
            'min_f1': float(min_f1),
            'max_f1': float(max_f1),
            'mean_recall': float(mean_recall),
            'mean_precision': float(mean_prec),
            'mean_accuracy': float(mean_acc),
            'mean_fpr': float(mean_fpr),
            'mean_latency_us': float(mean_lat),
            'mean_throughput': float(mean_tput),
            'avg_rank': float(avg_rank),
            'num_scenarios': len(sub)
        })

    summary_df = pd.DataFrame(summary_rows)

    # 3. Calculate Normalized Multi-Objective Utility
    # Normalize features to [0, 1]
    f1_min, f1_max = summary_df['mean_f1'].min(), summary_df['mean_f1'].max()
    norm_f1 = (summary_df['mean_f1'] - f1_min) / (f1_max - f1_min + 1e-9)

    worst_min, worst_max = summary_df['min_f1'].min(), summary_df['min_f1'].max()
    norm_worst = (summary_df['min_f1'] - worst_min) / (worst_max - worst_min + 1e-9) if worst_max > worst_min else np.ones(len(summary_df))

    fpr_min, fpr_max = summary_df['mean_fpr'].min(), summary_df['mean_fpr'].max()
    norm_fpr = (summary_df['mean_fpr'] - fpr_min) / (fpr_max - fpr_min + 1e-9) if fpr_max > fpr_min else np.zeros(len(summary_df))

    lat_min, lat_max = summary_df['mean_latency_us'].min(), summary_df['mean_latency_us'].max()
    norm_lat = (summary_df['mean_latency_us'] - lat_min) / (lat_max - lat_min + 1e-9) if lat_max > lat_min else np.zeros(len(summary_df))

    # Composite Utility (Higher is better)
    utility = (w_f1 * norm_f1 + 
               w_worst * norm_worst + 
               w_fpr * (1.0 - norm_fpr) + 
               w_lat * (1.0 - norm_lat))

    summary_df['utility_score'] = utility
    summary_df['final_rank'] = summary_df['utility_score'].rank(ascending=False, method='min').astype(int)
    summary_df = summary_df.sort_values('history_size').reset_index(drop=True)

    return summary_df


def plot_heatmap(df, summary_df, output_path):
    """
    Generates a high-density publication heatmap of Scenarios vs Window Sizes.
    Uses stacked subplots to cleanly separate individual attack scenarios from
    aggregate summary rows (Macro Mean, Worst-case, Latency).
    """
    pivot_f1 = df.pivot(index='scenario', columns='history_size', values='f1_score')
    pivot_f1 = pivot_f1.sort_index()

    w_sizes = list(pivot_f1.columns)
    
    # Summary rows
    mean_series = pd.Series({w: summary_df.loc[summary_df['history_size'] == w, 'mean_f1'].values[0] for w in w_sizes}, name="[Macro Mean F1]")
    worst_series = pd.Series({w: summary_df.loc[summary_df['history_size'] == w, 'min_f1'].values[0] for w in w_sizes}, name="[Worst-Case F1]")
    lat_series = pd.Series({w: summary_df.loc[summary_df['history_size'] == w, 'mean_latency_us'].values[0] for w in w_sizes}, name="[Avg Latency (µs)]")

    summary_matrix = pd.DataFrame([mean_series, worst_series, lat_series])

    num_scenarios = len(pivot_f1)
    num_summaries = len(summary_matrix)

    best_w = int(summary_df.sort_values('utility_score', ascending=False).iloc[0]['history_size'])
    best_w_idx = w_sizes.index(best_w)

    fig, (ax_top, ax_bot) = plt.subplots(
        2, 1, 
        figsize=(max(11, len(w_sizes) * 0.95), max(6, num_scenarios * 0.9 + num_summaries * 0.7 + 2.5)),
        gridspec_kw={'height_ratios': [num_scenarios, num_summaries], 'hspace': 0.12},
        sharex=True
    )

    # 1. Scenarios Heatmap
    sns.heatmap(
        pivot_f1, 
        annot=True, 
        fmt=".3f", 
        cmap="YlGnBu", 
        cbar_kws={'label': 'F1 Score'},
        linewidths=1.2, 
        linecolor='white',
        ax=ax_top
    )
    ax_top.set_title(f"VeReMi Multi-Scenario Window Sensitivity Matrix (Optimal: W* = {best_w})", 
                     fontsize=14, fontweight='bold', pad=12)
    ax_top.set_ylabel("Attack Scenarios", fontsize=12, fontweight='bold', labelpad=8)
    ax_top.set_xlabel("")
    ax_top.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)

    # Highlight optimal column on top
    ax_top.add_patch(plt.Rectangle((best_w_idx, 0), 1, num_scenarios, fill=False, edgecolor='#d62728', lw=2.8, linestyle='--'))

    # 2. Summary Rows Heatmap
    sns.heatmap(
        summary_matrix, 
        annot=True, 
        fmt=".3f", 
        cmap="Blues", 
        cbar=False,
        linewidths=1.2, 
        linecolor='white',
        ax=ax_bot
    )
    ax_bot.set_ylabel("Aggregates", fontsize=12, fontweight='bold', labelpad=8)
    ax_bot.set_xlabel("Tracking History Window Size (W)", fontsize=12, fontweight='bold', labelpad=10)

    # Highlight optimal column on bottom
    ax_bot.add_patch(plt.Rectangle((best_w_idx, 0), 1, num_summaries, fill=False, edgecolor='#d62728', lw=2.8, linestyle='--'))

    plt.subplots_adjust(top=0.92, bottom=0.12, left=0.18, right=0.92, hspace=0.15)
    plt.savefig(output_path, dpi=300)
    pdf_path = os.path.splitext(output_path)[0] + ".pdf"
    plt.savefig(pdf_path)
    plt.close()
    print(f"Heatmap saved to: {output_path} (and {pdf_path})")


def plot_dashboard(df, summary_df, output_path):
    """
    Generates a 4-panel multi-scenario decision dashboard.
    """
    best_row = summary_df.sort_values('utility_score', ascending=False).iloc[0]
    best_w = int(best_row['history_size'])

    w_vals = summary_df['history_size']
    mean_f1 = summary_df['mean_f1'] * 100
    std_f1 = summary_df['std_f1'] * 100
    mean_recall = summary_df['mean_recall']
    mean_prec = summary_df['mean_precision']
    mean_fpr = summary_df['mean_fpr']
    min_f1 = summary_df['min_f1'] * 100
    latencies = summary_df['mean_latency_us']
    tputs = summary_df['mean_throughput']
    utilities = summary_df['utility_score']

    plt.rcParams.update({
        'font.size': 11,
        'axes.labelsize': 12,
        'axes.titlesize': 13,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'legend.fontsize': 10,
        'figure.titlesize': 15
    })

    fig, axs = plt.subplots(2, 2, figsize=(15, 11))
    fig.suptitle("VeReMi Multi-Scenario Window Optimization Dashboard", fontweight='bold', y=0.98)

    # -------------------------------------------------------------------------
    # Panel 1: Macro Detection Performance (Mean +/- Std Dev Band)
    # -------------------------------------------------------------------------
    ax1 = axs[0, 0]
    ax1.plot(w_vals, mean_f1, marker='o', color='#1f77b4', linewidth=2.5, label='Macro Mean F1 (%)')
    ax1.fill_between(w_vals, np.maximum(0, mean_f1 - std_f1), mean_f1 + std_f1, color='#1f77b4', alpha=0.2, label='±1 Std Dev (Scenario Variance)')
    ax1.plot(w_vals, mean_recall, marker='s', color='#2ca02c', linewidth=1.8, linestyle='--', label='Macro Recall (%)')
    ax1.plot(w_vals, mean_prec, marker='^', color='#ff7f0e', linewidth=1.8, linestyle=':', label='Macro Precision (%)')
    ax1.axvline(best_w, color='#d62728', linestyle='-.', alpha=0.8, label=f'Optimal W*={best_w}')

    ax1.set_title("1. Macro Detection Performance vs. Window Size", fontweight='bold')
    ax1.set_xlabel("Tracking History Window Size (W)")
    ax1.set_ylabel("Score (%)")
    ax1.grid(True, linestyle='--', alpha=0.5)
    ax1.legend(loc='lower right', frameon=True)

    # -------------------------------------------------------------------------
    # Panel 2: Worst-Case Robustness & False Alarms
    # -------------------------------------------------------------------------
    ax2 = axs[0, 1]
    ax2_twin = ax2.twinx()

    line1 = ax2.plot(w_vals, min_f1, marker='v', color='#d62728', linewidth=2.2, label='Worst-Case Scenario F1 (%)')
    line2 = ax2_twin.plot(w_vals, mean_fpr, marker='d', color='#7f7f7f', linewidth=2.0, linestyle='--', label='Macro False Positive Rate (FPR %)')
    ax2.axvline(best_w, color='#1f77b4', linestyle='-.', alpha=0.7)

    ax2.set_title("2. Worst-Case Attack Robustness vs. False Alarms", fontweight='bold')
    ax2.set_xlabel("Tracking History Window Size (W)")
    ax2.set_ylabel("Worst-Case F1 (%)", color='#d62728')
    ax2_twin.set_ylabel("Macro FPR (%)", color='#555555')
    ax2.grid(True, linestyle='--', alpha=0.5)

    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax2.legend(lines, labels, loc='center right', frameon=True)

    # -------------------------------------------------------------------------
    # Panel 3: Computational Cost Scaling (Latency & Throughput)
    # -------------------------------------------------------------------------
    ax3 = axs[1, 0]
    ax3_twin = ax3.twinx()

    line3 = ax3.plot(w_vals, latencies, marker='o', color='#9467bd', linewidth=2.2, label='Avg Latency (µs/msg)')
    line4 = ax3_twin.plot(w_vals, tputs, marker='x', color='#8c564b', linewidth=2.0, linestyle='--', label='Throughput (msg/s)')
    ax3.axvline(best_w, color='#d62728', linestyle='-.', alpha=0.8)

    ax3.set_title("3. Computational Overhead vs. Window Size", fontweight='bold')
    ax3.set_xlabel("Tracking History Window Size (W)")
    ax3.set_ylabel("Avg Latency per Message (µs)", color='#9467bd')
    ax3_twin.set_ylabel("Throughput (messages / sec)", color='#8c564b')
    ax3.grid(True, linestyle='--', alpha=0.5)

    lines = line3 + line4
    labels = [l.get_label() for l in lines]
    ax3.legend(lines, labels, loc='center right', frameon=True)

    # -------------------------------------------------------------------------
    # Panel 4: Multi-Objective Utility & Pareto Sweet Spot
    # -------------------------------------------------------------------------
    ax4 = axs[1, 1]
    
    # Plot composite utility curve
    ax4.plot(w_vals, utilities, marker='o', color='#008080', linewidth=2.4, label='Composite Utility Score')
    ax4.scatter([best_w], [best_row['utility_score']], color='#d62728', s=160, marker='*', zorder=5, label=f'Best Compromise (W*={best_w})')

    for i, w in enumerate(w_vals):
        ax4.annotate(f"W={w}", (w, utilities[i]), textcoords="offset points", xytext=(0, 6), ha='center', fontsize=8)

    ax4.set_title("4. Multi-Objective Decision Curve (Trade-Off Score)", fontweight='bold')
    ax4.set_xlabel("Tracking History Window Size (W)")
    ax4.set_ylabel("Normalized Composite Utility [0 - 1]")
    ax4.grid(True, linestyle='--', alpha=0.5)
    ax4.legend(loc='lower right', frameon=True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    plt.savefig(output_path, dpi=300)
    pdf_path = os.path.splitext(output_path)[0] + ".pdf"
    plt.savefig(pdf_path)
    plt.close()
    print(f"Dashboard saved to: {output_path} (and {pdf_path})")


def print_and_save_decision_table(summary_df, scenarios, output_md_path, output_csv_path):
    """
    Formats and prints the multi-scenario decision matrix to console and markdown.
    """
    best_row = summary_df.sort_values('utility_score', ascending=False).iloc[0]
    best_w = int(best_row['history_size'])

    header = (f"{'Window (W)':<11} | {'Macro F1':<18} | {'Worst F1':<10} | {'Macro Rec':<11} | "
              f"{'Macro Prec':<12} | {'Macro FPR':<10} | {'Latency (us)':<13} | {'Avg Rank':<9} | {'Utility':<8}")
    div = "-" * len(header)

    print("\n" + "=" * len(header))
    print(f"{'VE-RE-MI MULTI-SCENARIO DECISION MATRIX':^{len(header)}}")
    print(f"{f'Evaluated across {len(scenarios)} scenarios: ' + ', '.join(scenarios):^{len(header)}}")
    print("=" * len(header))
    print(header)
    print(div)

    for _, r in summary_df.iterrows():
        opt_flag = " *" if int(r['history_size']) == best_w else ""
        row_str = (
            f"W = {int(r['history_size']):<4}{opt_flag:<3} | "
            f"{r['mean_f1']*100:>5.2f}% ± {r['std_f1']*100:<5.2f}% | "
            f"{r['min_f1']*100:>6.2f}%   | "
            f"{r['mean_recall']:>7.2f}%    | "
            f"{r['mean_precision']:>8.2f}%   | "
            f"{r['mean_fpr']:>6.2f}%   | "
            f"{r['mean_latency_us']:>7.2f} µs    | "
            f"{r['avg_rank']:>6.2f}    | "
            f"{r['utility_score']:>6.3f}"
        )
        print(row_str)
    print("=" * len(header))
    print(f"(*) RECOMMENDED OPTIMAL HISTORY WINDOW SIZE: W* = {best_w} (Utility Score: {best_row['utility_score']:.3f})")
    print("=" * len(header) + "\n")

    # Save to CSV
    summary_df.to_csv(output_csv_path, index=False)
    print(f"Saved CSV decision summary to: {output_csv_path}")

    # Save to Markdown
    md_lines = [
        f"# VeReMi Multi-Scenario Window Size Decision Summary\n",
        f"**Evaluated Scenarios ({len(scenarios)})**: {', '.join(f'`{s}`' for s in scenarios)}\n",
        f"**Recommended Optimal History Window Size**: **`W* = {best_w}`**\n\n",
        "## Performance Decision Matrix\n",
        "| Window Size (W) | Macro F1 (%) | Worst-Case F1 (%) | Macro Recall (%) | Macro Precision (%) | Macro FPR (%) | Mean Latency (µs) | Average Rank | Composite Utility |",
        "|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|"
    ]

    for _, r in summary_df.iterrows():
        opt = " **(Optimal)**" if int(r['history_size']) == best_w else ""
        md_lines.append(
            f"| **{int(r['history_size'])}**{opt} | "
            f"{r['mean_f1']*100:.2f}% ± {r['std_f1']*100:.2f}% | "
            f"{r['min_f1']*100:.2f}% | "
            f"{r['mean_recall']:.2f}% | "
            f"{r['mean_precision']:.2f}% | "
            f"{r['mean_fpr']:.2f}% | "
            f"{r['mean_latency_us']:.2f} µs | "
            f"{r['avg_rank']:.2f} | "
            f"**{r['utility_score']:.3f}** |"
        )

    md_lines.append("\n### Decision Rationale:")
    md_lines.append(f"- **Accuracy Trade-off**: At `W = {best_w}`, the detector achieves a macro F1 of {best_row['mean_f1']*100:.2f}% with balanced recall ({best_row['mean_recall']:.2f}%) and precision ({best_row['mean_precision']:.2f}%).")
    md_lines.append(f"- **Computational Scalability**: Average per-message latency is only **{best_row['mean_latency_us']:.2f} µs**, which easily satisfies real-time 10 Hz / 100 ms vehicular message budgets (sustaining over {int(best_row['mean_throughput']):,} messages/sec).")
    md_lines.append(f"- **Multi-Scenario Robustness**: Averaged across all attack types, this window size delivers the highest composite utility score ({best_row['utility_score']:.3f}).\n")

    with open(output_md_path, 'w') as f:
        f.write("\n".join(md_lines))
    print(f"Saved Markdown decision summary to: {output_md_path}")


def main():
    parser = argparse.ArgumentParser(description="Aggregate and compress multi-scenario window sweeps to determine best general window size")
    parser.add_argument("--results-dir", default=None, help="Directory containing run_* folders with JSON sweep files")
    parser.add_argument("--files", nargs="+", default=None, help="Explicit list of sweep JSON files to compare")
    parser.add_argument("--scenarios", nargs="+", default=None, help="Filter specific scenarios (e.g. ConstPosOffset_1416 ConstSpeed_1416)")
    parser.add_argument("--output-prefix", default=None, help="Custom prefix for output plots and reports")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.abspath(os.path.join(script_dir, "../../.."))
    results_dir = args.results_dir or os.path.join(os.path.dirname(script_dir), "results")
    plots_dir = os.path.join(results_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # 1. Resolve files
    if args.files:
        file_map = {os.path.basename(f): f for f in args.files}
    else:
        file_map = find_latest_scenario_jsons(results_dir)

    if not file_map:
        print(f"Error: No window sweep JSON files found in {results_dir}.")
        print("Run plot_window_sweep.py on your scenarios first!")
        sys.exit(1)

    # Filter scenarios if requested
    if args.scenarios:
        file_map = {k: v for k, v in file_map.items() if any(sc in k for sc in args.scenarios)}

    print(f"Discovered {len(file_map)} scenario runs:")
    for sc, p in file_map.items():
        print(f"  - {sc}: {p}")

    # 2. Load data
    df = load_sweep_data(file_map)
    scenarios = sorted(df['scenario'].unique())

    # Keep common or valid window sizes
    summary_df = compute_decision_metrics(df)

    # 3. Generate Output File Paths
    prefix = args.output_prefix or f"multi_scenario_{timestamp}"
    heatmap_file = os.path.join(plots_dir, f"{prefix}_heatmap.png")
    dashboard_file = os.path.join(plots_dir, f"{prefix}_dashboard.png")
    decision_md = os.path.join(results_dir, f"{prefix}_decision_summary.md")
    decision_csv = os.path.join(results_dir, f"{prefix}_decision_summary.csv")

    # 4. Generate Visualizations & Tables
    plot_heatmap(df, summary_df, heatmap_file)
    plot_dashboard(df, summary_df, dashboard_file)
    print_and_save_decision_table(summary_df, scenarios, decision_md, decision_csv)


if __name__ == "__main__":
    main()
