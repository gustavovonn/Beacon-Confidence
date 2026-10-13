#!/usr/bin/env python3
"""
report.py -- read experiment_runner's per-vector CSV and answer the questions the MVP exists to ask.

The runner prints run-wide totals. This goes further, into the breakdowns that decide whether the
confidence number is worth anything:

  1. Does confidence separate correct associations from wrong ones? If the two distributions sit on
     top of each other, the number is decoration. This is the MVP's central question.
  2. Where do wrong associations happen -- contested gates, or uncontested ones? A wrong association
     with k = 1 is a gating failure; with k > 1 it is an assignment failure. They have different
     fixes, so lumping them together hides which one to work on.
  3. Does CAM-sourced content behave like CPM-sourced content? The premise of the experiment is that
     the container stops mattering past the parser. If the two split, the premise is wrong.
  4. A confidence-threshold sweep: what is actually gained by rejecting below a threshold.

Reads only the CSV. No engine, no simulator, no dependencies beyond the standard library.
"""

import argparse
import csv
import math
import os
import sys
from collections import defaultdict


def auc_and_roc(pos, neg):
    """
    Area under the ROC of confidence against "the association was correct".

    Computed as the Mann-Whitney U statistic over midranks, which is exactly the probability that a
    randomly chosen correct update scores higher than a randomly chosen wrong one, with ties counted
    as half. That tie handling matters here: the confidence is a ratio of small integers early in a
    track's life, so exact ties are common, and the naive "count strictly greater" formula would
    report a visibly pessimistic number.

    Returns (auc, roc_points) with roc_points as (false positive rate, true positive rate), where
    "positive" means "this update was a correct association".
    """
    if not pos or not neg:
        return float("nan"), []

    merged = sorted([(v, 1) for v in pos] + [(v, 0) for v in neg])
    # midranks: every run of equal values shares the average of the ranks it spans
    ranks = [0.0] * len(merged)
    i = 0
    while i < len(merged):
        j = i
        while j + 1 < len(merged) and merged[j + 1][0] == merged[i][0]:
            j += 1
        mid = (i + j) / 2.0 + 1.0
        for k in range(i, j + 1):
            ranks[k] = mid
        i = j + 1
    rank_sum_pos = sum(r for r, (_, lab) in zip(ranks, merged) if lab == 1)
    n_p, n_n = len(pos), len(neg)
    auc = (rank_sum_pos - n_p * (n_p + 1) / 2.0) / (n_p * n_n)

    # The curve, in ONE pass down the already-sorted merge. The obvious version -- loop over every
    # distinct threshold and rescan both lists -- is quadratic, which on a few hundred thousand
    # updates means minutes instead of milliseconds.
    roc = [(0.0, 0.0)]
    tp = fp = 0
    i = len(merged) - 1
    while i >= 0:
        value = merged[i][0]
        while i >= 0 and merged[i][0] == value:   # consume the whole tie group before emitting
            if merged[i][1] == 1:
                tp += 1
            else:
                fp += 1
            i -= 1
        roc.append((fp / n_n, tp / n_p))
    return auc, roc


def quantiles(values, qs=(5, 25, 50, 75, 95)):
    if not values:
        return {q: float("nan") for q in qs}
    s = sorted(values)
    return {q: s[min(len(s) - 1, int(round(q / 100.0 * (len(s) - 1))))] for q in qs}


def fmt_row(label, values, width=30):
    if not values:
        return f"  {label:<{width}} no samples"
    q = quantiles(values)
    mean = sum(values) / len(values)
    return (f"  {label:<{width}} n={len(values):<7} mean={mean:6.3f}  "
            f"p5={q[5]:6.3f}  p25={q[25]:6.3f}  p50={q[50]:6.3f}  "
            f"p75={q[75]:6.3f}  p95={q[95]:6.3f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", default="results/run.csv")
    args = ap.parse_args()
    if not os.path.exists(args.csv):
        sys.exit(f"missing {args.csv} (run: make run)")

    conf_correct, conf_wrong = [], []
    gap_correct, gap_wrong = [], []
    by_origin = defaultdict(lambda: {"correct": 0, "wrong": 0, "n": 0})
    by_k = defaultdict(lambda: {"correct": 0, "wrong": 0})
    outcomes = defaultdict(int)
    contributors_correct, contributors_wrong = [], []
    rows = 0

    with open(args.csv, newline="") as f:
        for row in csv.DictReader(f):
            rows += 1
            outcomes[row["outcome"]] += 1
            origin = row["origin"]
            by_origin[origin]["n"] += 1

            correct = row["correct"]
            if correct not in ("0", "1"):
                continue   # not scorable: dropped, started a track, or the oracle did not resolve

            is_correct = correct == "1"
            conf = float(row["confidence"])
            gap = float(row["normalised_gap"])
            k = int(row["gated"])
            contrib = int(row["contributors"])

            if is_correct:
                conf_correct.append(conf)
                gap_correct.append(gap)
                contributors_correct.append(contrib)
                by_origin[origin]["correct"] += 1
                by_k[k]["correct"] += 1
            else:
                conf_wrong.append(conf)
                gap_wrong.append(gap)
                contributors_wrong.append(contrib)
                by_origin[origin]["wrong"] += 1
                by_k[k]["wrong"] += 1

    print(f"{args.csv}: {rows} motion-vector rows\n")
    print("outcomes")
    for name in ("fused", "started", "rejected", "dropped"):
        if outcomes.get(name):
            print(f"  {name:<12} {outcomes[name]}")

    scorable = len(conf_correct) + len(conf_wrong)
    if scorable == 0:
        print("\nNothing scorable against the oracle: no accuracy to report.")
        return

    print(f"\n1. does confidence separate correct from wrong associations?  "
          f"({scorable} scorable)")
    print(fmt_row("confidence | correct", conf_correct))
    print(fmt_row("confidence | wrong", conf_wrong))
    mc = sum(conf_correct) / len(conf_correct) if conf_correct else float("nan")
    mw = sum(conf_wrong) / len(conf_wrong) if conf_wrong else float("nan")
    print(f"  separation: {mc - mw:+.3f} in the mean")
    if conf_correct and conf_wrong:
        # Cohen's d: the gap in units of the spread. A gap that is small next to the noise is not a
        # usable signal however large it looks in absolute terms.
        def var(v, m):
            return sum((x - m) ** 2 for x in v) / max(1, len(v) - 1)
        pooled = math.sqrt((var(conf_correct, mc) + var(conf_wrong, mw)) / 2.0)
        if pooled > 0:
            print(f"  standardised separation (Cohen's d): {abs(mc - mw) / pooled:.2f}")

    print(fmt_row("normalised gap | correct", gap_correct))
    print(fmt_row("normalised gap | wrong", gap_wrong))

    # The number the hypothesis is stated against: refuted at AUC ~ 0.5.
    auc, roc = auc_and_roc(conf_correct, conf_wrong)
    print(f"\n  AUC of confidence vs. association correctness: {auc:.4f}")
    if not math.isnan(auc):
        if auc >= 0.8:
            verdict = "strong: confidence is a usable track-quality signal"
        elif auc >= 0.65:
            verdict = "usable: the signal is real but not sharp"
        elif auc >= 0.55:
            verdict = "weak: barely better than guessing"
        else:
            verdict = "REFUTED: confidence does not carry association error"
        print(f"  {verdict}")
    if roc:
        print("\n  ROC, thinned to a readable number of points")
        print(f"    {'FPR':>8} {'TPR':>8}")
        step = max(1, len(roc) // 12)
        for i in range(0, len(roc), step):
            fpr, tpr = roc[i]
            print(f"    {fpr:>8.4f} {tpr:>8.4f}")

    print("\n2. where do wrong associations happen?")
    print(f"  {'candidates in gate (k)':<24} {'correct':>10} {'wrong':>10} {'error rate':>12}")
    for k in sorted(by_k):
        c, w = by_k[k]["correct"], by_k[k]["wrong"]
        rate = 100.0 * w / (c + w) if (c + w) else 0.0
        print(f"  {k:<24} {c:>10} {w:>10} {rate:>11.2f}%")
    print("  k = 1 errors are gating failures; k > 1 errors are assignment failures.")

    print("\n3. does the container matter past the parser?")
    print(f"  {'origin':<10} {'rows':>10} {'correct':>10} {'wrong':>10} {'accuracy':>12}")
    for origin in sorted(by_origin):
        d = by_origin[origin]
        tot = d["correct"] + d["wrong"]
        acc = 100.0 * d["correct"] / tot if tot else float("nan")
        print(f"  {origin:<10} {d['n']:>10} {d['correct']:>10} {d['wrong']:>10} {acc:>11.2f}%")
    print("  The premise holds only if these accuracies are close.")

    print("\n4. rejecting below a confidence threshold")
    print(f"  {'threshold':>10} {'kept':>8} {'wrong kept':>12} {'error rate':>12} "
          f"{'correct lost':>14}")
    for thr in (0.0, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95):
        kept_c = sum(1 for v in conf_correct if v >= thr)
        kept_w = sum(1 for v in conf_wrong if v >= thr)
        kept = kept_c + kept_w
        rate = 100.0 * kept_w / kept if kept else 0.0
        lost = len(conf_correct) - kept_c
        print(f"  {thr:>10.2f} {kept:>8} {kept_w:>12} {rate:>11.2f}% {lost:>14}")
    print("  A threshold is worth having only if the error rate falls faster than the correct"
          " associations\n  it discards.")

    print("\n5. corroboration")
    print(fmt_row("contributors | correct", [float(v) for v in contributors_correct]))
    print(fmt_row("contributors | wrong", [float(v) for v in contributors_wrong]))


if __name__ == "__main__":
    main()
