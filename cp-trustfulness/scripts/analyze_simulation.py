#!/usr/bin/env python3
"""
analyze_simulation.py
Analysis engine for V2X Trust Evaluation simulation results.
"""

import sqlite3
import sys
import os
import math

FLAGGED_THRESHOLD = 0.5

def parse_ground_truth(sca_path):
    """Parses ground truth from scalar file and returns sets and mappings."""
    faulty_nodes = set()
    honest_nodes = set()
    st_map = {}
    fault_map = {}
    sumo_map = {}
    ground_truth_available = False

    if os.path.exists(sca_path):
        try:
            sca_conn = sqlite3.connect(sca_path)
            sca_cur = sca_conn.cursor()
            for mod, sname, val in sca_cur.execute(
                'SELECT moduleName, scalarName, scalarValue FROM scalar '
                'WHERE scalarName IN ("stationID", "isFaulty", "sumoVehicleID")'
            ):
                node_key = mod.split('.')[1] if '.' in mod else mod
                if sname == 'stationID':
                    st_map[node_key] = str(int(val))
                elif sname == 'isFaulty':
                    fault_map[node_key] = int(val)
                elif sname == 'sumoVehicleID':
                    sumo_map[node_key] = str(int(val))
                    
            for node_key, st_id in st_map.items():
                if fault_map.get(node_key, 0) == 1:
                    faulty_nodes.add(st_id)
                else:
                    honest_nodes.add(st_id)
            ground_truth_available = len(st_map) > 0
            sca_conn.close()
        except Exception as e:
            print(f"[WARNING] Could not read ground truth scalars: {e}")
            pass

    return faulty_nodes, honest_nodes, ground_truth_available, st_map, sumo_map


def parse_vector_data(vec_path):
    """Parses TrustScore vectors from the database."""
    conn = sqlite3.connect(vec_path)
    cur = conn.cursor()
    try:
        rows = cur.execute("""
            SELECT vector.vectorName, vectorData.simtimeRaw * 1e-12 AS simtime, vectorData.value
            FROM vectorData JOIN vector USING (vectorId)
            WHERE vectorName LIKE 'TrustScore%'
            ORDER BY vectorData.simtimeRaw ASC
        """).fetchall()
    except Exception as e:
        print(f"[ERROR] Failed to query vector database: {e}")
        rows = []
    finally:
        conn.close()
    return rows


def analyze_trust_statistics(rows):
    """Aggregates node counts, trust scores, and min trust values."""
    nodes_evaluated = set()
    trust_scores = []
    node_min_trust = {}
    node_eval_count = {}

    for name, simtime, val in rows:
        station_id = name.replace("TrustScore_Node_", "")
        nodes_evaluated.add(station_id)
        trust_scores.append(val)
        node_eval_count[station_id] = node_eval_count.get(station_id, 0) + 1
        if station_id not in node_min_trust or val < node_min_trust[station_id]:
            node_min_trust[station_id] = val

    total_evals = len(rows)
    mean_trust = sum(trust_scores) / total_evals if total_evals > 0 else 0.0
    if total_evals > 0:
        std_trust = math.sqrt(sum((x - mean_trust) ** 2 for x in trust_scores) / total_evals)
    else:
        std_trust = 0.0

    return nodes_evaluated, trust_scores, node_min_trust, node_eval_count, mean_trust, std_trust


def analyze_confusion_matrix(node_min_trust, nodes_evaluated, faulty_nodes, honest_nodes):
    """Calculates classification metrics based on ground truth mappings."""
    flagged_nodes = {node for node, min_t in node_min_trust.items() if min_t < FLAGGED_THRESHOLD}
    active_faulty = faulty_nodes.intersection(nodes_evaluated)
    active_honest = honest_nodes.intersection(nodes_evaluated)

    tp_set = flagged_nodes.intersection(active_faulty)
    fn_set = active_faulty - flagged_nodes
    fp_set = flagged_nodes.intersection(active_honest)
    tn_set = active_honest - flagged_nodes

    tp = len(tp_set)
    fn = len(fn_set)
    fp = len(fp_set)
    tn = len(tn_set)

    recall = (tp / len(active_faulty) * 100) if active_faulty else 100.0
    fpr = (fp / len(active_honest) * 100) if active_honest else 0.0
    precision = (tp / (tp + fp) * 100) if (tp + fp) > 0 else 0.0
    accuracy = ((tp + tn) / len(nodes_evaluated) * 100) if nodes_evaluated else 100.0
    f1 = (2 * precision * recall / (precision + recall)) if (precision + recall) > 0 else 0.0

    return {
        "tp": tp, "fn": fn, "fp": fp, "tn": tn,
        "recall": recall, "fpr": fpr, "precision": precision,
        "accuracy": accuracy, "f1_score": f1,
        "flagged_nodes": flagged_nodes,
        "active_faulty": active_faulty,
        "active_honest": active_honest,
        "tp_set": tp_set, "fn_set": fn_set, "fp_set": fp_set, "tn_set": tn_set
    }


def analyze_simulation(vec_path):
    """Master engine that parses data, analyzes stats, and returns a unified metrics dict."""
    if not os.path.exists(vec_path):
        return {"error": f"Vector database not found: {vec_path}"}

    sca_path = vec_path.replace(".vec", ".sca")
    
    # Parse Data
    faulty_nodes, honest_nodes, gt_avail, st_map, sumo_map = parse_ground_truth(sca_path)
    rows = parse_vector_data(vec_path)
    
    if not rows:
        return {"error": "No TrustScore vectors found in database"}

    # Analyze Statistics
    nodes_eval, t_scores, node_min_trust, node_eval_count, mean_trust, std_trust = analyze_trust_statistics(rows)
    
    # Analyze Confusion Matrix
    cm = analyze_confusion_matrix(node_min_trust, nodes_eval, faulty_nodes, honest_nodes)

    # Per-node details mapping
    per_node = {}
    for station_id, min_t in node_min_trust.items():
        role = "faulty" if station_id in faulty_nodes else "honest" if station_id in honest_nodes else "unknown"
        flagged = min_t < FLAGGED_THRESHOLD
        if role == "faulty":
            classification = "TP" if flagged else "FN"
        elif role == "honest":
            classification = "FP" if flagged else "TN"
        else:
            classification = "flagged" if flagged else "ok"
            
        per_node[station_id] = {
            "role": role,
            "min_trust": round(min_t, 4),
            "eval_count": node_eval_count.get(station_id, 0),
            "classification": classification,
        }

    return {
        "vec_path": vec_path,
        "sca_path": sca_path,
        "total_evaluations": len(rows),
        "total_nodes": len(nodes_eval),
        "faulty_nodes": len(cm["active_faulty"]),
        "honest_nodes": len(cm["active_honest"]),
        "mean_trust": round(mean_trust, 4),
        "std_trust": round(std_trust, 4),
        "trust_scores": t_scores,
        "tp": cm["tp"], "fn": cm["fn"], "fp": cm["fp"], "tn": cm["tn"],
        "recall": round(cm["recall"], 2),
        "fpr": round(cm["fpr"], 2),
        "precision": round(cm["precision"], 2),
        "accuracy": round(cm["accuracy"], 2),
        "f1_score": round(cm["f1_score"], 4),
        "ground_truth_available": gt_avail,
        "st_map": st_map,
        "sumo_map": sumo_map,
        "node_min_trust": node_min_trust,
        "cm": cm,
        "per_node": per_node,
    }


def print_efficacy_report(metrics):
    """Prints human-readable terminal report."""
    if "error" in metrics:
        print(f"[ERROR] {metrics['error']}")
        return

    print("=" * 70)
    print("      V2X TRUST EVALUATION & FAULTY DETECTION EFFICACY REPORT      ")
    print("=" * 70)
    print(f"Vector Database (Predictions) : {metrics['vec_path']}")
    print(f"Scalar Database (Ground Truth): {metrics['sca_path'] if metrics['ground_truth_available'] else 'Not found'}")
    print("-" * 70)

    print("GENERAL CAM TRAFFIC METRICS")
    print(f"  Total CAM Messages Evaluated : {metrics['total_evaluations']:,}")
    print(f"  Monitored Unique Nodes      : {metrics['total_nodes']}")
    print(f"  Mean Trust Score            : {metrics['mean_trust']:.4f} (StdDev: {metrics['std_trust']:.4f})")
    print(f"  Min / Max Trust Score       : {min(metrics['trust_scores']):.4f} / {max(metrics['trust_scores']):.4f}")
    print("-" * 70)

    if metrics["ground_truth_available"]:
        cm = metrics["cm"]
        print("GROUND-TRUTH EFFICACY & CONFUSION MATRIX")
        print(f"  Total Faulty / Malicious Nodes Injected : {metrics['faulty_nodes']}")
        print(f"  Total Honest Nodes Active                 : {metrics['honest_nodes']}")
        print("  " + "-" * 50)
        print(f"  [TP] True Positives  (Faulty Correctly Flagged) : {metrics['tp']} / {metrics['faulty_nodes']}")
        print(f"  [FN] False Negatives (Faulty Missed)            : {metrics['fn']}")
        print(f"  [FP] False Positives (Honest Falsely Flagged)      : {metrics['fp']} / {metrics['honest_nodes']}")
        print(f"  [TN] True Negatives  (Honest Correctly Trusted)    : {metrics['tn']}")
        print("  " + "-" * 50)
        print(f"  Faulty Detection Rate (Recall) : {metrics['recall']:.2f}%")
        print(f"  False Positive Rate (FPR)       : {metrics['fpr']:.2f}%")
        print(f"  Precision                       : {metrics['precision']:.2f}%")
        print(f"  Overall Classification Accuracy : {metrics['accuracy']:.2f}%")
        print(f"  F1 Score                        : {metrics['f1_score']:.4f}")
    else:
        print("CLASSIFICATION BREAKDOWN (No Ground-Truth Scalar File Found)")
        print(f"  Total Flagged Nodes (< {FLAGGED_THRESHOLD}) : {len(metrics['cm']['flagged_nodes'])} / {metrics['total_nodes']}")

    print("-" * 120)
    print("DETAILED PER-NODE EVALUATION BREAKDOWN (Ground Truth Mapping)")
    print(f"  {'Station ID':<12} | {'SUMO ID (Locate)':<18} | {'OMNeT Module':<18} | {'Role':<12} | {'Min Trust':<10} | {'Classification':<15}")
    print("  " + "-" * 116)

    st_to_mod = {st_id: node_key for node_key, st_id in metrics["st_map"].items()} if metrics["ground_truth_available"] else {}
    
    sorted_nodes = sorted(metrics["node_min_trust"].items(), key=lambda x: x[1])
    for station_id, min_t in sorted_nodes:
        omnet_mod = st_to_mod.get(station_id, "Unknown")
        sumo_id = metrics["sumo_map"].get(omnet_mod, omnet_mod.replace("node[", "").replace("]", "") if "node[" in omnet_mod else "N/A")

        if metrics["ground_truth_available"]:
            cm = metrics["cm"]
            role = "FAULTY" if station_id in cm["active_faulty"] else "HONEST"
            if station_id in cm["tp_set"]:
                result = "TRUE POSITIVE"
            elif station_id in cm["fp_set"]:
                result = "FALSE POSITIVE"
            elif station_id in cm["fn_set"]:
                result = "FALSE NEGATIVE"
            else:
                result = "TRUE NEGATIVE"
        else:
            role = "UNKNOWN"
            result = "FLAGGED" if min_t < FLAGGED_THRESHOLD else "OK"

        # Highlight anomalies first
        if min_t < FLAGGED_THRESHOLD or role == "FAULTY":
            print(f"  {station_id:<12} | {sumo_id:<18} | {omnet_mod:<18} | {role:<12} | {min_t:<10.4f} | {result:<15}")

    print("=" * 120)


if __name__ == '__main__':
    db_file = sys.argv[1] if len(sys.argv) > 1 else "results/grid-#0.vec"
    metrics_result = analyze_simulation(db_file)
    print_efficacy_report(metrics_result)
