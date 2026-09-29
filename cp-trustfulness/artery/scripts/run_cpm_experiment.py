#!/usr/bin/env python3
"""
run_cpm_experiment.py
Batch runner for the confidence experiments (CollectiveTrustService).

Every configuration in cpm_batch_config.yaml is merged with `defaults`, translated into OMNeT++ parameter overrides
through PARAMS (unknown keys are an error), and run once per seed inside the `artery` container. Each run writes its
own vehicles.xml and result files under results/cpm_experiments/exp_<timestamp>_<tag>/runs/<id>_s<seed>/.

Metrics (per message, as in Paper 1; honest receivers only, after the warm-up), for CAMs and CPM objects and three
variants computed from the same run:
  p1     Paper 1 alone: flagged when P < 0.5
  ego    + the receiver's radar: flagged when implausible or contradicted (radar-only anchoring)
  full   + independent authors (peers only confirm, so they change anchoring, not what is flagged)
  precision, recall, F1, FPR over messages; `anchored` = share of honest messages anchored.
"""

import os
import sys

# --- VENV AUTO-BOOTSTRAP ---
if sys.prefix == sys.base_prefix:
    venv_python = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".venv", "bin", "python3"))
    if os.path.exists(venv_python):
        os.execv(venv_python, [venv_python] + sys.argv)
# ---------------------------

import argparse
import csv
import hashlib
import json
import math
import platform
import sqlite3
import statistics
import subprocess
import time
import threading
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from typing import Any, Dict, List, Optional

import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../../.."))
ARTERY_DIR = os.path.join(WORKSPACE_DIR, "cp-trustfulness", "artery")
RESULTS_DIR = os.path.join(ARTERY_DIR, "results", "cpm_experiments")
CONTAINER = "artery"
CONTAINER_ARTERY_DIR = "/workspace/cp-trustfulness/artery"

# Keys handled by the runner itself (not OMNeT parameters)
RUNNER_KEYS = {"id", "description", "scenario", "time_limit", "seeds", "faulty_rate", "sensor_setup"}

# Coverage study: Artery sensor file and the matching receiver coverage model (range:aperture:mountYaw:mountForward).
# ITU-R M.2057-1: Radar B (100 m, +-25 deg) front/rear; Radar C (corner class, 100 m, +-30 deg) on the sides.
SENSOR_SETUPS = {
    "front": ("config/sensors_front.xml", "100:50:0:2.5"),
    "front_rear": ("config/sensors.xml", "100:50:0:2.5,100:50:180:-2.5"),
    "surround": ("config/sensors_surround.xml", "100:50:0:2.5,100:50:180:-2.5,100:60:90:0,100:60:270:0"),
}

# YAML key -> (middleware submodule, NED parameter, kind); kind in {"str", "bool", "num", "m", "s"}
PARAMS = {
    # faults of faulty stations (FaultyCar only): VeReMi Extension malfunctions, F2MD parameters
    "gnss_fault_profile":    ("Noise", "gnssFaultMode", "str"),      # const_pos, const_pos_offset, random_pos, random_pos_offset
    "gnss_fault_start_time": ("Noise", "gnssFaultStartTime", "s"),
    "fault_profile":         ("CA", "faultMode", "str"),             # const_speed, const_speed_offset, random_speed, random_speed_offset
    "fault_magnitude":       ("CA", "faultMagnitude", "m"),          # Paper 1's CAM position faults only
    "fault_start_time":      ("CA", "faultStartTime", "s"),
    "cpm_fault_profile":     ("CP", "cpmFaultMode", "str"),          # ghost, object_const_offset, object_random_offset
    "cpm_fault_magnitude":   ("CP", "cpmFaultMagnitude", "m"),
    "cpm_fault_start_time":  ("CP", "cpmFaultStartTime", "s"),
    # error models of every station (cp-trustfulness/include/ErrorModels.h)
    "gnss_error_model":      ("Noise", "gnssErrorModel", "str"),     # veremi | none
    "radar_error_model":     ("Noise", "radarErrorModel", "str"),    # jacobs | none
    # components (ablation)
    "enable_radar":             ("DR", "enableRadar", "bool"),
    "enable_peers":             ("DR", "enablePeers", "bool"),
    "enable_pose_check":        ("DR", "enablePoseCheck", "bool"),
    "enable_admissibility":     ("DR", "enableAdmissibility", "bool"),
    "enable_object_kinematics": ("DR", "enableObjectKinematics", "bool"),
    # Paper 1
    "dr_model":               ("DR", "drAlgorithm", "str"),
    "history_size":           ("DR", "historySize", "num"),
    "max_position_deviation": ("DR", "maxPositionDeviation", "m"),
    # anchoring and states
    "anchor_support":     ("DR", "anchorSupport", "num"),
    "peer_support":       ("DR", "peerSupport", "num"),
    "anchor_penalty":     ("DR", "anchorPenalty", "num"),
    "inclusion_period":   ("DR", "inclusionPeriod", "s"),
    "implausible_below":  ("DR", "implausibleBelow", "num"),
    "contradicted_below": ("DR", "contradictedBelow", "num"),
    "anchored_from":      ("DR", "anchoredFrom", "num"),
    # resolution and gating
    "max_sync_dt":        ("DR", "maxSyncDt", "s"),
    "kappa_gate":         ("DR", "kappaGate", "num"),
    "ego_gnss_error":     ("DR", "egoGnssError", "bool"),
    "building_occlusion": ("DR", "buildingOcclusion", "bool"),
    # metrics
    "metrics_warmup":     ("DR", "metricsWarmup", "s"),
}

KINDS = ("cam", "object")
REASONS = ("physics", "kinematics", "impossible", "pose", "radar")  # which component a flag is owed to
VARIANTS = (("p1", "flagged_p1"), ("ego", "flagged_ego"), ("", "flagged"))


def format_value(value: Any, kind: str) -> str:
    if kind == "str":
        return f'\\"{value}\\"'
    if kind == "bool":
        return "true" if bool(value) else "false"
    if kind == "m":
        return f"{float(value)}m"
    if kind == "s":
        return f"{float(value)}s"
    return str(value)


def omnet_overrides(cfg: Dict[str, Any]) -> List[str]:
    unknown = [k for k in cfg if k not in PARAMS and k not in RUNNER_KEYS]
    if unknown:
        raise ValueError(f"unknown configuration keys: {', '.join(sorted(unknown))}")
    args = []
    for key, value in cfg.items():
        if key in PARAMS and value is not None:
            module, param, kind = PARAMS[key]
            args.append(f"--*.node[*].middleware.{module}.{param}={format_value(value, kind)}")
    return args


def vehicle_mix(cfg: Dict[str, Any]) -> Dict[str, float]:
    faulty = float(cfg.get("faulty_rate", 0.0))
    if not 0.0 <= faulty <= 1.0:
        raise ValueError(f"invalid faulty_rate {faulty}")
    return {"config.neds.HonestCar": 1.0 - faulty, "config.neds.FaultyCar": faulty}


def write_vehicles_xml(path: str, mix: Dict[str, float]):
    lines = ['<?xml version="1.0" encoding="utf-8"?>', "<vehicles>"]
    for vtype, rate in mix.items():
        if rate > 0.0:
            lines.append(f'    <vehicle type="{vtype}" rate="{rate:.4f}" />')
    lines.append("</vehicles>")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def read_scalars(sca_path: str) -> Dict[str, float]:
    conn = sqlite3.connect(sca_path)
    try:
        return {name: float(val) for name, val in
                conn.execute("SELECT scalarName, SUM(scalarValue) FROM scalar GROUP BY scalarName")}
    finally:
        conn.close()


def ratio(num: float, den: float) -> float:
    return num / den if den > 0 else float("nan")


def compute_metrics(s: Dict[str, float]) -> Dict[str, float]:
    """Per-message confusion for CAMs and objects, for each variant (see the module docstring)."""
    g = lambda k: s.get(k, 0.0)
    m: Dict[str, float] = {}
    for kind in KINDS:
        n_honest, n_false = g(f"cl_{kind}_honest_n"), g(f"cl_{kind}_falsified_n")
        m[f"{kind}_honest_n"], m[f"{kind}_falsified_n"] = n_honest, n_false
        for suffix, counter in VARIANTS:
            tp, fp = g(f"cl_{kind}_falsified_{counter}"), g(f"cl_{kind}_honest_{counter}")
            precision, recall = ratio(tp, tp + fp), ratio(tp, n_false)
            tag = f"_{suffix}" if suffix else ""
            m[f"{kind}_recall{tag}"] = recall
            m[f"{kind}_precision{tag}"] = precision
            m[f"{kind}_fpr{tag}"] = ratio(fp, n_honest)
            m[f"{kind}_f1{tag}"] = (2 * precision * recall / (precision + recall)
                                    if precision + recall > 0 else float("nan"))
        m[f"{kind}_anchored_ego"] = ratio(g(f"cl_{kind}_honest_anchored_ego"), n_honest)
        m[f"{kind}_anchored"] = ratio(g(f"cl_{kind}_honest_anchored"), n_honest)
        m[f"{kind}_falsified_contradicted"] = ratio(g(f"cl_{kind}_falsified_contradicted"), n_false)
        # coverage study: how often the receiver's radar could judge, and what it said when it could
        for label, n in (("falsified", n_false), ("honest", n_honest)):
            sup, con = g(f"cl_{kind}_{label}_ego_supported"), g(f"cl_{kind}_{label}_ego_contradicted")
            m[f"{kind}_{label}_judged"] = ratio(sup + con, n)
            m[f"{kind}_{label}_contradicted_when_judged"] = ratio(con, sup + con)
        m[f"{kind}_peer_confirmed"] = ratio(g(f"cl_{kind}_honest_peer_confirmed"), n_honest)
        for label in ("falsified", "honest"):
            flagged = g(f"cl_{kind}_{label}_flagged")
            for reason in REASONS:
                m[f"{kind}_{label}_by_{reason}"] = ratio(g(f"cl_{kind}_{label}_reason_{reason}"), flagged)
    assoc_all = g("assoc_tp") + g("assoc_fp") + g("assoc_fn") + g("assoc_tn")
    m["assoc_resolved_share"] = ratio(g("assoc_tp") + g("assoc_fp"), assoc_all)
    m["assoc_precision"] = ratio(g("assoc_tp"), g("assoc_tp") + g("assoc_fp"))
    m["assoc_recall"] = ratio(g("assoc_tp"), g("assoc_tp") + g("assoc_fn"))
    m["cpms_received"] = g("cpms_received")
    m["invalid_cpms"] = g("fault_invalid_cpms") + g("noise_invalid_cpms")
    return m


def execute_run(cfg: Dict[str, Any], seed: int, run_dir: str, dry_run: bool) -> Optional[Dict[str, float]]:
    rel_dir = os.path.relpath(run_dir, ARTERY_DIR)
    sca = os.path.join(run_dir, "results.sca")
    if not dry_run:
        os.makedirs(run_dir, exist_ok=True)
        write_vehicles_xml(os.path.join(run_dir, "vehicles.xml"), vehicle_mix(cfg))
        for stale in (sca, os.path.join(run_dir, "results.vec")):
            if os.path.exists(stale):
                os.remove(stale)

    overrides = [
        f"--seed-set={seed}",
        # SUMO draws its own randomness (speed factors, driver imperfection): vary it with the seed, otherwise
        # every "seed" replays the same traffic.
        f"--*.traci.launcher.seed={1000 + seed}",
        f"--sim-time-limit={cfg['time_limit']}",
        "--cmdenv-express-mode=true",
        f'--*.traci.mapper.vehicleTypes=xmldoc(\\"{rel_dir}/vehicles.xml\\")',
        f"--output-scalar-file={rel_dir}/results.sca",
        f"--output-vector-file={rel_dir}/results.vec",
        "--**.vector-recording=false",  # Artery's generic vectors are large and unused here
        f'--*.node[*].middleware.DR.claimRecordFile=\\"{rel_dir}/claims.csv\\"',
    ] + omnet_overrides(cfg)
    if cfg.get("sensor_setup"):
        sensors, coverage = SENSOR_SETUPS[cfg["sensor_setup"]]
        overrides += [f'--*.node[*].environmentModel.sensors=xmldoc(\\"{sensors}\\")',
                      f'--*.node[*].middleware.Noise.sensorCoverage=\\"{coverage}\\"']

    script = f"""
        cd {CONTAINER_ARTERY_DIR} || exit 1
        INI_LIBS=$(grep '^libraries =' /workspace/build/artery/run-artery.ini | cut -d'=' -f2- || true)
        INI_NED=$(grep '^nedFolders =' /workspace/build/artery/run-artery.ini | cut -d'=' -f2- || true):{CONTAINER_ARTERY_DIR}
        LD_LIBRARY_PATH=/workspace/build/artery:/workspace/build/artery/src/artery/envmod:/workspace/build/artery/src/traci:/workspace/build/artery/extern:/workspace/build/cp-trustfulness/artery:$LD_LIBRARY_PATH \\
        opp_run \\
            -l /workspace/build/artery/src/artery/envmod/libartery_envmod.so \\
            $INI_LIBS \\
            -n $INI_NED \\
            -u Cmdenv \\
            -c {cfg['scenario']} \\
            -l /workspace/build/cp-trustfulness/artery/libartery_cp_trustfulness.so \\
            {' '.join(f'"{o}"' for o in overrides)} \\
            omnetpp.ini
    """
    if dry_run:
        print(script)
        return None

    t0 = time.time()
    proc = subprocess.run(["docker", "exec", CONTAINER, "bash", "-c", script],
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    wall = time.time() - t0
    with open(os.path.join(run_dir, "run.log"), "w") as f:
        f.write(proc.stdout)
    if proc.returncode != 0 or not os.path.exists(sca):
        print(f"     [FAIL] exit code {proc.returncode}; see {os.path.join(run_dir, 'run.log')}")
        for line in proc.stdout.strip().split("\n")[-8:]:
            print(f"       {line}")
        return None
    metrics = compute_metrics(read_scalars(sca))
    metrics["wall_time_s"] = wall
    return metrics


# two-sided 95 % Student-t quantiles by degrees of freedom
T95 = [12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179, 2.160, 2.145,
       2.131, 2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048, 2.045,
       2.042]


def t95(df: int) -> float:
    """Two-sided 95 % Student-t quantile (exact table up to df 30)."""
    if df < 1:
        return float("nan")
    return T95[df - 1] if df <= len(T95) else 1.96 + 2.4 / df


def write_csv(rows: List[Dict[str, Any]], path: str):
    keys: List[str] = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def fmt(x: float, pct: bool = True) -> str:
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "–"
    return f"{100 * x:.1f}%" if pct else f"{x:.2f}"


def aggregate(rows: List[Dict[str, Any]], key: str) -> Dict[str, Any]:
    """Mean over seeds with a 95 % Student-t half-width (NaN with a single seed)."""
    values = [r[key] for r in rows if r.get("status") == "ok" and not math.isnan(r.get(key, float("nan")))]
    if not values:
        return {"mean": float("nan"), "ci": float("nan")}
    if len(values) == 1:
        return {"mean": values[0], "ci": float("nan")}
    sd = statistics.stdev(values)
    return {"mean": statistics.fmean(values), "ci": t95(len(values) - 1) * sd / math.sqrt(len(values))}


def write_run_report(run_dir: str, row: Dict[str, Any], cfg: Dict[str, Any]):
    """RUN.md: a readable summary of one simulation."""
    lines = [f"# {row['id']} — seed {row['seed']}", ""]
    status = row.get("status", "?")
    lines += [f"- **status:** {status}", f"- **block:** `{row.get('block', '')}`",
              f"- **wall time:** {row.get('wall_time_s', float('nan')):.0f} s" if "wall_time_s" in row else "- **wall time:** –",
              ""]
    config = ["", "## Configuration", "", "| key | value |", "| :--- | :--- |"]
    for k in sorted(cfg):
        if k != "id":
            config.append(f"| `{k}` | `{cfg[k]}` |")
    sca = os.path.join(run_dir, "results.sca")
    if status != "ok" or not os.path.exists(sca):
        lines += ["No results (see `run.log`)."] + config
        with open(os.path.join(run_dir, "RUN.md"), "w") as f:
            f.write("\n".join(lines) + "\n")
        return
    s = read_scalars(sca)
    g = lambda k: s.get(k, 0.0)
    lines += ["## Messages judged by honest receivers", "",
              "| | honest | falsified | flagged honest (P1 / +radar / +peers) | flagged falsified (P1 / +radar / +peers) "
              "| honest anchored (radar / +peers) |",
              "| :--- | ---: | ---: | :---: | :---: | :---: |"]
    for kind in KINDS:
        h = lambda c: g(f"cl_{kind}_honest_{c}")
        x = lambda c: g(f"cl_{kind}_falsified_{c}")
        lines.append(f"| {kind} | {h('n'):.0f} | {x('n'):.0f} "
                     f"| {h('flagged_p1'):.0f} / {h('flagged_ego'):.0f} / {h('flagged'):.0f} "
                     f"| {x('flagged_p1'):.0f} / {x('flagged_ego'):.0f} / {x('flagged'):.0f} "
                     f"| {fmt(ratio(h('anchored_ego'), h('n')))} / {fmt(ratio(h('anchored'), h('n')))} |")
    lines += ["", "| | F1 P1 | F1 +radar | F1 +peers | FPR +peers |", "| :--- | :---: | :---: | :---: | :---: |"]
    for kind in KINDS:
        lines.append(f"| {kind} | {fmt(row.get(f'{kind}_f1_p1'), False)} | {fmt(row.get(f'{kind}_f1_ego'), False)} "
                     f"| {fmt(row.get(f'{kind}_f1'), False)} | {fmt(row.get(f'{kind}_fpr'))} |")
    lines += ["", "## Association", "", "| metric | value |", "| :--- | :---: |",
              f"| resolved right / wrong | {g('assoc_tp'):.0f} / {g('assoc_fp'):.0f} |",
              f"| unresolved but resolvable / correctly unresolved | {g('assoc_fn'):.0f} / {g('assoc_tn'):.0f} |",
              f"| precision / recall | {fmt(row.get('assoc_precision'))} / {fmt(row.get('assoc_recall'))} |",
              "", "## Traffic", "",
              f"CAMs {g('cams_received'):.0f} · CPMs {g('cpms_received'):.0f} · objects {g('objects_received'):.0f} · "
              f"invalid CPMs {g('fault_invalid_cpms') + g('noise_invalid_cpms'):.0f}"]
    lines += config
    with open(os.path.join(run_dir, "RUN.md"), "w") as f:
        f.write("\n".join(lines) + "\n")


def write_report(rows: List[Dict[str, Any]], path: str, header: Dict[str, str]):
    lines = ["# Confidence Experiments", ""]
    for k, v in header.items():
        lines.append(f"- **{k}:** `{v}`")
    lines += ["", "Per message, honest receivers, after the warm-up; mean over successful seeds ± the 95 % Student-t "
              "half-width. **P1** = Paper 1 alone (P < 0.5); **+radar** = flagged when implausible or contradicted by "
              "the receiver's radar; **+peers** adds confirmations by independent authors (they only change "
              "anchoring). **anchored** = share of honest messages anchored (radar only / with peers).", ""]
    columns = []
    for kind in KINDS:
        columns += [(f"{kind}_f1_p1", f"{kind} F1 P1", False), (f"{kind}_f1_ego", f"{kind} F1 +radar", False),
                    (f"{kind}_f1", f"{kind} F1 +peers", False), (f"{kind}_fpr", f"{kind} FPR", True),
                    (f"{kind}_anchored_ego", f"{kind} anchored (radar)", True),
                    (f"{kind}_anchored", f"{kind} anchored (+peers)", True)]
    columns.append(("assoc_precision", "assoc prec", True))
    blocks: Dict[str, List[Dict[str, Any]]] = {}
    for r in rows:
        blocks.setdefault(r["block"], []).append(r)
    for block, block_rows in blocks.items():
        lines += [f"## {block}", "",
                  "| id | seeds ok | " + " | ".join(c[1] for c in columns) + " |",
                  "| :--- | :---: | " + " | ".join(":---:" for _ in columns) + " |"]
        ids: List[str] = []
        for r in block_rows:
            if r["id"] not in ids:
                ids.append(r["id"])
        for run_id in ids:
            runs = [r for r in block_rows if r["id"] == run_id]
            ok = sum(r["status"] == "ok" for r in runs)
            cells = []
            for key, _, pct in columns:
                a = aggregate(runs, key)
                cell = fmt(a["mean"], pct)
                if not math.isnan(a["ci"]):
                    cell += f" ± {fmt(a['ci'], pct)}"
                cells.append(cell)
            status = f"{ok}/{len(runs)}" + ("" if ok == len(runs) else " **FAILED**")
            lines.append(f"| `{run_id}` | {status} | " + " | ".join(cells) + " |")
        # attribution: which component each flag is owed to (share of the flagged messages, mean over seeds)
        lines += ["", "Why flagged (share of flagged messages, " + " / ".join(REASONS) + "); association:", "",
                  "| id | CAM falsified | CAM honest (false positives) | object falsified | object honest (false positives) "
                  "| objects resolved | assoc precision / recall |",
                  "| :--- | :---: | :---: | :---: | :---: | :---: | :---: |"]
        for run_id in ids:
            runs = [r for r in block_rows if r["id"] == run_id]
            share = lambda kind, label: " / ".join(fmt(aggregate(runs, f"{kind}_{label}_by_{x}")["mean"]) for x in REASONS)
            lines.append(f"| `{run_id}` | {share('cam', 'falsified')} | {share('cam', 'honest')} "
                         f"| {share('object', 'falsified')} | {share('object', 'honest')} "
                         f"| {fmt(aggregate(runs, 'assoc_resolved_share')['mean'])} "
                         f"| {fmt(aggregate(runs, 'assoc_precision')['mean'])} / {fmt(aggregate(runs, 'assoc_recall')['mean'])} |")
        lines.append("")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


class Tee:
    """Mirrors the runner's console output into <experiment>/runner.log."""

    def __init__(self, stream, path: str):
        self.stream = stream
        self.file = open(path, "a", buffering=1)
        self.lock = threading.Lock()

    def write(self, text: str):
        with self.lock:
            self.stream.write(text)
            self.file.write(text)

    def flush(self):
        self.stream.flush()
        self.file.flush()


def git(*args: str) -> str:
    try:
        return subprocess.run(["git", "-C", WORKSPACE_DIR, *args], stdout=subprocess.PIPE,
                              stderr=subprocess.DEVNULL, text=True).stdout
    except OSError:
        return ""


def source_manifest() -> Dict[str, str]:
    """SHA-256 of every tracked or untracked source file of cp-trustfulness (`git diff HEAD` misses untracked
    files)."""
    manifest = {}
    listing = git("ls-files", "--cached", "--others", "--exclude-standard", "cp-trustfulness")
    for rel in sorted(listing.splitlines()):
        if "/results/" in rel or "/tests/build/" in rel or rel.endswith((".pyc", ".sca", ".vec", ".csv")):
            continue
        path = os.path.join(WORKSPACE_DIR, rel)
        if os.path.isfile(path):
            with open(path, "rb") as f:
                manifest[rel] = hashlib.sha256(f.read()).hexdigest()
    return manifest


def file_sha256(path: str) -> Optional[str]:
    if not os.path.exists(path):
        return None
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def write_metadata(exp_dir: str, args, selected: List[str], seeds: List[int], plan):
    """Ties the results to the exact code and configuration (read-only git queries)."""
    diff = git("diff", "HEAD")
    status = git("status", "--porcelain")
    manifest = source_manifest()
    meta = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "command": sys.argv,
        "host": platform.node(),
        "git_commit": git("rev-parse", "HEAD").strip(),
        "git_branch": git("rev-parse", "--abbrev-ref", "HEAD").strip(),
        "uncommitted_files": [line[3:] for line in status.splitlines()],
        "uncommitted_diff_sha256": hashlib.sha256(diff.encode()).hexdigest(),
        "source_tree_sha256": hashlib.sha256(json.dumps(manifest, sort_keys=True).encode()).hexdigest(),
        "source_manifest": manifest,
        "library_sha256": file_sha256(os.path.join(WORKSPACE_DIR, "build", "cp-trustfulness", "artery",
                                                   "libartery_cp_trustfulness.so")),
        "library_mtime": datetime.fromtimestamp(os.path.getmtime(os.path.join(
            WORKSPACE_DIR, "build", "cp-trustfulness", "artery", "libartery_cp_trustfulness.so"))).isoformat(timespec="seconds")
            if os.path.exists(os.path.join(WORKSPACE_DIR, "build", "cp-trustfulness", "artery", "libartery_cp_trustfulness.so")) else None,
        "config_file": os.path.relpath(args.config, ARTERY_DIR),
        "blocks": selected,
        "seeds": seeds,
        "runs": [{"block": b, **m} for b, m in plan],
    }
    with open(os.path.join(exp_dir, "metadata.json"), "w") as f:
        json.dump(meta, f, indent=2)
    with open(os.path.join(exp_dir, "uncommitted.diff"), "w") as f:
        f.write(diff)


def main():
    parser = argparse.ArgumentParser(description="Confidence experiment runner")
    parser.add_argument("--config", default=os.path.join(ARTERY_DIR, "cpm_batch_config.yaml"))
    parser.add_argument("--block", action="append", help="block(s) to run (default: every block)")
    parser.add_argument("--id", help="run only this configuration id")
    parser.add_argument("--seeds", type=int, help="number of seeds (1..N), overrides defaults.seeds")
    parser.add_argument("--seed-list", help="explicit seeds, e.g. 2 or 2,3 (overrides --seeds)")
    parser.add_argument("--time-limit", help="override simulation time limit, e.g. 60s")
    parser.add_argument("--scenario", help="override scenario (omnetpp.ini config), e.g. grid")
    parser.add_argument("--tag", help="suffix of the experiment directory")
    parser.add_argument("--dry-run", action="store_true", help="print the commands without running")
    parser.add_argument("--jobs", type=int, default=1,
                        help="simulations run in parallel (each needs one core and ~0.5 GB on LuST)")
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    defaults = cfg.get("defaults", {})
    blocks = [k for k, v in cfg.items() if k != "defaults" and isinstance(v, list)]
    selected = args.block or blocks
    for b in selected:
        if b not in cfg:
            sys.exit(f"[FATAL] block '{b}' not in {args.config}; available: {', '.join(blocks)}")

    seeds = list(range(1, args.seeds + 1)) if args.seeds else list(defaults.get("seeds", [1]))
    if args.seed_list:
        seeds = [int(x) for x in args.seed_list.split(",")]
    tag = args.tag or args.id or ("all_blocks" if selected == blocks else "_".join(selected))
    exp_name = f"exp_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{tag}"
    exp_dir = os.path.join(RESULTS_DIR, exp_name)
    if not args.dry_run:
        os.makedirs(exp_dir, exist_ok=True)

    plan = []
    for block in selected:
        for run_cfg in cfg[block]:
            if args.id and run_cfg.get("id") != args.id:
                continue
            merged = {**defaults, **run_cfg}
            merged.pop("seeds", None)
            if args.time_limit:
                merged["time_limit"] = args.time_limit
            if args.scenario:
                merged["scenario"] = args.scenario
            omnet_overrides(merged)  # validate keys before starting anything
            plan.append((block, merged))
    if not plan:
        sys.exit("[FATAL] nothing to run")

    if not args.dry_run:
        sys.stdout = Tee(sys.stdout, os.path.join(exp_dir, "runner.log"))
        write_metadata(exp_dir, args, selected, seeds, plan)
        latest = os.path.join(RESULTS_DIR, "latest")  # points to the running experiment from the start
        if os.path.islink(latest):
            os.remove(latest)
        if not os.path.exists(latest):
            os.symlink(exp_name, latest)
    print(f"Experiment {exp_name}: {len(plan)} configurations x {len(seeds)} seeds, {args.jobs} parallel jobs")
    tasks = [(block, merged, seed) for block, merged in plan for seed in seeds]

    def run_task(task):
        block, merged, seed = task
        run_id = merged["id"]
        print(f"  -> [{block}] {run_id} seed {seed} started", flush=True)
        metrics = execute_run(merged, seed, os.path.join(exp_dir, "runs", f"{run_id}_s{seed}"), args.dry_run)
        if metrics:
            print(f"     [{run_id} s{seed}] CAM F1 {fmt(metrics['cam_f1_p1'], False)} -> {fmt(metrics['cam_f1'], False)}  "
                  f"object F1 {fmt(metrics['object_f1_p1'], False)} -> {fmt(metrics['object_f1'], False)}  "
                  f"honest CAMs anchored {fmt(metrics['cam_anchored'])}  ({metrics['wall_time_s']:.0f}s)", flush=True)
        return {"block": block, "id": run_id, "seed": seed, "status": "ok" if metrics else "failed",
                "run_dir": os.path.join(exp_dir, "runs", f"{run_id}_s{seed}"),
                **{k: v for k, v in merged.items() if k != "id"}, **(metrics or {})}

    lock = threading.Lock()
    finished: Dict[int, Dict[str, Any]] = {}
    header = {"experiment": exp_name, "config": os.path.relpath(args.config, ARTERY_DIR),
              "blocks": ", ".join(selected), "seeds": ", ".join(map(str, seeds))}

    def run_and_report(indexed):
        index, task = indexed
        row = run_task(task)
        if args.dry_run:
            return row
        write_run_report(row["run_dir"], row, task[1])
        with lock:  # REPORT.md and the CSV grow after every finished run
            finished[index] = row
            done = [finished[i] for i in sorted(finished)]
            write_report(done, os.path.join(exp_dir, "REPORT.md"),
                         {**header, "progress": f"{len(done)}/{len(tasks)} runs finished"})
            write_csv(done, os.path.join(exp_dir, "cpm_results.csv"))
        return row

    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        rows: List[Dict[str, Any]] = list(pool.map(run_and_report, enumerate(tasks)))  # keeps plan order
    if args.dry_run:
        return

    write_csv(rows, os.path.join(exp_dir, "cpm_results.csv"))
    write_report(rows, os.path.join(exp_dir, "REPORT.md"), {**header, "progress": f"complete ({len(rows)} runs)"})

    latest = os.path.join(RESULTS_DIR, "latest")
    if os.path.islink(latest):
        os.remove(latest)
    if not os.path.exists(latest):
        os.symlink(exp_name, latest)
    failed = sum(r["status"] != "ok" for r in rows)
    print(f"\nResults: {exp_dir}/REPORT.md ({failed} failed runs)")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
