#!/usr/bin/env python3
"""
experiment.py
Unified Experiment Orchestrator for V2X Trust Evaluation Simulations.

Single entry point for: simulation execution -> result collection -> analysis -> archival.

"""

import subprocess
import argparse
import sys
import os
import json
import shutil
from datetime import datetime
from io import StringIO
from analyze_simulation import analyze_simulation, print_efficacy_report

# Resolve paths relative to this script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../.."))
ARTERY_DIR = os.path.join(WORKSPACE_DIR, "artery")
RESULTS_BASE = os.path.join(SCRIPT_DIR, "../results")
EXPERIMENTS_DIR = os.path.join(RESULTS_BASE, "experiments")

sys.path.insert(0, SCRIPT_DIR)

DR_MODELS = ["CTRA", "HistoryAwareCTRA"]
TRUST_MODELS = ["BetaReputation"]


def create_experiment_dir(map_config, dr_alg, trust_alg, name=None):
    """Create a timestamped experiment directory and return its path."""
    os.makedirs(EXPERIMENTS_DIR, exist_ok=True)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M")
    if name:
        dirname = f"{timestamp}_{name}"
    else:
        dirname = f"{timestamp}_{map_config}_{dr_alg}_{trust_alg}"

    experiment_path = os.path.join(EXPERIMENTS_DIR, dirname)
    os.makedirs(experiment_path, exist_ok=True)
    return experiment_path


def update_faulty_rate(rate):
    """Dynamically update vehicles.xml with requested FaultyCar ratio."""
    xml_path = os.path.join(WORKSPACE_DIR, "cp-trustfulness", "config", "vehicles.xml")
    honest_rate = max(0.0, 1.0 - rate)

    content = f"""<?xml version="1.0" encoding="utf-8"?>
<vehicles>
    <vehicle type="src.modules.HonestCar" rate="{honest_rate:.2f}" />
    <vehicle type="src.modules.FaultyCar" rate="{rate:.2f}" />
</vehicles>
"""

    with open(xml_path, "w") as f:
        f.write(content)


def get_sumo_begin_time(map_config):
    """Parse the SUMO <begin> value from the .sumocfg referenced in omnetpp.ini."""
    ini_path = os.path.join(WORKSPACE_DIR, "cp-trustfulness", "omnetpp.ini")
    sumocfg_rel = None

    if not os.path.exists(ini_path):
        return 0

    in_target_config = False
    with open(ini_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('[Config '):
                config_name = line.replace('[Config ', '').replace(']', '').strip()
                in_target_config = (config_name == map_config)

            if in_target_config and line.startswith('*.traci.launcher.sumocfg'):
                val = line.split('=')[1].split('#')[0].strip().strip('"')
                sumocfg_rel = val
                break

    if not sumocfg_rel:
        return 0

    # Resolve path relative to cp-trustfulness/
    sumocfg_path = os.path.normpath(os.path.join(WORKSPACE_DIR, "cp-trustfulness", sumocfg_rel))
    if not os.path.exists(sumocfg_path):
        return 0

    # Parse <begin value="..."/> from the XML
    import re
    with open(sumocfg_path, 'r') as f:
        content = f.read()
    match = re.search(r'<begin\s+value="(\d+)"', content)
    if match:
        return int(match.group(1))
    return 0


def get_start_time_from_ini(map_config):
    """Parse omnetpp.ini to find *.traci.core.startTime for the given map config."""
    ini_path = os.path.join(WORKSPACE_DIR, "cp-trustfulness", "omnetpp.ini")
    start_time = 0
    if not os.path.exists(ini_path):
        return start_time

    in_target_config = False
    with open(ini_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('[Config '):
                config_name = line.replace('[Config ', '').replace(']', '').strip()
                in_target_config = (config_name == map_config)
            
            if in_target_config and line.startswith('*.traci.core.startTime'):
                val_str = line.split('=')[1].split('#')[0].strip()
                if val_str.endswith('s'):
                    val_str = val_str[:-1]
                try:
                    start_time = int(val_str)
                except ValueError:
                    pass
                break
                
    return start_time

def run_simulation(map_config, dr_alg, trust_alg, time_limit=None, mode="headless", cam_freq=None, fault_profile=None, faulty_cam_freq=None, seed=None, extra_args=None):
    """Execute the simulation directly via opp_run (with or without Docker) and return success status."""

    # 1. Check if Docker is running
    use_docker = False
    try:
        res = subprocess.run(["docker", "ps", "--format", "{{.Names}}"], capture_output=True, text=True)
        if "artery" in res.stdout.split():
            use_docker = True
    except:
        pass

    if use_docker:
        build_dir = "/workspace/build"
        scenario_dir = "/workspace/cp-trustfulness"
        host_wrapper = ["docker", "exec", "artery", "bash", "-c"]
    else:
        build_dir = os.path.join(WORKSPACE_DIR, "build")
        scenario_dir = os.path.join(WORKSPACE_DIR, "cp-trustfulness")
        host_wrapper = ["bash", "-c"]

    # 2. Construct EXTRA_ARGS
    extra = []
    if dr_alg: extra.append(f'--*.node[*].middleware.DR.drAlgorithm=\\"{dr_alg}\\"')
    if trust_alg: extra.append(f'--*.node[*].middleware.DR.trustAlgorithm=\\"{trust_alg}\\"')
    if cam_freq is not None:
        interval = 1.0 / float(cam_freq)
        extra.append('--*.node[*].middleware.CA.fixedRate=true')
        extra.append(f'--*.node[*].middleware.CA.minInterval={interval:.3f}s')
    if faulty_cam_freq is not None:
        interval = 1.0 / float(faulty_cam_freq)
        extra.append('--**.FaultyCar.middleware.CA.fixedRate=true')
        extra.append(f'--**.FaultyCar.middleware.CA.minInterval={interval:.3f}s')
    if fault_profile:
        extra.append(f'--*.node[*].middleware.CA.faultMode=\\"{fault_profile}\\"')
    if seed is not None:
        extra.append(f'--seed-set={seed}')
    if extra_args:
        extra.extend(extra_args)
        
    extra_str = " ".join(extra)

    # 3. Construct Time Limit
    if time_limit:
        start_time_offset = get_start_time_from_ini(map_config)
        if time_limit.endswith('s'):
            t = int(time_limit[:-1])
            time_limit = f"{start_time_offset + t}s"
        else:
            try:
                time_limit = f"{start_time_offset + int(time_limit)}s"
            except:
                pass

    # 4. Construct internal Bash script
    u_flag = "Cmdenv"
    c_flag = map_config
    if mode == "qtenv":
        u_flag = "Qtenv"
    elif mode == "gui":
        c_flag = f"{map_config}-gui"

    script = f"""
        cd {scenario_dir} || exit 1
        INI_LIBS=$(grep '^libraries =' {build_dir}/artery/run-artery.ini | cut -d'=' -f2- || true)
        INI_NED=$(grep '^nedFolders =' {build_dir}/artery/run-artery.ini | cut -d'=' -f2- || true):{scenario_dir}

        export XDG_RUNTIME_DIR=/tmp/runtime-devcontainer
        export QT_X11_NO_MITSHM=1
        export LIBGL_ALWAYS_SOFTWARE=1
        export MESA_GL_VERSION_OVERRIDE=3.3
        export LD_LIBRARY_PATH={build_dir}/artery:{build_dir}/artery/src/artery/envmod:{build_dir}/artery/src/traci:{build_dir}/artery/extern:{build_dir}/cp-trustfulness:$LD_LIBRARY_PATH
        mkdir -p /tmp/runtime-devcontainer

        DISPLAY=$DISPLAY opp_run \
            -l {build_dir}/artery/src/artery/envmod/libartery_envmod.so \
            $INI_LIBS \
            -n $INI_NED \
            -u {u_flag} \
            -c {c_flag} \
            -l {build_dir}/cp-trustfulness/libartery_cp_trustfulness.so \
    """

    if time_limit:
        script += f"    --sim-time-limit={time_limit} \\\n"
        
    script += f"    {extra_str} \\\n    omnetpp.ini\n"

    cmd = host_wrapper + [script]

    print(f"  [RUN] Executing via {'Docker' if use_docker else 'Host'}...")

    if mode != "headless" and use_docker:
        subprocess.run("xhost +local:docker >/dev/null 2>&1 || true", shell=True)

    if mode == "headless":
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        if proc.returncode != 0:
            print(f"  [FAIL] Simulation failed (exit code {proc.returncode})")
            lines = proc.stdout.strip().split('\\n')
            for line in lines[-10:]:
                print(f"    {line}")
            return False
    else:
        env = os.environ.copy()
        proc = subprocess.run(cmd, env=env)
        if proc.returncode != 0:
            print(f"  [FAIL] Simulation failed (exit code {proc.returncode})")
            return False

    return True


def collect_results(map_config, experiment_path):
    """Copy simulation output files (.vec, .sca, logs) into the experiment directory."""
    collected = []
    
    sqlite_dir = os.path.join(experiment_path, "sqlite")
    os.makedirs(sqlite_dir, exist_ok=True)

    for suffix in [".vec", ".sca"]:
        src = os.path.join(RESULTS_BASE, "sqlite", f"{map_config}-#0{suffix}")
        if os.path.exists(src):
            dst = os.path.join(sqlite_dir, f"{map_config}-#0{suffix}")
            shutil.copy2(src, dst)
            collected.append(os.path.join("sqlite", os.path.basename(src)))

    # Copy SUMO log if present
    sumo_log = os.path.join(RESULTS_BASE, "sumo-0.log")
    if os.path.exists(sumo_log):
        shutil.copy2(sumo_log, os.path.join(experiment_path, "sumo-0.log"))
        collected.append("sumo-0.log")

    return collected


def run_analysis(experiment_path, map_config):
    """Run the canonical analysis engine on collected results.
    
    Returns:
        metrics dict, or None on failure
    """
    vec_path = os.path.join(experiment_path, "sqlite", f"{map_config}-#0.vec")

    if not os.path.exists(vec_path):
        print(f"  [FAIL] No vector database found at: {vec_path}")
        return None

    # 1. Machine readable metrics
    metrics = analyze_simulation(vec_path)
    if "error" in metrics:
        print(f"  [FAIL] Analysis error: {metrics['error']}")
        return None
        
    # Clean up massive objects before returning to external callers
    clean_metrics = {k: v for k, v in metrics.items() if k not in ["trust_scores", "cm", "node_min_trust", "st_map", "sumo_map", "vec_path", "sca_path"]}

    # 2. Human readable report (capture stdout)
    old_stdout = sys.stdout
    sys.stdout = report_buffer = StringIO()
    print_efficacy_report(metrics)
    sys.stdout = old_stdout
    report_text = report_buffer.getvalue()

    # Save report
    report_path = os.path.join(experiment_path, "analysis_report.txt")
    with open(report_path, "w") as f:
        f.write(report_text)

    # Print it live too
    print(report_text)

    return clean_metrics


def save_experiment(experiment_path, config, metrics):
    """Save experiment configuration and metrics as JSON files."""
    with open(os.path.join(experiment_path, "config.json"), "w") as f:
        json.dump(config, f, indent=2)

    with open(os.path.join(experiment_path, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)


def update_latest_link(experiment_path):
    """Update the results/latest symlink to point to this experiment."""
    latest_link = os.path.join(RESULTS_BASE, "latest")
    if os.path.islink(latest_link):
        os.unlink(latest_link)
    elif os.path.exists(latest_link):
        os.remove(latest_link)
    os.symlink(experiment_path, latest_link)


def print_summary_table(results_list):
    """Print a formatted comparison table for sweep results."""
    lines = []
    lines.append("")
    lines.append("=" * 115)
    lines.append("                    AUTOMATED STRATEGY BENCHMARK & INFERENCE COMPARISON REPORT")
    lines.append("=" * 115)
    lines.append(f" {'DR Model':<10} | {'Trust Model':<15} | {'Recall':<8} | {'Precision':<10} | {'FPR':<8} | {'Accuracy':<10} | {'F1 Score':<8}")
    lines.append(" " + "-" * 98)

    for r in results_list:
        lines.append(
            f" {r['dr_algorithm']:<10} | {r['trust_algorithm']:<15} | "
            f"{r['recall']:<7.1f}% | {r['precision']:<9.1f}% | "
            f"{r['fpr']:<7.1f}% | {r['accuracy']:<9.1f}% | "
            f"{r['f1_score']:<8.4f}"
        )

    lines.append("=" * 115)
    table_str = "\n".join(lines)
    print(table_str)
    return table_str


def run_single_experiment(map_config, dr_alg, trust_alg, time_limit=None, name=None, mode="headless", cam_freq=None, fault_profile=None, faulty_rate=None, faulty_cam_freq=None, seed=None, extra_args=None):
    """Execute a complete experiment: simulate -> collect -> analyze -> archive."""
    if faulty_rate is not None:
        update_faulty_rate(faulty_rate)

    print(f"\n{'='*70}")
    print(f"  EXPERIMENT: DR={dr_alg} | Trust={trust_alg} | Map={map_config}")
    
    if time_limit:
        start_time_offset = get_start_time_from_ini(map_config)
        sumo_begin = get_sumo_begin_time(map_config)
        duration_s = 0
        if time_limit.endswith('s'):
            try:
                duration_s = int(time_limit[:-1])
            except ValueError:
                pass
        else:
            try:
                duration_s = int(time_limit)
            except ValueError:
                pass
        finish_time = start_time_offset + duration_s

        def seconds_to_hhmm(s):
            h = (s // 3600) % 24
            m = (s % 3600) // 60
            sec = s % 60
            return f"{h:02d}:{m:02d}:{sec:02d}"

        sumo_start = sumo_begin + start_time_offset
        sumo_end = sumo_start + duration_s
        print(f"  Simulation Time: {duration_s}s. SUMO Scenario: {seconds_to_hhmm(sumo_start)} -> {seconds_to_hhmm(sumo_end)}")
        
    if cam_freq:
        print(f"  CAM Frequency (All/Honest): {cam_freq} Hz (CBR)")
    if faulty_cam_freq:
        print(f"  Faulty Sensor CAM Frequency: {faulty_cam_freq} Hz")
    if fault_profile:
        print(f"  Sensor Fault Profile: {fault_profile}")
    if faulty_rate is not None:
        print(f"  Faulty Vehicle Ratio: {faulty_rate * 100:.1f}%")
    if seed is not None:
        print(f"  RNG Seed Set: {seed}")
    if mode != "headless":
        print(f"  Mode: {mode.upper()}")
    print(f"{'='*70}")

    # 1. Create experiment directory
    experiment_path = create_experiment_dir(map_config, dr_alg, trust_alg, name)
    print(f"  Experiment dir: {os.path.relpath(experiment_path, WORKSPACE_DIR)}")

    config = {
        "map": map_config,
        "dr_algorithm": dr_alg,
        "trust_algorithm": trust_alg,
        "time_limit": time_limit or "35s",
        "cam_freq_hz": cam_freq,
        "faulty_cam_freq_hz": faulty_cam_freq,
        "fault_profile": fault_profile or "constant_offset",
        "faulty_rate": faulty_rate or 0.15,
        "seed": seed or 0,
        "timestamp": datetime.now().isoformat(),
        "experiment_dir": experiment_path,
    }

    # 2. Run simulation
    print(f"\n  Running simulation (Mode: {mode})...")
    success = run_simulation(map_config, dr_alg, trust_alg, time_limit, mode, cam_freq, fault_profile, faulty_cam_freq, seed, extra_args)
    if not success:
        config["status"] = "simulation_failed"
        save_experiment(experiment_path, config, {})
        return None

    # 3. Collect results
    print("  Collecting results...")
    collected = collect_results(map_config, experiment_path)
    print(f"     Collected: {', '.join(collected)}")

    # 4. Run analysis
    print("  Running analysis...\n")
    metrics = run_analysis(experiment_path, map_config)
    if not metrics:
        config["status"] = "analysis_failed"
        save_experiment(experiment_path, config, {})
        return None

    # 5. Save everything
    config["status"] = "completed"
    metrics["dr_algorithm"] = dr_alg
    metrics["trust_algorithm"] = trust_alg
    metrics["map"] = map_config
    metrics["fault_profile"] = fault_profile or "constant_offset"
    metrics["faulty_rate"] = faulty_rate or 0.15
    metrics["seed"] = seed or 0
    save_experiment(experiment_path, config, metrics)

    # 6. Update latest symlink
    update_latest_link(experiment_path)

    print(f"\n  [OK] Experiment complete.")
    print(f"    Path: {os.path.relpath(experiment_path, WORKSPACE_DIR)}")
    print(f"    Files: metrics.json | analysis_report.txt | config.json")

    return metrics



def main():
    parser = argparse.ArgumentParser(
        description="V2X Trust Evaluation — Unified Experiment Orchestrator",
        formatter_class=argparse.RawDescriptionHelpFormatter)

    parser.add_argument("--map", type=str, default="lust", choices=['grid', 'rsu', 'lust', 'highway', 'most', 'erlangen'],
                        help="Map configuration to run")
    parser.add_argument("--dr", default="HistoryAwareCTRA", choices=DR_MODELS,
                        help="Dead Reckoning model (default: HistoryAwareCTRA)")
    parser.add_argument("--trust", default="BetaReputation", choices=TRUST_MODELS,
                        help="Trust evaluation algorithm (default: BetaReputation)")
    parser.add_argument("--time-limit", "--time", default=None,
                        help="Simulation duration (e.g. 35s, 60s, 100s)")
    parser.add_argument("--gui", action="store_true",
                        help="Run simulation with SUMO-GUI")
    parser.add_argument("--qtenv", action="store_true",
                        help="Run simulation with OMNeT++ Qtenv GUI")
    parser.add_argument("--name", default=None,
                        help="Custom experiment name (overrides auto-generated name)")
    parser.add_argument("--sweep", action="store_true",
                        help="Run full matrix sweep: all DR × Trust combinations")
    parser.add_argument("--cam-freq", type=float, default=None,
                        help="Force a fixed CAM transmission frequency in Hz for honest vehicles (e.g. 2 for 2Hz, 10 for 10Hz)")
    parser.add_argument("--faulty-cam-freq", type=float, default=None,
                        help="Force a fixed CAM transmission frequency in Hz for faulty vehicles only")
    parser.add_argument("--fault-profile", type=str, default="constant_offset",
                        choices=["constant_offset", "random_noise", "gradual_drift", "offset", "noise", "drift"],
                        help="Sensor fault profile (default: constant_offset)")
    parser.add_argument("--faulty-rate", type=float, default=None,
                        help="Ratio of faulty vehicles in traffic (e.g., 0.05, 0.10, 0.15, 0.20)")
    parser.add_argument("--seed", type=int, default=None,
                        help="OMNeT++ pseudo-random number generator seed set (for statistical confidence)")
    parser.add_argument('extra_args', nargs=argparse.REMAINDER, help="Additional arguments to pass to OMNeT++")

    args = parser.parse_args()

    # --- Sweep mode ---
    mode = "gui" if args.gui else ("qtenv" if args.qtenv else "headless")

    if args.sweep:
        sweep_name = args.name or f"sweep_{args.map}"
        sweep_dir = create_experiment_dir(args.map, "sweep", "all", name=sweep_name)

        print(f"\n{'='*70}")
        print(f"  MATRIX SWEEP: {len(DR_MODELS)} DR × {len(TRUST_MODELS)} Trust = {len(DR_MODELS) * len(TRUST_MODELS)} combinations")
        print(f"  Map: {args.map} | Experiment: {os.path.relpath(sweep_dir, WORKSPACE_DIR)}")
        print(f"{'='*70}")

        results_list = []
        for dr in DR_MODELS:
            for trust in TRUST_MODELS:
                metrics = run_single_experiment(args.map, dr, trust, args.time_limit, mode=mode, cam_freq=args.cam_freq, fault_profile=args.fault_profile, faulty_rate=args.faulty_rate, faulty_cam_freq=args.faulty_cam_freq)
                if metrics:
                    results_list.append(metrics)

        if results_list:
            table_text = print_summary_table(results_list)

            # Save sweep summary
            with open(os.path.join(sweep_dir, "sweep_results.json"), "w") as f:
                json.dump(results_list, f, indent=2)
            with open(os.path.join(sweep_dir, "sweep_summary.txt"), "w") as f:
                f.write(table_text)
            save_experiment(sweep_dir, {
                "type": "sweep",
                "map": args.map,
                "combinations": len(results_list),
                "timestamp": datetime.now().isoformat(),
            }, {"sweep_results": results_list})

            update_latest_link(sweep_dir)

            print(f"\n  [OK] Sweep complete: {len(results_list)}/{len(DR_MODELS) * len(TRUST_MODELS)} succeeded")
            print(f"    Path: {os.path.relpath(sweep_dir, WORKSPACE_DIR)}")
        return

    # --- Single experiment mode ---
    metrics = run_single_experiment(args.map, args.dr, args.trust, args.time_limit, args.name, mode=mode, cam_freq=args.cam_freq, fault_profile=args.fault_profile, faulty_rate=args.faulty_rate, faulty_cam_freq=args.faulty_cam_freq, seed=args.seed, extra_args=args.extra_args)
    if not metrics:
        sys.exit(1)


if __name__ == "__main__":
    main()
