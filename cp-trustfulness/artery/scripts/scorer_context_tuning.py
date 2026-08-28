#!/usr/bin/env python3
import os
import sys
import subprocess
import argparse
import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression
from experiment import run_single_experiment, create_experiment_config

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "results"))
DEFAULT_CSV = os.path.join(RESULTS_DIR, "offline-training", "trust_context_dataset.csv")

def train_and_get_weights(csv_file=DEFAULT_CSV):
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Dataset {csv_file} not found.")
        return None
        
    honest_df = df.copy()
    
    print(f"Loaded {len(honest_df)} total rows (All honest).")
    
    if len(honest_df) == 0:
        print("No honest data found for training.")
        return None
    
    # We want to model distError as a function of kinematics and delay.
    # Let's engineer features that represent physical displacement potential:
    # 1. dist_expected: The distance the car could cover during the delay (speed * dt)
    # 2. yaw_impact: The centrifugal/lateral displacement impact during turns (abs(yaw) * speed)
    honest_df['dist_expected'] = honest_df['speed'] * honest_df['dt']
    honest_df['yaw_impact'] = np.abs(honest_df['yawRate']) * honest_df['speed']
    
    # Linear regression to find a baseline boundary
    X = honest_df[['dist_expected', 'yaw_impact']]
    y = honest_df['distError']
    
    reg = LinearRegression(fit_intercept=True)
    reg.fit(X, y)
    
    print(f"\n[MODEL] distError ~ {reg.intercept_:.2f} + {reg.coef_[0]:.4f}*(speed*dt) + {reg.coef_[1]:.4f}*(yawRate*speed)")
    
    # Calculate residuals to find a safe physical margin
    preds = reg.predict(X)
    residuals = y - preds
    
    margin_99 = np.percentile(residuals, 99.5) # Use 99.5th percentile for extreme safety
    
    print(f"[STATS] 99.5th Percentile Margin: +{margin_99:.2f}m")
    
    safe_intercept = reg.intercept_ + margin_99
    
    print("\n=======================================================")
    print("--- RECOMMENDED CONTEXT-AWARE DYNAMIC LIMIT ---")
    print(f"Intercept (maxPosDev): {safe_intercept:.2f}")
    print(f"Speed Weight (contextSpeedWeight): {reg.coef_[0]:.4f}")
    print(f"Yaw Weight (contextYawWeight): {reg.coef_[1]:.4f}")
    print("=======================================================\n")

    return {
        "intercept": safe_intercept,
        "speed_weight": reg.coef_[0],
        "yaw_weight": reg.coef_[1]
    }

def main():
    parser = argparse.ArgumentParser(
        description="Context-Aware Dynamic Limit Optimization for V2X Trust Evaluation",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    # Forward all core arguments to experiment.py
    parser.add_argument("--map", type=str, default="lust", help="Scenario map configuration")
    parser.add_argument("--dr", default="HistoryAwareCTRA", help="Dead Reckoning model")
    parser.add_argument("--trust", default="BetaReputation", help="Trust evaluation algorithm")
    parser.add_argument("--validation-time", default="45s", help="Final simulation validation duration (Phase 3)")
    parser.add_argument("--fault-profile", type=str, default="random_noise", help="Cyber attack fault profile")
    parser.add_argument("--faulty-rate", type=float, default=0.15, help="Ratio of faulty vehicles for the Phase 3 Evaluation run (Phase 1 Training is always 0.0)")
    parser.add_argument("--training-time", default="45s", help="Duration for the baseline training run (Phase 1, default: 20s)")
    parser.add_argument("--start-time", "--start", default="RUSH_HOUR",
                        help="Simulation start time. Can be specified as an integer, in seconds, as 0 or 25200, or as a \
                        TimeOfDay enum name (e.g., MORNING, AFTERNOON, EVENING, RUSH_HOUR, NIGHT).")

    args = parser.parse_args()

    csv_file = DEFAULT_CSV
    if os.path.exists(csv_file):
        os.remove(csv_file)

    print("\n" + "="*70)
    print(" PHASE 1: DATA COLLECTION (BASELINE TRAFFIC)")
    print("="*70)
    print(f"Running {args.training_time} clean baseline on map '{args.map}' to learn physics...")
    
    # Run the baseline simulation in memory
    config = create_experiment_config(
        map_config=args.map,
        dr_alg=args.dr,
        trust_alg=args.trust,
        start_time=args.start_time,
        time_limit=args.training_time,
        faulty_rate=0.0,
        experiment_name=f"baseline_{args.map}",
        mode="headless"
    )
    metrics = run_single_experiment(config)

    if not metrics:
        print("[FAIL] Baseline simulation failed. Aborting auto-tune.")
        sys.exit(1)

    print("\n" + "="*70)
    print(" PHASE 2: CONTEXT-AWARE TRAINING")
    print("="*70)
    
    weights = train_and_get_weights(csv_file)
    if not weights:
        print("[FAIL] Optimization failed. Check if trust_context_dataset.csv contains valid data.")
        sys.exit(1)

    intercept = weights["intercept"]
    speed_w = weights["speed_weight"]
    yaw_w = weights["yaw_weight"]

    print("\n" + "="*70)
    print(" PHASE 3: EVALUATION WITH OPTIMIZED WEIGHTS")
    print("="*70)

    # Build OMNeT++ parameter overrides
    omnet_overrides = [
        f"--*.node[*].middleware.DR.maxPositionDeviation={intercept:.4f}m",
        f"--*.node[*].middleware.DR.contextSpeedWeight={speed_w:.6f}",
        f"--*.node[*].middleware.DR.contextYawWeight={yaw_w:.6f}"
    ]

    print("Injecting learned context weights into simulation...")
    
    config = create_experiment_config(
        map_config=args.map,
        dr_alg=args.dr,
        trust_alg=args.trust,
        experiment_name=f"tuned_{args.map}",
        start_time=args.start_time,
        time_limit=args.validation_time,
        fault_profile=args.fault_profile,
        faulty_rate=args.faulty_rate,
        mode="headless"
    )
    run_single_experiment(config, extra_args=omnet_overrides)

if __name__ == "__main__":
    main()
