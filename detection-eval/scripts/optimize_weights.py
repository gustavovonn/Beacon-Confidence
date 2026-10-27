import subprocess
import pandas as pd
import io
import statsmodels.api as sm
import numpy as np

print("Running veremi_runner on honest senders to extract prediction errors...")

import os
import glob
import re
import argparse

parser = argparse.ArgumentParser(description="Calibrate the dynamic threshold weights (paper Eq. 10).")
parser.add_argument("--scenario", default="ConstPos_1416", help="Calibration scenario (default: ConstPos_1416)")
parser.add_argument("--quantile", type=float, default=0.99, help="Upper-envelope quantile (default: 0.99)")
parser.add_argument("--history-size", type=int, default=4, help="History window W (default: 4)")
args = parser.parse_args()

script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)

# Every subset of a time window shares the same traffic, so any fault subset of 14h-16h
# provides the same honest trajectories; 07h-09h stays held out for evaluation.
source_dirs = sorted(glob.glob(os.path.abspath(os.path.join(
    parent_dir, "../veremi-extension/parquet", args.scenario, "VeReMi_*"))))
if not source_dirs:
    print(f"No parquet directories found for {args.scenario}")
    exit(1)

def extract_attackers(d):
    attackers = set()
    for f in glob.glob(os.path.join(d, "traceJSON-*.parquet")):
        m = re.match(r"traceJSON-(\d+)-\d+-A([0-9]+)-", os.path.basename(f))
        if m and int(m.group(2)) > 0:
            attackers.add(int(m.group(1)))
    return attackers

runner_path = os.path.join(parent_dir, "build", "veremi_runner")

frames = []
for d in source_dirs:
    cmd = [
        runner_path,
        "--dir", d,
        "--calib",
        "--historySize", str(args.history_size)
    ]

    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print("Error running veremi_runner:")
        print(result.stderr)
        exit(1)

    # Read the CSV output and keep only messages sent by honest vehicles
    # (receiver logs also contain messages from attackers).
    d_df = pd.read_csv(io.StringIO(result.stdout))
    d_df = d_df[~d_df['sender'].isin(extract_attackers(d))]
    print(f"  {os.path.basename(d)}: {len(d_df)} honest-sender samples")
    frames.append(d_df)

df = pd.concat(frames, ignore_index=True)

print(f"Extracted {len(df)} samples from {args.scenario}.")

# The regression models the natural prediction error as a function of the same terms
# used by the dynamic threshold at runtime:
# distError = beta_0 + beta_v * (v_safe * dt) + beta_yaw * (|yawRateHat| * v_safe)

# Prepare features
df = df.dropna()

df['X1_speed_dt'] = df['safeSpeed'] * df['dt']
df['X2_yaw_speed'] = df['yawRateHat'].abs() * df['safeSpeed']

X = df[['X1_speed_dt', 'X2_yaw_speed']]
# Add constant for beta_0
X = sm.add_constant(X)
y = df['distError']

print(f"Performing Quantile Regression (q = {args.quantile}) for the upper-envelope boundary...")

# Fit the conditional quantile: (q * 100)% of honest errors fall below d_max
model = sm.QuantReg(y, X).fit(q=args.quantile)

print(model.summary())

# Extract coefficients
beta_0 = model.params['const']
beta_v = model.params['X1_speed_dt']
beta_yaw = model.params['X2_yaw_speed']

coverage = float(np.mean(y <= model.predict(X)) * 100)

print("\n=============================================")
print("OPTIMIZED VEREMI PARAMETERS FOR veremi_runner")
print("=============================================")
print(f"maxPosDev (beta_0):       {beta_0:.4f}")
print(f"contextSpeedWeight (w_v): {beta_v:.4f}")
print(f"contextYawWeight (w_yaw): {beta_yaw:.4f}")
print(f"Honest samples <= d_max:  {coverage:.2f}%")
print("=============================================\n")
