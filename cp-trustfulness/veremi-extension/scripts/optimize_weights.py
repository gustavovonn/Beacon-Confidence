import subprocess
import pandas as pd
import io
import statsmodels.api as sm
import numpy as np

print("Running veremi_runner on honest baseline to extract prediction errors...")

import os
import shutil

script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)

print("Setting up honest baseline dataset...")
tmp_dir = os.path.join(parent_dir, "honest_baseline_tmp")
os.makedirs(tmp_dir, exist_ok=True)
# Copy a few honest files (A0) to the temp directory
source_dir = os.path.abspath(os.path.join(parent_dir, "../../veremi-extension/parquet/ConstPos_0709/VeReMi_28800_32400_2022-9-11_20:45:32"))
copied = 0
for f in os.listdir(source_dir):
    if "-A0-" in f:
        shutil.copy(os.path.join(source_dir, f), tmp_dir)
        copied += 1
        if copied >= 10:  # 10 honest vehicles is plenty of data
            break

print(f"Copied {copied} honest vehicle traces to {tmp_dir}")

runner_path = os.path.join(parent_dir, "build", "veremi_runner")
cmd = [
    runner_path,
    "--dir",
    tmp_dir
]

result = subprocess.run(cmd, capture_output=True, text=True)

if result.returncode != 0:
    print("Error running veremi_runner:")
    print(result.stderr)
    exit(1)

# Read the CSV output
df = pd.read_csv(io.StringIO(result.stdout))

print(f"Extracted {len(df)} samples.")

# The regression models natural prediction error as a function of kinematic state:
# distError = beta_0 + beta_v * (speed * dt) + beta_yaw * (yawRate * speed)
# Note: Since dt is currently fixed at 0.1s in our extraction, speed*dt is just 0.1 * speed

# Prepare features
# Let's drop any rows with NaN or excessive outliers (e.g. initial start-up)
df = df.dropna()

# We only care about positive prediction errors
# Our features are: X1 = speed * dt, X2 = |yawRate| * speed
df['X1_speed_dt'] = df['speed'] * df['dt']
df['X2_yaw_speed'] = df['yawRate'] * df['speed']

X = df[['X1_speed_dt', 'X2_yaw_speed']]
# Add constant for beta_0
X = sm.add_constant(X)
y = df['distError']

print("Performing OLS Multiple Linear Regression...")

# Fit OLS
model = sm.OLS(y, X).fit()

print(model.summary())

# Extract coefficients
beta_0 = model.params['const']
beta_v = model.params['X1_speed_dt']
beta_yaw = model.params['X2_yaw_speed']

print("\n=============================================")
print("OPTIMIZED VEREMI PARAMETERS FOR veremi_runner")
print("=============================================")
print(f"maxPosDev (beta_0):       {beta_0:.4f}")
print(f"contextSpeedWeight (w_v): {beta_v:.4f}")
print(f"contextYawWeight (w_yaw): {beta_yaw:.4f}")
print("=============================================\n")
