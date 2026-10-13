#!/usr/bin/env python3
"""
Check our unscented filter against FilterPy, step by step.

WHY THIS EXISTS

The motion model is the laboratory's, taken verbatim, and the linear algebra is Eigen's.
What remains ours is the unscented transform itself: where the sigma points go, how they are
weighted, how an angle is averaged, and how the update folds a report in.
That is the part nobody else has validated for us, and it is small enough to check against a
reference instead of reasoning about.

FilterPy is that reference.
It is Roger Labbe's implementation, the one most research code and most teaching material uses,
and its formulation is the one our C++ was written to match.

WHAT IS COMPARED

The same deterministic sequence is run through both, and the belief is compared after EVERY
predict and EVERY update: six state values and thirty-six covariance values, two hundred times.
Nothing is sampled, nothing is random, and the input is closed-form, so there is no generator on
either side whose agreement could be the thing actually under test.

TWO THINGS THE COMPARISON HAD TO ACCOUNT FOR

First, our C++ does not run sigma points through the observation, because the observation selects
the first four states and is therefore linear, which makes `Pz = H P H' + R` and `Pxz = P H'`
exact rather than approximate.
FilterPy does run them.
The two are algebraically identical, and this comparison confirms it.

Second, and this one is a real difference rather than a reformulation.
FilterPy's `update` reuses `sigmas_f`, the points it propagated during `predict`, WITHOUT
regenerating them from the covariance after the process noise Q was added.
Its `Pz` is therefore `H (P - Q) H' + R`, not `H P H' + R`.
The textbook unscented filter regenerates the points, and so does our C++.
Running FilterPy its own way differs from ours by 1.4 in the state after two hundred steps;
regenerating the points first brings the two to 6e-13.
Both runs are reported below, because the size of that gap is worth knowing: it is how much the
shortcut costs, measured rather than argued.

    ./scripts/validate_against_filterpy.py
"""

import math
import os
import subprocess
import sys

import numpy as np

try:
    from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
except ImportError:
    sys.exit("filterpy is not installed. Run: .venv/bin/pip install filterpy")

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TRACER = os.path.join(ROOT, "build", "ukf_trace")

N = 6
PX, PY, SPEED, THETA, YAWRATE, ACCEL = range(6)
M = 4


def wrap_pi(a):
    """Wrap to (-pi, pi], matching ot::wrapPi."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def ctra_predict(x, dt):
    """
    The laboratory's CTRA model, transcribed from shared/LabCtraPredict.h.

    This is a transcription and not a second implementation: every branch and every expression
    follows that file line for line, because the point of the comparison is the TRANSFORM around
    the model, not the model itself.
    """
    px, py, v, theta, omega, a = x

    v_dt = v + a * dt
    theta_dt = theta + omega * dt

    if abs(omega) < 2e-2:
        dist = v * dt + 0.5 * a * dt * dt
        px = px + dist * math.cos(theta)
        py = py + dist * math.sin(theta)
    else:
        div_omega = 1.0 / omega
        a_over_squared_omega = a / (omega * omega)
        px = px + div_omega * (v_dt * math.sin(theta_dt) - v * math.sin(theta)) \
             + a_over_squared_omega * (math.cos(theta_dt) - math.cos(theta))
        py = py - div_omega * (v_dt * math.cos(theta_dt) - v * math.cos(theta)) \
             + a_over_squared_omega * (math.sin(theta_dt) - math.sin(theta))

    return np.array([px, py, v_dt, wrap_pi(theta_dt), omega, a])


def hx(x):
    """The observation: the leading four states, which is why the update is linear."""
    return np.array([x[PX], x[PY], x[SPEED], x[THETA]])


def residual_x(a, b):
    """State residual with theta wrapped; everything else is ordinary subtraction."""
    r = np.subtract(a, b)
    r[THETA] = wrap_pi(r[THETA])
    return r


def residual_z(a, b):
    r = np.subtract(a, b)
    r[3] = wrap_pi(r[3])
    return r


def _circular_mean(sigmas, Wm, angle_index):
    """
    Weighted mean with one circular row, matching ot::unscentedMean.

    The linear rows are a plain weighted sum.
    The angle row averages WRAPPED OFFSETS from the centre point and adds the centre back, which
    is the only way to get +-180 rather than 0 when averaging +179 and -179.
    """
    mean = np.dot(Wm, sigmas)
    reference = sigmas[0, angle_index]
    offset = sum(Wm[i] * wrap_pi(sigmas[i, angle_index] - reference)
                 for i in range(sigmas.shape[0]))
    mean[angle_index] = wrap_pi(reference + offset)
    return mean


def mean_x(sigmas, Wm):
    return _circular_mean(sigmas, Wm, THETA)


def mean_z(sigmas, Wm):
    return _circular_mean(sigmas, Wm, 3)


def scripted_measurement(i):
    """Identical to tools/ukf_trace.cc::scriptedMeasurement, in the model's frame."""
    t = float(i)
    return np.array([
        0.3 * math.sin(0.10 * t),
        2.0 * t + 0.2 * math.cos(0.07 * t),
        20.0 + 0.5 * math.sin(0.05 * t),
        0.4 + 0.01 * math.sin(0.03 * t),
    ])


def parse_trace(path_or_text):
    """Read the C++ trace into (tag, step, state, covariance) records."""
    config = {}
    records = []
    for line in path_or_text.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("#"):
            for token in line.split()[2:]:
                if "=" in token:
                    k, _, v = token.partition("=")
                    config[k] = v
            continue
        parts = line.split()
        tag, step = parts[0], int(parts[1])
        values = [float(v) for v in parts[2:]]
        state = np.array(values[:N])
        cov = np.array(values[N:N + N * N]).reshape(N, N)
        records.append((tag, step, state, cov))
    return config, records


def main():
    if not os.path.exists(TRACER):
        sys.exit(f"missing {TRACER} -- run: make all")

    proc = subprocess.run([TRACER], capture_output=True, text=True)
    if proc.returncode != 0:
        sys.exit(f"ukf_trace failed:\n{proc.stderr}")
    config, records = parse_trace(proc.stdout)

    dt = float(config["dt"])
    steps = int(config["steps"])
    alpha = float(config["alpha"])
    beta = float(config["beta"])
    kappa = float(config["kappa"])
    q_diag = [float(v) for v in config["q"].split(",")]
    r_diag = [float(v) for v in config["r"].split(",")]

    print(f"configuration taken from the C++ trace:")
    print(f"  dt={dt}  steps={steps}  alpha={alpha}  beta={beta}  kappa={kappa}")
    print(f"  Q/s diag = {q_diag}")
    print(f"  R   diag = {r_diag}")

    initial = next(r for r in records if r[0] == "INITIAL")

    by_key = {(tag, step): (x, p) for tag, step, x, p in records}

    def run(regenerate_sigmas):
        """Drive FilterPy through the same sequence and return the worst disagreement."""
        pts = MerweScaledSigmaPoints(n=N, alpha=alpha, beta=beta, kappa=kappa,
                                     subtract=residual_x)
        g = UnscentedKalmanFilter(dim_x=N, dim_z=M, dt=dt, fx=ctra_predict, hx=hx, points=pts,
                                  x_mean_fn=mean_x, z_mean_fn=mean_z,
                                  residual_x=residual_x, residual_z=residual_z)
        g.x = initial[2].copy()
        g.P = initial[3].copy()
        g.Q = np.diag([q * dt for q in q_diag])
        g.R = np.diag(r_diag)

        worst = {"PREDICT": [0.0, 0.0, -1], "UPDATE": [0.0, 0.0, -1]}
        for i in range(steps):
            g.predict(dt=dt)
            ours_x, ours_p = by_key[("PREDICT", i)]
            dx = np.max(np.abs(residual_x(g.x, ours_x)))
            dp = np.max(np.abs(g.P - ours_p))
            if dx > worst["PREDICT"][0]:
                worst["PREDICT"] = [dx, max(dp, worst["PREDICT"][1]), i]
            worst["PREDICT"][1] = max(worst["PREDICT"][1], dp)

            if regenerate_sigmas:
                # What the textbook unscented filter does, and what our C++ does: the update's
                # sigma points come from the covariance AFTER the process noise was added.
                g.sigmas_f = g.points_fn.sigma_points(g.x, g.P)

            g.update(scripted_measurement(i))
            ours_x, ours_p = by_key[("UPDATE", i)]
            dx = np.max(np.abs(residual_x(g.x, ours_x)))
            dp = np.max(np.abs(g.P - ours_p))
            if dx > worst["UPDATE"][0]:
                worst["UPDATE"] = [dx, max(dp, worst["UPDATE"][1]), i]
            worst["UPDATE"][1] = max(worst["UPDATE"][1], dp)
        return worst

    print()
    print(f"compared {steps} predicts and {steps} updates, "
          f"{N} state values and {N * N} covariance values each")

    textbook = run(regenerate_sigmas=True)
    asis = run(regenerate_sigmas=False)

    print()
    print("  FilterPy made to regenerate its sigma points after Q, which is the textbook")
    print("  formulation and the one our C++ implements:")
    print()
    print(f"    {'stage':<10} {'worst |dx|':>14} {'worst |dP|':>14}   worst at step")
    print("    " + "-" * 58)
    ok = True
    for stage in ("PREDICT", "UPDATE"):
        dx, dp, at = textbook[stage]
        print(f"    {stage:<10} {dx:>14.3e} {dp:>14.3e}   {at}")
        # The two do the same arithmetic in different orders, in different languages, so exact
        # equality is not on offer; agreement at this level is.
        if dx > 1e-9 or dp > 1e-8:
            ok = False

    print()
    print("  FilterPy left at its default, which reuses the sigma points from before Q was")
    print("  added. This is NOT a failure of either side; it is the size of that shortcut:")
    print()
    print(f"    {'stage':<10} {'worst |dx|':>14} {'worst |dP|':>14}   worst at step")
    print("    " + "-" * 58)
    for stage in ("PREDICT", "UPDATE"):
        dx, dp, at = asis[stage]
        print(f"    {stage:<10} {dx:>14.3e} {dp:>14.3e}   {at}")

    print()
    if ok:
        print("  AGREES with FilterPy.")
        print("  The unscented transform, the circular mean and the update are not ours to")
        print("  defend on their own: they reproduce the reference implementation.")
    else:
        print("  DISAGREES with FilterPy. Do not trust the filter until this is resolved.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
