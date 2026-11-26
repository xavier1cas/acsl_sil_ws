import glob
import json
import os
import re
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# ==============================================================================
# Controllers to compare. Comment out the ones you don't want in this run.
# Each entry is either "ControllerName" (auto-picks the latest non-empty log under
# src/flightstack/log/*/<ControllerName>/logs/) or "ControllerName=path/to/log_xxx.log"
# to pin an explicit file instead.
# ==============================================================================
#CONTROLLERS = [
#    "PID",
#    "MRAC",
#   "TwoLayerMRAC",
#    "HybridTwoLayerMRAC",
#    "FunnelTwoLayerMRAC",
#]

CONTROLLERS = [
    "PID",
    "MRAC",
    "TwoLayerMRAC",
    #"FunnelTwoLayerMRAC",
    #"HybridTwoLayerMRAC",
]

def resolve_log_path(spec):
    """Returns (label, log_path) for a 'Name' or 'Name=path' spec."""
    if "=" in spec:
        name, path = spec.split("=", 1)
        return name, path
    candidates = glob.glob(f"src/flightstack/log/*/{spec}/logs/*.log")
    candidates = [c for c in candidates if os.path.getsize(c) > 0]
    if not candidates:
        raise SystemExit(f"No non-empty log found for controller '{spec}' under src/flightstack/log/*/{spec}/logs/")
    return spec, max(candidates, key=os.path.getmtime)


def load_log(log_path):
    """Parses a flightstack .log file into a DataFrame (same format as plot_logs.py)."""
    headers = []
    data_rows = []
    with open(log_path, "r") as f:
        for line in f:
            if "Initial timestamp" in line and not headers:
                match = re.search(r"\]\s*\[\d+\]\s*(.*)", line)
                if match:
                    raw_headers = match.group(1).strip()
                    headers = [h.strip() for h in raw_headers.split(",") if h.strip()]
                continue
            if "Initial timestamp" in line:
                continue
            match = re.search(r"\]\s*\[\d+\]\s*(.*)", line)
            if match:
                raw_data = match.group(1).strip()
                try:
                    values = [float(x) for x in raw_data.split(",") if x.strip()]
                    if len(values) >= len(headers) and len(headers) > 0:
                        data_rows.append(values[:len(headers)])
                except ValueError:
                    continue
    df = pd.DataFrame(data_rows, columns=headers)
    return df


def load_run_gains(log_path):
    """Loads the gains-*.json copied next to this specific log run (sibling 'gains' dir, same timestamp)."""
    match = re.search(r"(\d{8}_\d{6})\.log$", os.path.basename(log_path))
    if not match:
        return None
    run_dir = os.path.dirname(os.path.dirname(log_path))
    candidates = glob.glob(os.path.join(run_dir, "gains", f"*_{match.group(1)}.json"))
    if not candidates:
        return None
    with open(candidates[0], "r") as f:
        return json.load(f)


def dead_zone_thresholds(run_gains):
    """Returns (freeze_translational, freeze_rotational), either possibly None."""
    if run_gains is None or "ADAPTIVE" not in run_gains:
        return None, None
    adaptive = run_gains["ADAPTIVE"]
    freeze_translational = None
    freeze_rotational = None
    if "dead_zone_delta_translational" in adaptive and "dead_zone_e0_translational" in adaptive:
        freeze_translational = adaptive["dead_zone_delta_translational"] * adaptive["dead_zone_e0_translational"]
    if "dead_zone_delta_rotational" in adaptive and "dead_zone_e0_rotational" in adaptive:
        freeze_rotational = adaptive["dead_zone_delta_rotational"] * adaptive["dead_zone_e0_rotational"]
    return freeze_translational, freeze_rotational


# ==============================================================================
# Load every requested run
# ==============================================================================
runs = []
for spec in CONTROLLERS:
    label, log_path = resolve_log_path(spec)
    print(f"[{label}] {log_path}")
    df = load_log(log_path)
    run_gains = load_run_gains(log_path)
    freeze_translational, freeze_rotational = dead_zone_thresholds(run_gains)
    runs.append({
        "label": label,
        "log_path": log_path,
        "df": df,
        "freeze_translational": freeze_translational,
        "freeze_rotational": freeze_rotational,
    })

colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
color_of = {run["label"]: colors[i % len(colors)] for i, run in enumerate(runs)}


def axis_error_plot(ax, runs, cols_target, cols_actual, ylabel):
    for run in runs:
        df = run["df"]
        if cols_target not in df.columns or cols_actual not in df.columns:
            continue
        err = df[cols_actual] - df[cols_target]
        ax.plot(df["Current time [s]"], err, label=run["label"], color=color_of[run["label"]])
    ax.set_ylabel(ylabel)
    ax.grid(True)


def norm_error_plot(ax, runs, target_cols, actual_cols, ylabel, freeze_key=None):
    for run in runs:
        df = run["df"]
        if not all(c in df.columns for c in target_cols + actual_cols):
            continue
        err = np.sqrt(sum(
            (df[a] - df[t]) ** 2 for t, a in zip(target_cols, actual_cols)
        ))
        ax.plot(df["Current time [s]"], err, label=run["label"], color=color_of[run["label"]])
        if freeze_key is not None and run[freeze_key] is not None:
            ax.axhline(run[freeze_key], linestyle="--", color=color_of[run["label"]], alpha=0.6,
                       label=f"{run['label']} freeze thr.")
    ax.set_ylabel(ylabel)
    ax.grid(True)
    ax.legend(fontsize=7, ncol=2)


# ==============================================================================
# FIGURA 1: POSITION TRACKING ERROR (vs user-defined trajectory)
# ==============================================================================
fig1, axs1 = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
fig1.suptitle("COMPARISON 1: POSITION TRACKING ERROR", fontsize=14, fontweight='bold')

for ax, axis in zip(axs1.flat[:3], ("x", "y", "z")):
    axis_error_plot(ax, runs, f"User-defined position {axis} [m]", f"Position {axis} [m]",
                     f"Position Tracking Error {axis} [m]")
axs1[0, 0].legend(fontsize=8)

norm_error_plot(
    axs1[1, 1], runs,
    [f"User-defined position {a} [m]" for a in ("x", "y", "z")],
    [f"Position {a} [m]" for a in ("x", "y", "z")],
    "||Position Tracking Error||_2 [m]"
)
for ax in axs1[1, :]:
    ax.set_xlabel("Time [s]")

# ==============================================================================
# FIGURA 2: VELOCITY TRACKING ERROR (vs user-defined trajectory)
# ==============================================================================
fig2, axs2 = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
fig2.suptitle("COMPARISON 2: VELOCITY TRACKING ERROR", fontsize=14, fontweight='bold')

for ax, axis in zip(axs2.flat[:3], ("x", "y", "z")):
    axis_error_plot(ax, runs, f"User-defined velocity {axis} [m/s]", f"Velocity {axis} [m/s]",
                     f"Velocity Tracking Error {axis} [m/s]")
axs2[0, 0].legend(fontsize=8)

norm_error_plot(
    axs2[1, 1], runs,
    [f"User-defined velocity {a} [m/s]" for a in ("x", "y", "z")],
    [f"Velocity {a} [m/s]" for a in ("x", "y", "z")],
    "||Velocity Tracking Error||_2 [m/s]"
)
for ax in axs2[1, :]:
    ax.set_xlabel("Time [s]")

# ==============================================================================
# FIGURA 3: ATTITUDE TRACKING ERROR (Roll/Pitch/Yaw error, as logged by the controller)
# ==============================================================================
fig3, axs3 = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
fig3.suptitle("COMPARISON 3: ATTITUDE TRACKING ERROR", fontsize=14, fontweight='bold')

attitude_error_cols = ["Roll error [rad]", "Pitch error [rad]", "Yaw error [rad]"]
for ax, col, name in zip(axs3.flat[:3], attitude_error_cols, ("Roll", "Pitch", "Yaw")):
    for run in runs:
        df = run["df"]
        if col not in df.columns:
            continue
        ax.plot(df["Current time [s]"], np.degrees(df[col]), label=run["label"], color=color_of[run["label"]])
    ax.set_ylabel(f"Errore {name} [deg]")
    ax.grid(True)
axs3[0, 0].legend(fontsize=8)

for run in runs:
    df = run["df"]
    if not all(c in df.columns for c in attitude_error_cols):
        continue
    norm = np.sqrt(sum(df[c] ** 2 for c in attitude_error_cols))
    axs3[1, 1].plot(df["Current time [s]"], np.degrees(norm), label=run["label"], color=color_of[run["label"]])
axs3[1, 1].set_ylabel("||Attitude Tracking Error||_2 [deg]")
axs3[1, 1].grid(True)
axs3[1, 1].legend(fontsize=8)
for ax in axs3[1, :]:
    ax.set_xlabel("Time [s]")

# ==============================================================================
# FIGURA 4: ANGULAR VELOCITY TRACKING ERROR (desired vs actual roll_dot/pitch_dot/yaw_dot)
# ==============================================================================
fig4, axs4 = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
fig4.suptitle("COMPARISON 4: ANGULAR VELOCITY TRACKING ERROR", fontsize=14, fontweight='bold')

angvel_pairs = [
    ("Roll_dot desired [rad/s]", "Roll_dot [rad/s]", "Roll_dot"),
    ("Pitch_dot desired [rad/s]", "Pitch_dot [rad/s]", "Pitch_dot"),
    ("User-defined yaw_dot [rad/s]", "Yaw_dot [rad/s]", "Yaw_dot"),
]
for ax, (target_col, actual_col, name) in zip(axs4.flat[:3], angvel_pairs):
    axis_error_plot(ax, runs, target_col, actual_col, f"Error {name} [rad/s]")
axs4[0, 0].legend(fontsize=8)

norm_error_plot(
    axs4[1, 1], runs,
    [p[0] for p in angvel_pairs],
    [p[1] for p in angvel_pairs],
    "||Angular Velocity Tracking Error||_2 [rad/s]"
)
for ax in axs4[1, :]:
    ax.set_xlabel("Time [s]")

# ==============================================================================
# FIGURA 5: ADAPTIVE TRACKING ERROR vs DEAD-ZONE FREEZE THRESHOLD
# (only meaningful for MRAC-family controllers that log a reference model;
#  this is the e = x - x_ref signal the dead-zone modulation function actually acts on)
# ==============================================================================
has_refmodel = any(
    "Translational reference model position x [m]" in run["df"].columns for run in runs
)
if has_refmodel:
    fig5, axs5 = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig5.suptitle("COMPARISON 5: TRACKING ERROR (WRT REFERENCE MODEL) vs FREEZE THRESHOLD",
                   fontsize=14, fontweight='bold')

    norm_error_plot(
        axs5[0], runs,
        [f"Translational reference model position {a} [m]" for a in ("x", "y", "z")] +
        [f"Translational reference model velocity {a} [m/s]" for a in ("x", "y", "z")],
        [f"Position {a} [m]" for a in ("x", "y", "z")] +
        [f"Velocity {a} [m/s]" for a in ("x", "y", "z")],
        "||e_translational||_2",
        freeze_key="freeze_translational"
    )
    axs5[0].set_title("Translational")

    norm_error_plot(
        axs5[1], runs,
        [f"Rotational reference model omega {a} [rad/s]" for a in ("x", "y", "z")],
        [f"Angular velocity {a} [rad/s]" for a in ("x", "y", "z")],
        "||e_rotational||_2",
        freeze_key="freeze_rotational"
    )
    axs5[1].set_title("Rotational")
    axs5[1].set_xlabel("Time [s]")

# ==============================================================================
# FIGURA 6: CONTROL EFFORT - NORMALIZED THRUST FOR EACH MOTOR (X8COPTER)
# ==============================================================================
fig6, axs6 = plt.subplots(4, 2, figsize=(13, 14), sharex=True)
fig6.suptitle("COMPARISON 6: CONTROL EFFORT - NORMALIZED THRUST FOR EACH MOTOR (X8)",
               fontsize=14, fontweight='bold')

for i, ax in enumerate(axs6.flat, start=1):
    col = f"Thrust motors normalized X8COPTER T{i} [-]"
    for run in runs:
        df = run["df"]
        if col not in df.columns:
            continue
        ax.plot(df["Current time [s]"], df[col], label=run["label"], color=color_of[run["label"]], alpha=0.8)
    ax.set_ylabel(f"T{i} [-]")
    ax.set_ylim(-0.05, 1.05)
    ax.grid(True)
axs6[0, 0].legend(fontsize=7, ncol=2)
for ax in axs6[-1, :]:
    ax.set_xlabel("Time [s]")

plt.tight_layout()
plt.show()
