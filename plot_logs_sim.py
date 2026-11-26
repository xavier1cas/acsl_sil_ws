"""
Diagnostic plots for the PyChrono simulator's workspace .mat logs, optionally
overlaid with a flightstack .log for side-by-side comparison.

Frame conventions: the simulator's PIDLogger already stores
position/velocity/euler angles in NED and angular velocity in body FRD -- the
same frames the C++ flightstack uses, so sim and flightstack columns can be
overlaid directly without further conversion.

Note: in SIL mode, "desired_euler_angles", "mu_translational", "control_input"
and "euler_angles_dot" in the sim log are placeholder zeros (the bridge does
not receive these internal quantities back from the C++ controller).
"""

import argparse
import glob
import os
import re

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.io import loadmat

AXES = ["x", "y", "z"]

# sil_interface.cpp: kMixerRowToX8Motor -- maps flightstack mixer-row i (its
# "T{i+1}" thrust_vector entry) to the PyChrono motor index it is sent to.
MIXER_ROW_TO_X8_MOTOR = [6, 5, 4, 7, 1, 2, 3, 0]
X8_MOTOR_TO_MIXER_ROW = [MIXER_ROW_TO_X8_MOTOR.index(m) for m in range(8)]

SIM_COLUMN_MAP = {
    "time": "time",
    "position_x": "pos_x", "position_y": "pos_y", "position_z": "pos_z",
    "velocity_x": "vel_x", "velocity_y": "vel_y", "velocity_z": "vel_z",
    "euler_angles_roll": "roll", "euler_angles_pitch": "pitch", "euler_angles_yaw": "yaw",
    "angular_velocity_x": "wx", "angular_velocity_y": "wy", "angular_velocity_z": "wz",
    "desired_euler_angles_roll": "roll_des", "desired_euler_angles_pitch": "pitch_des",
    "user_defined_position_x": "pos_x_ref", "user_defined_position_y": "pos_y_ref", "user_defined_position_z": "pos_z_ref",
    "user_defined_velocity_x": "vel_x_ref", "user_defined_velocity_y": "vel_y_ref", "user_defined_velocity_z": "vel_z_ref",
    "user_defined_acceleration_x": "acc_x_ref", "user_defined_acceleration_y": "acc_y_ref", "user_defined_acceleration_z": "acc_z_ref",
    "user_defined_yaw": "yaw_ref",
    "control_input_U1": "U1", "control_input_U2": "U2", "control_input_U3": "U3", "control_input_U4": "U4",
    **{f"thrust_motors_N_T{i}": f"T{i}" for i in range(1, 9)},
}

FLIGHTSTACK_COLUMN_MAP = {
    "Current time [s]": "time",
    "Position x [m]": "pos_x", "Position y [m]": "pos_y", "Position z [m]": "pos_z",
    "Velocity x [m/s]": "vel_x", "Velocity y [m/s]": "vel_y", "Velocity z [m/s]": "vel_z",
    "Roll [rad]": "roll", "Pitch [rad]": "pitch", "Yaw [rad]": "yaw",
    "Angular velocity x [rad/s]": "wx", "Angular velocity y [rad/s]": "wy", "Angular velocity z [rad/s]": "wz",
    "Roll desired [rad]": "roll_des", "Pitch desired [rad]": "pitch_des",
    "User-defined position x [m]": "pos_x_ref", "User-defined position y [m]": "pos_y_ref", "User-defined position z [m]": "pos_z_ref",
    "User-defined velocity x [m/s]": "vel_x_ref", "User-defined velocity y [m/s]": "vel_y_ref", "User-defined velocity z [m/s]": "vel_z_ref",
    "User-defined acceleration x [m/s^2]": "acc_x_ref", "User-defined acceleration y [m/s^2]": "acc_y_ref", "User-defined acceleration z [m/s^2]": "acc_z_ref",
    "User-defined yaw [rad]": "yaw_ref",
    "U control input U1 [N]": "U1", "U control input U2 [Nm]": "U2", "U control input U3 [Nm]": "U3", "U control input U4 [Nm]": "U4",
    **{f"Thrust motors X8COPTER T{i} [N]": f"T{i}" for i in range(1, 9)},
}


def find_latest(patterns):
    files = []
    for pattern in patterns:
        files.extend(glob.glob(pattern, recursive=True))
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def load_sim_log(path: str) -> pd.DataFrame:
    mat = loadmat(path, simplify_cells=True)
    log = mat["log"]

    flat = {}

    def _flatten(d, prefix=""):
        for key, value in d.items():
            full_key = f"{prefix}{key}" if prefix else key
            if isinstance(value, dict):
                _flatten(value, full_key + "_")
            else:
                flat[full_key] = np.asarray(value).reshape(-1)

    _flatten(log)
    return pd.DataFrame(flat).rename(columns=SIM_COLUMN_MAP)


def load_flightstack_log(path: str) -> pd.DataFrame:
    header_pattern = re.compile(r"\]\s*\[\d+\]\s*(.*)")
    headers = []
    rows = []

    with open(path, "r") as f:
        for line in f:
            if "Initial timestamp" in line:
                if not headers:
                    match = header_pattern.search(line)
                    if match:
                        headers = [h.strip() for h in match.group(1).strip().split(",") if h.strip()]
                continue

            match = header_pattern.search(line)
            if not match:
                continue
            try:
                values = [float(x) for x in match.group(1).strip().split(",") if x.strip()]
            except ValueError:
                continue
            if headers and len(values) >= len(headers):
                rows.append(values[:len(headers)])

    if not headers or not rows:
        raise ValueError(f"Could not parse flightstack log: {path}")

    return pd.DataFrame(rows, columns=headers).rename(columns=FLIGHTSTACK_COLUMN_MAP)


def load_sil_bridge_log(path: str) -> pd.DataFrame:
    return pd.read_csv(path)


def describe(label: str, df: pd.DataFrame | None):
    if df is None:
        return
    t = df["time"]
    print(f"[{label}] {len(df)} samples, t = [{t.min():.3f}, {t.max():.3f}] s")


def plot_position(datasets, hover_target=None):
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("Position Tracking (NED) - Sim vs Flightstack", fontsize=13, fontweight="bold")
    for i, axis in enumerate(AXES):
        ax = axs[i]
        for label, df in datasets.items():
            if df is None:
                continue
            ref_col, act_col = f"pos_{axis}_ref", f"pos_{axis}"
            if ref_col in df.columns:
                ax.plot(df["time"], df[ref_col], "--", alpha=0.7, label=f"{label} ref")
            if act_col in df.columns:
                ax.plot(df["time"], df[act_col], "-", label=f"{label} actual")
        if hover_target is not None:
            ax.axhline(hover_target[i], color="gray", linestyle=":",
                        label="SIL-P hardcoded target" if i == 0 else None)
        ax.set_ylabel(f"{axis} [m]")
        ax.grid(True)
        if i == 0:
            ax.legend(fontsize=8)
    axs[-1].set_xlabel("Time [s]")
    return fig


def plot_velocity(datasets):
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("Velocity Tracking (NED) - Sim vs Flightstack", fontsize=13, fontweight="bold")
    for i, axis in enumerate(AXES):
        ax = axs[i]
        for label, df in datasets.items():
            if df is None:
                continue
            ref_col, act_col = f"vel_{axis}_ref", f"vel_{axis}"
            if ref_col in df.columns:
                ax.plot(df["time"], df[ref_col], "--", alpha=0.7, label=f"{label} ref")
            if act_col in df.columns:
                ax.plot(df["time"], df[act_col], "-", label=f"{label} actual")
        ax.set_ylabel(f"v_{axis} [m/s]")
        ax.grid(True)
        if i == 0:
            ax.legend(fontsize=8)
    axs[-1].set_xlabel("Time [s]")
    return fig


def plot_attitude(datasets):
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("Attitude (Euler Angles, NED)", fontsize=13, fontweight="bold")
    desired_cols = {"roll": "roll_des", "pitch": "pitch_des", "yaw": "yaw_ref"}
    for i, field in enumerate(["roll", "pitch", "yaw"]):
        ax = axs[i]
        for label, df in datasets.items():
            if df is None:
                continue
            if field in df.columns:
                ax.plot(df["time"], np.degrees(df[field]), "-", label=f"{label} {field}")
            des_col = desired_cols[field]
            if des_col in df.columns:
                ax.plot(df["time"], np.degrees(df[des_col]), "--", alpha=0.7, label=f"{label} {des_col}")
        ax.set_ylabel(f"{field} [°]")
        ax.grid(True)
        if i == 0:
            ax.legend(fontsize=8)
    axs[-1].set_xlabel("Time [s]")
    return fig


def plot_angular_velocity(datasets):
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("Angular Velocity (Body FRD)", fontsize=13, fontweight="bold")
    for i, axis in enumerate(AXES):
        ax = axs[i]
        col = f"w{axis}"
        for label, df in datasets.items():
            if df is None or col not in df.columns:
                continue
            ax.plot(df["time"], df[col], "-", label=label)
        ax.set_ylabel(f"ω_{axis} [rad/s]")
        ax.grid(True)
        if i == 0:
            ax.legend(fontsize=8)
    axs[-1].set_xlabel("Time [s]")
    return fig


def plot_thrusts(datasets, sil_df=None):
    fig, axs = plt.subplots(2, 4, figsize=(15, 7), sharex=True)
    fig.suptitle("Motor Thrusts [N] - sil_interface (sent) vs Sim (applied) vs Flightstack (computed)",
                  fontsize=13, fontweight="bold")
    for i in range(8):
        ax = axs[i // 4, i % 4]
        col = f"T{i + 1}"
        for label, df in datasets.items():
            if df is None:
                continue
            if label == "flightstack":
                # flightstack logs raw mixer-row outputs; remap row -> PyChrono motor i+1
                src_col = f"T{X8_MOTOR_TO_MIXER_ROW[i] + 1}"
            else:
                src_col = col
            if src_col not in df.columns:
                continue
            ax.plot(df["time"], df[src_col], label=f"{label} applied", alpha=0.8)
        if sil_df is not None:
            sent_col = f"T{i + 1}_sent"
            if sent_col in sil_df.columns:
                ax.plot(sil_df["time"], sil_df[sent_col], "--",
                        label="sil_interface sent", alpha=0.8)
        ax.set_title(f"Motor {i + 1}")
        ax.grid(True)
        if i == 0:
            ax.legend(fontsize=8)
    for ax in axs[-1, :]:
        ax.set_xlabel("Time [s]")
    return fig


ADAPTIVE_MATRIX_NAMES = ["K_hat_x", "K_hat_r", "Theta_hat", "K_hat_g"]


def plot_adaptive_gains(fs_df):
    """Frobenius-norm history of each adaptive gain matrix + projection-operator
    activation flags. Columns present depend on CONTROLLER_TYPE: PID has none,
    MRAC has K_hat_x/r + Theta_hat, the two-layer variants add K_hat_g."""
    if fs_df is None:
        return None

    def matrix_norm(prefix):
        cols = [c for c in fs_df.columns if c.startswith(prefix + " index-")]
        if not cols:
            return None
        return np.linalg.norm(fs_df[cols].to_numpy(), axis=1)

    norms = {"translational": {}, "rotational": {}}
    for loop in norms:
        for name in ADAPTIVE_MATRIX_NAMES:
            norm = matrix_norm(f"{name}_{loop}")
            if norm is not None:
                norms[loop][name] = norm

    proj_op_cols = [c for c in fs_df.columns if c.startswith("proj_op_activated_")]
    if not any(norms["translational"]) and not any(norms["rotational"]):
        return None

    fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle("Adaptive Gains (Frobenius norm) and Projection Operator - Flightstack",
                 fontsize=13, fontweight="bold")

    for name, norm in norms["translational"].items():
        axs[0].plot(fs_df["time"], norm, label=name)
    axs[0].set_ylabel("||·||_F Translational")
    axs[0].set_yscale("log")
    axs[0].grid(True)
    axs[0].legend(fontsize=8)

    for name, norm in norms["rotational"].items():
        axs[1].plot(fs_df["time"], norm, label=name)
    axs[1].set_ylabel("||·||_F Rotational")
    axs[1].set_yscale("log")
    axs[1].grid(True)
    axs[1].legend(fontsize=8)

    for col in proj_op_cols:
        axs[2].step(fs_df["time"], fs_df[col], where="post",
                     label=col.replace("proj_op_activated_", ""), alpha=0.8)
    axs[2].set_ylabel("Projection Operator\nActivated [0/1]")
    axs[2].set_yticks([0, 1])
    axs[2].set_xlabel("Time [s]")
    axs[2].grid(True)
    axs[2].legend(fontsize=7, ncol=2)

    return fig


def plot_timing(sil_df):
    fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle("SIL Bridge Timing Diagnostics", fontsize=13, fontweight="bold")

    axs[0].plot(sil_df["time"], sil_df["dt_sim"], ".-")
    axs[0].axhline(0.01, color="gray", linestyle=":", label="expected (time_step_rk4_ = 0.01 s)")
    axs[0].set_ylabel("Δt sim [s]")
    axs[0].grid(True)
    axs[0].legend(fontsize=8)

    axs[1].plot(sil_df["time"], sil_df["wall_dt"] * 1000, ".-")
    axs[1].set_ylabel("Δt wall-clock [ms]")
    axs[1].set_xlabel("Time [s]")
    axs[1].grid(True)
    return fig


def plot_acceleration_reference(datasets):
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("User-Defined Acceleration Reference (NED)", fontsize=13, fontweight="bold")
    for i, axis in enumerate(AXES):
        ax = axs[i]
        col = f"acc_{axis}_ref"
        for label, df in datasets.items():
            if df is None or col not in df.columns:
                continue
            ax.plot(df["time"], df[col], label=label)
        ax.set_ylabel(f"acc_{axis} [m/s^2]")
        ax.grid(True)
        if i == 0:
            ax.legend(fontsize=8)
    axs[-1].set_xlabel("Time [s]")
    return fig


def main():
    parser = argparse.ArgumentParser(description="Compare ACSL simulator (.mat) and flightstack (.log) diagnostic logs.")
    parser.add_argument("--sim", help="Path to simulator workspace .mat log (default: latest under src/simulator/logs)")
    parser.add_argument("--flightstack", help="Path to flightstack .log file (default: latest under src/flightstack/log)")
    parser.add_argument("--no-flightstack", action="store_true", help="Plot simulator data only")
    parser.add_argument("--sil-bridge", default="src/simulator/logs/sil_bridge_diagnostics.csv",
                         help="Path to the SIL bridge diagnostics CSV (sent thrusts + timing)")
    parser.add_argument("--no-sil-bridge", action="store_true",
                         help="Skip the SIL bridge diagnostics overlay/timing plot")
    parser.add_argument("--no-sil-target", action="store_true",
                         help="Hide the SIL-P hardcoded hover target reference line")
    parser.add_argument("--sil-target", type=float, nargs=3, default=[0.0, 0.0, -1.0],
                         metavar=("X", "Y", "Z"),
                         help="NED hover target hardcoded in sil_interface.cpp simpleProportionalController (default: 0 0 -1)")
    args = parser.parse_args()

    sim_path = args.sim or find_latest(["src/simulator/logs/**/workspaces/*.mat"])
    if sim_path is None:
        raise SystemExit("No simulator .mat log found. Pass - sim explicitly.")
    sim_df = load_sim_log(sim_path)

    fs_df = None
    fs_path = None
    if not args.no_flightstack:
        fs_path = args.flightstack or find_latest(["src/flightstack/log/**/logs/*.log"])
        if fs_path:
            try:
                fs_df = load_flightstack_log(fs_path)
            except ValueError as exc:
                print(f"[flightstack] {fs_path}: {exc}")
                print("[flightstack] empty/unparseable (logLogData() is only called by the real "
                      "flightstack pipeline, not SIL_CONTROLLER_MODE=simple) - plotting without it")
                fs_path = None

    sil_df = None
    if not args.no_sil_bridge and os.path.exists(args.sil_bridge):
        sil_df = load_sil_bridge_log(args.sil_bridge)

    print(f"[sim]         {sim_path}")
    describe("sim", sim_df)
    if fs_path:
        print(f"[flightstack] {fs_path}")
        describe("flightstack", fs_df)
    elif not args.no_flightstack:
        print("[flightstack] none found - plotting simulator data only")
    if sil_df is not None:
        print(f"[sil bridge]  {args.sil_bridge}")
        describe("sil bridge", sil_df)
    elif not args.no_sil_bridge:
        print("[sil bridge]  none found - skipping sent-thrust overlay and timing plot")

    datasets = {"sim": sim_df, "flightstack": fs_df}
    hover_target = None if args.no_sil_target else args.sil_target

    figs = [
        plot_position(datasets, hover_target),
        plot_velocity(datasets),
        plot_attitude(datasets),
        plot_angular_velocity(datasets),
        plot_thrusts(datasets, sil_df),
        plot_acceleration_reference(datasets),
    ]
    if sil_df is not None:
        figs.append(plot_timing(sil_df))
    adaptive_fig = plot_adaptive_gains(fs_df)
    if adaptive_fig is not None:
        figs.append(adaptive_fig)
    for fig in figs:
        fig.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()
