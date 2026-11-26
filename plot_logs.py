import argparse
import glob
import json
import os
import re
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

parser = argparse.ArgumentParser(description="Plot flightstack log.")
parser.add_argument("--log", help="Path to flightstack .log file (default: latest under src/flightstack/log)")
args = parser.parse_args()

if args.log:
    log_file_path = args.log
else:
    candidates = glob.glob("src/flightstack/log/**/logs/*.log", recursive=True)
    if not candidates:
        raise SystemExit("No flightstack .log found. Pass --log explicitly.")
    log_file_path = max(candidates, key=os.path.getmtime)

print(f"[flightstack] {log_file_path}")

headers = []
data_rows = []

with open(log_file_path, "r") as f:
    for line in f:
        if "Initial timestamp" in line and not headers:
            match = re.search(r"\]\s*\[\d+\]\s*(.*)", line)
            if match:
                raw_headers = match.group(1).strip()
                headers = [h.strip() for h in raw_headers.split(",") if h.strip()]
            continue
        
        # Rows Extraction
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

# DataFrame
df = pd.DataFrame(data_rows, columns=headers)
time = df["Current time [s]"]

print(f"Log File correctly processed. Lines Read: {len(df)}")

has_filtered_D = "Outer loop Derivative term Filtered x [-]" in df.columns

# ==============================================================================
# FIGURE 1: Global position and velocity tracking (X, Y, Z)
# ==============================================================================
fig1, axs1 = plt.subplots(3, 2, figsize=(14, 9), sharex=True)
fig1.suptitle("DIAGNOSTIC 1: Kinematic Tracking", fontsize=14, fontweight='bold')

# Positions (X, Y, Z)
axs1[0, 0].plot(time, df["User-defined position x [m]"], "r--", label="Target X")
axs1[0, 0].plot(time, df["Position x [m]"], "b-", label="Real X")
axs1[0, 0].set_ylabel("Position X [m]")
axs1[0, 0].legend()
axs1[0, 0].grid(True)

axs1[1, 0].plot(time, df["User-defined position y [m]"], "r--", label="Target Y")
axs1[1, 0].plot(time, df["Position y [m]"], "b-", label="Real Y")
axs1[1, 0].set_ylabel("Position Y [m]")
axs1[1, 0].grid(True)

axs1[2, 0].plot(time, df["User-defined position z [m]"], "r--", label="Target Z")
axs1[2, 0].plot(time, df["Position z [m]"], "b-", label="Real Z")
axs1[2, 0].set_ylabel("Position Z [m]")
axs1[2, 0].set_xlabel("Time [s]")
axs1[2, 0].grid(True)

# Velocities (X, Y, Z)
axs1[0, 1].plot(time, df["User-defined velocity x [m/s]"], "r--", label="Target Vx")
axs1[0, 1].plot(time, df["Velocity x [m/s]"], "b-", label="Real Vx")
axs1[0, 1].set_ylabel("Velocity X [m/s]")
axs1[0, 1].grid(True)

axs1[1, 1].plot(time, df["User-defined velocity y [m/s]"], "r--", label="Target Vy")
axs1[1, 1].plot(time, df["Velocity y [m/s]"], "b-", label="Real Vy")
axs1[1, 1].set_ylabel("Velocity Y [m/s]")
axs1[1, 1].grid(True)

axs1[2, 1].plot(time, df["User-defined velocity z [m/s]"], "r--", label="Target Vz")
axs1[2, 1].plot(time, df["Velocity z [m/s]"], "b-", label="Real Vz")
axs1[2, 1].set_ylabel("Velocity Z [m/s]")
axs1[2, 1].set_xlabel("Time [s]")
axs1[2, 1].grid(True)

# ==============================================================================
# FIGURE 2: DRONE ORIENTATION (EULER ANGLES)
# ==============================================================================
fig2, axs2 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
fig2.suptitle("DIAGNOSTIC 2: Drone Orientation (Euler Angles)", fontsize=14, fontweight='bold')

axs2[0].plot(time, np.degrees(df["Roll desired [rad]"]), "r--", label="Desired Roll")
axs2[0].plot(time, np.degrees(df["Roll [rad]"]), "b-", label="Real Roll")
axs2[0].set_ylabel("Roll [°]")
axs2[0].legend()
axs2[0].grid(True)

axs2[1].plot(time, np.degrees(df["Pitch desired [rad]"]), "r--", label="Desired Pitch")
axs2[1].plot(time, np.degrees(df["Pitch [rad]"]), "b-", label="Real Pitch")
axs2[1].set_ylabel("Pitch [°]")
axs2[1].grid(True)

axs2[2].plot(time, np.degrees(df["User-defined yaw [rad]"]), "r--", label="Yaw Target")
axs2[2].plot(time, np.degrees(df["Yaw [rad]"]), "b-", label="Yaw Real")
axs2[2].set_ylabel("Yaw [°]")
axs2[2].set_xlabel("Time [s]")
axs2[2].grid(True)

# ==============================================================================
# FIGURE 3: INTERNAL COMPONENTS OF THE PID (Outer Loop vs Inner Loop)
# ==============================================================================
fig3, axs3 = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
fig3.suptitle("DIAGNOSTIC 3: Analysis of PID Controller Terms (X, Y, Z)", fontsize=14, fontweight='bold')

# Outer Loop (Position -> Desired Angles)
axs3[0, 0].plot(time, df["Outer loop Proportional term x [-]"], label="P")
axs3[0, 0].plot(time, df["Outer loop Integral term x [-]"], label="I")
axs3[0, 0].plot(time, df["Outer loop Derivative term x [-]"], label="D (raw)")
if has_filtered_D:
    axs3[0, 0].plot(time, df["Outer loop Derivative term Filtered x [-]"], label="D (filtered)")
axs3[0, 0].set_title("Outer Loop - Asse X")
axs3[0, 0].legend()
axs3[0, 0].grid(True)

axs3[0, 1].plot(time, df["Outer loop Proportional term y [-]"], label="P")
axs3[0, 1].plot(time, df["Outer loop Integral term y [-]"], label="I")
axs3[0, 1].plot(time, df["Outer loop Derivative term y [-]"], label="D (raw)")
if has_filtered_D:
    axs3[0, 1].plot(time, df["Outer loop Derivative term Filtered y [-]"], label="D (filtered)")
axs3[0, 1].set_title("Outer Loop - Y Axis")
axs3[0, 1].grid(True)

axs3[0, 2].plot(time, df["Outer loop Proportional term z [-]"], label="P")
axs3[0, 2].plot(time, df["Outer loop Integral term z [-]"], label="I")
axs3[0, 2].plot(time, df["Outer loop Derivative term z [-]"], label="D (raw)")
if has_filtered_D:
    axs3[0, 2].plot(time, df["Outer loop Derivative term Filtered z [-]"], label="D (filtered)")
axs3[0, 2].set_title("Outer Loop - Z Axis")
axs3[0, 2].grid(True)

# Inner Loop (Angles -> Control Moments of the Rotors)
axs3[1, 0].plot(time, df["Inner loop Proportional term x [-]"], label="P")
axs3[1, 0].plot(time, df["Inner loop Integral term x [-]"], label="I")
axs3[1, 0].plot(time, df["Inner loop Derivative term x [-]"], label="D")
axs3[1, 0].set_title("Inner Loop - Roll Axis (p)")
axs3[1, 0].set_xlabel("Time [s]")
axs3[1, 0].grid(True)

axs3[1, 1].plot(time, df["Inner loop Proportional term y [-]"], label="P")
axs3[1, 1].plot(time, df["Inner loop Integral term y [-]"], label="I")
axs3[1, 1].plot(time, df["Inner loop Derivative term y [-]"], label="D")
axs3[1, 1].set_title("Inner Loop - Pitch Axis (q)")
axs3[1, 1].set_xlabel("Time [s]")
axs3[1, 1].grid(True)

axs3[1, 2].plot(time, df["Inner loop Proportional term z [-]"], label="P")
axs3[1, 2].plot(time, df["Inner loop Integral term z [-]"], label="I")
axs3[1, 2].plot(time, df["Inner loop Derivative term z [-]"], label="D")
axs3[1, 2].set_title("Inner Loop - Yaw Axis (r)")
axs3[1, 2].set_xlabel("Time [s]")
axs3[1, 2].grid(True)

# ==============================================================================
# FIGURE 4: GENERAL CONTROL INPUTS AND REAL FORCES OF THE 8 MOTORS
# ==============================================================================
fig4, axs4 = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
fig4.suptitle("DIAGNOSTIC 4: Mixer Output and Forces Provided by the Motors X8", fontsize=14, fontweight='bold')

# Collective Thrust U1
axs4[0].plot(time, df["U control input U1 [N]"], "g-", label="Total Thrust Requested (U1)")
axs4[0].set_ylabel("Forza [N]")
axs4[0].legend()
axs4[0].grid(True)

# Required Moments U2, U3, U4
axs4[1].plot(time, df["U control input U2 [Nm]"], label="Required Roll (U2)")
axs4[1].plot(time, df["U control input U3 [Nm]"], label="Required Pitch (U3)")
axs4[1].plot(time, df["U control input U4 [Nm]"], label="Required Yaw (U4)")
axs4[1].set_ylabel("Moments [Nm]")
axs4[1].legend()
axs4[1].grid(True)

# Thrust for 8 Rotors
for i in range(1, 9):
    axs4[2].plot(time, df[f"Thrust motors X8COPTER T{i} [N]"], label=f"Motore {i}", alpha=0.7)
axs4[2].set_ylabel("Single Rotor Thrust [N]")
axs4[2].set_xlabel("Time [s]")
axs4[2].legend(ncol=4, loc="upper right")
axs4[2].grid(True)

# ==============================================================================
# FIGURE 5: NORMS OF THE ADAPTIVE GAINS (solo per controllori MRAC-based)
# ==============================================================================
# Data extraction depends on the selected CONTROLLER_TYPE in docker-compose.yml:
# PID has no adaptive gains.
ADAPTIVE_MATRIX_NAMES = ["K_hat_x", "K_hat_r", "Theta_hat", "K_hat_g"]

def matrix_norm(df, prefix):
    cols = [c for c in df.columns if c.startswith(prefix + " index-")]
    if not cols:
        return None
    return np.linalg.norm(df[cols].to_numpy(), axis=1)

adaptive_norms = {"translational": {}, "rotational": {}}
for loop in adaptive_norms:
    for name in ADAPTIVE_MATRIX_NAMES:
        norm = matrix_norm(df, f"{name}_{loop}")
        if norm is not None:
            adaptive_norms[loop][name] = norm

proj_op_cols = [c for c in df.columns if c.startswith("proj_op_activated_")]

if any(adaptive_norms["translational"]) or any(adaptive_norms["rotational"]):
    fig5, axs5 = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig5.suptitle("DIAGNOSTIC 5: Adaptive Gains (Frobenius Norm) and Projection Operator",
                   fontsize=14, fontweight='bold')

    for name, norm in adaptive_norms["translational"].items():
        axs5[0].plot(time, norm, label=name)
    axs5[0].set_ylabel("||·||_F Translational")
    axs5[0].set_yscale("log")
    axs5[0].legend()
    axs5[0].grid(True)

    for name, norm in adaptive_norms["rotational"].items():
        axs5[1].plot(time, norm, label=name)
    axs5[1].set_ylabel("||·||_F Rotational")
    axs5[1].set_yscale("log")
    axs5[1].legend()
    axs5[1].grid(True)

    for col in proj_op_cols:
        axs5[2].step(time, df[col], where="post", label=col.replace("proj_op_activated_", ""), alpha=0.8)
    axs5[2].set_ylabel("Projection Operator\nActivated [0/1]")
    axs5[2].set_xlabel("Tempo [s]")
    axs5[2].set_yticks([0, 1])
    axs5[2].legend(fontsize=7, ncol=2)
    axs5[2].grid(True)

# ==============================================================================
# FIGURE 6: FUNNEL CONDITION (e^T*M*e vs eta_max^2 - eta_funnel^2)
# ==============================================================================
# Only for FunnelTwoLayerMRAC. H_funnel_translational = (eta_max - eta_funnel^2) - e^T*M*e
# It is already logged, so e^T*M*e is obtained by difference, without needing new C++ logging.
# eta_max_funnel_translational is a constant (not logged) taken from the gains file.
if "eta_funnel_translational [-]" in df.columns and "H_funnel_translational [-]" in df.columns:
    gains_path = os.path.join(
        "src", "flightstack", "params", "control", "funnel_two_layer_mrac",
        "gains_funnel_two_layer_mrac.json"
    )
    with open(gains_path, "r") as f:
        eta_max_funnel_translational = json.load(f)["ADAPTIVE"]["eta_max_funnel_translational"]

    eta_funnel = df["eta_funnel_translational [-]"]
    H_function = df["H_funnel_translational [-]"]
    funnel_diameter = eta_max_funnel_translational - eta_funnel ** 2  # eta_max^2 - eta_funnel^2
    eT_M_e = funnel_diameter - H_function  # H = funnel_diameter - e^T*M*e

    fig6, axs6 = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig6.suptitle("DIAGNOSTIC 6: Funnel Condition (FunnelTwoLayerMRAC)",
                   fontsize=14, fontweight='bold')

    axs6[0].plot(time, eT_M_e, "b-", label=r"$e^T M e$")
    axs6[0].plot(time, funnel_diameter, "r--", label=r"$\eta_{max}^2 - \eta_{funnel}^2$")
    axs6[0].set_ylabel("Value")
    axs6[0].legend()
    axs6[0].grid(True)

    axs6[1].plot(time, H_function, "g-", label=r"$H = \eta_{max}^2 - \eta_{funnel}^2 - e^T M e$")
    axs6[1].axhline(0.0, color="k", linewidth=0.8)
    axs6[1].set_ylabel("H Margin")
    axs6[1].set_xlabel("Time [s]")
    axs6[1].legend()
    axs6[1].grid(True)

# ==============================================================================
# FIGURE 7: QUATERNION, ANGULAR VELOCITY, TIMING (common to all controllers)
# ==============================================================================
fig7, axs7 = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
fig7.suptitle("DIAGNOSTIC 7: Quaternion, Angular Velocity, Timing", fontsize=14, fontweight='bold')

axs7[0].plot(time, df["Quaternion q0 [-]"], label="q0")
axs7[0].plot(time, df["Quaternion q1 [-]"], label="q1")
axs7[0].plot(time, df["Quaternion q2 [-]"], label="q2")
axs7[0].plot(time, df["Quaternion q3 [-]"], label="q3")
axs7[0].set_ylabel("Quaternion [-]")
axs7[0].legend()
axs7[0].grid(True)

axs7[1].plot(time, df["Angular velocity x [rad/s]"], label="p (x)")
axs7[1].plot(time, df["Angular velocity y [rad/s]"], label="q (y)")
axs7[1].plot(time, df["Angular velocity z [rad/s]"], label="r (z)")
axs7[1].set_ylabel("Angular Velocity [rad/s]")
axs7[1].legend()
axs7[1].grid(True)

axs7[2].plot(time, df["Algorithm execution time [us]"], "k-")
axs7[2].set_ylabel("Execution Time [us]")
axs7[2].set_xlabel("Time [s]")
axs7[2].grid(True)

# ==============================================================================
# FIGURE 8: ATTITUDE ERRORS AND EULER ANGULAR VELOCITIES (common to all controllers)
# ==============================================================================
fig8, axs8 = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
fig8.suptitle("DIAGNOSTIC 8: Attitude Errors and Euler Angular Velocities", fontsize=14, fontweight='bold')

axs8[0].plot(time, np.degrees(df["Roll error [rad]"]), label="Roll error")
axs8[0].plot(time, np.degrees(df["Pitch error [rad]"]), label="Pitch error")
axs8[0].plot(time, np.degrees(df["Yaw error [rad]"]), label="Yaw error")
axs8[0].set_ylabel("Error [°]")
axs8[0].legend()
axs8[0].grid(True)

axs8[1].plot(time, np.degrees(df["Roll_dot desired [rad/s]"]), "r--", label="Roll_dot desired")
axs8[1].plot(time, np.degrees(df["Roll_dot [rad/s]"]), "r-", label="Roll_dot real")
axs8[1].plot(time, np.degrees(df["Pitch_dot desired [rad/s]"]), "g--", label="Pitch_dot desired")
axs8[1].plot(time, np.degrees(df["Pitch_dot [rad/s]"]), "g-", label="Pitch_dot real")
axs8[1].plot(time, np.degrees(df["User-defined yaw_dot [rad/s]"]), "b--", label="Yaw_dot target")
axs8[1].plot(time, np.degrees(df["Yaw_dot [rad/s]"]), "b-", label="Yaw_dot real")
axs8[1].set_ylabel("Angular Velocity [°/s]")
axs8[1].set_xlabel("Time [s]")
axs8[1].legend(ncol=3, fontsize=8)
axs8[1].grid(True)

# ==============================================================================
# FIGURE 9: RAW CONTROL FORCE AND DYNAMIC INVERSION TERMS
# ==============================================================================
has_mu_raw_local = "Mu translational raw local x [N]" in df.columns
fig9, axs9 = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
fig9.suptitle("DIAGNOSTIC 9: Raw Control Force and Dynamic Inversion Terms", fontsize=14, fontweight='bold')

for axis in ("x", "y", "z"):
    axs9[0].plot(time, df[f"Mu translational raw global {axis} [N]"], label=f"Mu raw global {axis}")
for axis in ("x", "y", "z"):
    axs9[0].plot(time, df[f"Mu translational local {axis} [N]"], "--", label=f"Mu local {axis}")
if has_mu_raw_local:
    for axis in ("x", "y", "z"):
        axs9[0].plot(time, df[f"Mu translational raw local {axis} [N]"], ":", label=f"Mu raw local {axis}")
axs9[0].set_ylabel("Force [N]")
axs9[0].legend(ncol=3, fontsize=7)
axs9[0].grid(True)

axs9[1].plot(time, df["Outer loop Dynamic Inversion term x [-]"], label="DI x")
axs9[1].plot(time, df["Outer loop Dynamic Inversion term y [-]"], label="DI y")
axs9[1].plot(time, df["Outer loop Dynamic Inversion term z [-]"], label="DI z")
axs9[1].set_ylabel("Outer Loop DI [-]")
axs9[1].legend()
axs9[1].grid(True)

axs9[2].plot(time, df["Inner loop Dynamic Inversion term x [-]"], label="DI roll")
axs9[2].plot(time, df["Inner loop Dynamic Inversion term y [-]"], label="DI pitch")
axs9[2].plot(time, df["Inner loop Dynamic Inversion term z [-]"], label="DI yaw")
axs9[2].set_ylabel("Inner Loop DI [-]")
axs9[2].set_xlabel("Time [s]")
axs9[2].legend()
axs9[2].grid(True)

# ==============================================================================
# FIGURE 10: DETAIL OF MOTOR MIXER (normalized thrusts X8 and QUADCOPTER)
# ==============================================================================
fig10, axs10 = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
fig10.suptitle("DIAGNOSTIC 10: Detail of Motor Mixer", fontsize=14, fontweight='bold')

for i in range(1, 9):
    axs10[0].plot(time, df[f"Thrust motors normalized X8COPTER T{i} [-]"], label=f"Motor {i}", alpha=0.7)
axs10[0].set_ylabel("Normalized Thrust X8 [-]")
axs10[0].legend(ncol=4, loc="upper right", fontsize=7)
axs10[0].grid(True)

for i in range(1, 5):
    axs10[1].plot(time, df[f"Thrust motors QUADCOPTER T{i} [N]"], label=f"Motor {i}")
axs10[1].set_ylabel("Thrust QUADCOPTER [N]")
axs10[1].set_xlabel("Time [s]")
axs10[1].legend()
axs10[1].grid(True)

# ==============================================================================
# FIGURE 11: REFERENCE MODEL MRAC AND FORCE DECOMPOSITION
# ==============================================================================
# Present in MRAC, TwoLayerMRAC, HybridTwoLayerMRAC, FunnelTwoLayerMRAC.
if "K_hat_x_translational index-00 [-]" in df.columns:
    fig11, axs11 = plt.subplots(4, 2, figsize=(15, 13), sharex=True)
    fig11.suptitle("DIAGNOSTIC 11: Reference Model MRAC and Force Decomposition",
                    fontsize=14, fontweight='bold')

    for axis, color in zip(("x", "y", "z"), ("r", "g", "b")):
        axs11[0, 0].plot(time, df[f"Translational reference model position {axis} [m]"],
                          f"{color}--", label=f"Ref {axis}")
        axs11[0, 0].plot(time, df[f"Position {axis} [m]"], f"{color}-", label=f"Real {axis}")
    axs11[0, 0].set_ylabel("Position [m]")
    axs11[0, 0].legend(ncol=2, fontsize=7)
    axs11[0, 0].grid(True)

    for axis, color in zip(("x", "y", "z"), ("r", "g", "b")):
        axs11[0, 1].plot(time, df[f"Translational reference model velocity {axis} [m/s]"],
                          f"{color}--", label=f"Ref {axis}")
        axs11[0, 1].plot(time, df[f"Velocity {axis} [m/s]"], f"{color}-", label=f"Real {axis}")
    axs11[0, 1].set_ylabel("Velocity [m/s]")
    axs11[0, 1].legend(ncol=2, fontsize=7)
    axs11[0, 1].grid(True)

    for axis in ("x", "y", "z"):
        axs11[1, 0].plot(time, df[f"mu_PID_baseline_translational {axis} [N]"], label=axis)
    axs11[1, 0].set_ylabel("mu_PID_baseline_translational [N]")
    axs11[1, 0].legend()
    axs11[1, 0].grid(True)

    for axis in ("x", "y", "z"):
        axs11[1, 1].plot(time, df[f"mu_adaptive_translational {axis} [N]"], label=axis)
    axs11[1, 1].set_ylabel("mu_adaptive_translational [N]")
    axs11[1, 1].legend()
    axs11[1, 1].grid(True)

    for axis, color in zip(("x", "y", "z"), ("r", "g", "b")):
        axs11[2, 0].plot(time, df[f"Rotational reference model omega {axis} [rad/s]"],
                          f"{color}--", label=f"Ref omega {axis}")
        axs11[2, 0].plot(time, df[f"Angular velocity {axis} [rad/s]"], f"{color}-", label=f"Real omega {axis}")
    axs11[2, 0].set_ylabel("Omega [rad/s]")
    axs11[2, 0].legend(ncol=2, fontsize=7)
    axs11[2, 0].grid(True)

    for axis in ("x", "y", "z"):
        axs11[2, 1].plot(time, df[f"omega_cmd_rotational {axis} [rad/s]"], label=axis)
    axs11[2, 1].set_ylabel("omega_cmd_rotational [rad/s]")
    axs11[2, 1].legend()
    axs11[2, 1].grid(True)

    for axis in ("x", "y", "z"):
        axs11[3, 0].plot(time, df[f"tau_PID_baseline_rotational {axis} [Nm]"], label=axis)
    axs11[3, 0].set_ylabel("tau_PID_baseline_rotational [Nm]")
    axs11[3, 0].legend()
    axs11[3, 0].grid(True)

    for axis in ("x", "y", "z"):
        axs11[3, 1].plot(time, df[f"tau_adaptive_rotational {axis} [Nm]"], label=axis)
    axs11[3, 1].set_ylabel("tau_adaptive_rotational [Nm]")
    axs11[3, 0].set_xlabel("Time [s]")
    axs11[3, 1].set_xlabel("Time [s]")
    axs11[3, 1].legend()
    axs11[3, 1].grid(True)

# ==============================================================================
# FIGURE 11b: DEAD ZONE vs TRACKING ERROR (norm 2)
# ==============================================================================
# The dead zone of the adaptation activates based on the tracking error with respect
# to the reference model: translational error = [pos-pos_ref, vel-vel_ref] (6 components),
# rotational error = omega - omega_ref (3 components, the only state of the reference
# rotational model logged).
def load_run_gains(log_file_path):
    """Loads the gains-*.json copied next to this specific log run (sibling 'gains' dir, same timestamp)."""
    match = re.search(r"(\d{8}_\d{6})\.log$", os.path.basename(log_file_path))
    if not match:
        return None
    run_dir = os.path.dirname(os.path.dirname(log_file_path))
    candidates = glob.glob(os.path.join(run_dir, "gains", f"*_{match.group(1)}.json"))
    if not candidates:
        return None
    with open(candidates[0], "r") as f:
        return json.load(f)


if "K_hat_x_translational index-00 [-]" in df.columns:
    translational_tracking_error_norm = np.sqrt(
        (df["Position x [m]"] - df["Translational reference model position x [m]"]) ** 2
        + (df["Position y [m]"] - df["Translational reference model position y [m]"]) ** 2
        + (df["Position z [m]"] - df["Translational reference model position z [m]"]) ** 2
        + (df["Velocity x [m/s]"] - df["Translational reference model velocity x [m/s]"]) ** 2
        + (df["Velocity y [m/s]"] - df["Translational reference model velocity y [m/s]"]) ** 2
        + (df["Velocity z [m/s]"] - df["Translational reference model velocity z [m/s]"]) ** 2
    )
    rotational_tracking_error_norm = np.sqrt(
        (df["Angular velocity x [rad/s]"] - df["Rotational reference model omega x [rad/s]"]) ** 2
        + (df["Angular velocity y [rad/s]"] - df["Rotational reference model omega y [rad/s]"]) ** 2
        + (df["Angular velocity z [rad/s]"] - df["Rotational reference model omega z [rad/s]"]) ** 2
    )

    # Freeze threshold = delta * e0: below this error norm the adaptive law dead-zone
    # modulation function is identically zero (adaptation frozen). See deadZoneModulationFunction
    # in control.hpp: max(0, min(1, (||e|| - delta*e0) / ((1-delta)*e0))).
    run_gains = load_run_gains(log_file_path)
    freeze_threshold_translational = None
    freeze_threshold_rotational = None
    if run_gains is not None and "ADAPTIVE" in run_gains:
        adaptive_gains = run_gains["ADAPTIVE"]
        if "dead_zone_delta_translational" in adaptive_gains and "dead_zone_e0_translational" in adaptive_gains:
            freeze_threshold_translational = (adaptive_gains["dead_zone_delta_translational"]
                                               * adaptive_gains["dead_zone_e0_translational"])
        if "dead_zone_delta_rotational" in adaptive_gains and "dead_zone_e0_rotational" in adaptive_gains:
            freeze_threshold_rotational = (adaptive_gains["dead_zone_delta_rotational"]
                                            * adaptive_gains["dead_zone_e0_rotational"])

    fig11b, axs11b = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig11b.suptitle("DIAGNOSTIC 11b: Tracking Error (norm 2) vs Adaptation Freeze Threshold",
                     fontsize=14, fontweight='bold')

    axs11b[0].plot(time, rotational_tracking_error_norm, "r-", label=r"$\|\omega - \omega_{ref}\|_2$")
    if freeze_threshold_rotational is not None:
        axs11b[0].axhline(freeze_threshold_rotational, color="k", linestyle="--",
                           label=f"Freeze threshold = delta*e0 = {freeze_threshold_rotational:.4g}")
    axs11b[0].set_ylabel(r"$\|\omega - \omega_{ref}\|_2$ [rad/s]")
    axs11b[0].set_title("Rotational")
    axs11b[0].legend(loc="upper right")
    axs11b[0].grid(True)

    axs11b[1].plot(time, translational_tracking_error_norm, "r-", label=r"$\|[x,v] - [x,v]_{ref}\|_2$")
    if freeze_threshold_translational is not None:
        axs11b[1].axhline(freeze_threshold_translational, color="k", linestyle="--",
                           label=f"Freeze threshold = delta*e0 = {freeze_threshold_translational:.4g}")
    axs11b[1].set_ylabel(r"$\|[x,v] - [x,v]_{ref}\|_2$")
    axs11b[1].set_title("Translational")
    axs11b[1].set_xlabel("Time [s]")
    axs11b[1].legend(loc="upper right")
    axs11b[1].grid(True)

# ==============================================================================
# FIGURE 12: SAFETY MECHANISM (TwoLayerMRAC, HybridTwoLayerMRAC, FunnelTwoLayerMRAC)
# ==============================================================================
if "Safety Mechanism tSphere [-]" in df.columns:
    fig12, axs12 = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig12.suptitle("DIAGNOSTIC 12: Safety Mechanism", fontsize=14, fontweight='bold')

    axs12[0].plot(time, df["Safety Mechanism tSphere [-]"], label="tSphere")
    axs12[0].plot(time, df["Safety Mechanism tEllipticCone [-]"], label="tEllipticCone")
    axs12[0].plot(time, df["Safety Mechanism tPlane [-]"], label="tPlane")
    axs12[0].plot(time, df["Safety Mechanism tPrime [-]"], label="tPrime")
    axs12[0].set_ylabel("Value [-]")
    axs12[0].legend()
    axs12[0].grid(True)

    axs12[1].step(time, df["Safety Mechanism safe_mech_activated [-]"], where="post")
    axs12[1].set_ylabel("Safe Mech Activated [0/1]")
    axs12[1].set_xlabel("Time [s]")
    axs12[1].set_yticks([0, 1])
    axs12[1].grid(True)

# ==============================================================================
# FIGURE 13: INTERNAL HybridTwoLayerMRAC
# ==============================================================================
if "s_hybrid_translational [-]" in df.columns:
    fig13, axs13 = plt.subplots(2, 3, figsize=(16, 8), sharex=True)
    fig13.suptitle("DIAGNOSTIC 13: Internal HybridTwoLayerMRAC", fontsize=14, fontweight='bold')

    axs13[0, 0].plot(time, df["eQe_translational [-]"], label="eQe")
    axs13[0, 0].plot(time, df["integral_eQe_translational [-]"], label="integral_eQe")
    axs13[0, 0].set_title("eQe / integral_eQe")
    axs13[0, 0].legend()
    axs13[0, 0].grid(True)

    axs13[0, 1].plot(time, df["summation_hybrid_P_translational [-]"], label="summation_hybrid_P")
    axs13[0, 1].plot(time, df["s_hybrid_translational [-]"], label="s_hybrid")
    axs13[0, 1].set_title("summation_hybrid_P / s_hybrid")
    axs13[0, 1].legend()
    axs13[0, 1].grid(True)

    axs13[0, 2].step(time, df["time_of_last_trajectory_reset_translational [-]"], where="post")
    axs13[0, 2].set_title("time_of_last_trajectory_reset")
    axs13[0, 2].grid(True)

    for axis in ("x", "y", "z"):
        axs13[1, 0].plot(time, df[f"e_translational {axis} [m]"], label=f"e {axis}")
        axs13[1, 0].plot(time, df[f"e_previous_translational {axis} [m]"], "--", label=f"e_prev {axis}")
    axs13[1, 0].set_title("e_translational pos vs e_previous")
    axs13[1, 0].set_xlabel("Time [s]")
    axs13[1, 0].legend(ncol=2, fontsize=7)
    axs13[1, 0].grid(True)

    for axis in ("vx", "vy", "vz"):
        axs13[1, 1].plot(time, df[f"e_translational {axis} [m/s]"], label=f"e {axis}")
        axs13[1, 1].plot(time, df[f"e_previous_translational {axis} [m/s]"], "--", label=f"e_prev {axis}")
    axs13[1, 1].set_title("e_translational vel vs e_previous")
    axs13[1, 1].set_xlabel("Time [s]")
    axs13[1, 1].legend(ncol=2, fontsize=7)
    axs13[1, 1].grid(True)

    for axis in ("x", "y", "z"):
        axs13[1, 2].plot(time, df[f"mu_adaptive_ebci_translational {axis} [N]"], label=f"ebci {axis}")
        axs13[1, 2].plot(time, df[f"mu_adaptive_mrac_translational {axis} [N]"], "--", label=f"mrac {axis}")
    axs13[1, 2].set_title("mu_adaptive ebci vs mrac")
    axs13[1, 2].set_xlabel("Time [s]")
    axs13[1, 2].legend(ncol=2, fontsize=7)
    axs13[1, 2].grid(True)

# ==============================================================================
# EXTENSION FIGURE 6: additional internal variables FunnelTwoLayerMRAC
# ==============================================================================
if "eta_dot_funnel_translational [-]" in df.columns:
    fig6b, axs6b = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig6b.suptitle("DIAGNOSTIC 6b: Internal Variables of the Funnel (FunnelTwoLayerMRAC)",
                    fontsize=14, fontweight='bold')

    axs6b[0].plot(time, df["eta_funnel_translational [-]"], label=r"$\eta$")
    axs6b[0].plot(time, df["eta_dot_funnel_translational [-]"], label=r"$\dot{\eta}$")
    axs6b[0].set_ylabel("eta_funnel")
    axs6b[0].legend()
    axs6b[0].grid(True)

    axs6b[1].plot(time, df["xi_funnel_translational [-]"], label=r"$\xi$")
    axs6b[1].plot(time, df["Ve_funnel_translational [-]"], label=r"$V_e$")
    axs6b[1].plot(time, df["lambda_sat_funnel_translational [-]"], label=r"$\lambda_{sat}$")
    axs6b[1].plot(time, df["sigma_nom_funnel_translational [-]"], label=r"$\sigma_{nom}$")
    axs6b[1].plot(time, df["sigma_ideal_funnel_translational [-]"], label=r"$\sigma_{ideal}$")
    axs6b[1].plot(time, df["case_eta_dot_funnel_translational [-]"], label="case_eta_dot")
    axs6b[1].set_ylabel("Internal Variables Funnel [-]")
    axs6b[1].legend(ncol=3, fontsize=7)
    axs6b[1].grid(True)

    for axis in ("x", "y", "z"):
        axs6b[2].plot(time, df[f"Translational tracking error derivative {axis} [m/s]"], label=f"e_dot {axis}")
    for axis in ("vx", "vy", "vz"):
        axs6b[2].plot(time, df[f"Translational tracking error derivative {axis} [m/s^2]"], "--", label=f"e_dot {axis}")
    axs6b[2].set_ylabel("Tracking Error Derivative")
    axs6b[2].set_xlabel("Time [s]")
    axs6b[2].legend(ncol=3, fontsize=7)
    axs6b[2].grid(True)

plt.tight_layout()
plt.show()
