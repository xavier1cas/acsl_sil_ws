"""Isolated test of gain_serializer.write_pid_gains_json - no docker, no simulation.

Run from repo root:
    python3 debug_gain_writer.py

Loads gain_serializer.py directly by file path (it has no package-relative
imports), builds the exact gain_dict/tuned_parameter_names the real SIL
pipeline would pass in for the seed candidate, writes to a scratch copy of
gains_pid.json, and prints the result so you can inspect exactly what the
writer produced without anything else in the pipeline able to interfere.
"""
import importlib.util
import json
import shutil
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent

SERIALIZER_PATH = REPO_ROOT / "src/simulator/acsl_pychrono/ga_tuner/integration/sil/gain_serializer.py"
REAL_JSON = REPO_ROOT / "src/flightstack/params/control/pid/gains_pid.json"
SCRATCH_JSON = REPO_ROOT / "gains_pid.DEBUG_COPY.json"

if not SERIALIZER_PATH.exists():
    sys.exit(f"Can't find {SERIALIZER_PATH} - run this from the repo root")

spec = importlib.util.spec_from_file_location("gain_serializer", SERIALIZER_PATH)
gain_serializer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gain_serializer)

import numpy as np

# Exactly the seed vector we injected: [KP_x, KI_x, KD_x, KP_y, KI_y, KD_y, KP_z, KI_z, KD_z]
gain_dict = {
    "KP_tran": np.diag([3.75, 3.75, 4.5]),
    "KI_tran": np.diag([0.5, 0.5, 0.5]),
    "KD_tran": np.diag([2.5, 2.5, 1.75]),
    # Rotational values are irrelevant to the write since tuned_parameter_names below
    # only covers translational - included only because write_pid_gains_json requires
    # all 6 keys present in gain_dict.
    "KP_rot": np.diag([45.0, 45.0, 22.5]),
    "KI_rot": np.diag([8.0, 8.0, 4.0]),
    "KD_rot": np.diag([8.7, 11.6, 8.7]),
}

tuned_parameter_names = [
    "KP_tran_x", "KP_tran_y", "KP_tran_z",
    "KI_tran_x", "KI_tran_y", "KI_tran_z",
    "KD_tran_x", "KD_tran_y", "KD_tran_z",
]

shutil.copy(REAL_JSON, SCRATCH_JSON)
print(f"--- BEFORE ({SCRATCH_JSON.name}) ---")
print(SCRATCH_JSON.read_text())

gain_serializer.write_pid_gains_json(gain_dict, SCRATCH_JSON, tuned_parameter_names)

print(f"--- AFTER ({SCRATCH_JSON.name}) ---")
print(SCRATCH_JSON.read_text())

result = json.loads(SCRATCH_JSON.read_text())
print("--- Effective gains (scaling_coef * matrix diagonal) ---")
for key in ["KP_translational", "KI_translational", "KD_translational",
            "KP_rotational", "KI_rotational", "KD_rotational"]:
    block = result[key]
    diag = [block["matrix"][i][i] * block["scaling_coef"] for i in range(3)]
    print(f"{key}: scaling_coef={block['scaling_coef']}, effective diag={diag}")

SCRATCH_JSON.unlink()
print(f"\n(cleaned up {SCRATCH_JSON.name})")
