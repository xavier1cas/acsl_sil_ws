# Software-In-the-Loop (SIL) Infrastructure

_Author: Giorgio A. Orlando - 2026-07-24_

This document describes the files that make up the SIL infrastructure itself -
everything that glues the `flightstack` and `simulator` packages together,
**outside of `src/flightstack/` and `src/simulator/`**.

Changes made *inside* those two packages to support SIL are documented in their
own changelogs, not here:
- [`src/flightstack/FLIGHTSTACK_CHANGES.md`](src/flightstack/FLIGHTSTACK_CHANGES.md)
- [`src/simulator/SIMULATOR_CHANGES.md`](src/simulator/SIMULATOR_CHANGES.md)

For a full step-by-step build/setup walkthrough, see
[`ACSL_SIL_Flightstack_Guide.md`](ACSL_SIL_Flightstack_Guide.md).

## `src/uav_interfaces/` - shared ROS2 service definition

New ROS2 package, built before both `flightstack` and `simulator` since they
each depend on the message types it generates.

- **`srv/ComputeControl.srv`** - Defines the `/sil/compute_control` service
  contract: the simulator sends time, pose (position + quaternion), linear
  velocity, and angular velocity as the request; the flightstack returns
  `motor_commands` (one dimensional thrust value per motor) as the response.
  This is the only data contract between the two processes - the simulator
  never talks to flightstack internals directly.
- **`package.xml`** / **`CMakeLists.txt`** - Standard `ament_cmake` /
  `rosidl_default_generators` package boilerplate that turns the `.srv` file
  into generated C++ and Python message/service bindings, exported via
  `rosidl_default_runtime` so downstream packages (`flightstack`, `simulator`)
  can `find_package(uav_interfaces)`.

## `docker-compose.yml` - orchestration

Defines and wires together the two containers:

- **`flightstack` service** - builds from `docker/flightstack.Dockerfile`,
  mounts `src/flightstack` and `src/uav_interfaces`, and exposes a
  healthcheck that polls `ros2 service list` for `/sil/compute_control`
  before the simulator is allowed to start. `CONTROLLER_TYPE` is read from
  the shared `x-controller-type` anchor at the top of the file so both
  containers always agree on which controller (`PID`/`MRAC`/`TwoLayerMRAC`/
  `FunnelTwoLayerMRAC`/`HybridTwoLayerMRAC`) is active.
- **`simulator` service** - builds from `docker/simulator.Dockerfile`, mounts
  `src/simulator` and `src/uav_interfaces` (read-only mount of
  `flightstack/params/config/config.json` so it can see which trajectory is
  selected), forwards WSLg's X11/D3D12 GPU passthrough paths for visualization,
  and only starts once `flightstack` reports healthy
  (`depends_on: condition: service_healthy`).
- Both services run with `network_mode: host` and share `ROS_DOMAIN_ID` /
  the `fastdds_no_shm.xml` profile so ROS2 DDS discovery works across the two
  containers.
- **`ga_tuner` service** - runs the GA gain-tuner against the real SIL
  flightstack (see
  [`src/simulator/acsl_pychrono/ga_tuner/GA_TUNER_SIL.md`](src/simulator/acsl_pychrono/ga_tuner/GA_TUNER_SIL.md)
  for the full picture). `profiles: ["tools"]` keeps it out of a bare
  `docker compose up`; invoke explicitly with `docker compose run --rm
  ga_tuner`. Builds from `docker/simulator.Dockerfile` (needs `rclpy` +
  PyChrono), mounts `/var/run/docker.sock` and `${PWD}:${PWD}`
  (docker-outside-of-docker, so the `docker compose ...` calls this
  container issues internally control the host's `flightstack` container
  the same way they would from the host shell) and sets
  `SIL_REPO_ROOT=${PWD}` so the GA's repo-root discovery resolves paths
  correctly from inside the container. Runs
  `ga_tuner/run_tuner_sil.py`, which restarts `flightstack` with freshly
  written gains before every candidate simulation.

## `docker/` - container build context

- **`flightstack.Dockerfile`** - `ros:jazzy` base; installs system Boost
  (`libboost-all-dev`) to replace the bundled source-only Boost tree, builds
  `uav_interfaces` first, then builds `flightstack` with
  `-DSIL_MODE=ON -DSELECTED_CONTROLLER=${CONTROLLER_TYPE}`. The container's
  `CMD` rebuilds `flightstack` on every start (single-threaded, `MAKEFLAGS=-j1`,
  to bound RAM usage) so that changing `CONTROLLER_TYPE` in `docker-compose.yml`
  takes effect without rebuilding the image, then runs the flightstack node.
- **`simulator.Dockerfile`** - `ros:jazzy` base; installs Miniconda, creates a
  `chrono_v9` conda environment (Python 3.12) with the official PyChrono
  9.0.1 package plus `numpy`/`matplotlib`/`scipy`/`pyyaml`/`deap`/`pymoo`/etc.,
  and installs `docker/simulator_entrypoint.sh` as the container entrypoint.
- **`simulator_entrypoint.sh`** - Runs at container start: builds only the
  `uav_interfaces` package against system Python, sources the ROS2 + workspace
  overlays, activates the `chrono_v9` conda environment, and merges
  `PYTHONPATH` across PyChrono, conda site-packages, and the ROS2/workspace
  Python paths before `exec`-ing the container's `command:` (the simulator's
  `main.py`).
- **`fastdds_no_shm.xml`** - FastDDS profile that forces UDP-only transport
  instead of the default shared-memory transport, which doesn't work across
  containers since each has its own `/dev/shm`. Mounted read-only into both
  containers and pointed to via `FASTRTPS_DEFAULT_PROFILES_FILE`.

## Root-level diagnostic scripts

These are standalone, locally-run (not containerized) Python scripts for
inspecting log output after a SIL run; none of them are part of the
`flightstack` or `simulator` ROS2 packages.

- **`plot_logs.py`** - Parses a single flightstack `.log` file (defaults to
  the most recently modified one under `src/flightstack/log/`) into a
  `pandas` DataFrame and renders a series of `matplotlib` diagnostic figures
  (kinematic tracking, Euler angles, PID/MRAC controller terms, motor mixer
  output, adaptive gains / projection operator norms, funnel condition,
  safety mechanism, etc.) - figures are controller-type-dependent, so MRAC-only
  plots are skipped for a PID log and vice versa.
- **`plot_logs_sim.py`** - Compares a simulator `.mat` log against a
  flightstack `.log` for the same run: position/velocity/attitude/angular
  velocity tracking (sim vs. flightstack), motor thrusts (sent vs. applied vs.
  computed), adaptive gains, and SIL bridge round-trip timing diagnostics
  (`src/simulator/logs/sil_bridge_diagnostics.csv`). Can run flightstack-only
  or simulator-only via `--no-flightstack`/`--sil-bridge` flags.
- **`log_compare.py`** - Overlays multiple flightstack controller runs
  (e.g. `TwoLayerMRAC` vs. `FunnelTwoLayerMRAC` vs. `HybridTwoLayerMRAC`) on
  the same plots for side-by-side comparison. Each entry in `CONTROLLERS` auto-
  picks the latest non-empty log for that controller under
  `src/flightstack/log/*/<ControllerName>/logs/`, or can pin an explicit file
  via `"ControllerName=path/to/log.log"`.

## `.vscode/`

Editor configuration only (`c_cpp_properties.json`, `launch.json`,
`settings.json`) - not part of the SIL runtime, kept here for IntelliSense
and debug configuration across the multi-package workspace.
