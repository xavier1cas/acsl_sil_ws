# SIL Usage Guide - Running Simulations, Selecting Trajectories, Tuning Gains

_Author: Giorgio A. Orlando - 2026-07-24_

This document is the day-to-day operator's manual for the already-built SIL
stack: what to edit to run a different scenario, where to look at results,
and how (and whether) to automate gain tuning.

- For the one-time setup that produces this stack, see
  [`ACSL_SIL_Flightstack_Guide.md`](ACSL_SIL_Flightstack_Guide.md).
- For what each file in the SIL plumbing is/does, see [`SIL.md`](SIL.md).

Everything below assumes you're in `~/projects/acsl_sil_ws` with the stack
already built (`docker compose build` has been run at least once).

## 1. Running a single simulation

```bash
docker compose up
```

Three independent knobs control what that run does. All three are read
fresh on every `docker compose up` - none require an image rebuild
(`flightstack`'s `CMD` recompiles from the bind-mounted source, and
`config.json`/gain files are read at runtime).

### 1.1 Controller - `docker-compose.yml`

```yaml
x-controller-type: &controller_type "TwoLayerMRAC"
```

One anchor at the top of `docker-compose.yml`, consumed by both services
(`CONTROLLER_TYPE` env var), so `flightstack` and `simulator` can never
disagree on which controller is active. Valid values: `PID`, `MRAC`,
`TwoLayerMRAC`, `FunnelTwoLayerMRAC`, `HybridTwoLayerMRAC` (anything else
fails the CMake configure step inside the container). Changing this forces a
recompile of the affected translation units on the next `docker compose up`
(see Part 8 of the setup guide) - expect the healthcheck to take longer than
a params-only change.

### 1.2 Trajectory (and mission timing) - `src/flightstack/params/config/config.json`

```json
{
  "user_defined_trajectory_file": "stadium.json",
  "hover_after_trajectory_time_seconds": 5.0,
  "motor_failure_start_time_seconds": 10.0,
  "motor_failure_duration_time_seconds": 10.0,
  "motor_failure_efficiencies": [[1.0],[1.0],[1.0],[1.0],[1.0],[1.0],[1.0],[1.0]]
}
```

`user_defined_trajectory_file` must name a file that exists under
`src/flightstack/params/user_defined_trajectory/` (`stadium.json`,
`rollercoaster_trajectory1p2.json`, `bean_trajectory0p3.json`,
`takeoff_1meter.json`, `landing_1meter.json`, ...). This file is the single
source of truth: `docker-compose.yml` bind-mounts it read-only into the
`simulator` container too, so the simulator always renders/tracks the same
trajectory the flightstack controller is targeting - you never set the
trajectory in two places.

`motor_failure_efficiencies` is an 8x1 array (one row per motor, X8 has 8
motors) of thrust-efficiency multipliers; combine with `--apply_motor_failure
true` on the simulator command (below) to inject an actual failure during
the run.

### 1.3 Simulation-run parameters - `docker-compose.yml`, `simulator` service `command:`

```yaml
command: >
  python /ros2_ws/src/simulator/main.py
    --uav X8
    --simulation_duration 45.0
    --visualize false
    --add_payload true
    --payload_type two_steel_balls
    --drop_two_steel_balls false
    --two_steel_balls_drop_time 15.0
    --sequential_drop false
    --sequential_drop_start 15.0
    --sequential_drop_interval 10.0
    --apply_motor_failure false
    --motor_failure_time 15.0
```

Everything about *this specific run* that isn't "which trajectory" or "which
controller" lives here: UAV airframe template, wall-clock sim duration,
whether to render (`--visualize true` needs the WSLg passthrough from the
setup guide's Part 1), payload configuration, and motor-failure injection
timing. Edit in place and `docker compose up` - no rebuild needed, this is
just a process argv change.

**Rule of thumb**: controller → `docker-compose.yml` anchor; trajectory →
`config.json`; everything else about the run → `docker-compose.yml`
`command:` block.

## 2. Tuning controller gains

Each controller has its own gain file under
`src/flightstack/params/control/`:

| `CONTROLLER_TYPE` | Gain file |
|---|---|
| `PID` | `params/control/pid/gains_pid.json` |
| `MRAC` | `params/control/mrac/gains_mrac.json` |
| `TwoLayerMRAC` | `params/control/two_layer_mrac/gains_two_layer_mrac.json` |
| `FunnelTwoLayerMRAC` | `params/control/funnel_two_layer_mrac/gains_funnel_two_layer_mrac.json` |
| `HybridTwoLayerMRAC` | `params/control/hybrid_two_layer_mrac/gains_hybrid_two_layer_mrac.json` |

Open the file matching whichever controller is currently selected in
`docker-compose.yml`'s `x-controller-type`, edit gains, `docker compose up`.
Each directory also keeps a `.bak` copy of the file as it stood before the
current round of hand-tuning - worth diffing against if a tuning session
goes sideways and you want to get back to a known-good baseline.

Two other gain-adjacent files worth knowing about, both under
`params/control/`:
- `outer_loop_safety_mechanism.json` - position-loop safety envelope
  (max thrust, max roll/pitch angle). Not a controller gain, but will clip
  aggressive commands from a newly-tuned controller before they reach the
  motor mixer, which can look like a tuning bug if you forget it's there.
- `differentiator_gains.json` / `low_pass_filter.json` - shared signal
  conditioning used ahead of the controllers, independent of which
  controller is selected.

## 3. Inspecting results

After a run, logs land under `src/flightstack/log/<date>/<Controller>/logs/`
and `src/simulator/logs/`. Three root-level scripts (run locally, not in a
container - they're not part of either ROS2 package) consume them:

```bash
python plot_logs.py                       # latest flightstack .log -> diagnostic figures
python plot_logs_sim.py                    # latest sim .mat + flightstack .log, side by side
python plot_logs_sim.py --no-flightstack   # simulator-only, e.g. to sanity-check trajectory alone
python log_compare.py                      # overlay multiple controllers (edit CONTROLLERS list in the file)
```

`plot_logs.py` and `plot_logs_sim.py` figures are controller-type-dependent
(MRAC-only panels are skipped for a PID log). `log_compare.py` auto-picks the
latest non-empty log per controller under `src/flightstack/log/*/<Controller>/logs/`,
or you can pin an explicit file with `"ControllerName=path/to/log.log"` in
its `CONTROLLERS` list.

## 4. The GA tuner - pure-Python by default, real SIL on request

`src/simulator/acsl_pychrono/ga_tuner/` is a genetic-algorithm gain
optimizer (DEAP or pymoo backend, single- or multi-objective/Pareto) with its
own config split across `ga_tuner/ga_config/` (`ga_config.py` for
algorithm/population/generations, `param_config.py` for which gains to tune,
`mission_config.py` for its own trajectory/duration, `evaluator_config.py`
for inner/outer/combined fitness and parallel-worker settings). See
`ga_tuner/README.md` for the full option reference.

**By default it's pure-Python, no SIL.** The tuner's evaluator
(`ga_tuner/integration/uav_integration.py`) calls the exact same
`simulateMission()` (`acsl_pychrono/executor/simulate_mission.py`) that the
plain simulator uses. Normally that function instantiates the GA's candidate
gains into the *Python* controller reimplementations under
`acsl_pychrono/control/` (via `instantiateControllerWithGA` →
`get_controller_classes`) and evaluates entirely in-process - no ROS2, no
`flightstack`/`simulator` containers. That's why `evaluator_config.py`'s
`PARALLEL_CONFIG` (`n_workers`, process-vs-thread) works out of the box:
each worker is just another independent, stateless PyChrono + Python
controller, so pymoo/DEAP can fan them out with a process pool like any
CPU-bound optimization loop. Run via `python
acsl_pychrono/ga_tuner/run_tuner.py`.

**There is now a real, working SIL path** (`ga_tuner/integration/sil/`,
`ga_tuner/run_tuner_sil.py`), which tunes the *real C++ flightstack*
controller instead of the Python reimplementation. Earlier this document
described the existing `UAV_SIL_MODE` code path as a dead end for GA tuning,
because `FlightStackServiceBridge` only exchanges state and motor commands
over `/sil/compute_control` - it has no way to push a gain vector into the
running C++ process. That's still true of the bridge itself, but the missing
piece has since been built on top of it: **restart-per-candidate**. Before
every candidate's simulation, `SILUAVModelAdapter.run_simulation()`
(`ga_tuner/integration/sil/sil_adapter.py`) writes that candidate's gains
into `gains_pid.json` and restarts the `flightstack` container so the C++
process reads them fresh at startup
(`ga_tuner/integration/sil/docker_lifecycle.py:restart_flightstack_with_gains`),
verifying via the container's `StartedAt` timestamp that a real process
restart happened (needed because some C++ node state - e.g. the odometry
zero-offset - only resets on a fresh process). Only then does it hand off to
the normal `FlightStackServiceBridge` lock-step run.

Run it with:
```bash
docker compose run --rm ga_tuner
```
from the repo root - a dedicated `profiles: ["tools"]` compose service (not
part of a bare `docker compose up`) that mounts the Docker socket
(docker-outside-of-docker) so it can restart `flightstack` itself. Don't
also run a manual `docker compose up` at the same time; both would fight
over the one shared `flightstack` container.

**Scope limits, as of this writing**: SIL-mode GA tuning only supports
`controller_type = "PID"`, `evaluator_type = "outer"`, and forces
`parallel_config["enabled"] = False` - `create_uav_ga_tuner(sil_mode=True)`
raises if you ask for anything else. It's also inherently slower per
candidate than the pure-Python path (a container restart + healthcheck
round-trip on top of the simulation itself), and inherently serial (one
shared container, no multi-stack isolation - see the full writeup for why).

**For the full architecture, file-by-file breakdown of what's new, and
step-by-step usage**, see
[`src/simulator/acsl_pychrono/ga_tuner/GA_TUNER_SIL.md`](src/simulator/acsl_pychrono/ga_tuner/GA_TUNER_SIL.md).

**Bottom line**: for a quick gain-space search, the pure-Python path is still
the right default - it's fast and trivially parallel, but only validates the
Python controller reimplementation. Use SIL-mode GA tuning when you need the
optimizer to score candidates against the actual C++ controller that flies
the hardware; expect it to be much slower per generation in exchange for that
fidelity, and either way, confirm the winning gains with a normal `docker
compose up` SIL run before trusting them.

## 5. Other things worth knowing before your next SIL session

- **You've only run single simulations so far** - meaning `docker compose
  up` / `down` once per scenario. That's the intended per-run mode: nothing
  here batches multiple SIL runs back-to-back automatically. If you want a
  sweep across trajectories or controllers, script the sequence yourself
  (edit `config.json`/anchor, `docker compose up`, wait for healthy +
  duration, `docker compose down`, repeat) - the containers themselves don't
  support queuing runs.
- **`docker compose down` vs `restart`**: `down` tears down containers (but
  keeps the `flightstack_build`/`flightstack_install` named volumes, so the
  next `up` is still an incremental recompile, not a full rebuild). Use
  `docker compose restart flightstack` if you only want to force a
  recompile/rerun without touching the simulator container.
- **Controller-switch runs cost more than params-only runs** - see Part 8 of
  the setup guide. If you're doing a same-controller trajectory or gain
  sweep, the recompile is a near no-op each time; if you're alternating
  controllers, budget for the fuller recompile each switch.
