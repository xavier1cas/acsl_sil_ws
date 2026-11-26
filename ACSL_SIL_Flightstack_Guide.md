# ACSL Flightstack - SIL Setup Guide
## Complete Step-by-Step Instructions: From Zero to a Running `docker compose` Stack

_Author: Giorgio A. Orlando - 2026-07-24_

> **What this guide achieves**: a two-container stack, orchestrated by
> `docker compose`, running the ACSL-flightstack in Software-In-the-Loop (SIL)
> mode against the PyChrono simulator. The simulator advances physics, calls the
> ROS2 service `/sil/compute_control` once per control tick, the flightstack
> runs the chosen control algorithm (PID / MRAC / TwoLayerMRAC /
> FunnelTwoLayerMRAC / HybridTwoLayerMRAC) and returns per-motor thrusts in
> Newtons, and the simulator applies them. No PX4, no Pixhawk, no VICON
> required.
>
> For the exhaustive list of every file changed inside the two packages, see
> [`src/flightstack/FLIGHTSTACK_CHANGES.md`](src/flightstack/FLIGHTSTACK_CHANGES.md)
> and [`src/simulator/SIMULATOR_CHANGES.md`](src/simulator/SIMULATOR_CHANGES.md).
> For everything that lives *outside* those two packages (the Dockerfiles,
> `docker-compose.yml`, `uav_interfaces`, the plotting scripts), see
> [`SIL.md`](SIL.md). This guide is the narrative walkthrough that ties all
> three together.

---

## Architecture Overview

```
┌───────────────────────────────────┐        ┌───────────────────────────────────┐
│   Container: simulator             │        │   Container: flightstack           │
│   (PyChrono, Python 3.12, rclpy)   │        │   (C++, ROS2 Jazzy)                 │
│                                     │        │                                     │
│  1. Advance physics one step       │        │  advertises /sil/compute_control   │
│  2. Read UAV state                 │──────▶ │  on each call:                      │
│  3. Call /sil/compute_control      │        │   - populateVehicleState(req)       │
│  4. Apply per-motor thrusts [N] ◀──│        │   - runFlightstackController()      │
│  5. Repeat                         │        │   - remap to X8 motor order, return │
└───────────────────────────────────┘        └───────────────────────────────────┘
        ROS2 service call (lock-step, simulator owns the clock, network_mode: host)
```

The two containers communicate exclusively via a **ROS2 service**
(`uav_interfaces/srv/ComputeControl`) over `network_mode: host`, with FastDDS
forced to UDP-only transport (`docker/fastdds_no_shm.xml`) since the
containers don't share `/dev/shm`. `docker-compose.yml` starts `flightstack`
first and gates `simulator`'s startup on a healthcheck that polls
`ros2 service list` for `/sil/compute_control`, so the simulator never tries to
call a service that doesn't exist yet.

The simulator calls the service every control tick, blocks until the
controller responds, applies the returned thrusts to the physics engine, and
advances again - deterministic, lock-step simulation with no timing drift.
The response is **per-motor thrust in Newtons** (clamped to `[0, 9.5]` and
remapped into the simulator's X8 motor index order), not a normalized `[0, 1]`
command.

---

## Prerequisites - Host Machine Setup

### 1. Operating System

These instructions are written for **Windows 11 with WSL2 (Ubuntu 24.04)**.
All commands run inside the WSL2 Ubuntu terminal unless stated otherwise.

> **Never use Git Bash for these commands.** Git Bash lacks `apt`, `colcon`,
> and proper Linux tool support. Use the WSL2 Ubuntu terminal exclusively.

### 2. Install WSL2 with Ubuntu 24.04

In Windows PowerShell (as Administrator):

```powershell
wsl --install -d Ubuntu-24.04
```

After installation reboot your host machine, open **Windows Terminal** and
select the Ubuntu profile, or search "Ubuntu" in the Start menu. You should see:

```
yourname@DESKTOP-XXXXX:~$
```

### 3. Install Docker Desktop for Windows

Download from https://www.docker.com/products/docker-desktop/

During installation:
- Enable **"Use the WSL 2 based engine"** (not Hyper-V)
- After install: Docker Desktop → Settings → Resources → WSL Integration →
  enable your Ubuntu 24.04 distro

Verify Docker and the Compose v2 plugin work inside WSL2:

```bash
docker --version
docker compose version
```

### 4. Install Git

```bash
sudo apt-get update
sudo apt-get install -y git
git --version
```

### 5. (Optional) WSLg GPU passthrough for visualization

The simulator container renders with Irrlicht via hardware-accelerated
OpenGL/D3D12 through WSLg. This works out of the box on a recent WSL2 install
(`wsl --update`) - no extra driver install is needed inside the container,
since `docker-compose.yml` mounts `/usr/lib/wsl`, `/mnt/wslg`, and
`/tmp/.X11-unix` from the host and sets `GALLIUM_DRIVER=d3d12`. If you only
need headless runs (no plotting window), pass `--visualize false` to the
simulator's `command:` in `docker-compose.yml` and you can skip this entirely.

### 6. (Optional) VS Code

Download from https://code.visualstudio.com/ and install the
**"Remote - WSL"** extension, then open the workspace with:

```bash
code ~/projects/acsl_sil_ws
```

> **Important**: Keep all project files inside the WSL2 filesystem (`~/`), NOT
> on the mounted Windows drive (`/mnt/c/...`). The Windows mount is slow and
> causes issues with `colcon` and Docker builds.

---

## Part 1 - Workspace Setup

### Step 1 - Create the workspace directory tree

```bash
mkdir -p ~/projects/acsl_sil_ws/src
mkdir -p ~/projects/acsl_sil_ws/docker
```

Target structure (see the Appendix for the full, current tree):

```
~/projects/acsl_sil_ws/
├── docker/                    ← Dockerfiles, entrypoint, FastDDS profile
├── docker-compose.yml         ← orchestrates the two containers
├── plot_logs.py, plot_logs_sim.py, log_compare.py   ← local diagnostics, see SIL.md
└── src/
    ├── flightstack/           ← ACSL-flightstack repo (cloned directly, no symlink)
    ├── simulator/              ← PyChrono simulator repo (cloned directly, no symlink)
    └── uav_interfaces/         ← custom ROS2 service package (created by hand)
```

> **Symlink warning**: Do NOT use a symlink to either repo inside `src/`.
> Docker's `COPY` command cannot follow symlinks that point outside the build
> context. Clone or move each repo directly into `src/`.

### Step 2 - Clone the ACSL-flightstack and the simulator

```bash
cd ~/projects/acsl_sil_ws/src
git clone --recurse-submodules \
  https://github.com/andrealaffly/ACSL-flightstack.git \
  flightstack

git clone <your-pychrono-simulator-repo-url> simulator
```

Verify both are real directories (not symlinks):

```bash
ls -la ~/projects/acsl_sil_ws/src/
# flightstack/ and simulator/ must show drwxr-xr-x, NOT lrwxrwxrwx
```

---

## Part 2 - Create the `uav_interfaces` ROS2 Package

This package defines the single service type used by both containers. It
lives at `src/uav_interfaces/` and is built first inside each container,
since both `flightstack` and `simulator` depend on the generated message
types.

### Step 3 - Create the package directory

```bash
mkdir -p ~/projects/acsl_sil_ws/src/uav_interfaces/srv
```

### Step 4 - Create `srv/ComputeControl.srv`

**File**: `src/uav_interfaces/srv/ComputeControl.srv`

```
# srv/ComputeControl.srv

# ── REQUEST: simulator → flightstack ─────────────────────────────────
float64 time

float64 px
float64 py
float64 pz

float64 vx
float64 vy
float64 vz

float64 qx
float64 qy
float64 qz
float64 qw

float64 wx
float64 wy
float64 wz

---
# ── RESPONSE: flightstack → simulator ───────────────────────────────
float64[] motor_commands
```

> **Why `float64[]` (dynamic array)?** The vehicle motor count is set in
> `vehicle_info.hpp` and may change (4 for a quadrotor, 8 for X8-copter). A
> dynamic array makes the service vehicle-agnostic.

> **`motor_commands` units**: per-motor thrust in **Newtons**, already
> remapped into the simulator's X8 motor index order - not a normalized
> `[0, 1]` actuator command. See Part 4, Step 13.

### Step 5 - Create `package.xml`

**File**: `src/uav_interfaces/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>uav_interfaces</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="root@todo.todo">root</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Step 6 - Create `CMakeLists.txt`

**File**: `src/uav_interfaces/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(uav_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the ComputeControl service type
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeControl.srv"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export runtime dependency so downstream packages can find the generated types
ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

---

## Part 3 - Modify the ACSL-flightstack for SIL

The flightstack is designed for hardware (PX4 + VICON). For SIL we:

- Remove all PX4 (`px4_msgs`) and mocap (UDP) dependencies at compile time
- Replace them with a ROS2 service server (`SilInterface`)
- Use a CMake `option(SIL_MODE)` flag so the **same repo** builds for both
  hardware and SIL - zero permanent code deletion
- Add a `SELECTED_CONTROLLER` CMake cache variable so the active controller
  can be picked at build time (`-DSELECTED_CONTROLLER=...`) instead of only
  via the hardcoded `#define` in `config.hpp`

This Part covers the changes that make `SIL_MODE` exist as a build target at
all. A handful of smaller controller-pipeline tweaks (an explicit-yaw variant
of `compute_U1_RollDes_PitchDes`, an outer-loop derivative filter bypass under
`SIL_MODE`, `overwrite_existing` on log file snapshots, gain retuning, and the
55° safety envelope) are SIL-*motivated* but not required for the bridge to
function - they're listed in full in
[`FLIGHTSTACK_CHANGES.md`](src/flightstack/FLIGHTSTACK_CHANGES.md) rather than
repeated here.

### Step 7 - Modify `CMakeLists.txt`

**File**: `src/flightstack/CMakeLists.txt`

Apply the following changes in order. Do not delete anything - only add
guards and cache variables.

**Change 1** - declare `SIL_MODE` and guard the Boost path.
*Where*: right after the compiler-flags check block, before `# find dependencies`.

```cmake
#SIL modifications -----------------------------------------------------------------------------
# Declare SIL_MODE first so all subsequent if() checks see it
option(SIL_MODE "Build for Software-In-The-Loop simulation" OFF)
if(SIL_MODE)
  message(STATUS "--- Building in SIL MODE (Using uav_interfaces) ---")
else()
  message(STATUS "--- Building in HARDWARE MODE (Using px4_msgs) ---")
endif()
# --------------------------------------------------------------------------------------------

# Controller selection -----------------------------------------------------------------------
# Override with -DSELECTED_CONTROLLER=<name> (docker-compose.yml passes this through via
# flightstack.Dockerfile's CMD, using the CONTROLLER_TYPE env var). Falls back to PID.
set(SELECTED_CONTROLLER "PID" CACHE STRING "Controller type: PID, MRAC, TwoLayerMRAC, FunnelTwoLayerMRAC, HybridTwoLayerMRAC")
if(SELECTED_CONTROLLER STREQUAL "PID")
  set(SELECTED_CONTROLLER_VALUE 1)
elseif(SELECTED_CONTROLLER STREQUAL "MRAC")
  set(SELECTED_CONTROLLER_VALUE 2)
elseif(SELECTED_CONTROLLER STREQUAL "TwoLayerMRAC")
  set(SELECTED_CONTROLLER_VALUE 3)
elseif(SELECTED_CONTROLLER STREQUAL "FunnelTwoLayerMRAC")
  set(SELECTED_CONTROLLER_VALUE 4)
elseif(SELECTED_CONTROLLER STREQUAL "HybridTwoLayerMRAC")
  set(SELECTED_CONTROLLER_VALUE 5)
else()
  message(FATAL_ERROR "Invalid SELECTED_CONTROLLER '${SELECTED_CONTROLLER}'. Valid options: PID, MRAC, TwoLayerMRAC, FunnelTwoLayerMRAC, HybridTwoLayerMRAC")
endif()
add_compile_definitions(SELECTED_CONTROLLER=${SELECTED_CONTROLLER_VALUE})
message(STATUS "--- Controller: ${SELECTED_CONTROLLER} ---")
# --------------------------------------------------------------------------------------------

# SIL modifications -----------------------------------------------------------------------------
# In hardware mode use the bundled Boost source tree (headers + staged libs)
# In SIL mode use system Boost installed via apt in Docker
if(NOT SIL_MODE)
  set(BOOST_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/include/boost")
endif()
# --------------------------------------------------------------------------------------------
```

> **Why guard Boost?** The bundled `include/boost/` contains Boost SOURCE
> only (no compiled `.a`/`.so` files). `Boost::log` and `Boost::log_setup` are
> compiled components and cannot be found from source headers alone. In SIL
> mode, Docker installs `libboost-all-dev` via apt which provides the
> compiled libraries.

**Change 2** - guard `find_package(px4_msgs)` and add `uav_interfaces`.
*Where*: inside `# find dependencies`, replacing `find_package(px4_msgs REQUIRED)`.

```cmake
if(NOT SIL_MODE)
  find_package(px4_msgs REQUIRED)
else()
  find_package(uav_interfaces REQUIRED)
endif()
```

**Change 3** - add the SIL include directory.
*Where*: immediately after the last `include_directories(...)` entry.

```cmake
if(SIL_MODE)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/sil)
endif()
```

**Change 4** - wrap hardware-only sources and add the SIL source, inside
`set(SOURCES ...)`. Use CMake generator expressions so the *same*
`CMakeLists.txt` produces either binary.

```cmake
  # SIL modifications -----------------------------------------------------------------------------
  $<$<NOT:$<BOOL:${SIL_MODE}>>:src/logging/logging_mocap.cpp>
  #---------------------------------------------------------------------------------------------
  src/main.cpp

  # SIL modifications -----------------------------------------------------------------------------
  $<$<NOT:$<BOOL:${SIL_MODE}>>:src/mocap/io_context.cpp>
  $<$<NOT:$<BOOL:${SIL_MODE}>>:src/mocap/mocap.cpp>
  $<$<NOT:$<BOOL:${SIL_MODE}>>:src/mocap/udp_driver.cpp>
  $<$<NOT:$<BOOL:${SIL_MODE}>>:src/mocap/udp_socket.cpp>

  # SIL-only source - included only if SIL_MODE is ON
  $<$<BOOL:${SIL_MODE}>:src/sil/sil_interface.cpp>
  #----------------------------------------------------------------------------------------------

  src/multi_threaded_node.cpp

  # SIL modifications -----------------------------------------------------------------------------
  $<$<NOT:$<BOOL:${SIL_MODE}>>:src/pixhawk_interface/pixhawk_actuator_motors.cpp>
  $<$<NOT:$<BOOL:${SIL_MODE}>>:src/pixhawk_interface/pixhawk_vehicle_odometry.cpp>
  src/pixhawk_interface/vehicle_state.cpp
  #----------------------------------------------------------------------------------------------
```

*(`src/pixhawk_interface/vehicle_state.cpp` stays un-wrapped - it's needed by
both build targets.)*

**Change 5** - guard `ament_target_dependencies` and define the `SIL_MODE`
macro for the compiler.
*Where*: right after `target_link_libraries(flightstack ${Boost_LIBRARIES})`.

```cmake
if(NOT SIL_MODE)
  ament_target_dependencies(flightstack rclcpp px4_msgs lifecycle_msgs rclcpp_lifecycle)
else()
  ament_target_dependencies(flightstack rclcpp uav_interfaces lifecycle_msgs rclcpp_lifecycle)
  target_compile_definitions(flightstack PRIVATE SIL_MODE)
endif()
```

### Step 8 - Modify `include/flightstack/multi_threaded_node.hpp`

This header is included by almost everything; guards here affect the whole
build.

**Change 1** - guard the PX4 message headers, right after `#include <rclcpp/rclcpp.hpp>`:

```cpp
#ifndef SIL_MODE
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#endif
```

**Change 2** - conditionally include `mocap.hpp` or `sil_interface.hpp`, right
after `#include "control.hpp"`:

```cpp
#ifndef SIL_MODE
#include "mocap.hpp"
#endif

#ifdef SIL_MODE
#include "sil_interface.hpp"
#endif

#include "print_helpers.hpp"
```

**Change 3** - guard the hardware `arm()`/`disarm()` public methods:

```cpp
public:
  MultiThreadedNode(rclcpp::NodeOptions options);

  #ifndef SIL_MODE
  void arm();
  void disarm();
  #endif

  void updateCurrentTimeInSeconds();
```

**Change 4** - hoist the members both build modes need (`vehicle_state_`,
`user_defined_trajectory_`, `control_`, `timestamp_initial_`, `time_current_`,
`global_params_`) out of the hardware-only block, then wrap callback groups 1
and 2 (Pixhawk odometry subscriber; actuator motors / offboard publishers) in
a negative guard:

```cpp
private:
  ConfigurationParameters config_;
  std::shared_ptr<VehicleState>  vehicle_state_;
  std::shared_ptr<PiecewisePolynomialTrajectory> user_defined_trajectory_;
  std::shared_ptr<ControlType> control_;

  std::atomic<uint64_t> timestamp_initial_;
  std::atomic<double> time_current_;

  GlobalParameters global_params_;

  #ifndef SIL_MODE
  /***** CALLBACK GROUP 1 (Hardware Specific) *****/
  rclcpp::CallbackGroup::SharedPtr callback_group_sub_pixhawk_odometry_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_pixhawk_odometry_;
  void subscriber_pixhawk_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
  void setupPixhawkOdometrySubscriber();

  /***** CALLBACK GROUP 2 (Hardware Specific) *****/
  rclcpp::CallbackGroup::SharedPtr callback_group_pub_actuator_motors_;
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr publisher_actuator_motors_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_vehicle_command_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr publisher_offboard_control_mode_;
  rclcpp::TimerBase::SharedPtr timer_actuator_motors_;

  void publisher_actuator_motors_callback();
  void publish_offboard_control_mode();
  void publish_actuator_motors();
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
  inline void goIntoOffboardMode();

  int offboard_flag_;

  void setupPixhawkActuatorMotorsPublisher();
  void controller_and_publisher_actuator_motors_callback();
```

**Change 5** - inject the `SilInterface` handle in the `#else` branch and
grant it access via `friend`:

```cpp
  #else
  // SIL specific interface pointer for handling simulated telemetry and control
  std::shared_ptr<SilInterface> sil_interface_;
  #endif

  void controller_callback();

  #ifdef SIL_MODE
  friend class SilInterface;
  #endif
};
```

### Step 9 - Modify `multi_threaded_node.cpp`

**Change 1** - guard `offboard_flag_` in the constructor's initializer list
(it doesn't exist in `SIL_MODE`):

```cpp
MultiThreadedNode::MultiThreadedNode(rclcpp::NodeOptions options)
: Node("MultiThreadedNode", options),
  config_(ConfigurationParameters::readConfigurationParametersFile("config.json")),
  vehicle_state_(std::make_shared<VehicleState>(*this)),
  user_defined_trajectory_(std::make_shared<PiecewisePolynomialTrajectory>(*this)),
  control_(std::make_shared<ControlType>(*this)),
  timestamp_initial_(this->get_clock()->now().nanoseconds() / 1000),
  global_params_(GlobalParameters(*this))
  #ifndef SIL_MODE
    , offboard_flag_(0)
  #endif
{
```

**Change 2** - conditionally set up Pixhawk pipelines or instantiate the
`SilInterface`, inside the constructor body:

```cpp
  std::cout << "SELECTED CONTROLLER: " << ControlType::getControllerName() << std::endl;

  #ifndef SIL_MODE
    setupPixhawkOdometrySubscriber();
    setupPixhawkActuatorMotorsPublisher();
  #else
    sil_interface_ = std::make_shared<SilInterface>(*this);
  #endif
}
```

**Change 3** - make `updateCurrentTimeInSeconds()` a no-op in `SIL_MODE` so
the simulator's injected `time_current_` (see Step 13) isn't immediately
overwritten by the wall clock:

```cpp
void MultiThreadedNode::updateCurrentTimeInSeconds()
{
#ifndef SIL_MODE
  time_current_ = ((this->get_clock()->now().nanoseconds() / 1000) - timestamp_initial_) / 1e6;
#endif
  // SIL_MODE leaves time_current_ untouched so your simulator can inject it deterministically
}
```

**Change 4** - wrap the Pixhawk subscriber/publisher setup functions and
`controller_and_publisher_actuator_motors_callback()` at the bottom of the
file in `#ifndef SIL_MODE ... #endif` (unchanged bodies - see the upstream
file for the full implementation).

### Step 10 - Modify `src/main.cpp`

Add a `SIL_MODE` branch at the top of `main()`, right after `rclcpp::init()`,
that spins a plain `MultiThreadedExecutor` waiting on the service and skips
the hardware EKF2/MOCAP startup paths entirely:

```cpp
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  #ifdef SIL_MODE
  {
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions options;
    auto node = std::make_shared<MultiThreadedNode>(options);
    executor.add_node(node);
    RCLCPP_INFO(node->get_logger(), "SIL MODE - waiting for /sil/compute_control calls");
    executor.spin();
    rclcpp::shutdown();
    return 0;
  }
  #else

  if constexpr (CURRENT_EKF2_FUSION_MODE == EKF2FusionMode::GPS)
  {
    // ... unchanged hardware GPS path
  } else if constexpr (CURRENT_EKF2_FUSION_MODE == EKF2FusionMode::MOCAP)
  {
    // ... unchanged hardware MOCAP path
  }

  #endif

  return 0;
}
```

### Step 11 - Modify `control_callback.cpp`

`postIntegrationAlgorithm()` (a Hybrid-Two-Layer-MRAC-only step) is now gated
by the *compile-time* controller selection instead of a `constexpr` trait
check, since `SELECTED_CONTROLLER` is available as a preprocessor macro
(Step 7, Change 1):

```cpp
  control_->stepper.do_step(std::cref(controllerBind),
                            control_->getStateController().full_state,
                            this->time_current_,
                            control_->getTimeStepRK4());

  #if (SELECTED_CONTROLLER == __HYBRID_TWO_LAYER_MRAC__)
    control_->postIntegrationAlgorithm(this->time_current_.load());
  #endif
```

> **Why**: only the Hybrid Two-Layer MRAC variant needs a post-integration
> update step; wrapping it in a preprocessor directive (rather than relying on
> a compile-time trait that depended on `SELECTED_CONTROLLER` being a `#define`
> at all times) avoids invalid calls when a simpler controller is selected.

### Step 12 - Add `SIL_MODE` gating in `config.cpp`

`GlobalParameters`'s `ARM_START_TIME_SECONDS` / `TAKEOFF_START_TIME_SECONDS`
are wrapped in `#ifdef SIL_MODE` / `#else` so a SIL-specific gate timing can
be introduced later without touching the hardware path. Both branches
currently evaluate to the same values:

```cpp
GlobalParameters::GlobalParameters(MultiThreadedNode& node) :
  ARM_START_TIME_SECONDS(
#ifdef SIL_MODE
    5.0
#else
    5.0
#endif
  ),
  TAKEOFF_START_TIME_SECONDS(
#ifdef SIL_MODE
    10.0
#else
    10.0
#endif
  ),
  // ... LANDING_START_TIME_SECONDS, etc. unchanged
```

---

## Part 4 - Create the SIL Interface Files

These two files are the entire SIL communication layer - the thing that
replaces `mocap`/Pixhawk on the wire.

### Step 13 - Create `include/flightstack/sil/sil_interface.hpp`

```bash
mkdir -p ~/projects/acsl_sil_ws/src/flightstack/include/flightstack/sil
```

**File**: `src/flightstack/include/flightstack/sil/sil_interface.hpp`

```cpp
#pragma once
#ifdef SIL_MODE

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "uav_interfaces/srv/compute_control.hpp"

class MultiThreadedNode; // forward declaration

class SilInterface
{
public:
  explicit SilInterface(MultiThreadedNode & node);

private:
  void computeControlCallback(
    const uav_interfaces::srv::ComputeControl::Request::SharedPtr  req,
    uav_interfaces::srv::ComputeControl::Response::SharedPtr       res);

  void populateVehicleState(
    const uav_interfaces::srv::ComputeControl::Request::SharedPtr & req);

  // Runs the real flightstack control pipeline (PID/MRAC/etc., whichever
  // ControlType is selected at compile time) for one step and returns the
  // resulting per-motor thrust in Newtons (8 motors, X8 layout).
  Eigen::Matrix<double, 8, 1> runFlightstackController();

  MultiThreadedNode & node_;
  rclcpp::Service<uav_interfaces::srv::ComputeControl>::SharedPtr service_;
};

#endif // SIL_MODE
```

### Step 14 - Create `src/sil/sil_interface.cpp`

```bash
mkdir -p ~/projects/acsl_sil_ws/src/flightstack/src/sil
```

**File**: `src/flightstack/src/sil/sil_interface.cpp`

```cpp
#ifdef SIL_MODE

#include <array>
#include <algorithm>
#include "sil_interface.hpp"
#include "multi_threaded_node.hpp"  // brings in VehicleState, ControlType, etc.

SilInterface::SilInterface(MultiThreadedNode & node)
: node_(node)
{
  service_ = node_.create_service<uav_interfaces::srv::ComputeControl>(
    "/sil/compute_control",
    std::bind(&SilInterface::computeControlCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(node_.get_logger(),
    "SilInterface ready - advertising /sil/compute_control");
}

void SilInterface::populateVehicleState(
  const uav_interfaces::srv::ComputeControl::Request::SharedPtr & req)
{
  auto & vs = *node_.vehicle_state_;

  // Inject simulator time directly - bypasses wall-clock updateCurrentTimeInSeconds()
  node_.time_current_ = req->time;

  // Build a fake microsecond timestamp consistent with the initial timestamp
  uint64_t sim_us = node_.timestamp_initial_.load() +
                    static_cast<uint64_t>(req->time * 1e6);
  vs.setTimestamp(sim_us);

  // On the first call, capture the drone's actual initial pose as the offset -
  // exactly as the hardware path does in pixhawk_vehicle_odometry.cpp.
  // This ensures the controller sees position = (0,0,0) and yaw = 0 at t=0,
  // i.e. zero initial error against the trajectory reference.
  if (vs.getIsOffsetOdometryMsg()) {
    vs.setIsOffsetOdometryMsg(false);
    vs.setPositionOffset(Eigen::Vector3d(req->px, req->py, req->pz));
    Eigen::Quaterniond q_init(req->qw, req->qx, req->qy, req->qz);
    vs.setQuaternionOffset(q_init);
    vs.setEulerAnglesRPYOffset(q_init);
    vs.setYawQuaternionInverseOffset();
  }

  // Position: subtract initial offset so controller sees relative position (0,0,0 at start)
  vs.setPosition(Eigen::Vector3d(req->px - vs.getPositionOffset()(0),
                                  req->py - vs.getPositionOffset()(1),
                                  req->pz - vs.getPositionOffset()(2)));
  vs.setVelocity(Eigen::Vector3d(req->vx, req->vy, req->vz));

  // Attitude - remove initial yaw bias, matching hardware behaviour.
  // Request uses scalar-last (qx,qy,qz,qw); Eigen uses scalar-first (w,x,y,z).
  Eigen::Quaterniond q(req->qw, req->qx, req->qy, req->qz);
  Eigen::Quaterniond yaw_unbiased_q = vs.getYawQuaternionInverseOffset() * q;
  vs.setQuaternion(yaw_unbiased_q);
  vs.setEulerAnglesRPY(yaw_unbiased_q);

  vs.setAngularVelocity(Eigen::Vector3d(req->wx, req->wy, req->wz));
  vs.setTimeOdometryInSeconds();
}

// Runs one step of the real flightstack control pipeline via
// MultiThreadedNode::controller_callback() (SilInterface is a friend of
// MultiThreadedNode), then reads back the resulting per-motor thrust vector.
Eigen::Matrix<double, 8, 1> SilInterface::runFlightstackController()
{
  node_.controller_callback();

  // thrust_vector (physical Newtons) is used instead of its normalized
  // counterpart, because the normalization polynomial is not linear and
  // only exists on the hardware actuator path, not in the simulator.
  Eigen::Matrix<double, 8, 1> thrusts =
    node_.control_->getControlInternalMembers().thrust_vector;

  for (int i = 0; i < 8; ++i) {
    thrusts(i) = std::clamp(thrusts(i), 0.0, 9.5);
  }

  return thrusts;
}

void SilInterface::computeControlCallback(
  const uav_interfaces::srv::ComputeControl::Request::SharedPtr  req,
  uav_interfaces::srv::ComputeControl::Response::SharedPtr       res)
{
  populateVehicleState(req);
  Eigen::Matrix<double, 8, 1> thrusts = runFlightstackController();

  // mixer_matrix (vehicle_info.hpp) row k assumes a specific
  // (front/back, left/right, CW/CCW) motor at position k. The simulator's
  // X8_config.yaml motor_index_map + force_positions + spin_directions place
  // controller.motor_thrusts[m] at a DIFFERENT (position, spin) for each m.
  // Remap row k's output to the index m whose (position, spin) matches what
  // row k was designed for, so the mixer's roll/pitch/yaw decoupling is
  // physically correct in the X8 layout.
  static constexpr std::array<int, 8> kMixerRowToX8Motor = {6, 5, 4, 7, 1, 2, 3, 0};

  res->motor_commands.resize(8);
  for (int i = 0; i < 8; ++i) {
    res->motor_commands[kMixerRowToX8Motor[i]] = thrusts(i);
  }
}

#endif // SIL_MODE
```

> **Note**: `runFlightstackController()` is split out as its own method (it
> isn't just inlined into `computeControlCallback`) specifically so unit tests
> or future bridges can call it without going through the ROS2 service layer.

---

## Part 5 - Write the Dockerfiles

### Step 15 - Create `docker/fastdds_no_shm.xml`

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<!--
  Force FastDDS to use UDP-only transport.
  Without this, FastDDS defaults to shared-memory transport for same-host
  communication, which fails across Docker containers because each container
  has its own /dev/shm.
-->
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udpv4_transport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="default_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>udpv4_transport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
```

### Step 16 - Create `docker/flightstack.Dockerfile`

```dockerfile
FROM ros:jazzy

# Build tools + system Boost (replaces bundled source-only Boost in SIL mode)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy the full workspace source into the image
WORKDIR /ros2_ws
COPY src/ src/

# Build uav_interfaces first (flightstack depends on it)
RUN bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select uav_interfaces"

# Build flightstack with a RAM limit (a few files at a time via MAKEFLAGS)
RUN bash -c "export MAKEFLAGS=-j4 && \
    source /opt/ros/jazzy/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    colcon build \
      --packages-select flightstack \
      --parallel-workers 1 \
      --event-handlers console_direct+ \
      --cmake-args -DSIL_MODE=ON -DCMAKE_BUILD_TYPE=Release --no-warn-unused-cli"

# Source both overlays on container start
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'source /ros2_ws/install/setup.bash' >> /root/.bashrc

CMD ["bash", "-c", \
     "source /opt/ros/jazzy/setup.bash && \
      source /ros2_ws/install/setup.bash && \
      export MAKEFLAGS=-j4 && \
      colcon build --packages-select flightstack \
        --cmake-args -DSIL_MODE=ON -DCMAKE_BUILD_TYPE=Release -DSELECTED_CONTROLLER=${CONTROLLER_TYPE:-PID} && \
      source /ros2_ws/install/setup.bash && \
      ros2 run flightstack flightstack"]
```

> **Why does `CMD` rebuild on every container start?** `docker-compose.yml`
> bind-mounts `src/flightstack` into the container and reads `CONTROLLER_TYPE`
> from a host environment variable. Re-running `colcon build` with
> `-DSELECTED_CONTROLLER=${CONTROLLER_TYPE}` at start time means you can switch
> controllers or edit C++ source on the host and just restart the container -
> no `docker compose build` (image rebuild) required. See Part 8.
>
> `MAKEFLAGS=-j4` caps make's file-level parallelism (rather than leaving it
> unbounded) to avoid exhausting WSL2's RAM during compilation; raise or lower
> it based on available cores/memory. `/ros2_ws/build` and `/ros2_ws/install`
> are also bind-mounted as named volumes in `docker-compose.yml` (see Part 8),
> so this `CMD` rebuild is only a full recompile on the very first start -
> later starts with unchanged C++ source are near-instant.

### Step 17 - Create `docker/simulator.Dockerfile`

```dockerfile
FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    wget \
    build-essential \
    libgl1 \
    libglx-mesa0 \
    libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-empy \
    && rm -rf /var/lib/apt/lists/*

# Install Miniconda
RUN wget -q https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh \
    -O /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p /opt/conda && \
    rm /tmp/miniconda.sh

ENV PATH=/opt/conda/bin:$PATH

RUN conda tos accept --override-channels \
    --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels \
    --channel https://repo.anaconda.com/pkgs/r

RUN conda config --add channels conda-forge && \
    conda config --set channel_priority strict

# Create the chrono_v9 environment and install dependencies
RUN conda create -y -n chrono_v9 python=3.12 pip
RUN conda run -n chrono_v9 conda install -y \
    numpy=2.3.5 \
    matplotlib \
    irrlicht=1.8.5 \
    pytz \
    scipy \
    pyyaml \
    deap \
    pymoo \
    ruamel.yaml \
    -c conda-forge

# Install the official PyChrono 9.0.1 package
RUN wget -O /tmp/pychrono.conda \
    https://anaconda.org/projectchrono/pychrono/9.0.1/download/linux-64/pychrono-9.0.1-py312hf1de3a3_6463.conda && \
    conda run -n chrono_v9 conda install -y /tmp/pychrono.conda

WORKDIR /ros2_ws

COPY docker/simulator_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
```

> **Why conda and not apt/pip for PyChrono?** PyChrono ships as a conda
> package with native bindings; the official build is only published on the
> `projectchrono` conda channel, pinned against specific `numpy`/`irrlicht`
> versions also installed from conda-forge.

### Step 18 - Create `docker/simulator_entrypoint.sh`

```bash
#!/bin/bash
set -e

# 1. Source ROS Jazzy
source /opt/ros/jazzy/setup.bash

# 2. Build only what the simulator needs (uav_interfaces) with system Python
cd /ros2_ws
colcon build \
    --packages-select uav_interfaces \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash

# 3. Activate the conda environment
source /opt/conda/etc/profile.d/conda.sh
conda activate chrono_v9

# 4. Merge PYTHONPATHs: PyChrono + conda env packages + ROS2 + workspace
export PYTHONPATH="\
/opt/conda/envs/chrono_v9/share/chrono/python:\
/opt/conda/envs/chrono_v9/lib/python3.12/site-packages:\
/ros2_ws/install/uav_interfaces/local/lib/python3.12/dist-packages:\
/opt/ros/jazzy/lib/python3.12/site-packages:\
/opt/ros/jazzy/local/lib/python3.12/dist-packages:\
${PYTHONPATH}"

# 5. Prioritize conda's Python on PATH
export PATH="/opt/conda/envs/chrono_v9/bin:${PATH}"

exec "$@"
```

`ENTRYPOINT` runs this script, then `exec "$@"` hands off to whatever
`docker-compose.yml`'s `command:` specifies (the simulator's `main.py`).

> **Why mix conda and the system ROS2 Python?** ROS2 Jazzy's `rclpy` is
> installed against the system Python (`apt`), while PyChrono is only
> available via conda. Rather than rebuilding ROS2 inside conda, this script
> runs the *conda* Python interpreter (`PATH` prioritizes it) but manually
> appends the *system* ROS2 site-packages to `PYTHONPATH`, so a single
> process can `import pychrono` and `import rclpy` simultaneously.

---

## Part 6 - Orchestrate with `docker-compose.yml`

### Step 19 - Create `docker-compose.yml`

**File**: `docker-compose.yml` (workspace root)

```yaml
# Single source of truth for the controller type, shared by both services below
# Controller type: PID, MRAC, TwoLayerMRAC, FunnelTwoLayerMRAC, HybridTwoLayerMRAC (falls back to PID if unset)
x-controller-type: &controller_type "FunnelTwoLayerMRAC"

services:

  flightstack:
    build:
      context: .
      dockerfile: docker/flightstack.Dockerfile
    environment:
      ROS_DOMAIN_ID: "42"
      FASTRTPS_DEFAULT_PROFILES_FILE: /fastdds_no_shm.xml
      CONTROLLER_TYPE: *controller_type
    network_mode: host
    volumes:
      - ./src/flightstack/log:/ros2_ws/src/flightstack/log
      - ./src/flightstack:/ros2_ws/src/flightstack
      - ./src/uav_interfaces:/ros2_ws/src/uav_interfaces
      - ./docker/fastdds_no_shm.xml:/fastdds_no_shm.xml:ro
      - flightstack_build:/ros2_ws/build
      - flightstack_install:/ros2_ws/install
    healthcheck:
      test: ["CMD", "bash", "-c",
             "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 service list 2>/dev/null | grep -q '/sil/compute_control'"]
      interval: 10s
      timeout: 6s
      # Generous budget: changing CONTROLLER_TYPE (or any C++ source) forces a
      # recompile of flightstack on container start, which can take several
      # minutes on the first run before build/install state is cached in the
      # named volumes below.
      retries: 180
      start_period: 30s

  simulator:
    build:
      context: .
      dockerfile: docker/simulator.Dockerfile
    network_mode: host
    environment:
      ROS_DOMAIN_ID: "42"
      UAV_SIL_MODE: "true"
      DISPLAY: ":0"
      GALLIUM_DRIVER: "d3d12"
      LD_LIBRARY_PATH: "/usr/lib/wsl/lib"
      FASTRTPS_DEFAULT_PROFILES_FILE: /fastdds_no_shm.xml
      CONTROLLER_TYPE: *controller_type
    devices:
      - /dev/dxg
    volumes:
      - ./src/simulator:/ros2_ws/src/simulator
      - ./src/uav_interfaces:/ros2_ws/src/uav_interfaces
      - ./docker/fastdds_no_shm.xml:/fastdds_no_shm.xml:ro
      # Read-only: lets the simulator inherit the trajectory selected for the flightstack
      - ./src/flightstack/params/config/config.json:/ros2_ws/src/flightstack/params/config/config.json:ro
      # WSLg: GPU-backed X11 socket + D3D12 Mesa driver libs, for hardware-accelerated rendering
      - /tmp/.X11-unix:/tmp/.X11-unix
      - /mnt/wslg:/mnt/wslg
      - /usr/lib/wsl:/usr/lib/wsl
    command: >
      python /ros2_ws/src/simulator/main.py
        --uav X8
        --simulation_duration 35.0
        --visualize true
        --add_payload true
        --payload_type two_steel_balls
        --drop_two_steel_balls false
        --two_steel_balls_drop_time 10.0
        --sequential_drop false
        --sequential_drop_start 5.0
        --sequential_drop_interval 1.0
        --apply_motor_failure false
        --motor_failure_time 5.0
    depends_on:
      flightstack:
        condition: service_healthy

volumes:
  flightstack_build:
  flightstack_install:
```

> **Why `network_mode: host` on both services?** ROS2's DDS discovery uses
> UDP multicast, which does not traverse Docker's default bridge network
> without extra configuration. `network_mode: host` gives each container
> direct access to the host network stack, so DDS discovery (and the
> `/sil/compute_control` service call) works with zero extra config - that's
> also why both services must share the same `ROS_DOMAIN_ID`.

> **Why `depends_on: condition: service_healthy`** instead of just
> `depends_on: [flightstack]`? Plain `depends_on` only waits for the
> container to *start*, not for the ROS2 node inside it to finish building
> and advertise its service. The simulator would otherwise race the
> flightstack's (multi-minute, on first boot) compile step and fail to
> connect. The healthcheck polls `ros2 service list` until
> `/sil/compute_control` actually exists.

### Step 20 - Build and run the stack

```bash
cd ~/projects/acsl_sil_ws
docker compose build       # first time only, or after Dockerfile/system-dep changes
docker compose up
```

Expected log sequence:

```
flightstack-1  | --- Building in SIL MODE (Using uav_interfaces) ---
flightstack-1  | --- Controller: FunnelTwoLayerMRAC ---
flightstack-1  | SELECTED CONTROLLER: FunnelTwoLayerMRAC
flightstack-1  | [INFO] [...] [MultiThreadedNode]: SilInterface ready - advertising /sil/compute_control
flightstack-1  | [INFO] [...] [MultiThreadedNode]: SIL MODE - waiting for /sil/compute_control calls
simulator-1    | FlightStack Bridge: waiting for /sil/compute_control ...
simulator-1    | FlightStack Bridge: connected.
```

`simulator-1` only starts printing once `flightstack-1`'s healthcheck passes
- if you see `simulator-1` waiting more than ~2 minutes on first boot, that's
expected (the C++ recompile inside `flightstack`'s `CMD` is the bottleneck,
not the simulator).

To stop:

```bash
docker compose down
```

### Optional - GA gain tuning against SIL

A third compose service, `ga_tuner` (`profiles: ["tools"]`, so it's excluded
from the `docker compose up` above), runs a genetic-algorithm optimizer
against the real flightstack container instead of a fixed gain file -
restarting `flightstack` with freshly-written candidate gains before every
simulated individual. It's not part of this base setup walkthrough; see
[`src/simulator/acsl_pychrono/ga_tuner/GA_TUNER_SIL.md`](src/simulator/acsl_pychrono/ga_tuner/GA_TUNER_SIL.md)
for the architecture and
[`SIL_USAGE_GUIDE.md`](SIL_USAGE_GUIDE.md) §4 for a shorter operator-facing
summary. Invoke with `docker compose run --rm ga_tuner` once the base stack
above builds and runs successfully - it currently only supports the `PID`
controller.

---

## Part 7 - Timing and Configuration Reference

### Flightstack timing thresholds (`src/config/config.cpp`)

| Parameter | Value | Description |
|---|---|---|
| `ARM_START_TIME_SECONDS` | 5.0 s | Motors arm at this sim time |
| `TAKEOFF_START_TIME_SECONDS` | 10.0 s | Controller activates at this sim time |
| `LANDING_START_TIME_SECONDS` | computed | Takeoff + trajectory duration + hover time |
| `DISARM_START_TIME_SECONDS` | computed | Landing end + 1.0 s |

**Implication for SIL**: the simulator must send `time > 10.0` before the
controller computes non-zero motor commands. Before that, the thrust vector
is zero-initialized and the response will be all zeros.

### Runtime configuration file (`params/config/config.json`)

```json
{
  "user_defined_trajectory_file": "rollercoaster_trajectory1p2.json",
  "hover_after_trajectory_time_seconds": 5.0,
  "motor_failure_start_time_seconds": 10.0,
  "motor_failure_duration_time_seconds": 10.0,
  "motor_failure_efficiencies": [[1.0],[1.0],[1.0],[1.0],[1.0],[1.0],[1.0],[1.0]]
}
```

Since `docker-compose.yml` bind-mounts `src/flightstack` (not just `params/`),
editing this file on the host takes effect on the next `docker compose up`
(the container's `CMD` recompiles `flightstack` - and re-reads the JSON -
every start) without any image rebuild.

### Outer-loop safety envelope (`params/control/outer_loop_safety_mechanism.json`)

```json
{
  "maximum_total_thrust": 65.0,
  "maximum_roll_angle": 55.0,
  "maximum_pitch_angle": 55.0,
  "alpha_plane": 0.1
}
```

Roll/pitch limits were raised from the upstream default of 45° to 55° to
accommodate the more aggressive rollercoaster trajectory used for SIL testing.

### Controller selection

Two independent knobs select the controller, and they must agree:

1. **Compile-time fallback** - `SELECTED_CONTROLLER` `#define` in
   `include/flightstack/config/config.hpp` (defaults to `__PID__`), only used
   if the CMake variable below isn't set.
2. **Build-time override** - the `SELECTED_CONTROLLER` CMake cache variable
   (Part 3, Step 7), driven by `docker-compose.yml`'s `CONTROLLER_TYPE`
   environment variable via `flightstack.Dockerfile`'s `CMD`. **This is the
   one you actually want to edit** - change the `x-controller-type` anchor at
   the top of `docker-compose.yml`, then `docker compose up` (no image
   rebuild needed; see Part 8).

The running container prints `SELECTED CONTROLLER: <name>` on startup so you
can always confirm which one is active. The SIL interface works identically
regardless of which controller is selected.

---

## Part 8 - Rebuild After Changes

Because `docker-compose.yml` bind-mounts the entire `src/flightstack` and
`src/simulator` trees, and `flightstack.Dockerfile`'s `CMD` recompiles
`flightstack` from source on every container start, most changes need only a
container restart - **not** an image rebuild.

`docker-compose.yml` also mounts `/ros2_ws/build` and `/ros2_ws/install` as
named volumes (`flightstack_build`, `flightstack_install`) for the
`flightstack` service. This means colcon's build state survives container
recreation, so the `CMD` rebuild on each start is incremental:

- **Params/trajectories only** (e.g. `config.json`) - no C++ source changed,
  so the rebuild is a near no-op (a few seconds).
- **C++ source edits** - only the changed files (and anything depending on
  them) get recompiled, not the whole `flightstack` package.
- **Switching `CONTROLLER_TYPE`** - changes the `-DSELECTED_CONTROLLER`
  compile define passed to every translation unit it affects, so this still
  forces a fuller recompile of those files regardless of the cached volumes.

The very first `docker compose up` after creating these volumes (or after
`docker compose down -v` / `docker volume rm`) pays the full build cost,
since there's no prior build state to reuse.

### Changing the controller, params, trajectories, or any C++ source

```bash
cd ~/projects/acsl_sil_ws
docker compose up        # recreates containers; flightstack's CMD recompiles
```

`docker compose up` is enough even for `.cpp`/`.hpp` edits, since the bind
mount means the container sees your edited files and the `CMD` rebuild step
picks them up.

### Changing a Dockerfile or a system dependency (apt/conda package)

A full image rebuild is required:

```bash
docker compose build
docker compose up
```

Docker layer caching means only the layers after the changed instruction are
invalidated.

### Forcing a clean restart of one service

```bash
docker compose restart flightstack
# or, to recreate from scratch:
docker compose up --force-recreate flightstack
```

---

## Part 9 - Troubleshooting

### Service never appears in `ros2 service list` / healthcheck never passes

The node crashed before advertising the service, or the recompile is still
running. Check logs:

```bash
docker compose logs -f flightstack
```

Common causes:
- Missing `params/config/config.json` or the referenced trajectory file
- Missing `params/control/<controller>/gains_<controller>.json`
- Invalid `SELECTED_CONTROLLER` value in `docker-compose.yml`'s
  `x-controller-type` anchor (must be one of `PID`, `MRAC`, `TwoLayerMRAC`,
  `FunnelTwoLayerMRAC`, `HybridTwoLayerMRAC` - CMake will `FATAL_ERROR` and
  the build fails)

### `Boost not found` during the flightstack build

The `if(NOT SIL_MODE)` guard around `set(BOOST_ROOT ...)` must appear *after*
`option(SIL_MODE ...)` in `CMakeLists.txt`. Verify the ordering:

```bash
grep -n "option(SIL_MODE\|BOOST_ROOT" src/flightstack/CMakeLists.txt
```

### Motor commands are all zeros

The sim time sent is `≤ 10.0`. See the timing table in Part 7 - the
controller doesn't activate until `TAKEOFF_START_TIME_SECONDS`.

### Simulator container exits immediately with a `pychrono`/`rclpy` import error

Usually a `PYTHONPATH` ordering issue in `simulator_entrypoint.sh`. Confirm
both imports succeed manually:

```bash
docker compose run --rm simulator bash -c "python -c 'import pychrono, rclpy; print(\"ok\")'"
```

### Simulator window doesn't render / GPU errors under WSLg

Confirm WSL is updated (`wsl --update` from Windows PowerShell) and that
`/dev/dxg`, `/usr/lib/wsl`, and `/mnt/wslg` exist on the host before
`docker compose up`. As a workaround, pass `--visualize false` in
`docker-compose.yml`'s `command:` for the simulator service to run headless.

### DDS nodes not seeing each other between containers

Both containers must use identical `ROS_DOMAIN_ID` and `network_mode: host`,
and both must mount the same `fastdds_no_shm.xml`. Verify:

```bash
docker compose exec flightstack env | grep ROS
docker compose exec simulator    env | grep ROS
```

Both must show the same `ROS_DOMAIN_ID`.

### Manually exercising the service (debugging without the simulator)

Useful when you suspect the bridge itself, not the simulator, is at fault.
Use `time: 11.0` or higher - the controller only activates after
`TAKEOFF_START_TIME_SECONDS = 10.0`:

```bash
docker compose exec flightstack \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source /ros2_ws/install/setup.bash && \
           ros2 service call /sil/compute_control \
           uav_interfaces/srv/ComputeControl \
           '{time: 11.0,
             px: 0.0, py: 0.0, pz: -1.0,
             vx: 0.0, vy: 0.0, vz: 0.0,
             qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0,
             wx: 0.0, wy: 0.0, wz: 0.0}'"
```

Expected response - **8 per-motor thrusts in Newtons**, not normalized
`[0, 1]` values (values will differ based on controller and gains):

```
response:
uav_interfaces.srv.ComputeControl_Response(
  motor_commands=[2.84, 1.91, 3.27, 2.05, 2.91, 1.88, 3.10, 2.42])
```

Eight non-zero, physically-reasonable thrust values confirms the full
pipeline: state injected → controller computed → motor commands returned.

---

## Appendix - Complete File Tree

```
~/projects/acsl_sil_ws/
├── ACSL_SIL_Flightstack_Guide.md     ← this guide
├── SIL.md                            ← reference for everything outside src/flightstack and src/simulator
├── docker-compose.yml                ← orchestrates both containers
├── plot_logs.py                      ← flightstack-only log diagnostics (local, not containerized)
├── plot_logs_sim.py                  ← sim-vs-flightstack comparison diagnostics
├── log_compare.py                    ← overlays multiple controller runs
├── docker/
│   ├── flightstack.Dockerfile        ← builds + (re)compiles the flightstack SIL image
│   ├── simulator.Dockerfile          ← ROS2 Jazzy + Miniconda + PyChrono 9.0.1
│   ├── simulator_entrypoint.sh       ← builds uav_interfaces, merges PYTHONPATH, execs the command
│   └── fastdds_no_shm.xml            ← forces UDP-only DDS transport across containers
├── .vscode/                          ← editor config only, not part of the SIL runtime
└── src/
    ├── uav_interfaces/                ← ROS2 service definition package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── srv/
    │       └── ComputeControl.srv     ← the service contract between sim and flightstack
    ├── flightstack/                   ← ACSL-flightstack (cloned, then modified)
    │   ├── FLIGHTSTACK_CHANGES.md     ← full diff list vs. upstream
    │   ├── CMakeLists.txt             ← MODIFIED: SIL_MODE option, SELECTED_CONTROLLER cache var, guards
    │   ├── include/flightstack/
    │   │   ├── multi_threaded_node.hpp  ← MODIFIED: px4/mocap guards, SilInterface member
    │   │   ├── config/config.hpp        ← MODIFIED: SELECTED_CONTROLLER fallback only
    │   │   └── sil/
    │   │       └── sil_interface.hpp    ← NEW: SIL service server header
    │   ├── src/
    │   │   ├── main.cpp                 ← MODIFIED: SIL_MODE branch
    │   │   ├── multi_threaded_node.cpp  ← MODIFIED: hardware setup guards
    │   │   ├── config/config.cpp        ← MODIFIED: SIL_MODE timing scaffold
    │   │   ├── control/control_callback.cpp ← MODIFIED: SELECTED_CONTROLLER-gated post-integration
    │   │   └── sil/
    │   │       └── sil_interface.cpp    ← NEW: SIL service implementation
    │   ├── log/                         ← run logs (bind-mounted into the container)
    │   └── params/
    │       ├── config/config.json               ← trajectory + timing config (runtime)
    │       └── control/<controller>/gains_*.json ← per-controller gain matrices (runtime)
    └── simulator/                     ← PyChrono simulator (cloned, then modified)
        ├── SIMULATOR_CHANGES.md       ← full diff list vs. upstream
        ├── main.py                    ← entry point launched by docker-compose.yml's command:
        ├── logs/                      ← .mat / sil_bridge_diagnostics.csv run logs
        └── acsl_pychrono/
            ├── control/
            │   └── flighstack_client.py   ← NEW: ROS2 client bridging sim → /sil/compute_control
            └── ga_tuner/                  ← genetic-algorithm gain optimizer (PID/MRAC variants)
                ├── README.md               ← general (non-SIL) usage
                ├── GA_TUNER_SIL.md         ← NEW: architecture + SIL restart-per-candidate integration
                ├── run_tuner.py            ← entry point: pure-Python evaluation
                ├── run_tuner_sil.py        ← NEW: entry point for `docker compose run --rm ga_tuner`
                └── integration/sil/        ← NEW: writes gains_pid.json + restarts flightstack per candidate
```
