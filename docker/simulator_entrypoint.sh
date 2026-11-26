#!/bin/bash
set -e

# 1. Source ROS Jazzy
source /opt/ros/jazzy/setup.bash

# 2. Build the workspace with system Python
cd /ros2_ws
#colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
# REPLACE WITH (only build what the simulator needs):
colcon build \
    --packages-select uav_interfaces \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash

# 3. Activate Conda environment properly
source /opt/conda/etc/profile.d/conda.sh
conda activate chrono_v9

# 4. Merge PYTHONPATHs: PyChrono + Conda Env Packages + ROS 2 + Workspace
# CRITICAL: We added /opt/conda/envs/chrono_v9/lib/python3.12/site-packages
export PYTHONPATH="\
/opt/conda/envs/chrono_v9/share/chrono/python:\
/opt/conda/envs/chrono_v9/lib/python3.12/site-packages:\
/ros2_ws/install/uav_interfaces/local/lib/python3.12/dist-packages:\
/opt/ros/jazzy/lib/python3.12/site-packages:\
/opt/ros/jazzy/local/lib/python3.12/dist-packages:\
${PYTHONPATH}"

# 5. Set PATH to prioritize conda Python
export PATH="/opt/conda/envs/chrono_v9/bin:${PATH}"

exec "$@"