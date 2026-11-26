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
RUN bash -c "export MAKEFLAGS=-j3 && \
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

# Change to:
CMD ["bash", "-c", \
     "source /opt/ros/jazzy/setup.bash && \
      source /ros2_ws/install/setup.bash && \
      export MAKEFLAGS=-j3 && \
      colcon build --packages-select flightstack \
        --cmake-args -DSIL_MODE=ON -DCMAKE_BUILD_TYPE=Release -DSELECTED_CONTROLLER=${CONTROLLER_TYPE:-PID} && \
      source /ros2_ws/install/setup.bash && \
      ros2 run flightstack flightstack"]