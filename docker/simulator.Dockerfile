FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

# -------------------------------------------------
# System deps
# -------------------------------------------------
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

# -------------------------------------------------
# Docker CLI (docker-outside-of-docker): lets the SIL-GA tuner (run_tuner_sil.py,
# running in this same image via the ga_tuner service) shell out to
# `docker compose restart flightstack` against the *host's* Docker daemon, via a
# mounted /var/run/docker.sock. Not used by the normal `simulator` service/main.py.
# -------------------------------------------------
RUN apt-get update && apt-get install -y \
    docker.io \
    docker-compose-v2 \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------------------------
# Install Miniconda
# -------------------------------------------------
RUN wget -q https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh \
    -O /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p /opt/conda && \
    rm /tmp/miniconda.sh

ENV PATH=/opt/conda/bin:$PATH

# -------------------------------------------------
# Accept conda ToS
# -------------------------------------------------
RUN conda tos accept --override-channels \
    --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels \
    --channel https://repo.anaconda.com/pkgs/r

# -------------------------------------------------
# Configure conda
# -------------------------------------------------
RUN conda config --add channels conda-forge && \
    conda config --set channel_priority strict

# -------------------------------------------------
# Create chrono_v9 environment
# -------------------------------------------------
RUN conda create -y -n chrono_v9 python=3.12 pip

# -------------------------------------------------
# Install dependencies
# -------------------------------------------------
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

# -------------------------------------------------
# Install official PyChrono 9.0.1 package
# -------------------------------------------------
RUN wget -O /tmp/pychrono.conda \
    https://anaconda.org/projectchrono/pychrono/9.0.1/download/linux-64/pychrono-9.0.1-py312hf1de3a3_6463.conda && \
    conda run -n chrono_v9 conda install -y /tmp/pychrono.conda

# -------------------------------------------------
# ROS workspace
# -------------------------------------------------
WORKDIR /ros2_ws

# 1. Take the specialized simulator script from your local 'docker' folder, 
# copy it into the root of the container, and rename it to just '/entrypoint.sh'
COPY docker/simulator_entrypoint.sh /entrypoint.sh

# 2. Make that script executable inside the container
RUN chmod +x /entrypoint.sh

# 3. Tell the container to run this script every time it starts up
ENTRYPOINT ["/entrypoint.sh"]