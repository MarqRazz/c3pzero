FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c", "-o", "pipefail"]

# Bring the container up to date to get the latest ROS2 humble sync
# hadolint ignore=DL3008, DL3013
RUN apt-get update && apt-get upgrade -y && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# Install recommended developer dependencies
# hadolint ignore=DL3008, DL3013
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    clang-format \
    cmake \
    git \
    libbullet-dev \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    tmux \
    tree \
    wget \
    lld \
    ccache \
    python3-colcon-mixin \
    ros-humble-rmw-cyclonedds-cpp \
  && apt-get clean \
  && python3 -m pip install --upgrade --no-cache-dir \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest \
    && rm -rf /var/lib/apt/lists/*

# Install packages required to run the robot
# hadolint ignore=DL3008, DL3013
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-moveit \
    ros-humble-moveit-resources \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Copy over the source code to rosdep install other dependencies.
# The source code in this workspace will be mounted from the host at runtime.
RUN mkdir -p /root/c3pzero_ws/src
WORKDIR /root/c3pzero_ws/src
COPY ./ ./c3pzero/
RUN vcs import < ./c3pzero/c3pzero.repos
WORKDIR /root/c3pzero_ws
# hadolint ignore=SC1091
RUN source /opt/ros/humble/setup.bash \
    && apt-get update -y \
    && apt-get upgrade -y \
    && rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y \
    && rm -rf /var/lib/apt/lists/*

# Use CycloneDDS as the default but also setup Fast DDS to support NVIDIA Isaac Sim.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/root/.ros/cyclonedds_localhost.xml
RUN mkdir -p /root/.ros
COPY ./.docker/fastdds.xml /root/.ros/fastdds.xml
COPY ./.docker/cyclonedds_localhost.xml /root/.ros/cyclonedds_localhost.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml

# Pre-download the gazebo world so it doesn't have to on the first sim launch
RUN ign fuel download -v 4 -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot

# Set up the entrypoint for both container start and interactive terminals.
COPY ./.docker/ros_entrypoint.sh /root/.ros/
RUN echo 'export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{name} {time}]: {message}"' >> /root/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /root/.bashrc && \
    echo "alias cyclone='export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'" >> /root/.bashrc && \
    echo "alias fastdds='export RMW_IMPLEMENTATION=rmw_fastrtps_cpp'" >> /root/.bashrc && \
    echo "alias sws='source ./install/setup.bash'" >> /root/.bashrc && \
    echo "alias symlink_build='colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --event-handlers log-'" >> /root/.bashrc && \
    echo "export PATH=/usr/lib/ccache:$PATH" >> /root/.bashrc && \
    echo "source /root/.ros/ros_entrypoint.sh" >> ~/.bashrc
WORKDIR /root/c3pzero_ws/
