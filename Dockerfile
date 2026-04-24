FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-catkin-tools \
    python3-pip \
    python3-rosdep \
    ros-noetic-cv-bridge \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-robot-state-publisher \
    ros-noetic-rosbag \
    ros-noetic-tf \
    ros-noetic-xacro \
    libatlas-base-dev \
    libblas-dev \
    libceres-dev \
    libdw-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libgtsam-dev \
    liblapack-dev \
    libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws
COPY glins /catkin_ws/src/glins
COPY rtklib /catkin_ws/src/rtklib
COPY docker/entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh \
    && source /opt/ros/noetic/setup.bash \
    && catkin_make

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
