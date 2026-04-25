FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ARG GTSAM_VERSION=4.2a9
ARG GTSAM_BUILD_JOBS=2
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    curl \
    dbus-x11 \
    fluxbox \
    git \
    python3-catkin-tools \
    python3-pip \
    python3-rosdep \
    novnc \
    ros-noetic-cv-bridge \
    ros-noetic-jsk-recognition-msgs \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-robot-state-publisher \
    ros-noetic-rosbag \
    ros-noetic-tf \
    ros-noetic-xacro \
    libatlas-base-dev \
    libbackward-cpp-dev \
    libblas-dev \
    libceres-dev \
    libdw-dev \
    libeigen3-dev \
    libgflags-dev \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libgoogle-glog-dev \
    liblapack-dev \
    libsuitesparse-dev \
    mesa-utils \
    websockify \
    x11vnc \
    x11-xserver-utils \
    xauth \
    xterm \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --branch ${GTSAM_VERSION} --depth 1 https://github.com/borglab/gtsam.git /tmp/gtsam \
    && cmake -S /tmp/gtsam -B /tmp/gtsam/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
    && cmake --build /tmp/gtsam/build -j"${GTSAM_BUILD_JOBS}" \
    && cmake --install /tmp/gtsam/build \
    && echo "/usr/local/lib" > /etc/ld.so.conf.d/usr-local.conf \
    && ldconfig \
    && rm -rf /tmp/gtsam

WORKDIR /catkin_ws
COPY glins /catkin_ws/src/glins
COPY rtklib /catkin_ws/src/rtklib
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/start-desktop.sh /usr/local/bin/start-desktop.sh

RUN chmod +x /entrypoint.sh /usr/local/bin/start-desktop.sh

ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
