ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} AS deps

SHELL ["/bin/bash", "-c"]

# Fix Hash Sum mismatch: HTTPS + no pipelining + no cache
RUN sed -i "s|http://archive.ubuntu.com/ubuntu/|https://archive.ubuntu.com/ubuntu/|g" /etc/apt/sources.list && \
    echo 'Acquire::http::Pipeline-Depth "0";' > /etc/apt/apt.conf.d/99fix-hash-sum-mismatch && \
    echo 'Acquire::http::No-Cache "true";' >> /etc/apt/apt.conf.d/99fix-hash-sum-mismatch && \
    echo 'Acquire::BrokenProxy "true";' >> /etc/apt/apt.conf.d/99fix-hash-sum-mismatch && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# --- essential system deps ---
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# --- initialize rosdep properly ---
RUN rosdep init || true
RUN rosdep update

# Workspace
WORKDIR /root/ros2_ws
COPY . ./src

# --- install python deps ---
RUN pip3 install --no-cache-dir -r src/requirements.txt

# --- install missing apt deps via rosdep ---
RUN rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y --skip-keys="cv_bridge"

# Build
FROM deps AS builder
ARG CMAKE_BUILD_TYPE=Release
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build

# Source the workspace
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]