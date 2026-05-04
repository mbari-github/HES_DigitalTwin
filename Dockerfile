# ────────────────────────────────────────────────────────────────────────────
# HES_DigitalTwin — development image
# Base: ROS 2 Jazzy on Ubuntu 24.04 (Noble)
# ────────────────────────────────────────────────────────────────────────────
FROM ros:jazzy

# ── System packages ─────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
        gnupg \
        lsb-release \
        python3-pip \
        python3-numpy \
        python3-scipy \
        python3-tk \
        # ROS utilities
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-rviz2 \
        # Extra ROS message / service packages
        ros-jazzy-visualization-msgs \
        ros-jazzy-std-srvs \
        ros-jazzy-pluginlib \
        # Testing
        ros-jazzy-launch-testing-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# ── Pinocchio via robotpkg ───────────────────────────────────────────────────
# robotpkg is the official upstream distribution channel for Pinocchio.
RUN mkdir -p /etc/apt/keyrings \
    && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
        -o /etc/apt/keyrings/robotpkg.asc \
    && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] \
        http://robotpkg.openrobots.org/packages/debian/pub \
        $(lsb_release -cs) robotpkg" \
        > /etc/apt/sources.list.d/robotpkg.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        robotpkg-py312-pinocchio \
    && rm -rf /var/lib/apt/lists/*

# ── Pinocchio environment variables ─────────────────────────────────────────
ENV PATH=/opt/openrobots/bin:${PATH}
ENV PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:${PKG_CONFIG_PATH}
ENV LD_LIBRARY_PATH=/opt/openrobots/lib:${LD_LIBRARY_PATH}
ENV PYTHONPATH=/opt/openrobots/lib/python3.12/site-packages:${PYTHONPATH}
ENV CMAKE_PREFIX_PATH=/opt/openrobots:${CMAKE_PREFIX_PATH}

# ── Workspace setup ─────────────────────────────────────────────────────────
WORKDIR /ros2_ws

# Copy only the repository source (assumes the build context is the repo root)
COPY . /ros2_ws/src/HES_DigitalTwin

# Build all packages
RUN /bin/bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"

# ── Shell auto-source ────────────────────────────────────────────────────────
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo 'source /ros2_ws/install/setup.bash' >> /root/.bashrc \
    && echo 'export PYTHONPATH=/opt/openrobots/lib/python3.12/site-packages:${PYTHONPATH}' \
        >> /root/.bashrc

# Default: interactive shell with workspace already sourced
CMD ["/bin/bash"]
