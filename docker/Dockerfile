# syntax=docker/dockerfile:1.4

###################################
# 1) Dependencies Stage
###################################
# ARG ROS_DISTRO=humble
# ARG BASE_IMAGE=ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ros:humble-perception
FROM --platform=$BUILDPLATFORM ${BASE_IMAGE} AS deps

# Install apt deps and ROS2 Humble libraries with cache
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt/lists \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        curl gnupg lsb-release build-essential cmake python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-angles \
        ros-${ROS_DISTRO}-apriltag \
        ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
        ros-${ROS_DISTRO}-bondcpp \
        ros-${ROS_DISTRO}-camera-calibration-parsers \
        ros-${ROS_DISTRO}-camera-info-manager \
        ros-${ROS_DISTRO}-compressed-image-transport \
        ros-${ROS_DISTRO}-compressed-depth-image-transport \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-demo-nodes-cpp \
        ros-${ROS_DISTRO}-demo-nodes-py \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-example-interfaces \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-image-geometry \
        ros-${ROS_DISTRO}-image-pipeline \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-image-transport-plugins \
        ros-${ROS_DISTRO}-launch-xml \
        ros-${ROS_DISTRO}-launch-yaml \
        ros-${ROS_DISTRO}-launch-testing \
        ros-${ROS_DISTRO}-launch-testing-ament-cmake \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-nav2-msgs \
        ros-${ROS_DISTRO}-nav2-mppi-controller \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-ompl \
        ros-${ROS_DISTRO}-resource-retriever \
        ros-${ROS_DISTRO}-rqt-graph \
        ros-${ROS_DISTRO}-rqt-reconfigure \
        ros-${ROS_DISTRO}-rqt-image-view \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-rviz-common \
        ros-${ROS_DISTRO}-rviz-default-plugins \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-v4l2-camera \
        ros-${ROS_DISTRO}-vision-opencv \
        ros-${ROS_DISTRO}-vision-msgs \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# (Optional) Refresh ROS key
RUN rm -f /usr/share/keyrings/ros-archive-keyring.gpg \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

###################################
# 2) Builder Stage (librealsense)
###################################
FROM deps AS librealsense
ARG LIBREALSENSE_SOURCE_VERSION=v2.55.1

# Build librealsense as root
WORKDIR /opt/librealsense_build
COPY scripts/install-realsense-dependencies.sh ./
RUN chmod +x install-realsense-dependencies.sh && ./install-realsense-dependencies.sh
ARG LIBREALSENSE_JOBS
COPY scripts/build-librealsense.sh ./
RUN chmod +x build-librealsense.sh && \
    ./build-librealsense.sh -n -v $LIBREALSENSE_SOURCE_VERSION ${LIBREALSENSE_JOBS:+ -j $LIBREALSENSE_JOBS}

###################################
# 3) Runtime Stage
###################################
FROM deps AS runtime
ARG ROS_DISTRO=humble USERNAME=admin USER_UID=1000 USER_GID=1000

# Copy ROS libraries
# COPY --from=deps /opt/ros /opt/ros

# Copy librealsense install (binaries, headers, Python bindings)
COPY --from=librealsense /usr/local /usr/local

# Install udev rules
COPY udev_rules/99-realsense-libusb-custom.rules /etc/udev/rules.d/
# RUN udevadm control --reload-rules && udevadm trigger

# # Copy helper scripts
# COPY scripts/setup.sh /usr/local/bin/setup.sh
# COPY scripts/workspace-entrypoint.sh /usr/local/bin/workspace-entrypoint.sh
# RUN chmod +x /usr/local/bin/*.sh

# # Create non-root user
# RUN groupadd --gid $USER_GID $USERNAME \
#     && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
#     && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME

# Environment
ENV PATH="/opt/ros/${ROS_DISTRO}/bin:/usr/local/bin:$PATH"
ENV LD_LIBRARY_PATH="/usr/local/lib"
ENV ROS_DISTRO=${ROS_DISTRO}

# Install helpful tools
RUN apt-get update && apt-get install -y \
        vim \
        nano \
        dos2unix \
        ffmpeg \
        python3-pip \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean
RUN pip3 install zstandard

# Switch to non-root and defer workspace build to entrypoint
# USER $USERNAME
# WORKDIR /home/$USERNAME
# ENTRYPOINT ["workspace-entrypoint.sh"]
CMD ["bash"]