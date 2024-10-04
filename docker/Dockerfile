FROM ros:humble-perception

ENV WS_DIR="/ros2_ws"
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

# Setup non-root admin user
ARG USERNAME=admin
ARG USER_UID=1000
ARG USER_GID=1000

# Install prerequisites
RUN apt-get update && apt-get install -y \
        sudo \
        udev \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Reuse triton-server user as 'admin' user if exists
RUN if [ $(getent group triton-server) ]; then \
        groupmod -o --gid ${USER_GID} -n ${USERNAME} triton-server ; \
        usermod -l ${USERNAME} -u ${USER_UID} -m -d /home/${USERNAME} triton-server ; \
        mkdir -p /home/${USERNAME} ; \
        sudo chown ${USERNAME}:${USERNAME} /home/${USERNAME} ; \
    fi

# Create the 'admin' user if not already exists
RUN if [ ! $(getent passwd ${USERNAME}) ]; then \
        groupadd --gid ${USER_GID} ${USERNAME} ; \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} ; \
    fi

# Update 'admin' user
RUN echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && adduser ${USERNAME} video && adduser ${USERNAME} plugdev && adduser ${USERNAME} sudo && adduser ${USERNAME} dialout

ENV USERNAME=${USERNAME}
ENV USER_GID=${USER_GID}
ENV USER_UID=${USER_UID}

# Install Realsense libraries
RUN apt-get update \
 && apt-get install -y \
    build-essential \
    cmake \
    git-all \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
 && rm -rf /var/lib/apt/lists/*

 # Install ROS libraries
RUN apt-get update \
 && apt-get install -y \
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
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Install helpful tools
RUN apt-get update && apt-get install -y \
        vim \
        nano \
        dos2unix \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Copy scripts
RUN  mkdir -p /usr/local/bin/scripts
COPY scripts/*.sh /usr/local/bin/scripts/
RUN  dos2unix /usr/local/bin/scripts/*.sh
RUN  chmod +x /usr/local/bin/scripts/*.sh

ENV USERNAME=${USERNAME}
ENV USER_GID=${USER_GID}
ENV USER_UID=${USER_UID}

# Switch to non-root user and return to root
USER ${USERNAME}
RUN --mount=type=cache,target=/var/cache/apt \
    rosdep update
USER root

ARG DEBIAN_FRONTEND=dialog