#!/bin/bash

set -e

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/utils/print_color.sh
WORKSPACE_ROOT="${ROOT}/.."
DOCKER_DIR="${ROOT}/../docker"

BASE_NAME="robomaster-cv"
IMAGE_NAME="$BASE_NAME-image"
CONTAINER_NAME="$BASE_NAME-container"
DEV_DIR="../"
PLATFORM="$(uname -m)"
if [ -f /proc/device-tree/model ] && [[ "$(cat /proc/device-tree/model)" =~ "Jetson Orin Nano" ]]; then
    print_info "Detected Jetson Orin Nano, building librealsense from source"
    IMAGE_KEY="ros2_humble.realsense_source.tools.user"
    MODEL="JON"
else
    print_info "Did not detect Jetson Orin Nano, installing librealsense from package"
    IMAGE_KEY="ros2_humble.realsense_pkg.tools.user"
    MODEL="PC"
fi


DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
DOCKER_ARGS+=("-e ROS_DOMAIN_ID")
DOCKER_ARGS+=("-e USER")

# DOCKER_ARGS+=("-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml")

if [[ $MODEL == "JON" ]]; then
    # Map host's display socket to docker
    # This should only happen for non-windows
    DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix") # TODO @arthur can probably forward for linux and/or macOS too
    DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")

    DOCKER_ARGS+=("-v /usr/bin/tegrastats:/usr/bin/tegrastats")
    DOCKER_ARGS+=("-v /tmp/argus_socket:/tmp/argus_socket")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcufft.so.10:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcufft.so.10")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h:/usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h")
    DOCKER_ARGS+=("-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
    DOCKER_ARGS+=("-v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
    DOCKER_ARGS+=("-v /opt/nvidia/nsight-systems-cli:/opt/nvidia/nsight-systems-cli")
    DOCKER_ARGS+=("--pid=host")
    DOCKER_ARGS+=("-v /opt/nvidia/vpi2:/opt/nvidia/vpi2")
    DOCKER_ARGS+=("-v /usr/share/vpi2:/usr/share/vpi2")

    # If jtop present, give the container access
    if [[ $(getent group jtop) ]]; then
        DOCKER_ARGS+=("-v /run/jtop.sock:/run/jtop.sock:ro")
        JETSON_STATS_GID="$(getent group jtop | cut -d: -f3)"
        DOCKER_ARGS+=("--group-add $JETSON_STATS_GID")
    fi
fi

# Remove any exited containers.
if [ "$(docker ps -a --quiet --filter status=exited --filter name=$CONTAINER_NAME)" ]; then
    docker rm $CONTAINER_NAME > /dev/null
fi

# Re-use existing container.
if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
    print_info "Attaching to running container: $CONTAINER_NAME"
    MSYS_NO_PATHCONV=1 \
    docker exec -i -t -u admin --workdir /robomaster_cv/ $CONTAINER_NAME /bin/bash $@
    exit 0
fi

BUILD_ARGS+=("--build-arg" "USERNAME="admin"")
BUILD_ARGS+=("--build-arg" "USER_UID=`id -u`")
BUILD_ARGS+=("--build-arg" "USER_GID=`id -g`")
BUILD_ARGS+=("--build-arg" "PLATFORM=$PLATFORM")

# Check if GPU is installed
if [[ $PLATFORM == "x86_64" ]]; then
    if type nvidia-smi &>/dev/null; then
        GPU_ATTACHED=(`nvidia-smi -a | grep "Attached GPUs"`)
        if [ ! -z $GPU_ATTACHED ]; then
            BUILD_ARGS+=("--build-arg" "HAS_GPU="true"")
        fi
    fi
fi

print_info "Building $IMAGE_KEY base as image: $BASE_NAME"
print_info "Running $ROOT/build_image_layers.sh -i \"$IMAGE_KEY\" --image_name \"$BASE_NAME\""
$ROOT/build_image_layers.sh -i "$IMAGE_KEY" --image_name "$BASE_NAME" -r

# Check result
if [ $? -ne 0 ]; then
    if [[ -z $(docker image ls --quiet $BASE_NAME) ]]; then
        print_error "Building image failed and no cached image found for $BASE_NAME, aborting."
        exit 1
    else
        print_warning "Unable to build image, but cached image found."
    fi
fi

# Check image is available
if [[ -z $(docker image ls --quiet $BASE_NAME) ]]; then
    print_error "No built image found for $BASE_NAME, aborting."
    exit 1
fi

MSYS_NO_PATHCONV=1 \
docker run -it --rm \
    --privileged \
    --network host \
    ${DOCKER_ARGS[@]} \
    -v $WORKSPACE_ROOT:/robomaster_cv/ \
    -v /dev/*:/dev/* \
    --name "$CONTAINER_NAME" \
    --user="admin" \
    --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
    --workdir /robomaster_cv/ \
    --runtime nvidia \
    $@ \
    $BASE_NAME \
    /bin/bash

# If we run on jetson, we should add this back
# --runtime nvidia
