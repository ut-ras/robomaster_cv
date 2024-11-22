#!/bin/bash

set -e

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/utils/print_color.sh

BASE_NAME="robomaster-cv"
IMAGE_NAME="$BASE_NAME-image"
CONTAINER_NAME="$BASE_NAME-container"
IMAGE_KEY="ros2_humble.realsense_source.tools"


# Remove any exited containers.
if [ "$(docker ps -a --quiet --filter status=exited --filter name=$CONTAINER_NAME)" ]; then
    docker rm $CONTAINER_NAME > /dev/null
fi

print_info "Building $IMAGE_KEY base as image: $BASE_NAME"
print_info "Running $ROOT/build_image_layers.sh -i \"$IMAGE_KEY\" --image_name \"$BASE_NAME\""
$ROOT/build_image_layers.sh -i "$IMAGE_KEY" --image_name "$BASE_NAME" -r

if [[ $BUILD_ONLY == "TRUE" ]]; then
    print_info "Build complete, exiting."
    exit 0
fi

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