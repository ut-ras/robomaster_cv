#!/bin/bash

set -e

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/utils/print_color.sh

BASE_NAME="robomaster-cv"
IMAGE_NAME="azhangvo/robomaster_cv"
IMAGE_KEY="ros2_humble.realsense_source.tools"

print_info "Building $IMAGE_KEY base as image: $BASE_NAME"
print_info "Running $ROOT/build_image_layers.sh -i \"$IMAGE_KEY\" --image_name \"$BASE_NAME\""
$ROOT/build_image_layers.sh -i "$IMAGE_KEY" --image_name "$IMAGE_NAME" -r --docker_arg "--platform=linux/amd64,linux/arm64"

# Check result
if [ $? -ne 0 ]; then
    if [[ -z $(docker image ls --quiet $IMAGE_NAME) ]]; then
        print_error "Building image failed and no cached image found for $IMAGE_NAME, aborting."
        exit 1
    else
        print_warning "Unable to build image, but cached image found."
    fi
fi

print_info "Finished building $IMAGE_KEY base as image: $BASE_NAME"