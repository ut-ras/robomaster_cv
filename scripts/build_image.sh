#!/usr/bin/env bash
set -euo pipefail

: "${LIBREALSENSE_JOBS:=}"
set -euo pipefail

# Paths
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker_dir="$script_dir/../docker"

# Enable BuildKit & Buildx
export DOCKER_BUILDKIT=1

# Create or reuse a Buildx builder instance specific to this project
BUILDER_NAME="builder_robomaster_cv"
if ! docker buildx inspect "$BUILDER_NAME" >/dev/null 2>&1; then
  docker buildx create --name "$BUILDER_NAME" --use
else
  docker buildx use "$BUILDER_NAME"
fi
# Ensure builder is ready
docker buildx inspect --bootstrap

IMAGE_NAME="azhangvo/robomaster_cv:latest"

# Build & push multi-arch image
docker buildx build \
  --builder "$BUILDER_NAME" \
  --platform linux/amd64,linux/arm64 \
  --build-arg LIBREALSENSE_JOBS="$LIBREALSENSE_JOBS" \
  --file "$docker_dir/Dockerfile" \
  --tag "$IMAGE_NAME" \
  --cache-to=type=inline \
  --push \
  "$docker_dir"