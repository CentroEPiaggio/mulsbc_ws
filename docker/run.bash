#!/usr/bin/env bash
# Run the ARM64 testing container with source bind-mount for iterative development.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
IMAGE_NAME="mulsbc-arm64-test:humble"
CONTAINER_NAME="mulsbc-arm64-dev"

docker run -it --rm \
    --platform linux/arm64 \
    --name "$CONTAINER_NAME" \
    -e ROS_DOMAIN_ID=10 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e MAKEFLAGS="-j1" \
    -v "$WS_ROOT:/ws" \
    "$IMAGE_NAME"
