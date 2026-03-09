#!/usr/bin/env bash
# Build the ARM64 Docker testing image on an x86_64 host via QEMU emulation.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
IMAGE_NAME="mulsbc-arm64-test:humble"
BUILDER_NAME="arm64-builder"

echo "==> Registering QEMU binfmt handlers..."
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

echo "==> Setting up buildx builder '${BUILDER_NAME}'..."
if ! docker buildx inspect "$BUILDER_NAME" >/dev/null 2>&1; then
    docker buildx create --name "$BUILDER_NAME" --use
else
    docker buildx use "$BUILDER_NAME"
fi

echo "==> Building ARM64 image '${IMAGE_NAME}'..."
docker buildx build \
    --platform linux/arm64 \
    --load \
    -t "$IMAGE_NAME" \
    -f "$SCRIPT_DIR/Dockerfile" \
    "$WS_ROOT"

echo "==> Done. Run with: bash docker/run.bash"
