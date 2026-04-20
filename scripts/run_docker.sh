#!/usr/bin/env bash
# run_docker.sh
#
# Build (if needed) and run nova2_revo2_bridge_standalone inside Docker.
# Must be executed from the REPOSITORY ROOT, e.g.:
#
#   bash scripts/run_docker.sh
#
# Requirements on the host:
#   • Docker installed and running
#   • SenseCom running on the HOST (the container uses --network host so the
#     SDK inside the container can reach SenseCom's named-pipe / socket)
#   • Revo2 hands connected via USB serial ports on the HOST
#     (/dev/ttyUSB0, /dev/ttyUSB1 by default – see RIGHT_PORT / LEFT_PORT in
#     nova2_revo2_bridge_standalone.py)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="nova2-bridge"

echo "==> Building Docker image '${IMAGE_NAME}' (context: ${REPO_ROOT})"
docker build \
    -t "${IMAGE_NAME}" \
    -f "${SCRIPT_DIR}/Dockerfile" \
    "${REPO_ROOT}"

echo "==> Starting container (--network host, --privileged for serial access)"
exec docker run \
    --rm \
    --interactive \
    --tty \
    --privileged \
    --network host \
    --device /dev/ttyUSB0 \
    --device /dev/ttyUSB1 \
    "${IMAGE_NAME}"
