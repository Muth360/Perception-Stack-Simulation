#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# build.sh
#
# Configures and builds perception_sim.
#
# Usage:
#   ./scripts/build.sh                 # normal build
#   EIGEN3_INCLUDE_DIR=/path/to/eigen ./scripts/build.sh   # use a modified Eigen
# -----------------------------------------------------------------------------
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"

CMAKE_EXTRA_ARGS=()
if [[ -n "${EIGEN3_INCLUDE_DIR:-}" ]]; then
    echo "Using custom Eigen include dir: ${EIGEN3_INCLUDE_DIR}"
    CMAKE_EXTRA_ARGS+=("-DEIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}")
fi

mkdir -p "${BUILD_DIR}"
cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release "${CMAKE_EXTRA_ARGS[@]}"
cmake --build "${BUILD_DIR}" -j"$(nproc)"

echo "Build complete. Binaries in ${BUILD_DIR}/"
