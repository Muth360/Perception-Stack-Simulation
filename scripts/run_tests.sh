#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# run_tests.sh
#
# Builds (if needed) and runs the unit test suite via ctest.
# -----------------------------------------------------------------------------
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"

"${ROOT_DIR}/scripts/build.sh"

cd "${BUILD_DIR}"
ctest --output-on-failure
