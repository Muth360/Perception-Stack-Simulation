#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# run_pipeline.sh
#
# Runs the perception_sim_app executable, optionally against a real point
# cloud file (.pcd/.ply). Without an argument, a synthetic cloud is used.
#
# Usage:
#   ./scripts/run_pipeline.sh
#   ./scripts/run_pipeline.sh data/my_scan.pcd
# -----------------------------------------------------------------------------
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${ROOT_DIR}/build/perception_sim_app"

if [[ ! -x "${BIN}" ]]; then
    echo "Executable not found at ${BIN}. Run scripts/build.sh first." >&2
    exit 1
fi

mkdir -p "${ROOT_DIR}/data"
cd "${ROOT_DIR}"

if [[ $# -ge 1 ]]; then
    "${BIN}" "$1"
else
    "${BIN}"
fi
