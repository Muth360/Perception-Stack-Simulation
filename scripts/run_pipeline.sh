#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# run_pipeline.sh
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

# ------------------------------------------------------------
# Run with varying seed if no input file provided
# ------------------------------------------------------------
if [[ $# -ge 1 ]]; then
    "${BIN}" "$1"
else
    for i in {0..5}; do
        echo "Running seed $i"
        "${BIN}" dummy "$i"
    done
fi
