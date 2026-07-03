#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${ROOT_DIR}/build/perception_sim_app"

if [[ ! -x "${BIN}" ]]; then
    echo "Executable not found at ${BIN}. Run scripts/build.sh first." >&2
    exit 1
fi

cd "${ROOT_DIR}"

mkdir -p data/runs data/csv

# ------------------------------------------------------------
# Run multiple experiments with different seeds
# ------------------------------------------------------------
for i in {0..5}; do
    echo "Running seed $i"
    "${BIN}" synthetic "$i"
done
