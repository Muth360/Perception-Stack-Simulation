#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# compare_runs.sh
#
# Convenience wrapper: sets up a venv with the analysis dependencies (if
# needed) and runs compare_runs.py.
#
# Usage:
#   ./scripts/compare_runs.sh RUN_ID_A RUN_ID_B
#   ./scripts/compare_runs.sh RUN_ID_A RUN_ID_B --output-dir data/comparison/my_run
#
# List available run_ids first with:
#   ./scripts/analysis/list_runs.py --csv-dir data/csv
# -----------------------------------------------------------------------------
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${ROOT_DIR}/.venv-analysis"

if [[ ! -d "${VENV_DIR}" ]]; then
    echo "Setting up analysis venv at ${VENV_DIR}..."
    python3 -m venv "${VENV_DIR}"
    "${VENV_DIR}/bin/pip" install --quiet -r "${ROOT_DIR}/scripts/analysis/requirements.txt"
fi

cd "${ROOT_DIR}"
"${VENV_DIR}/bin/python3" "${ROOT_DIR}/scripts/analysis/compare_runs.py" --csv-dir data/csv "$@"
