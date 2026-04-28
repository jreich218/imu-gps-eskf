#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build"
EXECUTABLE_PATH="${BUILD_DIR}/imu-gps-eskf"
PLOT_VENV_DIR="${BUILD_DIR}/plot_asset_venv"
PYTHON_BIN="${PLOT_VENV_DIR}/bin/python"
PIP_BIN="${PLOT_VENV_DIR}/bin/pip"
PLOT_REQUIREMENTS_PATH="${REPO_ROOT}/scripts/requirements-plot.txt"
PLOT_SCRIPT_PATH="${REPO_ROOT}/scripts/plot_xy.py"
PLOT_ASSET_REL_PATH="assets/xy_trajectory.png"
PLOT_ASSET_PATH="${REPO_ROOT}/${PLOT_ASSET_REL_PATH}"

if [[ ! -x "${EXECUTABLE_PATH}" ]]; then
  echo "Missing built app: ${EXECUTABLE_PATH}" >&2
  exit 1
fi

python3 -m venv "${PLOT_VENV_DIR}"
"${PIP_BIN}" install --upgrade pip
"${PIP_BIN}" install -r "${PLOT_REQUIREMENTS_PATH}"

cd "${REPO_ROOT}"

"${EXECUTABLE_PATH}"
"${PYTHON_BIN}" "${PLOT_SCRIPT_PATH}"

if ! git diff --exit-code -- "${PLOT_ASSET_REL_PATH}"; then
  echo "Plot asset is out of date: ${PLOT_ASSET_PATH}" >&2
  exit 1
fi
