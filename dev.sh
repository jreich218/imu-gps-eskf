#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${REPO_ROOT}/build"
EXECUTABLE_PATH="${BUILD_DIR}/imu-gps-eskf"

cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" -DBUILD_TESTING=ON
cmake --build "${BUILD_DIR}"
ctest --test-dir "${BUILD_DIR}" --output-on-failure

cd "${REPO_ROOT}"

"${EXECUTABLE_PATH}"
