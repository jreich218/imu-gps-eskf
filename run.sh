#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${REPO_ROOT}/build"
SYNC_SCRIPT="${REPO_ROOT}/sync_docs_from_quarto_site.sh"
EXECUTABLE_PATH="${BUILD_DIR}/state_estimation"

cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" -DBUILD_TESTING=ON
cmake --build "${BUILD_DIR}"
ctest --test-dir "${BUILD_DIR}" --output-on-failure

"${SYNC_SCRIPT}"

cd "${REPO_ROOT}"

"${EXECUTABLE_PATH}"
