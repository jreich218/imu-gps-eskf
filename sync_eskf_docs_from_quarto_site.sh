#!/usr/bin/env bash

set -euo pipefail

SOURCE_ESKF_DOCS_DIR="/home/dfw/dev/quarto_site/eskf_docs"
DEST_ESKF_DOCS_DIR="/home/dfw/dev/imu-gps-eskf/eskf_docs"

if [[ ! -d "${SOURCE_ESKF_DOCS_DIR}" ]]; then
    echo "Missing source eskf_docs directory: ${SOURCE_ESKF_DOCS_DIR}" >&2
    exit 1
fi

if ! command -v rsync >/dev/null 2>&1; then
    echo "rsync is required to sync eskf_docs." >&2
    exit 1
fi

mkdir -p "${DEST_ESKF_DOCS_DIR}"

rsync -a --delete \
    --exclude='*.yml' \
    --exclude='*.yaml' \
    "${SOURCE_ESKF_DOCS_DIR}/" "${DEST_ESKF_DOCS_DIR}/"

echo "Synced eskf_docs to ${DEST_ESKF_DOCS_DIR}"
