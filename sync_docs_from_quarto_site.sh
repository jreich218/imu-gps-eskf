#!/usr/bin/env bash

set -euo pipefail

SOURCE_DOCS_DIR="/home/dfw/dev/quarto_site/docs"
DEST_DOCS_DIR="/home/dfw/dev/state-estimation/docs"

if [[ ! -d "${SOURCE_DOCS_DIR}" ]]; then
    echo "Missing source docs directory: ${SOURCE_DOCS_DIR}" >&2
    exit 1
fi

if ! command -v rsync >/dev/null 2>&1; then
    echo "rsync is required to sync docs." >&2
    exit 1
fi

mkdir -p "${DEST_DOCS_DIR}"

rsync -a --delete \
    --exclude='*.yml' \
    --exclude='*.yaml' \
    "${SOURCE_DOCS_DIR}/" "${DEST_DOCS_DIR}/"

echo "Synced docs to ${DEST_DOCS_DIR}"
