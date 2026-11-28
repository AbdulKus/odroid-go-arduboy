#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
: "${IDF_PATH:?Activate ESP-IDF v5.4.3 with export.sh first}"
patch_file="$PWD/tools/idf-sdspi.patch"
if ! git -C "$IDF_PATH" apply --reverse --check "$patch_file" 2>/dev/null; then
    if ! git -C "$IDF_PATH" apply "$patch_file"; then
        echo "The ODROID-GO SDSPI patch does not match this ESP-IDF checkout." >&2
        exit 1
    fi
fi
version="${1:-dev}"
idf.py -D "PROJECT_VER=$version" -D ARDUGO_QUIET_TEST=OFF build
python3 tools/package.py --version "$version"
