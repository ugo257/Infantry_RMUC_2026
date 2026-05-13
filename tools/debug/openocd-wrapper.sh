#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO_ROOT=$(CDPATH= cd -- "$SCRIPT_DIR/../.." && pwd)
LOCAL_OPENOCD="$REPO_ROOT/.tools/openocd/current/bin/openocd"

if [ -x "$LOCAL_OPENOCD" ]; then
    exec "$LOCAL_OPENOCD" "$@"
fi

if command -v openocd >/dev/null 2>&1; then
    exec openocd "$@"
fi

echo "openocd not found." >&2
echo "Run '$REPO_ROOT/tools/install_openocd.sh' to install the project-local OpenOCD toolchain." >&2
exit 127
