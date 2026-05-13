#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO_ROOT=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
TOOLS_ROOT="$REPO_ROOT/.tools/openocd"

OPENOCD_VERSION="${OPENOCD_VERSION:-0.12.0-7}"

case "$(uname -s)" in
    Darwin) PLATFORM="darwin" ;;
    Linux) PLATFORM="linux" ;;
    *)
        echo "Unsupported platform: $(uname -s)" >&2
        exit 1
        ;;
esac

case "$(uname -m)" in
    arm64|aarch64) ARCH="arm64" ;;
    x86_64|amd64) ARCH="x64" ;;
    *)
        echo "Unsupported architecture: $(uname -m)" >&2
        exit 1
        ;;
esac

ARCHIVE="xpack-openocd-${OPENOCD_VERSION}-${PLATFORM}-${ARCH}.tar.gz"
URL="https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v${OPENOCD_VERSION}/${ARCHIVE}"
TMP_DIR=$(mktemp -d "${TMPDIR:-/tmp}/openocd-install.XXXXXX")
PKG_DIR="$TOOLS_ROOT/xpack-openocd-${OPENOCD_VERSION}"

cleanup() {
    rm -rf "$TMP_DIR"
}
trap cleanup EXIT

mkdir -p "$TOOLS_ROOT"

echo "Downloading $ARCHIVE"
curl -L --fail --retry 3 -o "$TMP_DIR/$ARCHIVE" "$URL"

echo "Extracting OpenOCD"
tar -xzf "$TMP_DIR/$ARCHIVE" -C "$TMP_DIR"

if [ ! -d "$TMP_DIR/xpack-openocd-${OPENOCD_VERSION}" ]; then
    echo "Unexpected archive layout." >&2
    exit 1
fi

rm -rf "$PKG_DIR"
mv "$TMP_DIR/xpack-openocd-${OPENOCD_VERSION}" "$PKG_DIR"
ln -sfn "$PKG_DIR" "$TOOLS_ROOT/current"

echo "OpenOCD installed to $PKG_DIR"
echo "VS Code can now use: $REPO_ROOT/tools/debug/openocd-wrapper.sh"
