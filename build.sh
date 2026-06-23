#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
BUILD_DIR="${RKV_BUILD_DIR:-$SCRIPT_DIR/build}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Debug}"
TARGET_NAME="PIVOT_VIEWER"

echo "========== 编译 PIVOT =========="
mkdir -p "$BUILD_DIR"
cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
cmake --build "$BUILD_DIR" --target "$TARGET_NAME" -j"$(nproc)"
echo "✅ 完成: $SCRIPT_DIR/bin/$TARGET_NAME"

if [ -f "$SCRIPT_DIR/print_logo.sh" ]; then
    bash "$SCRIPT_DIR/print_logo.sh"
fi
