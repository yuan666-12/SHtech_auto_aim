#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cmake -S "$root" -B "$root/build_real" \
    -DINFERENCE_BACKEND="${INFERENCE_BACKEND:-ONNX}" \
    -DUSE_HIKCAM="${USE_HIKCAM:-ON}" \
    -DENABLE_LEGACY_UART="${ENABLE_LEGACY_UART:-OFF}" "$@"
cmake --build "$root/build_real" --parallel "${BUILD_JOBS:-2}"
