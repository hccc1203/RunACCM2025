#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build-x86-mock"

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DENABLE_X86_MOCK=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" -j"$(nproc)"

echo "x86 mock build passed."
