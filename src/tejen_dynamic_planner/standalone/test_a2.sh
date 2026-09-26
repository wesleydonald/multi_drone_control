#!/usr/bin/env bash
set -euo pipefail
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/separator_demo
