#!/usr/bin/env bash
set -euo pipefail
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/octopus_demo --scene static_chicane --samples 7
