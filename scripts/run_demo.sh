#!/usr/bin/env bash
# End-to-end demo: simulate -> build -> fuse -> score & plot.
# Run from the repo root:  ./scripts/run_demo.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "==> [1/4] Python venv + deps"
# A venv keeps this reproducible on externally-managed Python (PEP 668).
if [ ! -d .venv ]; then
    python3 -m venv .venv
fi
PY=.venv/bin/python
"$PY" -m pip install -q -r python/requirements.txt

echo "==> [2/4] Simulate trajectory + sensor fixes"
"$PY" python/simulate.py --out-dir data

echo "==> [3/4] Build C++ tracker (Release)"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build build -j

echo "==> [4/4] Fuse + score + plot"
./build/sft_app data/measurements.csv data/track.csv
"$PY" python/visualize.py --data-dir data --track data/track.csv --out data/fusion.png

echo
echo "Done. See data/fusion.png"
