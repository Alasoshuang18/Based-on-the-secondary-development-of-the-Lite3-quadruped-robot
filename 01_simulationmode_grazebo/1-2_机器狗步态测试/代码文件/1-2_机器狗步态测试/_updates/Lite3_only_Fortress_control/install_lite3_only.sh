#!/usr/bin/env bash
set -euo pipefail
UPDATE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 "$UPDATE_ROOT/install_lite3_only.py"
