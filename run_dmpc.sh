#!/usr/bin/env bash
#
# run_dmpc.sh — run the DMPC solver on a single config and write its outputs
# back into the same folder:
#   <folder>/trajectories.txt   drone trajectories
#   <folder>/goals.txt          goal positions over time
#   <folder>/solver.log         solver stdout/stderr
#
# The folder and its config.json must already exist. The solver runs on a
# temporary copy of the config with its output paths redirected into the folder,
# so the original config.json is left untouched and the outputs always land
# beside it (independent of the current working directory).
#
# Usage:
#   ./run_dmpc.sh <folder>              # a folder containing config.json
#   ./run_dmpc.sh <folder>/config.json  # or the config file directly
#
# Example:
#   mkdir -p demos/my_demo
#   cp online_dmpc/cpp/config/config.json demos/my_demo/   # then edit as needed
#   ./run_dmpc.sh demos/my_demo
#
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <folder-or-config.json>" >&2
  exit 2
fi

# Accept either a folder (uses <folder>/config.json) or a config file directly.
arg="$1"
if [[ -d "$arg" ]]; then
  out="$(cd "$arg" && pwd)"
  cfg="$out/config.json"
elif [[ -f "$arg" ]]; then
  out="$(cd "$(dirname "$arg")" && pwd)"
  cfg="$out/$(basename "$arg")"
else
  echo "ERROR: not found: $arg" >&2
  exit 1
fi
[[ -f "$cfg" ]] || { echo "ERROR: no config file at $cfg" >&2; exit 1; }

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"   # repo root (this script lives here)
RUN="$ROOT/online_dmpc/cpp/bin/run"
PY="${PYTHON:-python3}"

if [[ ! -x "$RUN" ]]; then
  echo "ERROR: solver binary not found at $RUN" >&2
  echo "Build it: cmake -S \"$ROOT/online_dmpc/cpp\" -B \"$ROOT/online_dmpc/cpp/build\" \\" >&2
  echo "          && cmake --build \"$ROOT/online_dmpc/cpp/build\" -j" >&2
  exit 1
fi

# Run on a temporary copy whose output paths point into <out>, so the source
# config.json is never modified and the outputs use absolute paths.
tmp="$(mktemp)"
trap 'rm -f "$tmp"' EXIT
"$PY" - "$cfg" "$tmp" "$out" <<'PYEOF'
import json, sys
src, dst, outdir = sys.argv[1:4]
with open(src) as f:
    cfg = json.load(f)
cfg['output_trajectories_paths'] = [f"{outdir}/trajectories.txt"]
cfg['output_goals_paths'] = [f"{outdir}/goals.txt"]
with open(dst, 'w') as f:
    json.dump(cfg, f, indent=2)
PYEOF

echo ">> solver: $cfg"
"$RUN" "$tmp" > "$out/solver.log" 2>&1
echo "   wrote: $out/{trajectories.txt, goals.txt, solver.log}"
