#!/bin/bash

# Copyright 2026 RobosoftAI Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Run clang-tidy over the packages that make up the core gate and compare the result with the
# checked-in baseline.
#
# Usage: clang_tidy_gate.sh <build-base> <out-dir> [--update-baseline]
# The build base must hold compile_commands.json for every package in PACKAGES (colcon build with
# -DCMAKE_EXPORT_COMPILE_COMMANDS=ON). With --update-baseline the current result replaces the baseline
# instead of being compared with it.
set -u
BUILD=$1; OUT=${2:-clang_tidy_out}; MODE=${3:-}
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
BASELINE=$REPO/.github/clang-tidy-baseline.txt
PACKAGES="smacc2 cl_ros2_timer cl_lifecycle_node cl_nav2z sm_atomic sm_atomic_lifecycle"
CHECKS='-*,bugprone-*,concurrency-*,clang-analyzer-*'
# Only diagnostics located in this repository's own source directories count, wherever the repository
# is checked out. Headers reached through build/ or install/ copies are not matched, so a header is
# counted from its own package's translation units and not again from every dependent's.
REPO_RE=$(printf '%s' "$REPO" | sed 's/[.[\\*^$()+?{}|]/\\&/g')
HEADER_FILTER="^$REPO_RE/(smacc2|smacc2_client_library|smacc2_sm_reference_library|smacc2_performance_tools|smacc2_state_reactor_library|smacc2_event_generator_library)/.*"
mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd)
JOBS=$(nproc)
RAW=()
for p in $PACKAGES; do
  DB=$BUILD/$p/compile_commands.json
  [ -f "$DB" ] || { echo "FAIL: no compile database for $p at $DB"; exit 1; }
  FILES=$(python3 -c "import json,sys; print(' '.join(e['file'] for e in json.load(open(sys.argv[1]))))" "$DB")
  echo "### clang-tidy $p ($(echo $FILES | wc -w) translation units)"
  run-clang-tidy -p "$BUILD/$p" -j "$JOBS" -quiet -checks="$CHECKS" -header-filter="$HEADER_FILTER" $FILES > "$OUT/raw_$p.txt" 2>&1 || true
  RAW+=("$OUT/raw_$p.txt")
done
grep -h "Found compiler error" "${RAW[@]}" | head -3
python3 "$HERE/clang_tidy_baseline.py" normalize "$REPO" "${RAW[@]}" > "$OUT/current.txt"
if [ "$MODE" = "--update-baseline" ]; then
  { echo "# clang-tidy baseline for .github/scripts/clang_tidy_gate.sh: count, then path: kind: message [check]."
    echo "# Regenerate with: .github/scripts/clang_tidy_gate.sh <build-base> <out-dir> --update-baseline"
    cat "$OUT/current.txt"; } > "$BASELINE"
  echo "baseline written: $(grep -vc '^#' "$BASELINE") keys"
  exit 0
fi
python3 "$HERE/clang_tidy_baseline.py" compare "$BASELINE" "$OUT/current.txt"
