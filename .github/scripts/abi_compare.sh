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

# Compare the binary interface of the SMACC2 shared libraries between two git refs, for the release
# notes: which exported functions were removed, added or changed, which classes changed size and
# which virtual tables moved.
#
# Usage: abi_compare.sh <old-ref> <new-ref> <work-dir>      e.g. abi_compare.sh 3.1.0 HEAD /tmp/abi
# Needs abidiff (apt package abigail-tools) and an environment that already provides the
# dependencies of the SMACC2 packages (source the ROS and underlay setup files first).
# Both refs are exported with git archive, so uncommitted changes are not compared. Every package
# except the sm_* machines is built at RelWithDebInfo into <work-dir>/old and <work-dir>/new; about
# two minutes per side on 32 cores. Full abidiff reports go to <work-dir>/reports, the summary to
# stdout and <work-dir>/summary.md. Exit status: 0 when the comparison ran, whatever it found.
OLD=${1:-}; NEW=${2:-}; WORK=${3:-}
if [ -z "$WORK" ]; then sed -n '17,27p' "$0"; exit 2; fi
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(git -C "$HERE" rev-parse --show-toplevel) || exit 2
for tool in abidiff colcon; do
  command -v $tool > /dev/null || { echo "$tool not found"; exit 2; }
done
mkdir -p "$WORK"; WORK=$(cd "$WORK" && pwd)
if [ -n "$(ls -A "$WORK")" ]; then echo "work dir $WORK is not empty"; exit 2; fi
mkdir -p "$WORK/reports"

for side in old new; do
  [ $side = old ] && ref=$OLD || ref=$NEW
  mkdir -p "$WORK/$side/src"
  git -C "$REPO" archive "$ref" | tar -x -C "$WORK/$side/src" || { echo "cannot export $ref"; exit 2; }
  echo "### building $side ($ref, $(git -C "$REPO" rev-parse --short "$ref"))"
  (cd "$WORK/$side" && colcon --log-base log build --base-paths src --build-base build \
    --install-base install --packages-skip-regex '^sm_' --event-handlers console_direct- \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF > build.out 2>&1) ||
    { echo "build of $ref failed, see $WORK/$side/build.out"; exit 2; }
  # one line per package: "<package> <library path relative to the install base>" or "<package> -"
  for pkg in $(cd "$WORK/$side/install" && ls -d */ | tr -d /); do
    libs=$(cd "$WORK/$side/install" && ls $pkg/lib/*.so 2> /dev/null | grep -v '__rosidl\|_generator_')
    if [ -n "$libs" ]; then for l in $libs; do echo "$pkg $l"; done; else echo "$pkg -"; fi
  done > "$WORK/$side.libs"
done

: > "$WORK/status.tsv"
sort -u "$WORK/old.libs" "$WORK/new.libs" | while read -r pkg lib; do
  in_old=$(grep -cxF "$pkg $lib" "$WORK/old.libs"); in_new=$(grep -cxF "$pkg $lib" "$WORK/new.libs")
  if [ "$lib" = - ]; then
    # header-only on at least one side; report it only when it is header-only on the new side
    [ "$in_new" = 1 ] && printf '%s\t-\theader-only\n' "$pkg" >> "$WORK/status.tsv"
    continue
  fi
  name=$(basename "$lib" .so)
  if [ "$in_old" = 0 ]; then printf '%s\t%s\tnew\n' "$pkg" "$name" >> "$WORK/status.tsv"; continue; fi
  if [ "$in_new" = 0 ]; then printf '%s\t%s\tremoved\n' "$pkg" "$name" >> "$WORK/status.tsv"; continue; fi
  abidiff --headers-dir1 "$WORK/old/install/$pkg/include" --headers-dir2 "$WORK/new/install/$pkg/include" \
    "$WORK/old/install/$lib" "$WORK/new/install/$lib" > "$WORK/reports/$name.txt" 2>&1
  printf '%s\t%s\t%s\n' "$pkg" "$name" "$?" >> "$WORK/status.tsv"
done

python3 -B "$HERE/abi_summary.py" "$OLD" "$NEW" "$WORK/status.tsv" "$WORK/reports" | tee "$WORK/summary.md"
