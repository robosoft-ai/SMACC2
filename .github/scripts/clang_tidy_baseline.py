#!/usr/bin/env python3

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

"""Normalize clang-tidy output and compare it with a checked-in baseline.

Usage:
  clang_tidy_baseline.py normalize <repo-root> <raw-output>...   > current.txt
  clang_tidy_baseline.py compare <baseline.txt> <current.txt>

A normalized line is "<count>\t<path relative to the repo>: <warning|error>: <message> [<check>]",
without line or column numbers, so an edit above a known finding does not move it out of the
baseline, while a new finding or a second instance of a known one does fail. compare exits 1 on
any error line, any key missing from the baseline, or any count above the baseline's, and lists
baseline entries that no longer occur so the baseline can be pruned by regenerating it.
"""
import collections
import os
import re
import sys

LINE = re.compile(
    r"^(?P<path>/[^:]+):\d+:\d+: (?P<kind>warning|error): (?P<msg>.*?)(?: \[(?P<check>[^\]]+)\])?$"
)


def normalize(repo_root, raw_paths):
    root = os.path.realpath(repo_root) + os.sep
    counts = collections.Counter()
    for raw in raw_paths:
        for line in open(raw, errors="replace"):
            m = LINE.match(line.rstrip("\n"))
            if not m:
                continue
            path = os.path.realpath(m.group("path"))
            if not path.startswith(root):
                continue  # system or dependency header; the header filter should already exclude it
            if path[len(root) :].startswith(("build/", "install/", "log/")):
                continue  # a copy under the build or install tree; the same header counts from its source path
            check = f" [{m.group('check')}]" if m.group("check") else ""
            counts[f"{path[len(root):]}: {m.group('kind')}: {m.group('msg')}{check}"] += 1
    for key in sorted(counts):
        print(f"{counts[key]}\t{key}")


def load(path):
    out = {}
    for line in open(path, errors="replace"):
        line = line.rstrip("\n")
        if not line or line.startswith("#"):
            continue
        count, key = line.split("\t", 1)
        out[key] = int(count)
    return out


def compare(baseline_path, current_path):
    baseline, current = load(baseline_path), load(current_path)
    failures = 0
    for key, count in sorted(current.items()):
        if ": error: " in key:
            print(f"FAIL (compiler error): {count}x {key}")
            failures += 1
        elif key not in baseline:
            print(f"FAIL (new): {count}x {key}")
            failures += 1
        elif count > baseline[key]:
            print(f"FAIL (more instances, {baseline[key]} -> {count}): {key}")
            failures += 1
    stale = [k for k in baseline if current.get(k, 0) < baseline[k]]
    for key in sorted(stale):
        print(f"stale baseline entry ({baseline[key]} -> {current.get(key, 0)}): {key}")
    print(
        f"clang-tidy gate: {sum(current.values())} diagnostics in {len(current)} keys, "
        f"baseline {sum(baseline.values())} in {len(baseline)}, {failures} failure(s), {len(stale)} stale"
    )
    return 1 if failures else 0


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "normalize":
        normalize(sys.argv[2], sys.argv[3:])
    elif len(sys.argv) == 4 and sys.argv[1] == "compare":
        sys.exit(compare(sys.argv[2], sys.argv[3]))
    else:
        print(__doc__)
        sys.exit(2)
