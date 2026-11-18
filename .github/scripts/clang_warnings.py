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

"""List the compiler warnings and errors in a colcon build's console output.

Usage: clang_warnings.py <console log> [--markdown]
Each distinct warning is printed once with the number of times the compiler gave it: a warning in a
header is repeated for every source file that includes the header. A header reached through the
install base is named by its path in the repository. Always exits 0.
"""
import collections
import os
import re
import sys

REPO = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
DIAGNOSTIC = re.compile(r"^(/[^:\s]+):(\d+:\d+): (warning|error|fatal error): (.*)$")


def collect(path):
    found = {"warning": collections.Counter(), "error": collections.Counter()}
    places = {}
    try:
        lines = open(path, errors="replace")
    except OSError:
        return None
    for line in lines:
        m = DIAGNOSTIC.match(line.rstrip())
        if not m:
            continue
        name, at, kind, text = m.groups()
        if name not in places:
            real = os.path.realpath(name)  # a symlinked install header is its source file
            places[name] = os.path.relpath(real, REPO) if real.startswith(REPO + os.sep) else real
        kind = "warning" if kind == "warning" else "error"
        found[kind][f"{places[name]}:{at}: {kind}: {text}"] += 1
    return found


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    markdown = "--markdown" in sys.argv[2:]
    found = collect(sys.argv[1])
    if found is None:
        print("## clang build\n\nNo console output was captured; the build did not run.")
        return
    warnings, errors = found["warning"], found["error"]
    head = (
        f"clang build: {sum(errors.values())} errors, {sum(warnings.values())} warnings "
        f"in {len(warnings)} distinct places"
    )
    body = [f"{line}  (x{n})" for group in (errors, warnings) for line, n in sorted(group.items())]
    if markdown:
        print(f"## {head}\n")
        if body:
            print("```\n" + "\n".join(body) + "\n```")
    else:
        print("\n".join([head] + body))


if __name__ == "__main__":
    main()
