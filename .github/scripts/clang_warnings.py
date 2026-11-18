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

Usage: clang_warnings.py <console log> [--markdown [core test console log]]
The warnings are grouped by warning flag and sorted by place. Each distinct warning is printed once
with the number of times the compiler gave it: a warning in a header is repeated for every source
file that includes the header. A header reached through the
install base is named by its path in the repository. With --markdown the output is the run page:
a short summary, a table of the flags, then the list. Always exits 0.
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
        found[kind][(places[name], *map(int, at.split(":")), f"{kind}: {text}")] += 1
    return found


def groups(found):
    """Yield (title, lines): the errors, then one group per warning flag, each sorted by place."""
    by_flag = collections.defaultdict(list)
    for item, n in found["error"].items():
        by_flag["errors"].append((item, n))
    for item, n in found["warning"].items():
        m = re.search(r" \[(-W[^\]]+)\]$", item[3])
        by_flag[m.group(1) if m else "no flag"].append(
            ((*item[:3], item[3][: m.start()] if m else item[3]), n)
        )
    for flag, items in sorted(by_flag.items(), key=lambda g: (g[0] != "errors", g[0])):
        total = sum(n for _, n in items)
        lines = [f"{p}:{line}:{col}: {text}  (x{n})" for (p, line, col, text), n in sorted(items)]
        yield f"{flag} ({total} in {len(items)} {'place' if len(items) == 1 else 'places'})", lines


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
    listed = list(groups(found))
    if not markdown:
        print(head)
        for title, lines in listed:
            print(f"\n{title}\n" + "\n".join(lines))
        return
    failed = sum(errors.values()) or sum(warnings.values())
    print(f"## clang build: {'FAIL' if failed else 'PASS'}\n")
    tests = ""
    if len(sys.argv) > 3:
        try:
            result = re.findall(
                r"^### result: (\d+) failure", open(sys.argv[3], errors="replace").read(), re.M
            )
        except OSError:
            result = []
        tests = (
            f" The core tests on this build: {result[-1]} failures."
            if result
            else " The core tests did not finish."
        )
    print(
        f"{sum(errors.values())} errors and {sum(warnings.values())} warnings in {len(warnings)} distinct places. "
        f"The standard is zero warnings.{tests}\n"
    )
    if listed:
        print("| Warning flag | Warnings | Places |")
        print("|--|--|--|")
        for title, lines in listed:
            m = re.match(r"(.*) \((\d+) in (\d+) places?\)$", title)
            print(f"| {m.group(1)} | {m.group(2)} | {m.group(3)} |")
        print()
    for title, lines in listed:
        print(f"### {title}\n\n```\n" + "\n".join(lines) + "\n```\n")


if __name__ == "__main__":
    main()
