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

"""Turn a gate script's console output into a Markdown summary for the workflow run page.

Usage: gate_summary.py sanitizers <build name> <console log>
       gate_summary.py clang-tidy <console log> <current.txt>
Prints Markdown on stdout; the workflow appends it to $GITHUB_STEP_SUMMARY. Always exits 0: the
gate script's own exit status decides the job.
"""
import collections
import re
import sys


def read(path):
    try:
        return open(path, errors="replace").read().splitlines()
    except OSError:
        return []


def cell(text):
    return text.replace("|", "\\|")


def sanitizers(name, console):
    lines = read(console)
    if not lines:
        print(f"## {name}\n\nNo console output was captured; the gate script did not run.")
        return
    sections, cur = [], None
    for line in lines:
        if line.startswith("### "):
            cur = [line[4:], []]
            sections.append(cur)
        elif cur is not None:
            cur[1].append(line)
    result = next((s[0] for s in sections if s[0].startswith("result:")), "result: not reached")
    fails = [line for line in lines if line.startswith("FAIL")]
    verdict = "PASS" if result == "result: 0 failure(s)" else "FAIL"
    print(f"## {name}: {verdict}\n")
    print(f"{result[8:]}\n")
    print("| Test | Result | Measured |")
    print("|--|--|--|")
    for title, body in sections:
        if title.startswith("result:"):
            continue
        failed = sum(1 for line in body if line.startswith("FAIL"))
        runs = [line for line in body if re.search(r" run \d+ offset ", line)]
        facts = [
            line
            for line in body
            if line and not line.startswith("FAIL") and line not in runs and ": exit " not in line
        ]
        if runs:
            clean = sum(1 for line in runs if line.endswith("exit 0 hang no"))
            facts.insert(0, f"{clean} of {len(runs)} interrupted runs exited 0 without a hang")
        if title == "sanitizer reports":
            facts = [line for line in facts if line.startswith("sanitizer reports:")] or facts[-1:]
        status = f"FAIL ({failed})" if failed else "ok"
        print(f"| {cell(title)} | {status} | {cell('; '.join(facts))} |")
    if fails:
        print("\n### Failures\n")
        for line in fails:
            print(f"- `{line}`")
    print("\nThe full logs and report files are in this run's artifact.")


def clang_tidy(console, current):
    lines = read(console)
    total = next((line for line in reversed(lines) if line.startswith("clang-tidy gate:")), None)
    if total is None:
        print("## clang-tidy gate\n\nThe gate did not reach its result line; see the step log.")
        return
    failures = int(re.search(r"(\d+) failure", total).group(1))
    print(f"## clang-tidy gate: {'PASS' if failures == 0 else 'FAIL'}\n")
    print(f"{total[len('clang-tidy gate: '):]}\n")
    fails = [line for line in lines if line.startswith("FAIL")]
    stale = [line for line in lines if line.startswith("stale baseline entry")]
    if fails:
        print("### Not in the baseline\n")
        for line in fails:
            print(f"- `{line}`")
        print()
    if stale:
        print("### Baseline entries that no longer occur\n")
        for line in stale:
            print(f"- `{line}`")
        print()
    counts, keys = collections.Counter(), collections.Counter()
    for line in read(current):
        m = re.match(r"(\d+)\t.*\[([^\]]+)\]$", line)
        if m:
            counts[m.group(2)] += int(m.group(1))
            keys[m.group(2)] += 1
    if counts:
        print("### Diagnostics by check\n")
        print("| Check | Diagnostics | Keys |")
        print("|--|--|--|")
        for check, n in counts.most_common():
            print(f"| {check} | {n} | {keys[check]} |")
    print("\nThe raw clang-tidy output is in this run's artifact.")


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "sanitizers":
        sanitizers(sys.argv[2], sys.argv[3])
    elif len(sys.argv) == 4 and sys.argv[1] == "clang-tidy":
        clang_tidy(sys.argv[2], sys.argv[3])
    else:
        print(__doc__)
