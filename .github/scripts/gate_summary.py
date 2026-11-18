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
       gate_summary.py clang-tidy <console log> <diagnostics.txt>
Prints Markdown on stdout; the workflow appends it to $GITHUB_STEP_SUMMARY. Always exits 0: the
gate script's own exit status decides the job.
"""
import collections
import os
import re
import sys


def read(path):
    try:
        return open(path, errors="replace").read().splitlines()
    except OSError:
        return []


def cell(text):
    return text.replace("|", "\\|")


# The report kinds each build can give, as sanitizer_reports.py names them.
BUILD_KINDS = (
    ("Thread", ("data races", "lock-order inversions", "other thread errors")),
    ("Address", ("address errors", "memory leaks")),
    ("UBSan", ("undefined-behavior errors",)),
)


def report_counts(name, lines):
    """What the sanitizers of this build reported in SMACC2 code, with every kind named."""
    kinds = [k for word, group in BUILD_KINDS if word in name for k in group]
    if not kinds:
        return "no sanitizer in this build"
    found = next((line[14:] for line in lines if line.startswith("report kinds: ")), None)
    total = next((line for line in lines if line.startswith("sanitizer reports:")), None)
    if found is None or total is None:
        return total or "the report step was not reached"
    counts = {m.group(2): m.group(1) for m in re.finditer(r"(\d+) ([^,]+)", found)}
    external = re.search(r"(\d+) external", total).group(1)
    listed = ", ".join(f"{counts.get(k, '0')} {k}" for k in kinds)
    return f"in SMACC2 code: {listed}; in other libraries: {external} reports"


def sanitizers(name, console):
    lines = read(console)
    if not lines:
        print(f"## {name}\n\nNo console output was captured; the gate script did not run.")
        return
    sections, cur = [], None
    for line in lines:
        if line.startswith("### "):
            cur = [line[4:], [], ""]
            sections.append(cur)
        elif cur is not None and line.startswith("machine: "):
            cur[2] = line[9:]
        elif cur is not None:
            cur[1].append(line)
    result = next((s[0] for s in sections if s[0].startswith("result:")), "result: not reached")
    fails = [line for line in lines if line.startswith("FAIL")]
    verdict = "PASS" if result == "result: 0 failure(s)" else "FAIL"
    reports = report_counts(name, lines)
    print(f"## {name}: {verdict}\n")
    failed_tests = re.search(r"\d+", result)
    print(f"{failed_tests.group(0) if failed_tests else 'unknown'} test failures; {reports}\n")
    print("| Test | Result | Measured | State machine |")
    print("|--|--|--|--|")
    for title, body, machine in sections:
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
            facts, machine = [reports], "all of the above"
        status = f"FAIL ({failed})" if failed else "ok"
        title = title[0].upper() + title[1:]
        print(f"| {cell(title)} | {status} | {cell('; '.join(facts))} | {machine} |")
    reported = next((body for title, body, _ in sections if title == "sanitizer reports"), [])
    reported = [
        line for line in reported if not line.startswith(("report kinds:", "sanitizer reports:"))
    ]
    if fails:
        print("\n### Failures\n\n```\n" + "\n".join(fails) + "\n```")
    if reported:
        print("\n### Sanitizer reports\n\n```\n" + "\n".join(reported) + "\n```")
    print("\nThe full logs and report files are in this run's artifact.")


def clang_tidy(console, listing):
    lines = read(console)
    total = next((line for line in reversed(lines) if line.startswith("clang-tidy gate:")), None)
    if total is None:
        print("## clang-tidy\n\nThe gate did not reach its result line; see the step log.")
        return
    policy_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "clang-tidy-policy.txt"
    )
    policy = dict(
        line.split(": ", 1) for line in read(policy_file) if ": " in line and line[0] != "#"
    )
    fails = [line for line in lines if line.startswith("FAIL")]
    stale = [line for line in lines if line.startswith("stale baseline entry")]
    groups = collections.defaultdict(list)
    for line in read(listing):
        m = re.match(r"(.*?):(\d+:\d+): (.*?)(?: \[([^\]]+)\])?$", line)
        if m:
            key = f"{m.group(1)}: {m.group(3)}" + (f" [{m.group(4)}]" if m.group(4) else "")
            new = any(key in fail for fail in fails)
            groups[m.group(4) or "compiler"].append(("NEW  " if new else "") + line[: m.end(3)])
    count = sum(len(g) for g in groups.values())
    ordered = sorted(groups.items(), key=lambda g: (-len(g[1]), g[0]))
    print(f"## clang-tidy: {'FAIL' if fails else 'PASS'}\n")
    print(
        f"{count} diagnostics from {len(groups)} checks, {len(fails)} not in the baseline. "
        f"The run fails only on a diagnostic that is not in the baseline. "
        f"Each check is listed below with its diagnostics and the policy for it.\n"
    )
    print("| Check | Diagnostics | Not in the baseline |")
    print("|--|--|--|")
    for check, entries in ordered:
        print(f"| {check} | {len(entries)} | {sum(1 for e in entries if e.startswith('NEW'))} |")
    print()
    if fails:
        print("```")
        print("\n".join(fails))
        print("```\n")
    for check, entries in ordered:
        print(f"### {check} ({len(entries)})\n")
        print("```")
        print("\n".join(entries))
        print("```\n")
        print(
            f"Policy: {policy.get(check, 'none written yet; add a line to .github/clang-tidy-policy.txt.')}\n"
        )
    if stale:
        print("### Baseline entries that no longer occur\n")
        print("```")
        print("\n".join(stale))
        print("```")


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "sanitizers":
        sanitizers(sys.argv[2], sys.argv[3])
    elif len(sys.argv) == 4 and sys.argv[1] == "clang-tidy":
        clang_tidy(sys.argv[2], sys.argv[3])
    else:
        print(__doc__)
