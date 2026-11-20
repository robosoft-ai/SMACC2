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

"""Print the results of every CodeQL query except the MISRA ones as Markdown for the run page.

Usage: codeql_all_results.py <directory of .sarif files>
The results are read from raw_all/<query id>.csv, the queries' own output decoded by the workflow:
CodeQL leaves a result in a header reached through install/ out of the SARIF file. Only places in
this repository's source directories count; an installed header is named by its source path; a line
counts once per query. The suppressions are those of the workflow's filter step. A table of the
queries with their counts and how many of each are in the uploaded SARIF, then every result. Also
writes all_results.csv to the directory. Reports only: always exits 0.
"""
import collections
import csv
import glob
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from codeql_misra_summary import PROBE, REPO, URL, source_headers, source_path  # noqa: E402

# Keep in step with the filter step of codeql.yml.
SUPPRESSED_RULES = {"cpp/empty-block", "cpp/sizeof-side-effect"}


def source_line(place, line):
    try:
        return open(os.path.join(REPO, place), errors="replace").read().split("\n")[line - 1]
    except (OSError, IndexError):
        return ""


def suppressed(rule, place, line):
    if rule in SUPPRESSED_RULES:
        return True
    if "header" in rule and "smacc_tracing" in place:
        return True  # the lttng headers are included more than once on purpose
    return rule == "cpp/commented-out-code" and bool(
        re.search(r"//\s*RCLCPP_", source_line(place, line))
    )


def read_raw(directory):
    """Return {query id: {(place, line): message}} from raw_all/, or None when there is none."""
    files = sorted(glob.glob(os.path.join(directory, "raw_all", "*.csv")))
    if not files:
        return None
    index, found = source_headers(), collections.defaultdict(dict)
    for path in files:
        rule = os.path.basename(path)[: -len(".csv")].replace("__", "/")
        if rule.startswith("cpp/misra/"):
            continue
        found[rule]  # a query that ran and gave nothing is listed with 0
        for row in list(csv.reader(open(path, newline="")))[1:]:
            m = URL.match(row[0]) if row else None
            if not m:
                continue
            place, line = source_path(m.group(1), index), int(m.group(2))
            if place.startswith((PROBE, "build/", "install/")) or suppressed(rule, place, line):
                continue
            text = next((c for c in row[1:] if c and not c.startswith("file:")), "")
            found[rule].setdefault((place, line), " ".join(text.split()))
    return found


def read_sarif(directory):
    """Return {query id: {(place, line)}} of what the filtered SARIF files hold."""
    held = collections.defaultdict(set)
    for path in glob.glob(os.path.join(directory, "*.sarif")):
        for run in json.load(open(path)).get("runs", []):
            for result in run.get("results", []):
                where = (result.get("locations") or [{}])[0].get("physicalLocation", {})
                place = where.get("artifactLocation", {}).get("uri", "?")
                held[result.get("ruleId", "")].add(
                    (place, where.get("region", {}).get("startLine", 0))
                )
    return held


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    found = read_raw(sys.argv[1])
    print("\n## The other CodeQL queries\n")
    if found is None:
        print("The workflow decoded no query output (no raw_all directory); see the SARIF file.")
        return
    held = read_sarif(sys.argv[1])
    with_results = {rule: places for rule, places in found.items() if places}
    total = sum(len(places) for places in with_results.values())
    uploaded = sum(len(set(places) & held[rule]) for rule, places in with_results.items())
    print(
        f"{total} results in the SMACC2 sources from {len(with_results)} of {len(found)} queries, read "
        f"from the queries' own output. {uploaded} of them are in the uploaded results; CodeQL leaves "
        "out a result in a header reached through install/.\n"
    )
    print("| Query | Results | In the uploaded results |")
    print("|--|--|--|")
    order = sorted(with_results, key=lambda rule: (-len(with_results[rule]), rule))
    for rule in order:
        places = with_results[rule]
        print(f"| {rule} | {len(places)} | {len(set(places) & held[rule])} |")
    rows = []
    for rule in order:
        lines = []
        for (place, line), text in sorted(with_results[rule].items()):
            mark = "" if (place, line) in held[rule] else "  (not uploaded)"
            lines.append(f"{place}:{line}: {text}{mark}")
            rows.append((rule, place, line, text, "no" if mark else "yes"))
        print(f"\n### {rule} ({len(lines)})\n\n```\n" + "\n".join(lines) + "\n```")
    with open(os.path.join(sys.argv[1], "all_results.csv"), "w", newline="") as out:
        writer = csv.writer(out)
        writer.writerow(["query", "file", "line", "message", "uploaded"])
        writer.writerows(rows)


if __name__ == "__main__":
    main()
