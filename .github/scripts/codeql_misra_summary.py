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

"""Print the MISRA C++:2023 dynamic memory results of a CodeQL run as Markdown for the run page.

Usage: codeql_misra_summary.py <directory of .sarif files>
A table of the rules 21.6.2 to 21.6.5 with the result count of each query, then every result as
file:line and message. The results are read from raw_<query>.csv in the same directory when the
workflow decoded them: CodeQL leaves a result in a header reached through install/ out of the SARIF
file. Such a header is named by its source path, and a line counts once. Then the self-test: which lines marked EXPECT in .github/codeql/misra_probe.cpp
the queries found. The results in that file are removed from the SARIF files afterwards, so they are
not uploaded as alerts. Reports only: always exits 0.
"""
import collections
import csv
import glob
import json
import os
import re
import sys

PROBE = ".github/codeql/misra_probe"  # the .cpp and the .hpp
REPO = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
URL = re.compile(r"^file://.*?/((?:install|build|\.github|smacc2[^/]*)/.*?):(\d+):\d+:\d+:\d+$")

QUERIES = [
    ("21.6.2", "cpp/misra/dynamic-memory-managed-manually"),
    ("21.6.3", "cpp/misra/advanced-memory-management-used"),
    ("21.6.4", "cpp/misra/global-sized-operator-delete-shall-be-defined"),
    ("21.6.4", "cpp/misra/global-unsized-operator-delete-shall-be-defined"),
    ("21.6.5", "cpp/misra/pointer-to-an-incomplete-class-type-deleted"),
]


def source_headers():
    """Map a header's path below its include/ directory to its path in the repository."""
    index = collections.defaultdict(set)
    for top in sorted(os.listdir(REPO)):
        if not top.startswith("smacc2"):
            continue
        for folder, _, names in os.walk(os.path.join(REPO, top)):
            if "/include" not in folder:
                continue
            for name in names:
                path = os.path.relpath(os.path.join(folder, name), REPO)
                index[path.split("/include/", 1)[1]].add(path)
    return index


def source_path(place, index):
    """Name a header under install/<package>/include/ by its source file, when exactly one matches."""
    if not place.startswith("install/") or "/include/" not in place:
        return place
    rest = place.split("/include/", 1)[1]
    for key in (rest, rest.split("/", 1)[-1]):
        if len(index.get(key, ())) == 1:
            return next(iter(index[key]))
    return place


def read_raw(directory, found, probe):
    """Read the decoded query results; returns False when the workflow wrote none."""
    index, any_file = source_headers(), False
    for _, query in QUERIES:
        path = os.path.join(directory, "raw_" + query.rsplit("/", 1)[1] + ".csv")
        try:
            rows = open(path, newline="")
        except OSError:
            continue
        any_file = True
        for row in list(csv.reader(rows))[1:]:
            m = URL.match(row[0]) if row else None
            if not m:
                continue
            place, line, text = source_path(m.group(1), index), int(m.group(2)), row[-1]
            if place.startswith(PROBE):
                probe[(os.path.basename(place), line)].append(text)
            elif not place.startswith(("build/", "install/")):
                found[query].add((place, line, text))
    return any_file


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    found = collections.defaultdict(set)
    probe = collections.defaultdict(list)
    loaded, others, files = set(), 0, glob.glob(os.path.join(sys.argv[1], "*.sarif"))
    for path in files:
        sarif = json.load(open(path))
        for run in sarif.get("runs", []):
            tool = run.get("tool", {})
            for part in [tool.get("driver", {})] + tool.get("extensions", []):
                loaded.update(rule.get("id", "") for rule in part.get("rules", []))
            kept = []
            for result in run.get("results", []):
                rule = result.get("ruleId") or result.get("rule", {}).get("id", "")
                where = (result.get("locations") or [{}])[0].get("physicalLocation", {})
                place = where.get("artifactLocation", {}).get("uri", "?")
                line = where.get("region", {}).get("startLine", 0)
                text = " ".join(result.get("message", {}).get("text", "").split())
                if place.startswith(PROBE):
                    probe[(os.path.basename(place), line)].append(text)
                    continue
                kept.append(result)
                if rule.startswith("cpp/misra/"):
                    found[rule].add((place, line, text))
                else:
                    others += 1
            run["results"] = kept
        json.dump(sarif, open(path, "w"))
    sarif_found, sarif_probe = found, probe
    found, probe = collections.defaultdict(set), collections.defaultdict(list)
    raw = read_raw(sys.argv[1], found, probe)
    if not raw:
        found, probe = sarif_found, sarif_probe
    print("## MISRA C++:2023 dynamic memory rules (CodeQL)\n")
    if not files:
        print("No SARIF file was written; the analysis did not run.")
        return
    total = sum(len(found[query]) for _, query in QUERIES)
    print(
        f"{total} results for rules 21.6.2 to 21.6.5. Rule 21.6.1 is not checked: SMACC2 allocates by "
        f"design. The other queries of this run gave {others} results after the filter.\n"
    )
    if raw:
        in_sarif = sum(len(sarif_found[query]) for _, query in QUERIES)
        print(
            f"Read from the queries' own output. {in_sarif} of these are in the uploaded results; "
            "CodeQL leaves out a result in a header reached through install/.\n"
        )
    print("| Rule | Query | Results |")
    print("|--|--|--|")
    for rule, query in QUERIES:
        count = len(found[query]) if query in loaded else "query not loaded"
        print(f"| {rule} | {query} | {count} |")
    for rule, query in QUERIES:
        if found[query]:
            lines = [f"{place}:{line}: {text}" for place, line, text in sorted(found[query])]
            print(f"\n### {rule}: {query} ({len(lines)})\n\n```\n" + "\n".join(lines) + "\n```")
    here = os.path.dirname(os.path.abspath(__file__))
    marked = []
    for name in ("misra_probe.cpp", "misra_probe.hpp"):
        try:
            source = open(os.path.join(here, "..", "codeql", name)).read().split("\n")
        except OSError:
            continue
        for n, s in enumerate(source, 1):
            if m := re.search(r";\s*// EXPECT (.*)", s):
                marked.append(((name, n), m.group(1)))
    if not marked:
        return
    hit = sum(1 for key, _ in marked if key in probe)
    print(f"\n### Self-test: {hit} of {len(marked)} marked lines of {PROBE}.cpp and .hpp found\n")
    print("| File | Line | Shape | Found |")
    print("|--|--|--|--|")
    for key, what in marked:
        found_text = "; ".join(sorted(set(probe[key]))) if key in probe else "**no**"
        print(f"| {key[0]} | {key[1]} | {what} | {found_text} |")


if __name__ == "__main__":
    main()
