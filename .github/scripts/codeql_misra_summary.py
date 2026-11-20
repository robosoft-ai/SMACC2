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
file:line and message. Then the self-test: which lines marked EXPECT in .github/codeql/misra_probe.cpp
the queries found. The results in that file are removed from the SARIF files afterwards, so they are
not uploaded as alerts. Reports only: always exits 0.
"""
import collections
import glob
import json
import os
import re
import sys

PROBE = ".github/codeql/misra_probe"  # the .cpp and the .hpp

QUERIES = [
    ("21.6.2", "cpp/misra/dynamic-memory-managed-manually"),
    ("21.6.3", "cpp/misra/advanced-memory-management-used"),
    ("21.6.4", "cpp/misra/global-sized-operator-delete-shall-be-defined"),
    ("21.6.4", "cpp/misra/global-unsized-operator-delete-shall-be-defined"),
    ("21.6.5", "cpp/misra/pointer-to-an-incomplete-class-type-deleted"),
]


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
    print("## MISRA C++:2023 dynamic memory rules (CodeQL)\n")
    if not files:
        print("No SARIF file was written; the analysis did not run.")
        return
    total = sum(len(found[query]) for _, query in QUERIES)
    print(
        f"{total} results for rules 21.6.2 to 21.6.5. Rule 21.6.1 is not checked: SMACC2 allocates by "
        f"design. The other queries of this run gave {others} results after the filter.\n"
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
