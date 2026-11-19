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
file:line and message. Reports only: always exits 0.
"""
import collections
import glob
import json
import os
import sys

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
    loaded, others, files = set(), 0, glob.glob(os.path.join(sys.argv[1], "*.sarif"))
    for path in files:
        for run in json.load(open(path)).get("runs", []):
            tool = run.get("tool", {})
            for part in [tool.get("driver", {})] + tool.get("extensions", []):
                loaded.update(rule.get("id", "") for rule in part.get("rules", []))
            for result in run.get("results", []):
                rule = result.get("ruleId") or result.get("rule", {}).get("id", "")
                if not rule.startswith("cpp/misra/"):
                    others += 1
                    continue
                where = result.get("locations", [{}])[0].get("physicalLocation", {})
                place = where.get("artifactLocation", {}).get("uri", "?")
                line = where.get("region", {}).get("startLine", 0)
                text = " ".join(result.get("message", {}).get("text", "").split())
                found[rule].add((place, line, text))
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


if __name__ == "__main__":
    main()
