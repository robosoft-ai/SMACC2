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

"""Condense the abidiff reports written by abi_compare.sh into one Markdown summary.

Usage: abi_summary.py <old-ref> <new-ref> <status.tsv> <reports-dir>
Keeps what belongs to the project: symbols and types of the standard library, Boost, rclcpp and the
message packages that a library happens to instantiate are left to the full reports.
"""
import os
import re
import sys

FOREIGN = re.compile(
    r"^(std|boost|rclcpp\w*|rcl\w*|tf2\w*|__gnu_cxx|Eigen|fmt|\w+_msgs|rosidl\w*)$"
)
STRING = "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >"
TYPE_LINE = re.compile(r"'(?:class|struct) (.+?)(?: at [^' ]+)?'(?: at \S+?)?(?: changed)?:$")
MEMBER_LINE = re.compile(r"(?:type of '.*' changed|has sub-type changes):$")
SIZE_LINE = re.compile(r"type size changed from (\d+) to (\d+) \(in bits\)")
LAYOUT_LINE = re.compile(r"data members? (?:deletion|insertion)s?:$|offset changed from ")
VTABLE_LINE = re.compile(r"ABI incompatible change to the vtable of (?:class|struct) (.+)$")


def ours_mangled(symbol):
    """True for a function, variable or vtable whose outermost namespace is not a foreign one."""
    m = re.match(r"_Z(?:TV)?NK?(\d+)", symbol)
    if not m:
        return False
    n = int(m.group(1))
    return not FOREIGN.match(symbol[m.end() : m.end() + n])


def ours_type(name):
    return not FOREIGN.match(name.split("::")[0].split("<")[0])


def parse(path):
    removed, added, sizes, vtables, layouts, counts = [], [], [], [], [], ""
    types = []  # (indent, name) of the enclosing type lines
    for raw in open(path, errors="replace"):
        line = raw.rstrip("\n").replace(STRING, "std::string")
        if line.startswith("Functions changes summary:"):
            counts = line[len("Functions changes summary: ") :]
        m = re.match(r"\s+\[([DA])\] (?:'(.*)'\s+\{(\S+?)[,}]|(_Z\S+))", line)
        if m:
            symbol = m.group(3) or m.group(4)
            if ours_mangled(symbol):
                text = m.group(2) or ("vtable of a class" if symbol.startswith("_ZTV") else symbol)
                if m.group(2) or symbol.startswith("_ZTV"):
                    (removed if m.group(1) == "D" else added).append(f"{text} ({symbol})")
            continue
        indent = len(line) - len(line.lstrip())
        m = TYPE_LINE.search(line)
        if m or MEMBER_LINE.search(line):
            # a member or parameter line owns the size lines below it, so they are not the class's
            types = [t for t in types if t[0] < indent] + [(indent, m.group(1) if m else None)]
            continue
        m = SIZE_LINE.search(line)
        if m:
            owner = next((name for ind, name in reversed(types) if ind < indent), None)
            if owner and ours_type(owner):
                sizes.append((owner, int(m.group(1)) // 8, int(m.group(2)) // 8))
            continue
        if LAYOUT_LINE.search(line):
            owner = next((name for ind, name in reversed(types) if ind < indent), None)
            if owner and ours_type(owner):
                layouts.append(owner)
            continue
        m = VTABLE_LINE.search(line)
        if m and ours_type(m.group(1)):
            vtables.append(m.group(1))
    return removed, added, sizes, vtables, layouts, counts


def verdict(code):
    if code in ("new", "removed", "header-only"):
        return code
    rc = int(code)
    if rc & 3:
        return "abidiff error"
    if rc & 8:
        return "incompatible"
    return "changed, review" if rc & 4 else "identical"


def main(old, new, status, reports):
    rows = [line.rstrip("\n").split("\t") for line in open(status)]
    print(f"## Binary interface, {old} to {new}\n")
    print("| Library | Result | Functions |")
    print("|--|--|--|")
    removed, added, sizes, vtables, layouts = [], [], set(), set(), set()
    for pkg, lib, code in rows:
        if code == "header-only":
            continue
        counts = ""
        if code.isdigit():
            r, a, s, v, lay, counts = parse(os.path.join(reports, lib + ".txt"))
            removed += [(lib, x) for x in r]
            added += [(lib, x) for x in a]
            sizes.update(s)
            vtables.update(v)
            layouts.update(lay)
        print(f"| {lib} | {verdict(code)} | {counts} |")
    header_only = [pkg for pkg, lib, code in rows if code == "header-only"]
    if header_only:
        print(f"\nHeader-only, nothing to compare: {', '.join(header_only)}.")
    for title, items in (("Removed", removed), ("Added", added)):
        if items:
            print(
                f"\n### {title} functions, variables and classes of the project ({len(items)})\n"
            )
            for lib, text in items:
                print(f"- {lib}: `{text}`")
    if vtables:
        print(f"\n### Classes whose virtual table changed ({len(vtables)})\n")
        for name in sorted(vtables):
            print(f"- `{name}`")
    if sizes:
        print(f"\n### Classes whose size changed ({len(sizes)})\n")
        print("| Class | Bytes before | Bytes after |")
        print("|--|--|--|")
        for name, before, after in sorted(sizes):
            print(f"| `{name.replace('|', chr(92) + '|')}` | {before} | {after} |")
    same_size = sorted(layouts - {name for name, _, _ in sizes})
    if same_size:
        print(f"\n### Classes whose members moved at the same size ({len(same_size)})\n")
        for name in same_size:
            print(f"- `{name}`")
    print(
        "\nA removed function or a changed virtual table breaks binaries built against the old"
        " library. A size or member change breaks those that construct, derive from or embed the class."
        " The full abidiff output for each library is in the reports directory."
    )


if __name__ == "__main__":
    if len(sys.argv) == 5:
        main(*sys.argv[1:])
    else:
        print(__doc__)
