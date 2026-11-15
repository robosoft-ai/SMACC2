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

"""Condense the heaptrack output written by heap_attribution.sh.

Usage: heap_summary.py <out-dir> <unit> <units in the short run> <units in the long run>
Reads short.summary.txt, long.summary.txt, diff_leaks.txt and long_stacks.txt from the out dir.
"""
import re
import sys

UNITS = {"": 1, "B": 1, "K": 1024, "M": 1024**2, "G": 1024**3}
ENTRY = re.compile(r"^(-?[\d.]+)([BKMG]?)B? leaked over (-?\d+) calls from:$")
PROJECT = re.compile(r"\b(smacc2|sm_\w+|cl_\w+)::")


def summary(path):
    text = open(path, errors="replace").read()
    calls = re.search(r"calls to allocation functions: (\d+)", text)
    leaked = re.search(r"total memory leaked: ([\d.]+)([BKMG]?)", text)
    if not calls or not leaked:
        sys.exit(f"cannot read {path}")
    return int(calls.group(1)), float(leaked.group(1)) * UNITS[leaked.group(2)]


def project_frame(frames):
    """The innermost frame with a source line outside the compiler's and the system's headers."""
    for i, line in enumerate(frames):
        m = re.match(r"\s+at (\S+)$", line)
        if m and not m.group(1).startswith(("/usr/", "./", "../")) and i > 0:
            where = m.group(1).split("/include/")[-1].split("/SMACC2/")[-1]
            return f"{frames[i - 1].strip()} at {where}"
    return frames[0].strip() if frames else "?"


def entries(path, pattern):
    out, cur = [], None
    for line in open(path, errors="replace").read().splitlines():
        m = pattern.match(line)
        if m:
            cur = [m, []]
            out.append(cur)
        elif cur is not None and line.startswith("    "):
            cur[1].append(line)
        elif not line.strip():
            cur = None
    return out


def main(out, unit, short_units, long_units):
    (calls_s, live_s), (calls_l, live_l) = (
        summary(f"{out}/{r}.summary.txt") for r in ("short", "long")
    )
    work = int(long_units) - int(short_units)
    print(
        f"\nlive heap at the end: short run {live_s / 1024:.0f} kB, long run {live_l / 1024:.0f} kB,"
        f" difference {(live_l - live_s) / 1024:+.0f} kB over {work} more {unit}"
    )
    if work > 0:
        print(f"that is {(live_l - live_s) / work:+.1f} bytes per {unit.rstrip('s')}")
        print(
            f"heap allocations: about {(calls_l - calls_s) / work:.0f} per {unit.rstrip('s')}"
            f" ({calls_l - calls_s} more calls in the long run)"
        )
    by_frame = {}
    for m, frames in entries(f"{out}/diff_leaks.txt", ENTRY):
        frame = project_frame(frames)
        by_frame[frame] = by_frame.get(frame, 0) + float(m.group(1)) * UNITS[m.group(2)]
    grown = sorted(((size, frame) for frame, size in by_frame.items()), reverse=True)
    print("\ncall sites whose live memory differs most between the runs (long minus short):")
    if not grown:
        print("  none")
    for size, frame in grown[:8]:
        print(f"  {size / 1024:+9.1f} kB  {frame}")
    print(
        "  a few kB either way is what happened to be allocated at the instant each run was killed"
    )
    # one line per stack, root first: "frame;frame;...;frame count"; charge each to the innermost
    # frame in SMACC2 or machine code
    sites, total = {}, 0
    for line in open(f"{out}/long_stacks.txt", errors="replace"):
        stack, _, count = line.rstrip().rpartition(" ")
        if not count.isdigit():
            continue
        frames = [f for f in stack.split(";") if f.strip()]
        total += int(count)
        key = "(no SMACC2 frame)"
        for i in range(len(frames) - 1, -1, -1):
            if PROJECT.search(frames[i]):
                key = frames[i]
                break
        sites[key] = sites.get(key, 0) + int(count)
    per_unit = unit.rstrip("s")
    print(
        f"\nwhere the long run's {total} allocations come from (innermost SMACC2 function on the stack):"
    )
    for key, count in sorted(sites.items(), key=lambda kv: -kv[1])[:12]:
        rate = f"{count / int(long_units):8.1f} per {per_unit}" if int(long_units) else ""
        print(f"  {100 * count / total:5.1f} %  {rate}  {key[:150]}")
    print(f"\nfull backtraces: {out}/diff_leaks.txt; flame graph input: {out}/long_stacks.txt")


if __name__ == "__main__":
    if len(sys.argv) == 5:
        main(*sys.argv[1:])
    else:
        print(__doc__)
