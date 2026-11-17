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

"""Summarize a perf profile: tables of where the processor time went, folded stacks and a flame graph.

Input is the text of `perf script -F comm,tid,period,ip,sym,dso` recorded with `-e cpu-clock`, so
that each sample's period is processor time in nanoseconds.
"""
import argparse, collections as C, html, os, re, sys


def parse(path):
    """Yield (comm, tid, nanoseconds, [frames, outermost first])."""
    head, frames = None, []
    with open(path, errors="replace") as f:
        for line in f:
            if not line.strip():
                if head and frames:
                    yield head + (frames[::-1],)
                head, frames = None, []
            elif line[0] in " \t":
                m = re.match(r"\s*([0-9a-f]+) (.*) \(([^()]*)\)\s*$", line)
                if not m:
                    continue
                addr, sym, dso = m.groups()
                if dso == "[unknown]" and sym == "[unknown]":
                    name = "[kernel]" if addr.startswith("ffff") else "[unknown]"
                elif sym == "[unknown]":
                    name = (
                        os.path.basename(dso)
                        if dso[0] == "["
                        else "[" + os.path.basename(dso) + "]"
                    )
                else:
                    name = sym
                if (
                    not frames or frames[-1] != name or name[0] != "["
                ):  # one frame for a run of kernel frames
                    frames.append(name)
            else:
                m = re.match(r"(.*?)\s+(\d+)\s+(\d+)\s*$", line)
                head = (m.group(1).strip(), int(m.group(2)), int(m.group(3))) if m else None
    if head and frames:
        yield head + (frames[::-1],)


def short(name, width=96):
    """A function name cut to a table column: template arguments dropped, then truncated."""
    if len(name) > width:
        out, depth = [], 0
        for ch in name:
            if ch == "<":
                depth += 1
                if depth == 1:
                    out.append("<..>")
            elif ch == ">":
                depth = max(0, depth - 1)
            elif depth == 0:
                out.append(ch)
        name = "".join(out)
    return name if len(name) <= width else name[: width - 2] + ".."


def table(title, rows, total, unit):
    print(f"\n{title}")
    head = f"{'%':>6} {'ms':>9}" + (f" {'us/unit':>8}" if unit else "") + "  function"
    print(head)
    for name, ns in rows:
        per = f" {ns / 1000 / unit:8.3f}" if unit else ""
        print(f"{100 * ns / total:6.2f} {ns / 1e6:9.1f}{per}  {short(name)}")


def flame_svg(stacks, total, path, title):
    """Write a static flame graph: one box per frame, width = share of time, tooltip = name and time."""
    root = {"n": title, "v": total, "c": {}}
    for frames, ns in stacks.items():
        node = root
        for fr in frames:
            node = node["c"].setdefault(fr, {"n": fr, "v": 0, "c": {}})
            node["v"] += ns
    width, row, pad, minw = 1600.0, 17, 10, 0.4
    boxes, depth_max = [], [0]

    def place(node, x, depth):
        w = width * node["v"] / total
        if w < minw:
            return
        boxes.append((x, depth, w, node))
        depth_max[0] = max(depth_max[0], depth)
        cx = x
        for child in sorted(node["c"].values(), key=lambda c: c["n"]):
            place(child, cx, depth + 1)
            cx += width * child["v"] / total

    place(root, 0.0, 0)
    height = (depth_max[0] + 1) * row + 2 * pad + 20
    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width + 2 * pad:.0f}" height="{height}" '
        f'font-family="monospace" font-size="11">',
        f'<rect width="100%" height="100%" fill="#fbfaf7"/>',
        f'<text x="{pad}" y="{pad + 8}" font-size="13">{html.escape(title)}: {total / 1e6:.0f} ms of processor '
        f"time; hover a box for its name and share</text>",
    ]
    for x, depth, w, node in boxes:
        name = node["n"]
        y = height - pad - (depth + 1) * row
        hue = 10 + (sum(map(ord, name)) % 40)  # warm colours, stable per name
        fill = (
            "#d9d4c7" if name[0] == "[" else f"hsl({hue},70%,{62 + sum(map(ord, name[:8])) % 12}%)"
        )
        tip = f"{name}\n{100 * node['v'] / total:.2f} %, {node['v'] / 1e6:.1f} ms"
        out.append(
            f'<g><title>{html.escape(tip)}</title><rect x="{x + pad:.2f}" y="{y}" width="{max(w - 0.3, 0.1):.2f}" '
            f'height="{row - 1}" fill="{fill}"/>'
        )
        chars = int((w - 6) / 6.7)
        if chars >= 3:
            label = short(name, 400)
            label = label if len(label) <= chars else label[: chars - 2] + ".."
            out.append(f'<text x="{x + pad + 3:.2f}" y="{y + 12}">{html.escape(label)}</text>')
        out.append("</g>")
    out.append("</svg>")
    with open(path, "w") as f:
        f.write("\n".join(out))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("script_text")
    ap.add_argument("--out-dir", help="write stacks.folded and flame.svg here")
    ap.add_argument("--thread", help="keep one thread: a thread id, or 'busiest'")
    ap.add_argument(
        "--under", help="keep only the stacks through a function whose name contains this text"
    )
    ap.add_argument(
        "--per-unit",
        type=float,
        default=0,
        help="units of work in the run, to print microseconds per unit",
    )
    ap.add_argument("--top", type=int, default=25)
    a = ap.parse_args()

    samples = list(parse(a.script_text))
    if not samples:
        sys.exit("no samples in " + a.script_text)
    threads = C.Counter()
    comm_of = {}
    for comm, tid, ns, _ in samples:
        threads[tid] += ns
        comm_of[tid] = comm
    all_ns = sum(threads.values())
    print(
        f"samples: {len(samples)}, processor time {all_ns / 1e6:.0f} ms, threads: {len(threads)}"
    )
    print(f"\nthreads\n{'%':>6} {'ms':>9}  tid      name")
    for tid, ns in threads.most_common(8):
        print(f"{100 * ns / all_ns:6.2f} {ns / 1e6:9.1f}  {tid:<8} {comm_of[tid]}")

    keep = None
    if a.thread:
        keep = threads.most_common(1)[0][0] if a.thread == "busiest" else int(a.thread)
    stacks = C.Counter()
    for comm, tid, ns, frames in samples:
        if keep is not None and tid != keep:
            continue
        if a.under:
            at = next((i for i, fr in enumerate(frames) if a.under in fr), None)
            if at is None:
                continue
            frames = frames[at:]
        elif keep is None:
            frames = [f"{comm} ({tid})"] + frames
        stacks[tuple(frames)] += ns
    total = sum(stacks.values())
    if not total:
        sys.exit("no samples left after --thread and --under")
    scope = (f"thread {keep}" if keep is not None else "all threads") + (
        f", under '{a.under}'" if a.under else ""
    )
    print(f"\nscope: {scope}: {total / 1e6:.0f} ms, {100 * total / all_ns:.1f} % of the profile")
    if a.per_unit:
        print(
            f"per unit of work ({a.per_unit:.0f} units): {total / 1000 / a.per_unit:.3f} us of processor time"
        )

    own, incl = C.Counter(), C.Counter()
    for frames, ns in stacks.items():
        own[frames[-1]] += ns
        for fr in set(frames):
            incl[fr] += ns
    table("own time (the function's own instructions)", own.most_common(a.top), total, a.per_unit)
    table("time including callees", incl.most_common(a.top), total, a.per_unit)

    if a.out_dir:
        with open(os.path.join(a.out_dir, "stacks.folded"), "w") as f:
            for frames, ns in sorted(stacks.items()):
                f.write(";".join(fr.replace(";", ":") for fr in frames) + f" {ns // 1000}\n")
        flame_svg(stacks, total, os.path.join(a.out_dir, "flame.svg"), scope)
        print(
            f"\nwrote {a.out_dir}/stacks.folded (microseconds per stack) and {a.out_dir}/flame.svg"
        )


if __name__ == "__main__":
    main()
