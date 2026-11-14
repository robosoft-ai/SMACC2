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

"""Summarize a SMACC2 LTTng trace (babeltrace text on stdin or as argv[1])."""
import re, sys, statistics as st, collections as C

LINE = re.compile(
    r'^\[(\d+):(\d+):(\d+)\.(\d+)\] \(\S+\) \S+ ros2:(\w+): \{ cpu_id = (\d+) \}, \{ vtid = (\d+), procname = "([^"]*)" \}(?:, \{ (.*) \})?'
)


def parse(f):
    for l in f:
        m = LINE.match(l)
        if not m:
            continue
        h, mi, s, ns, name, cpu, tid, proc, fields = m.groups()
        t = (int(h) * 3600 + int(mi) * 60 + int(s)) * 10**9 + int(ns)
        kv = dict(re.findall(r'(\w+) = "([^"]*)"', fields or ""))
        yield t, name, int(tid), kv


def short(n):
    return re.sub(r"<.*", "", n).split("::")[-1]


def stats(us):
    us = sorted(us)
    n = len(us)
    p = lambda q: us[min(n - 1, int(q * n))]
    return f"n={n:<6d} median {st.median(us):9.1f}  p95 {p(0.95):9.1f}  p99 {p(0.99):9.1f}  max {us[-1]:10.1f}"


ev = list(parse(open(sys.argv[1]) if len(sys.argv) > 1 else sys.stdin))
if not ev:
    sys.exit("no events parsed")
t0, t1 = ev[0][0], ev[-1][0]
print(f"events {len(ev)}  span {(t1 - t0) / 1e9:.2f} s  threads {len({e[2] for e in ev})}")
print("\n== threads (events per thread, what each emits)")
per = C.defaultdict(C.Counter)
for t, n, tid, kv in ev:
    per[tid][n.replace("smacc2_", "")] += 1
for tid, c in sorted(per.items(), key=lambda x: -sum(x[1].values())):
    print(
        f"  tid {tid}: {sum(c.values())} events: "
        + ", ".join(f"{k} {v}" for k, v in c.most_common(4))
    )
spins = [t for t, n, tid, kv in ev if n == "spinOnce"]
if len(spins) > 2:
    d = [(b - a) / 1e3 for a, b in zip(spins, spins[1:])]
    print(
        f"\n== polling loop (spinOnce to spinOnce, microseconds)  rate {len(d) / ((spins[-1] - spins[0]) / 1e9):.1f} Hz"
    )
    print("  period  " + stats(d))
    late = sorted(d)[-3:]
    print(f"  three longest periods: {', '.join(f'{x / 1e3:.2f} ms' for x in late)}")
print("\n== durations, start to end on the same thread (microseconds)")
PAIRS = [
    ("state_onEntry", ("state_name",)),
    ("state_onExit", ("state_name",)),
    ("state_onRuntimeConfigure", ("state_name",)),
    ("client_behavior_on_entry", ("state_name", "client_behavior_name")),
    ("client_behavior_on_exit", ("state_name", "client_behavior_name")),
]
for base, keys in PAIRS:
    open_, dur = {}, C.defaultdict(list)
    for t, n, tid, kv in ev:
        k = (tid,) + tuple(kv.get(x) for x in keys)
        if n == f"smacc2_{base}_start":
            open_[k] = t
        elif n == f"smacc2_{base}_end" and k in open_:
            dur[" / ".join(short(kv[x]) for x in keys)].append((t - open_.pop(k)) / 1e3)
    for name, us in sorted(dur.items(), key=lambda x: -st.median(x[1]))[:8]:
        print(f"  {base:26s} {name:44s} {stats(us)}")
    if open_:
        print(
            f"  {base}: {len(open_)} start(s) with no end (still running at trace end or interrupted)"
        )
print(
    "\n== transitions: event posted -> old state's onExit start -> new state's onEntry end (microseconds)"
)
seq = [
    (t, n, kv)
    for t, n, tid, kv in ev
    if n
    in (
        "smacc2_event",
        "smacc2_state_onExit_start",
        "smacc2_state_onExit_end",
        "smacc2_state_onEntry_start",
        "smacc2_state_onEntry_end",
    )
]
react, exit_to_entry, whole, dwell, names = [], [], [], C.defaultdict(list), C.Counter()
last_event = last_exit_end = last_entry_end = None
cur = None
for t, n, kv in seq:
    if n == "smacc2_event":
        last_event = (t, short(kv.get("event_type", "")))
    elif n == "smacc2_state_onExit_start":
        if last_event:
            react.append((t - last_event[0]) / 1e3)
            names[last_event[1]] += 1
        if last_entry_end and cur:
            dwell[cur].append((t - last_entry_end) / 1e6)
    elif n == "smacc2_state_onExit_end":
        last_exit_end = t
    elif n == "smacc2_state_onEntry_start" and last_exit_end:
        exit_to_entry.append((t - last_exit_end) / 1e3)
    elif n == "smacc2_state_onEntry_end":
        if last_event and last_exit_end and last_exit_end > last_event[0]:
            whole.append((t - last_event[0]) / 1e3)
        last_entry_end = t
        cur = short(kv.get("state_name", ""))
        last_event = None
for lab, us in (
    ("event posted -> onExit start", react),
    ("onExit end -> next onEntry start", exit_to_entry),
    ("event posted -> next state entered", whole),
):
    if us:
        print(f"  {lab:36s} {stats(us)}")
if names:
    print("  triggering events: " + ", ".join(f"{k} x{v}" for k, v in names.most_common(5)))
hand, work = [], []
for i, (t, n, tid, kv) in enumerate(ev):
    if n != "smacc2_event":
        continue
    first = None
    for t2, n2, tid2, kv2 in ev[i + 1 : i + 400]:
        if n2 == "spinOnce" or t2 - t > 10**9:
            continue
        if tid2 != tid and first is None:
            first = t2
        if n2 == "smacc2_state_onEntry_end":
            if first:
                hand.append((first - t) / 1e3)
                work.append((t2 - first) / 1e3)
            break
        if n2 == "smacc2_event" and first is None:
            break
if hand:
    print("\n== events posted from another thread (microseconds)")
    print(f"  {'hand-off to the state machine thread':36s} {stats(hand)}")
    print(f"  {'transition work on that thread':36s} {stats(work)}")
if dwell:
    print("\n== time spent in each state (milliseconds, onEntry end to onExit start)")
    for s, ms in sorted(dwell.items(), key=lambda x: -sum(x[1]))[:10]:
        print(f"  {s:30s} {stats(ms)}")
open_, dur, starts = {}, C.defaultdict(list), 0
for t, n, tid, kv in ev:
    k = (tid, kv.get("updatable_element_name"))
    if n == "smacc2_state_update_start":
        open_[k] = t
        starts += 1
    elif n == "smacc2_state_update_end" and k in open_:
        dur[kv["updatable_element_name"].split("::")[-1][:44]].append((t - open_.pop(k)) / 1e3)
if starts:
    ends = sum(len(v) for v in dur.values())
    print(f"\n== update() durations (microseconds)  starts {starts}, ends {ends}")
    if not ends:
        print("  no update_end events: this build predates the update_end tracepoint fix")
    for name, us in sorted(dur.items(), key=lambda x: -st.median(x[1]))[:10]:
        print(f"  {name:44s} {stats(us)}")
