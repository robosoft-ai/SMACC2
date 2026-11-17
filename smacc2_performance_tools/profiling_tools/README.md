# Profiling tools

`perf_flame.sh` records one SMACC2 node with `perf` and shows where its processor time goes: tables
in the terminal, folded stacks and a flame graph. The LTTng trace (`../tracing_tools/`) says how long
each step of a transition takes; this says what the time inside a step is spent on. It is run by
hand, never in CI.

## Once per boot

Ubuntu forbids profiling to ordinary users by default. The script stops with this command if needed:

```bash
sudo sysctl kernel.perf_event_paranoid=1
```

It also needs `perf` (`linux-tools-$(uname -r)`), and the node built with debug information
(`RelWithDebInfo`). Frame pointers are not needed.

## Record

```
perf_flame.sh <out-dir> <seconds> <node-binary> [node args...]
```

For example the transition-speed test, from a sourced workspace, looking at its busiest thread:

```bash
P=install/sm_coretest_transition_speed_1
SUMMARY_ARGS="--thread busiest" smacc2_performance_tools/profiling_tools/perf_flame.sh /tmp/perf_ts 20 \
  $P/lib/sm_coretest_transition_speed_1/sm_coretest_transition_speed_1_node --ros-args --log-level FATAL \
  --params-file $P/share/sm_coretest_transition_speed_1/config/sm_coretest_transition_speed_1_config.yaml \
  -r __node:=sm_coretest_transition_speed_1
```

Run the node binary directly and give it the node name its launch file uses, as for the trace tool.
The node is stopped with SIGINT after `<seconds>`; "node exit 124" is the normal result.

| File in the output directory | Content |
|--|--|
| `summary.txt` | the tables printed to the terminal |
| `flame.svg` | the flame graph; open it in a browser and hover a box for its full name and share |
| `stacks.folded` | one line per distinct call stack with its microseconds; speedscope and `flamegraph.pl` read it |
| `script.txt` | every sample with its stack, the input of `perf_summary.py` |
| `perf.data`, `node.log`, `perf.err` | the recording, the node's output, `perf`'s messages |

Environment: `FREQ` samples per second (default 999), `STACK` bytes of stack copied per sample
(default 32768), `SUMMARY_ARGS` options for `perf_summary.py`.

## Look closer without recording again

```
perf_summary.py <out-dir>/script.txt [--thread busiest|TID] [--under TEXT] [--per-unit N] [--top N] [--out-dir DIR]
```

- `--thread` keeps one thread. The state machine thread and the signal detector thread carry the same
  name; the first table lists them with their share.
- `--under TEXT` keeps only the stacks that pass through a function whose name contains `TEXT`, and
  starts them there: `--under "notifyTransition<"` shows what one notification costs.
- `--per-unit N` adds a column of microseconds per unit of work, `N` being the number of units in the
  run (for the transition-speed test, its `Longtime` lines times 200: a line is printed every 100
  round trips of two transitions). These figures can be set beside the LTTng durations.
- `--out-dir` writes `stacks.folded` and `flame.svg` for that selection.

## Reading it

- "Own time" is time in a function's own instructions; "including callees" adds everything it calls.
  A function high in the second table and absent from the first is expensive through what it calls.
- Short names such as `react` or `operator()` are functions the compiler inlined; their caller is the
  box below them in the flame graph.
- `[libstdc++.so.6.0.33]`, `[librclcpp.so]` and the like are functions with no exported name. Their
  callers are named, which usually says what they are; installing the library's debug symbols names
  them.
- `[kernel]` is time inside system calls, as one box.

## Limits

- It is a sample, 999 times a second per busy thread. A 20 s run is good for anything above about
  1 % of a thread; it cannot time a single call.
- It shows processor time only. Waiting (a sleeping loop, the hand-off between threads) does not
  appear; the LTTng trace measures that.
- The recording is large, about 32 MB per second of busy thread at the defaults.
- A stack deeper than `STACK` bytes is cut short and shows as a tower that does not reach the
  thread's base. At 16384, 19 % of the transition test's stacks were cut; at 32768, 2 %.
- Recording slowed the transition-speed test by about 1 %.
