# Memory tests

Each test runs one core test machine for some minutes with logging off and fails if the process
keeps growing. They find memory that is still reachable at exit, which the leak sanitizer cannot
report: a container that gains an entry per transition, a thread or a socket that is never released.
None of them runs on a pull request.

| Script | What it stresses | Machine | Time |
|--|--|--|--|
| `transition_soak.sh` | a bare transition, about 100,000 a second | `sm_coretest_transition_speed_1` | 5 min |
| `object_churn_soak.sh` | every state-scoped object: a synchronous updatable behavior, an asynchronous behavior and its thread, an event generator, a state reactor and two signal connections, created and destroyed about 2000 times a second | `sm_coretest_object_churn_1` | 5 min |
| `idle_load_soak.sh` | the polling loop of a machine that stays in one state: a 500 Hz timer, a 1 kB message published and received on each tick, an event per message that no state consumes, and two updatables called 2000 times a second | `sm_coretest_idle_load_1` | 5 min |
| `action_goal_soak.sh` | action goals through `CpActionClient` against a server in the same process: succeeded, rejected, aborted, no server, and cancelled by a state exit, about 135 goals a second | `sm_coretest_action_client_1`, run fast | 5 min |
| `heap_attribution.sh` | not a pass or fail: runs one of the four machines under `heaptrack` and names the call sites whose memory grew, with the heap allocations per unit of work | any of the four | 1 to 2 min |
| `memory_sampler.sh` | the shared sampler the soaks call; runs any command | | |
| `test_commands.sh` | sourced by the others: each machine's command line, environment and work counter, in one place | | |

```bash
cd <workspace>
bash src/SMACC2/smacc2_performance_tools/core_test_tools/memory_tests/transition_soak.sh install 300 /tmp/soak_t
bash src/SMACC2/smacc2_performance_tools/core_test_tools/memory_tests/object_churn_soak.sh install 300 /tmp/soak_c
bash src/SMACC2/smacc2_performance_tools/core_test_tools/memory_tests/idle_load_soak.sh install 300 /tmp/soak_i
bash src/SMACC2/smacc2_performance_tools/core_test_tools/memory_tests/action_goal_soak.sh install 300 /tmp/soak_a
```

Run one at a time, with no other ROS node in the same domain.

## What the sampler measures

`memory_sampler.sh <out-dir> <seconds> <command> [args...]` starts the command, and every 2 seconds
reads its resident memory and thread count from `/proc/<pid>/status` and counts its open file
descriptors. After the run it compares the median of the first tenth of the samples taken after a
60 second warm-up with the median of the last tenth, for each of the three figures.

| Figure | Allowed growth | Override |
|--|--|--|
| resident memory | 4096 kB | `MAX_GROWTH_KB` |
| threads | 2 | `MAX_THREAD_GROWTH` |
| file descriptors | 2 | `MAX_FD_GROWTH` |

`SAMPLE_S` and `WARMUP_S` set the sample interval and the warm-up. The exit status is 1 when a figure
grew past its limit, when the command died, or when it did not exit within 20 seconds of SIGINT. The
samples are in `samples.csv` and the command's output in `node.log`, both in the output directory.

## Reference results

Measured 2026-11-14, RelWithDebInfo, 300 second runs.

| Test | Round trips | Resident memory | Threads | File descriptors |
|--|--|--|--|--|
| `transition_soak.sh` | about 15.7 million | 31,928 kB, growth 0 kB | 16, growth 0 | 60, growth 0 |
| `object_churn_soak.sh` | 301,000 | 32,432 kB, growth 4 kB | 16, growth 0 | 60, growth 0 |
| `idle_load_soak.sh` | none; 149,000 messages, 596,000 loop iterations | 32,020 kB, growth 0 kB | 16, growth 0 | 60, growth 0 |
| `action_goal_soak.sh` | 8100 cycles, 40,500 goals | 36,676 kB, growth 88 kB, none in the last 90 s | 17, growth 0 | 60, growth 0 |

The first two fail on a build of the tree as it was before the transition log was bounded: the
transition soak grew by 2.3 GB in 48 seconds and the object churn soak by 50 MB in 48 seconds. The
idle load soak makes no transitions, so its check is a switch in the machine: with
`SM_CORETEST_IDLE_LEAK=1` it keeps every message, and the soak fails with 23 MB of growth in 48
seconds. The action goal soak found its own control: the test's action server kept one finished,
unjoined thread per goal, and the soak failed on it with 9 MB of growth in 129 seconds at a
thirtieth of today's goal rate. The server now joins its finished threads.

A machine with a bounded history takes a while to settle: the transition log history keeps the last
1000 transitions, so memory rises by about half a megabyte until it is full. The 60 second warm-up
covers that for these machines at these rates.

## What they cannot see

Growth in code these four machines do not run (the client libraries, service clients), and growth
slower than about 1 MB per minute. A failing run says that memory grew, not
where; `heap_attribution.sh` is the next step.

## heap_attribution.sh

```bash
bash src/SMACC2/smacc2_performance_tools/core_test_tools/memory_tests/heap_attribution.sh install idle_load 20 /tmp/heap
```

The second argument is `transition`, `object_churn`, `idle_load` or `action_goal`. It runs that
machine under `heaptrack` for the given seconds and again for twice as long, ending each run with
SIGKILL so that everything still allocated counts as live, and compares the two. Needs the
`heaptrack` package. It prints three things:

- the live heap at the end of each run, the difference, and the difference per unit of work;
- the call sites whose live memory differs most between the runs, each named by the innermost
  frame in SMACC2 or machine code. On a healthy build these are a few kB either way: whatever was
  allocated at the instant each run was killed;
- where the allocations come from: allocations per unit of work, charged to the innermost SMACC2
  function on the stack.

The full backtraces are in `diff_leaks.txt` in the output directory, and `long_stacks.txt` there is
input for `flamegraph.pl`. `heaptrack` slows the machines by a quarter to a half, and the transition
machine writes about 200 kB of data a second, so 5 seconds is enough for it.

Checked 2026-11-14 with the idle load machine's deliberate leak (`SM_CORETEST_IDLE_LEAK=1`): the
live heap differed by 8141 kB over 7000 more messages, 1191 bytes a message, and the first two
call sites listed were the copy of the message and `CbMessageCounter::onMessage` at
`cb_message_counter.hpp:44`. Without the leak the difference was 0 kB.

Allocations measured the same day, RelWithDebInfo, logging off:

| Test | Heap allocations | Largest sources |
|--|--|--|
| `transition` | 252 per round trip, 126 per transition | 63 % in building type names: `demangleSymbol`, `cleanShortTypeName`, `TypeInfo::getFromStdTypeInfo` |
| `object_churn` | 1154 per round trip | `demangleSymbol` 22 %, the executor inside `pollingLoop` 14 %, `ISmaccClientBehavior::getName` 9 % |
| `idle_load` | 464 per message, 116 per polling loop iteration | the executor inside `pollingLoop` 66 %, `demangleSymbol` 13 % |
| `action_goal` | 1725 per goal | the executor inside `pollingLoop` 64 % |

None of this memory is kept: every one of these allocations is freed again. They are a cost in
time, not a leak.

## Adding a test

Add the machine to `test_commands.sh`, which gives it to both a soak and `heap_attribution.sh`.

Write a machine that loops without pause and logs a count at FATAL, then a script like
`object_churn_soak.sh`.
