# SMACC2 CI and Testing Strategy

What each check in this repository is for, when it runs, and what it can and cannot find.
Figures are as of 2026-11-13 and should be refreshed when a check changes.

## Rules

1. **Nothing gets in the way of pushing commits.** The `jazzy` branch has no branch protection and no
   required checks, so every check is a signal and none is a lock. A check may run on every pull
   request only if it is fast, deterministic and has no known false failures.
2. **The two build workflows finish in 20 minutes or less, without caching.** A cached build does not
   prove that the tree builds from scratch, which is the point of the workflow.
3. **Heavy checks are run on demand.** Sanitizers, clang-tidy and CodeQL are `workflow_dispatch` only.
   They are run before a release and after any change to the core's threading, lifetime or shutdown
   code, not on every pull request.
4. **A new check starts with a baseline, not a cleanup.** It records the findings that exist and fails
   only on new ones, so turning it on never blocks work on old code.
5. **A check earns its place by calibration.** Before a tool is enforced it is run against defects that
   are already known, and it stays only if it finds a class of them that nothing else finds.

## What runs when

| Check | Where | Trigger | On every PR | Time | What it is for |
|--|--|--|--|--|--|
| pre-commit (clang-format, `ament_cppcheck`, `ament_lint_cmake`, `ament_copyright`, file hygiene) | developer machine | every commit | local | seconds | formatting and licence headers; cppcheck at error severity only |
| Format (`ci-format.yml`) | GitHub | pull request | yes | about 2 min | the same pre-commit hooks over the whole tree |
| Pull request Doc Check (`doxygen-check-build.yml`) | GitHub | pull request | yes | about 5 min | the Doxygen build |
| jazzy Binary Build (`jazzy-binary-build.yml`) | GitHub | pull request, push, nightly | yes | **median 24 to 26 min** | the tree builds from scratch against released ROS packages |
| jazzy Semi-Binary Build (`jazzy-semi-binary-build.yml`) | GitHub | pull request, push, nightly | yes | **median 23 min** | as above, with a different skip list |
| Sanitizers (`ci-sanitizers.yml`) | GitHub | manual | no | up to 60 min | the seven core tests under TSan, ASan and a plain build |
| clang-tidy (`ci-clang-tidy.yml`) | GitHub | manual | no | up to 60 min | bug-prone patterns against a checked-in baseline |
| CodeQL (`codeql.yml`) | GitHub | manual | no | | security queries |
| LTTng trace (capture steps under "LTTng tracing" below) | developer machine | on demand | no | seconds to capture and read | timing: loop rate and jitter, how long each state, behavior, update and transition takes |
| ROS Lint (`ci-ros-lint.yaml`) | GitHub | **disabled** | no | | `ament_copyright`, `ament_cppcheck`, `ament_lint_cmake` per package; duplicates pre-commit |
| Doxygen Deployment, bloom release | GitHub | push to `jazzy` | no | about 10 min | publishing |

**The build workflows are over budget today.** Of the last 14 pull-request runs, 11 took longer than
20 minutes on each workflow (range 17 to 31 min). In a 17.4 min run, 2.8 min was dependency setup and
14.5 min was the build of 55 packages. The longest packages were `px4_msgs` (10.2 min, built from
source through `SMACC2.jazzy.repos`), `cl_nav2z` (7.6 min), `cl_px4_mr` (2.9 min), `micro_ros_agent`
(2.4 min) and `smacc2` (2.3 min). The two workflows import the same `.repos` file and differ only in
their `--packages-skip` list.

## Which tool finds which kind of defect

This table comes from the 2026-11 core defect study, where each tool was run against the tree as it
was before the fixes and scored on the defects already known.

| Defect class | Example from the study | Sanitizers | cppcheck | clang-tidy | Other |
|--|--|--|--|--|--|
| Lifetime: use after free, dangling pointer or reference | A (a `c_str()` of a temporary) | **ASan finds it** | missed | missed, even with the templates instantiated | reading the code |
| Concurrency: data race | the `isShutdownRequested_` race, posting an event during state exit | **TSan finds it** | missed | missed | core tests that provoke the race |
| Concurrency: shutdown ordering, hang, deadlock | F-6, the timer hand-off race (W), the async-thread locking rule | TSan for lock order; the hang itself shows as a timeout | missed | `concurrency-mt-unsafe` found one (`exit()` in a signal handler) | interrupt sweeps; **W shows only in a plain build** |
| Leak at exit | M | **LSan finds it**, on a clean exit only | missed | `owning-memory`, not gated | |
| Undefined behavior: bad cast, overflow, misaligned access | | **UBSan**, built into both sanitizer builds | `cstyleCast` as style | narrowing conversions | |
| Uninitialized member | H, I, the thirteen SA-1 pointers | seen only if the value is read on a path a test takes | **`uninitMemberVar` finds it** | **`pro-type-member-init` finds it**, in the gate since 2026-11-13 | |
| Bug-prone source pattern | unchecked optional access, branch clones, swallowed exceptions | no | partly | **`bugprone-*` finds it** | |
| Behavior regression | a state sequence that changes | no | no | no | regression suite, sequence diff against the last capture |
| Performance regression on the transition path | X (the transition log growing without bound) | no | no | no | transition-speed test against its band; an LTTng trace shows which step of the transition got slower |
| Timing: a slow state or behavior, loop jitter, late reaction to an event | a 90 to 220 us wait before a timer event reaches the state machine thread (the kernel waking the thread, not a defect) | no | no | no | **LTTng trace**: nanosecond timestamps on every entry, exit, update and event |

How to read it: the sanitizers are the only tools that find the lifetime and concurrency defects, and
those were most of the serious defects in the study. cppcheck found the uninitialized-member family,
a null dereference and a dead error branch; the first of these is now also covered by the
clang-tidy gate. clang-tidy's gated groups catch source patterns that tend to become defects. No static tool on this host saw defect A. LTTng finds no defects by itself; it is the only
tool here that says how long things take and on which thread.

## The sanitizers

Three builds, each run through `.github/scripts/sanitizer_core_tests.sh <tsan|asan|plain>`.

| Build | Flags | Finds |
|--|--|--|
| TSan + UBSan | `-fsanitize=thread,undefined -fno-omit-frame-pointer` | data races, lock-order inversions, undefined behavior |
| ASan + LSan + UBSan | `-fsanitize=address,undefined -fsanitize-recover=address -fno-omit-frame-pointer`, `detect_leaks=1` | use after free, buffer overflows, leaks at exit, undefined behavior |
| Plain | none | hangs and crashes that the sanitizer allocators hide (W needs glibc's immediate reuse of freed memory) |

**Why they are trusted.** A sanitizer report is an observation of the running program: this thread
wrote this address while that thread read it, with both stacks. It is not a guess about what the
code might do. The same is true in reverse: when a change is rebuilt and rerun and the report is
gone, the defect is gone on that path. That is the evidence a fix is a good one.

**What they cannot see.** Only code that runs is checked, so coverage is the tests' coverage. TSan
misses a race when an unrelated mutex happens to order the two accesses (the logging mutex hid the
`isShutdownRequested_` race for 18 cycles; it showed in a 95-cycle run). LSan reports only when the
process exits cleanly.

**The seven core tests** (`smacc2_performance_tools/performance_tests/`) and what each provokes:

| Test | What it exercises |
|--|--|
| `sm_coretest_forced_finish_1` | 20 state cycles, then 10 runs interrupted at random moments: async behaviors forced to finish at exit |
| `sm_coretest_signal_key_1` | signal connections keyed by object across state changes |
| `sm_coretest_reactor_order_1` | the order of a state reactor's output against the event that triggered it |
| `sm_coretest_timer_handoff_1` | a timer created on the state machine thread and serviced by the executor (W) |
| `sm_coretest_post_during_exit_1` | 200 Hz event posts from the executor thread across 50 ms state exits |
| `sm_coretest_poll_shutdown_1` | SIGINT landing inside the polling loop's update window |
| `sm_coretest_transition_speed_1` | 20 s of back-to-back transitions with the loop at 2000 Hz |

The script fails on any report with a SMACC2 frame, a crash, a hang or a broken test invariant.
Reports with no SMACC2 frame are counted as external and do not fail the run; the known one is a
string race inside Fast DDS.

**Rules for running a sanitized node by hand:**
- Run the node binary directly. `ros2 run` or a launch `stdbuf` prefix loads a library ahead of the
  ASan runtime and the process refuses to start.
- TSan needs `setarch x86_64 -R <binary>`, or it exits 66 on the memory layout.
- Each `called_from_lib:` line in `tsan.supp` must match exactly one library, with its `.so` suffix.
- For a leak report, send SIGINT while no async behavior thread is live.
- Suppressions live in `.github/scripts/tsan.supp` and `lsan.supp` and cover only the DDS and rclcpp
  layers. SMACC2 itself has no suppression.

**Not in CI.** The nav2, MoveIt and PX4 machines are run under the sanitizers on a developer machine
with their simulators; only the state machine node is instrumented.

## cppcheck

- The pre-commit hook runs bare `ament_cppcheck`, so only error-severity messages fail a commit, and
  the core reports none. The hook was enabled in June 2026 (#744). It sets
  `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1` because `ament_cppcheck` otherwise skips the cppcheck 2.x
  that Ubuntu 24.04 ships. The reference library is excluded.
- At `--enable=warning,style,performance,portability` the core has 33 lines (2026-11-13): 12
  `noExplicitConstructor`, 5 `duplInheritedMember`, 4 `useStlAlgorithm`, 3 `cstyleCast`, 2
  `unknownMacro`, 3 `uninitMemberVar`, and one each of four others.
- **Decided:** no `--inconclusive`; `noExplicitConstructor` is not enforced.
- **Known false positives** in `smacc_state_info.cpp`: `selfInitialization` on
  `eventType(eventType)` and `uninitMemberVar` on `tid_`.
- **By design, not defects:** `duplInheritedMember` on `onStateOrthogonalAllocation` and
  `onComponentInitialization`. A template cannot be virtual, so the derived definitions hide the base
  ones on purpose.
- **Decided 2026-11-13: cppcheck is to be removed from the commit path.** As configured it cannot fail on
  anything the study found, and a check that always passes reads as assurance it does not give. The
  hook and the disabled ROS Lint workflow go; cppcheck stays available as an on-demand tool for the
  one family it is good at (uninitialized members), run by hand before a release.

## clang-tidy

- `.github/scripts/clang_tidy_gate.sh <build-base> <out-dir> [--update-baseline]` runs `bugprone-*`,
  `concurrency-*`, `clang-analyzer-*` and two single checks, `cppcoreguidelines-pro-type-member-init`
  and `cppcoreguidelines-pro-type-cstyle-cast`, over `smacc2`, `cl_ros2_timer`, `cl_lifecycle_node`,
  `cl_nav2z`, `sm_atomic` and `sm_atomic_lifecycle`, and compares the result with
  `.github/clang-tidy-baseline.txt`. A diagnostic that is not in the baseline, or more instances of a
  known one, fails the gate.
- The two single checks were added on 2026-11-13 to cover what cppcheck caught in the study:
  uninitialized members and C-style casts. Removing one initializer from `ISmaccState` fails the gate.
- The baseline holds 60 diagnostics in 23 keys. 28 are `easily-swappable-parameters` (18 in `cl_nav2z`
  constructors, 8 generated by the lttng macros, 2 on the launch client). 27 are member-init lines in
  3 keys, none a defect: `CheckType::transitionTypeName` is assigned in the constructor body and is
  counted once per instantiation (25), and two launch-client buffers are written before they are read.
  The rest are the lttng header guard (2), "main may throw" (2) and one analyzer false positive in
  `signal_detector.cpp`.
- **Policy:** the rest of `cppcoreguidelines-*` is not in the gate and is not swept. It is a style
  guide, most of its lines on the core are public member variables, and silencing them would mean
  changing public signatures.
- **Measured and left out for now:** `cppcoreguidelines-init-variables` (34 findings, 33 in `cl_nav2z`,
  local variables declared without a value) and `cppcoreguidelines-explicit-virtual-functions` (107,
  missing `override`, 94 in `cl_nav2z`). Each would need a sweep of `cl_nav2z` first.

## LTTng tracing

The core defines 14 tracepoints under the `ros2` provider (`smacc2/include/smacc2/smacc_tracing/`):
the polling loop iteration (`spinOnce`), event posting, and start and end pairs for state runtime
configure, entry, exit and update, and for client behavior entry and exit. They are compiled in when
`liblttng-ust-dev` is found.

**Studied 2026-11-13: the traces can be captured and read as text, with no graphical viewer.** On
that evidence the tracepoints are worth keeping.

Capture, with `lttng-tools` and `babeltrace` installed; no root and no tracing group are needed:

```bash
lttng create smacc2 --output=/path/to/trace
lttng enable-event -u 'ros2:smacc2*'
lttng enable-event -u ros2:spinOnce
lttng add-context -u -t vtid -t procname
lttng start
<run the state machine node binary>
lttng stop && lttng destroy smacc2
babeltrace /path/to/trace > trace.txt
```

Each line of `trace.txt` is one event with a nanosecond timestamp, the thread id and the state,
behavior or event name. For a machine that transitions at full speed, enlarge the buffers with
`lttng enable-channel -u --subbuf-size 8M --num-subbuf 8 <name>` and pass `-c <name>` to the
`enable-event` and `add-context` lines; 2.5 million events in 4 seconds were captured with none lost.

**Cost.** With every tracepoint recording, the transition-speed test ran at 47,830 Hz against about
52,200 untraced, roughly 8 %. With no session running the tracepoints are part of the normal figures.

**One transition, step by step.** `sm_coretest_transition_speed_1` with the loop at 2000 Hz, medians
over 100,000 transitions on the state machine thread. The test's own "Hz" counts round trips of two
transitions, so 47,830 Hz is about 94,500 state entries per second.

| Step | Median |
|--|--|
| entry start to runtime configure start | 1.58 us |
| runtime configure | 0.20 us |
| runtime configure end to the event being posted | 1.52 us |
| event posted to entry end | 2.31 us |
| entry end to exit start (event dispatch and the transition decision) | 3.22 us |
| exit | 0.20 us |
| exit end to the next entry start (the old state destroyed, the new one constructed) | 1.37 us |
| **One transition** | **10.4 us** |

**An event from another thread.** In `sm_atomic` a timer fires on the signal detector thread and the
transition runs on the state machine thread. From the event being posted to the first activity on
the state machine thread took 90 to 220 us in five transitions, and the transition itself then took
60 to 80 us with logging off and 120 to 140 us with INFO logging on. The first figure is the time
Linux takes to wake a sleeping thread on a non-realtime kernel; it is not time spent in SMACC2. It
is the floor on reaction time for any event that originates outside the state machine thread.

**The polling loop.** `sm_atomic` configured for 200 Hz: measured 200.0 Hz, period median 4999.9 us,
99th percentile 5130 us, longest 5598 us over 12 s.

## Open items

1. Bring the two build workflows under 20 minutes without caching.
2. Remove the `ament_cppcheck` hook and the ROS Lint workflow.
3. Package the LTTng capture and summary so a trace can be asked for and presented in one step.
4. Give the manual workflows a result summary on the run page.
5. Add an on-demand clang build, never on a pull request, once its warning backlog is cleared. A trial
   build with clang 18 found a real defect GCC accepts silently: `demangleSymbol(const std::string &)`
   calls itself forever.
6. Add on-demand code coverage, a memory-growth soak, an ABI comparison before releases and a `perf`
   flame-graph recipe.
