# SMACC2 CI and Testing Strategy

What each check in this repository is for, when it runs, and what it can and cannot find.
Figures are as of 2026-11-13 and should be refreshed when a check changes.

## Rules

1. **Nothing gets in the way of pushing commits.** The `jazzy` branch has no branch protection and no
   required checks, so every check is a signal and none is a lock. A check may run on every pull
   request only if it is fast, deterministic and has no known false failures.
2. **The two build workflows finish in 20 minutes or less, and build SMACC2 from scratch.** A cached
   build of our own code does not prove that the tree builds, which is the point of the workflow.
   What may be cached is set by the strategic caching policy below.
3. **Heavy checks are run on demand.** Sanitizers, clang-tidy, the clang build and CodeQL are `workflow_dispatch` only.
   They are run before a release and after any change to the core's threading, lifetime or shutdown
   code, not on every pull request.
4. **A new check starts with a baseline, not a cleanup.** It records the findings that exist and fails
   only on new ones, so turning it on never blocks work on old code.
5. **A check earns its place by calibration.** Before a tool is enforced it is run against defects that
   are already known, and it stays only if it finds a class of them that nothing else finds.

## What runs when

| Check | Where | Trigger | On every PR | Time | What it is for |
|--|--|--|--|--|--|
| pre-commit (clang-format, `cppcheck`, `ament_lint_cmake`, `ament_copyright`, file hygiene) | developer machine | every commit | local | seconds | formatting and licence headers; cppcheck at warning, style and performance severity (uninitialized members, needless copies, shadowed variables, dead conditions) |
| Format (`ci-format.yml`) | GitHub | pull request | yes | about 2 min | the same pre-commit hooks over the whole tree |
| Pull request Doc Check (`doxygen-check-build.yml`) | GitHub | pull request | yes | about 5 min | the Doxygen build |
| jazzy Binary Build (`jazzy-binary-build.yml`) | GitHub | pull request, push, nightly | yes | **median 24 to 26 min** | the tree builds from scratch against released ROS packages |
| jazzy Semi-Binary Build (`jazzy-semi-binary-build.yml`) | GitHub | pull request, push, nightly | yes | **median 23 min** | as above, with a different skip list |
| Sanitizers (`ci-sanitizers.yml`) | GitHub | manual | no | up to 60 min | the ten core tests under TSan, ASan and a plain build |
| clang-tidy (`ci-clang-tidy.yml`) | GitHub | manual | no | up to 60 min | bug-prone patterns against a checked-in baseline |
| clang build (`ci-clang-build.yml`) | GitHub, or `.github/scripts/clang_build.sh <out-dir>` on a developer machine | manual | no | 41 min on GitHub (first run, 2026-11-19, passed); 14 min on a 32-core host | every package compiles with clang 18 with no warning, and the ten core tests pass on that build; clang reports classes of mistake GCC accepts silently |
| CodeQL (`codeql.yml`) | GitHub | manual | no | | security queries |
| LTTng trace (capture steps under "LTTng tracing" below) | developer machine | on demand | no | seconds to capture and read | timing: loop rate and jitter, how long each state, behavior, update and transition takes |
| Regression suite (`smacc2_performance_tools/core_test_tools/regression.sh`) | developer machine | on demand, before a core change is committed | no | 3 min | five reference machines still go through the same state sequence, compared with `regression_expected.out` |
| Transition speed (`core_test_tools/transition_speed.sh`) | developer machine | on demand, when the transition path changes | no | 1 min per run, three runs | transitions per second with logging off, against the band in the tools' README |
| Memory tests (`core_test_tools/memory_tests/transition_soak.sh`, `object_churn_soak.sh`, `idle_load_soak.sh`, `action_goal_soak.sh`) | developer machine | on demand, before a release and when the transition path or object lifetimes change | no | 5 min each | resident memory, threads and file descriptors stay flat while the machine transitions at full speed, while every state creates and destroys behaviors, a reactor, a generator and signal connections, while one state is held with a timer, a subscriber receiving messages and updatables running, and while action goals succeed, fail and are cancelled 135 times a second; finds growth that is still reachable at exit, which LSan cannot report |
| Heap attribution (`core_test_tools/memory_tests/heap_attribution.sh <install-prefix> <test>`) | developer machine | on demand, when a memory test fails or to see what the transition path allocates | no | 1 to 2 min | a report, not a pass or fail: the call sites whose live memory grew between a short and a long run under `heaptrack`, and heap allocations per transition, message or goal by SMACC2 function; needs `heaptrack` |
| Coverage (`core_test_tools/coverage.sh`) | developer machine | on demand | no | about 12 min | which lines, functions and branches of the core the core tests and the regression suite reach |
| Simulator runs (`smacc2_performance_tools/nav2_test_tools/nav2_run.sh`, `px4_test_tools/px4_run.sh`, `moveit_test_tools/panda_run.sh`) | developer machine | on demand, after a change to a client library or to shutdown | no | 1 to 7 min each | a reference machine completes its mission against Nav2, PX4 SITL or MoveIt and shuts down cleanly; each also runs under TSan or ASan |
| ABI comparison (`.github/scripts/abi_compare.sh <old-ref> <new-ref> <work-dir>`) | developer machine | on demand, before a release | no | about 5 min | which exported functions were removed, added or changed since the last tag, which classes changed size and which virtual tables moved, for the release notes and the version number; needs `abigail-tools` |
| ROS Lint (`ci-ros-lint.yaml`) | GitHub | **disabled** | no | | `ament_copyright`, `ament_cppcheck`, `ament_lint_cmake` per package; duplicates pre-commit |
| Doxygen Deployment, bloom release | GitHub | push to `jazzy` | no | about 10 min | publishing |

**Reading a manual run.** Each manual workflow writes its result to the run's Summary page; the full
logs are in the run's artifact. Every page has the same shape: pass or fail and a few sentences, a
summary table, then the findings one by one. `.github/scripts/gate_summary.py` and
`clang_warnings.py` build the pages from the gate scripts' output.
- Sanitizers: the number of test failures and of reports of each kind the build can give (data
  races, lock-order inversions, address errors, memory leaks, undefined-behavior errors); one row per
  core test with what it measured and the state machine it ran; then the failure lines and reports.
- clang-tidy: a row per check with its count; then every diagnostic as the tool printed it, grouped
  by check, and under each group the policy for that check from `.github/clang-tidy-policy.txt`. A
  diagnostic that is not in the baseline is marked `NEW`. A check that starts to report needs a line
  in the policy file.
- clang build: a row per warning flag with its count; then every compiler warning grouped by flag and
  sorted by place, each distinct warning once with the number of times it was given. The core tests
  on the clang build get one sentence, and their failure lines when there are any; their table is
  not on this page (removed 2026-11-19: it read as a sanitizer result).

## Strategic caching policy

Caching is the exception. A package may be cached between runs only when all four hold:

1. **It is third-party.** No SMACC2 package is ever cached; every one is compiled in every run.
2. **It is pinned** in `SMACC2.jazzy.repos`, and the cache key is the commit that pin resolves to, so
   an upstream change rebuilds it.
3. **It has no binary package.** A dependency with an apt package is installed, not built and cached.
4. **The payoff is high:** it takes minutes of every run and changes rarely. A package that saves a
   minute or two does not qualify; the cache is one more thing that can go wrong.

Two packages are cached. `px4_msgs` qualifies on all four (about 10 minutes, cached since 2026-11-18
in both build workflows). Its key is the commit of `release/1.16` plus a hash of the installed
`rosidl`, `rmw` and Fast DDS package versions, so a ROS sync in the container image also rebuilds it.
It is installed to `/opt/px4_msgs` and saved right after it is built, so a later failure in the run
does not lose it. `micro_ros_agent` (2.4 minutes) is cached since 2026-11-20 by the maintainer's
decision, although it is under the payoff bar of condition 4: the two build workflows ran 19 to 22
minutes against the 20 minute limit. It is cached the same way, in `/opt/micro_ros_agent`, keyed on
the commit of the tag the repos file names (`5.0.3`, the Jazzy line; 6.1.x is Kilted, 7.0.0 Rolling)
and the same hash of ROS package versions. The build step fails when its executable is missing. The
manual workflows (clang build, clang-tidy, sanitizers) do not use the cache.

Adding a package to this list is a decision for the maintainer, recorded here with its measured time.

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
| Memory that grows but is still reachable at exit | X (the transition log gaining 1 KB per transition) | no | no | no | `core_test_tools/memory_tests/`: resident memory, threads and file descriptors sampled over minutes of transitions |
| Undefined behavior: bad cast, overflow, misaligned access | | **UBSan**, built into both sanitizer builds | `cstyleCast` as style | narrowing conversions | |
| Uninitialized member | H, I, the thirteen SA-1 pointers | seen only if the value is read on a path a test takes | **`uninitMemberVar` finds it** | **`pro-type-member-init` finds it**, in the gate since 2026-11-13 | |
| Bug-prone source pattern | unchecked optional access, branch clones, swallowed exceptions | no | partly | **`bugprone-*` finds it** | |
| Behavior regression | a state sequence that changes | no | no | no | `core_test_tools/regression.sh`, the sequence compared with the expected output |
| Performance regression on the transition path | X (the transition log growing without bound) | no | no | no | `core_test_tools/transition_speed.sh` against its band; an LTTng trace shows which step of the transition got slower |
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

**The ten core tests** (`smacc2_performance_tools/performance_tests/`) and what each provokes:

| Test | What it exercises |
|--|--|
| `sm_coretest_forced_finish_1` | 20 state cycles, then 10 runs interrupted at random moments: async behaviors forced to finish at exit |
| `sm_coretest_signal_key_1` | signal connections keyed by object across state changes |
| `sm_coretest_reactor_order_1` | the order of a state reactor's output against the event that triggered it |
| `sm_coretest_timer_handoff_1` | a timer created on the state machine thread and serviced by the executor (W) |
| `sm_coretest_post_during_exit_1` | 200 Hz event posts from the executor thread across 50 ms state exits |
| `sm_coretest_poll_shutdown_1` | SIGINT landing inside the polling loop's update window |
| `sm_coretest_transition_speed_1` | 20 s of back-to-back transitions with the loop at 2000 Hz |
| `sm_coretest_exceptions_1` | user code that throws, one site per run: a state's `onExit`, a behavior's `onExit` or `update()`, a behavior's `runtimeConfigure` or `onEntry` (posts `EvCbInitializationFailure`), and a state's `runtimeConfigure` or `onEntry` (posts `EvStInitializationFailure` to the state, then to each container, then to the state machine's hook) |
| `sm_coretest_action_client_1` | `CpActionClient` and `CbActionClientBehaviorBase` against an action server in the same process: a goal that succeeds, is rejected, is aborted, has no server, and is cancelled by a state exit while in flight, with the next state sending its own goal at once |
| `sm_coretest_features_1` | an event generator, a `CbSequence` that completes, one aborted by a failing step and two cut short by a state exit, a behavior that posts its failure event, and an updatable with an update period |

The script fails on any report with a SMACC2 frame, a crash, a hang or a broken test invariant.
Reports with no SMACC2 frame are counted as external and do not fail the run; the known one is a
string race inside Fast DDS.
That report needs a second participant in the domain, so the script stops the `ros2` daemon of its
domain at the start and the end and exits with status 2 when a node is already running there; the
scripts in `smacc2_performance_tools` do the same.

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

- The pre-commit hook calls `cppcheck` directly with `--enable=warning,style,performance,portability` and
  `--error-exitcode=1` (2026-11-18). `ament_cppcheck` cannot raise the severity, and at error severity
  the hook reported nothing and could not fail. It reads every C++ file in the repository; the
  reference library was excluded until 2026-11-19 (898 files, 5 findings, fixed).
- What it catches is a member with no initializer (`uninitMemberVar`), in every library, on every
  commit. The clang-tidy gate checks the same family only in the packages it covers and only by hand.
  Each header is read alone, in under a second for the whole tree.
- `performance` was added the same day, after its 36 lines were fixed. A constructor that stores a
  string takes it by value and moves it into the member, which cppcheck accepts and which leaves
  the signature alone; a function that only reads a string takes a const reference.
- `style` was added the same day with four of its checks suppressed: `unusedStructMember` (487
  lines, false positives because a header is read without the source file that uses the member),
  `noExplicitConstructor` (86, not enforced, see below), `variableScope` (10) and `useStlAlgorithm`
  (2), both matters of taste. The other 27 lines were fixed, among them a dead emergency stop in
  the backward local planner.
- `style` is cppcheck's broadest class. CI has cppcheck 2.13 from Ubuntu 24.04; a newer cppcheck can
  add checks, so expect new lines when the image changes.
- At `--enable=warning,style,performance,portability` the core had 33 lines on 2026-11-13: 12
  `noExplicitConstructor`, 5 `duplInheritedMember`, 4 `useStlAlgorithm`, 3 `cstyleCast`, 2
  `unknownMacro`, 3 `uninitMemberVar`, and one each of four others.
- **Decided:** no `--inconclusive`; `noExplicitConstructor` is not enforced.
- **Known false positives** in `smacc_state_info.cpp`: `selfInitialization` on
  `eventType(eventType)` and `uninitMemberVar` on `tid_`.
- **By design, not defects:** `duplInheritedMember` on `onStateOrthogonalAllocation` and
  `onComponentInitialization`. A template cannot be virtual, so the derived definitions hide the base
  ones on purpose.
- Suppressed in the hook: `duplInheritedMember` (above), and `unknownMacro` and `internalAstError`
  as `ament_cppcheck` does.
- The decision of 2026-11-13 to remove the hook was reversed on 2026-11-18: the hook stays, tuned as
  above.

## MISRA C++:2023, dynamic memory

- Checked: rules 21.6.2 to 21.6.5 (no hand-written `new`, `delete`, `malloc` or `free`; no placement
  `new`, overloaded `operator new` or direct destructor call; sized and unsized global `operator delete`
  together; no `delete` of an incomplete type).
- Not checked, by decision (2026-11-19): rule 21.6.1, no dynamic memory at all. SMACC2 allocates by
  design, as do Boost.Statechart and `rclcpp` under it.
- Two tools. The CodeQL workflow runs the queries for the four rules from GitHub's pack
  `codeql/misra-cpp-coding-standards` and lists the results on its run page; it reports and does not
  fail. The clang-tidy gate runs five checks that cover 21.6.2 and part of 21.6.4 against its baseline.
- An event is created with `smacc2::makeEvent<Ev>()` and passed to `postEvent()`, never with `new`:
  both tools miss a `new` of an event (CodeQL because events have their own `operator new`), so this
  rule is kept by review.
- Deviations from 21.6.2, each marked by a comment at its line: the `new` inside `makeEvent`
  (`smacc_types.hpp`), the `new` of the state object in `shallow_construct` (`smacc_state_base.hpp`),
  and `free` as the deleter of the string `abi::__cxa_demangle` returns (`introspection.hpp`).
- This is a check with open-source tools mapped to the rules by us. It is not a MISRA compliance claim.

## clang-tidy

- `.github/scripts/clang_tidy_gate.sh <build-base> <out-dir> [--update-baseline]` runs `bugprone-*`,
  `concurrency-*`, `clang-analyzer-*` and two single checks, `cppcoreguidelines-pro-type-member-init`
  and `cppcoreguidelines-pro-type-cstyle-cast`, over `smacc2`, `cl_ros2_timer`, `cl_lifecycle_node`,
  `cl_nav2z`, `cl_px4_mr`, `cl_moveit2z`, `cl_http`, `cl_gcalcli`, `cl_modbus_tcp_relay`,
  `cl_keyboard`, `cl_generic_sensor`, `cl_foundation_pose`, `cl_mission_tracker`, `sm_atomic`,
  `sm_atomic_lifecycle` and
  `sm_panda_cl_moveit2z_cb_inventory`, and compares the result with
  `.github/clang-tidy-baseline.txt`. A diagnostic that is not in the baseline, or more instances of a
  known one, fails the gate.
- The two single checks were added on 2026-11-13 to cover what cppcheck caught in the study:
  uninitialized members and C-style casts. Removing one initializer from `ISmaccState` fails the gate.
- Five more were added on 2026-11-19 for the dynamic memory rules of MISRA C++:2023 (see "MISRA
  C++:2023, dynamic memory" below): `cppcoreguidelines-owning-memory`, `cppcoreguidelines-no-malloc`,
  `modernize-make-unique`, `modernize-make-shared` and `misc-new-delete-overloads`.
- A diagnostic in a header is counted once, however many source files include the header, so adding
  a source file does not raise a count (changed 2026-11-15; before that `CheckType` counted 25).
- The baseline holds 98 diagnostics in 73 keys. None is from the memory checks: the 28 they gave on
  2026-11-19 were fixed by 2026-11-20. Of the 98: 26 in the core, `cl_nav2z` and the launch client,
  64 in `cl_px4_mr`, 1 in the panda machine, 5 in `cl_http` and `cl_gcalcli`, and 2 in the
  `cl_moveit2z` headers, none a defect.
  - The 5 (added 2026-11-18 with the five libraries that have source files): 4
    `easily-swappable-parameters` and one `unused-return-value` in `http_session.cpp` that is a false
    positive. `cl_modbus_tcp_relay`, `cl_keyboard`, `cl_generic_sensor`, `cl_foundation_pose` and
    `cl_mission_tracker` have none. Two
    `concurrency-mt-unsafe` on `std::localtime` in `cl_gcalcli` were fixed with `localtime_r`.
  - **A header is counted only when a source file of its own package includes it.** A header reached
    only from a state machine comes through the install base and is not matched. So every client
    library has a source file that includes all of its headers (2026-11-18): `cl_moveit2z.cpp` in
    `cl_moveit2z`, and `headers_check.cpp`, built as an object library that is not installed, in
    the header-only `cl_foundation_pose` and `cl_mission_tracker`. A new header goes into that list.
    The same file makes the package's own build compile every header: before it, the two
    header-only libraries were compiled by nothing and did not compile, and
    `cb_end_effector_rotate.hpp` had no include guard.
  - The 2 in `cl_moveit2z`: `easily-swappable-parameters` on `createGraspableBox` and `setColor`.
    When its headers were first counted (2026-11-18) the library had 29; 27 were fixed the same day,
    among them a defect: `CbCircularPivotMotion` read its initial pose after a failed transform
    lookup had left it unset. It now generates no trajectory and posts a failure.
  - **The machines are not in the gate, by decision (2026-11-19), apart from the three listed.** All 45
    were measured once: 71 diagnostics, none a defect, 29 minutes. If machines are added later, the
    core test machines in `smacc2_performance_tools/performance_tests` stay out: they do unusual
    things on purpose (`std::getenv` to choose a fault, exceptions thrown to be caught).
  - `cl_isaac_apriltag` has a `COLCON_IGNORE` file (it needs `isaac_ros_apriltag_interfaces`) and is
    compiled by nothing, here or in CI.
  - The 27: 20 `easily-swappable-parameters` (10 in `cl_nav2z` constructors, 8 generated by the lttng
    macros, 2 on the launch client), 3 member-init lines (`CheckType::transitionTypeName` is assigned
    in the constructor body; two launch-client buffers are written before they are read), the lttng
    header guard, "main may throw" (2) and one analyzer false positive in `signal_detector.cpp`.
  - The 1 in the panda machine (added 2026-11-15 with `cl_moveit2z`): "main may throw". Until
    2026-11-18 the library's one source file compiled 3 of its 22 headers, and the machine was added
    to compile the rest, which did not work for the gate: see the note on headers above.
  - The 64 in `cl_px4_mr` (added 2026-11-15): 30 `easily-swappable-parameters` on setters and
    constructors that take x, y or latitude, longitude; 23 narrowing conversions and 7 widening casts
    on loop counters in the pattern generators; 2 integer divisions that are the intended leg number
    of a square spiral; 2 tf2 quaternions assigned before they are read.
- **Policy:** the rest of `cppcoreguidelines-*` is not in the gate and is not swept. It is a style
  guide, most of its lines on the core are public member variables, and silencing them would mean
  changing public signatures.
- **Measured and left out for now:** `cppcoreguidelines-init-variables` (34 findings, 33 in `cl_nav2z`,
  local variables declared without a value) and `cppcoreguidelines-explicit-virtual-functions` (107,
  missing `override`, 94 in `cl_nav2z`). Each would need a sweep of `cl_nav2z` first.

## The clang build

- `.github/scripts/clang_build.sh <out-dir>`, run from the workspace root with only
  `/opt/ros/jazzy/setup.bash` sourced, builds every SMACC2 package with clang 18 into its own build
  and install bases under the out dir, so the GCC build is not touched. Those bases are deleted first:
  a file that is not recompiled gives no warning. `smacc2_msgs` and `px4_msgs`
  are built first with the default compiler, because their code is generated.
- It prints every warning the compiler gave, grouped by warning flag, and writes the same list to `warnings.txt`
  (`.github/scripts/clang_warnings.py`). A warning in a header is given once for every source file
  that includes the header; the list shows it once, with the count, under its path in the repository.
- The standard is zero: the script exits 1 on a build error or on any warning. GCC is not held to
  this; the two compilers warn about different things.
- The tree is at zero since 2026-11-18. The last two warnings came from code that is not ours and are
  silenced where they arise, for clang only, with `#pragma clang diagnostic`: the `std::auto_ptr`
  inside Boost.Statechart, around the two Boost includes in `common.hpp` and
  `smacc_fifo_scheduler.hpp`, and the empty `spinOnce` tracepoint in `trace_provider.hpp`. No
  package's CMake file turns a warning off, so both warnings still fire anywhere else.
- The workflow then runs the ten core tests on the clang build
  (`sanitizer_core_tests.sh plain <out-dir>/install`).
- What clang has found here that GCC accepts silently: `demangleSymbol(const std::string &)` calling
  itself forever (a defect, fixed), 639 type announcements with `class` for a type defined with
  `struct`, `typeid` applied to a dereferenced smart pointer, counters and private fields nothing read.

## LTTng tracing

The core defines 14 tracepoints under the `ros2` provider (`smacc2/include/smacc2/smacc_tracing/`):
the polling loop iteration (`spinOnce`), event posting, and start and end pairs for state runtime
configure, entry, exit and update, and for client behavior entry and exit. They are compiled in when
`liblttng-ust-dev` is found.

**Studied 2026-11-13: the traces can be captured and read as text, with no graphical viewer.** On
that evidence the tracepoints are worth keeping.

`smacc2_performance_tools/tracing_tools/smacc2_trace.sh <out-dir> <seconds> <node-binary> [args]`
captures a trace and prints the timing summary in one step. By hand, with `lttng-tools` and
`babeltrace` installed; no root and no tracing group are needed:

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

1. Bring the two build workflows under 20 minutes. `px4_msgs` is cached since 2026-11-18; measure a
   miss and a hit on GitHub.
2. Remove the disabled ROS Lint workflow.
3. Done 2026-11-19: the work of an asynchronous behavior has its own tracepoints,
   `smacc2_client_behavior_async_on_entry_*` and `_async_on_exit_*`, emitted on the behavior's thread.
4. Add a `perf` flame-graph recipe. Coverage, the memory soak and the ABI comparison are in place:
   `core_test_tools/coverage.sh`, `core_test_tools/memory_tests/`, `.github/scripts/abi_compare.sh`.
