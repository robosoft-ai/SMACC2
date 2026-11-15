# Core test tools

Scripts that check the SMACC2 core (the `smacc2` package) on a developer machine. None of them runs
on a pull request. They take the colcon install prefix and an output directory as arguments, so they
work from any workspace. The sanitizer gate, which runs the ten `sm_coretest_*` machines, is
`.github/scripts/sanitizer_core_tests.sh`; the overall strategy is in `.github/CLAUDE.md`.

| Script | Answers | Time |
|--|--|--|
| `regression.sh` | do five reference machines still go through the same states? | 3 min |
| `transition_speed.sh` | did the transition path get slower? | 1 min per run |
| `memory_tests/` | do memory, threads or file descriptors keep growing as the machine runs? | 5 min per test |
| `coverage.sh` | which lines of the core do the tests reach? | about 12 min |
| `send_keys.py` | helper: scripted keypresses for a machine that uses `cl_keyboard` | |

Run one at a time, with no other ROS node in the same domain. Each script sets its own
`ROS_DOMAIN_ID` unless one is already exported.

## regression.sh

```bash
cd <workspace>
bash src/SMACC2/smacc2_performance_tools/core_test_tools/regression.sh install /tmp/reg \
  src/SMACC2/smacc2_performance_tools/core_test_tools/regression_expected.out
```

Runs `sm_atomic`, `sm_atomic_lifecycle`, `sm_simple_action_client`, `sm_retry_logic_1` (once to
exhaustion, once to success) and `sm_mode_state_behavior_1`, driving them with a lifecycle node, the
Fibonacci action server from `action_tutorials_cpp` and scripted keypresses. For each it prints the
states entered and a few counters. A healthy build prints the same text every time, so the last line
says whether the output is identical to `regression_expected.out`, and the exit status is 1 when it
is not. The per-machine logs are in the output directory as `reg_<machine>.log`.

When a change alters a state sequence on purpose, read the difference, then replace
`regression_expected.out` with the new `regression.out` in the same commit.

## transition_speed.sh

```bash
bash src/SMACC2/smacc2_performance_tools/core_test_tools/transition_speed.sh install 60 /tmp/speed.log
```

Runs `sm_coretest_transition_speed_1` for the given seconds with logging off and prints the rate of
its 100-iteration batches. Compare the per-batch median between two builds. One iteration is a round
trip of two transitions, so the time for one transition is 1 / (2 x median).

Reference: RelWithDebInfo, the machine these were measured on, November 2026: medians of 52,036 to
52,602 Hz. Take the figure on your own machine before a change and again after it; runs on one
machine and one build differ by up to about 4 %, and now and then by 10 % for minutes at a time, so
when a result is low, run the old and the new build alternately before concluding anything.

## memory_tests/

```bash
bash src/SMACC2/smacc2_performance_tools/core_test_tools/memory_tests/transition_soak.sh install 300 /tmp/soak_t
bash src/SMACC2/smacc2_performance_tools/core_test_tools/memory_tests/object_churn_soak.sh install 300 /tmp/soak_c
bash src/SMACC2/smacc2_performance_tools/core_test_tools/memory_tests/idle_load_soak.sh install 300 /tmp/soak_i
bash src/SMACC2/smacc2_performance_tools/core_test_tools/memory_tests/action_goal_soak.sh install 300 /tmp/soak_a
```

Each runs one core test machine with logging off, samples the process's resident memory, threads and
file descriptors, and fails if any of them keeps growing after a warm-up. They find growth that is
still reachable at exit, which the leak sanitizer cannot report. The tests, the limits and the
reference results are in `memory_tests/README.md`. When one fails, `memory_tests/heap_attribution.sh`
runs the same machine under `heaptrack` and names the call sites that grew.

## coverage.sh

```bash
bash src/SMACC2/smacc2_performance_tools/core_test_tools/coverage.sh <workspace> /tmp/cov
```

Builds `smacc2`, the core tests and the regression machines with `--coverage -O0` into
`/tmp/cov/build` and `/tmp/cov/install`, runs the sanitizer gate script in plain mode and the
regression suite against that install, and prints the line, function and branch totals for the
`smacc2` package. The data is `/tmp/cov/core.info`; `genhtml` turns it into a browsable report.
Needs `lcov` 2.0 or later. Measured 2026-11-14: 81.4 % of lines.

## send_keys.py

```bash
python3 send_keys.py n*5 f pause:1 n*5      # five n, one f, a one-second wait, five n
python3 send_keys.py period:0.01 n*20       # twenty n at 100 Hz
```

Publishes `std_msgs/UInt16` key codes on `/keyboard_unicode` from one publisher, after waiting for a
subscriber. Needs a sourced ROS environment.
